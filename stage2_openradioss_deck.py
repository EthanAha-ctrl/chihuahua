#!/usr/bin/env python3
"""Generate an OpenRadioss whole-body beam deck from the Stage 2 rod graph.

This is the first OpenRadioss deck export for the whole robot. It intentionally
stays topology-only: no gravity, no support reactions, no fixed feet, and no
load cases are applied here.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml
from PIL import Image

from dog_description import DEFAULT_DESCRIPTION_PATH, load_dog_description
import mass_model as stage1
import stage2_rod_model as rods


RUN_NAME = "stage2_whole_body_beam"
RADIOSS_LENGTH_SCALE = 1000.0
RADIOSS_DENSITY_SCALE = 1.0e-9
RADIOSS_MODULUS_SCALE = 1.0e-9
MIN_CONNECTOR_RADIUS_MM = 0.5
DEFAULT_UNIFORM_RADIUS_MM = 8.0


@dataclass(frozen=True)
class BeamDeckMember:
    member: rods.RodMember
    beam_id: int
    part_id: int
    prop_id: int
    node_a_id: int
    node_b_id: int
    length_mm: float
    area_mm2: float
    iyy_mm4: float
    izz_mm4: float
    ixx_mm4: float
    structural_mass_kg: float
    source_mass_kg: float
    node_a_name: str = ""
    node_b_name: str = ""


@dataclass(frozen=True)
class BeamDeck:
    run_name: str
    rod_model: rods.RodModel
    node_ids: dict[str, int]
    members: list[BeamDeckMember]
    admas_node_masses_kg: dict[str, float]
    starter_path: Path
    engine_path: Path
    summary_path: Path
    poster_path: Path
    gif_path: Path
    node_xyz_m: dict[str, np.ndarray] = field(default_factory=dict)
    node_interpolations: dict[str, tuple[str, str, float]] = field(default_factory=dict)
    target_element_length_mm: float | None = None
    use_nominal_radius_for_massless_members: bool = False
    uniform_radius_mm: float | None = None


def fmt(value: float) -> str:
    return f"{value:.14E}"


def real20(value: float) -> str:
    return f"{value:20.12E}"


def int10(value: int) -> str:
    return f"{value:10d}"


def sanitize_title(value: str, limit: int = 80) -> str:
    allowed = []
    for char in value:
        if char.isalnum() or char in {"_", "-", " ", ":", "/", "."}:
            allowed.append(char)
        else:
            allowed.append("_")
    return "".join(allowed)[:limit] or "unnamed"


def circular_section_from_area(area_mm2: float) -> tuple[float, float, float]:
    radius2 = max(area_mm2, 1.0e-12) / math.pi
    i_bending = math.pi * radius2 * radius2 / 4.0
    i_torsion = 2.0 * i_bending
    return i_bending, i_bending, i_torsion


def member_area_mm2(
    member: rods.RodMember,
    length_mm: float,
    density_kg_mm3: float,
    use_nominal_radius_for_massless_members: bool = False,
    uniform_radius_mm: float | None = None,
) -> tuple[float, float]:
    if uniform_radius_mm is not None:
        radius_mm = max(float(uniform_radius_mm), MIN_CONNECTOR_RADIUS_MM)
        area = math.pi * radius_mm * radius_mm
        structural_mass = density_kg_mm3 * area * length_mm
        return area, structural_mass

    if member.mass_kg > 0.0 and length_mm > 0.0 and density_kg_mm3 > 0.0:
        area = member.mass_kg / (density_kg_mm3 * length_mm)
        return area, member.mass_kg

    if use_nominal_radius_for_massless_members:
        radius_mm = max(MIN_CONNECTOR_RADIUS_MM, member.nominal_radius_m * RADIOSS_LENGTH_SCALE)
    else:
        radius_mm = MIN_CONNECTOR_RADIUS_MM
    area = math.pi * radius_mm * radius_mm
    structural_mass = density_kg_mm3 * area * length_mm
    return area, structural_mass


def build_beam_deck(
    rod_model: rods.RodModel,
    out_dir: Path,
    run_name: str = RUN_NAME,
    target_element_length_mm: float | None = None,
    use_nominal_radius_for_massless_members: bool = False,
    uniform_radius_mm: float | None = None,
) -> BeamDeck:
    out_dir.mkdir(parents=True, exist_ok=True)
    by_name = {node.name: node.xyz_m for node in rod_model.nodes}
    node_ids: dict[str, int] = {}
    node_xyz_m: dict[str, np.ndarray] = {}
    node_interpolations: dict[str, tuple[str, str, float]] = {}

    def add_node(name: str, xyz_m: np.ndarray, interpolation: tuple[str, str, float]) -> None:
        if name in node_ids:
            return
        node_ids[name] = len(node_ids) + 1
        node_xyz_m[name] = np.array(xyz_m, dtype=float)
        node_interpolations[name] = interpolation

    for node in rod_model.nodes:
        add_node(node.name, node.xyz_m, (node.name, node.name, 0.0))

    density_kg_mm3 = 1240.0 * RADIOSS_DENSITY_SCALE
    members: list[BeamDeckMember] = []
    beam_id = 1
    for member in rod_model.members:
        a = by_name[member.node_a]
        b = by_name[member.node_b]
        length_mm = float(np.linalg.norm(b - a) * RADIOSS_LENGTH_SCALE)
        segment_count = 1
        if target_element_length_mm is not None and target_element_length_mm > 0.0:
            segment_count = max(1, int(math.ceil(length_mm / target_element_length_mm)))
        area, structural_mass = member_area_mm2(
            member,
            length_mm,
            density_kg_mm3,
            use_nominal_radius_for_massless_members,
            uniform_radius_mm,
        )
        iyy, izz, ixx = circular_section_from_area(area)
        previous_node_name = member.node_a
        for segment_idx in range(1, segment_count + 1):
            t = segment_idx / segment_count
            if segment_idx == segment_count:
                next_node_name = member.node_b
            else:
                next_node_name = f"{member.name}_seg_{segment_idx:02d}"
                add_node(
                    next_node_name,
                    (1.0 - t) * a + t * b,
                    (member.node_a, member.node_b, t),
                )
            members.append(
                BeamDeckMember(
                    member=member,
                    beam_id=beam_id,
                    part_id=beam_id,
                    prop_id=beam_id,
                    node_a_id=node_ids[previous_node_name],
                    node_b_id=node_ids[next_node_name],
                    length_mm=length_mm / segment_count,
                    area_mm2=area,
                    iyy_mm4=iyy,
                    izz_mm4=izz,
                    ixx_mm4=ixx,
                    structural_mass_kg=structural_mass / segment_count,
                    source_mass_kg=member.mass_kg / segment_count,
                    node_a_name=previous_node_name,
                    node_b_name=next_node_name,
                )
            )
            previous_node_name = next_node_name
            beam_id += 1

    admas_node_masses: dict[str, float] = {}
    for mass in rod_model.lumped_masses:
        admas_node_masses[mass.attached_node] = admas_node_masses.get(mass.attached_node, 0.0) + mass.mass_kg

    return BeamDeck(
        run_name=run_name,
        rod_model=rod_model,
        node_ids=node_ids,
        members=members,
        admas_node_masses_kg=admas_node_masses,
        starter_path=out_dir / f"{run_name}_0000.rad",
        engine_path=out_dir / f"{run_name}_0001.rad",
        summary_path=out_dir / "openradioss_beam_deck_summary.yaml",
        poster_path=out_dir / "openradioss_whole_body_beam_deck_poster.png",
        gif_path=out_dir / "openradioss_whole_body_beam_deck.gif",
        node_xyz_m=node_xyz_m,
        node_interpolations=node_interpolations,
        target_element_length_mm=target_element_length_mm,
        use_nominal_radius_for_massless_members=use_nominal_radius_for_massless_members,
        uniform_radius_mm=uniform_radius_mm,
    )


def write_starter(deck: BeamDeck) -> None:
    material = deck.rod_model.source_model.geometry
    _ = material
    density = 1240.0 * RADIOSS_DENSITY_SCALE
    youngs = 3.2e9 * RADIOSS_MODULUS_SCALE
    poisson = 0.36

    lines: list[str] = [
        "#RADIOSS STARTER",
        "# Stage 2 whole-body OpenRadioss beam deck generated from stage2_rod_model.py.",
        "# Topology-only: no gravity, no fixed feet, no support reaction, no load case.",
        "#---1----|----2----|----3----|----4----|----5----|----6----|----7----|----8----|----9----|---10----|",
        "/BEGIN",
        deck.run_name,
        "      2026         0",
        "                  kg                  mm                  ms",
        "                  kg                  mm                  ms",
        "##",
        "/MAT/ELAST/1",
        "PLA_PLUS_STAGE2_ELASTIC_PLACEHOLDER",
        real20(density),
        f"{real20(youngs)}{real20(poisson)}",
        "##",
    ]

    for item in deck.members:
        title = sanitize_title(f"{item.member.name} A={item.area_mm2:.3f}mm2")
        lines.extend(
            [
                f"/PROP/TYPE3/{item.prop_id}",
                title,
                "                                                                                ",
                f"{real20(0.0)}{real20(0.0)}",
                f"{real20(item.area_mm2)}{real20(item.iyy_mm4)}{real20(item.izz_mm4)}{real20(item.ixx_mm4)}",
                "                                                                                ",
                f"/PART/{item.part_id}",
                sanitize_title(f"beam_part_{item.member.name}"),
                f"{int10(item.prop_id)}{int10(1)}{int10(0)}{real20(0.0)}",
            ]
        )

    lines.append("/NODE")
    source_node_xyz = {node.name: node.xyz_m for node in deck.rod_model.nodes}
    node_xyz = deck.node_xyz_m or source_node_xyz
    for node_name, node_id in sorted(deck.node_ids.items(), key=lambda item: item[1]):
        x, y, z = node_xyz[node_name] * RADIOSS_LENGTH_SCALE
        lines.append(f"{node_id:10d}{x:20.8f}{y:20.8f}{z:20.8f}")

    for item in deck.members:
        lines.append(f"/BEAM/{item.part_id}")
        lines.append(f"{item.beam_id:10d}{item.node_a_id:10d}{item.node_b_id:10d}")

    if deck.admas_node_masses_kg:
        lines.extend(
            [
                "/ADMAS/5/1",
                "stage1_payload_actuator_electronics_footpad_masses_at_nearest_rod_nodes",
            ]
        )
        for node_name, mass_kg in sorted(deck.admas_node_masses_kg.items(), key=lambda item: deck.node_ids[item[0]]):
            lines.append(f"{real20(mass_kg)}{int10(deck.node_ids[node_name])}")

    lines.extend(["/END", ""])
    deck.starter_path.write_text("\n".join(lines), encoding="utf-8")


def write_engine(deck: BeamDeck, stop_time_ms: float = 0.1) -> None:
    lines = [
        "#RADIOSS ENGINE",
        "/VERS/2026",
        f"/RUN/{deck.run_name}/1/",
        fmt(stop_time_ms),
        "/H3D/NODA/VEL",
        "/H3D/DT",
        "0.0 0.01",
        "/TFILE/0",
        "0.01",
        "/PRINT/-100",
        "/MON/ON",
        "/DT/NODA/CST/0",
        "0.900000000000000    0.000000000000000",
        "",
    ]
    deck.engine_path.write_text("\n".join(lines), encoding="utf-8")


def deck_mass_kg(deck: BeamDeck) -> float:
    return sum(item.structural_mass_kg for item in deck.members) + sum(deck.admas_node_masses_kg.values())


def deck_com_m(deck: BeamDeck) -> np.ndarray:
    by_name = deck.node_xyz_m or {node.name: node.xyz_m for node in deck.rod_model.nodes}
    total = deck_mass_kg(deck)
    if total <= 0.0:
        return np.zeros(3)
    weighted = np.zeros(3)
    for item in deck.members:
        node_a_name = item.node_a_name or item.member.node_a
        node_b_name = item.node_b_name or item.member.node_b
        midpoint = 0.5 * (by_name[node_a_name] + by_name[node_b_name])
        weighted += item.structural_mass_kg * midpoint
    for node_name, mass_kg in deck.admas_node_masses_kg.items():
        weighted += mass_kg * by_name[node_name]
    return weighted / total


def summary_dict(deck: BeamDeck) -> dict[str, Any]:
    rod = deck.rod_model
    source_com = rod.source_model.com_m
    deck_com = deck_com_m(deck)
    return {
        "stage": "stage_2_openradioss_whole_body_beam_deck",
        "run_name": deck.run_name,
        "source": "stage2_rod_model whole-body rod graph",
        "openradioss_keywords": {
            "elements": "/BEAM",
            "property": "/PROP/TYPE3 (BEAM)",
            "part": "/PART",
            "material": "/MAT/LAW1 (ELAST)",
            "lumped_mass": "/ADMAS/5",
        },
        "analysis_state": {
            "topology_only": True,
            "gravity_applied": False,
            "fixed_boundary_conditions_applied": False,
            "support_reactions_applied": False,
            "load_cases_applied": False,
            "solved_deformation": False,
        },
        "counts": {
            "rod_graph_nodes": len(rod.nodes),
            "solver_nodes": len(deck.node_ids),
            "beam_elements": len(deck.members),
            "parts": len(deck.members),
            "properties": len(deck.members),
            "admas_nodes": len(deck.admas_node_masses_kg),
            "contact_candidate_nodes": sum(1 for node in rod.nodes if node.contact_candidate),
        },
        "mass": {
            "stage1_source_total_mass_kg": float(rod.source_model.total_mass_kg),
            "rod_model_total_mass_kg": float(rod.total_mass_kg),
            "openradioss_deck_total_mass_kg": float(deck_mass_kg(deck)),
            "structural_beam_mass_kg": float(sum(item.structural_mass_kg for item in deck.members)),
            "admas_lumped_mass_kg": float(sum(deck.admas_node_masses_kg.values())),
        },
        "com_m": {
            "stage1_source": {
                "x": float(source_com[0]),
                "y": float(source_com[1]),
                "z": float(source_com[2]),
            },
            "openradioss_deck_admas_at_nodes": {
                "x": float(deck_com[0]),
                "y": float(deck_com[1]),
                "z": float(deck_com[2]),
            },
            "delta_from_stage1": {
                "x": float(deck_com[0] - source_com[0]),
                "y": float(deck_com[1] - source_com[1]),
                "z": float(deck_com[2] - source_com[2]),
            },
        },
        "lumped_mass_policy": "ADMAS/5 masses are attached to nearest rod graph nodes; exact payload COM nodes are not introduced yet.",
        "beam_section_radius_policy": (
            f"All beam elements use a uniform circular radius of {deck.uniform_radius_mm:g} mm."
            if deck.uniform_radius_mm is not None
            else (
                "Mass-carrying rods derive area from source mass and length; zero-mass connector rods use their rod-model nominal radius."
                if deck.use_nominal_radius_for_massless_members
                else f"Mass-carrying rods derive area from source mass and length; zero-mass connector rods use the minimum {MIN_CONNECTOR_RADIUS_MM} mm radius."
            )
        ),
        "elementization_policy": (
            f"Members are split to target {deck.target_element_length_mm:g} mm beam elements."
            if deck.target_element_length_mm
            else "Each rod graph member is exported as one beam element."
        ),
        "starter_file": deck.starter_path.name,
        "engine_file": deck.engine_path.name,
        "notes": [
            "This is a whole-body OpenRadioss deck, not a single-link or subassembly FEM route.",
            "No gravity, no fixed feet, no support reaction, and no external load are present in this export.",
            "The Engine file is a no-load smoke run file; useful for checking the deck survives Starter/Engine, not for design conclusions.",
        ],
    }


def write_summary(deck: BeamDeck) -> None:
    with deck.summary_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(summary_dict(deck), handle, sort_keys=False)


def make_visualization(deck: BeamDeck, frames: int, duration_ms: int) -> None:
    images: list[Image.Image] = []
    for idx in range(frames):
        phase = 0.0 if frames <= 1 else idx / (frames - 1)
        azim = -56.0 + 112.0 * phase
        elev = 24.0 + 7.0 * math.sin(2.0 * math.pi * phase)
        images.append(rods.render_frame(deck.rod_model, elev=elev, azim=azim, title="OpenRadioss Whole-Body Beam Deck"))
    poster = rods.render_frame(deck.rod_model, elev=27.0, azim=-42.0, title="OpenRadioss Whole-Body Beam Deck")
    poster.save(deck.poster_path)
    images[0].save(deck.gif_path, save_all=True, append_images=images[1:], duration=duration_ms, loop=0)


def write_deck(deck: BeamDeck, make_gif: bool, frames: int, duration_ms: int) -> None:
    write_starter(deck)
    write_engine(deck)
    write_summary(deck)
    if make_gif:
        make_visualization(deck, frames=max(1, frames), duration_ms=max(20, duration_ms))


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--description", type=Path, default=DEFAULT_DESCRIPTION_PATH)
    parser.add_argument("--materials", type=Path, default=stage1.DEFAULT_MATERIALS_PATH)
    parser.add_argument("--actuators", type=Path, default=stage1.DEFAULT_ACTUATORS_PATH)
    parser.add_argument("--batteries", type=Path, default=stage1.DEFAULT_BATTERIES_PATH)
    parser.add_argument("--case-name", default=rods.DEFAULT_CASE_NAME)
    parser.add_argument("--run-name", default=RUN_NAME)
    parser.add_argument("--waist-yaw-deg", type=float, default=None)
    parser.add_argument("--waist-pitch-deg", type=float, default=None)
    parser.add_argument("--out-dir", type=Path, default=Path("stage2_outputs/openradioss_beam_deck"))
    parser.add_argument("--target-element-length-mm", type=float, default=None)
    parser.add_argument("--use-nominal-radius-for-massless-members", action="store_true")
    parser.add_argument(
        "--uniform-radius-mm",
        type=float,
        default=None,
        help="Use one circular beam radius for every rod member, overriding mass-derived and nominal radii.",
    )
    parser.add_argument("--no-gif", action="store_true")
    parser.add_argument("--frames", type=int, default=36)
    parser.add_argument("--duration-ms", type=int, default=65)
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    description = load_dog_description(args.description)
    catalog = stage1.load_catalog(args.materials, args.actuators, args.batteries)
    source_model = rods.build_stage1_model(
        description,
        catalog,
        args.case_name,
        args.waist_yaw_deg,
        args.waist_pitch_deg,
    )
    rod_model = rods.build_whole_body_rod_model(source_model)
    deck = build_beam_deck(
        rod_model,
        args.out_dir,
        args.run_name,
        args.target_element_length_mm,
        args.use_nominal_radius_for_massless_members,
        args.uniform_radius_mm,
    )
    write_deck(deck, make_gif=not args.no_gif, frames=args.frames, duration_ms=args.duration_ms)

    print(f"wrote OpenRadioss whole-body beam deck to {args.out_dir}")
    print(f"starter: {deck.starter_path.name}")
    print(f"engine: {deck.engine_path.name}")
    print(f"rod graph nodes: {len(rod_model.nodes)}")
    print(f"solver nodes: {len(deck.node_ids)}")
    print(f"beam elements: {len(deck.members)}")
    print(f"admas nodes: {len(deck.admas_node_masses_kg)}")
    print(f"deck mass kg: {deck_mass_kg(deck):.9g}")
    print("analysis state: topology only; no gravity, no fixed feet, no support reaction")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
