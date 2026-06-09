#!/usr/bin/env python3
"""Stage 2 whole-body rod abstraction derived from the Stage 1 linkage model.

This is a coarse structural preprocessor, not solved FEM. It preserves the
whole-robot discipline: the body, waist, hips, legs, neck, and head claw are
represented together as one connected rod graph.
"""

from __future__ import annotations

import argparse
import csv
import io
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image

from dog_description import DEFAULT_DESCRIPTION_PATH, DogDescription, JointRange, load_dog_description
from endpoint_geometry import LEG_ORDER
import mass_model as stage1


DEFAULT_CASE_NAME = "stage2_viewer_rods"


@dataclass(frozen=True)
class RodNode:
    name: str
    xyz_m: np.ndarray
    role: str
    contact_candidate: bool = False


@dataclass(frozen=True)
class RodMember:
    name: str
    node_a: str
    node_b: str
    group: str
    source: str
    mass_kg: float = 0.0
    nominal_radius_m: float = 0.006


@dataclass(frozen=True)
class LumpedMass:
    name: str
    kind: str
    mass_kg: float
    xyz_m: np.ndarray
    attached_node: str
    source: str


@dataclass(frozen=True)
class RodModel:
    case_name: str
    source_model: stage1.MassModel
    nodes: list[RodNode]
    members: list[RodMember]
    lumped_masses: list[LumpedMass]
    skipped_zero_length_members: list[str]

    @property
    def total_member_mass_kg(self) -> float:
        return sum(member.mass_kg for member in self.members)

    @property
    def total_lumped_mass_kg(self) -> float:
        return sum(mass.mass_kg for mass in self.lumped_masses)

    @property
    def total_mass_kg(self) -> float:
        return self.total_member_mass_kg + self.total_lumped_mass_kg

    @property
    def com_m(self) -> np.ndarray:
        total = self.total_mass_kg
        if total <= 0.0:
            return np.zeros(3)
        by_name = {node.name: node.xyz_m for node in self.nodes}
        weighted = np.zeros(3)
        for member in self.members:
            midpoint = 0.5 * (by_name[member.node_a] + by_name[member.node_b])
            weighted += member.mass_kg * midpoint
        for mass in self.lumped_masses:
            weighted += mass.mass_kg * mass.xyz_m
        return weighted / total


def vtolist(vec: np.ndarray) -> list[float]:
    return [float(vec[0]), float(vec[1]), float(vec[2])]


def ranged_joint_angle_deg(ranges: dict[str, JointRange], name: str) -> float:
    joint = ranges[name]
    return max(joint.min_deg, min(joint.max_deg, joint.bias_deg))


def build_stage1_model(
    description: DogDescription,
    catalog: stage1.PhysicalCatalog,
    case_name: str,
    waist_yaw_deg: float | None = None,
    waist_pitch_deg: float | None = None,
) -> stage1.MassModel:
    ranges = description.joint_ranges
    yaw = ranged_joint_angle_deg(ranges, "waist_yaw") if waist_yaw_deg is None else waist_yaw_deg
    pitch = ranged_joint_angle_deg(ranges, "waist_pitch") if waist_pitch_deg is None else waist_pitch_deg
    return stage1.build_mass_model(
        case_name,
        description,
        catalog,
        stage1.Stage1Assumptions(),
        yaw,
        pitch,
    )


def structural_source_to_member_name(element_name: str) -> str | None:
    structural_links = {
        "front_body_shell": "front_body_spine",
        "rear_body_shell": "rear_body_spine",
        "waist_link_shell": "waist_yaw_pitch",
        "neck_link": "head_neck",
        "upper_claw_jaw": "head_upper_jaw",
        "lower_claw_jaw": "head_lower_jaw",
    }
    if element_name in structural_links:
        return structural_links[element_name]
    for leg_name in LEG_ORDER:
        prefix = f"{leg_name}_"
        if element_name == f"{prefix}upper_link":
            return f"{leg_name}_upper"
        if element_name == f"{prefix}lower_link":
            return f"{leg_name}_lower"
        if element_name == f"{prefix}toe_link":
            return f"{leg_name}_toe"
    return None


def build_whole_body_rod_model(model: stage1.MassModel) -> RodModel:
    nodes: dict[str, RodNode] = {}
    members: dict[str, RodMember] = {}
    skipped: list[str] = []

    def add_node(name: str, xyz_m: np.ndarray, role: str, contact_candidate: bool = False) -> str:
        nodes[name] = RodNode(
            name=name,
            xyz_m=np.array(xyz_m, dtype=float),
            role=role,
            contact_candidate=contact_candidate,
        )
        return name

    def add_member(
        name: str,
        node_a: str,
        node_b: str,
        group: str,
        source: str,
        nominal_radius_m: float = 0.006,
    ) -> None:
        length = float(np.linalg.norm(nodes[node_b].xyz_m - nodes[node_a].xyz_m))
        if length < 1e-9:
            skipped.append(name)
            return
        members[name] = RodMember(
            name=name,
            node_a=node_a,
            node_b=node_b,
            group=group,
            source=source,
            nominal_radius_m=nominal_radius_m,
        )

    pose = model.pose
    add_node("waist_yaw", pose.yaw_joint, "waist_joint")
    add_node("waist_pitch", pose.pitch_joint, "waist_joint")
    add_node("front_mid", pose.front_mid, "body_centerline")
    add_node("rear_mid", pose.rear_mid, "body_centerline")

    for leg_name in LEG_ORDER:
        chain = model.legs[leg_name]
        add_node(f"{leg_name}_hip", chain.hip, "hip")
        add_node(f"{leg_name}_knee", chain.knee, "knee")
        add_node(f"{leg_name}_toe_joint", chain.toe_joint, "toe_joint")
        add_node(f"{leg_name}_toe_endpoint", chain.toe_endpoint, "toe_endpoint", contact_candidate=True)

    head = model.head
    add_node("head_hinge", head.hinge, "head")
    add_node("head_upper_hinge", head.upper_hinge, "head")
    add_node("head_lower_hinge", head.lower_hinge, "head")
    add_node("head_upper_tip", head.upper_tip, "head")
    add_node("head_lower_tip", head.lower_tip, "head")

    add_member("waist_yaw_pitch", "waist_yaw", "waist_pitch", "waist", "viewer yaw->pitch", 0.010)
    add_member("front_body_spine", "waist_pitch", "front_mid", "body", "viewer pitch->front_mid", 0.012)
    add_member("rear_body_spine", "waist_yaw", "rear_mid", "body", "viewer yaw->rear_mid", 0.012)

    for leg_name in ("front_left", "front_right"):
        add_member(
            f"{leg_name}_hip_cross",
            "front_mid",
            f"{leg_name}_hip",
            "hip_cross",
            "viewer front hip cross-link split at body center",
            0.007,
        )
    for leg_name in ("rear_left", "rear_right"):
        add_member(
            f"{leg_name}_hip_cross",
            "rear_mid",
            f"{leg_name}_hip",
            "hip_cross",
            "viewer rear hip cross-link split at body center",
            0.007,
        )

    for leg_name in LEG_ORDER:
        add_member(f"{leg_name}_upper", f"{leg_name}_hip", f"{leg_name}_knee", "leg", "viewer hip->knee", 0.006)
        add_member(
            f"{leg_name}_lower",
            f"{leg_name}_knee",
            f"{leg_name}_toe_joint",
            "leg",
            "viewer knee->toe_joint",
            0.006,
        )
        add_member(
            f"{leg_name}_toe",
            f"{leg_name}_toe_joint",
            f"{leg_name}_toe_endpoint",
            "toe",
            "viewer toe_joint->toe_endpoint",
            0.0045,
        )

    add_member("head_neck", "front_mid", "head_hinge", "head", "viewer neck_origin->hinge", 0.0045)
    add_member("head_upper_hinge_mount", "head_hinge", "head_upper_hinge", "head", "solver hinge mount split", 0.003)
    add_member("head_lower_hinge_mount", "head_hinge", "head_lower_hinge", "head", "solver hinge mount split", 0.003)
    add_member("head_upper_jaw", "head_upper_hinge", "head_upper_tip", "head", "viewer upper jaw", 0.004)
    add_member("head_lower_jaw", "head_lower_hinge", "head_lower_tip", "head", "viewer lower jaw", 0.004)

    member_masses = {name: 0.0 for name in members}
    lumped_masses: list[LumpedMass] = []
    node_names = list(nodes)
    node_xyz = np.array([nodes[name].xyz_m for name in node_names])

    def nearest_node(point: np.ndarray) -> str:
        dists = np.linalg.norm(node_xyz - point, axis=1)
        return node_names[int(np.argmin(dists))]

    for element in model.elements:
        member_name = structural_source_to_member_name(element.name)
        if element.kind == "printed_structure" and member_name is not None and member_name in member_masses:
            member_masses[member_name] += element.mass_kg
            continue
        lumped_masses.append(
            LumpedMass(
                name=element.name,
                kind=element.kind,
                mass_kg=element.mass_kg,
                xyz_m=np.array(element.com_m, dtype=float),
                attached_node=nearest_node(element.com_m),
                source="stage1 MassElement",
            )
        )

    members_with_mass = [
        RodMember(
            name=member.name,
            node_a=member.node_a,
            node_b=member.node_b,
            group=member.group,
            source=member.source,
            mass_kg=member_masses[member.name],
            nominal_radius_m=member.nominal_radius_m,
        )
        for member in members.values()
    ]

    return RodModel(
        case_name=model.case_name,
        source_model=model,
        nodes=list(nodes.values()),
        members=members_with_mass,
        lumped_masses=lumped_masses,
        skipped_zero_length_members=skipped,
    )


def connected_node_names(rod_model: RodModel) -> set[str]:
    if not rod_model.nodes:
        return set()
    edges: dict[str, set[str]] = {node.name: set() for node in rod_model.nodes}
    for member in rod_model.members:
        edges[member.node_a].add(member.node_b)
        edges[member.node_b].add(member.node_a)
    seen = {rod_model.nodes[0].name}
    queue: deque[str] = deque(seen)
    while queue:
        name = queue.popleft()
        for other in edges[name]:
            if other not in seen:
                seen.add(other)
                queue.append(other)
    return seen


def write_nodes_csv(rod_model: RodModel, path: Path) -> None:
    fields = ["name", "x_m", "y_m", "z_m", "role", "contact_candidate"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for node in rod_model.nodes:
            writer.writerow(
                {
                    "name": node.name,
                    "x_m": f"{node.xyz_m[0]:.9g}",
                    "y_m": f"{node.xyz_m[1]:.9g}",
                    "z_m": f"{node.xyz_m[2]:.9g}",
                    "role": node.role,
                    "contact_candidate": "yes" if node.contact_candidate else "no",
                }
            )


def write_members_csv(rod_model: RodModel, path: Path) -> None:
    fields = [
        "name",
        "node_a",
        "node_b",
        "group",
        "length_m",
        "mass_kg",
        "nominal_radius_m",
        "source",
    ]
    by_name = {node.name: node.xyz_m for node in rod_model.nodes}
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for member in rod_model.members:
            length = float(np.linalg.norm(by_name[member.node_b] - by_name[member.node_a]))
            writer.writerow(
                {
                    "name": member.name,
                    "node_a": member.node_a,
                    "node_b": member.node_b,
                    "group": member.group,
                    "length_m": f"{length:.9g}",
                    "mass_kg": f"{member.mass_kg:.9g}",
                    "nominal_radius_m": f"{member.nominal_radius_m:.9g}",
                    "source": member.source,
                }
            )


def write_lumped_masses_csv(rod_model: RodModel, path: Path) -> None:
    fields = ["name", "kind", "mass_kg", "x_m", "y_m", "z_m", "attached_node", "source"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for mass in rod_model.lumped_masses:
            writer.writerow(
                {
                    "name": mass.name,
                    "kind": mass.kind,
                    "mass_kg": f"{mass.mass_kg:.9g}",
                    "x_m": f"{mass.xyz_m[0]:.9g}",
                    "y_m": f"{mass.xyz_m[1]:.9g}",
                    "z_m": f"{mass.xyz_m[2]:.9g}",
                    "attached_node": mass.attached_node,
                    "source": mass.source,
                }
            )


def summary_dict(rod_model: RodModel) -> dict[str, Any]:
    source = rod_model.source_model
    connected = connected_node_names(rod_model)
    contact_nodes = [node.name for node in rod_model.nodes if node.contact_candidate]
    return {
        "stage": "stage_2_whole_body_rod_abstraction",
        "case_name": rod_model.case_name,
        "source": "derived from Stage 1 MassModel / pygame_mass_viewer linkage topology",
        "whole_robot_only": True,
        "counts": {
            "nodes": len(rod_model.nodes),
            "members": len(rod_model.members),
            "lumped_masses": len(rod_model.lumped_masses),
            "contact_candidate_nodes": len(contact_nodes),
            "connected_nodes": len(connected),
        },
        "analysis_state": {
            "topology_only": True,
            "gravity_applied": False,
            "fixed_boundary_conditions_applied": False,
            "support_reactions_applied": False,
            "load_cases_applied": False,
            "solved_deformation": False,
        },
        "mass": {
            "source_stage1_total_mass_kg": float(source.total_mass_kg),
            "rod_total_mass_kg": float(rod_model.total_mass_kg),
            "member_mass_kg": float(rod_model.total_member_mass_kg),
            "lumped_mass_kg": float(rod_model.total_lumped_mass_kg),
        },
        "com_m": {
            "stage1": {
                "x": float(source.com_m[0]),
                "y": float(source.com_m[1]),
                "z": float(source.com_m[2]),
            },
            "rod_model": {
                "x": float(rod_model.com_m[0]),
                "y": float(rod_model.com_m[1]),
                "z": float(rod_model.com_m[2]),
            },
        },
        "contact_candidate_nodes": contact_nodes,
        "zero_mass_connector_members": [member.name for member in rod_model.members if member.mass_kg <= 0.0],
        "skipped_zero_length_members": rod_model.skipped_zero_length_members,
        "notes": [
            "This is a connected whole-body rod graph, not a single-part or subassembly FEM route.",
            "No gravity, support reaction, or fixed boundary condition is applied in this topology export.",
            "Hip cross-links are split at the body centerline so the solver graph is connected.",
            "Payloads, actuators, and foot pads remain lumped masses with their Stage 1 COM positions.",
            "The GIF is topology and mass visualization only; it is not solved deformation.",
        ],
    }


def write_summary_yaml(rod_model: RodModel, path: Path) -> None:
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(summary_dict(rod_model), handle, sort_keys=False)


def plot_rods(ax: Any, rod_model: RodModel, elev: float, azim: float) -> None:
    by_name = {node.name: node.xyz_m for node in rod_model.nodes}
    colors = {
        "body": "#9aa5b1",
        "waist": "#f59e0b",
        "hip_cross": "#60a5fa",
        "leg": "#d1d5db",
        "toe": "#34d399",
        "head": "#f472b6",
    }
    for member in rod_model.members:
        a = by_name[member.node_a]
        b = by_name[member.node_b]
        width = 1.2 + 120.0 * member.nominal_radius_m
        alpha = 0.46 if member.mass_kg <= 0.0 else 0.92
        ax.plot(
            [a[0], b[0]],
            [a[1], b[1]],
            [a[2], b[2]],
            color=colors.get(member.group, "#cbd5e1"),
            linewidth=width,
            alpha=alpha,
        )

    nodes = np.array([node.xyz_m for node in rod_model.nodes])
    ax.scatter(nodes[:, 0], nodes[:, 1], nodes[:, 2], s=22, c="#f8fafc", edgecolors="#111827", linewidths=0.6)

    contact = np.array([node.xyz_m for node in rod_model.nodes if node.contact_candidate])
    if len(contact):
        ax.scatter(
            contact[:, 0],
            contact[:, 1],
            contact[:, 2],
            s=72,
            marker="v",
            c="none",
            edgecolors="#f97316",
            linewidths=1.3,
        )

    lump_style = {
        "battery": ("s", "#facc15", "#713f12", 260.0),
        "electronics": ("P", "#38bdf8", "#075985", 170.0),
        "actuator": ("D", "#fb7185", "#881337", 150.0),
        "foot_pad": ("X", "#f97316", "#7c2d12", 130.0),
    }
    for kind in sorted({mass.kind for mass in rod_model.lumped_masses}):
        marker, face, edge, scale = lump_style.get(kind, ("h", "#cbd5e1", "#334155", 140.0))
        kind_masses = [mass for mass in rod_model.lumped_masses if mass.kind == kind]
        coords = np.array([mass.xyz_m for mass in kind_masses])
        sizes = np.array([mass.mass_kg for mass in kind_masses])
        sizes = 26.0 + scale * sizes / max(float(sizes.max()), 1e-9)
        ax.scatter(
            coords[:, 0],
            coords[:, 1],
            coords[:, 2],
            s=sizes,
            marker=marker,
            c=face,
            alpha=0.70,
            edgecolors=edge,
            linewidths=0.9,
        )

    com = rod_model.com_m
    ax.scatter([com[0]], [com[1]], [com[2]], s=160, c="#5eead4", edgecolors="#0f172a", linewidths=1.0)

    mins = nodes.min(axis=0)
    maxs = nodes.max(axis=0)
    center = 0.5 * (mins + maxs)
    span = max(float((maxs - mins).max()), 0.1)
    radius = 0.58 * span
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(max(0.0, center[2] - radius * 0.74), center[2] + radius * 0.86)
    try:
        ax.set_box_aspect((1, 1, 0.62), zoom=1.38)
    except TypeError:
        ax.set_box_aspect((1, 1, 0.62))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()


def add_visual_legend(fig: Any) -> None:
    ax = fig.add_axes([0.705, 0.565, 0.265, 0.315])
    ax.set_facecolor("#111827")
    for spine in ax.spines.values():
        spine.set_edgecolor("#334155")
        spine.set_linewidth(0.8)
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.text(0.07, 0.91, "visual key", color="#f8fafc", fontsize=10.5, weight="bold")
    entries = [
        ("rod node", "o", "#f8fafc", "#111827"),
        ("toe/contact candidate, not fixed", "v", "none", "#f97316"),
        ("COM", "o", "#5eead4", "#0f172a"),
        ("battery mass", "s", "#facc15", "#713f12"),
        ("electronics mass", "P", "#38bdf8", "#075985"),
        ("actuator mass", "D", "#fb7185", "#881337"),
        ("foot-pad mass", "X", "#f97316", "#7c2d12"),
    ]
    for idx, (label, marker, face, edge) in enumerate(entries):
        y = 0.78 - idx * 0.105
        ax.scatter([0.10], [y], marker=marker, s=72, c=face, edgecolors=edge, linewidths=1.1)
        ax.text(0.19, y, label, color="#cbd5e1", fontsize=8.7, va="center")


def render_frame(rod_model: RodModel, elev: float, azim: float, title: str) -> Image.Image:
    fig = plt.figure(figsize=(12.8, 7.2), dpi=110)
    ax = fig.add_subplot(111, projection="3d")
    fig.patch.set_facecolor("#0f172a")
    ax.set_facecolor("#0f172a")
    ax.set_position([0.04, 0.04, 0.92, 0.86])
    plot_rods(ax, rod_model, elev, azim)
    fig.text(0.035, 0.935, title, color="#f8fafc", fontsize=18, weight="bold")
    fig.text(
        0.035,
        0.895,
        f"{len(rod_model.nodes)} nodes | {len(rod_model.members)} rods | "
        f"{rod_model.total_mass_kg:.3f} kg | COM "
        f"({rod_model.com_m[0]:+.3f}, {rod_model.com_m[1]:+.3f}, {rod_model.com_m[2]:+.3f}) m",
        color="#cbd5e1",
        fontsize=11,
    )
    add_visual_legend(fig)
    fig.text(
        0.035,
        0.055,
        "Topology only: no gravity, no fixed feet, no support reaction, no solved deformation",
        color="#94a3b8",
        fontsize=10,
    )
    buffer = io.BytesIO()
    fig.savefig(buffer, format="png", facecolor=fig.get_facecolor(), pad_inches=0.0)
    plt.close(fig)
    buffer.seek(0)
    return Image.open(buffer).convert("RGB")


def make_visualization(rod_model: RodModel, gif_path: Path, poster_path: Path, frames: int, duration_ms: int) -> None:
    gif_path.parent.mkdir(parents=True, exist_ok=True)
    poster_path.parent.mkdir(parents=True, exist_ok=True)
    images: list[Image.Image] = []
    for idx in range(frames):
        phase = 0.0 if frames <= 1 else idx / (frames - 1)
        azim = -56.0 + 112.0 * phase
        elev = 24.0 + 7.0 * math.sin(2.0 * math.pi * phase)
        images.append(render_frame(rod_model, elev=elev, azim=azim, title="Stage 2 Whole-Body Rod Model"))
    poster = render_frame(rod_model, elev=27.0, azim=-42.0, title="Stage 2 Whole-Body Rod Model")
    poster.save(poster_path)
    images[0].save(gif_path, save_all=True, append_images=images[1:], duration=duration_ms, loop=0)


def write_outputs(rod_model: RodModel, out_dir: Path, make_gif: bool, frames: int, duration_ms: int) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    write_nodes_csv(rod_model, out_dir / "nodes.csv")
    write_members_csv(rod_model, out_dir / "members.csv")
    write_lumped_masses_csv(rod_model, out_dir / "lumped_masses.csv")
    write_summary_yaml(rod_model, out_dir / "rod_model_summary.yaml")
    if make_gif:
        make_visualization(
            rod_model,
            out_dir / "stage2_whole_body_rods.gif",
            out_dir / "stage2_whole_body_rods_poster.png",
            frames,
            duration_ms,
        )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--description", type=Path, default=DEFAULT_DESCRIPTION_PATH)
    parser.add_argument("--materials", type=Path, default=stage1.DEFAULT_MATERIALS_PATH)
    parser.add_argument("--actuators", type=Path, default=stage1.DEFAULT_ACTUATORS_PATH)
    parser.add_argument("--batteries", type=Path, default=stage1.DEFAULT_BATTERIES_PATH)
    parser.add_argument("--case-name", default=DEFAULT_CASE_NAME)
    parser.add_argument("--waist-yaw-deg", type=float, default=None)
    parser.add_argument("--waist-pitch-deg", type=float, default=None)
    parser.add_argument("--out-dir", type=Path, default=Path("stage2_outputs/rod_model"))
    parser.add_argument("--no-gif", action="store_true")
    parser.add_argument("--frames", type=int, default=36)
    parser.add_argument("--duration-ms", type=int, default=70)
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    description = load_dog_description(args.description)
    catalog = stage1.load_catalog(args.materials, args.actuators, args.batteries)
    source_model = build_stage1_model(
        description,
        catalog,
        args.case_name,
        args.waist_yaw_deg,
        args.waist_pitch_deg,
    )
    rod_model = build_whole_body_rod_model(source_model)
    write_outputs(
        rod_model,
        args.out_dir,
        make_gif=not args.no_gif,
        frames=max(1, args.frames),
        duration_ms=max(20, args.duration_ms),
    )

    print(f"wrote Stage 2 whole-body rod model to {args.out_dir}")
    print(f"nodes: {len(rod_model.nodes)}")
    print(f"rods: {len(rod_model.members)}")
    print(f"lumped masses: {len(rod_model.lumped_masses)}")
    print(f"total mass kg: {rod_model.total_mass_kg:.6g}")
    print(f"connected nodes: {len(connected_node_names(rod_model))}/{len(rod_model.nodes)}")
    print(f"contact candidate nodes: {sum(1 for node in rod_model.nodes if node.contact_candidate)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
