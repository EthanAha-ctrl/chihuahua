#!/usr/bin/env python3
"""Generate and post-process a whole-body periodic-motion OpenRadioss beam case.

This case is intentionally different from the no-load smoke deck in
stage2_openradioss_deck.py. It applies kinematic displacement functions derived
from pygame_mass_viewer.py's periodic motion and then visualizes the solved
T01 displacement history as a whole-body rod FEM animation.
"""

from __future__ import annotations

import argparse
import csv
import io
import itertools
import math
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection, PatchCollection
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.cm import ScalarMappable
from matplotlib.patches import Circle, Polygon
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection
import numpy as np
import yaml
from PIL import Image

from dog_description import DEFAULT_DESCRIPTION_PATH, load_dog_description
from endpoint_geometry import LEG_ORDER
import mass_model as stage1
import pygame_mass_viewer as viewer_model
import stage2_openradioss_deck as beam_deck
import stage2_rod_model as rods


RUN_NAME = "stage2_whole_body_periodic_motion"
TH_DISP_GROUP = "rod_motion_displacement"
TH_REAC_GROUP = "rod_motion_reaction"
TH_BEAM_GROUP = "rod_beam_section_resultants"
YOUNGS_MPA = 3200.0
RADIOSS_FORCE_UNIT_TO_N = 1000.0
GLOBAL_HISTORY_COLUMNS = {
    "INTERNAL ENERGY",
    "KINETIC ENERGY",
    "EXTERNAL WORK",
    "PLASTIC WORK",
    "CONTACT ENERGY",
    "MASS",
    "TIME STEP",
}


@dataclass(frozen=True)
class MotionSample:
    time_ms: float
    viewer_time_s: float
    rod_model: rods.RodModel


@dataclass(frozen=True)
class PeriodicMotionCase:
    deck: beam_deck.BeamDeck
    samples: list[MotionSample]
    target_displacements_mm: dict[str, np.ndarray]
    control_node_names: tuple[str, ...]
    control_policy: str
    guide_stiffness: float
    guide_damping: float
    uniform_radius_mm: float | None
    torque_replay_samples: tuple[TorqueReplaySample, ...]
    torque_scale: float
    torque_replay_csv_path: Path
    viewer_start_seconds: float
    target_csv_path: Path
    result_gif_path: Path
    result_poster_path: Path
    result_3d_gif_path: Path
    result_3d_poster_path: Path


@dataclass(frozen=True)
class TorqueReplaySample:
    time_ms: float
    joint: str
    proximal_node: str
    distal_node: str
    axis: np.ndarray
    torque_nm: float
    notes: str


def real20(value: float) -> str:
    return f"{value:20.12E}"


def sanitize_title(value: str, limit: int = 80) -> str:
    return beam_deck.sanitize_title(value, limit=limit)


def unit_vector(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm <= 1.0e-12:
        raise ValueError("cannot normalize a zero-length vector")
    return np.array(vector, dtype=float) / norm


def make_viewer_state(description: Any, viewer_time_s: float, babble_scale: float) -> viewer_model.ViewerState:
    state = viewer_model.ViewerState(
        waist_babble=True,
        joint_babble=True,
        show_help=False,
        show_materials=False,
        babble_scale=babble_scale,
        sim_time=viewer_time_s,
    )
    viewer_model.update_autoplay(state, description)
    return state


def live_rod_model(
    description: Any,
    catalog: stage1.PhysicalCatalog,
    viewer_time_s: float,
    babble_scale: float,
    case_name: str,
) -> rods.RodModel:
    state = make_viewer_state(description, viewer_time_s, babble_scale)
    model = viewer_model.make_live_stage1_model(description, state, catalog)
    model = stage1.MassModel(
        case_name=case_name,
        geometry=model.geometry,
        pose=model.pose,
        legs=model.legs,
        head=model.head,
        elements=model.elements,
    )
    return rods.build_whole_body_rod_model(model)


def build_motion_samples(
    description: Any,
    catalog: stage1.PhysicalCatalog,
    sample_count: int,
    solver_duration_ms: float,
    viewer_start_seconds: float,
    viewer_motion_seconds: float,
    babble_scale: float,
    case_name: str,
) -> list[MotionSample]:
    if sample_count < 2:
        raise ValueError("sample_count must be at least 2 for a motion case")
    samples: list[MotionSample] = []
    for idx in range(sample_count):
        phase = idx / (sample_count - 1)
        solver_time_ms = solver_duration_ms * phase
        viewer_time_s = viewer_start_seconds + viewer_motion_seconds * phase
        samples.append(
            MotionSample(
                time_ms=solver_time_ms,
                viewer_time_s=viewer_time_s,
                rod_model=live_rod_model(description, catalog, viewer_time_s, babble_scale, case_name),
            )
        )
    return samples


def node_positions_mm(rod_model: rods.RodModel) -> dict[str, np.ndarray]:
    return {node.name: np.array(node.xyz_m, dtype=float) * beam_deck.RADIOSS_LENGTH_SCALE for node in rod_model.nodes}


def deck_node_positions_mm(deck: beam_deck.BeamDeck, sample: MotionSample) -> dict[str, np.ndarray]:
    rod_positions = node_positions_mm(sample.rod_model)
    output: dict[str, np.ndarray] = {}
    for node_name in deck.node_ids:
        if node_name in rod_positions:
            output[node_name] = rod_positions[node_name]
            continue
        node_a, node_b, t = deck.node_interpolations[node_name]
        output[node_name] = (1.0 - t) * rod_positions[node_a] + t * rod_positions[node_b]
    return output


def target_displacements(
    deck: beam_deck.BeamDeck,
    samples: list[MotionSample],
    motion_scale: float,
) -> dict[str, np.ndarray]:
    initial = deck_node_positions_mm(deck, samples[0])
    output: dict[str, list[np.ndarray]] = {name: [] for name in initial}
    for sample in samples:
        current = deck_node_positions_mm(deck, sample)
        for name, xyz0 in initial.items():
            output[name].append((current[name] - xyz0) * motion_scale)
    return {name: np.vstack(values) for name, values in output.items()}


def select_control_node_names(rod_model: rods.RodModel, policy: str) -> tuple[str, ...]:
    if policy in {"uniform-joint-guides", "all-joints-hard", "stage1-torque-replay"}:
        return tuple(node.name for node in rod_model.nodes)
    raise ValueError(f"unknown control policy: {policy}")


def torque_replay_specs(model: stage1.MassModel, row: stage1.TorqueRow) -> list[tuple[str, str, np.ndarray]]:
    joint = row.joint
    if joint == "waist_yaw":
        return [("waist_yaw", "waist_pitch", np.array([0.0, 0.0, 1.0], dtype=float))]
    if joint == "waist_pitch":
        return [("waist_pitch", "front_mid", model.pose.front_left)]
    if joint == "neck_yaw":
        return [("front_mid", "head_hinge", model.pose.front_up)]
    if joint == "neck_pitch":
        return [("front_mid", "head_hinge", model.pose.front_left)]
    if joint == "head_claw":
        half_axis = model.pose.front_left
        return [
            ("head_hinge", "head_upper_tip", half_axis),
            ("head_hinge", "head_lower_tip", half_axis),
        ]

    for leg_name in LEG_ORDER:
        prefix = f"{leg_name}_"
        if not joint.startswith(prefix):
            continue
        joint_type = joint.removeprefix(prefix)
        forward, outward, _down = model.pose.bases[leg_name]
        if joint_type == "hip_ab":
            return [(f"{leg_name}_hip", f"{leg_name}_knee", forward)]
        if joint_type == "hip_pitch":
            return [(f"{leg_name}_hip", f"{leg_name}_knee", outward)]
        if joint_type == "knee_bend":
            return [(f"{leg_name}_knee", f"{leg_name}_toe_joint", outward)]
        if joint_type == "toe_bend":
            return [(f"{leg_name}_toe_joint", f"{leg_name}_toe_endpoint", outward)]

    raise ValueError(f"no torque replay node mapping for {joint}")


def build_torque_replay_samples(
    samples: list[MotionSample],
    catalog: stage1.PhysicalCatalog,
    torque_scale: float,
) -> tuple[TorqueReplaySample, ...]:
    output: list[TorqueReplaySample] = []
    for sample in samples:
        model = sample.rod_model.source_model
        for row in stage1.estimate_torques(model, catalog):
            specs = torque_replay_specs(model, row)
            split = 1.0 / len(specs)
            for proximal_node, distal_node, axis in specs:
                output.append(
                    TorqueReplaySample(
                        time_ms=sample.time_ms,
                        joint=row.joint,
                        proximal_node=proximal_node,
                        distal_node=distal_node,
                        axis=unit_vector(axis),
                        torque_nm=row.required_torque_nm * torque_scale * split,
                        notes=row.notes,
                    )
                )
    return tuple(output)


def torque_replay_pair_series(case: PeriodicMotionCase) -> dict[tuple[str, str, str], np.ndarray]:
    times = [round(sample.time_ms, 12) for sample in case.samples]
    time_index = {time_ms: idx for idx, time_ms in enumerate(times)}
    series: dict[tuple[str, str, str], np.ndarray] = {}
    for sample in case.torque_replay_samples:
        key = (sample.joint, sample.proximal_node, sample.distal_node)
        values = series.setdefault(key, np.zeros((len(case.samples), 3), dtype=float))
        values[time_index[round(sample.time_ms, 12)]] += sample.torque_nm * sample.axis
    return series


def torque_replay_component_count(case: PeriodicMotionCase) -> int:
    if case.control_policy != "stage1-torque-replay":
        return 0
    count = 0
    for values in torque_replay_pair_series(case).values():
        for component_idx in range(3):
            if np.any(np.abs(values[:, component_idx]) > 1.0e-12):
                count += 2
    return count


def guide_node_name(node_name: str) -> str:
    return f"guide_target_{node_name}"


def guide_node_ids(case: PeriodicMotionCase) -> dict[str, int]:
    first_guide_id = max(case.deck.node_ids.values()) + 1
    return {
        node_name: first_guide_id + idx
        for idx, node_name in enumerate(case.control_node_names)
    }


def write_motion_targets_csv(case: PeriodicMotionCase) -> None:
    case.target_csv_path.parent.mkdir(parents=True, exist_ok=True)
    node_ids = case.deck.node_ids
    initial = deck_node_positions_mm(case.deck, case.samples[0])
    control_nodes = set(case.control_node_names)
    with case.target_csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "time_ms",
                "viewer_time_s",
                "node_id",
                "node_name",
                "x_target_mm",
                "y_target_mm",
                "z_target_mm",
                "dx_target_mm",
                "dy_target_mm",
                "dz_target_mm",
                "prescribed_in_solver",
                "motion_application",
            ],
        )
        writer.writeheader()
        control_nodes = set(case.control_node_names)
        for sample_idx, sample in enumerate(case.samples):
            for name in node_ids:
                disp = case.target_displacements_mm[name][sample_idx]
                target = initial[name] + disp
                if name in control_nodes and case.control_policy == "uniform-joint-guides":
                    prescribed = "no"
                    application = "uniform ghost target spring guide"
                elif name in control_nodes and case.control_policy == "stage1-torque-replay":
                    prescribed = "no"
                    application = "stage1 torque replay reference pose only"
                elif name in control_nodes:
                    prescribed = "yes"
                    application = "hard prescribed robot joint"
                else:
                    prescribed = "no"
                    application = "solved beam mesh node"
                writer.writerow(
                    {
                        "time_ms": f"{sample.time_ms:.9g}",
                        "viewer_time_s": f"{sample.viewer_time_s:.9g}",
                        "node_id": node_ids[name],
                        "node_name": name,
                        "x_target_mm": f"{target[0]:.9g}",
                        "y_target_mm": f"{target[1]:.9g}",
                        "z_target_mm": f"{target[2]:.9g}",
                        "dx_target_mm": f"{disp[0]:.9g}",
                        "dy_target_mm": f"{disp[1]:.9g}",
                        "dz_target_mm": f"{disp[2]:.9g}",
                        "prescribed_in_solver": prescribed,
                        "motion_application": application,
                    }
                )


def write_torque_replay_csv(case: PeriodicMotionCase) -> None:
    case.torque_replay_csv_path.parent.mkdir(parents=True, exist_ok=True)
    with case.torque_replay_csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "time_ms",
                "joint",
                "proximal_node",
                "distal_node",
                "axis_x",
                "axis_y",
                "axis_z",
                "torque_nm",
                "distal_mx_nm",
                "distal_my_nm",
                "distal_mz_nm",
                "proximal_mx_nm",
                "proximal_my_nm",
                "proximal_mz_nm",
                "notes",
            ],
        )
        writer.writeheader()
        for sample in case.torque_replay_samples:
            distal = sample.torque_nm * sample.axis
            proximal = -distal
            writer.writerow(
                {
                    "time_ms": f"{sample.time_ms:.9g}",
                    "joint": sample.joint,
                    "proximal_node": sample.proximal_node,
                    "distal_node": sample.distal_node,
                    "axis_x": f"{sample.axis[0]:.9g}",
                    "axis_y": f"{sample.axis[1]:.9g}",
                    "axis_z": f"{sample.axis[2]:.9g}",
                    "torque_nm": f"{sample.torque_nm:.9g}",
                    "distal_mx_nm": f"{distal[0]:.9g}",
                    "distal_my_nm": f"{distal[1]:.9g}",
                    "distal_mz_nm": f"{distal[2]:.9g}",
                    "proximal_mx_nm": f"{proximal[0]:.9g}",
                    "proximal_my_nm": f"{proximal[1]:.9g}",
                    "proximal_mz_nm": f"{proximal[2]:.9g}",
                    "notes": sample.notes,
                }
            )


def function_card(function_id: int, title: str, times_ms: np.ndarray, values_mm: np.ndarray) -> list[str]:
    lines = [f"/FUNCT/{function_id}", sanitize_title(title)]
    for time_ms, value_mm in zip(times_ms, values_mm):
        lines.append(f"{real20(float(time_ms))}{real20(float(value_mm))}")
    return lines


def group_card(group_id: int, title: str, node_id: int) -> list[str]:
    return [f"/GRNOD/NODE/{group_id}", sanitize_title(title), f"{node_id:10d}"]


def impdisp_card(
    impdisp_id: int,
    title: str,
    function_id: int,
    direction: str,
    group_id: int,
    stop_time_ms: float,
) -> list[str]:
    blank = " " * 10
    return [
        f"/IMPDISP/{impdisp_id}",
        sanitize_title(title),
        f"{function_id:10d}{direction:>10}{blank}{blank}{group_id:10d}{0:10d}",
        f"{real20(1.0)}{real20(1.0)}{real20(0.0)}{real20(stop_time_ms)}",
    ]


def cload_card(cload_id: int, title: str, function_id: int, direction: str, group_id: int) -> list[str]:
    return [
        f"/CLOAD/{cload_id}",
        sanitize_title(title),
        f"{function_id:10d}{direction:>10}{0:10d}{0:10d}{group_id:10d}{0:10d}{real20(1.0)}{real20(1.0)}",
    ]


def th_node_card(group_id: int, title: str, var_line: str, node_ids: Iterable[int]) -> list[str]:
    lines = [f"/TH/NODE/{group_id}", sanitize_title(title), var_line]
    for node_id in node_ids:
        lines.append(f"{node_id:10d}{0:10d}")
    return lines


def th_beam_card(group_id: int, title: str, var_line: str, members: Iterable[beam_deck.BeamDeckMember]) -> list[str]:
    lines = [f"/TH/BEAM/{group_id}", sanitize_title(title), var_line]
    for item in members:
        lines.append(f"{item.beam_id:10d}          {sanitize_title(beam_element_key(item))}")
    return lines


def spring_gene_property_card(prop_id: int, title: str, stiffness: float, damping: float) -> list[str]:
    spring_mass = 1.0e-4
    spring_inertia = 1.0e-4
    lines = [
        f"/PROP/TYPE8/{prop_id}",
        sanitize_title(title),
        f"{real20(spring_mass)}{real20(spring_inertia)}{0:10d}{0:10d}{0:10d}{0:10d}{0:10d}{0:10d}",
    ]
    for mode_idx in range(6):
        k = stiffness if mode_idx < 3 else 0.0
        c = damping if mode_idx < 3 else 0.0
        lines.extend(
            [
                f"{real20(k)}{real20(c)}{real20(0.0)}{real20(0.0)}{real20(0.0)}",
                f"{0:10d}{0:10d}{0:10d}{0:10d}{0:10d}{'':20s}{real20(0.0)}{real20(0.0)}",
                f"{real20(0.0)}{real20(0.0)}{real20(1.0)}{real20(1.0)}",
            ]
        )
    lines.append(f"{real20(0.0)}{real20(0.0)}")
    return lines


def spring_part_card(part_id: int, prop_id: int, title: str) -> list[str]:
    return [
        f"/PART/{part_id}",
        sanitize_title(title),
        f"{prop_id:10d}{0:10d}{0:10d}{real20(0.0)}",
    ]


def spring_element_card(part_id: int, spring_items: Iterable[tuple[int, int, int]]) -> list[str]:
    lines = [f"/SPRING/{part_id}"]
    for spring_id, robot_node_id, guide_node_id in spring_items:
        lines.append(
            f"{spring_id:10d}{robot_node_id:10d}{guide_node_id:10d}{0:10d}{0:10d}{0:10d}{0:10d}{'':20s}{0:10d}"
        )
    return lines


def append_motion_cards(case: PeriodicMotionCase) -> None:
    starter = case.deck.starter_path.read_text(encoding="utf-8").splitlines()
    while starter and starter[-1] == "":
        starter.pop()
    if starter[-1] != "/END":
        raise ValueError(f"{case.deck.starter_path} does not end with /END")
    starter.pop()

    times_ms = np.array([sample.time_ms for sample in case.samples], dtype=float)
    control_nodes = set(case.control_node_names)
    control_node_items = [
        item for item in sorted(case.deck.node_ids.items(), key=lambda item: item[1]) if item[0] in control_nodes
    ]
    history_node_items = sorted(case.deck.node_ids.items(), key=lambda item: item[1])
    load_description = (
        "stage1 torque replay moment-couple loads"
        if case.control_policy == "stage1-torque-replay"
        else "periodic viewer-motion uniform joint guide loads"
    )
    lines: list[str] = [
        "##",
        f"## Periodic whole-body {load_description}",
        "## Source: pygame_mass_viewer.py constrained periodic motion.",
        "## No gravity, no contact, and no fixed feet are used in this case.",
    ]
    function_id = 1001
    group_base = 2001
    impdisp_id = 3001
    directions = ("X", "Y", "Z")

    if case.control_policy == "stage1-torque-replay":
        pair_series = torque_replay_pair_series(case)
        load_nodes = sorted(
            {node_name for _joint, proximal, distal in pair_series for node_name in (proximal, distal)},
            key=lambda name: case.deck.node_ids[name],
        )
        load_group_ids = {node_name: group_base + idx for idx, node_name in enumerate(load_nodes)}
        lines.extend(
            [
                "##",
                "## Stage 1 torque rows replayed as equal-and-opposite nodal moment couples.",
                "## No target displacement or ghost spring is applied in this load policy.",
            ]
        )
        for node_name in load_nodes:
            lines.extend(group_card(load_group_ids[node_name], f"torque_load_node_{node_name}", case.deck.node_ids[node_name]))

        cload_id = impdisp_id
        moment_directions = ("XX", "YY", "ZZ")
        for pair_idx, ((joint, proximal_node, distal_node), values) in enumerate(sorted(pair_series.items())):
            for component_idx, direction in enumerate(moment_directions):
                component = values[:, component_idx]
                if not np.any(np.abs(component) > 1.0e-12):
                    continue
                for node_name, sign, end_label in (
                    (distal_node, 1.0, "distal"),
                    (proximal_node, -1.0, "proximal"),
                ):
                    load_values = sign * component
                    title = f"{joint}_{end_label}_{node_name}_{direction}_moment"
                    lines.extend(function_card(function_id, title, times_ms, load_values))
                    lines.extend(cload_card(cload_id, title, function_id, direction, load_group_ids[node_name]))
                    function_id += 1
                    cload_id += 1

    elif case.control_policy == "uniform-joint-guides":
        initial = initial_coords_mm(case)
        guide_ids = guide_node_ids(case)
        lines.extend(
            [
                "##",
                "## Ghost target nodes for uniform soft guides; robot joint nodes are not hard-prescribed.",
                "/NODE",
            ]
        )
        for node_name, guide_id in guide_ids.items():
            x, y, z = initial[node_name]
            lines.append(f"{guide_id:10d}{x:20.8f}{y:20.8f}{z:20.8f}")
        spring_prop_id = 900001
        spring_part_id = 900001
        first_spring_id = max((item.beam_id for item in case.deck.members), default=0) + 100000
        spring_items = [
            (first_spring_id + idx, case.deck.node_ids[node_name], guide_ids[node_name])
            for idx, (node_name, _node_id) in enumerate(control_node_items)
        ]
        lines.extend(
            [
                "##",
                "## Uniform translational spring guides from each whole-body joint node to its ghost target.",
                *spring_gene_property_card(
                    spring_prop_id,
                    "uniform_joint_motion_guide_spring",
                    case.guide_stiffness,
                    case.guide_damping,
                ),
                *spring_part_card(spring_part_id, spring_prop_id, "uniform_joint_motion_guide_part"),
                *spring_element_card(spring_part_id, spring_items),
            ]
        )

    if case.control_policy != "stage1-torque-replay":
        guide_ids_for_motion = guide_node_ids(case) if case.control_policy == "uniform-joint-guides" else {}
        for node_idx, (node_name, node_id) in enumerate(control_node_items):
            group_id = group_base + node_idx
            motion_node_id = guide_ids_for_motion[node_name] if case.control_policy == "uniform-joint-guides" else node_id
            group_title = (
                f"guide_target_{node_name}" if case.control_policy == "uniform-joint-guides" else f"motion_node_{node_name}"
            )
            lines.extend(group_card(group_id, group_title, motion_node_id))
            displacement = case.target_displacements_mm[node_name]
            for dir_idx, direction in enumerate(directions):
                lines.extend(
                    function_card(
                        function_id,
                        f"{node_name}_{direction}_target_displacement",
                        times_ms,
                        displacement[:, dir_idx],
                    )
                )
                lines.extend(
                    impdisp_card(
                        impdisp_id,
                        f"{node_name}_{direction}_{case.control_policy}",
                        function_id,
                        direction,
                        group_id,
                        float(times_ms[-1]),
                    )
                )
                function_id += 1
                impdisp_id += 1

    node_ids = [node_id for _name, node_id in history_node_items]
    lines.extend(
        [
            "##",
            "## Time history for solved displacement and imposed-motion reactions",
            *th_node_card(2, TH_DISP_GROUP, "D", node_ids),
            *th_node_card(3, TH_REAC_GROUP, "REACX     REACY     REACZ", node_ids),
            "##",
            "## Beam section resultants for Radioss-native stress/strain post-processing",
            *th_beam_card(4, TH_BEAM_GROUP, "F1        M2        M3", case.deck.members),
            "/END",
            "",
        ]
    )
    case.deck.starter_path.write_text("\n".join([*starter, *lines]), encoding="utf-8")


def write_motion_engine(case: PeriodicMotionCase, anim_frames: int) -> None:
    stop_time_ms = case.samples[-1].time_ms
    anim_dt = stop_time_ms / max(anim_frames - 1, 1)
    tfile_dt = stop_time_ms / 200.0
    lines = [
        "#RADIOSS ENGINE",
        "/VERS/2026",
        f"/RUN/{case.deck.run_name}/1/",
        beam_deck.fmt(stop_time_ms),
        "/ANIM/DT",
        f"{beam_deck.fmt(0.0)} {beam_deck.fmt(anim_dt)} {beam_deck.fmt(stop_time_ms)}",
        "/ANIM/VECT/DISP",
        "/ANIM/VECT/VEL",
        "/ANIM/VECT/FREAC",
        "/H3D/NODA/VEL",
        "/H3D/DT",
        f"{beam_deck.fmt(0.0)} {beam_deck.fmt(anim_dt)}",
        "/TFILE/0",
        beam_deck.fmt(tfile_dt),
        "/PRINT/-100",
        "/MON/ON",
        "/DT/NODA/CST/0",
        "0.900000000000000    0.000000000000000",
        "",
    ]
    case.deck.engine_path.write_text("\n".join(lines), encoding="utf-8")


def motion_summary(case: PeriodicMotionCase) -> dict[str, Any]:
    max_disp = max(float(np.linalg.norm(values, axis=1).max()) for values in case.target_displacements_mm.values())
    is_torque_replay = case.control_policy == "stage1-torque-replay"
    torque_pairs = sorted(
        {(sample.joint, sample.proximal_node, sample.distal_node) for sample in case.torque_replay_samples}
    )
    return {
        "stage": "stage_2_openradioss_whole_body_periodic_motion",
        "run_name": case.deck.run_name,
        "source": "pygame_mass_viewer.py constrained periodic motion sampled onto the whole-body rod graph",
        "openradioss_keywords": {
            "elements": "/BEAM",
            "property": "/PROP/TYPE3 (BEAM)",
            "material": "/MAT/LAW1 (ELAST)",
            "lumped_mass": "/ADMAS/5",
            "load": "/CLOAD moment couples" if is_torque_replay else "/IMPDISP",
            "displacement_history": "/TH/NODE D",
            "reaction_history": "/TH/NODE REACX REACY REACZ",
            "beam_resultant_history": "/TH/BEAM F1 M2 M3",
        },
        "analysis_state": {
            "topology_only": False,
            "gravity_applied": False,
            "fixed_feet_applied": False,
            "contact_applied": False,
            "periodic_pose_sampled": True,
            "kinematic_periodic_motion_applied": not is_torque_replay,
            "stage1_torque_replay_applied": is_torque_replay,
            "solved_deformation_expected": True,
        },
        "counts": {
            "rod_graph_nodes": len(case.deck.rod_model.nodes),
            "solver_nodes": len(case.deck.node_ids),
            "beam_elements": len(case.deck.members),
            "beam_resultant_history_elements": len(case.deck.members),
            "imposed_displacement_functions": 0 if is_torque_replay else len(case.control_node_names) * 3,
            "uniform_guided_joint_nodes": len(case.control_node_names) if case.control_policy == "uniform-joint-guides" else 0,
            "hard_prescribed_robot_joint_nodes": len(case.control_node_names) if case.control_policy == "all-joints-hard" else 0,
            "stage1_torque_replay_samples": len(case.torque_replay_samples) if is_torque_replay else 0,
            "stage1_torque_replay_joints": len({sample.joint for sample in case.torque_replay_samples})
            if is_torque_replay
            else 0,
            "stage1_torque_replay_moment_couples": len(torque_pairs) if is_torque_replay else 0,
            "concentrated_moment_functions": torque_replay_component_count(case),
            "motion_samples": len(case.samples),
        },
        "timing": {
            "solver_duration_ms": float(case.samples[-1].time_ms),
            "viewer_start_seconds": float(case.viewer_start_seconds),
            "viewer_motion_seconds_sampled": float(case.samples[-1].viewer_time_s - case.viewer_start_seconds),
            "viewer_end_seconds": float(case.samples[-1].viewer_time_s),
        },
        "control_policy": case.control_policy,
        "whole_body_joint_node_names": list(case.control_node_names),
        "uniform_guided_joint_node_names": (
            list(case.control_node_names) if case.control_policy == "uniform-joint-guides" else []
        ),
        "guide_stiffness": float(case.guide_stiffness),
        "guide_damping": float(case.guide_damping),
        "torque_scale": float(case.torque_scale),
        "torque_replay_joint_node_pairs": [
            {"joint": joint, "proximal_node": proximal, "distal_node": distal}
            for joint, proximal, distal in torque_pairs
        ],
        "max_target_displacement_mm": max_disp,
        "target_element_length_mm": case.deck.target_element_length_mm,
        "uniform_radius_mm": case.deck.uniform_radius_mm,
        "beam_section_radius_policy": (
            f"uniform circular radius {case.deck.uniform_radius_mm:g} mm"
            if case.deck.uniform_radius_mm is not None
            else (
                "mass-derived / nominal radius"
                if case.deck.use_nominal_radius_for_massless_members
                else "mass-derived / minimum fallback radius"
            )
        ),
        "target_csv": case.target_csv_path.name,
        "torque_replay_csv": case.torque_replay_csv_path.name if is_torque_replay else None,
        "starter_file": case.deck.starter_path.name,
        "engine_file": case.deck.engine_path.name,
        "result_gif": case.result_gif_path.name,
        "result_poster": case.result_poster_path.name,
        "result_3d_gif": case.result_3d_gif_path.name,
        "result_3d_poster": case.result_3d_poster_path.name,
        "notes": [
            (
                "This is a whole-body Stage 1 torque-replay FEM case, not a single-link or subassembly case."
                if is_torque_replay
                else "This is a whole-body kinematic FEM case, not a single-link or subassembly case."
            ),
            "The FEM window samples a short slice of the second-scale periodic viewer motion.",
            "All original whole-body rod graph nodes remain uniform joint nodes in the FEM mesh.",
            (
                "Stage 1 torque rows are replayed as equal-and-opposite nodal moment couples."
                if is_torque_replay
                else "Each original whole-body joint receives the same ghost-target spring-guide treatment."
            ),
            (
                "No target displacement or ghost spring is applied in the torque replay load policy."
                if is_torque_replay
                else "The prescribed displacement is applied to ghost target nodes, not directly to robot joint nodes."
            ),
            (
                "OpenRadioss solves the connected beam mesh response to those Stage 1 moment couples."
                if is_torque_replay
                else "OpenRadioss solves the connected beam mesh response to those uniform soft guide loads."
            ),
            "No gravity, contact, or fixed-foot support reaction is present in this case.",
        ],
    }


def write_motion_summary(case: PeriodicMotionCase) -> None:
    with case.deck.summary_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(motion_summary(case), handle, sort_keys=False)


def build_periodic_motion_case(
    description: Any,
    catalog: stage1.PhysicalCatalog,
    out_dir: Path,
    run_name: str,
    sample_count: int,
    solver_duration_ms: float,
    viewer_start_seconds: float,
    viewer_motion_seconds: float,
    babble_scale: float,
    motion_scale: float,
    target_element_length_mm: float,
    use_nominal_radius_for_massless_members: bool,
    uniform_radius_mm: float | None,
    case_name: str,
    control_policy: str = "stage1-torque-replay",
    guide_stiffness: float = 0.004,
    guide_damping: float = 0.00004,
    torque_scale: float = 1.0,
) -> PeriodicMotionCase:
    samples = build_motion_samples(
        description,
        catalog,
        sample_count=sample_count,
        solver_duration_ms=solver_duration_ms,
        viewer_start_seconds=viewer_start_seconds,
        viewer_motion_seconds=viewer_motion_seconds,
        babble_scale=babble_scale,
        case_name=case_name,
    )
    deck = beam_deck.build_beam_deck(
        samples[0].rod_model,
        out_dir,
        run_name,
        target_element_length_mm=target_element_length_mm,
        use_nominal_radius_for_massless_members=use_nominal_radius_for_massless_members,
        uniform_radius_mm=uniform_radius_mm,
    )
    deck = beam_deck.BeamDeck(
        run_name=deck.run_name,
        rod_model=deck.rod_model,
        node_ids=deck.node_ids,
        members=deck.members,
        admas_node_masses_kg=deck.admas_node_masses_kg,
        starter_path=deck.starter_path,
        engine_path=deck.engine_path,
        summary_path=out_dir / "openradioss_periodic_motion_summary.yaml",
        poster_path=out_dir / "openradioss_whole_body_periodic_motion_targets_poster.png",
        gif_path=out_dir / "openradioss_whole_body_periodic_motion_targets.gif",
        node_xyz_m=deck.node_xyz_m,
        node_interpolations=deck.node_interpolations,
        target_element_length_mm=deck.target_element_length_mm,
        use_nominal_radius_for_massless_members=deck.use_nominal_radius_for_massless_members,
        uniform_radius_mm=deck.uniform_radius_mm,
    )
    control_node_names = select_control_node_names(samples[0].rod_model, control_policy)
    return PeriodicMotionCase(
        deck=deck,
        samples=samples,
        target_displacements_mm=target_displacements(deck, samples, motion_scale),
        control_node_names=control_node_names,
        control_policy=control_policy,
        guide_stiffness=guide_stiffness,
        guide_damping=guide_damping,
        uniform_radius_mm=uniform_radius_mm,
        torque_replay_samples=build_torque_replay_samples(samples, catalog, torque_scale),
        torque_scale=torque_scale,
        torque_replay_csv_path=out_dir / "stage1_torque_replay_loads.csv",
        viewer_start_seconds=viewer_start_seconds,
        target_csv_path=out_dir / "periodic_motion_targets.csv",
        result_gif_path=out_dir / "openradioss_whole_body_periodic_motion.gif",
        result_poster_path=out_dir / "openradioss_whole_body_periodic_motion_poster.png",
        result_3d_gif_path=out_dir / "openradioss_whole_body_periodic_motion_3d.gif",
        result_3d_poster_path=out_dir / "openradioss_whole_body_periodic_motion_3d_poster.png",
    )


def write_case(case: PeriodicMotionCase, make_preview_gif: bool, preview_frames: int, preview_duration_ms: int) -> None:
    beam_deck.write_starter(case.deck)
    append_motion_cards(case)
    write_motion_engine(case, anim_frames=max(2, preview_frames))
    write_motion_targets_csv(case)
    if case.control_policy == "stage1-torque-replay":
        write_torque_replay_csv(case)
    write_motion_summary(case)
    if make_preview_gif:
        make_target_preview(case, frames=preview_frames, duration_ms=preview_duration_ms)


def read_t01_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
        headers = list(reader.fieldnames or [])
    if not rows:
        raise ValueError(f"no rows in {path}")
    return headers, rows


def numeric_column(rows: list[dict[str, str]], name: str, default: float = 0.0) -> np.ndarray:
    if name not in rows[0]:
        return np.full(len(rows), default, dtype=float)
    return np.array([float(row[name]) if row[name] else default for row in rows], dtype=float)


def var_number(header: str) -> int:
    match = re.search(r"var\s+(\d+)", header)
    return int(match.group(1)) if match else -1


def columns_for_node(headers: list[str], group_name: str, node_id: int, expected_count: int) -> list[str]:
    needle = re.compile(rf"{re.escape(group_name)}\s+{node_id}\s+.*var\s+\d+")
    cols = [header for header in headers if needle.search(header)]
    cols.sort(key=var_number)
    if len(cols) < expected_count:
        raise ValueError(f"expected {expected_count} {group_name} columns for node {node_id}, found {len(cols)}")
    return cols[:expected_count]


def parse_displacement_history(
    t01_csv: Path,
    node_ids: dict[str, int],
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray], dict[str, np.ndarray]]:
    headers, rows = read_t01_csv(t01_csv)
    global_history = {
        "time": numeric_column(rows, "time"),
        **{name: numeric_column(rows, name) for name in GLOBAL_HISTORY_COLUMNS},
    }
    displacements: dict[str, np.ndarray] = {}
    reactions: dict[str, np.ndarray] = {}
    for node_name, node_id in node_ids.items():
        disp_cols = columns_for_node(headers, TH_DISP_GROUP, node_id, 3)
        reac_cols = columns_for_node(headers, TH_REAC_GROUP, node_id, 3)
        displacements[node_name] = np.array([[float(row[col]) for col in disp_cols] for row in rows], dtype=float)
        reactions[node_name] = np.array([[float(row[col]) for col in reac_cols] for row in rows], dtype=float)
    return global_history, displacements, reactions


def parse_beam_resultant_history(
    t01_csv: Path,
    case: PeriodicMotionCase,
) -> dict[str, dict[str, np.ndarray]]:
    headers, rows = read_t01_csv(t01_csv)
    resultants: dict[str, dict[str, np.ndarray]] = {}
    for item in case.deck.members:
        try:
            cols = columns_for_node(headers, TH_BEAM_GROUP, item.beam_id, 3)
        except ValueError:
            return {}
        resultants[beam_element_key(item)] = {
            "F1": np.array([float(row[cols[0]]) for row in rows], dtype=float),
            "M2": np.array([float(row[cols[1]]) for row in rows], dtype=float),
            "M3": np.array([float(row[cols[2]]) for row in rows], dtype=float),
        }
    return resultants


def beam_resultant_stress_strain_history(
    case: PeriodicMotionCase,
    resultants: dict[str, dict[str, np.ndarray]],
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    stresses_mpa: dict[str, np.ndarray] = {}
    strains: dict[str, np.ndarray] = {}
    for item in case.deck.members:
        key = beam_element_key(item)
        if key not in resultants:
            continue
        values = resultants[key]
        radius_mm = element_radius_mm(item)
        axial_model = values["F1"] / max(item.area_mm2, 1.0e-12)
        bending_model = radius_mm * np.sqrt(
            (values["M2"] / max(item.iyy_mm4, 1.0e-12)) ** 2
            + (values["M3"] / max(item.izz_mm4, 1.0e-12)) ** 2
        )
        tensile_model = axial_model + bending_model
        compression_model = axial_model - bending_model
        governing_model = np.where(np.abs(tensile_model) >= np.abs(compression_model), tensile_model, compression_model)
        stress_mpa = governing_model * RADIOSS_FORCE_UNIT_TO_N
        stresses_mpa[key] = stress_mpa
        strains[key] = stress_mpa / YOUNGS_MPA
    return stresses_mpa, strains


def load_beam_resultant_strains(t01_csv: Path, case: PeriodicMotionCase) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    resultants = parse_beam_resultant_history(t01_csv, case)
    if not resultants:
        return {}, {}
    return beam_resultant_stress_strain_history(case, resultants)


def initial_coords_mm(case: PeriodicMotionCase) -> dict[str, np.ndarray]:
    return {name: np.array(xyz_m, dtype=float) * beam_deck.RADIOSS_LENGTH_SCALE for name, xyz_m in case.deck.node_xyz_m.items()}


def beam_element_key(item: beam_deck.BeamDeckMember) -> str:
    return f"{item.member.name}_elem_{item.beam_id:04d}"


def element_radius_mm(item: beam_deck.BeamDeckMember) -> float:
    return math.sqrt(max(item.area_mm2, 1.0e-12) / math.pi)


def absolute_seismic_cmap() -> LinearSegmentedColormap:
    colors = plt.get_cmap("seismic")(np.linspace(0.5, 1.0, 256))
    return LinearSegmentedColormap.from_list("absolute_seismic_tension", colors)


def safe_angle_between(a: np.ndarray, b: np.ndarray) -> float | None:
    norm_a = float(np.linalg.norm(a))
    norm_b = float(np.linalg.norm(b))
    if norm_a <= 1.0e-9 or norm_b <= 1.0e-9:
        return None
    dot = float(np.dot(a, b) / (norm_a * norm_b))
    return math.acos(max(-1.0, min(1.0, dot)))


def other_element_node(item: beam_deck.BeamDeckMember, shared_node: str) -> str:
    if item.node_a_name == shared_node:
        return item.node_b_name
    if item.node_b_name == shared_node:
        return item.node_a_name
    raise ValueError(f"{shared_node} is not attached to {beam_element_key(item)}")


def member_bending_strain_proxies(
    case: PeriodicMotionCase,
    coords_mm: dict[str, np.ndarray],
) -> dict[str, float]:
    initial = initial_coords_mm(case)
    by_node: dict[str, list[beam_deck.BeamDeckMember]] = {}
    for item in case.deck.members:
        by_node.setdefault(item.node_a_name, []).append(item)
        by_node.setdefault(item.node_b_name, []).append(item)

    bending = {beam_element_key(item): 0.0 for item in case.deck.members}
    for node_name, attached in by_node.items():
        if len(attached) < 2:
            continue
        for first, second in itertools.combinations(attached, 2):
            first_other = other_element_node(first, node_name)
            second_other = other_element_node(second, node_name)
            initial_angle = safe_angle_between(
                initial[first_other] - initial[node_name],
                initial[second_other] - initial[node_name],
            )
            current_angle = safe_angle_between(
                coords_mm[first_other] - coords_mm[node_name],
                coords_mm[second_other] - coords_mm[node_name],
            )
            if initial_angle is None or current_angle is None:
                continue
            delta_angle = abs(current_angle - initial_angle)
            average_length = max(0.5 * (first.length_mm + second.length_mm), 1.0e-9)
            for item in (first, second):
                key = beam_element_key(item)
                strain = element_radius_mm(item) * delta_angle / average_length
                bending[key] = max(bending[key], strain)
    return bending


def member_strains(
    case: PeriodicMotionCase,
    coords_mm: dict[str, np.ndarray],
) -> dict[str, float]:
    initial = initial_coords_mm(case)
    bending = member_bending_strain_proxies(case, coords_mm)
    strains: dict[str, float] = {}
    for item in case.deck.members:
        a0 = initial[item.node_a_name]
        b0 = initial[item.node_b_name]
        a = coords_mm[item.node_a_name]
        b = coords_mm[item.node_b_name]
        l0 = max(float(np.linalg.norm(b0 - a0)), 1.0e-9)
        axial = (float(np.linalg.norm(b - a)) - l0) / l0
        bend = bending[beam_element_key(item)]
        tensile_fiber = axial + bend
        compression_fiber = axial - bend
        strains[beam_element_key(item)] = (
            tensile_fiber if abs(tensile_fiber) >= abs(compression_fiber) else compression_fiber
        )
    return strains


def member_area_by_name(case: PeriodicMotionCase) -> dict[str, float]:
    return {beam_element_key(item): item.area_mm2 for item in case.deck.members}


def member_axial_forces_n(case: PeriodicMotionCase, strains: dict[str, float]) -> dict[str, float]:
    areas = member_area_by_name(case)
    youngs_n_per_mm2 = 3200.0
    return {name: youngs_n_per_mm2 * areas[name] * strain for name, strain in strains.items()}


def current_coords_mm(
    case: PeriodicMotionCase,
    displacements: dict[str, np.ndarray],
    frame_idx: int,
) -> dict[str, np.ndarray]:
    initial = initial_coords_mm(case)
    return {name: xyz + displacements[name][frame_idx] for name, xyz in initial.items()}


def result_metrics(
    case: PeriodicMotionCase,
    displacements: dict[str, np.ndarray],
    reactions: dict[str, np.ndarray],
    element_strains: dict[str, np.ndarray] | None = None,
) -> dict[str, np.ndarray]:
    frame_count = len(next(iter(displacements.values())))
    max_disp = np.zeros(frame_count)
    max_tension_strain = np.zeros(frame_count)
    max_compression_strain = np.zeros(frame_count)
    max_abs_strain = np.zeros(frame_count)
    max_tension_force_n = np.zeros(frame_count)
    max_compression_force_n = np.zeros(frame_count)
    reaction_sum = np.zeros(frame_count)

    for idx in range(frame_count):
        if element_strains:
            strains = {name: values[idx] for name, values in element_strains.items()}
        else:
            coords = current_coords_mm(case, displacements, idx)
            strains = member_strains(case, coords)
        forces = member_axial_forces_n(case, strains)
        max_disp[idx] = max(float(np.linalg.norm(values[idx])) for values in displacements.values())
        max_tension_strain[idx] = max(max(strains.values()), 0.0)
        max_compression_strain[idx] = min(min(strains.values()), 0.0)
        max_abs_strain[idx] = max(abs(value) for value in strains.values())
        max_tension_force_n[idx] = max(max(forces.values()), 0.0)
        max_compression_force_n[idx] = min(min(forces.values()), 0.0)
        reaction_sum[idx] = sum(float(np.linalg.norm(values[idx])) for values in reactions.values())

    return {
        "max_disp": max_disp,
        "max_tension_strain": max_tension_strain,
        "max_compression_strain": max_compression_strain,
        "max_abs_strain": max_abs_strain,
        "max_tension_force_n": max_tension_force_n,
        "max_compression_force_n": max_compression_force_n,
        "reaction_sum": reaction_sum,
    }


def set_equal_3d(ax: Any, coords: np.ndarray) -> None:
    mins = coords.min(axis=0)
    maxs = coords.max(axis=0)
    center = 0.5 * (mins + maxs)
    span = max(float((maxs - mins).max()), 1.0)
    radius = 0.58 * span
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(max(0.0, center[2] - radius * 0.72), center[2] + radius * 0.84)
    try:
        ax.set_box_aspect((1, 1, 0.60), zoom=1.20)
    except TypeError:
        ax.set_box_aspect((1, 1, 0.60))


def draw_deformed_mesh(ax: Any, case: PeriodicMotionCase, displacements: dict[str, np.ndarray], frame_idx: int) -> float:
    initial = initial_coords_mm(case)
    coords = {name: xyz + displacements[name][frame_idx] for name, xyz in initial.items()}
    strains = member_strains(case, coords)
    max_abs_strain = max(abs(value) for value in strains.values())
    norm_limit = max(max_abs_strain, 1.0e-6)
    cmap = plt.get_cmap("seismic")

    for member in case.deck.rod_model.members:
        a0 = initial[member.node_a]
        b0 = initial[member.node_b]
        ax.plot([a0[0], b0[0]], [a0[1], b0[1]], [a0[2], b0[2]], color="#c7c9cc", lw=1.0, alpha=0.25)

    for member in case.deck.rod_model.members:
        a = coords[member.node_a]
        b = coords[member.node_b]
        color = cmap(0.5 + 0.5 * strains[member.name] / norm_limit)
        ax.plot([a[0], b[0]], [a[1], b[1]], [a[2], b[2]], color=color, lw=3.2, alpha=0.95)

    all_coords = np.array([coords[node.name] for node in case.deck.rod_model.nodes], dtype=float)
    ax.scatter(all_coords[:, 0], all_coords[:, 1], all_coords[:, 2], s=20, c="#f8fafc", edgecolors="#111827", lw=0.6)
    contact = np.array([coords[node.name] for node in case.deck.rod_model.nodes if node.contact_candidate], dtype=float)
    if len(contact):
        ax.scatter(
            contact[:, 0],
            contact[:, 1],
            contact[:, 2],
            s=78,
            marker="v",
            c="none",
            edgecolors="#f97316",
            linewidths=1.3,
        )

    ax.view_init(elev=27.0, azim=-42.0)
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")
    ax.set_zlabel("z [mm]")
    ax.grid(color="#d9d9d9", linewidth=0.45, alpha=0.35)
    set_equal_3d(ax, np.vstack([np.array(list(initial.values())), all_coords]))
    ax.text2D(0.02, 0.92, "kinematic periodic motion", transform=ax.transAxes, color="#0f766e", fontsize=10, weight="bold")
    ax.text2D(0.02, 0.86, "no gravity / no contact", transform=ax.transAxes, color="#b91c1c", fontsize=9.5, weight="bold")
    ax.text2D(
        0.02,
        0.80,
        "feet follow prescribed motion; they are not fixed supports",
        transform=ax.transAxes,
        color="#b91c1c",
        fontsize=9.5,
        weight="bold",
    )
    ax.text2D(
        0.99,
        0.02,
        "gray=initial rods, color=solved axial strain proxy from T01 displacement",
        transform=ax.transAxes,
        ha="right",
        va="bottom",
        fontsize=8.0,
        color="#555555",
    )
    return max_abs_strain


def project_point(xyz: np.ndarray, view: str) -> tuple[float, float]:
    if view == "side":
        return float(xyz[0]), float(xyz[2])
    if view == "top":
        return float(xyz[0]), float(xyz[1])
    raise ValueError(f"unknown projection view: {view}")


def set_equal_2d(ax: Any, points: np.ndarray, view: str) -> None:
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    center = 0.5 * (mins + maxs)
    span = max(float((maxs - mins).max()), 1.0)
    radius = 0.56 * span
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("z [mm]" if view == "side" else "y [mm]")
    ax.grid(color="#d9d9d9", linewidth=0.5, alpha=0.55)


def projected_cylinder_patches(
    start: tuple[float, float],
    end: tuple[float, float],
    radius_mm: float,
) -> tuple[list[Any], list[np.ndarray]]:
    p0 = np.array(start, dtype=float)
    p1 = np.array(end, dtype=float)
    segment = p1 - p0
    length = float(np.linalg.norm(segment))
    radius = max(float(radius_mm), 0.25)
    if length <= 1.0e-9:
        return [Circle(p0, radius)], []

    normal = np.array([-segment[1], segment[0]], dtype=float) / length
    corners = np.vstack([p0 + normal * radius, p1 + normal * radius, p1 - normal * radius, p0 - normal * radius])
    patches = [
        Polygon(corners, closed=True, joinstyle="miter"),
        Circle(p0, radius),
        Circle(p1, radius),
    ]
    seams = [
        np.vstack([p0 - normal * radius, p0 + normal * radius]),
        np.vstack([p1 - normal * radius, p1 + normal * radius]),
    ]
    return patches, seams


def add_projected_cylinder_collection(
    ax: Any,
    cylinders: list[tuple[tuple[float, float], tuple[float, float], float]],
    facecolors: list[Any],
    edgecolor: str,
    linewidth: float,
    alpha: float,
    zorder: int,
    mesh_lines: bool,
) -> None:
    patches: list[Any] = []
    patch_colors: list[Any] = []
    seam_segments: list[np.ndarray] = []
    for (start, end, radius_mm), facecolor in zip(cylinders, facecolors):
        cylinder_patches, seams = projected_cylinder_patches(start, end, radius_mm)
        patches.extend(cylinder_patches)
        patch_colors.extend([facecolor] * len(cylinder_patches))
        if mesh_lines:
            seam_segments.extend(seams)
    if patches:
        ax.add_collection(
            PatchCollection(
                patches,
                match_original=False,
                facecolors=patch_colors,
                edgecolors=edgecolor,
                linewidths=linewidth,
                alpha=alpha,
                zorder=zorder,
            )
        )
    if mesh_lines and seam_segments:
        ax.add_collection(
            LineCollection(
                seam_segments,
                colors=edgecolor,
                linewidths=max(0.28, linewidth * 0.85),
                alpha=min(1.0, alpha + 0.08),
                zorder=zorder + 0.04,
            )
        )


def cylinder_frame_3d(start: np.ndarray, end: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    axis = end - start
    length = float(np.linalg.norm(axis))
    if length <= 1.0e-9:
        return None
    axis = axis / length
    reference = np.array([0.0, 0.0, 1.0], dtype=float)
    if abs(float(np.dot(axis, reference))) > 0.90:
        reference = np.array([0.0, 1.0, 0.0], dtype=float)
    normal_a = np.cross(axis, reference)
    normal_a_norm = float(np.linalg.norm(normal_a))
    if normal_a_norm <= 1.0e-9:
        return None
    normal_a = normal_a / normal_a_norm
    normal_b = np.cross(axis, normal_a)
    return axis, normal_a, normal_b


def cylinder_mesh_3d(
    start: np.ndarray,
    end: np.ndarray,
    radius_mm: float,
    sides: int = 10,
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    frame = cylinder_frame_3d(start, end)
    radius = max(float(radius_mm), 0.25)
    if frame is None:
        return [], []
    _axis, normal_a, normal_b = frame
    angles = np.linspace(0.0, 2.0 * math.pi, max(6, sides), endpoint=False)
    ring_start = np.array([start + radius * (math.cos(angle) * normal_a + math.sin(angle) * normal_b) for angle in angles])
    ring_end = np.array([end + radius * (math.cos(angle) * normal_a + math.sin(angle) * normal_b) for angle in angles])

    faces: list[np.ndarray] = []
    seams: list[np.ndarray] = []
    side_count = len(angles)
    for idx in range(side_count):
        nxt = (idx + 1) % side_count
        faces.append(np.vstack([ring_start[idx], ring_start[nxt], ring_end[nxt], ring_end[idx]]))
        seams.append(np.vstack([ring_start[idx], ring_start[nxt]]))
        seams.append(np.vstack([ring_end[idx], ring_end[nxt]]))
    faces.append(ring_start[::-1])
    faces.append(ring_end)
    return faces, seams


def add_cylinder_mesh_3d_collection(
    ax: Any,
    cylinders: list[tuple[np.ndarray, np.ndarray, float]],
    facecolors: list[Any],
    edgecolor: str,
    linewidth: float,
    alpha: float,
    zorder: int,
    sides: int = 10,
) -> None:
    faces: list[np.ndarray] = []
    colors: list[Any] = []
    seams: list[np.ndarray] = []
    for (start, end, radius_mm), color in zip(cylinders, facecolors):
        item_faces, item_seams = cylinder_mesh_3d(start, end, radius_mm, sides=sides)
        faces.extend(item_faces)
        colors.extend([color] * len(item_faces))
        seams.extend(item_seams)
    if faces:
        poly = Poly3DCollection(
            faces,
            facecolors=colors,
            edgecolors=edgecolor,
            linewidths=linewidth,
            alpha=alpha,
            zorder=zorder,
        )
        poly.set_zsort("average")
        ax.add_collection3d(poly)
    if seams:
        ax.add_collection3d(
            Line3DCollection(
                seams,
                colors=edgecolor,
                linewidths=max(0.18, linewidth * 0.72),
                alpha=min(1.0, alpha + 0.08),
                zorder=zorder + 1,
            )
        )


def frame_element_strains(
    case: PeriodicMotionCase,
    coords_mm: dict[str, np.ndarray],
    frame_idx: int,
    element_strains: dict[str, np.ndarray] | None = None,
) -> dict[str, float]:
    if element_strains:
        return {name: float(values[frame_idx]) for name, values in element_strains.items()}
    return member_strains(case, coords_mm)


def draw_tension_3d(
    ax: Any,
    case: PeriodicMotionCase,
    displacements: dict[str, np.ndarray],
    frame_idx: int,
    strain_norm: float,
    element_strains: dict[str, np.ndarray] | None = None,
) -> tuple[str, float]:
    initial = initial_coords_mm(case)
    coords = current_coords_mm(case, displacements, frame_idx)
    strains = frame_element_strains(case, coords, frame_idx, element_strains)
    hottest_name = max(strains, key=lambda name: abs(strains[name]))
    hottest_strain = strains[hottest_name]
    cmap = absolute_seismic_cmap()
    norm = Normalize(vmin=0.0, vmax=strain_norm)

    cylinders: list[tuple[np.ndarray, np.ndarray, float]] = []
    colors: list[Any] = []
    for item in case.deck.members:
        cylinders.append((coords[item.node_a_name], coords[item.node_b_name], element_radius_mm(item)))
        colors.append(cmap(norm(abs(strains[beam_element_key(item)]))))
    add_cylinder_mesh_3d_collection(
        ax,
        cylinders,
        colors,
        edgecolor="#172033",
        linewidth=0.18,
        alpha=0.96,
        zorder=4,
        sides=10,
    )

    solver_points = np.array([coords[name] for name in case.deck.node_ids], dtype=float)
    ax.scatter(
        solver_points[:, 0],
        solver_points[:, 1],
        solver_points[:, 2],
        s=4.0,
        c="#111827",
        alpha=0.42,
        depthshade=False,
        zorder=6,
    )
    joint_points = np.array([coords[node.name] for node in case.deck.rod_model.nodes], dtype=float)
    ax.scatter(
        joint_points[:, 0],
        joint_points[:, 1],
        joint_points[:, 2],
        s=24,
        c="#f8fafc",
        edgecolors="#111827",
        linewidths=0.65,
        alpha=0.96,
        depthshade=False,
        zorder=7,
    )
    contact_points = np.array([coords[node.name] for node in case.deck.rod_model.nodes if node.contact_candidate], dtype=float)
    if len(contact_points):
        ax.scatter(
            contact_points[:, 0],
            contact_points[:, 1],
            contact_points[:, 2],
            s=64,
            marker="v",
            c="none",
            edgecolors="#f97316",
            linewidths=1.25,
            depthshade=False,
            zorder=8,
        )

    all_coords = np.vstack([np.array(list(initial.values()), dtype=float), solver_points])
    set_equal_3d(ax, all_coords)
    ax.view_init(elev=28.0, azim=-42.0)
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("y [mm]")
    ax.set_zlabel("z [mm]")
    ax.grid(color="#d9d9d9", linewidth=0.45, alpha=0.35)
    try:
        ax.xaxis.pane.set_facecolor((0.98, 0.98, 0.96, 0.22))
        ax.yaxis.pane.set_facecolor((0.98, 0.98, 0.96, 0.22))
        ax.zaxis.pane.set_facecolor((0.98, 0.98, 0.96, 0.22))
    except AttributeError:
        pass
    return hottest_name, hottest_strain


def draw_tension_projection(
    ax: Any,
    case: PeriodicMotionCase,
    displacements: dict[str, np.ndarray],
    frame_idx: int,
    view: str,
    strain_norm: float,
    element_strains: dict[str, np.ndarray] | None = None,
) -> tuple[str, float]:
    initial = initial_coords_mm(case)
    coords = current_coords_mm(case, displacements, frame_idx)
    strains = frame_element_strains(case, coords, frame_idx, element_strains)
    hottest_name = max(strains, key=lambda name: abs(strains[name]))
    hottest_strain = strains[hottest_name]
    cmap = absolute_seismic_cmap()
    norm = Normalize(vmin=0.0, vmax=strain_norm)

    initial_cylinders = [
        (
            project_point(initial[item.node_a_name], view),
            project_point(initial[item.node_b_name], view),
            element_radius_mm(item),
        )
        for item in case.deck.members
    ]
    add_projected_cylinder_collection(
        ax,
        initial_cylinders,
        ["#d7dce3"] * len(initial_cylinders),
        edgecolor="#aeb7c2",
        linewidth=0.36,
        alpha=0.20,
        zorder=1,
        mesh_lines=False,
    )

    deformed_cylinders: list[tuple[tuple[float, float], tuple[float, float], float]] = []
    deformed_colors: list[Any] = []
    for item in case.deck.members:
        a = project_point(coords[item.node_a_name], view)
        b = project_point(coords[item.node_b_name], view)
        strain = strains[beam_element_key(item)]
        deformed_cylinders.append((a, b, element_radius_mm(item)))
        deformed_colors.append(cmap(norm(abs(strain))))
    add_projected_cylinder_collection(
        ax,
        deformed_cylinders,
        deformed_colors,
        edgecolor="#1f2937",
        linewidth=0.52,
        alpha=0.96,
        zorder=4,
        mesh_lines=True,
    )

    node_points = np.array([project_point(coords[name], view) for name in case.deck.node_ids], dtype=float)
    ax.scatter(node_points[:, 0], node_points[:, 1], s=5.2, c="#111827", alpha=0.55, zorder=5)
    joint_points = np.array(
        [project_point(coords[node.name], view) for node in case.deck.rod_model.nodes],
        dtype=float,
    )
    ax.scatter(
        joint_points[:, 0],
        joint_points[:, 1],
        s=18,
        c="#f8fafc",
        edgecolors="#111827",
        linewidths=0.55,
        alpha=0.95,
        zorder=6,
    )
    contact_points = np.array(
        [project_point(coords[node.name], view) for node in case.deck.rod_model.nodes if node.contact_candidate],
        dtype=float,
    )
    if len(contact_points):
        ax.scatter(
            contact_points[:, 0],
            contact_points[:, 1],
            s=58,
            marker="v",
            c="none",
            edgecolors="#f97316",
            linewidths=1.2,
            zorder=7,
        )

    all_points = np.vstack(
        [
            np.array([project_point(xyz, view) for xyz in initial.values()], dtype=float),
            node_points,
        ]
    )
    set_equal_2d(ax, all_points, view)
    ax.set_title("side tension/bend map x-z" if view == "side" else "top tension/bend map x-y", fontsize=11)
    return hottest_name, hottest_strain


def add_seismic_strain_colorbar(fig: Any, strain_norm: float, rect: tuple[float, float, float, float]) -> None:
    cmap = absolute_seismic_cmap()
    norm = Normalize(vmin=0.0, vmax=strain_norm)
    cax = fig.add_axes(rect)
    colorbar = fig.colorbar(ScalarMappable(norm=norm, cmap=cmap), cax=cax, orientation="horizontal")
    colorbar.set_label("absolute outer-fiber strain | white=0, red=tension/load hot", fontsize=8.5)
    colorbar.set_ticks([0.0, strain_norm])
    colorbar.ax.set_xticklabels(["0", f"{strain_norm:.1e}"])
    colorbar.ax.tick_params(labelsize=7.5, pad=1)


def render_result_frame(
    case: PeriodicMotionCase,
    global_history: dict[str, np.ndarray],
    displacements: dict[str, np.ndarray],
    reactions: dict[str, np.ndarray],
    metrics: dict[str, np.ndarray],
    frame_idx: int,
    strain_norm: float,
    element_strains: dict[str, np.ndarray] | None = None,
    strain_source: str = "displacement proxy",
) -> Image.Image:
    time = global_history["time"]
    max_disp = metrics["max_disp"]

    fig = plt.figure(figsize=(12.8, 7.2), dpi=110)
    fig.patch.set_facecolor("#fbfbf8")
    gs = fig.add_gridspec(1, 2, wspace=0.18, left=0.06, right=0.98, top=0.87, bottom=0.10)
    ax_side = fig.add_subplot(gs[0, 0])
    ax_top = fig.add_subplot(gs[0, 1])

    hot_side, hot_side_strain = draw_tension_projection(
        ax_side, case, displacements, frame_idx, "side", strain_norm, element_strains
    )
    hot_top, hot_top_strain = draw_tension_projection(
        ax_top, case, displacements, frame_idx, "top", strain_norm, element_strains
    )
    _hot_name, hot_strain = (
        (hot_side, hot_side_strain) if abs(hot_side_strain) >= abs(hot_top_strain) else (hot_top, hot_top_strain)
    )

    fig.suptitle(
        "OpenRadioss whole-body articulated bend | "
        f"t={time[frame_idx]:.3f} ms | max disp={max_disp[frame_idx]:.2f} mm | "
        f"max |fiber strain|={abs(hot_strain):.3e} | {strain_source}",
        fontsize=12.5,
        y=0.965,
    )
    add_seismic_strain_colorbar(fig, strain_norm, (0.34, 0.035, 0.32, 0.018))
    buffer = io.BytesIO()
    fig.savefig(buffer, format="png", facecolor=fig.get_facecolor(), bbox_inches="tight", pad_inches=0.12)
    plt.close(fig)
    buffer.seek(0)
    return Image.open(buffer).convert("RGB")


def make_result_gif(case: PeriodicMotionCase, t01_csv: Path, frames: int, duration_ms: int) -> None:
    global_history, displacements, reactions = parse_displacement_history(t01_csv, case.deck.node_ids)
    _beam_stresses_mpa, element_strains = load_beam_resultant_strains(t01_csv, case)
    strain_source = "Radioss beam F/M outer-fiber strain" if element_strains else "displacement-derived strain proxy"
    metrics = result_metrics(case, displacements, reactions, element_strains or None)
    strain_norm = max(float(np.nanmax(metrics["max_abs_strain"])), 1.0e-6)
    sample_idx = np.linspace(0, len(global_history["time"]) - 1, max(1, frames)).round().astype(int)
    images = [
        render_result_frame(
            case,
            global_history,
            displacements,
            reactions,
            metrics,
            int(idx),
            strain_norm,
            element_strains or None,
            strain_source,
        )
        for idx in sample_idx
    ]
    poster_idx = int(np.argmax(metrics["max_abs_strain"]))
    poster = render_result_frame(
        case,
        global_history,
        displacements,
        reactions,
        metrics,
        poster_idx,
        strain_norm,
        element_strains or None,
        strain_source,
    )
    case.result_gif_path.parent.mkdir(parents=True, exist_ok=True)
    poster.save(case.result_poster_path)
    images[0].save(case.result_gif_path, save_all=True, append_images=images[1:], duration=max(20, duration_ms), loop=0)


def render_result_3d_frame(
    case: PeriodicMotionCase,
    global_history: dict[str, np.ndarray],
    displacements: dict[str, np.ndarray],
    reactions: dict[str, np.ndarray],
    metrics: dict[str, np.ndarray],
    frame_idx: int,
    strain_norm: float,
    element_strains: dict[str, np.ndarray] | None = None,
    strain_source: str = "displacement proxy",
) -> Image.Image:
    _ = reactions
    time = global_history["time"]
    max_disp = metrics["max_disp"]

    fig = plt.figure(figsize=(12.8, 7.2), dpi=110)
    fig.patch.set_facecolor("#fbfbf8")
    ax = fig.add_subplot(111, projection="3d")
    _hot_name, hot_strain = draw_tension_3d(ax, case, displacements, frame_idx, strain_norm, element_strains)
    fig.suptitle(
        "OpenRadioss whole-body 3D FEM cylinders | "
        f"t={time[frame_idx]:.3f} ms | max disp={max_disp[frame_idx]:.2f} mm | "
        f"max |fiber strain|={abs(hot_strain):.3e} | {strain_source}",
        fontsize=12.5,
        y=0.965,
    )
    fig.subplots_adjust(left=0.02, right=0.98, top=0.92, bottom=0.14)
    add_seismic_strain_colorbar(fig, strain_norm, (0.34, 0.045, 0.32, 0.018))
    buffer = io.BytesIO()
    fig.savefig(buffer, format="png", facecolor=fig.get_facecolor(), bbox_inches="tight", pad_inches=0.10)
    plt.close(fig)
    buffer.seek(0)
    return Image.open(buffer).convert("RGB")


def make_result_3d_gif(case: PeriodicMotionCase, t01_csv: Path, frames: int, duration_ms: int) -> None:
    global_history, displacements, reactions = parse_displacement_history(t01_csv, case.deck.node_ids)
    _beam_stresses_mpa, element_strains = load_beam_resultant_strains(t01_csv, case)
    strain_source = "Radioss beam F/M outer-fiber strain" if element_strains else "displacement-derived strain proxy"
    metrics = result_metrics(case, displacements, reactions, element_strains or None)
    strain_norm = max(float(np.nanmax(metrics["max_abs_strain"])), 1.0e-6)
    sample_idx = np.linspace(0, len(global_history["time"]) - 1, max(1, frames)).round().astype(int)
    images = [
        render_result_3d_frame(
            case,
            global_history,
            displacements,
            reactions,
            metrics,
            int(idx),
            strain_norm,
            element_strains or None,
            strain_source,
        )
        for idx in sample_idx
    ]
    poster_idx = int(np.argmax(metrics["max_abs_strain"]))
    poster = render_result_3d_frame(
        case,
        global_history,
        displacements,
        reactions,
        metrics,
        poster_idx,
        strain_norm,
        element_strains or None,
        strain_source,
    )
    case.result_3d_gif_path.parent.mkdir(parents=True, exist_ok=True)
    poster.save(case.result_3d_poster_path)
    images[0].save(case.result_3d_gif_path, save_all=True, append_images=images[1:], duration=max(20, duration_ms), loop=0)


def render_target_preview_frame(case: PeriodicMotionCase, sample_idx: int) -> Image.Image:
    displacements = {
        node_name: np.vstack([values[sample_idx] for _ in range(1)])
        for node_name, values in case.target_displacements_mm.items()
    }
    reactions = {node_name: np.zeros((1, 3), dtype=float) for node_name in case.target_displacements_mm}
    fake_history = {
        "time": np.array([case.samples[sample_idx].time_ms], dtype=float),
        "INTERNAL ENERGY": np.zeros(1),
        "KINETIC ENERGY": np.zeros(1),
        "EXTERNAL WORK": np.zeros(1),
        "PLASTIC WORK": np.zeros(1),
        "CONTACT ENERGY": np.zeros(1),
    }
    metrics = result_metrics(case, displacements, reactions)
    strain_norm = max(float(np.nanmax(metrics["max_abs_strain"])), 1.0e-6)
    return render_result_frame(case, fake_history, displacements, reactions, metrics, 0, strain_norm)


def make_target_preview(case: PeriodicMotionCase, frames: int, duration_ms: int) -> None:
    sample_idx = np.linspace(0, len(case.samples) - 1, max(1, frames)).round().astype(int)
    images = [render_target_preview_frame(case, int(idx)) for idx in sample_idx]
    poster = render_target_preview_frame(case, int(sample_idx[-1]))
    poster.save(case.deck.poster_path)
    images[0].save(case.deck.gif_path, save_all=True, append_images=images[1:], duration=max(20, duration_ms), loop=0)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--description", type=Path, default=DEFAULT_DESCRIPTION_PATH)
    parser.add_argument("--materials", type=Path, default=stage1.DEFAULT_MATERIALS_PATH)
    parser.add_argument("--actuators", type=Path, default=stage1.DEFAULT_ACTUATORS_PATH)
    parser.add_argument("--batteries", type=Path, default=stage1.DEFAULT_BATTERIES_PATH)
    parser.add_argument("--case-name", default="stage2_viewer_periodic_motion")
    parser.add_argument("--run-name", default=RUN_NAME)
    parser.add_argument("--out-dir", type=Path, default=Path("stage2_outputs/openradioss_periodic_motion"))
    parser.add_argument("--samples", type=int, default=25)
    parser.add_argument("--solver-duration-ms", type=float, default=8.0)
    parser.add_argument("--viewer-start-seconds", type=float, default=0.0)
    parser.add_argument(
        "--viewer-motion-seconds",
        type=float,
        default=0.005,
        help="Second-scale periodic-motion slice represented inside the millisecond FEM window.",
    )
    parser.add_argument("--babble-scale", type=float, default=1.0)
    parser.add_argument("--motion-scale", type=float, default=1.0)
    parser.add_argument("--target-element-length-mm", type=float, default=8.0)
    parser.add_argument(
        "--uniform-radius-mm",
        type=float,
        default=beam_deck.DEFAULT_UNIFORM_RADIUS_MM,
        help="Use one circular beam radius for every rod member.",
    )
    parser.add_argument(
        "--control-policy",
        choices=("stage1-torque-replay", "uniform-joint-guides", "all-joints-hard"),
        default="stage1-torque-replay",
    )
    parser.add_argument(
        "--guide-stiffness",
        type=float,
        default=0.004,
        help="Translational stiffness for each uniform joint guide spring in model units.",
    )
    parser.add_argument(
        "--guide-damping",
        type=float,
        default=0.00004,
        help="Translational damping for each uniform joint guide spring in model units.",
    )
    parser.add_argument(
        "--torque-scale",
        type=float,
        default=1.0,
        help="Scale factor applied to Stage 1 torque rows before writing /CLOAD moment couples.",
    )
    parser.add_argument(
        "--minimum-radius-for-massless-members",
        action="store_true",
        help="Use the legacy 0.5 mm fallback radius for massless connector rods instead of each rod's nominal radius.",
    )
    parser.add_argument("--no-preview-gif", action="store_true")
    parser.add_argument("--preview-frames", type=int, default=20)
    parser.add_argument("--preview-duration-ms", type=int, default=90)
    parser.add_argument("--result-csv", type=Path, default=None)
    parser.add_argument(
        "--result-view",
        choices=("map", "3d", "both"),
        default="map",
        help="Render the solved result as the side/top tension map, the 3D cylinder view, or both.",
    )
    parser.add_argument("--result-frames", type=int, default=36)
    parser.add_argument("--result-duration-ms", type=int, default=90)
    return parser


def load_inputs(args: argparse.Namespace) -> tuple[Any, stage1.PhysicalCatalog]:
    description = load_dog_description(args.description)
    catalog = stage1.load_catalog(args.materials, args.actuators, args.batteries)
    return description, catalog


def main(argv: Iterable[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    description, catalog = load_inputs(args)
    case = build_periodic_motion_case(
        description,
        catalog,
        out_dir=args.out_dir,
        run_name=args.run_name,
        sample_count=args.samples,
        solver_duration_ms=args.solver_duration_ms,
        viewer_start_seconds=args.viewer_start_seconds,
        viewer_motion_seconds=args.viewer_motion_seconds,
        babble_scale=args.babble_scale,
        motion_scale=args.motion_scale,
        target_element_length_mm=args.target_element_length_mm,
        use_nominal_radius_for_massless_members=not args.minimum_radius_for_massless_members,
        uniform_radius_mm=args.uniform_radius_mm,
        case_name=args.case_name,
        control_policy=args.control_policy,
        guide_stiffness=args.guide_stiffness,
        guide_damping=args.guide_damping,
        torque_scale=args.torque_scale,
    )
    write_case(
        case,
        make_preview_gif=not args.no_preview_gif,
        preview_frames=args.preview_frames,
        preview_duration_ms=args.preview_duration_ms,
    )
    print(f"wrote OpenRadioss periodic-motion whole-body beam case to {args.out_dir}")
    print(f"starter: {case.deck.starter_path.name}")
    print(f"engine: {case.deck.engine_path.name}")
    print(f"target motion: {case.target_csv_path.name}")
    print(f"rod graph nodes: {len(case.deck.rod_model.nodes)}")
    print(f"solver nodes: {len(case.deck.node_ids)}")
    print(f"beam elements: {len(case.deck.members)}")
    summary = motion_summary(case)
    print(f"imposed displacement functions: {summary['counts']['imposed_displacement_functions']}")
    print(f"control policy: {case.control_policy}")
    print(f"uniform guided joint nodes: {summary['counts']['uniform_guided_joint_nodes']}")
    print(f"hard prescribed robot joint nodes: {summary['counts']['hard_prescribed_robot_joint_nodes']}")
    print(f"stage1 torque replay moment functions: {summary['counts']['concentrated_moment_functions']}")
    print(f"uniform radius mm: {case.deck.uniform_radius_mm}")
    print(f"max target displacement mm: {summary['max_target_displacement_mm']:.6g}")

    result_csv = args.result_csv or (args.out_dir / f"{args.run_name}T01.csv")
    if result_csv.is_file():
        if args.result_view in {"map", "both"}:
            make_result_gif(case, result_csv, frames=args.result_frames, duration_ms=args.result_duration_ms)
            print(f"result gif: {case.result_gif_path.name}")
            print(f"result poster: {case.result_poster_path.name}")
        if args.result_view in {"3d", "both"}:
            make_result_3d_gif(case, result_csv, frames=args.result_frames, duration_ms=args.result_duration_ms)
            print(f"result 3d gif: {case.result_3d_gif_path.name}")
            print(f"result 3d poster: {case.result_3d_poster_path.name}")
    else:
        print(f"result csv not found yet: {result_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
