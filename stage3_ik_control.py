#!/usr/bin/env python3
"""Stage 3 IK and control-side safety scaffold.

This stage turns foot targets into joint commands, then checks those commands
against the existing Stage 1 mass/torque model. It is not a contact dynamics
solver, a MuJoCo model, or a whole-body balance controller.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np
import yaml

from dog_description import DEFAULT_DESCRIPTION_PATH, DogDescription, JointRange, load_dog_description
from endpoint_geometry import LEG_ORDER
import mass_model as stage1


DEFAULT_OUT_DIR = Path("stage3_outputs/ik_control")
LEG_JOINT_TYPES = ("hip_ab", "hip_pitch", "knee_bend", "toe_bend")
BODY_JOINT_TYPES = ("waist_yaw", "waist_pitch")
HEAD_JOINT_TYPES = ("neck_yaw", "neck_pitch", "head_claw")


@dataclass(frozen=True)
class LegIKSolution:
    leg_name: str
    target_m: np.ndarray
    chain: stage1.LegChain
    angles_rad: dict[str, float]
    residual_m: float
    iterations: int
    reached: bool


@dataclass(frozen=True)
class ControlSafetyReport:
    max_ik_residual_m: float
    min_continuous_torque_margin: float | None
    worst_margin_joint: str
    min_joint_limit_margin_deg: float
    support_polygon_margin_m: float
    safe_to_execute: bool
    failure_reasons: tuple[str, ...]


@dataclass(frozen=True)
class TrajectoryFrame:
    frame_index: int
    time_s: float
    primitive: str
    waist_yaw_deg: float
    waist_pitch_deg: float
    support_legs: tuple[str, ...]
    ik_solutions: dict[str, LegIKSolution]
    model: stage1.MassModel
    torque_rows: list[stage1.TorqueRow]
    safety: ControlSafetyReport


@dataclass(frozen=True)
class Stage3Case:
    primitive: str
    frames: list[TrajectoryFrame]
    min_torque_margin: float
    min_joint_limit_margin_deg: float
    ik_tolerance_m: float
    out_dir: Path


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def clamped_array(values: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    return np.minimum(np.maximum(values, lower), upper)


def normalized(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        return vector
    return vector / norm


def joint_range(description: DogDescription, joint_type: str) -> JointRange:
    return description.joint_ranges[joint_type]


def leg_bounds_rad(description: DogDescription) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    specs = [joint_range(description, joint) for joint in LEG_JOINT_TYPES]
    lower = np.array([math.radians(spec.min_deg) for spec in specs], dtype=float)
    upper = np.array([math.radians(spec.max_deg) for spec in specs], dtype=float)
    bias = np.array([math.radians(spec.bias_deg) for spec in specs], dtype=float)
    return lower, upper, bias


def leg_endpoint_from_angles(
    description: DogDescription,
    pose: stage1.BodyPose,
    leg_name: str,
    angles_rad: np.ndarray,
    target_m: np.ndarray,
) -> np.ndarray:
    chain = stage1.solve_leg_chain_from_angles(
        description,
        pose,
        leg_name,
        float(angles_rad[0]),
        float(angles_rad[1]),
        float(angles_rad[2]),
        float(angles_rad[3]),
        requested_toe_endpoint=target_m,
    )
    return chain.toe_endpoint


def initial_leg_ik_seeds(
    description: DogDescription,
    pose: stage1.BodyPose,
    leg_name: str,
    target_m: np.ndarray,
    previous_angles_rad: Mapping[str, float] | None,
) -> list[np.ndarray]:
    lower, upper, bias = leg_bounds_rad(description)
    hip = pose.hips[leg_name]
    forward, outward, down = pose.bases[leg_name]
    offset = target_m - hip
    total_length = (
        description.viewer.links.upper_m
        + description.viewer.links.lower_m
        + description.viewer.links.distal_endpoint_m
    )
    lateral = float(np.dot(offset, outward))
    forward_offset = float(np.dot(offset, forward))
    down_offset = max(float(np.dot(offset, down)), 1e-9)
    hip_ab_guess = math.asin(clamp(lateral / max(total_length, 1e-9), -0.95, 0.95))
    hip_pitch_guess = math.atan2(forward_offset, down_offset)
    knee_bias = math.radians(description.joint_ranges["knee_bend"].bias_deg)
    toe_bias = math.radians(description.joint_ranges["toe_bend"].bias_deg)

    seeds: list[np.ndarray] = []
    if previous_angles_rad is not None:
        seeds.append(np.array([previous_angles_rad[joint] for joint in LEG_JOINT_TYPES], dtype=float))

    seeds.extend(
        [
            np.array([hip_ab_guess, hip_pitch_guess, knee_bias, toe_bias], dtype=float),
            np.array([hip_ab_guess, hip_pitch_guess, math.radians(40.0), math.radians(-12.0)], dtype=float),
            np.array([hip_ab_guess, hip_pitch_guess, math.radians(75.0), math.radians(-20.0)], dtype=float),
            bias,
        ]
    )
    return [clamped_array(seed, lower, upper) for seed in seeds]


def finite_difference_jacobian(
    description: DogDescription,
    pose: stage1.BodyPose,
    leg_name: str,
    target_m: np.ndarray,
    angles_rad: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
) -> np.ndarray:
    jacobian = np.zeros((3, 4), dtype=float)
    epsilon = 1e-5
    for idx in range(4):
        plus = angles_rad.copy()
        minus = angles_rad.copy()
        plus[idx] = min(upper[idx], plus[idx] + epsilon)
        minus[idx] = max(lower[idx], minus[idx] - epsilon)
        denom = plus[idx] - minus[idx]
        if denom <= 1e-12:
            continue
        plus_endpoint = leg_endpoint_from_angles(description, pose, leg_name, plus, target_m)
        minus_endpoint = leg_endpoint_from_angles(description, pose, leg_name, minus, target_m)
        jacobian[:, idx] = (plus_endpoint - minus_endpoint) / denom
    return jacobian


def solve_leg_ik(
    description: DogDescription,
    pose: stage1.BodyPose,
    leg_name: str,
    target_m: np.ndarray,
    previous_angles_rad: Mapping[str, float] | None = None,
    tolerance_m: float = 1e-5,
    max_iterations: int = 70,
) -> LegIKSolution:
    lower, upper, _bias = leg_bounds_rad(description)
    best_angles: np.ndarray | None = None
    best_residual = math.inf
    best_iterations = 0

    for seed in initial_leg_ik_seeds(description, pose, leg_name, target_m, previous_angles_rad):
        angles = seed.copy()
        damping = 1e-4
        iterations_used = 0
        for iteration in range(max_iterations):
            iterations_used = iteration + 1
            endpoint = leg_endpoint_from_angles(description, pose, leg_name, angles, target_m)
            residual = target_m - endpoint
            residual_norm = float(np.linalg.norm(residual))
            if residual_norm <= tolerance_m:
                break

            jacobian = finite_difference_jacobian(description, pose, leg_name, target_m, angles, lower, upper)
            lhs = jacobian.T @ jacobian + damping * np.eye(4)
            rhs = jacobian.T @ residual
            try:
                delta = np.linalg.solve(lhs, rhs)
            except np.linalg.LinAlgError:
                delta = np.linalg.lstsq(lhs, rhs, rcond=None)[0]

            max_step = math.radians(10.0)
            step_norm = float(np.linalg.norm(delta))
            if step_norm > max_step:
                delta *= max_step / step_norm

            candidate = clamped_array(angles + delta, lower, upper)
            candidate_residual = float(
                np.linalg.norm(target_m - leg_endpoint_from_angles(description, pose, leg_name, candidate, target_m))
            )
            if candidate_residual <= residual_norm + 1e-12:
                angles = candidate
                damping = max(damping * 0.7, 1e-8)
            else:
                damping = min(damping * 10.0, 1e2)

        final_endpoint = leg_endpoint_from_angles(description, pose, leg_name, angles, target_m)
        final_residual = float(np.linalg.norm(target_m - final_endpoint))
        if final_residual < best_residual:
            best_residual = final_residual
            best_angles = angles.copy()
            best_iterations = iterations_used

    if best_angles is None:
        raise RuntimeError(f"No IK seed was attempted for {leg_name}")

    chain = stage1.solve_leg_chain_from_angles(
        description,
        pose,
        leg_name,
        float(best_angles[0]),
        float(best_angles[1]),
        float(best_angles[2]),
        float(best_angles[3]),
        requested_toe_endpoint=target_m,
    )
    return LegIKSolution(
        leg_name=leg_name,
        target_m=np.array(target_m, dtype=float),
        chain=chain,
        angles_rad={joint: float(best_angles[idx]) for idx, joint in enumerate(LEG_JOINT_TYPES)},
        residual_m=best_residual,
        iterations=best_iterations,
        reached=best_residual <= tolerance_m,
    )


def angle_limit_margin_deg(value_deg: float, spec: JointRange) -> float:
    return min(value_deg - spec.min_deg, spec.max_deg - value_deg)


def convex_hull(points: list[np.ndarray]) -> list[np.ndarray]:
    unique = sorted({(float(point[0]), float(point[1])) for point in points})
    if len(unique) <= 1:
        return [np.array(point, dtype=float) for point in unique]

    def cross(origin: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0])

    lower: list[tuple[float, float]] = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)

    upper: list[tuple[float, float]] = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)

    return [np.array(point, dtype=float) for point in lower[:-1] + upper[:-1]]


def distance_to_segment(point: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-12:
        return float(np.linalg.norm(point - a))
    t = clamp(float(np.dot(point - a, ab) / denom), 0.0, 1.0)
    closest = a + t * ab
    return float(np.linalg.norm(point - closest))


def support_polygon_margin(com_xy: np.ndarray, support_points_xy: Iterable[np.ndarray]) -> float:
    points = [np.array(point, dtype=float)[:2] for point in support_points_xy]
    if not points:
        return -math.inf
    if len(points) == 1:
        return -float(np.linalg.norm(com_xy - points[0]))
    if len(points) == 2:
        return -distance_to_segment(com_xy, points[0], points[1])

    hull = convex_hull(points)
    if len(hull) < 3:
        return -min(distance_to_segment(com_xy, hull[idx], hull[(idx + 1) % len(hull)]) for idx in range(len(hull)))

    edge_distances = [distance_to_segment(com_xy, hull[idx], hull[(idx + 1) % len(hull)]) for idx in range(len(hull))]
    inside = True
    for idx, a in enumerate(hull):
        b = hull[(idx + 1) % len(hull)]
        edge = b - a
        rel = com_xy - a
        if edge[0] * rel[1] - edge[1] * rel[0] < -1e-10:
            inside = False
            break
    distance = min(edge_distances)
    return distance if inside else -distance


def frame_joint_limit_margin_deg(description: DogDescription, frame: TrajectoryFrame) -> float:
    margins = [
        angle_limit_margin_deg(frame.waist_yaw_deg, description.joint_ranges["waist_yaw"]),
        angle_limit_margin_deg(frame.waist_pitch_deg, description.joint_ranges["waist_pitch"]),
    ]
    for joint in HEAD_JOINT_TYPES:
        spec = description.joint_ranges[joint]
        margins.append(angle_limit_margin_deg(spec.bias_deg, spec))
    for solution in frame.ik_solutions.values():
        for joint in LEG_JOINT_TYPES:
            spec = description.joint_ranges[joint]
            margins.append(angle_limit_margin_deg(math.degrees(solution.angles_rad[joint]), spec))
    return min(margins)


def evaluate_safety(
    description: DogDescription,
    ik_solutions: dict[str, LegIKSolution],
    model: stage1.MassModel,
    torque_rows: list[stage1.TorqueRow],
    waist_yaw_deg: float,
    waist_pitch_deg: float,
    support_legs: tuple[str, ...],
    min_torque_margin: float,
    min_joint_limit_margin_deg: float,
    ik_tolerance_m: float,
) -> ControlSafetyReport:
    finite_rows = [row for row in torque_rows if row.continuous_margin is not None]
    worst_margin = min(finite_rows, key=lambda row: row.continuous_margin or math.inf)
    min_torque = worst_margin.continuous_margin
    support_points = [ik_solutions[leg].chain.toe_endpoint[:2] for leg in support_legs]
    support_margin = support_polygon_margin(model.com_m[:2], support_points)

    joint_margins = [
        angle_limit_margin_deg(waist_yaw_deg, description.joint_ranges["waist_yaw"]),
        angle_limit_margin_deg(waist_pitch_deg, description.joint_ranges["waist_pitch"]),
    ]
    for joint in HEAD_JOINT_TYPES:
        spec = description.joint_ranges[joint]
        joint_margins.append(angle_limit_margin_deg(spec.bias_deg, spec))
    for solution in ik_solutions.values():
        for joint in LEG_JOINT_TYPES:
            spec = description.joint_ranges[joint]
            joint_margins.append(angle_limit_margin_deg(math.degrees(solution.angles_rad[joint]), spec))

    max_residual = max((solution.residual_m for solution in ik_solutions.values()), default=0.0)
    min_joint_margin = min(joint_margins)
    reasons: list[str] = []
    if max_residual > ik_tolerance_m:
        reasons.append("ik_residual")
    if min_torque is None or min_torque < min_torque_margin:
        reasons.append("torque_margin")
    if min_joint_margin < min_joint_limit_margin_deg:
        reasons.append("joint_limit_margin")
    if not math.isfinite(support_margin) or support_margin < 0.0:
        reasons.append("support_polygon")

    return ControlSafetyReport(
        max_ik_residual_m=max_residual,
        min_continuous_torque_margin=None if min_torque is None else float(min_torque),
        worst_margin_joint=worst_margin.joint,
        min_joint_limit_margin_deg=float(min_joint_margin),
        support_polygon_margin_m=float(support_margin),
        safe_to_execute=not reasons,
        failure_reasons=tuple(reasons),
    )


def build_stand_targets(geometry: Any) -> dict[str, np.ndarray]:
    return {leg: np.array(point, dtype=float) for leg, point in stage1.leg_endpoint_targets(geometry).items()}


def crawl_step_targets(
    base_targets: dict[str, np.ndarray],
    swing_leg: str,
    phase: float,
    step_length_m: float,
    step_height_m: float,
) -> dict[str, np.ndarray]:
    targets = {leg: np.array(point, dtype=float) for leg, point in base_targets.items()}
    swing = targets[swing_leg].copy()
    swing[0] += (phase - 0.5) * step_length_m
    swing[2] += math.sin(math.pi * phase) * step_height_m
    targets[swing_leg] = swing
    return targets


def support_legs_for_primitive(primitive: str, swing_leg: str, phase: float) -> tuple[str, ...]:
    if primitive == "crawl_step" and 0.0 < phase < 1.0:
        return tuple(leg for leg in LEG_ORDER if leg != swing_leg)
    return tuple(LEG_ORDER)


def build_trajectory_frame(
    description: DogDescription,
    catalog: stage1.PhysicalCatalog,
    frame_index: int,
    time_s: float,
    primitive: str,
    waist_yaw_deg: float,
    waist_pitch_deg: float,
    foot_targets: dict[str, np.ndarray],
    support_legs: tuple[str, ...],
    previous_angles: dict[str, dict[str, float]],
    min_torque_margin: float,
    min_joint_limit_margin_deg: float,
    ik_tolerance_m: float,
) -> TrajectoryFrame:
    assumptions = stage1.Stage1Assumptions()
    geometry = stage1.robot_geometry(description)
    pose = stage1.make_body_pose(geometry, description.viewer.body_z_m, waist_yaw_deg, waist_pitch_deg)

    ik_solutions: dict[str, LegIKSolution] = {}
    for leg in LEG_ORDER:
        solution = solve_leg_ik(
            description,
            pose,
            leg,
            foot_targets[leg],
            previous_angles.get(leg),
            tolerance_m=ik_tolerance_m,
        )
        ik_solutions[leg] = solution
        previous_angles[leg] = solution.angles_rad

    ranges = description.joint_ranges
    head = stage1.make_head_chain_from_angles(
        description,
        pose,
        math.radians(ranges["neck_yaw"].bias_deg),
        math.radians(ranges["neck_pitch"].bias_deg),
        math.radians(ranges["head_claw"].bias_deg),
    )
    model = stage1.build_mass_model_from_chains(
        f"{primitive}_{frame_index:03d}",
        description,
        catalog,
        assumptions,
        pose,
        {leg: solution.chain for leg, solution in ik_solutions.items()},
        head,
        geometry,
    )
    torque_rows = stage1.estimate_torques(model, catalog, assumptions)
    safety = evaluate_safety(
        description,
        ik_solutions,
        model,
        torque_rows,
        waist_yaw_deg,
        waist_pitch_deg,
        support_legs,
        min_torque_margin,
        min_joint_limit_margin_deg,
        ik_tolerance_m,
    )
    return TrajectoryFrame(
        frame_index=frame_index,
        time_s=time_s,
        primitive=primitive,
        waist_yaw_deg=waist_yaw_deg,
        waist_pitch_deg=waist_pitch_deg,
        support_legs=support_legs,
        ik_solutions=ik_solutions,
        model=model,
        torque_rows=torque_rows,
        safety=safety,
    )


def build_stage3_case(
    description: DogDescription,
    catalog: stage1.PhysicalCatalog,
    out_dir: Path,
    primitive: str = "stand",
    frame_count: int = 5,
    duration_s: float = 1.0,
    waist_yaw_deg: float = 0.0,
    waist_pitch_deg: float = 0.0,
    swing_leg: str = "front_left",
    step_length_m: float = 0.040,
    step_height_m: float = 0.025,
    min_torque_margin: float = 1.0,
    min_joint_limit_margin_deg: float = 1.0,
    ik_tolerance_m: float = 1e-4,
) -> Stage3Case:
    if primitive not in {"stand", "crawl_step"}:
        raise ValueError(f"unknown Stage 3 primitive: {primitive}")
    if frame_count < 1:
        raise ValueError("frame_count must be at least 1")
    if swing_leg not in LEG_ORDER:
        raise ValueError(f"unknown swing leg: {swing_leg}")

    geometry = stage1.robot_geometry(description)
    base_targets = build_stand_targets(geometry)
    previous_angles: dict[str, dict[str, float]] = {}
    frames: list[TrajectoryFrame] = []
    for idx in range(frame_count):
        phase = 0.0 if frame_count == 1 else idx / (frame_count - 1)
        time_s = duration_s * phase
        if primitive == "crawl_step":
            targets = crawl_step_targets(base_targets, swing_leg, phase, step_length_m, step_height_m)
        else:
            targets = {leg: point.copy() for leg, point in base_targets.items()}
        frames.append(
            build_trajectory_frame(
                description,
                catalog,
                idx,
                time_s,
                primitive,
                waist_yaw_deg,
                waist_pitch_deg,
                targets,
                support_legs_for_primitive(primitive, swing_leg, phase),
                previous_angles,
                min_torque_margin,
                min_joint_limit_margin_deg,
                ik_tolerance_m,
            )
        )

    return Stage3Case(
        primitive=primitive,
        frames=frames,
        min_torque_margin=min_torque_margin,
        min_joint_limit_margin_deg=min_joint_limit_margin_deg,
        ik_tolerance_m=ik_tolerance_m,
        out_dir=out_dir,
    )


def fmt(value: float | None) -> str:
    if value is None:
        return ""
    if not math.isfinite(value):
        return str(value)
    return f"{value:.9g}"


def write_joint_commands_csv(description: DogDescription, frames: list[TrajectoryFrame], path: Path) -> None:
    fields = [
        "frame_index",
        "time_s",
        "scope",
        "leg",
        "joint",
        "command_deg",
        "min_deg",
        "max_deg",
        "limit_margin_deg",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for frame in frames:
            body_commands = {
                "waist_yaw": frame.waist_yaw_deg,
                "waist_pitch": frame.waist_pitch_deg,
            }
            for joint, value in body_commands.items():
                spec = description.joint_ranges[joint]
                writer.writerow(
                    {
                        "frame_index": frame.frame_index,
                        "time_s": fmt(frame.time_s),
                        "scope": "body",
                        "leg": "",
                        "joint": joint,
                        "command_deg": fmt(value),
                        "min_deg": fmt(spec.min_deg),
                        "max_deg": fmt(spec.max_deg),
                        "limit_margin_deg": fmt(angle_limit_margin_deg(value, spec)),
                    }
                )
            for joint in HEAD_JOINT_TYPES:
                spec = description.joint_ranges[joint]
                writer.writerow(
                    {
                        "frame_index": frame.frame_index,
                        "time_s": fmt(frame.time_s),
                        "scope": "head",
                        "leg": "",
                        "joint": joint,
                        "command_deg": fmt(spec.bias_deg),
                        "min_deg": fmt(spec.min_deg),
                        "max_deg": fmt(spec.max_deg),
                        "limit_margin_deg": fmt(angle_limit_margin_deg(spec.bias_deg, spec)),
                    }
                )
            for leg in LEG_ORDER:
                solution = frame.ik_solutions[leg]
                for joint in LEG_JOINT_TYPES:
                    spec = description.joint_ranges[joint]
                    value = math.degrees(solution.angles_rad[joint])
                    writer.writerow(
                        {
                            "frame_index": frame.frame_index,
                            "time_s": fmt(frame.time_s),
                            "scope": "leg",
                            "leg": leg,
                            "joint": f"{leg}_{joint}",
                            "command_deg": fmt(value),
                            "min_deg": fmt(spec.min_deg),
                            "max_deg": fmt(spec.max_deg),
                            "limit_margin_deg": fmt(angle_limit_margin_deg(value, spec)),
                        }
                    )


def write_foot_targets_csv(frames: list[TrajectoryFrame], path: Path) -> None:
    fields = [
        "frame_index",
        "time_s",
        "leg",
        "support_state",
        "target_x_m",
        "target_y_m",
        "target_z_m",
        "solved_x_m",
        "solved_y_m",
        "solved_z_m",
        "residual_m",
        "reached",
        "iterations",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for frame in frames:
            support = set(frame.support_legs)
            for leg in LEG_ORDER:
                solution = frame.ik_solutions[leg]
                target = solution.target_m
                solved = solution.chain.toe_endpoint
                writer.writerow(
                    {
                        "frame_index": frame.frame_index,
                        "time_s": fmt(frame.time_s),
                        "leg": leg,
                        "support_state": "support" if leg in support else "swing",
                        "target_x_m": fmt(float(target[0])),
                        "target_y_m": fmt(float(target[1])),
                        "target_z_m": fmt(float(target[2])),
                        "solved_x_m": fmt(float(solved[0])),
                        "solved_y_m": fmt(float(solved[1])),
                        "solved_z_m": fmt(float(solved[2])),
                        "residual_m": fmt(solution.residual_m),
                        "reached": "yes" if solution.reached else "no",
                        "iterations": solution.iterations,
                    }
                )


def write_trajectory_frames_csv(frames: list[TrajectoryFrame], path: Path) -> None:
    fields = [
        "frame_index",
        "time_s",
        "primitive",
        "waist_yaw_deg",
        "waist_pitch_deg",
        "support_legs",
        "total_mass_kg",
        "com_x_m",
        "com_y_m",
        "com_z_m",
        "max_ik_residual_m",
        "min_continuous_torque_margin",
        "worst_margin_joint",
        "support_polygon_margin_m",
        "safe_to_execute",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for frame in frames:
            safety = frame.safety
            com = frame.model.com_m
            writer.writerow(
                {
                    "frame_index": frame.frame_index,
                    "time_s": fmt(frame.time_s),
                    "primitive": frame.primitive,
                    "waist_yaw_deg": fmt(frame.waist_yaw_deg),
                    "waist_pitch_deg": fmt(frame.waist_pitch_deg),
                    "support_legs": " ".join(frame.support_legs),
                    "total_mass_kg": fmt(frame.model.total_mass_kg),
                    "com_x_m": fmt(float(com[0])),
                    "com_y_m": fmt(float(com[1])),
                    "com_z_m": fmt(float(com[2])),
                    "max_ik_residual_m": fmt(safety.max_ik_residual_m),
                    "min_continuous_torque_margin": fmt(safety.min_continuous_torque_margin),
                    "worst_margin_joint": safety.worst_margin_joint,
                    "support_polygon_margin_m": fmt(safety.support_polygon_margin_m),
                    "safe_to_execute": "yes" if safety.safe_to_execute else "no",
                }
            )


def write_control_safety_csv(case: Stage3Case, path: Path) -> None:
    fields = [
        "frame_index",
        "time_s",
        "max_ik_residual_m",
        "ik_tolerance_m",
        "min_continuous_torque_margin",
        "required_torque_margin",
        "min_joint_limit_margin_deg",
        "required_joint_limit_margin_deg",
        "support_polygon_margin_m",
        "safe_to_execute",
        "failure_reasons",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for frame in case.frames:
            safety = frame.safety
            writer.writerow(
                {
                    "frame_index": frame.frame_index,
                    "time_s": fmt(frame.time_s),
                    "max_ik_residual_m": fmt(safety.max_ik_residual_m),
                    "ik_tolerance_m": fmt(case.ik_tolerance_m),
                    "min_continuous_torque_margin": fmt(safety.min_continuous_torque_margin),
                    "required_torque_margin": fmt(case.min_torque_margin),
                    "min_joint_limit_margin_deg": fmt(safety.min_joint_limit_margin_deg),
                    "required_joint_limit_margin_deg": fmt(case.min_joint_limit_margin_deg),
                    "support_polygon_margin_m": fmt(safety.support_polygon_margin_m),
                    "safe_to_execute": "yes" if safety.safe_to_execute else "no",
                    "failure_reasons": "ok" if not safety.failure_reasons else " ".join(safety.failure_reasons),
                }
            )


def summary_dict(case: Stage3Case) -> dict[str, Any]:
    safe_frames = [frame for frame in case.frames if frame.safety.safe_to_execute]
    max_residual = max((frame.safety.max_ik_residual_m for frame in case.frames), default=0.0)
    min_torque = min(
        (
            frame.safety.min_continuous_torque_margin
            for frame in case.frames
            if frame.safety.min_continuous_torque_margin is not None
        ),
        default=None,
    )
    min_support = min((frame.safety.support_polygon_margin_m for frame in case.frames), default=math.inf)
    return {
        "stage": "stage_3_ik_and_control_development",
        "primitive": case.primitive,
        "analysis_state": {
            "ik_solver_applied": True,
            "joint_trajectory_generated": True,
            "torque_margin_checked_against_stage1": True,
            "support_polygon_awareness_applied": True,
            "gravity_contact_dynamics_solved": False,
            "mujoco_model_exported": False,
        },
        "counts": {
            "frames": len(case.frames),
            "safe_frames": len(safe_frames),
            "unsafe_frames": len(case.frames) - len(safe_frames),
            "legs": len(LEG_ORDER),
            "leg_joint_commands_per_frame": len(LEG_ORDER) * len(LEG_JOINT_TYPES),
        },
        "thresholds": {
            "ik_tolerance_m": float(case.ik_tolerance_m),
            "min_torque_margin": float(case.min_torque_margin),
            "min_joint_limit_margin_deg": float(case.min_joint_limit_margin_deg),
        },
        "aggregate": {
            "max_ik_residual_m": float(max_residual),
            "min_continuous_torque_margin": None if min_torque is None else float(min_torque),
            "min_support_polygon_margin_m": float(min_support),
        },
        "outputs": {
            "joint_commands": "joint_commands.csv",
            "foot_targets": "foot_targets.csv",
            "trajectory_frames": "trajectory_frames.csv",
            "control_safety_summary": "control_safety_summary.csv",
        },
        "notes": [
            "Foot-target IK is solved against the existing Stage 1 linkage axes.",
            "Torque margins are free-space Stage 1 inertia estimates, not gravity or contact reaction loads.",
            "Support polygon margin is a geometric COM projection check only.",
            "Stage 4 MuJoCo/contact validation is still separate.",
        ],
    }


def write_summary_yaml(case: Stage3Case, path: Path) -> None:
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(summary_dict(case), handle, sort_keys=False)


def write_outputs(case: Stage3Case, description: DogDescription) -> None:
    case.out_dir.mkdir(parents=True, exist_ok=True)
    write_joint_commands_csv(description, case.frames, case.out_dir / "joint_commands.csv")
    write_foot_targets_csv(case.frames, case.out_dir / "foot_targets.csv")
    write_trajectory_frames_csv(case.frames, case.out_dir / "trajectory_frames.csv")
    write_control_safety_csv(case, case.out_dir / "control_safety_summary.csv")
    write_summary_yaml(case, case.out_dir / "stage3_ik_control_summary.yaml")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--description", type=Path, default=DEFAULT_DESCRIPTION_PATH)
    parser.add_argument("--materials", type=Path, default=stage1.DEFAULT_MATERIALS_PATH)
    parser.add_argument("--actuators", type=Path, default=stage1.DEFAULT_ACTUATORS_PATH)
    parser.add_argument("--batteries", type=Path, default=stage1.DEFAULT_BATTERIES_PATH)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--primitive", choices=("stand", "crawl_step"), default="stand")
    parser.add_argument("--frame-count", type=int, default=5)
    parser.add_argument("--duration-s", type=float, default=1.0)
    parser.add_argument("--waist-yaw-deg", type=float, default=0.0)
    parser.add_argument("--waist-pitch-deg", type=float, default=0.0)
    parser.add_argument("--swing-leg", choices=LEG_ORDER, default="front_left")
    parser.add_argument("--step-length-m", type=float, default=0.040)
    parser.add_argument("--step-height-m", type=float, default=0.025)
    parser.add_argument("--min-torque-margin", type=float, default=1.0)
    parser.add_argument("--min-joint-limit-margin-deg", type=float, default=1.0)
    parser.add_argument("--ik-tolerance-m", type=float, default=1e-4)
    return parser


def load_inputs(args: argparse.Namespace) -> tuple[DogDescription, stage1.PhysicalCatalog]:
    description = load_dog_description(args.description)
    catalog = stage1.load_catalog(args.materials, args.actuators, args.batteries)
    return description, catalog


def main(argv: Iterable[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    description, catalog = load_inputs(args)
    case = build_stage3_case(
        description,
        catalog,
        out_dir=args.out_dir,
        primitive=args.primitive,
        frame_count=args.frame_count,
        duration_s=args.duration_s,
        waist_yaw_deg=args.waist_yaw_deg,
        waist_pitch_deg=args.waist_pitch_deg,
        swing_leg=args.swing_leg,
        step_length_m=args.step_length_m,
        step_height_m=args.step_height_m,
        min_torque_margin=args.min_torque_margin,
        min_joint_limit_margin_deg=args.min_joint_limit_margin_deg,
        ik_tolerance_m=args.ik_tolerance_m,
    )
    write_outputs(case, description)

    safe_count = sum(1 for frame in case.frames if frame.safety.safe_to_execute)
    max_residual = max(frame.safety.max_ik_residual_m for frame in case.frames)
    min_torque = min(
        frame.safety.min_continuous_torque_margin or math.inf
        for frame in case.frames
    )
    print(f"wrote Stage 3 IK/control outputs to {case.out_dir}")
    print(f"primitive: {case.primitive}")
    print(f"frames: {len(case.frames)}")
    print(f"safe frames: {safe_count}/{len(case.frames)}")
    print(f"max IK residual m: {max_residual:.9g}")
    print(f"min continuous torque margin: {min_torque:.6g}")
    print("analysis state: IK/control scaffold only; no MuJoCo, no contact dynamics")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
