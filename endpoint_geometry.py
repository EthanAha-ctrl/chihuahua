#!/usr/bin/env python3
"""Top-view endpoint geometry model for a yaw/pitch-waist quadruped.

This is intentionally a simple kinematic sketch, not a dynamics simulator.
It answers early CAD questions like:

- Where are the hips when the waist yaws around the rear waist joint?
- If endpoint targets stay fixed in the world frame, how much XY reach does each leg need?
- How much foot clearance remains between neighboring endpoint targets?

Coordinate convention:

- +x points forward.
- +y points to the robot's left.
- The yaw waist joint sits behind the pitch waist joint.
- Top-view scans include yaw geometry; waist pitch is drawn in the pygame viewer.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt
import numpy as np

from dog_description import DEFAULT_DESCRIPTION_PATH, load_dog_description


LEG_ORDER = ("front_left", "front_right", "rear_left", "rear_right")


@dataclass(frozen=True)
class RobotGeometry:
    """Editable geometric parameters, in meters and degrees."""

    front_body_length: float
    rear_body_length: float
    waist_joint_spacing: float
    body_half_width: float

    hip_half_width: float

    # Nominal endpoint target relative to the hip when the robot is straight.
    # A small outward lateral offset gives the robot a slightly wider stance.
    foot_x_offset: float
    foot_lateral_outset: float

    # Top-view leg reach limits. These are not full 3D leg lengths.
    # They approximate the usable XY workspace around each hip.
    min_reach_xy: float
    max_reach_xy: float

    min_foot_clearance: float


@dataclass(frozen=True)
class FrameState:
    waist_deg: float
    hips: dict[str, np.ndarray]
    feet: dict[str, np.ndarray]
    metrics: dict[str, float | int]


def rot2(theta_rad: float) -> np.ndarray:
    c = math.cos(theta_rad)
    s = math.sin(theta_rad)
    return np.array([[c, -s], [s, c]], dtype=float)


def min_pair_distance(points: Iterable[np.ndarray]) -> float:
    pts = list(points)
    if len(pts) < 2:
        return math.inf
    best = math.inf
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            best = min(best, float(np.linalg.norm(pts[i] - pts[j])))
    return best


def local_hip_points(g: RobotGeometry) -> dict[str, np.ndarray]:
    return hip_points(g, waist_rad=0.0)


def local_foot_points(g: RobotGeometry) -> dict[str, np.ndarray]:
    return foot_points(g, waist_rad=0.0)


def segment_rotation_for_leg(waist_rad: float, leg_name: str) -> np.ndarray:
    if leg_name.startswith("front"):
        return rot2(waist_rad)
    return rot2(0.0)


def waist_joint_points(g: RobotGeometry, waist_rad: float) -> tuple[np.ndarray, np.ndarray]:
    """Return top-view yaw and pitch waist joint positions.

    The yaw joint is the rear waist joint. The pitch joint is a fixed spacing
    forward from it along the yawed waist link.
    """

    yaw_joint = np.array([0.0, 0.0])
    pitch_joint = yaw_joint + rot2(waist_rad) @ np.array([g.waist_joint_spacing, 0.0])
    return yaw_joint, pitch_joint


def segment_origin_for_leg(g: RobotGeometry, waist_rad: float, leg_name: str) -> np.ndarray:
    yaw_joint, pitch_joint = waist_joint_points(g, waist_rad)
    if leg_name.startswith("front"):
        return pitch_joint
    return yaw_joint


def segment_local_hip_vector(g: RobotGeometry, leg_name: str) -> np.ndarray:
    side = 1.0 if leg_name.endswith("left") else -1.0
    if leg_name.startswith("front"):
        return np.array([g.front_body_length, side * g.hip_half_width])
    return np.array([-g.rear_body_length, side * g.hip_half_width])


def segment_local_foot_vector(g: RobotGeometry, leg_name: str) -> np.ndarray:
    side = 1.0 if leg_name.endswith("left") else -1.0
    return segment_local_hip_vector(g, leg_name) + np.array([g.foot_x_offset, side * g.foot_lateral_outset])


def hip_points(g: RobotGeometry, waist_rad: float) -> dict[str, np.ndarray]:
    return {
        name: segment_origin_for_leg(g, waist_rad, name)
        + segment_rotation_for_leg(waist_rad, name) @ segment_local_hip_vector(g, name)
        for name in LEG_ORDER
    }


def foot_points(g: RobotGeometry, waist_rad: float) -> dict[str, np.ndarray]:
    return {
        name: segment_origin_for_leg(g, waist_rad, name)
        + segment_rotation_for_leg(waist_rad, name) @ segment_local_foot_vector(g, name)
        for name in LEG_ORDER
    }


def body_segment_polygon(g: RobotGeometry, waist_rad: float, front: bool) -> np.ndarray:
    leg_name = "front_left" if front else "rear_left"
    origin = segment_origin_for_leg(g, waist_rad, leg_name)
    rotation = segment_rotation_for_leg(waist_rad, leg_name)
    if front:
        corners = np.array(
            [
                [0.0, -g.body_half_width],
                [g.front_body_length, -g.body_half_width],
                [g.front_body_length, g.body_half_width],
                [0.0, g.body_half_width],
            ]
        )
        return origin + corners @ rotation.T

    corners = np.array(
        [
            [-g.rear_body_length, -g.body_half_width],
            [0.0, -g.body_half_width],
            [0.0, g.body_half_width],
            [-g.rear_body_length, g.body_half_width],
        ]
    )
    return origin + corners @ rotation.T


def build_state(g: RobotGeometry, waist_deg: float, fixed_world_endpoints: bool = True) -> FrameState:
    waist_rad = math.radians(waist_deg)
    feet_local = local_foot_points(g)
    hips = hip_points(g, waist_rad)

    if fixed_world_endpoints:
        # Endpoint targets remain at the neutral global positions while the body yaws.
        feet = feet_local
    else:
        # Endpoint targets follow each body segment. Useful for comparing
        # body-frame endpoint geometry against fixed-world endpoint geometry.
        feet = foot_points(g, waist_rad)

    reaches = {name: float(np.linalg.norm(feet[name] - hips[name])) for name in LEG_ORDER}
    over_reach = {
        name: max(0.0, reaches[name] - g.max_reach_xy) + max(0.0, g.min_reach_xy - reaches[name])
        for name in LEG_ORDER
    }
    min_clearance = min_pair_distance(feet.values())
    clearance_violation = max(0.0, g.min_foot_clearance - min_clearance)

    metrics: dict[str, float | int] = {
        "max_reach_xy": max(reaches.values()),
        "mean_reach_xy": float(np.mean(list(reaches.values()))),
        "max_reach_usage": max(reaches.values()) / g.max_reach_xy,
        "mean_reach_usage": float(np.mean(list(reaches.values()))) / g.max_reach_xy,
        "workspace_violation": float(sum(over_reach.values())),
        "min_endpoint_clearance": min_clearance,
        "clearance_violation": clearance_violation,
    }

    for name in LEG_ORDER:
        metrics[f"{name}_reach_xy"] = reaches[name]
        metrics[f"{name}_workspace_violation"] = over_reach[name]

    return FrameState(
        waist_deg=waist_deg,
        hips=hips,
        feet=feet,
        metrics=metrics,
    )


def geometry_loss(state: FrameState) -> float:
    """Tiny editable endpoint loss.

    This is deliberately simple. The weights make hard failures visible while
    keeping units readable in the CSV.
    """

    m = state.metrics
    return float(
        80.0 * float(m["workspace_violation"])
        + 30.0 * float(m["clearance_violation"])
        + 2.0 * float(m["max_reach_usage"]) ** 2
        + 0.5 * float(m["mean_reach_usage"]) ** 2
    )


def scan_angles(g: RobotGeometry, max_angle: float, step: float, fixed_world_endpoints: bool) -> list[FrameState]:
    n = int(round((2 * max_angle) / step))
    angles = [-max_angle + i * step for i in range(n + 1)]
    return [build_state(g, angle, fixed_world_endpoints=fixed_world_endpoints) for angle in angles]


def write_csv(states: list[FrameState], path: Path) -> None:
    base_fields = ["waist_deg", "loss"]
    metric_fields = list(states[0].metrics.keys())
    fields = base_fields + metric_fields
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for state in states:
            row = {"waist_deg": state.waist_deg, "loss": geometry_loss(state)}
            row.update(state.metrics)
            writer.writerow(row)


def draw_robot(ax: plt.Axes, g: RobotGeometry, state: FrameState, fixed_world_endpoints: bool) -> None:
    waist_rad = math.radians(state.waist_deg)
    yaw_joint, pitch_joint = waist_joint_points(g, waist_rad)
    front_poly = body_segment_polygon(g, waist_rad, front=True)
    rear_poly = body_segment_polygon(g, waist_rad, front=False)

    for poly, color in ((front_poly, "#87aade"), (rear_poly, "#f2b880")):
        closed = np.vstack([poly, poly[0]])
        ax.fill(closed[:, 0], closed[:, 1], color=color, alpha=0.35, linewidth=0)
        ax.plot(closed[:, 0], closed[:, 1], color="#333333", linewidth=1.0)

    for name in LEG_ORDER:
        hip = state.hips[name]
        foot = state.feet[name]
        ax.plot([hip[0], foot[0]], [hip[1], foot[1]], color="#333333", linewidth=1.0)
        circle = plt.Circle(hip, g.max_reach_xy, fill=False, color="#777777", alpha=0.25, linestyle=":")
        ax.add_patch(circle)
        ax.scatter(hip[0], hip[1], marker="s", s=28, color="#222222", zorder=3)
        ax.scatter(foot[0], foot[1], marker="o", s=36, color="#d84a38", zorder=4)

    ax.plot(
        [yaw_joint[0], pitch_joint[0]],
        [yaw_joint[1], pitch_joint[1]],
        color="#111111",
        linewidth=2.0,
        zorder=5,
    )
    ax.scatter(yaw_joint[0], yaw_joint[1], marker="x", s=45, color="#8f5f2a", zorder=6)
    ax.scatter(pitch_joint[0], pitch_joint[1], marker="x", s=45, color="#315f9f", zorder=6)

    mode = "fixed-world endpoints" if fixed_world_endpoints else "body-frame endpoints"
    ax.set_title(
        f"waist {state.waist_deg:+.0f} deg | loss {geometry_loss(state):.2f}\n{mode}",
        fontsize=10,
    )
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("x forward (m)")
    ax.set_ylabel("y left (m)")
    ax.set_xlim(-0.45, 0.45)
    ax.set_ylim(-0.32, 0.38)


def plot_geometry(states: list[FrameState], g: RobotGeometry, path: Path, fixed_world_endpoints: bool) -> None:
    requested = [-35, -20, 0, 20, 35]
    chosen = [min(states, key=lambda s, a=a: abs(s.waist_deg - a)) for a in requested]

    fig, axes = plt.subplots(1, len(chosen), figsize=(17, 4.4), constrained_layout=True)
    for ax, state in zip(axes, chosen):
        draw_robot(ax, g, state, fixed_world_endpoints)
    fig.suptitle("Yaw/pitch-waist quadruped endpoint geometry", fontsize=14)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_metrics(states: list[FrameState], path: Path) -> None:
    angles = np.array([s.waist_deg for s in states])
    max_reach = np.array([s.metrics["max_reach_xy"] for s in states], dtype=float)
    min_clearance = np.array([s.metrics["min_endpoint_clearance"] for s in states], dtype=float)
    loss = np.array([geometry_loss(s) for s in states], dtype=float)

    fig, axes = plt.subplots(3, 1, figsize=(8.5, 8.4), sharex=True, constrained_layout=True)
    axes[0].plot(angles, max_reach, color="#2f6fbb")
    axes[0].set_ylabel("max reach XY (m)")
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(angles, min_clearance, color="#b15d13")
    axes[1].set_ylabel("min endpoint clearance (m)")
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(angles, loss, color="#8d2d87")
    axes[2].set_ylabel("endpoint loss")
    axes[2].set_xlabel("waist yaw (deg)")
    axes[2].grid(True, alpha=0.25)

    fig.suptitle("Endpoint geometry scan metrics", fontsize=14)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def print_summary(states: list[FrameState], out_dir: Path) -> None:
    worst = max(states, key=geometry_loss)
    best = min(states, key=geometry_loss)
    max_reach_state = max(states, key=lambda s: float(s.metrics["max_reach_xy"]))

    print(f"Wrote outputs to: {out_dir}")
    print(f"Best angle by loss:  {best.waist_deg:+.1f} deg, loss={geometry_loss(best):.3f}")
    print(f"Worst angle by loss: {worst.waist_deg:+.1f} deg, loss={geometry_loss(worst):.3f}")
    print(
        "Largest XY reach:   "
        f"{max_reach_state.metrics['max_reach_xy']:.3f} m "
        f"at {max_reach_state.waist_deg:+.1f} deg"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-angle", type=float, default=35.0, help="scan from -angle to +angle, degrees")
    parser.add_argument("--step", type=float, default=1.0, help="angle step, degrees")
    parser.add_argument("--out-dir", type=Path, default=Path("endpoint_outputs"))
    parser.add_argument("--config", type=Path, default=DEFAULT_DESCRIPTION_PATH, help="dog description YAML")
    parser.add_argument(
        "--feet-mode",
        choices=("fixed-world", "body-frame"),
        default="fixed-world",
        help="fixed-world holds endpoint targets in world XY; body-frame rotates endpoint targets with each body segment",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.step <= 0:
        raise SystemExit("--step must be positive")
    if args.max_angle <= 0:
        raise SystemExit("--max-angle must be positive")

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    description = load_dog_description(args.config)
    geometry = RobotGeometry(**description.geometry.robot_geometry_kwargs())
    fixed_world_endpoints = args.feet_mode == "fixed-world"
    states = scan_angles(geometry, args.max_angle, args.step, fixed_world_endpoints=fixed_world_endpoints)

    write_csv(states, out_dir / "endpoint_scan.csv")
    plot_geometry(states, geometry, out_dir / "endpoint_geometry_scan.png", fixed_world_endpoints=fixed_world_endpoints)
    plot_metrics(states, out_dir / "endpoint_metrics.png")
    print_summary(states, out_dir)


if __name__ == "__main__":
    main()
