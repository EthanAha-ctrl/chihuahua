#!/usr/bin/env python3
"""Interactive pygame linkage viewer for yaw/pitch-waist endpoint geometry.

No solid body rendering. No physics. No contact. No controller.

The drawing is only a constrained linkage diagram:

- rear waist yaw joint
- front waist pitch joint
- short head claw link
- front/rear hip cross links
- hip -> knee -> toe joint -> toe endpoint links
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import numpy as np

try:
    import pygame
except ModuleNotFoundError as exc:
    raise SystemExit("pygame is required. Install dependencies with: uv sync") from exc

from endpoint_geometry import LEG_ORDER, RobotGeometry, waist_joint_points
from dog_description import (
    DEFAULT_DESCRIPTION_PATH,
    DogDescription,
    JointRange,
    load_dog_description,
    save_dog_description,
)


Color = tuple[int, int, int]
Point2 = tuple[int, int]
DOF_KEYS = (
    "waist:yaw",
    "waist:pitch",
    *(f"{leg}:{joint}" for leg in LEG_ORDER for joint in ("hip_ab", "hip_pitch", "knee_bend", "toe_bend")),
    "neck:yaw",
    "neck:pitch",
    "head:claw",
)

# Dog geometry, link lengths, joint ranges, and source notes live in
# dog_description.yaml. The Python code below is only the renderer/loader.


@dataclass
class Camera:
    yaw: float = math.radians(-42.0)
    pitch: float = math.radians(42.0)
    distance: float = 1.25
    target: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.11]))
    focal: float = 720.0


@dataclass
class ViewerState:
    waist_deg: float = 24.0
    waist_pitch_deg: float = 0.0
    waist_babble: bool = True
    joint_babble: bool = True
    dof_solo: bool = False
    dof_index: int = 0
    babble_scale: float = 1.0
    show_help: bool = True
    show_tuning: bool = True
    sim_time: float = 0.0
    camera: Camera = field(default_factory=Camera)


@dataclass(frozen=True)
class DrawCommand:
    depth: float
    kind: str
    points: tuple[Point2, ...]
    color: Color
    width: int = 1


@dataclass
class TuningControl:
    kind: str
    group_name: str
    target_name: str
    field_name: str
    label: str
    original_value: float
    lower: float
    upper: float
    rect: pygame.Rect = field(default_factory=lambda: pygame.Rect(0, 0, 0, 0))


@dataclass
class TuningOverlay:
    controls: list[TuningControl]
    active_index: int | None = None
    panel_rect: pygame.Rect = field(default_factory=lambda: pygame.Rect(0, 0, 0, 0))
    save_button: pygame.Rect = field(default_factory=lambda: pygame.Rect(0, 0, 0, 0))
    status: str = ""
    dirty: bool = False


@dataclass(frozen=True)
class BodyPose:
    yaw_joint: np.ndarray
    pitch_joint: np.ndarray
    front_mid: np.ndarray
    rear_mid: np.ndarray
    hips: dict[str, np.ndarray]
    bases: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]]


def v3(x: float, y: float, z: float = 0.0) -> np.ndarray:
    return np.array([x, y, z], dtype=float)


def rot_z(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def rotate_about_axis(vector: np.ndarray, axis: np.ndarray, theta: float) -> np.ndarray:
    axis = normalized(axis)
    c = math.cos(theta)
    s = math.sin(theta)
    return vector * c + np.cross(axis, vector) * s + axis * float(np.dot(axis, vector)) * (1.0 - c)


def stable_seed(key: str) -> int:
    return sum((i + 1) * ord(ch) for i, ch in enumerate(key))


def active_dof_key(viewer: ViewerState) -> str:
    return DOF_KEYS[viewer.dof_index % len(DOF_KEYS)]


def periodic_signal(viewer: ViewerState, key: str, amplitude: float, bias: float = 0.0) -> float:
    if viewer.dof_solo:
        if key != active_dof_key(viewer):
            return bias
        phase = 2.0 * math.pi * 0.42 * viewer.sim_time
        return bias + amplitude * viewer.babble_scale * math.sin(phase)

    if not viewer.joint_babble and not key.startswith("waist:"):
        return bias

    seed = stable_seed(key)
    freq = 0.35 + (seed % 89) * 0.010
    phase = (seed % 628) * 0.01
    primary = math.sin(2.0 * math.pi * freq * viewer.sim_time + phase)
    secondary = 0.25 * math.sin(2.0 * math.pi * freq * 0.43 * viewer.sim_time + phase * 1.59)
    return bias + amplitude * viewer.babble_scale * (primary + secondary)


def constrained_angle(
    viewer: ViewerState,
    key: str,
    base_deg: float,
    amp_deg: float,
    min_deg: float,
    max_deg: float,
) -> float:
    raw = periodic_signal(viewer, key, amp_deg, base_deg)
    return math.radians(max(min_deg, min(max_deg, raw)))


def ranged_joint_angle(viewer: ViewerState, key: str, spec: JointRange) -> float:
    return constrained_angle(viewer, key, spec.bias_deg, spec.amp_deg, spec.min_deg, spec.max_deg)


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def camera_basis(camera: Camera) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    cp = math.cos(camera.pitch)
    sp = math.sin(camera.pitch)
    cy = math.cos(camera.yaw)
    sy = math.sin(camera.yaw)
    camera_pos = camera.target + camera.distance * np.array([cp * cy, cp * sy, sp])

    forward = camera.target - camera_pos
    forward = forward / np.linalg.norm(forward)
    right = np.array([-sy, cy, 0.0])
    up = np.cross(right, forward)
    up = up / np.linalg.norm(up)
    return camera_pos, right, up, forward


def project(point: np.ndarray, camera: Camera, screen_size: tuple[int, int]) -> tuple[Point2, float] | None:
    camera_pos, right, up, forward = camera_basis(camera)
    rel = point - camera_pos
    x = float(np.dot(rel, right))
    y = float(np.dot(rel, up))
    z = float(np.dot(rel, forward))
    if z <= 0.02:
        return None
    width, height = screen_size
    scale = camera.focal / z
    return (int(width * 0.5 + x * scale), int(height * 0.57 - y * scale)), z


def project_points(
    points: Iterable[np.ndarray], camera: Camera, screen_size: tuple[int, int]
) -> tuple[tuple[Point2, ...], float] | None:
    projected: list[Point2] = []
    depths: list[float] = []
    for point in points:
        out = project(point, camera, screen_size)
        if out is None:
            return None
        point2, depth = out
        projected.append(point2)
        depths.append(depth)
    return tuple(projected), float(np.mean(depths))


def add_line(
    commands: list[DrawCommand],
    camera: Camera,
    screen_size: tuple[int, int],
    a: np.ndarray,
    b: np.ndarray,
    color: Color,
    width: int = 2,
) -> None:
    out = project_points((a, b), camera, screen_size)
    if out is None:
        return
    points, depth = out
    commands.append(DrawCommand(depth, "line", points, color, width))


def add_joint(
    commands: list[DrawCommand],
    camera: Camera,
    screen_size: tuple[int, int],
    point3: np.ndarray,
    radius: int,
    color: Color,
) -> None:
    out = project(point3, camera, screen_size)
    if out is None:
        return
    point2, depth = out
    commands.append(DrawCommand(depth, "circle", (point2,), color, radius))


def draw_grid(commands: list[DrawCommand], camera: Camera, screen_size: tuple[int, int]) -> None:
    for i in range(-6, 7):
        u = i * 0.10
        add_line(commands, camera, screen_size, v3(-0.60, u, 0.0), v3(0.60, u, 0.0), (44, 51, 59), 1)
        add_line(commands, camera, screen_size, v3(u, -0.45, 0.0), v3(u, 0.45, 0.0), (44, 51, 59), 1)
    add_line(commands, camera, screen_size, v3(-0.65, 0.0, 0.0), v3(0.65, 0.0, 0.0), (105, 118, 134), 2)
    add_line(commands, camera, screen_size, v3(0.0, -0.50, 0.0), v3(0.0, 0.50, 0.0), (105, 118, 134), 2)


def normalized(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        return vector
    return vector / norm


def make_body_pose(g: RobotGeometry, viewer: ViewerState, description: DogDescription) -> BodyPose:
    waist_yaw_rad = math.radians(viewer.waist_deg)
    waist_pitch_rad = math.radians(viewer.waist_pitch_deg)
    yaw_xy, pitch_xy = waist_joint_points(g, waist_yaw_rad)
    body_z = description.viewer.body_z_m
    yaw_joint = v3(float(yaw_xy[0]), float(yaw_xy[1]), body_z)
    pitch_joint = v3(float(pitch_xy[0]), float(pitch_xy[1]), body_z)

    rear_yaw = rot_z(0.0)
    front_yaw = rot_z(waist_yaw_rad)
    rear_forward = rear_yaw @ np.array([1.0, 0.0, 0.0])
    rear_left = rear_yaw @ np.array([0.0, 1.0, 0.0])
    rear_up = np.array([0.0, 0.0, 1.0])

    front_forward_level = front_yaw @ np.array([1.0, 0.0, 0.0])
    front_left = front_yaw @ np.array([0.0, 1.0, 0.0])
    world_up = np.array([0.0, 0.0, 1.0])
    front_forward = normalized(
        front_forward_level * math.cos(waist_pitch_rad) + world_up * math.sin(waist_pitch_rad)
    )
    front_up = normalized(np.cross(front_forward, front_left))

    front_mid = pitch_joint + front_forward * g.front_body_length
    rear_mid = yaw_joint - rear_forward * g.rear_body_length

    hips: dict[str, np.ndarray] = {}
    bases: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
    for name in LEG_ORDER:
        side = 1.0 if name.endswith("left") else -1.0
        if name.startswith("front"):
            outward = side * front_left
            hips[name] = front_mid + outward * g.hip_half_width
            bases[name] = (front_forward, outward, -front_up)
        else:
            outward = side * rear_left
            hips[name] = rear_mid + outward * g.hip_half_width
            bases[name] = (rear_forward, outward, -rear_up)

    return BodyPose(
        yaw_joint=yaw_joint,
        pitch_joint=pitch_joint,
        front_mid=front_mid,
        rear_mid=rear_mid,
        hips=hips,
        bases=bases,
    )


def leg_chain(
    viewer: ViewerState,
    description: DogDescription,
    name: str,
    hip: np.ndarray,
    basis: tuple[np.ndarray, np.ndarray, np.ndarray],
) -> dict[str, np.ndarray]:
    forward, outward, down = basis
    links = description.viewer.links
    ranges = description.joint_ranges

    hip_ab = ranged_joint_angle(viewer, f"{name}:hip_ab", ranges["hip_ab"])
    hip_pitch = ranged_joint_angle(viewer, f"{name}:hip_pitch", ranges["hip_pitch"])
    knee_bend = ranged_joint_angle(viewer, f"{name}:knee_bend", ranges["knee_bend"])
    toe_bend = ranged_joint_angle(viewer, f"{name}:toe_bend", ranges["toe_bend"])

    hip_ab_axis = normalized(np.cross(down, outward))
    leg_pitch_axis = normalized(-outward)

    upper_dir = rotate_about_axis(down, hip_ab_axis, hip_ab)
    upper_dir = normalized(rotate_about_axis(upper_dir, leg_pitch_axis, hip_pitch))
    upper = upper_dir * links.upper_m
    knee = hip + upper

    lower_dir = normalized(rotate_about_axis(upper_dir, leg_pitch_axis, knee_bend))
    lower = lower_dir * links.lower_m
    toe_joint = knee + lower

    toe_dir = normalized(rotate_about_axis(lower_dir, leg_pitch_axis, toe_bend))
    toe = toe_dir * links.distal_endpoint_m
    toe_endpoint = toe_joint + toe

    return {
        "hip": hip,
        "knee": knee,
        "toe_joint": toe_joint,
        "toe_endpoint": toe_endpoint,
    }


def head_claw_chain(
    viewer: ViewerState,
    description: DogDescription,
    front_anchor: np.ndarray,
    front_forward: np.ndarray,
    front_left: np.ndarray,
    front_up: np.ndarray,
) -> dict[str, np.ndarray]:
    base_forward = front_forward
    base_left = front_left
    up = front_up
    ranges = description.joint_ranges
    claw = description.viewer.head_claw

    neck_yaw = ranged_joint_angle(viewer, "neck:yaw", ranges["neck_yaw"])
    neck_pitch = ranged_joint_angle(viewer, "neck:pitch", ranges["neck_pitch"])
    claw_open = ranged_joint_angle(viewer, "head:claw", ranges["head_claw"])

    yaw_forward = base_forward * math.cos(neck_yaw) + base_left * math.sin(neck_yaw)
    yaw_left = -base_forward * math.sin(neck_yaw) + base_left * math.cos(neck_yaw)
    forward = yaw_forward * math.cos(neck_pitch) + up * math.sin(neck_pitch)
    forward = forward / np.linalg.norm(forward)
    left = yaw_left / np.linalg.norm(yaw_left)
    local_up = np.cross(forward, left)
    local_up = local_up / np.linalg.norm(local_up)

    root = front_anchor + base_forward * claw.root_forward_m + up * claw.root_up_m
    hinge = root + forward * claw.neck_length_m
    upper_hinge = hinge + local_up * claw.hinge_half_gap_m
    lower_hinge = hinge - local_up * claw.hinge_half_gap_m
    jaw_len = claw.jaw_length_m
    upper_tip = upper_hinge + jaw_len * (forward * math.cos(claw_open) + local_up * math.sin(claw_open))
    lower_tip = lower_hinge + jaw_len * (forward * math.cos(claw_open) - local_up * math.sin(claw_open))

    return {
        "root": root,
        "hinge": hinge,
        "upper_hinge": upper_hinge,
        "lower_hinge": lower_hinge,
        "upper_tip": upper_tip,
        "lower_tip": lower_tip,
    }


def add_head_claw(
    commands: list[DrawCommand],
    viewer: ViewerState,
    description: DogDescription,
    front_anchor: np.ndarray,
    front_forward: np.ndarray,
    front_left: np.ndarray,
    front_up: np.ndarray,
    screen_size: tuple[int, int],
) -> None:
    chain = head_claw_chain(viewer, description, front_anchor, front_forward, front_left, front_up)
    root = chain["root"]
    hinge = chain["hinge"]
    upper_hinge = chain["upper_hinge"]
    lower_hinge = chain["lower_hinge"]
    upper_tip = chain["upper_tip"]
    lower_tip = chain["lower_tip"]

    color = (198, 224, 255)
    jaw_color = (255, 215, 118)
    add_line(commands, viewer.camera, screen_size, root, hinge, color, 4)
    add_line(commands, viewer.camera, screen_size, upper_hinge, lower_hinge, color, 3)
    add_line(commands, viewer.camera, screen_size, upper_hinge, upper_tip, jaw_color, 4)
    add_line(commands, viewer.camera, screen_size, lower_hinge, lower_tip, jaw_color, 4)
    add_joint(commands, viewer.camera, screen_size, root, 4, color)
    add_joint(commands, viewer.camera, screen_size, upper_hinge, 4, jaw_color)
    add_joint(commands, viewer.camera, screen_size, lower_hinge, 4, jaw_color)


def add_linkage_scene(
    commands: list[DrawCommand],
    g: RobotGeometry,
    viewer: ViewerState,
    description: DogDescription,
    screen_size: tuple[int, int],
) -> dict[str, float]:
    draw_grid(commands, viewer.camera, screen_size)
    pose = make_body_pose(g, viewer, description)
    hips = pose.hips

    add_line(commands, viewer.camera, screen_size, pose.yaw_joint, pose.pitch_joint, (238, 241, 245), 5)
    add_line(commands, viewer.camera, screen_size, pose.pitch_joint, pose.front_mid, (116, 170, 231), 4)
    add_line(commands, viewer.camera, screen_size, pose.yaw_joint, pose.rear_mid, (236, 176, 103), 4)
    add_line(commands, viewer.camera, screen_size, hips["front_left"], hips["front_right"], (116, 170, 231), 4)
    add_line(commands, viewer.camera, screen_size, hips["rear_left"], hips["rear_right"], (236, 176, 103), 4)
    add_joint(commands, viewer.camera, screen_size, pose.yaw_joint, 7, (236, 176, 103))
    add_joint(commands, viewer.camera, screen_size, pose.pitch_joint, 7, (116, 170, 231))
    front_forward, front_outward, front_down = pose.bases["front_left"]
    front_left = front_outward
    front_up = -front_down
    add_head_claw(
        commands,
        viewer,
        description,
        pose.front_mid,
        front_forward,
        front_left,
        front_up,
        screen_size,
    )

    toe_endpoints: list[np.ndarray] = []
    for name in LEG_ORDER:
        chain = leg_chain(viewer, description, name, hips[name], pose.bases[name])
        leg_color = (202, 218, 232) if name.startswith("front") else (226, 209, 187)
        toe_color = (245, 209, 83)

        add_line(commands, viewer.camera, screen_size, chain["hip"], chain["knee"], leg_color, 4)
        add_line(commands, viewer.camera, screen_size, chain["knee"], chain["toe_joint"], leg_color, 4)
        add_line(commands, viewer.camera, screen_size, chain["toe_joint"], chain["toe_endpoint"], toe_color, 4)

        add_joint(commands, viewer.camera, screen_size, chain["hip"], 5, (236, 240, 244))
        add_joint(commands, viewer.camera, screen_size, chain["knee"], 5, (168, 184, 198))
        add_joint(commands, viewer.camera, screen_size, chain["toe_joint"], 5, (245, 209, 83))
        add_joint(commands, viewer.camera, screen_size, chain["toe_endpoint"], 4, (255, 232, 128))
        toe_endpoints.append(chain["toe_endpoint"])

    reaches = [float(np.linalg.norm(p[:2] - hips[name][:2])) for p, name in zip(toe_endpoints, LEG_ORDER)]
    return {
        "max_reach_xy": max(reaches),
        "mean_reach_xy": float(np.mean(reaches)),
        "max_toe_z": max(float(p[2]) for p in toe_endpoints),
        "min_toe_z": min(float(p[2]) for p in toe_endpoints),
    }


def make_scene_commands(
    g: RobotGeometry,
    viewer: ViewerState,
    description: DogDescription,
    screen_size: tuple[int, int],
) -> tuple[list[DrawCommand], dict[str, float]]:
    commands: list[DrawCommand] = []
    metrics = add_linkage_scene(commands, g, viewer, description, screen_size)
    return commands, metrics


def render_commands(screen: pygame.Surface, commands: list[DrawCommand]) -> None:
    for command in sorted(commands, key=lambda c: c.depth, reverse=True):
        if command.kind == "line":
            pygame.draw.line(screen, command.color, command.points[0], command.points[1], command.width)
        elif command.kind == "circle":
            pygame.draw.circle(screen, command.color, command.points[0], command.width)


def slider_bounds(original_value: float, zero_span: float = 1.0) -> tuple[float, float]:
    if abs(original_value) < 1e-9:
        return -zero_span, zero_span
    a = original_value * 0.5
    b = original_value * 1.5
    return min(a, b), max(a, b)


def add_scalar_tuning_control(
    controls: list[TuningControl],
    group_name: str,
    target_name: str,
    label: str,
    original_value: float,
    enabled: bool,
) -> None:
    if not enabled:
        return
    lower, upper = slider_bounds(original_value, zero_span=0.05)
    if group_name == "geometry" and target_name == "front_body_fraction":
        lower = max(0.0, lower)
        upper = min(1.0, upper)
    controls.append(
        TuningControl(
            kind="scalar",
            group_name=group_name,
            target_name=target_name,
            field_name="value",
            label=label,
            original_value=original_value,
            lower=lower,
            upper=upper,
        )
    )


def build_tuning_overlay(description: DogDescription) -> TuningOverlay:
    controls: list[TuningControl] = []
    for key, enabled in description.geometry.visual_tuning.items():
        add_scalar_tuning_control(
            controls,
            group_name="geometry",
            target_name=key,
            label=key,
            original_value=float(getattr(description.geometry, key)),
            enabled=enabled,
        )

    add_scalar_tuning_control(
        controls,
        group_name="viewer",
        target_name="body_z_m",
        label="body_z_m",
        original_value=description.viewer.body_z_m,
        enabled=description.viewer.visual_tuning.get("body_z_m", False),
    )

    for attr_name, yaml_name in (
        ("upper_m", "upper"),
        ("lower_m", "lower"),
        ("distal_endpoint_m", "distal_endpoint"),
    ):
        add_scalar_tuning_control(
            controls,
            group_name="viewer.links_m",
            target_name=attr_name,
            label=yaml_name,
            original_value=float(getattr(description.viewer.links, attr_name)),
            enabled=description.viewer.links.visual_tuning.get(attr_name, False),
        )

    for key, enabled in description.viewer.head_claw.visual_tuning.items():
        add_scalar_tuning_control(
            controls,
            group_name="viewer.head_claw",
            target_name=key,
            label=key,
            original_value=float(getattr(description.viewer.head_claw, key)),
            enabled=enabled,
        )

    for joint_name, spec in description.joint_ranges.items():
        if not spec.visual_tuning:
            continue
        for field_name in ("min_deg", "max_deg"):
            original_value = float(getattr(spec, field_name))
            lower, upper = slider_bounds(original_value, zero_span=1.0)
            controls.append(
                TuningControl(
                    kind="range",
                    group_name="joint_ranges_deg",
                    target_name=joint_name,
                    field_name=field_name,
                    label="min" if field_name == "min_deg" else "max",
                    original_value=original_value,
                    lower=lower,
                    upper=upper,
                )
            )
    return TuningOverlay(controls=controls)


def layout_tuning_overlay(overlay: TuningOverlay, screen_size: tuple[int, int]) -> None:
    width, height = screen_size
    panel_width = min(760, max(520, width - 32))
    panel_x = max(16, width - panel_width - 20)
    panel_y = 20
    y = panel_y + 76
    last_group = ""
    slider_left = panel_x + min(420, panel_width - 220)

    for control in overlay.controls:
        group_label = control.target_name if control.kind == "range" else control.group_name
        if group_label != last_group:
            y += 34
            last_group = group_label
        control.rect = pygame.Rect(slider_left, y + 12, panel_x + panel_width - slider_left - 42, 12)
        y += 42

    panel_height = max(170, min(height - 40, y - panel_y + 70))
    overlay.panel_rect = pygame.Rect(panel_x, panel_y, panel_width, panel_height)
    overlay.save_button = pygame.Rect(panel_x + panel_width - 112, panel_y + 18, 88, 38)


def slider_value_from_x(control: TuningControl, mouse_x: int) -> float:
    if control.rect.width <= 0 or abs(control.upper - control.lower) < 1e-12:
        return control.lower
    t = (mouse_x - control.rect.left) / control.rect.width
    t = max(0.0, min(1.0, t))
    return control.lower + t * (control.upper - control.lower)


def slider_x_from_value(control: TuningControl, value: float) -> int:
    if abs(control.upper - control.lower) < 1e-12:
        return control.rect.left
    t = (value - control.lower) / (control.upper - control.lower)
    t = max(0.0, min(1.0, t))
    return int(control.rect.left + t * control.rect.width)


def tuning_control_value(description: DogDescription, control: TuningControl) -> float:
    if control.kind == "range":
        return float(getattr(description.joint_ranges[control.target_name], control.field_name))
    if control.group_name == "geometry":
        return float(getattr(description.geometry, control.target_name))
    if control.group_name == "viewer":
        return float(getattr(description.viewer, control.target_name))
    if control.group_name == "viewer.links_m":
        return float(getattr(description.viewer.links, control.target_name))
    if control.group_name == "viewer.head_claw":
        return float(getattr(description.viewer.head_claw, control.target_name))
    raise ValueError(f"Unknown tuning control group: {control.group_name}")


def normalized_scalar_tuning_value(description: DogDescription, control: TuningControl, value: float) -> float:
    if control.group_name == "geometry" and control.target_name == "front_body_fraction":
        return max(0.0, min(1.0, value))
    if control.group_name == "geometry" and control.target_name == "body_length_total_m":
        return max(description.geometry.waist_joint_spacing_m + 1e-6, value)
    if control.group_name == "geometry" and control.target_name == "waist_joint_spacing_m":
        return max(1e-6, min(description.geometry.body_length_total_m - 1e-6, value))
    return value


def set_scalar_tuning_value(description: DogDescription, control: TuningControl, value: float) -> None:
    value = normalized_scalar_tuning_value(description, control, value)
    if control.group_name == "geometry":
        setattr(description.geometry, control.target_name, value)
    elif control.group_name == "viewer":
        setattr(description.viewer, control.target_name, value)
    elif control.group_name == "viewer.links_m":
        setattr(description.viewer.links, control.target_name, value)
    elif control.group_name == "viewer.head_claw":
        setattr(description.viewer.head_claw, control.target_name, value)
    else:
        raise ValueError(f"Unknown tuning control group: {control.group_name}")


def set_tuned_control_value(
    description: DogDescription,
    viewer: ViewerState,
    overlay: TuningOverlay,
    control: TuningControl,
    value: float,
) -> None:
    if control.kind == "scalar":
        set_scalar_tuning_value(description, control, value)
        viewer.sim_time = 0.0
        overlay.status = "unsaved changes"
        overlay.dirty = True
        return

    spec = description.joint_ranges[control.target_name]
    min_deg = spec.min_deg
    max_deg = spec.max_deg
    if control.field_name == "min_deg":
        min_deg = min(value, max_deg)
    else:
        max_deg = max(value, min_deg)

    bias_deg = 0.5 * (min_deg + max_deg)
    amp_deg = 0.5 * max(0.0, max_deg - min_deg)
    description.joint_ranges[control.target_name] = JointRange(
        visual_tuning=spec.visual_tuning,
        bias_deg=bias_deg,
        amp_deg=amp_deg,
        min_deg=min_deg,
        max_deg=max_deg,
    )
    viewer.sim_time = 0.0
    overlay.status = "unsaved changes"
    overlay.dirty = True


def save_tuning_overlay(path: Path, description: DogDescription, overlay: TuningOverlay) -> None:
    try:
        save_dog_description(path, description)
    except OSError as exc:
        overlay.status = f"save failed: {exc}"
        return
    overlay.status = f"saved: {path.name}"
    overlay.dirty = False


def handle_tuning_overlay_event(
    event: pygame.event.Event,
    viewer: ViewerState,
    overlay: TuningOverlay,
    description: DogDescription,
    config_path: Path,
) -> bool:
    if not viewer.show_tuning:
        return False

    if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
        if not overlay.panel_rect.collidepoint(event.pos):
            return False
        if overlay.controls and overlay.save_button.collidepoint(event.pos):
            save_tuning_overlay(config_path, description, overlay)
            return True
        for i, control in enumerate(overlay.controls):
            if control.rect.inflate(0, 18).collidepoint(event.pos):
                overlay.active_index = i
                set_tuned_control_value(description, viewer, overlay, control, slider_value_from_x(control, event.pos[0]))
                return True
        return True

    if event.type == pygame.MOUSEMOTION and overlay.active_index is not None:
        control = overlay.controls[overlay.active_index]
        set_tuned_control_value(description, viewer, overlay, control, slider_value_from_x(control, event.pos[0]))
        return True

    if event.type == pygame.MOUSEBUTTONUP and event.button == 1 and overlay.active_index is not None:
        overlay.active_index = None
        return True

    return False


def draw_tuning_overlay(
    screen: pygame.Surface,
    font: pygame.font.Font,
    small_font: pygame.font.Font,
    viewer: ViewerState,
    overlay: TuningOverlay,
    description: DogDescription,
) -> None:
    if not viewer.show_tuning:
        return

    panel = overlay.panel_rect
    pygame.draw.rect(screen, (19, 23, 29), panel, border_radius=8)
    pygame.draw.rect(screen, (74, 84, 97), panel, width=1, border_radius=8)

    title = font.render("visual tuning", True, (238, 241, 245))
    screen.blit(title, (panel.x + 20, panel.y + 22))

    save_color = (69, 125, 89) if overlay.controls else (50, 55, 62)
    if overlay.dirty:
        save_color = (112, 91, 53)
    pygame.draw.rect(screen, save_color, overlay.save_button, border_radius=6)
    pygame.draw.rect(screen, (128, 141, 155), overlay.save_button, width=1, border_radius=6)
    save_label = small_font.render("Save", True, (238, 241, 245) if overlay.controls else (130, 139, 150))
    screen.blit(save_label, save_label.get_rect(center=overlay.save_button.center))

    if not overlay.controls:
        msg1 = small_font.render("no visual_tuning: true items", True, (168, 178, 190))
        msg2 = small_font.render("enable items in dog_description.yaml", True, (128, 139, 152))
        screen.blit(msg1, (panel.x + 20, panel.y + 86))
        screen.blit(msg2, (panel.x + 20, panel.y + 116))
        return

    last_group = ""
    for control in overlay.controls:
        value = tuning_control_value(description, control)
        group_label = control.target_name if control.kind == "range" else control.group_name
        if group_label != last_group:
            header = small_font.render(group_label, True, (218, 225, 233))
            screen.blit(header, (panel.x + 20, control.rect.y - 25))
            last_group = group_label

        label = small_font.render(f"{control.label} {value:7.3f}", True, (178, 188, 200))
        screen.blit(label, (panel.x + 38, control.rect.y - 8))

        pygame.draw.rect(screen, (51, 59, 69), control.rect, border_radius=4)
        knob_x = slider_x_from_value(control, value)
        active = overlay.active_index is not None and overlay.controls[overlay.active_index] is control
        knob_color = (255, 217, 120) if active else (196, 216, 239)
        pygame.draw.circle(screen, knob_color, (knob_x, control.rect.centery), 11)

    status = overlay.status or "drag bars"
    status_color = (215, 199, 142) if overlay.dirty else (139, 151, 164)
    status_surface = small_font.render(status, True, status_color)
    screen.blit(status_surface, (panel.x + 20, panel.bottom - 38))


def draw_text_panel(
    screen: pygame.Surface,
    font: pygame.font.Font,
    small_font: pygame.font.Font,
    viewer: ViewerState,
    metrics: dict[str, float],
    fps: float,
) -> None:
    panel_width = min(screen.get_width() - 32, 1080)
    panel = pygame.Rect(16, 16, panel_width, 390 if viewer.show_help else 176)
    pygame.draw.rect(screen, (22, 27, 33), panel, border_radius=8)
    pygame.draw.rect(screen, (70, 79, 91), panel, width=1, border_radius=8)

    lines = [
        (
            f"linkage only | waist yaw {viewer.waist_deg:+.1f} deg "
            f"| pitch {viewer.waist_pitch_deg:+.1f} deg | fps {fps:4.1f}"
        ),
        f"joint babble {viewer.babble_scale:.2f} | toe joint + short head claw",
        f"max XY toe reach {metrics['max_reach_xy']:.3f} m | toe z {metrics['min_toe_z']:.3f}..{metrics['max_toe_z']:.3f} m",
        "no solid bodies, no physics/contact/control",
    ]
    if viewer.dof_solo:
        lines.append(f"DOF solo {viewer.dof_index % len(DOF_KEYS) + 1:02d}/{len(DOF_KEYS)} | {active_dof_key(viewer)}")

    y = panel.y + 20
    for i, line in enumerate(lines):
        surface = font.render(line, True, (238, 241, 245) if i == 0 else (207, 214, 222))
        screen.blit(surface, (panel.x + 20, y))
        y += 34

    if viewer.show_help:
        y += 12
        help_lines = [
            "left/right or A/D: waist yaw    up/down or W/S: waist pitch",
            "space: waist joint babble",
            "M: 21-DOF solo    N: next solo DOF",
            "B: joint babble    ,/.: babble amplitude",
            "left-drag mouse: orbit camera    wheel: zoom    T: tuning overlay",
            "P: screenshot    ?: help    Esc: quit",
        ]
        for line in help_lines:
            surface = small_font.render(line, True, (158, 169, 181))
            screen.blit(surface, (panel.x + 20, y))
            y += 30


def clamp_joint_degrees(value: float, spec: JointRange) -> float:
    return max(spec.min_deg, min(spec.max_deg, value))


def update_autoplay(viewer: ViewerState, description: DogDescription) -> None:
    if viewer.waist_babble or viewer.dof_solo:
        viewer.waist_deg = math.degrees(ranged_joint_angle(viewer, "waist:yaw", description.joint_ranges["waist_yaw"]))
        viewer.waist_pitch_deg = math.degrees(
            ranged_joint_angle(viewer, "waist:pitch", description.joint_ranges["waist_pitch"])
        )


def handle_key(event: pygame.event.Event, viewer: ViewerState, description: DogDescription) -> bool:
    yaw_spec = description.joint_ranges["waist_yaw"]
    pitch_spec = description.joint_ranges["waist_pitch"]
    if event.key in (pygame.K_ESCAPE, pygame.K_q):
        return False
    if event.key in (pygame.K_LEFT, pygame.K_a):
        viewer.waist_babble = False
        viewer.waist_deg = clamp_joint_degrees(viewer.waist_deg - 2.0, yaw_spec)
    elif event.key in (pygame.K_RIGHT, pygame.K_d):
        viewer.waist_babble = False
        viewer.waist_deg = clamp_joint_degrees(viewer.waist_deg + 2.0, yaw_spec)
    elif event.key in (pygame.K_UP, pygame.K_w):
        viewer.waist_babble = False
        viewer.waist_pitch_deg = clamp_joint_degrees(viewer.waist_pitch_deg + 2.0, pitch_spec)
    elif event.key in (pygame.K_DOWN, pygame.K_s):
        viewer.waist_babble = False
        viewer.waist_pitch_deg = clamp_joint_degrees(viewer.waist_pitch_deg - 2.0, pitch_spec)
    elif event.key == pygame.K_SPACE:
        viewer.waist_babble = not viewer.waist_babble
    elif event.key == pygame.K_b:
        viewer.joint_babble = not viewer.joint_babble
    elif event.key == pygame.K_m:
        viewer.dof_solo = not viewer.dof_solo
        viewer.sim_time = 0.0
    elif event.key == pygame.K_n:
        viewer.dof_solo = True
        viewer.dof_index = (viewer.dof_index + 1) % len(DOF_KEYS)
        viewer.sim_time = 0.0
    elif event.key == pygame.K_t:
        viewer.show_tuning = not viewer.show_tuning
    elif event.key == pygame.K_COMMA:
        viewer.babble_scale = max(0.0, viewer.babble_scale - 0.10)
    elif event.key == pygame.K_PERIOD:
        viewer.babble_scale = min(2.0, viewer.babble_scale + 0.10)
    elif event.key in (pygame.K_SLASH, pygame.K_QUESTION):
        viewer.show_help = not viewer.show_help
    return True


def save_screenshot(screen: pygame.Surface, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pygame.image.save(screen, path)
    print(f"saved screenshot: {path}")


def run_viewer(args: argparse.Namespace) -> None:
    if args.headless:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

    pygame.init()
    pygame.display.set_caption("Yaw/pitch-waist constrained linkage viewer")
    screen = pygame.display.set_mode((args.width, args.height))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Menlo,Consolas,monospace", 24)
    small_font = pygame.font.SysFont("Menlo,Consolas,monospace", 20)

    description = load_dog_description(args.config)
    tuning_overlay = build_tuning_overlay(description)
    viewer = ViewerState(
        waist_deg=args.waist,
        waist_pitch_deg=args.waist_pitch,
        waist_babble=not args.paused,
        joint_babble=not args.no_babble,
        dof_solo=args.dof_solo,
        dof_index=args.dof_index % len(DOF_KEYS),
        babble_scale=args.babble_scale,
    )
    dragging = False
    last_mouse = (0, 0)
    running = True
    frame = 0
    screenshot_path = args.screenshot

    while running:
        dt = clock.tick(60) / 1000.0
        viewer.sim_time += dt
        layout_tuning_overlay(tuning_overlay, screen.get_size())

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_p:
                    screenshot_path = args.out_dir / "pygame_viewer_screenshot.png"
                else:
                    running = handle_key(event, viewer, description)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if handle_tuning_overlay_event(event, viewer, tuning_overlay, description, args.config):
                    continue
                if event.button == 1:
                    dragging = True
                    last_mouse = event.pos
                elif event.button == 4:
                    viewer.camera.distance = max(0.75, viewer.camera.distance * 0.92)
                elif event.button == 5:
                    viewer.camera.distance = min(3.00, viewer.camera.distance * 1.08)
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                if handle_tuning_overlay_event(event, viewer, tuning_overlay, description, args.config):
                    continue
                dragging = False
            elif event.type == pygame.MOUSEMOTION and dragging:
                if handle_tuning_overlay_event(event, viewer, tuning_overlay, description, args.config):
                    continue
                dx = event.pos[0] - last_mouse[0]
                dy = event.pos[1] - last_mouse[1]
                viewer.camera.yaw = wrap_angle(viewer.camera.yaw - dx * 0.008)
                viewer.camera.pitch = wrap_angle(viewer.camera.pitch - dy * 0.006)
                last_mouse = event.pos
            elif event.type == pygame.MOUSEMOTION:
                handle_tuning_overlay_event(event, viewer, tuning_overlay, description, args.config)

        update_autoplay(viewer, description)
        viewer.waist_deg = clamp_joint_degrees(viewer.waist_deg, description.joint_ranges["waist_yaw"])
        viewer.waist_pitch_deg = clamp_joint_degrees(
            viewer.waist_pitch_deg, description.joint_ranges["waist_pitch"]
        )

        screen.fill((12, 15, 19))
        geometry = RobotGeometry(**description.geometry.robot_geometry_kwargs())
        commands, metrics = make_scene_commands(geometry, viewer, description, screen.get_size())
        render_commands(screen, commands)
        draw_text_panel(screen, font, small_font, viewer, metrics, clock.get_fps())
        draw_tuning_overlay(screen, font, small_font, viewer, tuning_overlay, description)
        pygame.display.flip()

        if screenshot_path is not None:
            save_screenshot(screen, screenshot_path)
            screenshot_path = None

        frame += 1
        if args.headless and frame >= args.frames:
            running = False

    pygame.quit()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1440)
    parser.add_argument("--waist", type=float, default=24.0)
    parser.add_argument("--waist-pitch", type=float, default=0.0)
    parser.add_argument("--paused", action="store_true", help="start with waist joint babble paused")
    parser.add_argument("--no-babble", action="store_true", help="freeze constrained joint babble")
    parser.add_argument("--dof-solo", action="store_true", help="animate one of the 21 DOFs at a time")
    parser.add_argument("--dof-index", type=int, default=0, help="zero-based DOF index for --dof-solo")
    parser.add_argument("--babble-scale", type=float, default=1.0, help="scale for joint oscillation")
    parser.add_argument("--headless", action="store_true", help="run with SDL dummy video driver")
    parser.add_argument("--frames", type=int, default=3, help="frames to render in headless mode")
    parser.add_argument("--screenshot", type=Path, default=None, help="save one screenshot and keep running unless headless")
    parser.add_argument("--out-dir", type=Path, default=Path("endpoint_outputs"))
    parser.add_argument("--config", type=Path, default=DEFAULT_DESCRIPTION_PATH, help="dog description YAML")
    return parser.parse_args()


def main() -> None:
    run_viewer(parse_args())


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
