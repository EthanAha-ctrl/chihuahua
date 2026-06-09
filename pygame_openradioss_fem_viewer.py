#!/usr/bin/env python3
"""Interactive pygame viewer for Stage 2 OpenRadioss whole-body FEM results."""

from __future__ import annotations

import argparse
import csv
import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from matplotlib import colormaps

try:
    import pygame
except ModuleNotFoundError as exc:
    raise SystemExit("pygame is required. Install dependencies with: uv sync") from exc

from dog_description import DEFAULT_DESCRIPTION_PATH, load_dog_description
import mass_model as stage1
import stage2_openradioss_periodic_motion as fem


RUN_DIR_DEFAULT = Path("/mnt/s8t/openradioss/runs/chihuahua_stage2_stage1_torque_replay_radius12_8mm")
CMAP = colormaps["seismic"]
Point2 = tuple[int, int]
Color = tuple[int, int, int]


@dataclass
class Camera:
    yaw: float = math.radians(-42.0)
    pitch: float = math.radians(34.0)
    distance: float = 1.25
    target: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.16], dtype=float))
    focal: float = 620.0


@dataclass
class DrawCommand:
    depth: float
    kind: str
    points: tuple[Point2, ...]
    color: Color
    width: int = 1
    label: str = ""


@dataclass
class HoverItem:
    label: str
    point: Point2
    depth: float
    distance_px: float


@dataclass
class ViewerState:
    camera: Camera
    frame_idx: int = 0
    paused: bool = False
    playback_speed: float = 1.0
    accumulator: float = 0.0
    show_help: bool = True
    show_mesh: bool = True
    show_nodes: bool = True
    show_solver_nodes: bool = False
    show_grid: bool = True
    show_torque: bool = True
    hover: HoverItem | None = None


@dataclass(frozen=True)
class TorqueOverlaySample:
    joint: str
    proximal_node: str
    distal_node: str
    torque_nm: float
    axis: np.ndarray


@dataclass(frozen=True)
class TorqueOverlaySeries:
    joint: str
    proximal_node: str
    distal_node: str
    time_ms: np.ndarray
    torque_nm: np.ndarray
    axis: np.ndarray


@dataclass
class FemViewerData:
    case: fem.PeriodicMotionCase
    global_history: dict[str, np.ndarray]
    displacements: dict[str, np.ndarray]
    reactions: dict[str, np.ndarray]
    metrics: dict[str, np.ndarray]
    strain_norm: float
    element_strains: dict[str, np.ndarray]
    element_stresses_mpa: dict[str, np.ndarray]
    strain_source: str
    torque_series: tuple[TorqueOverlaySeries, ...]
    torque_norm_nm: float


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def camera_basis(camera: Camera) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    cp = math.cos(camera.pitch)
    sp = math.sin(camera.pitch)
    cy = math.cos(camera.yaw)
    sy = math.sin(camera.yaw)
    camera_pos = camera.target + camera.distance * np.array([cp * cy, cp * sy, sp], dtype=float)
    forward = camera.target - camera_pos
    forward = forward / max(float(np.linalg.norm(forward)), 1.0e-12)
    right = np.array([-sy, cy, 0.0], dtype=float)
    up = np.cross(right, forward)
    up = up / max(float(np.linalg.norm(up)), 1.0e-12)
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
    return (int(width * 0.5 + x * scale), int(height * 0.56 - y * scale)), z


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


def color_for_strain(strain: float, strain_norm: float) -> Color:
    value = 0.5 + 0.5 * abs(strain) / max(strain_norm, 1.0e-12)
    rgba = CMAP(max(0.0, min(1.0, value)))
    return tuple(int(max(0.0, min(1.0, channel)) * 255) for channel in rgba[:3])


def color_for_torque(torque_nm: float, torque_norm_nm: float) -> Color:
    value = abs(torque_nm) / max(torque_norm_nm, 1.0e-12)
    rgba = CMAP(max(0.0, min(1.0, value)))
    return tuple(int(max(0.0, min(1.0, channel)) * 255) for channel in rgba[:3])


def add_line(
    commands: list[DrawCommand],
    camera: Camera,
    screen_size: tuple[int, int],
    a: np.ndarray,
    b: np.ndarray,
    color: Color,
    width: int = 1,
) -> None:
    out = project_points((a, b), camera, screen_size)
    if out is None:
        return
    points, depth = out
    commands.append(DrawCommand(depth, "line", points, color, width))


def add_projected_arrow(
    commands: list[DrawCommand],
    camera: Camera,
    screen_size: tuple[int, int],
    start_m: np.ndarray,
    end_m: np.ndarray,
    color: Color,
    width: int,
) -> None:
    out = project_points((start_m, end_m), camera, screen_size)
    if out is None:
        return
    points, depth = out
    start2, end2 = points
    commands.append(DrawCommand(depth, "line", (start2, end2), color, width))

    dx = end2[0] - start2[0]
    dy = end2[1] - start2[1]
    length = math.hypot(dx, dy)
    if length < 1.0:
        return
    ux = dx / length
    uy = dy / length
    px = -uy
    py = ux
    head = max(7.0, 4.0 + 2.2 * width)
    spread = 0.55 * head
    left = (int(end2[0] - ux * head + px * spread), int(end2[1] - uy * head + py * spread))
    right = (int(end2[0] - ux * head - px * spread), int(end2[1] - uy * head - py * spread))
    commands.append(DrawCommand(depth, "line", (end2, left), color, width))
    commands.append(DrawCommand(depth, "line", (end2, right), color, width))


def add_grid(commands: list[DrawCommand], camera: Camera, screen_size: tuple[int, int], center: np.ndarray) -> None:
    span = 0.80
    step = 0.10
    origin_x = round(float(center[0]) / step) * step
    origin_y = round(float(center[1]) / step) * step
    for idx in range(-6, 7):
        offset = idx * step
        add_line(
            commands,
            camera,
            screen_size,
            np.array([origin_x - span, origin_y + offset, 0.0], dtype=float),
            np.array([origin_x + span, origin_y + offset, 0.0], dtype=float),
            (42, 48, 56),
            1,
        )
        add_line(
            commands,
            camera,
            screen_size,
            np.array([origin_x + offset, origin_y - span, 0.0], dtype=float),
            np.array([origin_x + offset, origin_y + span, 0.0], dtype=float),
            (42, 48, 56),
            1,
        )


def add_element_wire_command(
    commands: list[DrawCommand],
    camera: Camera,
    screen_size: tuple[int, int],
    start_mm: np.ndarray,
    end_mm: np.ndarray,
    color: Color,
    show_mesh: bool,
) -> None:
    if not show_mesh:
        return
    start_m = start_mm * 0.001
    end_m = end_mm * 0.001
    out = project_points((start_m, end_m), camera, screen_size)
    if out is None:
        return
    points, depth = out
    commands.append(DrawCommand(depth, "line", points, color, 4))


def add_circle_command(
    commands: list[DrawCommand],
    camera: Camera,
    screen_size: tuple[int, int],
    point_m: np.ndarray,
    radius: int,
    color: Color,
    label: str = "",
) -> Point2 | None:
    out = project(point_m, camera, screen_size)
    if out is None:
        return None
    point2, depth = out
    commands.append(DrawCommand(depth, "circle", (point2,), color, radius, label=label))
    return point2


def add_ring_command(
    commands: list[DrawCommand],
    camera: Camera,
    screen_size: tuple[int, int],
    point_m: np.ndarray,
    radius: int,
    color: Color,
    label: str = "",
) -> Point2 | None:
    out = project(point_m, camera, screen_size)
    if out is None:
        return None
    point2, depth = out
    commands.append(DrawCommand(depth, "ring", (point2,), color, radius, label=label))
    return point2


def render_commands(screen: pygame.Surface, commands: list[DrawCommand]) -> None:
    for command in sorted(commands, key=lambda item: item.depth, reverse=True):
        if command.kind == "poly":
            pygame.draw.polygon(screen, command.color, command.points)
        elif command.kind == "line":
            pygame.draw.line(screen, command.color, command.points[0], command.points[1], command.width)
        elif command.kind == "circle":
            pygame.draw.circle(screen, command.color, command.points[0], command.width)
        elif command.kind == "ring":
            pygame.draw.circle(screen, command.color, command.points[0], command.width, max(2, command.width // 4))


def current_coords_mm(data: FemViewerData, frame_idx: int) -> dict[str, np.ndarray]:
    return fem.current_coords_mm(data.case, data.displacements, frame_idx)


def current_element_strains(data: FemViewerData, coords: dict[str, np.ndarray], frame_idx: int) -> dict[str, float]:
    if data.element_strains:
        return {name: float(values[frame_idx]) for name, values in data.element_strains.items()}
    return fem.member_strains(data.case, coords)


def sample_torque_overlay(
    torque_series: tuple[TorqueOverlaySeries, ...], time_ms: float
) -> tuple[TorqueOverlaySample, ...]:
    samples: list[TorqueOverlaySample] = []
    for series in torque_series:
        if len(series.time_ms) == 0:
            continue
        torque_nm = float(np.interp(time_ms, series.time_ms, series.torque_nm))
        axis = np.array(
            [float(np.interp(time_ms, series.time_ms, series.axis[:, idx])) for idx in range(3)],
            dtype=float,
        )
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm <= 1.0e-12:
            continue
        samples.append(
            TorqueOverlaySample(
                joint=series.joint,
                proximal_node=series.proximal_node,
                distal_node=series.distal_node,
                torque_nm=torque_nm,
                axis=axis / axis_norm,
            )
        )
    return tuple(samples)


def current_torque_samples(data: FemViewerData, frame_idx: int) -> tuple[TorqueOverlaySample, ...]:
    if not data.torque_series:
        return ()
    return sample_torque_overlay(data.torque_series, float(data.global_history["time"][frame_idx]))


def add_torque_overlay_commands(
    commands: list[DrawCommand],
    data: FemViewerData,
    state: ViewerState,
    coords: dict[str, np.ndarray],
    screen_size: tuple[int, int],
    mouse_pos: tuple[int, int],
    hover_candidates: list[HoverItem],
) -> None:
    if not state.show_torque or data.torque_norm_nm <= 1.0e-12:
        return

    for sample in current_torque_samples(data, state.frame_idx):
        if sample.proximal_node not in coords or sample.distal_node not in coords:
            continue
        intensity = min(1.0, abs(sample.torque_nm) / data.torque_norm_nm)
        color = color_for_torque(sample.torque_nm, data.torque_norm_nm)
        width = max(2, int(round(2 + 3 * math.sqrt(intensity))))
        radius = max(5, int(round(5 + 6 * math.sqrt(intensity))))
        length_m = 0.014 + 0.048 * math.sqrt(intensity)

        for node_name, sign in ((sample.proximal_node, -1.0), (sample.distal_node, 1.0)):
            center_m = coords[node_name] * 0.001
            vector_m = sample.axis * sign * length_m
            add_projected_arrow(
                commands,
                state.camera,
                screen_size,
                center_m - 0.5 * vector_m,
                center_m + 0.5 * vector_m,
                color,
                width,
            )
            point2 = add_ring_command(
                commands,
                state.camera,
                screen_size,
                center_m,
                radius,
                color,
                label=sample.joint,
            )
            if point2 is not None:
                hover_candidates.append(
                    hover_item(f"{sample.joint} applied torque {sample.torque_nm:.4f} Nm", point2, mouse_pos)
                )


def make_scene_commands(
    data: FemViewerData,
    state: ViewerState,
    screen_size: tuple[int, int],
    mouse_pos: tuple[int, int],
) -> list[DrawCommand]:
    coords = current_coords_mm(data, state.frame_idx)
    strains = current_element_strains(data, coords, state.frame_idx)
    commands: list[DrawCommand] = []
    center_m = np.array(list(coords.values()), dtype=float).mean(axis=0) * 0.001
    if state.show_grid:
        add_grid(commands, state.camera, screen_size, center_m)

    for item in data.case.deck.members:
        color = color_for_strain(strains[fem.beam_element_key(item)], data.strain_norm)
        add_element_wire_command(
            commands,
            state.camera,
            screen_size,
            coords[item.node_a_name],
            coords[item.node_b_name],
            color,
            state.show_mesh,
        )

    hover_candidates: list[HoverItem] = []
    if state.show_solver_nodes:
        for node_name in data.case.deck.node_ids:
            point2 = add_circle_command(
                commands,
                state.camera,
                screen_size,
                coords[node_name] * 0.001,
                2,
                (13, 17, 23),
                label=node_name,
            )
            if point2 is not None:
                hover_candidates.append(hover_item(node_name, point2, mouse_pos))

    if state.show_nodes:
        for node in data.case.deck.rod_model.nodes:
            color = (249, 250, 252)
            if node.contact_candidate:
                color = (255, 184, 103)
            point2 = add_circle_command(
                commands,
                state.camera,
                screen_size,
                coords[node.name] * 0.001,
                5 if not node.contact_candidate else 6,
                color,
                label=node.name,
            )
            if point2 is not None:
                hover_candidates.append(hover_item(node.name, point2, mouse_pos))

    add_torque_overlay_commands(commands, data, state, coords, screen_size, mouse_pos, hover_candidates)

    state.hover = min(hover_candidates, key=lambda item: item.distance_px) if hover_candidates else None
    if state.hover and state.hover.distance_px > 18.0:
        state.hover = None
    if state.hover is not None:
        commands.append(DrawCommand(state.hover.depth, "circle", (state.hover.point,), (255, 255, 255), 8))
    return commands


def hover_item(label: str, point: Point2, mouse_pos: tuple[int, int]) -> HoverItem:
    dx = point[0] - mouse_pos[0]
    dy = point[1] - mouse_pos[1]
    return HoverItem(label=label, point=point, depth=0.0, distance_px=math.hypot(dx, dy))


def draw_panel(screen: pygame.Surface, font: pygame.font.Font, small_font: pygame.font.Font, data: FemViewerData, state: ViewerState, fps: float) -> None:
    panel_width = 640
    panel_height = 254 if state.show_help else 162
    panel = pygame.Rect(16, 16, panel_width, panel_height)
    pygame.draw.rect(screen, (19, 23, 29), panel, border_radius=8)
    pygame.draw.rect(screen, (78, 89, 102), panel, width=1, border_radius=8)

    time_ms = data.global_history["time"][state.frame_idx]
    max_disp = data.metrics["max_disp"][state.frame_idx]
    max_strain = data.metrics["max_abs_strain"][state.frame_idx]
    max_stress = 0.0
    if data.element_stresses_mpa:
        max_stress = max(abs(float(values[state.frame_idx])) for values in data.element_stresses_mpa.values())
    lines = [
        "OpenRadioss FEM pygame viewer",
        (
            f"frame {state.frame_idx + 1}/{len(data.global_history['time'])} | "
            f"t={time_ms:.3f} ms | disp={max_disp:.3f} mm | strain={max_strain:.3e}"
        ),
        (
            f"radius={data.case.deck.uniform_radius_mm:g} mm | elements={len(data.case.deck.members)} | "
            f"nodes={len(data.case.deck.node_ids)} | {'paused' if state.paused else 'play'} x{state.playback_speed:.2g} | fps {fps:4.1f}"
        ),
    ]
    source_line = f"strain source: {data.strain_source}"
    if data.element_stresses_mpa:
        source_line += f" | max stress={max_stress:.2f} MPa"
    lines.append(source_line)
    if data.torque_series:
        samples = current_torque_samples(data, state.frame_idx)
        hot = max(samples, key=lambda sample: abs(sample.torque_nm)) if samples else None
        hot_text = f"{hot.joint} {hot.torque_nm:.3f} Nm" if hot is not None else "none"
        lines.append(
            f"torque overlay {'on' if state.show_torque else 'off'} | applied max {data.torque_norm_nm:.3f} Nm | frame hot {hot_text}"
        )
    if state.hover is not None:
        lines.append(f"hover: {state.hover.label}")
    if state.show_help:
        lines.extend(
            [
                "mouse drag: rotate camera | wheel: zoom | p: screenshot",
                "space: pause/play | left/right: step | home/end: first/last",
                "m: mesh elements | n: joint nodes | s: solver nodes | t: torque overlay | g: grid | h: help | q/esc: quit",
            ]
        )
    y = panel.y + 14
    for idx, line in enumerate(lines):
        surface = (font if idx == 0 else small_font).render(line, True, (231, 235, 240))
        screen.blit(surface, (panel.x + 14, y))
        y += 30 if idx == 0 else 24


def draw_seismic_legend(screen: pygame.Surface, small_font: pygame.font.Font, data: FemViewerData) -> None:
    width, height = screen.get_size()
    bar_width = 360
    bar_height = 16
    x = width - bar_width - 28
    y = height - 52
    panel = pygame.Rect(x - 12, y - 28, bar_width + 24, 66)
    pygame.draw.rect(screen, (19, 23, 29), panel, border_radius=7)
    pygame.draw.rect(screen, (78, 89, 102), panel, width=1, border_radius=7)
    legend_title = "absolute beam strain" if data.element_strains else "absolute strain proxy"
    title = small_font.render(legend_title, True, (231, 235, 240))
    screen.blit(title, (x, y - 24))
    for idx in range(bar_width):
        t = 0.5 + 0.5 * idx / max(bar_width - 1, 1)
        rgba = CMAP(t)
        color = tuple(int(channel * 255) for channel in rgba[:3])
        pygame.draw.line(screen, color, (x + idx, y), (x + idx, y + bar_height), 1)
    pygame.draw.rect(screen, (220, 226, 235), pygame.Rect(x, y, bar_width, bar_height), width=1)
    tick_labels = [
        ("0", x),
        ("tension/load hot", x + bar_width - 158),
    ]
    for label, tx in tick_labels:
        surface = small_font.render(label, True, (231, 235, 240))
        screen.blit(surface, (tx, y + bar_height + 5))
    max_label = small_font.render(f"{data.strain_norm:.1e}", True, (180, 188, 200))
    screen.blit(max_label, (x + bar_width - max_label.get_width(), y - 24))


def draw_torque_legend(screen: pygame.Surface, small_font: pygame.font.Font, data: FemViewerData, state: ViewerState) -> None:
    if not state.show_torque or data.torque_norm_nm <= 1.0e-12:
        return
    _width, height = screen.get_size()
    bar_width = 280
    bar_height = 13
    x = 28
    y = height - 50
    panel = pygame.Rect(x - 12, y - 28, bar_width + 24, 64)
    pygame.draw.rect(screen, (19, 23, 29), panel, border_radius=7)
    pygame.draw.rect(screen, (78, 89, 102), panel, width=1, border_radius=7)
    title = small_font.render("applied joint torque replay", True, (231, 235, 240))
    screen.blit(title, (x, y - 24))
    for idx in range(bar_width):
        t = idx / max(bar_width - 1, 1)
        rgba = CMAP(t)
        color = tuple(int(channel * 255) for channel in rgba[:3])
        pygame.draw.line(screen, color, (x + idx, y), (x + idx, y + bar_height), 1)
    pygame.draw.rect(screen, (220, 226, 235), pygame.Rect(x, y, bar_width, bar_height), width=1)
    zero_label = small_font.render("0", True, (231, 235, 240))
    max_label = small_font.render(f"{data.torque_norm_nm:.2f} Nm", True, (231, 235, 240))
    screen.blit(zero_label, (x, y + bar_height + 5))
    screen.blit(max_label, (x + bar_width - max_label.get_width(), y + bar_height + 5))


def default_result_csv(run_dir: Path, run_name: str) -> Path:
    csv_path = run_dir / f"{run_name}T01.csv"
    if csv_path.is_file():
        return csv_path
    return run_dir / "stage2_whole_body_periodic_motionT01.csv"


def default_torque_csv(run_dir: Path) -> Path:
    return run_dir / "stage1_torque_replay_loads.csv"


def load_torque_overlay_series(path: Path) -> tuple[TorqueOverlaySeries, ...]:
    if not path.is_file():
        return ()

    grouped: dict[tuple[str, str, str], list[tuple[float, float, np.ndarray]]] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            key = (row["joint"], row["proximal_node"], row["distal_node"])
            axis = np.array([float(row["axis_x"]), float(row["axis_y"]), float(row["axis_z"])], dtype=float)
            grouped.setdefault(key, []).append((float(row["time_ms"]), float(row["torque_nm"]), axis))

    series: list[TorqueOverlaySeries] = []
    for (joint, proximal_node, distal_node), values in grouped.items():
        values.sort(key=lambda item: item[0])
        time_ms = np.array([item[0] for item in values], dtype=float)
        torque_nm = np.array([item[1] for item in values], dtype=float)
        axis = np.array([item[2] for item in values], dtype=float)
        series.append(
            TorqueOverlaySeries(
                joint=joint,
                proximal_node=proximal_node,
                distal_node=distal_node,
                time_ms=time_ms,
                torque_nm=torque_nm,
                axis=axis,
            )
        )
    return tuple(sorted(series, key=lambda item: (item.proximal_node, item.distal_node, item.joint)))


def load_summary_value(summary: dict[str, Any], key: str, fallback: Any) -> Any:
    value = summary.get(key, fallback)
    return fallback if value is None else value


def load_case(args: argparse.Namespace) -> fem.PeriodicMotionCase:
    summary_path = args.run_dir / "openradioss_periodic_motion_summary.yaml"
    summary: dict[str, Any] = {}
    if summary_path.is_file():
        import yaml

        summary = yaml.safe_load(summary_path.read_text(encoding="utf-8")) or {}

    timing = summary.get("timing", {})
    counts = summary.get("counts", {})
    description = load_dog_description(args.description)
    catalog = stage1.load_catalog(args.materials, args.actuators, args.batteries)
    return fem.build_periodic_motion_case(
        description,
        catalog,
        out_dir=args.run_dir,
        run_name=args.run_name,
        sample_count=int(load_summary_value(counts, "motion_samples", args.samples)),
        solver_duration_ms=float(load_summary_value(timing, "solver_duration_ms", args.solver_duration_ms)),
        viewer_start_seconds=float(load_summary_value(timing, "viewer_start_seconds", args.viewer_start_seconds)),
        viewer_motion_seconds=float(load_summary_value(timing, "viewer_motion_seconds_sampled", args.viewer_motion_seconds)),
        babble_scale=args.babble_scale,
        motion_scale=args.motion_scale,
        target_element_length_mm=float(summary.get("target_element_length_mm") or args.target_element_length_mm),
        use_nominal_radius_for_massless_members=not args.minimum_radius_for_massless_members,
        uniform_radius_mm=float(summary.get("uniform_radius_mm") or args.uniform_radius_mm),
        case_name=args.case_name,
        control_policy=str(summary.get("control_policy") or args.control_policy),
        torque_scale=float(summary.get("torque_scale") or args.torque_scale),
    )


def load_viewer_data(args: argparse.Namespace) -> FemViewerData:
    case = load_case(args)
    result_csv = args.result_csv or default_result_csv(args.run_dir, args.run_name)
    if not result_csv.is_file():
        raise SystemExit(f"result CSV not found: {result_csv}")
    global_history, displacements, reactions = fem.parse_displacement_history(result_csv, case.deck.node_ids)
    element_stresses_mpa, element_strains = fem.load_beam_resultant_strains(result_csv, case)
    strain_source = "Radioss beam F/M outer-fiber strain" if element_strains else "displacement-derived strain proxy"
    metrics = fem.result_metrics(case, displacements, reactions, element_strains or None)
    strain_norm = max(float(np.nanmax(metrics["max_abs_strain"])), 1.0e-6)
    torque_series = load_torque_overlay_series(args.torque_csv or default_torque_csv(args.run_dir))
    torque_norm_nm = max((float(np.nanmax(series.torque_nm)) for series in torque_series), default=0.0)
    return FemViewerData(
        case=case,
        global_history=global_history,
        displacements=displacements,
        reactions=reactions,
        metrics=metrics,
        strain_norm=strain_norm,
        element_strains=element_strains,
        element_stresses_mpa=element_stresses_mpa,
        strain_source=strain_source,
        torque_series=torque_series,
        torque_norm_nm=torque_norm_nm,
    )


def fit_camera_to_case(camera: Camera, data: FemViewerData) -> None:
    coords = np.array(list(fem.initial_coords_mm(data.case).values()), dtype=float) * 0.001
    mins = coords.min(axis=0)
    maxs = coords.max(axis=0)
    center = 0.5 * (mins + maxs)
    span = max(float((maxs - mins).max()), 0.25)
    camera.target = center
    camera.distance = max(1.35, span * 2.25)


def save_screenshot(screen: pygame.Surface, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    pygame.image.save(screen, path)
    print(f"saved screenshot: {path}")


def handle_key(event: pygame.event.Event, state: ViewerState, frame_count: int) -> bool:
    if event.key in {pygame.K_ESCAPE, pygame.K_q}:
        return False
    if event.key == pygame.K_SPACE:
        state.paused = not state.paused
    elif event.key == pygame.K_LEFT:
        state.paused = True
        state.frame_idx = max(0, state.frame_idx - 1)
    elif event.key == pygame.K_RIGHT:
        state.paused = True
        state.frame_idx = min(frame_count - 1, state.frame_idx + 1)
    elif event.key == pygame.K_HOME:
        state.paused = True
        state.frame_idx = 0
    elif event.key == pygame.K_END:
        state.paused = True
        state.frame_idx = frame_count - 1
    elif event.key in {pygame.K_EQUALS, pygame.K_PLUS}:
        state.playback_speed = min(8.0, state.playback_speed * 1.25)
    elif event.key == pygame.K_MINUS:
        state.playback_speed = max(0.125, state.playback_speed / 1.25)
    elif event.key == pygame.K_h:
        state.show_help = not state.show_help
    elif event.key == pygame.K_m:
        state.show_mesh = not state.show_mesh
    elif event.key == pygame.K_n:
        state.show_nodes = not state.show_nodes
    elif event.key == pygame.K_s:
        state.show_solver_nodes = not state.show_solver_nodes
    elif event.key == pygame.K_t:
        state.show_torque = not state.show_torque
    elif event.key == pygame.K_g:
        state.show_grid = not state.show_grid
    return True


def render_once(screen: pygame.Surface, font: pygame.font.Font, small_font: pygame.font.Font, data: FemViewerData, state: ViewerState, mouse_pos: tuple[int, int], fps: float) -> None:
    screen.fill((12, 15, 19))
    commands = make_scene_commands(data, state, screen.get_size(), mouse_pos)
    render_commands(screen, commands)
    draw_panel(screen, font, small_font, data, state, fps)
    draw_torque_legend(screen, small_font, data, state)
    draw_seismic_legend(screen, small_font, data)
    pygame.display.flip()


def run_viewer(args: argparse.Namespace) -> None:
    if args.headless:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

    pygame.init()
    pygame.display.set_caption("Stage 2 OpenRadioss FEM viewer")
    screen = pygame.display.set_mode((args.width, args.height))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Menlo,Consolas,monospace", 24)
    small_font = pygame.font.SysFont("Menlo,Consolas,monospace", 19)
    data = load_viewer_data(args)
    camera = Camera()
    fit_camera_to_case(camera, data)
    state = ViewerState(camera=camera, paused=args.paused, frame_idx=args.frame)

    frame_count = len(data.global_history["time"])
    state.frame_idx = max(0, min(frame_count - 1, state.frame_idx))
    dragging = False
    last_mouse = (0, 0)
    screenshot_path = args.screenshot
    running = True
    headless_frames = 0

    while running:
        dt = clock.tick(args.fps) / 1000.0
        mouse_pos = pygame.mouse.get_pos()
        if not state.paused and frame_count > 1:
            state.accumulator += dt * args.fps * state.playback_speed
            while state.accumulator >= 1.0:
                state.frame_idx = (state.frame_idx + 1) % frame_count
                state.accumulator -= 1.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_p:
                    screenshot_path = args.run_dir / "pygame_openradioss_fem_viewer_screenshot.png"
                else:
                    running = handle_key(event, state, frame_count)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    dragging = True
                    last_mouse = event.pos
                elif event.button == 4:
                    state.camera.distance = max(0.20, state.camera.distance * 0.92)
                elif event.button == 5:
                    state.camera.distance = min(4.00, state.camera.distance * 1.08)
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                dragging = False
            elif event.type == pygame.MOUSEMOTION and dragging:
                dx = event.pos[0] - last_mouse[0]
                dy = event.pos[1] - last_mouse[1]
                state.camera.yaw = wrap_angle(state.camera.yaw - dx * 0.008)
                state.camera.pitch = max(math.radians(-82.0), min(math.radians(82.0), state.camera.pitch - dy * 0.006))
                last_mouse = event.pos

        render_once(screen, font, small_font, data, state, mouse_pos, clock.get_fps())
        if screenshot_path is not None:
            save_screenshot(screen, screenshot_path)
            screenshot_path = None
            if args.headless:
                running = False

        headless_frames += 1
        if args.headless and headless_frames >= args.max_headless_frames:
            running = False

    pygame.quit()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, default=RUN_DIR_DEFAULT)
    parser.add_argument("--run-name", default=fem.RUN_NAME)
    parser.add_argument("--result-csv", type=Path, default=None)
    parser.add_argument("--torque-csv", type=Path, default=None)
    parser.add_argument("--description", type=Path, default=DEFAULT_DESCRIPTION_PATH)
    parser.add_argument("--materials", type=Path, default=stage1.DEFAULT_MATERIALS_PATH)
    parser.add_argument("--actuators", type=Path, default=stage1.DEFAULT_ACTUATORS_PATH)
    parser.add_argument("--batteries", type=Path, default=stage1.DEFAULT_BATTERIES_PATH)
    parser.add_argument("--case-name", default="stage2_viewer_periodic_motion")
    parser.add_argument("--samples", type=int, default=25)
    parser.add_argument("--solver-duration-ms", type=float, default=8.0)
    parser.add_argument("--viewer-start-seconds", type=float, default=0.2)
    parser.add_argument("--viewer-motion-seconds", type=float, default=0.005)
    parser.add_argument("--babble-scale", type=float, default=1.0)
    parser.add_argument("--motion-scale", type=float, default=1.0)
    parser.add_argument("--target-element-length-mm", type=float, default=8.0)
    parser.add_argument("--uniform-radius-mm", type=float, default=12.0)
    parser.add_argument("--control-policy", default="stage1-torque-replay")
    parser.add_argument("--torque-scale", type=float, default=1.0)
    parser.add_argument("--minimum-radius-for-massless-members", action="store_true")
    parser.add_argument("--width", type=int, default=1440)
    parser.add_argument("--height", type=int, default=900)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--frame", type=int, default=0)
    parser.add_argument("--paused", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--screenshot", type=Path, default=None)
    parser.add_argument("--max-headless-frames", type=int, default=2)
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    run_viewer(build_arg_parser().parse_args(argv))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
