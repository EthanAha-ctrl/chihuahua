#!/usr/bin/env python3
"""Load the dog/robot description YAML used by the geometry tools."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


DEFAULT_DESCRIPTION_PATH = Path(__file__).with_name("dog_description.yaml")


@dataclass(frozen=True)
class JointRange:
    visual_tuning: bool
    bias_deg: float
    amp_deg: float
    min_deg: float
    max_deg: float


@dataclass
class GeometryDescription:
    body_length_total_m: float
    front_body_fraction: float
    waist_joint_spacing_m: float
    body_half_width_m: float
    hip_half_width_m: float
    foot_x_offset_m: float
    foot_lateral_outset_m: float
    min_reach_xy_m: float
    max_reach_xy_m: float
    min_foot_clearance_m: float
    visual_tuning: dict[str, bool] = field(default_factory=dict)

    def robot_geometry_kwargs(self) -> dict[str, float]:
        body_length_without_waist = self.body_length_total_m - self.waist_joint_spacing_m
        front_body_length = body_length_without_waist * self.front_body_fraction
        rear_body_length = body_length_without_waist * (1.0 - self.front_body_fraction)
        return {
            "front_body_length": front_body_length,
            "rear_body_length": rear_body_length,
            "waist_joint_spacing": self.waist_joint_spacing_m,
            "body_half_width": self.body_half_width_m,
            "hip_half_width": self.hip_half_width_m,
            "foot_x_offset": self.foot_x_offset_m,
            "foot_lateral_outset": self.foot_lateral_outset_m,
            "min_reach_xy": self.min_reach_xy_m,
            "max_reach_xy": self.max_reach_xy_m,
            "min_foot_clearance": self.min_foot_clearance_m,
        }


@dataclass
class LinkDescription:
    upper_m: float
    lower_m: float
    distal_endpoint_m: float
    visual_tuning: dict[str, bool] = field(default_factory=dict)


@dataclass
class HeadClawDescription:
    neck_length_m: float
    hinge_half_gap_m: float
    jaw_length_m: float
    visual_tuning: dict[str, bool] = field(default_factory=dict)


@dataclass
class ViewerDescription:
    body_z_m: float
    links: LinkDescription
    head_claw: HeadClawDescription
    visual_tuning: dict[str, bool] = field(default_factory=dict)


@dataclass
class DogDescription:
    name: str
    geometry: GeometryDescription
    viewer: ViewerDescription
    joint_ranges: dict[str, JointRange]
    sources: Mapping[str, Any]


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a YAML mapping")
    return value


def _required_mapping(parent: Mapping[str, Any], key: str, label: str) -> Mapping[str, Any]:
    if key not in parent:
        raise ValueError(f"{label}.{key} is required")
    return _mapping(parent[key], f"{label}.{key}")


def _required_float(parent: Mapping[str, Any], key: str, label: str) -> float:
    if key not in parent:
        raise ValueError(f"{label}.{key} is required")
    try:
        return float(parent[key])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label}.{key} must be numeric") from exc


def _scalar_value_and_flag(parent: Mapping[str, Any], key: str, label: str) -> tuple[float, bool]:
    if key not in parent:
        raise ValueError(f"{label}.{key} is required")
    value = parent[key]
    if isinstance(value, Mapping):
        if "value" not in value:
            raise ValueError(f"{label}.{key}.value is required")
        return _required_float(value, "value", f"{label}.{key}"), _optional_bool(value, "visual_tuning", False)
    try:
        return float(value), False
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label}.{key} must be numeric or a value mapping") from exc


def _optional_bool(parent: Mapping[str, Any], key: str, default: bool) -> bool:
    value = parent.get(key, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "yes", "1", "on"}:
            return True
        if normalized in {"false", "no", "0", "off"}:
            return False
    raise ValueError(f"{key} must be boolean")


def _load_geometry(data: Mapping[str, Any]) -> GeometryDescription:
    geometry = _required_mapping(data, "geometry", "description")
    has_total_length = "body_length_total_m" in geometry
    has_front_fraction = "front_body_fraction" in geometry
    if has_total_length != has_front_fraction:
        raise ValueError("geometry.body_length_total_m and geometry.front_body_fraction must be provided together")
    if has_total_length:
        length_keys = ("body_length_total_m", "front_body_fraction")
    else:
        front_length, front_flag = _scalar_value_and_flag(geometry, "front_body_length_m", "geometry")
        rear_length, rear_flag = _scalar_value_and_flag(geometry, "rear_body_length_m", "geometry")
        total_length = front_length + rear_length
        front_fraction = 0.5 if abs(total_length) < 1e-12 else front_length / total_length
        length_values = {
            "body_length_total_m": total_length,
            "front_body_fraction": front_fraction,
        }
        length_flags = {
            "body_length_total_m": front_flag or rear_flag,
            "front_body_fraction": front_flag or rear_flag,
        }
        length_keys = ()

    keys = (
        "body_half_width_m",
        "hip_half_width_m",
        "foot_x_offset_m",
        "foot_lateral_outset_m",
        "min_reach_xy_m",
        "max_reach_xy_m",
        "min_foot_clearance_m",
    )
    values: dict[str, float] = {}
    flags: dict[str, bool] = {}
    if length_keys:
        for key in length_keys:
            values[key], flags[key] = _scalar_value_and_flag(geometry, key, "geometry")
    else:
        values.update(length_values)
        flags.update(length_flags)

    for key in keys:
        values[key], flags[key] = _scalar_value_and_flag(geometry, key, "geometry")
    values["waist_joint_spacing_m"], flags["waist_joint_spacing_m"] = _scalar_value_and_flag(
        geometry, "waist_joint_spacing_m", "geometry"
    )

    if values["body_length_total_m"] <= 0.0:
        raise ValueError("geometry.body_length_total_m must be positive")
    if not 0.0 <= values["front_body_fraction"] <= 1.0:
        raise ValueError("geometry.front_body_fraction must be between 0 and 1")
    if values["waist_joint_spacing_m"] <= 0.0:
        raise ValueError("geometry.waist_joint_spacing_m must be positive")
    if values["waist_joint_spacing_m"] >= values["body_length_total_m"]:
        raise ValueError("geometry.waist_joint_spacing_m must be smaller than geometry.body_length_total_m")

    return GeometryDescription(
        body_length_total_m=values["body_length_total_m"],
        front_body_fraction=values["front_body_fraction"],
        waist_joint_spacing_m=values["waist_joint_spacing_m"],
        body_half_width_m=values["body_half_width_m"],
        hip_half_width_m=values["hip_half_width_m"],
        foot_x_offset_m=values["foot_x_offset_m"],
        foot_lateral_outset_m=values["foot_lateral_outset_m"],
        min_reach_xy_m=values["min_reach_xy_m"],
        max_reach_xy_m=values["max_reach_xy_m"],
        min_foot_clearance_m=values["min_foot_clearance_m"],
        visual_tuning=flags,
    )


def _load_viewer(data: Mapping[str, Any]) -> ViewerDescription:
    viewer = _required_mapping(data, "viewer", "description")
    links = _required_mapping(viewer, "links_m", "viewer")
    head_claw = _required_mapping(viewer, "head_claw", "viewer")
    body_z_m, body_z_flag = _scalar_value_and_flag(viewer, "body_z_m", "viewer")

    link_values: dict[str, float] = {}
    link_flags: dict[str, bool] = {}
    for yaml_key, attr_name in (
        ("upper", "upper_m"),
        ("lower", "lower_m"),
        ("distal_endpoint", "distal_endpoint_m"),
    ):
        link_values[attr_name], link_flags[attr_name] = _scalar_value_and_flag(links, yaml_key, "viewer.links_m")

    claw_keys = (
        "neck_length_m",
        "hinge_half_gap_m",
        "jaw_length_m",
    )
    claw_values: dict[str, float] = {}
    claw_flags: dict[str, bool] = {}
    for key in claw_keys:
        claw_values[key], claw_flags[key] = _scalar_value_and_flag(head_claw, key, "viewer.head_claw")

    return ViewerDescription(
        body_z_m=body_z_m,
        links=LinkDescription(
            upper_m=link_values["upper_m"],
            lower_m=link_values["lower_m"],
            distal_endpoint_m=link_values["distal_endpoint_m"],
            visual_tuning=link_flags,
        ),
        head_claw=HeadClawDescription(
            neck_length_m=claw_values["neck_length_m"],
            hinge_half_gap_m=claw_values["hinge_half_gap_m"],
            jaw_length_m=claw_values["jaw_length_m"],
            visual_tuning=claw_flags,
        ),
        visual_tuning={"body_z_m": body_z_flag},
    )


def _load_joint_ranges(data: Mapping[str, Any]) -> dict[str, JointRange]:
    ranges = _required_mapping(data, "joint_ranges_deg", "description")
    required = (
        "waist_yaw",
        "waist_pitch",
        "hip_ab",
        "hip_pitch",
        "knee_bend",
        "toe_bend",
        "neck_yaw",
        "neck_pitch",
        "head_claw",
    )
    loaded: dict[str, JointRange] = {}
    for name in required:
        item = _required_mapping(ranges, name, "joint_ranges_deg")
        min_deg = _required_float(item, "min_deg", f"joint_ranges_deg.{name}")
        max_deg = _required_float(item, "max_deg", f"joint_ranges_deg.{name}")
        if min_deg > max_deg:
            raise ValueError(f"joint_ranges_deg.{name}.min_deg must be <= max_deg")
        loaded[name] = JointRange(
            visual_tuning=_optional_bool(item, "visual_tuning", False),
            bias_deg=_required_float(item, "bias_deg", f"joint_ranges_deg.{name}"),
            amp_deg=_required_float(item, "amp_deg", f"joint_ranges_deg.{name}"),
            min_deg=min_deg,
            max_deg=max_deg,
        )
    return loaded


def load_dog_description(path: Path = DEFAULT_DESCRIPTION_PATH) -> DogDescription:
    path = Path(path)
    with path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)

    data = _mapping(raw, "description")
    name = str(data.get("name", path.stem))
    sources = _mapping(data.get("sources", {}), "sources")
    return DogDescription(
        name=name,
        geometry=_load_geometry(data),
        viewer=_load_viewer(data),
        joint_ranges=_load_joint_ranges(data),
        sources=sources,
    )


def _tunable_scalar_yaml(value: float, visual_tuning: bool) -> dict[str, bool | float]:
    return {
        "visual_tuning": bool(visual_tuning),
        "value": float(value),
    }


def save_dog_description(path: Path, description: DogDescription) -> None:
    path = Path(path)
    with path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)

    data = dict(_mapping(raw, "description"))

    geometry_keys = (
        "body_length_total_m",
        "front_body_fraction",
        "waist_joint_spacing_m",
        "body_half_width_m",
        "hip_half_width_m",
        "foot_x_offset_m",
        "foot_lateral_outset_m",
        "min_reach_xy_m",
        "max_reach_xy_m",
        "min_foot_clearance_m",
    )
    geometry = dict(_required_mapping(data, "geometry", "description"))
    geometry.pop("front_body_length_m", None)
    geometry.pop("rear_body_length_m", None)
    for key in geometry_keys:
        geometry[key] = _tunable_scalar_yaml(
            getattr(description.geometry, key),
            description.geometry.visual_tuning.get(key, False),
        )
    data["geometry"] = geometry

    viewer = dict(_required_mapping(data, "viewer", "description"))
    viewer["body_z_m"] = _tunable_scalar_yaml(
        description.viewer.body_z_m,
        description.viewer.visual_tuning.get("body_z_m", False),
    )

    links = dict(_required_mapping(viewer, "links_m", "viewer"))
    for yaml_key, attr_name in (
        ("upper", "upper_m"),
        ("lower", "lower_m"),
        ("distal_endpoint", "distal_endpoint_m"),
    ):
        links[yaml_key] = _tunable_scalar_yaml(
            getattr(description.viewer.links, attr_name),
            description.viewer.links.visual_tuning.get(attr_name, False),
        )
    viewer["links_m"] = links

    head_claw = {}
    for key in (
        "neck_length_m",
        "hinge_half_gap_m",
        "jaw_length_m",
    ):
        head_claw[key] = _tunable_scalar_yaml(
            getattr(description.viewer.head_claw, key),
            description.viewer.head_claw.visual_tuning.get(key, False),
        )
    viewer["head_claw"] = head_claw
    data["viewer"] = viewer

    ranges = dict(_required_mapping(data, "joint_ranges_deg", "description"))
    for name, spec in description.joint_ranges.items():
        item = dict(_mapping(ranges.get(name, {}), f"joint_ranges_deg.{name}"))
        item["visual_tuning"] = bool(spec.visual_tuning)
        item["bias_deg"] = float(spec.bias_deg)
        item["amp_deg"] = float(spec.amp_deg)
        item["min_deg"] = float(spec.min_deg)
        item["max_deg"] = float(spec.max_deg)
        ranges[name] = item
    data["joint_ranges_deg"] = ranges

    with path.open("w", encoding="utf-8") as handle:
        handle.write("# Link-only dog/robot description for the endpoint geometry viewer.\n")
        handle.write("# No physics. No contact model. No controller. No solid body rendering.\n\n")
        yaml.safe_dump(data, handle, sort_keys=False, allow_unicode=True)
