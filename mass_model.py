#!/usr/bin/env python3
"""Stage 1 mass, COM, inertia, and free-space torque estimates.

This is a rough electromechanical design scaffold, not a CAD-derived mass
model. It turns the current linkage geometry into structured mass elements and
representative free-space inertial torque estimates so design iterations have
a single place to update mass and actuator margin.
"""

from __future__ import annotations

import argparse
import csv
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from dog_description import DEFAULT_DESCRIPTION_PATH, DogDescription, load_dog_description
from endpoint_geometry import LEG_ORDER, RobotGeometry, foot_points, waist_joint_points


DEFAULT_MATERIALS_PATH = Path(__file__).with_name("materials.yaml")
DEFAULT_ACTUATORS_PATH = Path(__file__).with_name("actuators.yaml")
DEFAULT_BATTERIES_PATH = Path(__file__).with_name("batteries.yaml")


@dataclass(frozen=True)
class Material:
    name: str
    density_kg_m3: float
    properties: Mapping[str, Any]


@dataclass(frozen=True)
class Actuator:
    name: str
    mass_kg: float
    driver_mass_kg: float
    continuous_torque_nm: float
    max_torque_nm: float
    properties: Mapping[str, Any]


@dataclass(frozen=True)
class Battery:
    name: str
    mass_kg: float
    placement_m: np.ndarray
    properties: Mapping[str, Any]


@dataclass(frozen=True)
class ElectronicsItem:
    name: str
    mass_kg: float
    placement_m: np.ndarray
    properties: Mapping[str, Any]


@dataclass(frozen=True)
class PhysicalCatalog:
    materials: dict[str, Material]
    structural_material: str
    elastomer_material: str
    actuators: dict[str, Actuator]
    actuator_assignments: dict[str, str]
    battery: Battery
    electronics: list[ElectronicsItem]


@dataclass(frozen=True)
class BodyPose:
    yaw_joint: np.ndarray
    pitch_joint: np.ndarray
    front_mid: np.ndarray
    rear_mid: np.ndarray
    front_forward: np.ndarray
    front_left: np.ndarray
    front_up: np.ndarray
    hips: dict[str, np.ndarray]
    bases: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]]


@dataclass(frozen=True)
class LegChain:
    hip: np.ndarray
    knee: np.ndarray
    toe_joint: np.ndarray
    toe_endpoint: np.ndarray
    ik_stretch_m: float


@dataclass(frozen=True)
class HeadChain:
    root: np.ndarray
    hinge: np.ndarray
    upper_hinge: np.ndarray
    lower_hinge: np.ndarray
    upper_tip: np.ndarray
    lower_tip: np.ndarray


@dataclass(frozen=True)
class MassElement:
    name: str
    kind: str
    mass_kg: float
    com_m: np.ndarray
    material: str = ""
    leg: str | None = None
    distal_level: int | None = None
    notes: str = ""


@dataclass(frozen=True)
class MassModel:
    case_name: str
    geometry: RobotGeometry
    pose: BodyPose
    legs: dict[str, LegChain]
    head: HeadChain
    elements: list[MassElement]

    @property
    def total_mass_kg(self) -> float:
        return sum(element.mass_kg for element in self.elements)

    @property
    def com_m(self) -> np.ndarray:
        total = self.total_mass_kg
        if total <= 0.0:
            return np.zeros(3)
        weighted = sum((element.mass_kg * element.com_m for element in self.elements), start=np.zeros(3))
        return weighted / total

    @property
    def inertia_about_com_kg_m2(self) -> np.ndarray:
        com = self.com_m
        inertia = np.zeros((3, 3), dtype=float)
        for element in self.elements:
            r = element.com_m - com
            inertia += element.mass_kg * (float(np.dot(r, r)) * np.eye(3) - np.outer(r, r))
        return inertia


@dataclass(frozen=True)
class TorqueRow:
    case_name: str
    joint: str
    actuator: str
    required_torque_nm: float
    continuous_torque_nm: float
    max_torque_nm: float
    continuous_margin: float | None
    max_margin: float | None
    notes: str


@dataclass(frozen=True)
class CaseRecord:
    case_name: str
    waist_yaw_deg: float
    waist_pitch_deg: float
    model: MassModel


@dataclass(frozen=True)
class Stage1Assumptions:
    body_height_m: float = 0.055
    body_equivalent_fill_fraction: float = 0.055
    waist_equivalent_fill_fraction: float = 0.080
    leg_link_width_m: float = 0.018
    leg_link_depth_m: float = 0.012
    leg_equivalent_fill_fraction: float = 0.65
    toe_link_width_m: float = 0.014
    toe_link_depth_m: float = 0.010
    toe_link_equivalent_fill_fraction: float = 0.70
    foot_pad_mass_kg: float = 0.008
    head_link_width_m: float = 0.012
    head_link_depth_m: float = 0.008
    head_equivalent_fill_fraction: float = 0.60
    joint_angular_accel_rad_s2: float = 10.0
    waist_angular_accel_rad_s2: float = 6.0
    neck_angular_accel_rad_s2: float = 8.0


def v3(x: float, y: float, z: float = 0.0) -> np.ndarray:
    return np.array([x, y, z], dtype=float)


def rot_z(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def normalized(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        return vector
    return vector / norm


def yaml_mapping(path: Path, label: str) -> Mapping[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    if not isinstance(raw, Mapping):
        raise ValueError(f"{label} must be a YAML mapping")
    return raw


def as_mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a YAML mapping")
    return value


def required_mapping(parent: Mapping[str, Any], key: str, label: str) -> Mapping[str, Any]:
    if key not in parent:
        raise ValueError(f"{label}.{key} is required")
    return as_mapping(parent[key], f"{label}.{key}")


def required_float(parent: Mapping[str, Any], key: str, label: str) -> float:
    if key not in parent:
        raise ValueError(f"{label}.{key} is required")
    try:
        return float(parent[key])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label}.{key} must be numeric") from exc


def placement_vector(parent: Mapping[str, Any], label: str) -> np.ndarray:
    placement = required_mapping(parent, "placement_m", label)
    return v3(
        required_float(placement, "x", f"{label}.placement_m"),
        required_float(placement, "y", f"{label}.placement_m"),
        required_float(placement, "z", f"{label}.placement_m"),
    )


def load_materials(path: Path) -> tuple[dict[str, Material], str, str]:
    data = yaml_mapping(path, "materials")
    materials_raw = required_mapping(data, "materials", "materials")
    materials = {
        name: Material(
            name=str(name),
            density_kg_m3=required_float(as_mapping(item, f"materials.{name}"), "density_kg_m3", f"materials.{name}"),
            properties=as_mapping(item, f"materials.{name}"),
        )
        for name, item in materials_raw.items()
    }
    structural = str(data.get("default_structural_material", ""))
    elastomer = str(data.get("default_elastomer_material", ""))
    if structural not in materials:
        raise ValueError("default_structural_material must name a material")
    if elastomer not in materials:
        raise ValueError("default_elastomer_material must name a material")
    return materials, structural, elastomer


def load_actuators(path: Path) -> tuple[dict[str, Actuator], dict[str, str]]:
    data = yaml_mapping(path, "actuators")
    actuators_raw = required_mapping(data, "actuators", "actuators")
    actuators = {}
    for name, item in actuators_raw.items():
        item_map = as_mapping(item, f"actuators.{name}")
        actuators[str(name)] = Actuator(
            name=str(name),
            mass_kg=required_float(item_map, "mass_kg", f"actuators.{name}"),
            driver_mass_kg=required_float(item_map, "driver_mass_kg", f"actuators.{name}"),
            continuous_torque_nm=required_float(item_map, "continuous_torque_nm", f"actuators.{name}"),
            max_torque_nm=required_float(item_map, "max_torque_nm", f"actuators.{name}"),
            properties=item_map,
        )
    assignments = {str(k): str(v) for k, v in required_mapping(data, "default_actuator_set", "actuators").items()}
    for joint_type, actuator_name in assignments.items():
        if actuator_name not in actuators:
            raise ValueError(f"default_actuator_set.{joint_type} references unknown actuator {actuator_name}")
    return actuators, assignments


def load_battery_and_electronics(path: Path) -> tuple[Battery, list[ElectronicsItem]]:
    data = yaml_mapping(path, "batteries")
    batteries_raw = required_mapping(data, "batteries", "batteries")
    default_battery = str(data.get("default_battery", ""))
    if default_battery not in batteries_raw:
        raise ValueError("default_battery must name a battery")
    battery_map = as_mapping(batteries_raw[default_battery], f"batteries.{default_battery}")
    battery = Battery(
        name=default_battery,
        mass_kg=required_float(battery_map, "mass_kg", f"batteries.{default_battery}"),
        placement_m=placement_vector(battery_map, f"batteries.{default_battery}"),
        properties=battery_map,
    )

    electronics = []
    for name, item in required_mapping(data, "electronics", "batteries").items():
        item_map = as_mapping(item, f"electronics.{name}")
        electronics.append(
            ElectronicsItem(
                name=str(name),
                mass_kg=required_float(item_map, "mass_kg", f"electronics.{name}"),
                placement_m=placement_vector(item_map, f"electronics.{name}"),
                properties=item_map,
            )
        )
    return battery, electronics


def load_catalog(materials_path: Path, actuators_path: Path, batteries_path: Path) -> PhysicalCatalog:
    materials, structural, elastomer = load_materials(materials_path)
    actuators, assignments = load_actuators(actuators_path)
    battery, electronics = load_battery_and_electronics(batteries_path)
    return PhysicalCatalog(
        materials=materials,
        structural_material=structural,
        elastomer_material=elastomer,
        actuators=actuators,
        actuator_assignments=assignments,
        battery=battery,
        electronics=electronics,
    )


def robot_geometry(description: DogDescription) -> RobotGeometry:
    return RobotGeometry(**description.geometry.robot_geometry_kwargs())


def make_body_pose(g: RobotGeometry, body_z_m: float, waist_yaw_deg: float, waist_pitch_deg: float) -> BodyPose:
    waist_yaw_rad = math.radians(waist_yaw_deg)
    waist_pitch_rad = math.radians(waist_pitch_deg)
    yaw_xy, pitch_xy = waist_joint_points(g, waist_yaw_rad)
    yaw_joint = v3(float(yaw_xy[0]), float(yaw_xy[1]), body_z_m)
    pitch_joint = v3(float(pitch_xy[0]), float(pitch_xy[1]), body_z_m)

    rear_forward = np.array([1.0, 0.0, 0.0])
    rear_left = np.array([0.0, 1.0, 0.0])
    rear_up = np.array([0.0, 0.0, 1.0])

    front_yaw = rot_z(waist_yaw_rad)
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
        front_forward=front_forward,
        front_left=front_left,
        front_up=front_up,
        hips=hips,
        bases=bases,
    )


def leg_endpoint_targets(g: RobotGeometry, waist_yaw_deg: float) -> dict[str, np.ndarray]:
    waist_rad = math.radians(waist_yaw_deg)
    foot_xy = foot_points(g, waist_rad)
    return {name: v3(float(point[0]), float(point[1]), 0.0) for name, point in foot_xy.items()}


def solve_leg_chain(
    description: DogDescription,
    pose: BodyPose,
    leg_name: str,
    toe_endpoint: np.ndarray,
) -> LegChain:
    hip = pose.hips[leg_name]
    forward, _outward, down = pose.bases[leg_name]
    links = description.viewer.links
    hip_to_foot = toe_endpoint - hip
    toe_axis = normalized(hip_to_foot)
    toe_joint = toe_endpoint - toe_axis * links.distal_endpoint_m

    target = toe_joint - hip
    x = float(np.dot(target, forward))
    z = float(np.dot(target, down))
    planar_dist = math.hypot(x, z)
    upper = links.upper_m
    lower = links.lower_m
    min_reach = abs(upper - lower) + 1e-9
    max_reach = upper + lower - 1e-9
    clamped_dist = min(max(planar_dist, min_reach), max_reach)
    stretch = max(0.0, planar_dist - max_reach) + max(0.0, min_reach - planar_dist)

    target_angle = math.atan2(z, x)
    cos_alpha = (upper * upper + clamped_dist * clamped_dist - lower * lower) / (2.0 * upper * clamped_dist)
    alpha = math.acos(max(-1.0, min(1.0, cos_alpha)))
    knee_angle = target_angle + alpha
    knee = hip + forward * (upper * math.cos(knee_angle)) + down * (upper * math.sin(knee_angle))
    return LegChain(
        hip=hip,
        knee=knee,
        toe_joint=toe_joint,
        toe_endpoint=toe_endpoint,
        ik_stretch_m=stretch,
    )


def make_head_chain(description: DogDescription, pose: BodyPose) -> HeadChain:
    claw = description.viewer.head_claw
    ranges = description.joint_ranges
    neck_yaw = math.radians(ranges["neck_yaw"].bias_deg)
    neck_pitch = math.radians(ranges["neck_pitch"].bias_deg)
    claw_open = math.radians(ranges["head_claw"].bias_deg)

    base_forward = pose.front_forward
    base_left = pose.front_left
    up = pose.front_up
    yaw_forward = base_forward * math.cos(neck_yaw) + base_left * math.sin(neck_yaw)
    yaw_left = -base_forward * math.sin(neck_yaw) + base_left * math.cos(neck_yaw)
    forward = normalized(yaw_forward * math.cos(neck_pitch) + up * math.sin(neck_pitch))
    left = normalized(yaw_left)
    local_up = normalized(np.cross(forward, left))

    root = pose.front_mid
    neck_origin = root + base_forward * claw.root_forward_m + up * claw.root_up_m
    hinge = neck_origin + forward * claw.neck_length_m
    upper_hinge = hinge + local_up * claw.hinge_half_gap_m
    lower_hinge = hinge - local_up * claw.hinge_half_gap_m
    upper_tip = upper_hinge + claw.jaw_length_m * (forward * math.cos(claw_open) + local_up * math.sin(claw_open))
    lower_tip = lower_hinge + claw.jaw_length_m * (forward * math.cos(claw_open) - local_up * math.sin(claw_open))
    return HeadChain(
        root=root,
        hinge=hinge,
        upper_hinge=upper_hinge,
        lower_hinge=lower_hinge,
        upper_tip=upper_tip,
        lower_tip=lower_tip,
    )


def box_mass(material: Material, length_m: float, width_m: float, height_m: float, fill_fraction: float) -> float:
    return material.density_kg_m3 * length_m * width_m * height_m * fill_fraction


def strut_mass(material: Material, length_m: float, width_m: float, depth_m: float, fill_fraction: float) -> float:
    return material.density_kg_m3 * length_m * width_m * depth_m * fill_fraction


def add_actuator_element(
    elements: list[MassElement],
    catalog: PhysicalCatalog,
    joint_type: str,
    name: str,
    position: np.ndarray,
    leg: str | None = None,
    distal_level: int | None = None,
) -> None:
    actuator_name = catalog.actuator_assignments[joint_type]
    actuator = catalog.actuators[actuator_name]
    total_mass = actuator.mass_kg + actuator.driver_mass_kg
    elements.append(
        MassElement(
            name=name,
            kind="actuator",
            mass_kg=total_mass,
            com_m=position,
            leg=leg,
            distal_level=distal_level,
            notes=f"{actuator_name}; includes driver mass",
        )
    )


def build_mass_model(
    case_name: str,
    description: DogDescription,
    catalog: PhysicalCatalog,
    assumptions: Stage1Assumptions,
    waist_yaw_deg: float,
    waist_pitch_deg: float,
) -> MassModel:
    g = robot_geometry(description)
    pose = make_body_pose(g, description.viewer.body_z_m, waist_yaw_deg, waist_pitch_deg)
    targets = leg_endpoint_targets(g, waist_yaw_deg)
    legs = {name: solve_leg_chain(description, pose, name, targets[name]) for name in LEG_ORDER}
    head = make_head_chain(description, pose)
    structural = catalog.materials[catalog.structural_material]
    elastomer = catalog.materials[catalog.elastomer_material]

    body_width = 2.0 * g.body_half_width
    front_body_com = pose.pitch_joint + pose.front_forward * (0.5 * g.front_body_length)
    rear_body_com = pose.yaw_joint - np.array([1.0, 0.0, 0.0]) * (0.5 * g.rear_body_length)
    waist_com = 0.5 * (pose.yaw_joint + pose.pitch_joint)

    elements = [
        MassElement(
            name="front_body_shell",
            kind="printed_structure",
            mass_kg=box_mass(
                structural,
                g.front_body_length,
                body_width,
                assumptions.body_height_m,
                assumptions.body_equivalent_fill_fraction,
            ),
            com_m=front_body_com,
            material=structural.name,
            notes="equivalent box shell/rib placeholder",
        ),
        MassElement(
            name="rear_body_shell",
            kind="printed_structure",
            mass_kg=box_mass(
                structural,
                g.rear_body_length,
                body_width,
                assumptions.body_height_m,
                assumptions.body_equivalent_fill_fraction,
            ),
            com_m=rear_body_com,
            material=structural.name,
            notes="equivalent box shell/rib placeholder",
        ),
        MassElement(
            name="waist_link_shell",
            kind="printed_structure",
            mass_kg=box_mass(
                structural,
                g.waist_joint_spacing,
                body_width,
                assumptions.body_height_m,
                assumptions.waist_equivalent_fill_fraction,
            ),
            com_m=waist_com,
            material=structural.name,
            notes="short yaw-pitch bridge placeholder",
        ),
        MassElement(
            name=f"battery_{catalog.battery.name}",
            kind="battery",
            mass_kg=catalog.battery.mass_kg,
            com_m=catalog.battery.placement_m,
            notes="battery placement from batteries.yaml",
        ),
    ]

    for item in catalog.electronics:
        elements.append(
            MassElement(
                name=f"electronics_{item.name}",
                kind="electronics",
                mass_kg=item.mass_kg,
                com_m=item.placement_m,
                notes="electronics placement from batteries.yaml",
            )
        )

    add_actuator_element(elements, catalog, "waist_yaw", "waist_yaw_actuator", pose.yaw_joint)
    add_actuator_element(elements, catalog, "waist_pitch", "waist_pitch_actuator", pose.pitch_joint)

    for leg_name, chain in legs.items():
        upper_len = description.viewer.links.upper_m
        lower_len = description.viewer.links.lower_m
        toe_len = description.viewer.links.distal_endpoint_m
        elements.extend(
            [
                MassElement(
                    name=f"{leg_name}_upper_link",
                    kind="printed_structure",
                    mass_kg=strut_mass(
                        structural,
                        upper_len,
                        assumptions.leg_link_width_m,
                        assumptions.leg_link_depth_m,
                        assumptions.leg_equivalent_fill_fraction,
                    ),
                    com_m=0.5 * (chain.hip + chain.knee),
                    material=structural.name,
                    leg=leg_name,
                    distal_level=1,
                ),
                MassElement(
                    name=f"{leg_name}_lower_link",
                    kind="printed_structure",
                    mass_kg=strut_mass(
                        structural,
                        lower_len,
                        assumptions.leg_link_width_m,
                        assumptions.leg_link_depth_m,
                        assumptions.leg_equivalent_fill_fraction,
                    ),
                    com_m=0.5 * (chain.knee + chain.toe_joint),
                    material=structural.name,
                    leg=leg_name,
                    distal_level=2,
                ),
                MassElement(
                    name=f"{leg_name}_toe_link",
                    kind="printed_structure",
                    mass_kg=strut_mass(
                        structural,
                        toe_len,
                        assumptions.toe_link_width_m,
                        assumptions.toe_link_depth_m,
                        assumptions.toe_link_equivalent_fill_fraction,
                    ),
                    com_m=0.5 * (chain.toe_joint + chain.toe_endpoint),
                    material=structural.name,
                    leg=leg_name,
                    distal_level=3,
                ),
                MassElement(
                    name=f"{leg_name}_foot_pad",
                    kind="foot_pad",
                    mass_kg=assumptions.foot_pad_mass_kg,
                    com_m=chain.toe_endpoint,
                    material=elastomer.name,
                    leg=leg_name,
                    distal_level=4,
                    notes="fixed placeholder pad mass",
                ),
            ]
        )
        add_actuator_element(elements, catalog, "hip_ab", f"{leg_name}_hip_ab_actuator", chain.hip, leg_name, 0)
        add_actuator_element(elements, catalog, "hip_pitch", f"{leg_name}_hip_pitch_actuator", chain.hip, leg_name, 0)
        add_actuator_element(elements, catalog, "knee_bend", f"{leg_name}_knee_bend_actuator", chain.knee, leg_name, 1)
        add_actuator_element(elements, catalog, "toe_bend", f"{leg_name}_toe_bend_actuator", chain.toe_joint, leg_name, 2)

    neck_len = float(np.linalg.norm(head.hinge - head.root))
    upper_jaw_len = float(np.linalg.norm(head.upper_tip - head.upper_hinge))
    lower_jaw_len = float(np.linalg.norm(head.lower_tip - head.lower_hinge))
    elements.extend(
        [
            MassElement(
                name="neck_link",
                kind="printed_structure",
                mass_kg=strut_mass(
                    structural,
                    neck_len,
                    assumptions.head_link_width_m,
                    assumptions.head_link_depth_m,
                    assumptions.head_equivalent_fill_fraction,
                ),
                com_m=0.5 * (head.root + head.hinge),
                material=structural.name,
                notes="head-claw neck placeholder",
            ),
            MassElement(
                name="upper_claw_jaw",
                kind="printed_structure",
                mass_kg=strut_mass(
                    structural,
                    upper_jaw_len,
                    assumptions.head_link_width_m,
                    assumptions.head_link_depth_m,
                    assumptions.head_equivalent_fill_fraction,
                ),
                com_m=0.5 * (head.upper_hinge + head.upper_tip),
                material=structural.name,
                notes="head-claw jaw placeholder",
            ),
            MassElement(
                name="lower_claw_jaw",
                kind="printed_structure",
                mass_kg=strut_mass(
                    structural,
                    lower_jaw_len,
                    assumptions.head_link_width_m,
                    assumptions.head_link_depth_m,
                    assumptions.head_equivalent_fill_fraction,
                ),
                com_m=0.5 * (head.lower_hinge + head.lower_tip),
                material=structural.name,
                notes="head-claw jaw placeholder",
            ),
        ]
    )
    add_actuator_element(elements, catalog, "neck_yaw", "neck_yaw_actuator", head.root)
    add_actuator_element(elements, catalog, "neck_pitch", "neck_pitch_actuator", head.root)
    add_actuator_element(elements, catalog, "head_claw", "head_claw_actuator", head.hinge)

    return MassModel(
        case_name=case_name,
        geometry=g,
        pose=pose,
        legs=legs,
        head=head,
        elements=elements,
    )


def joint_type_for_name(joint: str) -> str:
    if joint in {"waist_yaw", "waist_pitch", "neck_yaw", "neck_pitch", "head_claw"}:
        return joint
    for leg_name in LEG_ORDER:
        prefix = f"{leg_name}_"
        if joint.startswith(prefix):
            return joint.removeprefix(prefix)
    raise ValueError(f"Cannot infer joint type for {joint}")


def actuator_for_joint(catalog: PhysicalCatalog, joint: str) -> Actuator:
    joint_type = joint_type_for_name(joint)
    return catalog.actuators[catalog.actuator_assignments[joint_type]]


def margin(limit: float, required: float) -> float | None:
    if required <= 1e-9:
        return None
    return limit / required


def point_mass_inertia_about_axis(point: np.ndarray, joint: np.ndarray, axis: np.ndarray, mass_kg: float) -> float:
    axis = normalized(axis)
    r = point - joint
    perpendicular = r - axis * float(np.dot(r, axis))
    return mass_kg * float(np.dot(perpendicular, perpendicular))


def inertia_torque(
    elements: list[MassElement],
    joint: np.ndarray,
    axis: np.ndarray,
    angular_accel_rad_s2: float,
) -> float:
    inertia = sum(
        point_mass_inertia_about_axis(element.com_m, joint, axis, element.mass_kg)
        for element in elements
    )
    return abs(inertia * angular_accel_rad_s2)


def leg_distal_elements(model: MassModel, leg_name: str, min_level: int) -> list[MassElement]:
    return [
        element
        for element in model.elements
        if element.leg == leg_name and element.distal_level is not None and element.distal_level >= min_level
    ]


def front_yaw_elements(model: MassModel) -> list[MassElement]:
    names = {
        "front_body_shell",
        "waist_link_shell",
        "waist_pitch_actuator",
        "neck_link",
        "upper_claw_jaw",
        "lower_claw_jaw",
        "neck_yaw_actuator",
        "neck_pitch_actuator",
        "head_claw_actuator",
    }
    return [
        element
        for element in model.elements
        if element.name in names or (element.leg is not None and element.leg.startswith("front"))
    ]


def front_pitch_elements(model: MassModel) -> list[MassElement]:
    names = {
        "front_body_shell",
        "neck_link",
        "upper_claw_jaw",
        "lower_claw_jaw",
        "neck_yaw_actuator",
        "neck_pitch_actuator",
        "head_claw_actuator",
    }
    return [
        element
        for element in model.elements
        if element.name in names or (element.leg is not None and element.leg.startswith("front"))
    ]


def named_elements(model: MassModel, names: set[str]) -> list[MassElement]:
    return [element for element in model.elements if element.name in names]


def estimate_torques(
    model: MassModel,
    catalog: PhysicalCatalog,
    assumptions: Stage1Assumptions | None = None,
) -> list[TorqueRow]:
    assumptions = assumptions or Stage1Assumptions()
    rows: list[TorqueRow] = []

    raw: dict[str, tuple[float, str]] = {
        "waist_yaw": (
            inertia_torque(
                front_yaw_elements(model),
                model.pose.yaw_joint,
                v3(0.0, 0.0, 1.0),
                assumptions.waist_angular_accel_rad_s2,
            ),
            "free-space point-mass inertia about waist yaw axis",
        ),
        "waist_pitch": (
            inertia_torque(
                front_pitch_elements(model),
                model.pose.pitch_joint,
                model.pose.front_left,
                assumptions.waist_angular_accel_rad_s2,
            ),
            "free-space point-mass inertia about waist pitch axis",
        ),
    }

    for leg_name in LEG_ORDER:
        chain = model.legs[leg_name]
        forward, outward, _down = model.pose.bases[leg_name]
        joint_specs = {
            "hip_ab": (chain.hip, forward, 1),
            "hip_pitch": (chain.hip, outward, 1),
            "knee_bend": (chain.knee, outward, 2),
            "toe_bend": (chain.toe_joint, outward, 3),
        }
        for joint_type, (joint, axis, min_level) in joint_specs.items():
            raw[f"{leg_name}_{joint_type}"] = (
                inertia_torque(
                    leg_distal_elements(model, leg_name, min_level),
                    joint,
                    axis,
                    assumptions.joint_angular_accel_rad_s2,
                ),
                f"free-space point-mass inertia about {joint_type} axis",
            )

    head_names = {"neck_link", "upper_claw_jaw", "lower_claw_jaw", "head_claw_actuator"}
    claw_names = {"upper_claw_jaw", "lower_claw_jaw"}
    raw["neck_yaw"] = (
        inertia_torque(
            named_elements(model, head_names),
            model.head.root,
            model.pose.front_up,
            assumptions.neck_angular_accel_rad_s2,
        ),
        "free-space point-mass inertia about neck yaw axis",
    )
    raw["neck_pitch"] = (
        inertia_torque(
            named_elements(model, head_names),
            model.head.root,
            model.pose.front_left,
            assumptions.neck_angular_accel_rad_s2,
        ),
        "free-space point-mass inertia about neck pitch axis",
    )
    raw["head_claw"] = (
        inertia_torque(
            named_elements(model, claw_names),
            model.head.hinge,
            model.pose.front_left,
            assumptions.neck_angular_accel_rad_s2,
        ),
        "free-space point-mass inertia about claw hinge axis",
    )

    for joint, (required, notes) in raw.items():
        actuator = actuator_for_joint(catalog, joint)
        rows.append(
            TorqueRow(
                case_name=model.case_name,
                joint=joint,
                actuator=actuator.name,
                required_torque_nm=required,
                continuous_torque_nm=actuator.continuous_torque_nm,
                max_torque_nm=actuator.max_torque_nm,
                continuous_margin=margin(actuator.continuous_torque_nm, required),
                max_margin=margin(actuator.max_torque_nm, required),
                notes=notes,
            )
        )
    return rows


def representative_cases(args: argparse.Namespace) -> list[tuple[str, float, float]]:
    return [
        ("neutral_pose", 0.0, 0.0),
        ("yaw_left_pose", args.worst_waist_yaw, 0.0),
        ("yaw_right_pose", -args.worst_waist_yaw, 0.0),
        ("pitch_up_pose", 0.0, args.worst_waist_pitch),
        ("pitch_down_pose", 0.0, -args.worst_waist_pitch),
    ]


def fmt_margin(value: float | None) -> str:
    if value is None:
        return ""
    return f"{value:.6g}"


def write_mass_elements_csv(model: MassModel, path: Path) -> None:
    fields = ["name", "kind", "mass_kg", "com_x_m", "com_y_m", "com_z_m", "material", "leg", "distal_level", "notes"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for element in model.elements:
            writer.writerow(
                {
                    "name": element.name,
                    "kind": element.kind,
                    "mass_kg": f"{element.mass_kg:.9g}",
                    "com_x_m": f"{element.com_m[0]:.9g}",
                    "com_y_m": f"{element.com_m[1]:.9g}",
                    "com_z_m": f"{element.com_m[2]:.9g}",
                    "material": element.material,
                    "leg": element.leg or "",
                    "distal_level": "" if element.distal_level is None else element.distal_level,
                    "notes": element.notes,
                }
            )


def write_torque_csv(rows: list[TorqueRow], path: Path) -> None:
    fields = [
        "case_name",
        "joint",
        "actuator",
        "required_torque_nm",
        "continuous_torque_nm",
        "max_torque_nm",
        "continuous_margin",
        "max_margin",
        "notes",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "case_name": row.case_name,
                    "joint": row.joint,
                    "actuator": row.actuator,
                    "required_torque_nm": f"{row.required_torque_nm:.9g}",
                    "continuous_torque_nm": f"{row.continuous_torque_nm:.9g}",
                    "max_torque_nm": f"{row.max_torque_nm:.9g}",
                    "continuous_margin": fmt_margin(row.continuous_margin),
                    "max_margin": fmt_margin(row.max_margin),
                    "notes": row.notes,
                }
            )


def case_torque_summary(case_name: str, rows: list[TorqueRow]) -> tuple[TorqueRow, TorqueRow]:
    case_rows = [row for row in rows if row.case_name == case_name]
    finite_rows = [row for row in case_rows if row.continuous_margin is not None]
    if not case_rows or not finite_rows:
        raise ValueError(f"No finite torque rows found for case {case_name}")
    worst_margin = min(finite_rows, key=lambda row: row.continuous_margin or math.inf)
    worst_required = max(case_rows, key=lambda row: row.required_torque_nm)
    return worst_margin, worst_required


def write_case_summary_csv(case_records: list[CaseRecord], rows: list[TorqueRow], path: Path) -> None:
    fields = [
        "case_name",
        "waist_yaw_deg",
        "waist_pitch_deg",
        "total_mass_kg",
        "com_x_m",
        "com_y_m",
        "com_z_m",
        "worst_margin_joint",
        "worst_continuous_margin",
        "max_required_joint",
        "max_required_torque_nm",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for record in case_records:
            worst_margin, worst_required = case_torque_summary(record.case_name, rows)
            com = record.model.com_m
            writer.writerow(
                {
                    "case_name": record.case_name,
                    "waist_yaw_deg": f"{record.waist_yaw_deg:.9g}",
                    "waist_pitch_deg": f"{record.waist_pitch_deg:.9g}",
                    "total_mass_kg": f"{record.model.total_mass_kg:.9g}",
                    "com_x_m": f"{com[0]:.9g}",
                    "com_y_m": f"{com[1]:.9g}",
                    "com_z_m": f"{com[2]:.9g}",
                    "worst_margin_joint": worst_margin.joint,
                    "worst_continuous_margin": fmt_margin(worst_margin.continuous_margin),
                    "max_required_joint": worst_required.joint,
                    "max_required_torque_nm": f"{worst_required.required_torque_nm:.9g}",
                }
            )


def summary_dict(
    model: MassModel,
    rows: list[TorqueRow],
    catalog: PhysicalCatalog,
    case_records: list[CaseRecord],
) -> dict[str, Any]:
    finite_rows = [row for row in rows if row.continuous_margin is not None]
    worst_margin = min(finite_rows, key=lambda row: row.continuous_margin or math.inf)
    inertia = model.inertia_about_com_kg_m2
    return {
        "stage": "stage_1_free_motion_mass_and_torque_body",
        "assumption_level": "rough placeholders; update from CAD, datasheets, and material coupons",
        "materials": {
            "structural": catalog.structural_material,
            "elastomer": catalog.elastomer_material,
        },
        "battery": catalog.battery.name,
        "total_mass_kg": float(model.total_mass_kg),
        "com_m": {
            "x": float(model.com_m[0]),
            "y": float(model.com_m[1]),
            "z": float(model.com_m[2]),
        },
        "point_mass_inertia_about_com_kg_m2": {
            "ixx": float(inertia[0, 0]),
            "iyy": float(inertia[1, 1]),
            "izz": float(inertia[2, 2]),
            "ixy": float(inertia[0, 1]),
            "ixz": float(inertia[0, 2]),
            "iyz": float(inertia[1, 2]),
        },
        "worst_continuous_margin": {
            "case_name": worst_margin.case_name,
            "joint": worst_margin.joint,
            "actuator": worst_margin.actuator,
            "required_torque_nm": float(worst_margin.required_torque_nm),
            "continuous_margin": None
            if worst_margin.continuous_margin is None
            else float(worst_margin.continuous_margin),
        },
        "case_summaries": [
            {
                "case_name": record.case_name,
                "waist_yaw_deg": float(record.waist_yaw_deg),
                "waist_pitch_deg": float(record.waist_pitch_deg),
                "com_m": {
                    "x": float(record.model.com_m[0]),
                    "y": float(record.model.com_m[1]),
                    "z": float(record.model.com_m[2]),
                },
                "worst_margin_joint": case_torque_summary(record.case_name, rows)[0].joint,
                "worst_continuous_margin": case_torque_summary(record.case_name, rows)[0].continuous_margin,
            }
            for record in case_records
        ],
        "notes": [
            "torque estimate is free-space point-mass inertia times assumed joint angular acceleration",
            "no ground, support polygon, foot contact, or gravity reaction is included",
            "inertia is still a point-mass approximation until CAD-derived tensors exist",
        ],
    }


def write_summary_yaml(
    model: MassModel,
    rows: list[TorqueRow],
    catalog: PhysicalCatalog,
    case_records: list[CaseRecord],
    path: Path,
) -> None:
    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(summary_dict(model, rows, catalog, case_records), handle, sort_keys=False)


def print_summary(out_dir: Path, neutral_model: MassModel, rows: list[TorqueRow]) -> None:
    finite_rows = [row for row in rows if row.continuous_margin is not None]
    worst_margin = min(finite_rows, key=lambda row: row.continuous_margin or math.inf)
    worst_required = max(rows, key=lambda row: row.required_torque_nm)
    com = neutral_model.com_m
    print(f"Wrote outputs to: {out_dir}")
    print(f"Total mass:        {neutral_model.total_mass_kg:.3f} kg")
    print(f"COM:               x={com[0]:+.3f} m, y={com[1]:+.3f} m, z={com[2]:+.3f} m")
    print(
        "Worst margin:      "
        f"{worst_margin.case_name} / {worst_margin.joint}, "
        f"required={worst_margin.required_torque_nm:.3f} Nm, "
        f"continuous margin={worst_margin.continuous_margin:.2f}x"
    )
    print(
        "Largest torque:    "
        f"{worst_required.case_name} / {worst_required.joint}, "
        f"required={worst_required.required_torque_nm:.3f} Nm"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_DESCRIPTION_PATH, help="dog description YAML")
    parser.add_argument("--materials", type=Path, default=DEFAULT_MATERIALS_PATH, help="material catalog YAML")
    parser.add_argument("--actuators", type=Path, default=DEFAULT_ACTUATORS_PATH, help="actuator catalog YAML")
    parser.add_argument("--batteries", type=Path, default=DEFAULT_BATTERIES_PATH, help="battery/electronics catalog YAML")
    parser.add_argument("--out-dir", type=Path, default=Path("mass_outputs"))
    parser.add_argument("--worst-waist-yaw", type=float, default=35.0, help="representative yaw load case, degrees")
    parser.add_argument("--worst-waist-pitch", type=float, default=14.0, help="representative pitch load case, degrees")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    description = load_dog_description(args.config)
    catalog = load_catalog(args.materials, args.actuators, args.batteries)
    assumptions = Stage1Assumptions()

    neutral_model: MassModel | None = None
    case_records: list[CaseRecord] = []
    all_rows: list[TorqueRow] = []
    for case_name, waist_yaw_deg, waist_pitch_deg in representative_cases(args):
        model = build_mass_model(
            case_name,
            description,
            catalog,
            assumptions,
            waist_yaw_deg,
            waist_pitch_deg,
        )
        if case_name == "neutral_pose":
            neutral_model = model
        case_records.append(
            CaseRecord(
                case_name=case_name,
                waist_yaw_deg=waist_yaw_deg,
                waist_pitch_deg=waist_pitch_deg,
                model=model,
            )
        )
        all_rows.extend(estimate_torques(model, catalog, assumptions))

    if neutral_model is None:
        raise RuntimeError("neutral_pose case was not generated")

    write_mass_elements_csv(neutral_model, out_dir / "mass_elements.csv")
    write_torque_csv(all_rows, out_dir / "joint_torque_estimate.csv")
    write_case_summary_csv(case_records, all_rows, out_dir / "case_summary.csv")
    write_summary_yaml(neutral_model, all_rows, catalog, case_records, out_dir / "mass_summary.yaml")
    print_summary(out_dir, neutral_model, all_rows)


if __name__ == "__main__":
    main()
