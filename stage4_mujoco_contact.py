#!/usr/bin/env python3
"""Stage 4 MuJoCo/contact first-light export and load feedback.

This stage creates a MuJoCo-ready rough model from the existing Stage 1/2/3
scaffolds and writes a contact/support load table that can be fed back into
future Stage 2 load-case generation. The default path is intentionally light:
it does not require the Python ``mujoco`` package. If ``--run-mujoco`` is used
and the package is installed, a short smoke simulation is attempted.
"""

from __future__ import annotations

import argparse
import csv
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml

from dog_description import DEFAULT_DESCRIPTION_PATH, DogDescription, load_dog_description
from endpoint_geometry import LEG_ORDER
import mass_model as stage1
import stage2_rod_model as rods
import stage3_ik_control as stage3


DEFAULT_OUT_DIR = Path("stage4_outputs/mujoco_contact")
GRAVITY_M_S2 = 9.80665
DEFAULT_FOOT_FRICTION = 0.90
DEFAULT_FOOT_RADIUS_M = 0.012
STATIC_RESIDUAL_FRACTION = 0.01


@dataclass(frozen=True)
class Stage4JointSpec:
    name: str
    joint_type: str
    position_m: np.ndarray
    axis: np.ndarray
    min_deg: float
    max_deg: float
    continuous_torque_nm: float
    max_torque_nm: float
    max_speed_rad_s: float


@dataclass(frozen=True)
class Stage4HingeExport:
    name: str
    driver_name: str
    coefficient: float
    axis: np.ndarray
    min_deg: float
    max_deg: float


@dataclass(frozen=True)
class ContactLoadRow:
    frame_index: int
    time_s: float
    primitive: str
    leg: str
    support_state: str
    foot_position_m: np.ndarray
    normal_force_n: float
    tangential_force_n: float
    friction_coeff: float
    friction_utilization: float
    static_solvable: bool
    support_polygon_margin_m: float
    equilibrium_residual_n: float


@dataclass(frozen=True)
class MujocoSmokeResult:
    attempted: bool
    mujoco_available: bool
    completed: bool
    steps: int = 0
    final_time_s: float = 0.0
    max_contacts: int = 0
    min_root_z_m: float | None = None
    error: str = ""


@dataclass(frozen=True)
class MujocoContactCsvResult:
    attempted: bool
    mujoco_available: bool
    completed: bool
    steps: int = 0
    rows: int = 0
    max_contacts: int = 0
    error: str = ""


@dataclass(frozen=True)
class Stage4Case:
    primitive: str
    stage3_case: stage3.Stage3Case
    rod_model: rods.RodModel
    joint_specs: tuple[Stage4JointSpec, ...]
    contact_rows: tuple[ContactLoadRow, ...]
    out_dir: Path
    viewer_safe: bool
    xml_path: Path
    mjcf_path: Path
    contact_csv_path: Path
    contact_proxy_csv_path: Path
    stage2_feedback_csv_path: Path
    summary_path: Path
    mujoco_smoke_csv_path: Path
    contact_csv_result: MujocoContactCsvResult
    smoke_result: MujocoSmokeResult


def fmt(value: float | None) -> str:
    if value is None:
        return ""
    if not math.isfinite(value):
        return str(value)
    return f"{value:.9g}"


def xml_float(value: float) -> str:
    return f"{value:.10g}"


def xml_vec(values: Iterable[float]) -> str:
    return " ".join(xml_float(float(value)) for value in values)


def normalized(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm <= 1.0e-12:
        return np.zeros(3)
    return np.array(vector, dtype=float) / norm


def actuator_for_joint_type(catalog: stage1.PhysicalCatalog, joint_type: str) -> stage1.Actuator:
    return catalog.actuators[catalog.actuator_assignments[joint_type]]


def make_joint_spec(
    description: DogDescription,
    catalog: stage1.PhysicalCatalog,
    name: str,
    joint_type: str,
    position_m: np.ndarray,
    axis: np.ndarray,
    current_deg: float = 0.0,
) -> Stage4JointSpec:
    spec = description.joint_ranges[joint_type]
    actuator = actuator_for_joint_type(catalog, joint_type)
    speed = float(actuator.properties.get("max_speed_rad_s", 0.0))
    return Stage4JointSpec(
        name=name,
        joint_type=joint_type,
        position_m=np.array(position_m, dtype=float),
        axis=normalized(axis),
        min_deg=float(spec.min_deg - current_deg),
        max_deg=float(spec.max_deg - current_deg),
        continuous_torque_nm=float(actuator.continuous_torque_nm),
        max_torque_nm=float(actuator.max_torque_nm),
        max_speed_rad_s=speed,
    )


def hinge_export_from_joint(
    spec: Stage4JointSpec,
    *,
    name: str | None = None,
    axis: np.ndarray | None = None,
    coefficient: float = 1.0,
) -> Stage4HingeExport:
    low = coefficient * spec.min_deg
    high = coefficient * spec.max_deg
    return Stage4HingeExport(
        name=spec.name if name is None else name,
        driver_name=spec.name,
        coefficient=float(coefficient),
        axis=normalized(spec.axis if axis is None else axis),
        min_deg=min(low, high),
        max_deg=max(low, high),
    )


def add_hinge_joint(body: ET.Element, hinge: Stage4HingeExport) -> None:
    ET.SubElement(
        body,
        "joint",
        name=hinge.name,
        type="hinge",
        axis=xml_vec(hinge.axis),
        range=f"{xml_float(hinge.min_deg)} {xml_float(hinge.max_deg)}",
    )


def is_following_hinge(hinge: Stage4HingeExport) -> bool:
    return hinge.name != hinge.driver_name or abs(hinge.coefficient - 1.0) > 1.0e-12


def add_joint_follower_equalities(root: ET.Element, hinges: Iterable[Stage4HingeExport]) -> None:
    followers = [hinge for hinge in hinges if is_following_hinge(hinge)]
    if not followers:
        return
    equality = ensure_xml_section(root, "equality")
    for hinge in followers:
        ET.SubElement(
            equality,
            "joint",
            name=f"{hinge.name}_follows_{hinge.driver_name}",
            joint1=hinge.driver_name,
            joint2=hinge.name,
            polycoef=f"0 {xml_float(hinge.coefficient)} 0 0 0",
            solref="0.01 1",
            solimp="0.95 0.99 0.001",
        )


def build_joint_specs(
    description: DogDescription,
    catalog: stage1.PhysicalCatalog,
    frame: stage3.TrajectoryFrame,
) -> tuple[Stage4JointSpec, ...]:
    model = frame.model
    ranges = description.joint_ranges
    specs: list[Stage4JointSpec] = [
        make_joint_spec(
            description,
            catalog,
            "waist_yaw",
            "waist_yaw",
            model.pose.yaw_joint,
            np.array([0.0, 0.0, 1.0]),
            frame.waist_yaw_deg,
        ),
        make_joint_spec(
            description,
            catalog,
            "waist_pitch",
            "waist_pitch",
            model.pose.pitch_joint,
            model.pose.front_left,
            frame.waist_pitch_deg,
        ),
    ]
    for leg_name in LEG_ORDER:
        chain = model.legs[leg_name]
        forward, outward, _down = model.pose.bases[leg_name]
        angles = frame.ik_solutions[leg_name].angles_rad
        specs.extend(
            [
                make_joint_spec(
                    description,
                    catalog,
                    f"{leg_name}_hip_ab",
                    "hip_ab",
                    chain.hip,
                    forward,
                    math.degrees(angles["hip_ab"]),
                ),
                make_joint_spec(
                    description,
                    catalog,
                    f"{leg_name}_hip_pitch",
                    "hip_pitch",
                    chain.hip,
                    outward,
                    math.degrees(angles["hip_pitch"]),
                ),
                make_joint_spec(
                    description,
                    catalog,
                    f"{leg_name}_knee_bend",
                    "knee_bend",
                    chain.knee,
                    outward,
                    math.degrees(angles["knee_bend"]),
                ),
                make_joint_spec(
                    description,
                    catalog,
                    f"{leg_name}_toe_bend",
                    "toe_bend",
                    chain.toe_joint,
                    outward,
                    math.degrees(angles["toe_bend"]),
                ),
            ]
        )
    specs.extend(
        [
            make_joint_spec(
                description,
                catalog,
                "neck_yaw",
                "neck_yaw",
                model.head.neck_origin,
                model.pose.front_up,
                ranges["neck_yaw"].bias_deg,
            ),
            make_joint_spec(
                description,
                catalog,
                "neck_pitch",
                "neck_pitch",
                model.head.neck_origin,
                model.pose.front_left,
                ranges["neck_pitch"].bias_deg,
            ),
            make_joint_spec(
                description,
                catalog,
                "head_claw",
                "head_claw",
                model.head.hinge,
                model.pose.front_left,
                ranges["head_claw"].bias_deg,
            ),
        ]
    )
    return tuple(specs)


def solve_nonnegative_support_forces(
    com_xy: np.ndarray,
    support_points_xy: dict[str, np.ndarray],
    total_force_n: float,
) -> tuple[dict[str, float], float]:
    names = list(support_points_xy)
    if not names:
        return {}, float(total_force_n)

    active = names[:]
    solution = {name: 0.0 for name in names}
    target = np.array(
        [
            total_force_n,
            total_force_n * float(com_xy[0]),
            total_force_n * float(com_xy[1]),
        ],
        dtype=float,
    )

    while active:
        matrix = np.array(
            [
                [1.0 for _name in active],
                [float(support_points_xy[name][0]) for name in active],
                [float(support_points_xy[name][1]) for name in active],
            ],
            dtype=float,
        )
        values, *_unused = np.linalg.lstsq(matrix, target, rcond=None)
        min_index = int(np.argmin(values))
        if values[min_index] >= -1.0e-8 or len(active) == 1:
            for name, value in zip(active, values):
                solution[name] = max(0.0, float(value))
            break
        active.pop(min_index)

    full_matrix = np.array(
        [
            [1.0 for _name in names],
            [float(support_points_xy[name][0]) for name in names],
            [float(support_points_xy[name][1]) for name in names],
        ],
        dtype=float,
    )
    vector = np.array([solution[name] for name in names], dtype=float)
    residual = float(np.linalg.norm(full_matrix @ vector - target))
    return solution, residual


def contact_rows_for_frame(
    frame: stage3.TrajectoryFrame,
    friction_coeff: float,
) -> tuple[ContactLoadRow, ...]:
    support_legs = set(frame.support_legs)
    total_weight = frame.model.total_mass_kg * GRAVITY_M_S2
    support_points = {
        leg: frame.ik_solutions[leg].chain.toe_endpoint[:2]
        for leg in frame.support_legs
    }
    support_forces, residual = solve_nonnegative_support_forces(frame.model.com_m[:2], support_points, total_weight)
    residual_limit = max(1.0e-6, STATIC_RESIDUAL_FRACTION * total_weight)
    static_solvable = frame.safety.support_polygon_margin_m >= 0.0 and residual <= residual_limit

    rows = []
    for leg in LEG_ORDER:
        solution = frame.ik_solutions[leg]
        normal_force = support_forces.get(leg, 0.0) if leg in support_legs else 0.0
        tangential_force = 0.0
        friction_capacity = max(friction_coeff * normal_force, 1.0e-12)
        rows.append(
            ContactLoadRow(
                frame_index=frame.frame_index,
                time_s=frame.time_s,
                primitive=frame.primitive,
                leg=leg,
                support_state="support" if leg in support_legs else "swing",
                foot_position_m=np.array(solution.chain.toe_endpoint, dtype=float),
                normal_force_n=float(normal_force),
                tangential_force_n=tangential_force,
                friction_coeff=friction_coeff,
                friction_utilization=abs(tangential_force) / friction_capacity,
                static_solvable=static_solvable,
                support_polygon_margin_m=frame.safety.support_polygon_margin_m,
                equilibrium_residual_n=residual,
            )
        )
    return tuple(rows)


def build_contact_rows(case: stage3.Stage3Case, friction_coeff: float) -> tuple[ContactLoadRow, ...]:
    rows: list[ContactLoadRow] = []
    for frame in case.frames:
        rows.extend(contact_rows_for_frame(frame, friction_coeff))
    return tuple(rows)


def root_inertia_diag(model: stage1.MassModel) -> np.ndarray:
    inertia = np.diag(model.inertia_about_com_kg_m2).astype(float)
    floor = max(model.total_mass_kg, 1.0e-9) * 1.0e-6
    return np.maximum(inertia, floor)


def mass_element_body_name(element: stage1.MassElement) -> str:
    if element.leg is not None:
        leg = element.leg
        if element.name.endswith("_hip_ab_actuator"):
            return f"{leg}_hip_ab_body"
        if element.name.endswith("_upper_link") or element.name.endswith("_hip_pitch_actuator"):
            return f"{leg}_hip_pitch_body"
        if element.name.endswith("_lower_link") or element.name.endswith("_knee_bend_actuator"):
            return f"{leg}_knee_body"
        if (
            element.name.endswith("_toe_link")
            or element.name.endswith("_foot_pad")
            or element.name.endswith("_toe_bend_actuator")
        ):
            return f"{leg}_toe_body"
        raise ValueError(f"unmapped leg mass element: {element.name}")

    fixed_body_names = {
        "rear_body_shell": "robot_free_root",
        "front_body_shell": "waist_pitch_body",
        "waist_link_shell": "waist_yaw_body",
        "waist_yaw_actuator": "waist_yaw_body",
        "waist_pitch_actuator": "waist_pitch_body",
        "neck_yaw_actuator": "neck_yaw_body",
        "neck_link": "neck_pitch_body",
        "neck_pitch_actuator": "neck_pitch_body",
        "head_claw_actuator": "head_hinge_body",
        "upper_claw_jaw": "head_upper_jaw_body",
        "lower_claw_jaw": "head_lower_jaw_body",
    }
    if element.name in fixed_body_names:
        return fixed_body_names[element.name]

    if element.kind in {"battery", "electronics"}:
        mount = element.notes.split(" mounted ", 1)[0]
        if mount == "rear":
            return "robot_free_root"
        if mount == "front":
            return "waist_pitch_body"
        if mount == "waist":
            return "waist_yaw_body"

    raise ValueError(f"unmapped mass element: {element.name}")


def mujoco_body_mass_assignments(model: stage1.MassModel) -> dict[str, tuple[stage1.MassElement, ...]]:
    assignments: dict[str, list[stage1.MassElement]] = {}
    for element in model.elements:
        assignments.setdefault(mass_element_body_name(element), []).append(element)
    return {name: tuple(elements) for name, elements in assignments.items()}


def mujoco_body_origins(model: stage1.MassModel) -> dict[str, np.ndarray]:
    origins = {
        "robot_free_root": np.zeros(3),
        "waist_yaw_body": model.pose.yaw_joint,
        "waist_pitch_body": model.pose.pitch_joint,
        "neck_yaw_body": model.head.neck_origin,
        "neck_pitch_body": model.head.neck_origin,
        "head_hinge_body": model.head.hinge,
        "head_upper_jaw_body": model.head.upper_hinge,
        "head_lower_jaw_body": model.head.lower_hinge,
    }
    for leg, chain in model.legs.items():
        origins[f"{leg}_hip_ab_body"] = chain.hip
        origins[f"{leg}_hip_pitch_body"] = chain.hip
        origins[f"{leg}_knee_body"] = chain.knee
        origins[f"{leg}_toe_body"] = chain.toe_joint
    return origins


def mass_element_inertial_values(
    elements: tuple[stage1.MassElement, ...],
    body_origin_m: np.ndarray,
) -> tuple[np.ndarray, float, np.ndarray]:
    if not elements:
        raise ValueError("cannot create a MuJoCo inertial from an empty mass element set")

    total_mass = sum(element.mass_kg for element in elements)
    if total_mass <= 0.0:
        raise ValueError("MuJoCo inertial mass must be positive")

    origin = np.array(body_origin_m, dtype=float)
    local_com = sum(
        (element.mass_kg * (element.com_m - origin) for element in elements),
        start=np.zeros(3),
    ) / total_mass

    inertia = np.zeros((3, 3), dtype=float)
    for element in elements:
        r = element.com_m - origin - local_com
        inertia += element.mass_kg * (float(np.dot(r, r)) * np.eye(3) - np.outer(r, r))
    floor = max(total_mass * 1.0e-6, 1.0e-9)
    return local_com, float(total_mass), np.maximum(np.diag(inertia), floor)


def add_mass_element_inertial(
    body: ET.Element,
    elements: tuple[stage1.MassElement, ...],
    body_origin_m: np.ndarray,
) -> None:
    local_com, total_mass, inertia_diag = mass_element_inertial_values(elements, body_origin_m)
    ET.SubElement(
        body,
        "inertial",
        pos=xml_vec(local_com),
        mass=xml_float(total_mass),
        diaginertia=xml_vec(inertia_diag),
    )


def add_body_inertial(
    body: ET.Element,
    body_name: str,
    mass_assignments: dict[str, tuple[stage1.MassElement, ...]] | None,
    body_origins: dict[str, np.ndarray] | None,
) -> None:
    if mass_assignments is None or body_origins is None:
        ET.SubElement(body, "inertial", pos="0 0 0", mass="1e-5", diaginertia="1e-8 1e-8 1e-8")
        return
    add_mass_element_inertial(body, mass_assignments.get(body_name, ()), body_origins[body_name])


def add_mujoco_assets(root: ET.Element) -> None:
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "material", name="structure", rgba="0.58 0.63 0.70 1")
    ET.SubElement(asset, "material", name="foot", rgba="0.95 0.45 0.14 1")
    ET.SubElement(asset, "material", name="payload", rgba="0.98 0.72 0.18 1")
    ET.SubElement(asset, "material", name="joint_stub", rgba="0.95 0.33 0.45 1")
    ET.SubElement(asset, "material", name="ground", rgba="0.22 0.24 0.27 1")


def add_default(root: ET.Element, friction_coeff: float) -> None:
    default = ET.SubElement(root, "default")
    ET.SubElement(default, "joint", damping="0.025", armature="0.0005", limited="true")
    ET.SubElement(
        default,
        "geom",
        density="0",
        friction=f"{friction_coeff:.6g} 0.02 0.001",
        condim="3",
        solref="0.02 1",
        solimp="0.9 0.95 0.001",
    )
    ET.SubElement(default, "motor", ctrllimited="true")


def add_rod_geoms_for_members(
    root_body: ET.Element,
    rod_model: rods.RodModel,
    include_member: Any,
    origin_m: np.ndarray | None = None,
) -> None:
    origin = np.zeros(3) if origin_m is None else np.array(origin_m, dtype=float)
    node_positions = {node.name: node.xyz_m for node in rod_model.nodes}
    for member in rod_model.members:
        if not include_member(member):
            continue
        a = node_positions[member.node_a] - origin
        b = node_positions[member.node_b] - origin
        radius = max(0.0025, float(member.nominal_radius_m))
        ET.SubElement(
            root_body,
            "geom",
            name=member.name,
            type="capsule",
            fromto=xml_vec([*a, *b]),
            size=xml_float(radius),
            material="structure",
            contype="1",
            conaffinity="1",
        )


def add_rod_geoms(root_body: ET.Element, rod_model: rods.RodModel) -> None:
    add_rod_geoms_for_members(
        root_body,
        rod_model,
        lambda member: member.group not in {"head", "leg", "toe"},
    )


def add_articulated_leg_chains(
    root_body: ET.Element,
    model: stage1.MassModel,
    joint_specs: tuple[Stage4JointSpec, ...],
    foot_radius_m: float,
    friction_coeff: float,
    show_auxiliary_geoms: bool,
    add_contact_feet: bool,
    leg_names: Iterable[str] = LEG_ORDER,
    origin_m: np.ndarray | None = None,
    mass_assignments: dict[str, tuple[stage1.MassElement, ...]] | None = None,
    body_origins: dict[str, np.ndarray] | None = None,
) -> tuple[Stage4HingeExport, ...]:
    specs_by_name = {spec.name: spec for spec in joint_specs}
    origin = np.zeros(3) if origin_m is None else np.array(origin_m, dtype=float)
    hinges: list[Stage4HingeExport] = []

    for leg in leg_names:
        chain = model.legs[leg]
        forward, outward, _down = model.pose.bases[leg]
        upper_vec = chain.knee - chain.hip
        lower_vec = chain.toe_joint - chain.knee
        toe_vec = chain.toe_endpoint - chain.toe_joint

        hip_ab_body = ET.SubElement(root_body, "body", name=f"{leg}_hip_ab_body", pos=xml_vec(chain.hip - origin))
        add_body_inertial(hip_ab_body, f"{leg}_hip_ab_body", mass_assignments, body_origins)
        hip_ab_hinge = hinge_export_from_joint(specs_by_name[f"{leg}_hip_ab"], axis=forward)
        add_hinge_joint(hip_ab_body, hip_ab_hinge)
        hinges.append(hip_ab_hinge)

        hip_pitch_body = ET.SubElement(hip_ab_body, "body", name=f"{leg}_hip_pitch_body", pos="0 0 0")
        add_body_inertial(hip_pitch_body, f"{leg}_hip_pitch_body", mass_assignments, body_origins)
        hip_pitch_hinge = hinge_export_from_joint(specs_by_name[f"{leg}_hip_pitch"], axis=outward)
        add_hinge_joint(hip_pitch_body, hip_pitch_hinge)
        hinges.append(hip_pitch_hinge)
        if show_auxiliary_geoms:
            ET.SubElement(
                hip_ab_body,
                "geom",
                name=f"{leg}_hip_ab_marker",
                type="sphere",
                pos="0 0 0",
                size="0.006",
                material="joint_stub",
                contype="0",
                conaffinity="0",
            )
        ET.SubElement(
            hip_pitch_body,
            "geom",
            name=f"{leg}_upper_link_live",
            type="capsule",
            fromto=xml_vec([0.0, 0.0, 0.0, *upper_vec]),
            size="0.006",
            material="structure",
            contype="1",
            conaffinity="1",
        )

        knee_body = ET.SubElement(hip_pitch_body, "body", name=f"{leg}_knee_body", pos=xml_vec(upper_vec))
        add_body_inertial(knee_body, f"{leg}_knee_body", mass_assignments, body_origins)
        knee_hinge = hinge_export_from_joint(specs_by_name[f"{leg}_knee_bend"], axis=outward)
        add_hinge_joint(knee_body, knee_hinge)
        hinges.append(knee_hinge)
        if show_auxiliary_geoms:
            ET.SubElement(
                knee_body,
                "geom",
                name=f"{leg}_knee_marker",
                type="sphere",
                pos="0 0 0",
                size="0.006",
                material="joint_stub",
                contype="0",
                conaffinity="0",
            )
        ET.SubElement(
            knee_body,
            "geom",
            name=f"{leg}_lower_link_live",
            type="capsule",
            fromto=xml_vec([0.0, 0.0, 0.0, *lower_vec]),
            size="0.0055",
            material="structure",
            contype="1",
            conaffinity="1",
        )

        toe_body = ET.SubElement(knee_body, "body", name=f"{leg}_toe_body", pos=xml_vec(lower_vec))
        add_body_inertial(toe_body, f"{leg}_toe_body", mass_assignments, body_origins)
        toe_hinge = hinge_export_from_joint(specs_by_name[f"{leg}_toe_bend"], axis=outward)
        add_hinge_joint(toe_body, toe_hinge)
        hinges.append(toe_hinge)
        if show_auxiliary_geoms:
            ET.SubElement(
                toe_body,
                "geom",
                name=f"{leg}_toe_joint_marker",
                type="sphere",
                pos="0 0 0",
                size="0.0055",
                material="joint_stub",
                contype="0",
                conaffinity="0",
            )
        ET.SubElement(
            toe_body,
            "geom",
            name=f"{leg}_toe_link_live",
            type="capsule",
            fromto=xml_vec([0.0, 0.0, 0.0, *toe_vec]),
            size="0.0045",
            material="structure",
            contype="1",
            conaffinity="1",
        )
        if add_contact_feet:
            ET.SubElement(
                toe_body,
                "geom",
                name=f"{leg}_foot_contact",
                type="sphere",
                pos=xml_vec(toe_vec),
                size=xml_float(foot_radius_m),
                material="foot",
                friction=f"{friction_coeff:.6g} 0.02 0.001",
                contype="1",
                conaffinity="1",
            )

    return tuple(hinges)


def add_articulated_head_chain(
    root_body: ET.Element,
    model: stage1.MassModel,
    joint_specs: tuple[Stage4JointSpec, ...],
    origin_m: np.ndarray | None = None,
    mass_assignments: dict[str, tuple[stage1.MassElement, ...]] | None = None,
    body_origins: dict[str, np.ndarray] | None = None,
) -> tuple[Stage4HingeExport, ...]:
    specs_by_name = {spec.name: spec for spec in joint_specs}
    required = {"neck_yaw", "neck_pitch", "head_claw"}
    if not required.issubset(specs_by_name):
        return ()
    origin = np.zeros(3) if origin_m is None else np.array(origin_m, dtype=float)

    head = model.head
    pose = model.pose
    neck_vec = head.hinge - head.neck_origin
    upper_mount = head.upper_hinge - head.hinge
    lower_mount = head.lower_hinge - head.hinge
    upper_jaw = head.upper_tip - head.upper_hinge
    lower_jaw = head.lower_tip - head.lower_hinge

    yaw_body = ET.SubElement(root_body, "body", name="neck_yaw_body", pos=xml_vec(head.neck_origin - origin))
    add_body_inertial(yaw_body, "neck_yaw_body", mass_assignments, body_origins)
    neck_yaw_hinge = hinge_export_from_joint(specs_by_name["neck_yaw"], axis=pose.front_up)
    add_hinge_joint(yaw_body, neck_yaw_hinge)

    pitch_body = ET.SubElement(yaw_body, "body", name="neck_pitch_body", pos="0 0 0")
    add_body_inertial(pitch_body, "neck_pitch_body", mass_assignments, body_origins)
    neck_pitch_hinge = hinge_export_from_joint(specs_by_name["neck_pitch"], axis=pose.front_left)
    add_hinge_joint(pitch_body, neck_pitch_hinge)
    ET.SubElement(
        pitch_body,
        "geom",
        name="head_neck_live",
        type="capsule",
        fromto=xml_vec([0.0, 0.0, 0.0, *neck_vec]),
        size="0.0045",
        material="structure",
        contype="1",
        conaffinity="1",
    )

    hinge_body = ET.SubElement(pitch_body, "body", name="head_hinge_body", pos=xml_vec(neck_vec))
    add_body_inertial(hinge_body, "head_hinge_body", mass_assignments, body_origins)
    ET.SubElement(
        hinge_body,
        "geom",
        name="head_upper_hinge_mount_live",
        type="capsule",
        fromto=xml_vec([0.0, 0.0, 0.0, *upper_mount]),
        size="0.003",
        material="structure",
        contype="1",
        conaffinity="1",
    )
    ET.SubElement(
        hinge_body,
        "geom",
        name="head_lower_hinge_mount_live",
        type="capsule",
        fromto=xml_vec([0.0, 0.0, 0.0, *lower_mount]),
        size="0.003",
        material="structure",
        contype="1",
        conaffinity="1",
    )

    upper_body = ET.SubElement(hinge_body, "body", name="head_upper_jaw_body", pos=xml_vec(upper_mount))
    add_body_inertial(upper_body, "head_upper_jaw_body", mass_assignments, body_origins)
    claw_axis = -pose.front_left
    head_claw_driver = hinge_export_from_joint(specs_by_name["head_claw"], axis=claw_axis)
    add_hinge_joint(upper_body, head_claw_driver)
    ET.SubElement(
        upper_body,
        "geom",
        name="head_upper_jaw_live",
        type="capsule",
        fromto=xml_vec([0.0, 0.0, 0.0, *upper_jaw]),
        size="0.004",
        material="structure",
        contype="1",
        conaffinity="1",
    )

    lower_body = ET.SubElement(hinge_body, "body", name="head_lower_jaw_body", pos=xml_vec(lower_mount))
    add_body_inertial(lower_body, "head_lower_jaw_body", mass_assignments, body_origins)
    lower_claw_follower = hinge_export_from_joint(
        specs_by_name["head_claw"],
        name="head_claw_lower",
        axis=claw_axis,
        coefficient=-1.0,
    )
    add_hinge_joint(lower_body, lower_claw_follower)
    ET.SubElement(
        lower_body,
        "geom",
        name="head_lower_jaw_live",
        type="capsule",
        fromto=xml_vec([0.0, 0.0, 0.0, *lower_jaw]),
        size="0.004",
        material="structure",
        contype="1",
        conaffinity="1",
    )

    return (neck_yaw_hinge, neck_pitch_hinge, head_claw_driver, lower_claw_follower)


def add_payload_sites(root_body: ET.Element, rod_model: rods.RodModel, show_auxiliary_geoms: bool) -> None:
    if not show_auxiliary_geoms:
        return
    for mass in rod_model.lumped_masses:
        if mass.kind not in {"battery", "electronics"}:
            continue
        ET.SubElement(
            root_body,
            "geom",
            name=f"payload_{mass.name}",
            type="sphere",
            pos=xml_vec(mass.xyz_m),
            size="0.018",
            material="payload",
            contype="0",
            conaffinity="0",
        )

def add_joint_stubs(
    root_body: ET.Element,
    joint_specs: tuple[Stage4JointSpec, ...],
    show_auxiliary_geoms: bool,
    live_driver_names: set[str],
) -> None:
    for spec in joint_specs:
        if spec.name in live_driver_names:
            continue
        body = ET.SubElement(root_body, "body", name=f"joint_stub_{spec.name}", pos=xml_vec(spec.position_m))
        ET.SubElement(body, "inertial", pos="0 0 0", mass="1e-6", diaginertia="1e-9 1e-9 1e-9")
        add_hinge_joint(body, hinge_export_from_joint(spec))
        if show_auxiliary_geoms:
            ET.SubElement(
                body,
                "geom",
                name=f"{spec.name}_joint_marker",
                type="sphere",
                size="0.006",
                material="joint_stub",
                contype="0",
                conaffinity="0",
            )


def ensure_xml_section(root: ET.Element, section_name: str) -> ET.Element:
    section = root.find(f"./{section_name}")
    if section is None:
        section = ET.SubElement(root, section_name)
    return section


def add_viewer_safe_articulated_body_tree(
    root_body: ET.Element,
    case: Stage4Case,
    model: stage1.MassModel,
    joint_specs: tuple[Stage4JointSpec, ...],
    foot_radius_m: float,
    friction_coeff: float,
    mass_assignments: dict[str, tuple[stage1.MassElement, ...]],
    body_origins: dict[str, np.ndarray],
) -> tuple[Stage4HingeExport, ...]:
    specs_by_name = {spec.name: spec for spec in joint_specs}

    pose = model.pose
    yaw_origin = pose.yaw_joint
    pitch_origin = pose.pitch_joint
    hinges: list[Stage4HingeExport] = []

    add_rod_geoms_for_members(
        root_body,
        case.rod_model,
        lambda member: member.name == "rear_body_spine" or member.name.startswith("rear_") and member.group == "hip_cross",
    )
    hinges.extend(
        add_articulated_leg_chains(
            root_body,
            model,
            joint_specs,
            foot_radius_m,
            friction_coeff,
            show_auxiliary_geoms=False,
            add_contact_feet=True,
            leg_names=("rear_left", "rear_right"),
            mass_assignments=mass_assignments,
            body_origins=body_origins,
        )
    )

    waist_yaw_body = ET.SubElement(root_body, "body", name="waist_yaw_body", pos=xml_vec(yaw_origin))
    add_body_inertial(waist_yaw_body, "waist_yaw_body", mass_assignments, body_origins)
    waist_yaw_hinge = hinge_export_from_joint(specs_by_name["waist_yaw"])
    add_hinge_joint(waist_yaw_body, waist_yaw_hinge)
    hinges.append(waist_yaw_hinge)
    add_rod_geoms_for_members(
        waist_yaw_body,
        case.rod_model,
        lambda member: member.name == "waist_yaw_pitch",
        origin_m=yaw_origin,
    )

    waist_pitch_body = ET.SubElement(
        waist_yaw_body,
        "body",
        name="waist_pitch_body",
        pos=xml_vec(pitch_origin - yaw_origin),
    )
    add_body_inertial(waist_pitch_body, "waist_pitch_body", mass_assignments, body_origins)
    waist_pitch_hinge = hinge_export_from_joint(specs_by_name["waist_pitch"])
    add_hinge_joint(waist_pitch_body, waist_pitch_hinge)
    hinges.append(waist_pitch_hinge)
    add_rod_geoms_for_members(
        waist_pitch_body,
        case.rod_model,
        lambda member: member.name == "front_body_spine" or member.name.startswith("front_") and member.group == "hip_cross",
        origin_m=pitch_origin,
    )
    hinges.extend(
        add_articulated_leg_chains(
            waist_pitch_body,
            model,
            joint_specs,
            foot_radius_m,
            friction_coeff,
            show_auxiliary_geoms=False,
            add_contact_feet=True,
            leg_names=("front_left", "front_right"),
            origin_m=pitch_origin,
            mass_assignments=mass_assignments,
            body_origins=body_origins,
        )
    )
    hinges.extend(
        add_articulated_head_chain(
            waist_pitch_body,
            model,
            joint_specs,
            origin_m=pitch_origin,
            mass_assignments=mass_assignments,
            body_origins=body_origins,
        )
    )
    return tuple(hinges)


def viewer_exported_joint_specs(joint_specs: tuple[Stage4JointSpec, ...]) -> tuple[Stage4JointSpec, ...]:
    return joint_specs


def exported_joint_specs(case: Stage4Case) -> tuple[Stage4JointSpec, ...]:
    if case.viewer_safe:
        return viewer_exported_joint_specs(case.joint_specs)
    return case.joint_specs


def mujoco_xml_tree(
    case: Stage4Case,
    description: DogDescription,
    friction_coeff: float,
    foot_radius_m: float,
) -> ET.ElementTree:
    first_model = case.stage3_case.frames[0].model
    active_joint_specs = exported_joint_specs(case)
    model_name = "chihuahua_stage4_viewer_safe" if case.viewer_safe else "chihuahua_stage4_first_light"
    root = ET.Element("mujoco", model=model_name)
    ET.SubElement(root, "compiler", angle="degree", coordinate="local", autolimits="true")
    ET.SubElement(root, "option", timestep="0.002", integrator="RK4", gravity=f"0 0 {-GRAVITY_M_S2:.6g}")
    add_default(root, friction_coeff)
    add_mujoco_assets(root)

    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", name="key", pos="0 -1 1.2", dir="0 1 -1", diffuse="0.8 0.8 0.8")
    ET.SubElement(
        world,
        "geom",
        name="ground",
        type="plane",
        pos=f"0 0 {-foot_radius_m:.10g}",
        size="2.0 2.0 0.02",
        material="ground",
        friction=f"{friction_coeff:.6g} 0.02 0.001",
    )
    root_body = ET.SubElement(world, "body", name="robot_free_root", pos="0 0 0")
    ET.SubElement(root_body, "freejoint", name="root_free")
    mass_assignments = mujoco_body_mass_assignments(first_model) if case.viewer_safe else None
    body_origins = mujoco_body_origins(first_model) if case.viewer_safe else None
    if case.viewer_safe:
        add_body_inertial(root_body, "robot_free_root", mass_assignments, body_origins)
    else:
        ET.SubElement(
            root_body,
            "inertial",
            pos=xml_vec(first_model.com_m),
            mass=xml_float(first_model.total_mass_kg),
            diaginertia=xml_vec(root_inertia_diag(first_model)),
        )
    show_auxiliary_geoms = not case.viewer_safe
    live_hinges: tuple[Stage4HingeExport, ...] = ()
    if case.viewer_safe:
        live_hinges = add_viewer_safe_articulated_body_tree(
            root_body,
            case,
            first_model,
            active_joint_specs,
            foot_radius_m,
            friction_coeff,
            mass_assignments or {},
            body_origins or {},
        )
    else:
        add_rod_geoms(root_body, case.rod_model)
        non_viewer_hinges: list[Stage4HingeExport] = []
        non_viewer_hinges.extend(
            add_articulated_leg_chains(
                root_body,
                first_model,
                active_joint_specs,
                foot_radius_m,
                friction_coeff,
                show_auxiliary_geoms=show_auxiliary_geoms,
                add_contact_feet=True,
            )
        )
        non_viewer_hinges.extend(add_articulated_head_chain(root_body, first_model, active_joint_specs))
        live_hinges = tuple(non_viewer_hinges)
        add_payload_sites(root_body, case.rod_model, show_auxiliary_geoms=show_auxiliary_geoms)
        add_joint_stubs(
            root_body,
            case.joint_specs,
            show_auxiliary_geoms=show_auxiliary_geoms,
            live_driver_names={hinge.driver_name for hinge in live_hinges},
        )
    add_joint_follower_equalities(root, live_hinges)

    actuator = ET.SubElement(root, "actuator")
    for spec in active_joint_specs:
        ET.SubElement(
            actuator,
            "motor",
            name=f"{spec.name}_motor",
            joint=spec.name,
            gear="1",
            ctrlrange=f"{xml_float(-spec.max_torque_nm)} {xml_float(spec.max_torque_nm)}",
            forcerange=f"{xml_float(-spec.max_torque_nm)} {xml_float(spec.max_torque_nm)}",
        )

    ET.SubElement(root, "sensor")
    _ = description
    return ET.ElementTree(root)


def write_mujoco_xml(
    case: Stage4Case,
    description: DogDescription,
    friction_coeff: float,
    foot_radius_m: float,
) -> None:
    case.xml_path.parent.mkdir(parents=True, exist_ok=True)
    tree = mujoco_xml_tree(case, description, friction_coeff, foot_radius_m)
    ET.indent(tree, space="  ")
    tree.write(case.xml_path, encoding="utf-8", xml_declaration=True)
    text = case.xml_path.read_text(encoding="utf-8")
    case.mjcf_path.write_text(text, encoding="utf-8")
    stale_mjsd_alias = case.out_dir / "mujoco_model.mjsd"
    if stale_mjsd_alias.exists():
        stale_mjsd_alias.unlink()


def write_contact_csv(rows: tuple[ContactLoadRow, ...], path: Path) -> None:
    fields = [
        "frame_index",
        "time_s",
        "primitive",
        "leg",
        "support_state",
        "foot_x_m",
        "foot_y_m",
        "foot_z_m",
        "normal_force_n",
        "tangential_force_n",
        "friction_coeff",
        "friction_utilization",
        "static_solvable",
        "support_polygon_margin_m",
        "equilibrium_residual_n",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "frame_index": row.frame_index,
                    "time_s": fmt(row.time_s),
                    "primitive": row.primitive,
                    "leg": row.leg,
                    "support_state": row.support_state,
                    "foot_x_m": fmt(float(row.foot_position_m[0])),
                    "foot_y_m": fmt(float(row.foot_position_m[1])),
                    "foot_z_m": fmt(float(row.foot_position_m[2])),
                    "normal_force_n": fmt(row.normal_force_n),
                    "tangential_force_n": fmt(row.tangential_force_n),
                    "friction_coeff": fmt(row.friction_coeff),
                    "friction_utilization": fmt(row.friction_utilization),
                    "static_solvable": "yes" if row.static_solvable else "no",
                    "support_polygon_margin_m": fmt(row.support_polygon_margin_m),
                    "equilibrium_residual_n": fmt(row.equilibrium_residual_n),
                }
            )


def write_stage2_feedback_csv(rows: tuple[ContactLoadRow, ...], path: Path) -> None:
    fields = [
        "load_case",
        "frame_index",
        "time_s",
        "source",
        "stage2_node",
        "leg",
        "force_x_n",
        "force_y_n",
        "force_z_n",
        "static_solvable",
        "notes",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            if row.normal_force_n <= 1.0e-9:
                continue
            writer.writerow(
                {
                    "load_case": f"{row.primitive}_{row.frame_index:03d}_{row.leg}_ground_reaction",
                    "frame_index": row.frame_index,
                    "time_s": fmt(row.time_s),
                    "source": "stage4_quasi_static_contact_proxy",
                    "stage2_node": f"{row.leg}_toe_endpoint",
                    "leg": row.leg,
                    "force_x_n": fmt(0.0),
                    "force_y_n": fmt(0.0),
                    "force_z_n": fmt(row.normal_force_n),
                    "static_solvable": "yes" if row.static_solvable else "no",
                    "notes": "ground reaction candidate for later Stage 2 no-gravity structural replay",
                }
            )


def write_mujoco_contact_csv(
    xml_path: Path,
    output_csv: Path,
    duration_s: float,
) -> MujocoContactCsvResult:
    try:
        import mujoco  # type: ignore[import-not-found]
    except ModuleNotFoundError as exc:
        return MujocoContactCsvResult(
            attempted=True,
            mujoco_available=False,
            completed=False,
            error=str(exc),
        )

    fields = [
        "step",
        "time_s",
        "source",
        "contact_index",
        "geom1",
        "geom2",
        "body1",
        "body2",
        "contact_x_m",
        "contact_y_m",
        "contact_z_m",
        "normal_x",
        "normal_y",
        "normal_z",
        "normal_force_n",
        "tangent_force_1_n",
        "tangent_force_2_n",
        "tangential_force_n",
        "torsional_friction_n_m",
        "rolling_friction_1_n_m",
        "rolling_friction_2_n_m",
        "friction_coeff",
    ]

    try:
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        data = mujoco.MjData(model)
        steps = max(1, int(math.ceil(duration_s / model.opt.timestep)))
        output_csv.parent.mkdir(parents=True, exist_ok=True)
        row_count = 0
        max_contacts = 0
        force = np.zeros(6, dtype=float)

        with output_csv.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for step_idx in range(steps):
                mujoco.mj_step(model, data)
                max_contacts = max(max_contacts, int(data.ncon))
                for contact_idx in range(data.ncon):
                    contact = data.contact[contact_idx]
                    mujoco.mj_contactForce(model, data, contact_idx, force)
                    geom1 = int(contact.geom1)
                    geom2 = int(contact.geom2)
                    body1 = int(model.geom_bodyid[geom1])
                    body2 = int(model.geom_bodyid[geom2])
                    normal = np.array(contact.frame[:3], dtype=float)
                    tangential_force = float(np.linalg.norm(force[1:3]))
                    writer.writerow(
                        {
                            "step": step_idx,
                            "time_s": fmt(float(data.time)),
                            "source": "mujoco_contact_force",
                            "contact_index": contact_idx,
                            "geom1": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom1) or "",
                            "geom2": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom2) or "",
                            "body1": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body1) or "",
                            "body2": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body2) or "",
                            "contact_x_m": fmt(float(contact.pos[0])),
                            "contact_y_m": fmt(float(contact.pos[1])),
                            "contact_z_m": fmt(float(contact.pos[2])),
                            "normal_x": fmt(float(normal[0])),
                            "normal_y": fmt(float(normal[1])),
                            "normal_z": fmt(float(normal[2])),
                            "normal_force_n": fmt(float(force[0])),
                            "tangent_force_1_n": fmt(float(force[1])),
                            "tangent_force_2_n": fmt(float(force[2])),
                            "tangential_force_n": fmt(tangential_force),
                            "torsional_friction_n_m": fmt(float(force[3])),
                            "rolling_friction_1_n_m": fmt(float(force[4])),
                            "rolling_friction_2_n_m": fmt(float(force[5])),
                            "friction_coeff": fmt(float(contact.friction[0])),
                        }
                    )
                    row_count += 1

        return MujocoContactCsvResult(
            attempted=True,
            mujoco_available=True,
            completed=True,
            steps=steps,
            rows=row_count,
            max_contacts=max_contacts,
        )
    except Exception as exc:  # pragma: no cover - depends on optional external solver.
        return MujocoContactCsvResult(
            attempted=True,
            mujoco_available=True,
            completed=False,
            error=f"{type(exc).__name__}: {exc}",
        )


def run_mujoco_smoke(
    xml_path: Path,
    output_csv: Path,
    duration_s: float,
) -> MujocoSmokeResult:
    try:
        import mujoco  # type: ignore[import-not-found]
    except ModuleNotFoundError as exc:
        return MujocoSmokeResult(
            attempted=True,
            mujoco_available=False,
            completed=False,
            error=str(exc),
        )

    try:
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        data = mujoco.MjData(model)
        steps = max(1, int(math.ceil(duration_s / model.opt.timestep)))
        output_csv.parent.mkdir(parents=True, exist_ok=True)
        max_contacts = 0
        min_root_z: float | None = None
        with output_csv.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=["step", "time_s", "root_z_m", "contact_count"])
            writer.writeheader()
            for step_idx in range(steps):
                mujoco.mj_step(model, data)
                root_z = float(data.qpos[2]) if data.qpos.size >= 3 else 0.0
                min_root_z = root_z if min_root_z is None else min(min_root_z, root_z)
                max_contacts = max(max_contacts, int(data.ncon))
                writer.writerow(
                    {
                        "step": step_idx,
                        "time_s": fmt(float(data.time)),
                        "root_z_m": fmt(root_z),
                        "contact_count": int(data.ncon),
                    }
                )
        return MujocoSmokeResult(
            attempted=True,
            mujoco_available=True,
            completed=True,
            steps=steps,
            final_time_s=float(data.time),
            max_contacts=max_contacts,
            min_root_z_m=min_root_z,
        )
    except Exception as exc:  # pragma: no cover - depends on optional external solver.
        return MujocoSmokeResult(
            attempted=True,
            mujoco_available=True,
            completed=False,
            error=f"{type(exc).__name__}: {exc}",
        )


def summary_dict(case: Stage4Case) -> dict[str, Any]:
    safe_frames = [frame for frame in case.stage3_case.frames if frame.safety.safe_to_execute]
    static_frames = {
        row.frame_index
        for row in case.contact_rows
        if row.static_solvable
    }
    support_rows = [row for row in case.contact_rows if row.normal_force_n > 1.0e-9]
    max_force = max((row.normal_force_n for row in support_rows), default=0.0)
    max_residual = max((row.equilibrium_residual_n for row in case.contact_rows), default=0.0)
    frame_mass = case.stage3_case.frames[0].model.total_mass_kg if case.stage3_case.frames else 0.0
    exported_specs = exported_joint_specs(case)
    mass_assignments = (
        mujoco_body_mass_assignments(case.stage3_case.frames[0].model)
        if case.viewer_safe and case.stage3_case.frames
        else {}
    )
    return {
        "stage": "stage_4_mujoco_contact_first_light",
        "primitive": case.primitive,
        "analysis_state": {
            "mujoco_xml_exported": True,
            "joint_actuators_exported": True,
            "contact_geometry_exported": True,
            "visible_structure_collision_enabled": True,
            "viewer_safe_foot_pins": False,
            "viewer_safe_uses_mujoco_foot_contacts": case.viewer_safe,
            "viewer_safe_uses_distributed_stage1_mass": case.viewer_safe,
            "viewer_safe_separates_leg_dofs_into_bodies": case.viewer_safe,
            "torque_limited_mujoco_motors": True,
            "position_servo_actuators": False,
            "contact_loadcases_from_mujoco": case.contact_csv_result.completed,
            "quasi_static_contact_proxy_applied": True,
            "mjsd_alias_written": False,
            "mujoco_simulation_attempted": case.smoke_result.attempted,
            "mujoco_python_available": case.smoke_result.mujoco_available,
            "mujoco_smoke_completed": case.smoke_result.completed,
            "closed_loop_balance_controller": False,
            "dynamic_contact_policy_validated": case.smoke_result.completed,
        },
        "counts": {
            "frames": len(case.stage3_case.frames),
            "stage3_safe_frames": len(safe_frames),
            "static_contact_solvable_frames": len(static_frames),
            "contact_rows": len(case.contact_rows),
            "support_contact_rows": len(support_rows),
            "rod_graph_nodes": len(case.rod_model.nodes),
            "rod_graph_members": len(case.rod_model.members),
            "joint_specs": len(case.joint_specs),
            "actuator_motors": len(exported_specs),
            "foot_contact_geoms": len(LEG_ORDER),
            "distributed_mujoco_inertial_bodies": len(mass_assignments),
            "mujoco_contact_rows": case.contact_csv_result.rows,
        },
        "mass": {
            "model_mass_kg": float(frame_mass),
            "model_weight_n": float(frame_mass * GRAVITY_M_S2),
            "mass_source": "stage1 MassModel.elements",
        },
        "contact_proxy": {
            "friction_coeff": DEFAULT_FOOT_FRICTION,
            "max_normal_force_n": float(max_force),
            "max_equilibrium_residual_n": float(max_residual),
            "residual_limit_fraction_of_weight": STATIC_RESIDUAL_FRACTION,
        },
        "mujoco_contact_loadcases": {
            "attempted": case.contact_csv_result.attempted,
            "mujoco_available": case.contact_csv_result.mujoco_available,
            "completed": case.contact_csv_result.completed,
            "steps": case.contact_csv_result.steps,
            "rows": case.contact_csv_result.rows,
            "max_contacts": case.contact_csv_result.max_contacts,
            "error": case.contact_csv_result.error,
        },
        "mujoco_smoke": {
            "attempted": case.smoke_result.attempted,
            "mujoco_available": case.smoke_result.mujoco_available,
            "completed": case.smoke_result.completed,
            "steps": case.smoke_result.steps,
            "final_time_s": float(case.smoke_result.final_time_s),
            "max_contacts": case.smoke_result.max_contacts,
            "min_root_z_m": case.smoke_result.min_root_z_m,
            "error": case.smoke_result.error,
        },
        "outputs": {
            "mujoco_xml": case.xml_path.name,
            "mujoco_mjcf": case.mjcf_path.name,
            "contact_loadcases": case.contact_csv_path.name,
            "quasi_static_contact_proxy": case.contact_proxy_csv_path.name,
            "stage2_feedback_loadcases": case.stage2_feedback_csv_path.name,
            "mujoco_smoke": case.mujoco_smoke_csv_path.name if case.smoke_result.completed else None,
        },
        "notes": [
            "The MuJoCo XML is a rough whole-body model built from the Stage 2 rod graph and Stage 3 pose.",
            "Viewer-safe waist, leg, neck, and head controls are wired to articulated visible geometry.",
            "The viewer entry uses MuJoCo gravity plus foot contact geoms, not invisible foot-pin equalities.",
            "Visible structural capsules are collision-enabled; geometry is still coarse rods, not CAD solids.",
            "Actuator controls are torque commands through MuJoCo motor actuators, not position-servo targets.",
            "contact_loadcases.csv is sampled from MuJoCo contact forces; quasi_static_contact_proxy.csv keeps the analytic support-force proxy.",
            "Stage 2 feedback rows are candidate ground reactions; they are not automatically applied by Stage 2.",
        ],
    }


def write_summary(case: Stage4Case) -> None:
    with case.summary_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(summary_dict(case), handle, sort_keys=False)


def build_stage4_case(
    description: DogDescription,
    catalog: stage1.PhysicalCatalog,
    out_dir: Path,
    primitive: str,
    frame_count: int,
    duration_s: float,
    waist_yaw_deg: float,
    waist_pitch_deg: float,
    swing_leg: str,
    step_length_m: float,
    step_height_m: float,
    min_torque_margin: float,
    min_joint_limit_margin_deg: float,
    ik_tolerance_m: float,
    friction_coeff: float,
    run_mujoco: bool,
    mujoco_duration_s: float,
    viewer_safe: bool = False,
) -> Stage4Case:
    stage3_case = stage3.build_stage3_case(
        description,
        catalog,
        out_dir=out_dir / "stage3_source",
        primitive=primitive,
        frame_count=frame_count,
        duration_s=duration_s,
        waist_yaw_deg=waist_yaw_deg,
        waist_pitch_deg=waist_pitch_deg,
        swing_leg=swing_leg,
        step_length_m=step_length_m,
        step_height_m=step_height_m,
        min_torque_margin=min_torque_margin,
        min_joint_limit_margin_deg=min_joint_limit_margin_deg,
        ik_tolerance_m=ik_tolerance_m,
    )
    first_model = stage3_case.frames[0].model
    rod_model = rods.build_whole_body_rod_model(first_model)
    joint_specs = build_joint_specs(description, catalog, stage3_case.frames[0])
    contact_rows = build_contact_rows(stage3_case, friction_coeff)
    case = Stage4Case(
        primitive=primitive,
        stage3_case=stage3_case,
        rod_model=rod_model,
        joint_specs=joint_specs,
        contact_rows=contact_rows,
        out_dir=out_dir,
        viewer_safe=viewer_safe,
        xml_path=out_dir / "mujoco_model.xml",
        mjcf_path=out_dir / "mujoco_model.mjcf",
        contact_csv_path=out_dir / "contact_loadcases.csv",
        contact_proxy_csv_path=out_dir / "quasi_static_contact_proxy.csv",
        stage2_feedback_csv_path=out_dir / "stage2_feedback_loadcases.csv",
        summary_path=out_dir / "stage4_mujoco_contact_summary.yaml",
        mujoco_smoke_csv_path=out_dir / "mujoco_smoke.csv",
        contact_csv_result=MujocoContactCsvResult(attempted=False, mujoco_available=False, completed=False),
        smoke_result=MujocoSmokeResult(attempted=False, mujoco_available=False, completed=False),
    )
    write_mujoco_xml(case, description, friction_coeff, DEFAULT_FOOT_RADIUS_M)
    contact_csv_result = write_mujoco_contact_csv(case.xml_path, case.contact_csv_path, mujoco_duration_s)
    write_contact_csv(contact_rows, case.contact_proxy_csv_path)
    write_stage2_feedback_csv(contact_rows, case.stage2_feedback_csv_path)

    smoke = (
        run_mujoco_smoke(case.xml_path, case.mujoco_smoke_csv_path, mujoco_duration_s)
        if run_mujoco
        else MujocoSmokeResult(attempted=False, mujoco_available=False, completed=False)
    )
    case = Stage4Case(
        primitive=case.primitive,
        stage3_case=case.stage3_case,
        rod_model=case.rod_model,
        joint_specs=case.joint_specs,
        contact_rows=case.contact_rows,
        out_dir=case.out_dir,
        viewer_safe=case.viewer_safe,
        xml_path=case.xml_path,
        mjcf_path=case.mjcf_path,
        contact_csv_path=case.contact_csv_path,
        contact_proxy_csv_path=case.contact_proxy_csv_path,
        stage2_feedback_csv_path=case.stage2_feedback_csv_path,
        summary_path=case.summary_path,
        mujoco_smoke_csv_path=case.mujoco_smoke_csv_path,
        contact_csv_result=contact_csv_result,
        smoke_result=smoke,
    )
    write_summary(case)
    return case


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
    parser.add_argument("--foot-friction", type=float, default=DEFAULT_FOOT_FRICTION)
    parser.add_argument("--run-mujoco", action="store_true")
    parser.add_argument("--mujoco-duration-s", type=float, default=0.05)
    return parser


def load_inputs(args: argparse.Namespace) -> tuple[DogDescription, stage1.PhysicalCatalog]:
    description = load_dog_description(args.description)
    catalog = stage1.load_catalog(args.materials, args.actuators, args.batteries)
    return description, catalog


def main(argv: Iterable[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    description, catalog = load_inputs(args)
    case = build_stage4_case(
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
        friction_coeff=args.foot_friction,
        run_mujoco=args.run_mujoco,
        mujoco_duration_s=args.mujoco_duration_s,
    )
    summary = summary_dict(case)
    print(f"wrote Stage 4 MuJoCo/contact outputs to {case.out_dir}")
    print(f"mujoco xml: {case.xml_path.name}")
    print(f"mujoco mjcf: {case.mjcf_path.name}")
    print(f"contact loadcases: {case.contact_csv_path.name}")
    print(f"quasi-static contact proxy: {case.contact_proxy_csv_path.name}")
    print(f"stage2 feedback loadcases: {case.stage2_feedback_csv_path.name}")
    print(f"frames: {summary['counts']['frames']}")
    print(f"static contact solvable frames: {summary['counts']['static_contact_solvable_frames']}")
    print(f"joint actuators exported: {summary['counts']['actuator_motors']}")
    print(f"mujoco contact rows: {summary['counts']['mujoco_contact_rows']}")
    print(f"max normal force N: {summary['contact_proxy']['max_normal_force_n']:.6g}")
    if case.smoke_result.attempted:
        print(
            "mujoco smoke: "
            f"available={case.smoke_result.mujoco_available} "
            f"completed={case.smoke_result.completed}"
        )
        if case.smoke_result.error:
            print(f"mujoco smoke error: {case.smoke_result.error}")
    else:
        print("mujoco smoke: not run; pass --run-mujoco to attempt optional solver smoke")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
