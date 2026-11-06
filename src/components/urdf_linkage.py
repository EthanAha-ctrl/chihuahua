"""URDF forward kinematics for the linkage viewer; joint positions follow URDF limits.

Visuals use precomputed mesh bounds, not solid meshes. No dynamics are simulated.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import json
import hashlib
import math
import xml.etree.ElementTree as ET

import numpy as np

MODEL_ROOT = Path(__file__).resolve().parents[2] / "vendor/reference/urdf"
MODEL_PATHS = {"pupper": MODEL_ROOT / "pupper_v3/robot.urdf", "go2": MODEL_ROOT / "go2/robot.urdf", "aibo": MODEL_ROOT / "aibo_ers7/robot.urdf"}
MODEL_LABELS = {"pupper": "Pupper v3", "go2": "Unitree Go2", "aibo": "Aibo ERS-7"}


def vector(text: str) -> np.ndarray:
    values = np.array([float(x) for x in text.split()])
    if values.shape != (3,) or not np.isfinite(values).all():
        raise ValueError(f"Invalid URDF vector: {text!r}")
    return values


def rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        raise ValueError("URDF rotation axis is zero")
    x, y, z = axis / norm
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


def origin(element: ET.Element | None) -> np.ndarray:
    t = np.eye(4)
    if element is not None:
        t[:3, 3] = vector(element.get("xyz", "0 0 0"))
        r, p, y = vector(element.get("rpy", "0 0 0"))
        t[:3, :3] = (rotation(np.array([0, 0, 1]), y)
                        @ rotation(np.array([0, 1, 0]), p)
                        @ rotation(np.array([1, 0, 0]), r))
    return t


def box_edges(lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    points = np.array([[upper[i] if mask & (1 << i) else lower[i] for i in range(3)] for mask in range(8)])
    return np.array([[points[m], points[m ^ (1 << i)]] for m in range(8) for i in range(3) if not m & (1 << i)])


@dataclass(frozen=True)
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    transform: np.ndarray
    axis: np.ndarray
    effort: float | None = None
    lower_deg: float = -math.inf
    upper_deg: float = math.inf

    def clamp(self, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError(f"Nonfinite angle for {self.name}")
        return max(self.lower_deg, min(self.upper_deg, value))


@dataclass(frozen=True)
class Link:
    name: str
    mass: float
    com: np.ndarray
    edges: np.ndarray


@dataclass(frozen=True)
class UrdfModel:
    name: str
    root: str
    links: dict[str, Link]
    joints: tuple[Joint, ...]  # parent before child

    display_anchors: dict[str, np.ndarray] = field(default_factory=dict)

    def joint_point(self, joint: Joint, poses: dict[str, np.ndarray]) -> np.ndarray:
        """Audited display landmark; does not alter the kinematic frame or axis."""
        t = poses[joint.child]
        return t[:3, :3] @ self.display_anchors.get(joint.name, np.zeros(3)) + t[:3, 3]

    def go2_linkage_edges(self) -> tuple[tuple[str, str], ...]:
        """Visible skeleton topology, not the full URDF frame/inertia tree.

        Rotors, sensors and calf collision helper frames are not extra bones.
        The torso outline joins its four leg mounts in perimeter order.
        """
        mounts = [f"{leg}_hip_joint" for leg in ("FL", "FR", "RR", "RL")]
        edges = list(zip(mounts, mounts[1:] + mounts[:1]))
        for leg in ("FL", "FR", "RL", "RR"):
            chain = [f"{leg}_{part}_joint" for part in ("hip", "thigh", "calf", "foot")]
            edges.extend(zip(chain, chain[1:]))
        names = {j.name for j in self.joints}
        missing = {n for edge in edges for n in edge} - names
        if missing:
            raise ValueError(f"Go2 skeleton missing joints: {sorted(missing)}")
        return tuple(edges)

    def aibo_linkage_edges(self) -> tuple[tuple[str, str], ...]:
        """Body perimeter, four legs, neck/head and tail; no ear frame stubs.

        Body midpoints are display connectors on the rigid torso, not joints.
        Shared neck/pan and tail pivots retain their original zero-length edges.
        """
        mounts = [f"leg{leg}1" for leg in ("RF", "LF", "LB", "RB")]
        edges = list(zip(mounts, mounts[1:] + mounts[:1]))
        for leg in ("RF", "LF", "RB", "LB"):
            chain = [f"leg{leg}{i}" for i in (1, 2, 3)] + [f"{leg}_paw_joint"]
            edges.extend(zip(chain, chain[1:]))
        edges.extend([("front_body_midpoint", "neck"), ("rear_body_midpoint", "tailTilt"),
                      ("neck", "headPan"), ("headPan", "headTilt"), ("tailTilt", "tailPan")])
        edges.extend(("headTilt", name) for name in ("mouth", "earR", "earL"))
        names = {j.name for j in self.joints} | {"front_body_midpoint", "rear_body_midpoint"}
        missing = {n for edge in edges for n in edge} - names
        if missing:
            raise ValueError(f"Aibo skeleton missing joints: {sorted(missing)}")
        return tuple(edges)

    def skeleton_segments(self, poses: dict[str, np.ndarray]) -> list[tuple[np.ndarray, np.ndarray]]:
        points = {j.name: self.joint_point(j, poses) for j in self.joints}
        if self.name == "go2_description":
            edges = self.go2_linkage_edges()
        elif self.name == "Aibo":
            edges = self.aibo_linkage_edges()
            points["front_body_midpoint"] = (points["legRF1"] + points["legLF1"]) / 2
            points["rear_body_midpoint"] = (points["legRB1"] + points["legLB1"]) / 2
        else:
            raise ValueError(f"Explicit skeleton not defined for {self.name}")
        return [(points[a], points[b]) for a, b in edges]

    @property
    def movable(self) -> tuple[Joint, ...]:
        return tuple(j for j in self.joints if j.kind != "fixed")

    def forward(self, angles_deg: dict[str, float], root_transform: np.ndarray | None = None) -> dict[str, np.ndarray]:
        unknown = angles_deg.keys() - {j.name for j in self.movable}
        if unknown:
            raise ValueError(f"Unknown movable joints: {sorted(unknown)}")
        poses = {self.root: np.eye(4) if root_transform is None else root_transform.copy()}
        for j in self.joints:
            motion = np.eye(4)
            if j.kind != "fixed":
                value = j.clamp(angles_deg.get(j.name, 0.0))
                if not math.isfinite(value):
                    raise ValueError(f"Nonfinite angle for {j.name}")
                motion[:3, :3] = rotation(j.axis, math.radians(value))
            poses[j.child] = poses[j.parent] @ j.transform @ motion
        return poses

    def mass_com(self, poses: dict[str, np.ndarray]) -> tuple[float, np.ndarray]:
        mass = sum(l.mass for l in self.links.values())
        if mass <= 0:
            raise ValueError("URDF has no positive total mass")
        com = sum((l.mass * (poses[l.name][:3, :3] @ l.com + poses[l.name][:3, 3])
                   for l in self.links.values()), start=np.zeros(3)) / mass
        return mass, com


def visual_edges(link: ET.Element, bounds: dict) -> np.ndarray:
    edges = []
    for visual in link.findall("visual"):
        geometry = visual.find("geometry")
        if geometry is None or len(geometry) != 1:
            raise ValueError(f"Invalid geometry in {link.get('name')}")
        shape = geometry[0]
        if shape.tag == "mesh":
            uri = shape.attrib["filename"]
            if uri not in bounds:
                raise ValueError(f"Missing precomputed mesh bounds: {uri}")
            scaled = np.asarray(bounds[uri], dtype=float) * vector(shape.get("scale", "1 1 1"))
            lower, upper = scaled.min(axis=0), scaled.max(axis=0)
        elif shape.tag == "box":
            upper = vector(shape.attrib["size"]) / 2
            lower = -upper
        elif shape.tag in ("sphere", "cylinder"):
            radius = float(shape.attrib["radius"])
            upper = np.array([radius, radius, radius if shape.tag == "sphere" else float(shape.attrib["length"]) / 2])
            lower = -upper
        else:
            raise ValueError(f"Unsupported visual shape: {shape.tag}")
        if not np.isfinite([lower, upper]).all() or np.any(lower > upper):
            raise ValueError(f"Invalid bounds in {link.get('name')}")
        # ElementTree.find selects the first origin, as common URDF loaders do.
        # Aibo's base visual contains a redundant second origin; preserve source semantics.
        t = origin(visual.find("origin"))
        edges.extend(box_edges(lower, upper) @ t[:3, :3].T + t[:3, 3])
    return np.array(edges, dtype=float).reshape(-1, 2, 3)


def load_urdf(path: Path, *, load_display_anchors: bool = True) -> UrdfModel:
    root = ET.parse(path).getroot()
    bounds = json.loads(path.with_name("mesh_bounds.json").read_text())
    links = {}
    for e in root.findall("link"):
        name = e.attrib["name"]
        if name in links:
            raise ValueError(f"Duplicate link: {name}")
        mass_element = e.find("inertial/mass")
        mass = float(mass_element.attrib["value"]) if mass_element is not None else 0.0
        if not math.isfinite(mass) or mass < 0:
            raise ValueError(f"Invalid mass for {name}")
        links[name] = Link(name, mass, origin(e.find("inertial/origin"))[:3, 3], visual_edges(e, bounds))
    joints = []
    names, children = set(), set()
    for e in root.findall("joint"):
        name, kind = e.attrib["name"], e.attrib["type"]
        if kind not in ("fixed", "revolute", "continuous"):
            raise ValueError(f"Unsupported joint type: {kind}")
        if e.find("mimic") is not None:
            raise ValueError(f"Mimic joint requires explicit support: {name}")
        parent, child = e.find("parent").attrib["link"], e.find("child").attrib["link"]
        if name in names or child in children or parent not in links or child not in links:
            raise ValueError(f"Invalid URDF topology at {name}")
        names.add(name); children.add(child)
        a = e.find("axis")
        axis = vector(a.get("xyz", "1 0 0")) if a is not None else np.array([1., 0., 0.])
        if kind != "fixed" and np.linalg.norm(axis) < 1e-12:
            raise ValueError(f"Zero axis at {name}")
        limit = e.find("limit")
        effort = float(limit.attrib["effort"]) if limit is not None and "effort" in limit.attrib else None
        if effort is not None and (not math.isfinite(effort) or effort < 0):
            raise ValueError(f"Invalid effort limit at {name}")
        lower, upper = -math.inf, math.inf
        if kind == "revolute":
            if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
                raise ValueError(f"Missing angular limits for {name}")
            lower, upper = (math.degrees(float(limit.attrib[k])) for k in ("lower", "upper"))
            if not math.isfinite(lower) or not math.isfinite(upper) or lower > upper:
                raise ValueError(f"Invalid angular limits for {name}: {lower}..{upper}")
        joints.append(Joint(name, kind, parent, child, origin(e.find("origin")), axis, effort, lower, upper))
    roots = links.keys() - children
    if len(roots) != 1:
        raise ValueError(f"Expected one URDF root, found {roots}")
    root_link = next(iter(roots))
    ordered, visited = [], {root_link}
    while joints:
        ready = [j for j in joints if j.parent in visited]
        if not ready:
            raise ValueError("URDF contains disconnected links or a cycle")
        for j in ready:
            ordered.append(j); visited.add(j.child); joints.remove(j)
    if visited != links.keys():
        raise ValueError("URDF has disconnected links")
    anchors = {}
    audit_path = path.with_name("axis_audit.json")
    if load_display_anchors and path.parent.name == "pupper_v3":
        audit = json.loads(audit_path.read_text())["joints"]
        if set(audit) != {j.name for j in ordered if j.kind != "fixed"}:
            raise ValueError("Pupper axis audit does not cover every movable joint")
        for j in ordered:
            if j.kind == "fixed":
                continue
            row = audit[j.name]
            center = np.asarray(row["center_link_m"], dtype=float)
            if center.shape != (3,) or not np.isfinite(center).all() or row["axis_distance_mm"] > .1:
                raise ValueError(f"Invalid physical joint anchor: {j.name}")
            # Project measured interface center to the URDF axis (STL/URDF rounding <0.05 mm).
            axis = j.axis / np.linalg.norm(j.axis)
            anchors[j.name] = axis * (center @ axis)
    if load_display_anchors and path.parent.name in ("go2", "aibo_ers7"):
        audit = json.loads(path.with_name("display_axis_audit.json").read_text())
        if audit["urdf_sha256"] != hashlib.sha256(path.read_bytes()).hexdigest():
            raise ValueError(f"Display axis audit is stale: {path}")
        if audit["mesh_sources_sha256"] != hashlib.sha256(path.with_name("mesh_sources.json").read_bytes()).hexdigest():
            raise ValueError(f"Display axis mesh provenance is stale: {path}")
        rows = audit["joints"]
        if set(rows) != {j.name for j in ordered if j.kind != "fixed"}:
            raise ValueError(f"Display axis audit does not cover every movable joint: {path}")
        for j in ordered:
            if j.kind == "fixed":
                continue
            row = rows[j.name]
            if row["kind"] not in ("urdf_reference", "circular_surface_projection", "shell_axis_exit"):
                raise ValueError(f"Unknown display anchor method: {j.name}")
            anchor = np.asarray(row["anchor_link_m"], dtype=float)
            if (anchor.shape != (3,) or not np.isfinite(anchor).all()
                    or np.linalg.norm(np.cross(anchor, j.axis)) > 1e-10):
                raise ValueError(f"Display anchor is not on the joint axis: {j.name}")
            if row["kind"] == "urdf_reference" and np.any(anchor != 0):
                raise ValueError(f"Reference anchor must be zero: {j.name}")
            anchors[j.name] = anchor
    return UrdfModel(root.attrib["name"], root_link, links, tuple(ordered), anchors)


def effector_point(model: UrdfModel, poses: dict[str, np.ndarray], link_name: str) -> np.ndarray:
    link = model.links[link_name]
    local = link.edges.reshape(-1, 3).mean(0) if link.edges.size else np.zeros(3)
    return poses[link_name][:3, :3] @ local + poses[link_name][:3, 3]


def solve_reach(model: UrdfModel, angles: dict[str, float], root: np.ndarray,
                target: np.ndarray, effector: str, iterations: int = 60) -> tuple[dict[str, float], float]:
    """Damped Jacobian IK over the effector's ancestors, within URDF angle limits."""
    parents = {j.child: j for j in model.joints}
    chain = []
    link = effector
    while link in parents:
        joint = parents[link]
        if joint.kind != "fixed":
            chain.append(joint)
        link = joint.parent
    result = {j.name: j.clamp(angles.get(j.name, 0.)) for j in model.movable}
    best, best_error = result.copy(), float('inf')
    for _ in range(iterations):
        poses = model.forward(result, root)
        point = effector_point(model, poses, effector)
        error = target - point
        distance = float(np.linalg.norm(error))
        if distance < best_error:
            best, best_error = result.copy(), distance
        if distance < 1e-4 or not chain:
            break
        columns = []
        for joint in chain:
            frame = poses[joint.parent] @ joint.transform
            axis = frame[:3, :3] @ (joint.axis / np.linalg.norm(joint.axis))
            columns.append(np.cross(axis, point - frame[:3, 3]))
        jacobian = np.array(columns).T
        delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + np.eye(3)*1e-5, error)
        delta *= min(1., .25 / max(float(np.max(np.abs(delta))), 1e-12))
        for joint, step in zip(chain, delta):
            result[joint.name] = joint.clamp(result[joint.name] + math.degrees(step))
    return best, best_error
