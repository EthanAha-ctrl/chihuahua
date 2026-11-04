"""Reproduce mesh-derived display anchors without editing URDF kinematics.

uv run --with trimesh --with scipy --with pycollada python tools/audit_display_axes.py
"""
from pathlib import Path
import hashlib
import json
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import urdf_linkage as u


def mesh_visual(folder, visual, sources):
    shape = visual.find('geometry/mesh')
    uri = shape.attrib['filename']
    entry = sources[uri]
    path = folder / entry['file']
    if hashlib.sha256(path.read_bytes()).hexdigest() != entry['sha256']:
        raise ValueError(f'Mesh hash mismatch: {path}')
    mesh = trimesh.load(path, force='scene').to_mesh()
    mesh.merge_vertices()
    t = u.origin(visual.find('origin'))
    scale = u.vector(shape.get('scale', '1 1 1'))
    vertices = (mesh.vertices * scale) @ t[:3, :3].T + t[:3, 3]
    return mesh, vertices, t, scale, uri


def circular_faces(mesh, vertices, transform, scale, axis):
    candidates, degenerate = [], 0
    for index, facet in enumerate(mesh.facets):
        normal = transform[:3, :3] @ (mesh.facets_normal[index] / scale)
        if np.linalg.norm(normal) < 1e-12:
            degenerate += 1
            continue
        normal /= np.linalg.norm(normal)
        angle = np.degrees(np.arccos(np.clip(abs(normal @ axis), 0, 1)))
        if angle > 1:
            continue
        points = vertices[np.unique(mesh.faces[facet])]
        if len(points) < 12:
            continue
        mean = points.mean(0)
        _, singular, vh = np.linalg.svd(points - mean, full_matrices=False)
        if singular[1] < 1e-10:
            degenerate += 1
            continue
        basis = vh[:2].T
        xy = (points - mean) @ basis
        hull = xy[ConvexHull(xy).vertices]
        if len(hull) < 12:
            continue
        coeff = np.linalg.lstsq(np.c_[2*hull, np.ones(len(hull))], (hull*hull).sum(1), rcond=None)[0]
        radius = np.sqrt(coeff[2] + coeff[:2] @ coeff[:2])
        residual = np.max(abs(np.linalg.norm(hull-coeff[:2], axis=1)-radius))
        center = mean + basis @ coeff[:2]
        if .006 < radius < .06 and residual < .00015:
            candidates.append(dict(facet=int(index), center_link_m=center.tolist(), radius_mm=float(radius*1000),
                                   circle_residual_mm=float(residual*1000), axis_distance_mm=float(np.linalg.norm(np.cross(center, axis))*1000),
                                   normal_axis_angle_deg=float(angle), hull_vertices=len(hull)))
    return candidates, degenerate


def positive_ray_hits(triangles, point, direction):
    edge1, edge2 = triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0]
    h = np.cross(direction, edge2)
    determinant = np.einsum('ij,ij->i', edge1, h)
    valid = abs(determinant) > 1e-12
    inverse = np.zeros_like(determinant)
    inverse[valid] = 1/determinant[valid]
    s = point-triangles[:, 0]
    a = np.einsum('ij,ij->i', s, h)*inverse
    q = np.cross(s, edge1)
    b = q @ direction * inverse
    distance = np.einsum('ij,ij->i', edge2, q)*inverse
    hit = valid & (a >= -1e-10) & (b >= -1e-10) & (a+b <= 1+1e-10) & (distance > 1e-8)
    return distance[hit]


def audit(key):
    path = u.MODEL_PATHS[key]
    folder = path.parent
    model = u.load_urdf(path, load_display_anchors=False)
    xml = ET.parse(path)
    sources = json.loads((folder/'mesh_sources.json').read_text())
    rows = {j.name: dict(kind='urdf_reference', anchor_link_m=[0., 0., 0.],
                        reason='URDF axis reference; no calibrated physical center inferred from this mesh.') for j in model.movable}
    if key == 'go2':
        for joint in model.movable:
            visual = xml.find(f"link[@name='{joint.child}']/visual")
            mesh, vertices, t, scale, uri = mesh_visual(folder, visual, sources['meshes'])
            candidates, degenerate = circular_faces(mesh, vertices, t, scale, joint.axis)
            rows[joint.name].update(mesh=uri, circular_candidates=candidates, degenerate_facets_excluded=degenerate)
            # Reproducible surface selection: 28.5 mm hip mounting lip;
            # nearest of the 47.75 mm thigh circular faces. These are surface
            # markers, not inferred motor COMs or changed kinematic pivots.
            radius = 28.5 if '_hip_' in joint.name else 47.75
            matches = [c for c in candidates if abs(c['radius_mm']-radius)<.01 and c['axis_distance_mm']<1.]
            if '_calf_' in joint.name:
                continue
            if not matches:
                raise ValueError(f'Expected mounting surface absent: {joint.name}')
            chosen = min(matches, key=lambda c: abs(np.dot(c['center_link_m'], joint.axis)))
            anchor = joint.axis * np.dot(chosen['center_link_m'], joint.axis)
            rows[joint.name].update(kind='circular_surface_projection', anchor_link_m=anchor.tolist(), measured=chosen,
                                   reason='Mesh circular face projected onto unchanged URDF axis; mesh-axis mismatch is recorded, not repaired.')
    else:
        visual = xml.find(f"link[@name='{model.root}']/visual")
        mesh, vertices, _, _, uri = mesh_visual(folder, visual, sources['meshes'])
        poses = model.forward({})
        for joint in model.movable:
            if not (joint.name.startswith('leg') and joint.name.endswith('1')):
                continue
            frame = poses[joint.child]
            point = frame[:3, 3]
            successor = next(j for j in model.joints if j.parent == joint.child)
            direction = poses[successor.child][:3, 3]-point
            direction /= np.linalg.norm(direction)
            if np.linalg.norm(np.cross(direction, frame[:3, :3] @ joint.axis)) > 1e-10:
                raise ValueError(f'Shoulder exit ray not on joint axis: {joint.name}')
            hits = positive_ray_hits(vertices[mesh.faces], point, direction)
            # This source has exactly one shell intersection on each ray.
            if len(hits) != 1:
                raise ValueError(f'Ambiguous shell intersection at {joint.name}: {hits}')
            exit_point = point + hits[0]*direction
            anchor = frame[:3, :3].T @ (exit_point-point)
            rows[joint.name] = dict(kind='shell_axis_exit', anchor_link_m=anchor.tolist(), mesh=uri,
                                    exit_root_m=exit_point.tolist(), distance_mm=float(hits[0]*1000),
                                    reason='Coarse source body STL intersected by the Tekkotsu/URDF axis toward this leg; display landmark only, not a measured motor center.')
    result = dict(model=key, urdf_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  mesh_sources_sha256=hashlib.sha256((folder/'mesh_sources.json').read_bytes()).hexdigest(),
                  joints=rows)
    (folder/'display_axis_audit.json').write_text(json.dumps(result, indent=2)+'\n')
    print(key, {name: row['anchor_link_m'] for name, row in rows.items() if row['kind'] != 'urdf_reference'})


if __name__ == '__main__':
    for key in ('go2', 'aibo'):
        audit(key)
