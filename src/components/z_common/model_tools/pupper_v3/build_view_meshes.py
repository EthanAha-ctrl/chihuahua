"""Rebuild display-only decimated meshes; URDF/STL geometry remains unchanged.
uv run --with trimesh --with fast-simplification python -m src.components.z_common.model_tools.pupper_v3.build_view_meshes
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
p=Path(__file__).resolve().parents[5]/'vendor/reference/urdf/pupper_v3'
arrays={}
for link in ET.parse(p/'robot.urdf').findall('link'):
    visual=link.find('visual')
    if visual is None:continue
    mesh=visual.find('geometry/mesh');m=trimesh.load(p/mesh.get('filename'),force='mesh')
    m=m.simplify_quadric_decimation(face_count=min(1200,len(m.faces)))
    arrays[link.get('name')]=m.triangles.astype(np.float32)
np.savez_compressed(p/'view_meshes.npz',**arrays)
