"""Measure 37 mm circular STL interface faces; retain residuals against URDF axes.
Run: uv run --with trimesh --with scipy python models/pupper_v3/audit_axes.py
The radius band identifies these interface disks; it is not a universal joint detector.
Axes are unoriented lines, so parallel and antiparallel normals are equivalent.
"""
import sys,json,xml.etree.ElementTree as E
from pathlib import Path
import numpy as np,trimesh
from scipy.spatial import ConvexHull
sys.path.insert(0,str(Path(__file__).resolve().parents[2]));import urdf_linkage as u
p=Path(__file__).resolve().parent;m=u.load_urdf(p/'robot.urdf', load_display_anchors=False);poses=m.forward({});r=E.parse(p/'robot.urdf');results={}
for j in m.movable:
 v=r.find(f"link[@name='{j.child}']/visual");ms=v.find('geometry/mesh');mesh=trimesh.load(p/ms.get('filename'),force='mesh');VT=u.origin(v.find('origin'));scale=u.vector(ms.get('scale','1 1 1')); candidates=[]
 axis=poses[j.child][:3,:3]@j.axis; origin=poses[j.child][:3,3]
 for k,facet in enumerate(mesh.facets):
  points=mesh.vertices[np.unique(mesh.faces[facet])];normal=mesh.facets_normal[k];_,_,vh=np.linalg.svd(points-points.mean(0),full_matrices=False);basis=vh[:2].T;xy=(points-points.mean(0))@basis
  if len(points)<16:continue
  hull=xy[ConvexHull(xy).vertices]
  if len(hull)<16:continue
  c=np.linalg.lstsq(np.c_[2*hull,np.ones(len(hull))],(hull*hull).sum(1),rcond=None)[0];rad=np.sqrt(c[2]+c[:2]@c[:2]);err=np.max(np.abs(np.linalg.norm(hull-c[:2],axis=1)-rad))
  if not (.0184<rad<.0186 and err<.00001):continue
  center=points.mean(0)+basis@c[:2];local=VT[:3,:3]@(center*scale)+VT[:3,3];world=poses[j.child][:3,:3]@local+origin
  n=poses[j.child][:3,:3]@VT[:3,:3]@(normal/scale);n/=np.linalg.norm(n);angle=np.degrees(np.arccos(np.clip(abs(axis@n),0,1)));dist=np.linalg.norm(np.cross(world-origin,axis))
  candidates.append(dict(facet=int(k),center_mesh_m=center.tolist(),center_link_m=local.tolist(),center_world_m=world.tolist(),normal_world=n.tolist(),radius_mm=rad*1000,max_radial_residual_mm=err*1000,axis_angle_deg=angle,axis_distance_mm=dist*1000,hull_vertices=len(hull)))
 candidates.sort(key=lambda x:(x['axis_angle_deg'],x['axis_distance_mm']))
 assert candidates,j.name
 results[j.name]=candidates[0]
 print(j.name,json.dumps(candidates[0]))
(p/'axis_audit.json').write_text(json.dumps({'method':'Least-squares circle fitted to convex hull of planar STL facets; radius 18.4–18.6 mm interface disks, at least 16 hull vertices, radial residual below 0.01 mm. Incoming-axis orientation selects disk; full residuals retained.','joints':results},indent=2)+'\n')
