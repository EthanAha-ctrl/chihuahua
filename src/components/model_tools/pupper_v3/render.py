"""Render bundled URDF visual meshes: uv run --with trimesh python -m src.components.model_tools.pupper_v3.render."""
from pathlib import Path
import sys, xml.etree.ElementTree as ET
import numpy as np
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[4]
from src.components import urdf_linkage as u
folder=ROOT/'vendor/reference/urdf/pupper_v3'
output=ROOT/'artifacts/model_review/pupper_v3'
output.mkdir(parents=True,exist_ok=True)
model=u.load_urdf(folder/'robot.urdf');poses=model.forward({});mass,com=model.mass_com(poses)
fig=plt.figure(figsize=(14,7),facecolor='#fafbfc')
for slot,(elev,azim,title) in enumerate([(22,-52,'Perspective'),(10,-90,'Side view')],1):
 ax=fig.add_subplot(1,2,slot,projection='3d',computed_zorder=False);allpoints=[]
 for l in ET.parse(folder/'robot.urdf').findall('link'):
  for v in l.findall('visual'):
   m=v.find('geometry/mesh')
   if m is None:continue
   mesh=trimesh.load(folder/m.get('filename'),force='mesh');T=poses[l.get('name')]@u.origin(v.find('origin'))
   vertices=mesh.vertices*u.vector(m.get('scale','1 1 1'));vertices=vertices@T[:3,:3].T+T[:3,3];allpoints.extend(vertices)
   color='#d7dde3' if l.get('name')=='base_link' else ['#317caa','#e7a34c','#556574'][int(l.get('name')[-1])-1]
   ax.add_collection3d(Poly3DCollection(vertices[mesh.faces],facecolors=color,shade=True,zsort='average'))
 points=np.array(allpoints);center=(points.max(0)+points.min(0))/2;span=np.ptp(points,axis=0).max()/2*1.1
 ax.set(xlim=(center[0]-span,center[0]+span),ylim=(center[1]-span,center[1]+span),zlim=(center[2]-span,center[2]+span));ax.set_box_aspect((1,1,1));ax.view_init(elev,azim);ax.set_axis_off();ax.set_title(title,color='#3c4958')
fig.suptitle(f'Pupper v3 | upstream URDF + STL | 12 actuated joints | URDF mass {mass:.3f} kg',fontsize=17,color='#233345')
fig.text(.5,.04,'Zero joint pose; full visual meshes. Kinematic visualization, not a dynamics validation.',ha='center',color='#536273')
fig.subplots_adjust(left=0,right=1,bottom=.08,top=.9,wspace=0)
fig.savefig(output/'pupper_v3.png',dpi=160,facecolor=fig.get_facecolor());print('mass',mass)
