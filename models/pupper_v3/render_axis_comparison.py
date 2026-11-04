"""Render bundled URDF visual meshes: uv run --with trimesh python models/pupper_v3/render.py."""
from pathlib import Path
import sys, xml.etree.ElementTree as ET
import numpy as np
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
import urdf_linkage as u
folder=Path(__file__).resolve().parent
model=u.load_urdf(folder/'robot.urdf');poses=model.forward({});mass,com=model.mass_com(poses)
fig=plt.figure(figsize=(14,7),facecolor='#fafbfc')
for slot,(elev,azim,title) in enumerate([(22,-52,'Perspective'),(22,-52,'Corrected linkage / physical interface centers')],1):
 ax=fig.add_subplot(1,2,slot,projection='3d',computed_zorder=False);allpoints=[]
 for l in ET.parse(folder/'robot.urdf').findall('link'):
  for v in l.findall('visual'):
   m=v.find('geometry/mesh')
   if m is None:continue
   mesh=trimesh.load(folder/m.get('filename'),force='mesh');T=poses[l.get('name')]@u.origin(v.find('origin'))
   vertices=mesh.vertices*u.vector(m.get('scale','1 1 1'));vertices=vertices@T[:3,:3].T+T[:3,3];allpoints.extend(vertices)
   color='#d7dde3' if l.get('name')=='base_link' else ['#317caa','#e7a34c','#556574'][int(l.get('name')[-1])-1]
   if slot == 1: ax.add_collection3d(Poly3DCollection(vertices[mesh.faces],facecolors=color,shade=True,zsort='average'))
 incoming={j.child:j for j in model.joints}
 mounts=[model.joint_point(j,poses) for j in model.joints if j.parent==model.root]
 mounts.sort(key=lambda x:np.arctan2(x[1],x[0]))
 for a,b in zip(mounts,mounts[1:]+mounts[:1]):
  if slot==2:ax.plot(*np.array([a,b]).T,color='#556574',lw=3)
 for j in model.joints:
  point=model.joint_point(j,poses)
  if j.parent in incoming and slot==2:
   parent=model.joint_point(incoming[j.parent],poses);ax.plot(*np.array([parent,point]).T,color='#556574',lw=3)
  if j.kind=='fixed':continue
  axis=poses[j.child][:3,:3]@j.axis;axis/=np.linalg.norm(axis)
  ax.plot(*np.array([point-axis*.024,point+axis*.024]).T,color='#d58312',lw=2)
  radial=np.cross(axis,[0.,0.,1.])
  if np.linalg.norm(radial)<1e-8:radial=np.cross(axis,[0.,1.,0.])
  radial/=np.linalg.norm(radial);tangent=np.cross(axis,radial)
  ring=np.array([point+.0185*(np.cos(a)*radial+np.sin(a)*tangent) for a in np.linspace(0,2*np.pi,49)])
  ax.plot(*ring.T,color='#2483b5',lw=1.5);ax.scatter(*point,color='#c44439',s=15)
 points=np.array(allpoints);center=(points.max(0)+points.min(0))/2;span=np.ptp(points,axis=0).max()/2*1.1
 ax.set(xlim=(center[0]-span,center[0]+span),ylim=(center[1]-span,center[1]+span),zlim=(center[2]-span,center[2]+span));ax.set_box_aspect((1,1,1));ax.view_init(elev,azim);ax.set_axis_off();ax.set_title(title,color='#3c4958')
fig.suptitle(f'Pupper v3 | upstream URDF + STL | 12 actuated joints | URDF mass {mass:.3f} kg',fontsize=17,color='#233345')
fig.text(.5,.04,'Same zero pose and camera | blue: fitted interface disks | gold: axes | red: distinct interface centers',ha='center',color='#536273')
fig.subplots_adjust(left=0,right=1,bottom=.08,top=.9,wspace=0)
fig.savefig(folder/'axis_comparison.png',dpi=160,facecolor=fig.get_facecolor());print('mass',mass)
