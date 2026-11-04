"""Render source meshes and before/after linkage in an identical pose/camera.

Diagnostic only; does not change the interactive viewer's appearance or camera.
uv run --with trimesh --with scipy --with pycollada python tools/render_axis_review.py
"""
from pathlib import Path
import json
import os
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
import sys
from unittest.mock import patch
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from audit_display_axes import mesh_visual
import urdf_linkage as u
import pygame_mass_viewer as viewer


def render(key):
    path = u.MODEL_PATHS[key]
    folder = path.parent
    with patch('sys.argv', ['viewer', '--model', key, '--no-babble', '--no-stage3-reach']):
        state = viewer.create_urdf_viewer(key, viewer.parse_args())
    model = state.model
    poses = model.forward(state.angles, state.root_transform)
    old = u.load_urdf(path, load_display_anchors=False)
    sources = json.loads((folder/'mesh_sources.json').read_text())['meshes']
    surfaces = []
    for link in ET.parse(path).findall('link'):
        for visual in link.findall('visual'):
            if visual.find('geometry/mesh') is None:
                continue
            mesh, vertices, _, _, _ = mesh_visual(folder, visual, sources)
            t = poses[link.attrib['name']]
            vertices = vertices @ t[:3, :3].T+t[:3, 3]
            surfaces.append((vertices, mesh.faces, link.attrib['name']))
    points = np.concatenate([s[0] for s in surfaces])
    center = (points.min(0)+points.max(0))/2
    half = np.ptp(points, axis=0).max()*.55
    fig = plt.figure(figsize=(18, 7), facecolor='#fafbfc')
    for slot, title in enumerate(('Source visual meshes', 'Before: URDF frame origins', 'After: audited display landmarks'), 1):
        ax = fig.add_subplot(1, 3, slot, projection='3d', computed_zorder=False)
        if slot == 1:
            for vertices, faces, name in surfaces:
                color = '#bac4ce' if name == model.root else '#647c91'
                ax.add_collection3d(Poly3DCollection(vertices[faces], facecolors=color, shade=True))
        else:
            m = old if slot == 2 else model
            incoming = {j.child: j for j in m.joints}
            root = m.links[m.root]
            t = poses[m.root]
            origin = t[:3, :3] @ root.edges.reshape(-1, 3).mean(0)+t[:3, 3]
            if slot == 3:
                segments = m.skeleton_segments(poses)
            else:
                segments = [(m.joint_point(incoming[j.parent], poses) if j.parent in incoming else origin,
                             m.joint_point(j, poses)) for j in m.joints]
            for a, p in segments:
                ax.plot(*np.array([a, p]).T, color='#7d8690', linewidth=2)
            for group in viewer.urdf_joint_groups(m, poses):
                p = m.joint_point(group[0], poses)
                ax.scatter(*p, color='#b23a38', s=22+15*(len(group)-1), depthshade=False)
                if len(group)>1:
                    ax.text(*p, f' {len(group)} DOF', fontsize=10)
        ax.set(xlim=(center[0]-half, center[0]+half), ylim=(center[1]-half, center[1]+half), zlim=(center[2]-half, center[2]+half))
        ax.set_box_aspect((1, 1, 1), zoom=1.15)
        ax.view_init(elev=24, azim=-56)
        ax.set_axis_off()
        ax.set_title(title, loc='left', x=.03, y=.96, fontsize=12)
    fig.suptitle(f'{u.MODEL_LABELS[key]} | mesh / linkage comparison', fontsize=16, y=.98)
    note = ('Hip/thigh: projected circular faces. Knees: URDF reference points.\nMesh-to-axis residual: 0.665-0.694 mm; kinematic axes unchanged.' if key == 'go2' else
            'Shoulders: body-shell axis exits, NOT measured motor centers.\nCoarse source meshes are not calibrated to Tekkotsu. Neck/tail shared pivots retained.')
    fig.text(.5, .025, note+'\nSame pose and camera. Gray lines: connectivity, not material shape.', ha='center', fontsize=11, color='#465565')
    fig.subplots_adjust(left=0, right=1, top=.86, bottom=.18, wspace=.02)
    fig.savefig(folder/'axis_comparison.png', dpi=140)
    plt.close(fig)
    print(folder/'axis_comparison.png')


if __name__ == '__main__':
    for key in ('go2', 'aibo'):
        render(key)
