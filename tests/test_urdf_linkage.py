import math
import json
import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np
import pygame
from src.components import pygame_mass_viewer as viewer
from src.components import urdf_linkage as urdf
class UrdfLinkageTests(unittest.TestCase):
    def test_pupper_mirrored_meshes_mass_and_feet(self):
        model = urdf.load_urdf(urdf.MODEL_PATHS["pupper"])
        self.assertEqual(len(model.movable), 12)
        poses = model.forward({})
        self.assertAlmostEqual(model.mass_com(poses)[0], 3.17)
        feet = [n for n in model.links if n.endswith("_foot")]
        self.assertEqual(len(feet), 4)
        for link in model.links.values():
            self.assertTrue(np.isfinite(link.edges).all())
        for foot in feet:
            joint = next(j for j in model.joints if j.child == foot)
            self.assertEqual(joint.kind, "fixed")
            self.assertGreater(np.linalg.norm(joint.transform[:3, 3]), 0.05)


    def test_pupper_physical_markers_preserve_axes_and_fk(self):
        from dataclasses import replace
        model = urdf.load_urdf(urdf.MODEL_PATHS["pupper"])
        bare = replace(model, display_anchors={})
        zero = model.forward({})
        joints = {j.name: j for j in model.movable}
        a, b = joints["leg_front_r_1"], joints["leg_front_r_2"]
        np.testing.assert_allclose(zero[a.child][:3, 3], zero[b.child][:3, 3])
        distance = np.linalg.norm(model.joint_point(a, zero)-model.joint_point(b, zero))
        self.assertAlmostEqual(distance, math.hypot(.0346, .01085), places=6)
        self.assertEqual(len(viewer.urdf_joint_groups(model, zero)), 12)
        rng = np.random.default_rng(61)
        for _ in range(10):
            q = {j.name: rng.uniform(j.lower_deg, j.upper_deg) for j in model.movable}
            poses = model.forward(q)
            original = bare.forward(q)
            for name in poses:
                np.testing.assert_array_equal(poses[name], original[name])
            for j in model.movable:
                point = model.joint_point(j, poses)
                axis = poses[j.child][:3, :3] @ j.axis
                self.assertLess(np.linalg.norm(np.cross(point-poses[j.child][:3, 3], axis)), 1e-12)
        with patch("sys.argv", ["viewer", "--model", "pupper"]):
            state = viewer.create_urdf_viewer("pupper", viewer.parse_args())
        before = state.angles.copy()
        self.assertTrue(state.autoplay)
        viewer.update_urdf_motion(state, .1, False)
        self.assertNotEqual(state.angles, before)
        with patch("sys.argv", ["viewer", "--model", "pupper", "--no-babble"]):
            state = viewer.create_urdf_viewer("pupper", viewer.parse_args())
        before = state.angles.copy()
        viewer.update_urdf_motion(state, .1, False)
        self.assertEqual(state.angles, before)

    def test_go2_aibo_audited_markers_and_unchanged_physics(self):
        from dataclasses import replace
        rng = np.random.default_rng(132)
        for key in ('go2', 'aibo'):
            model = urdf.load_urdf(urdf.MODEL_PATHS[key])
            reference = urdf.load_urdf(urdf.MODEL_PATHS[key], load_display_anchors=False)
            with patch('sys.argv', ['viewer', '--model', key]):
                state = viewer.create_urdf_viewer(key, viewer.parse_args())
            self.assertTrue(state.autoplay)
            for _ in range(8):
                q = {j.name: rng.uniform(j.lower_deg, j.upper_deg) for j in model.movable}
                poses, original = model.forward(q), reference.forward(q)
                for name in poses:
                    np.testing.assert_array_equal(poses[name], original[name])
                for j in model.movable:
                    displacement = model.joint_point(j, poses)-poses[j.child][:3, 3]
                    axis = poses[j.child][:3, :3] @ j.axis
                    self.assertLess(np.linalg.norm(np.cross(displacement, axis)), 1e-12)
                state.angles = q
                a = viewer.make_urdf_telemetry(state, poses)
                b = viewer.make_urdf_telemetry(replace(state, model=reference), original)
                self.assertEqual([r.required_torque_nm for r in a.torque_rows], [r.required_torque_nm for r in b.torque_rows])
                np.testing.assert_array_equal(model.mass_com(poses)[1], reference.mass_com(original)[1])
            moved, original_state = replace(state, angles=state.angles.copy()), replace(state, model=reference, angles=state.angles.copy())
            viewer.update_urdf_motion(moved, .02, False)
            viewer.update_urdf_motion(original_state, .02, False)
            self.assertEqual(moved.angles, original_state.angles)
            foot = next(n for n in model.links if n.endswith(('_foot', '_paw')))
            target = urdf.effector_point(model, poses, foot)+np.array([.005, 0, 0])
            self.assertEqual(urdf.solve_reach(model, q, np.eye(4), target, foot),
                             urdf.solve_reach(reference, q, np.eye(4), target, foot))

    def test_aibo_shoulders_separate_while_neck_tail_pivots_remain(self):
        model = urdf.load_urdf(urdf.MODEL_PATHS['aibo'])
        joints = {j.name: j for j in model.movable}
        poses = model.forward({})
        for side in ('F', 'B'):
            right, left = joints[f'legR{side}1'], joints[f'legL{side}1']
            np.testing.assert_array_equal(poses[right.child][:3, 3], poses[left.child][:3, 3])
            distance = np.linalg.norm(model.joint_point(right, poses)-model.joint_point(left, poses))
            self.assertGreater(distance, .092)
            self.assertLess(distance, .093)
        groups = [{j.name for j in g} for g in viewer.urdf_joint_groups(model, poses)]
        self.assertEqual(len(groups), 18)
        self.assertIn({'neck', 'headPan'}, groups)
        self.assertIn({'tailTilt', 'tailPan'}, groups)

    def test_aibo_skeleton_body_legs_and_head_topology(self):
        model = urdf.load_urdf(urdf.MODEL_PATHS['aibo'])
        edges = model.aibo_linkage_edges()
        self.assertEqual(len(edges), 24)
        self.assertEqual(set(edges[:4]), {
            ('legRF1', 'legLF1'), ('legLF1', 'legLB1'),
            ('legLB1', 'legRB1'), ('legRB1', 'legRF1')})
        for leg in ('RF', 'LF', 'RB', 'LB'):
            self.assertEqual([e for e in edges[4:16] if e[0].startswith(f'leg{leg}')], [
                (f'leg{leg}1', f'leg{leg}2'), (f'leg{leg}2', f'leg{leg}3'),
                (f'leg{leg}3', f'{leg}_paw_joint')])
        used = {name for edge in edges for name in edge}
        self.assertFalse({'earR_frame', 'earL_frame', 'base_link'} & used)
        self.assertTrue({j.name for j in model.movable} <= used)
        self.assertIn(('front_body_midpoint', 'neck'), edges)
        self.assertIn(('rear_body_midpoint', 'tailTilt'), edges)
        self.assertTrue({'earR_frame', 'earL_frame'} <= {j.name for j in model.joints})
        rng = np.random.default_rng(39)
        zero = model.skeleton_segments(model.forward({}))
        for _ in range(10):
            q = {j.name: rng.uniform(j.lower_deg, j.upper_deg) for j in model.movable}
            poses = model.forward(q)
            segments = model.skeleton_segments(poses)
            np.testing.assert_allclose(segments[:4], zero[:4], atol=1e-14)
            for edge, segment in zip(edges, segments):
                self.assertTrue(np.isfinite(segment).all())
                if edge == ('front_body_midpoint', 'neck'):
                    np.testing.assert_allclose(segment[0], (segments[0][0]+segments[0][1])/2)
                if edge in (('neck', 'headPan'), ('tailTilt', 'tailPan')):
                    np.testing.assert_allclose(segment[0], segment[1], atol=1e-14)

    def test_go2_skeleton_excludes_auxiliary_frames_and_body_spokes(self):
        model = urdf.load_urdf(urdf.MODEL_PATHS['go2'])
        edges = model.go2_linkage_edges()
        self.assertEqual(len(edges), 16)  # four torso edges + three per leg
        self.assertEqual(set(edges[:4]), {
            ('FL_hip_joint', 'FR_hip_joint'), ('FR_hip_joint', 'RR_hip_joint'),
            ('RR_hip_joint', 'RL_hip_joint'), ('RL_hip_joint', 'FL_hip_joint')})
        for leg in ('FL', 'FR', 'RL', 'RR'):
            self.assertEqual([e for e in edges[4:] if e[0].startswith(leg)], [
                (f'{leg}_hip_joint', f'{leg}_thigh_joint'),
                (f'{leg}_thigh_joint', f'{leg}_calf_joint'),
                (f'{leg}_calf_joint', f'{leg}_foot_joint')])
        used = {name for edge in edges for name in edge}
        self.assertEqual(len(used), 16)
        self.assertFalse(any('rotor' in n or 'calflower' in n for n in used))
        self.assertTrue(any('rotor' in j.name for j in model.joints))
        self.assertTrue(any('calflower' in j.name for j in model.joints))

    def test_go2_surface_measurements_and_original_axis_offsets(self):
        model = urdf.load_urdf(urdf.MODEL_PATHS['go2'])
        audit = json.loads(urdf.MODEL_PATHS['go2'].with_name('display_axis_audit.json').read_text())
        for prefix in ('FL', 'FR', 'RL', 'RR'):
            hip, thigh, calf = [next(j for j in model.movable if j.name == f'{prefix}_{part}_joint') for part in ('hip', 'thigh', 'calf')]
            self.assertAlmostEqual(abs(thigh.transform[1, 3]), .0955)
            self.assertAlmostEqual(calf.transform[2, 3], -.213)
            self.assertAlmostEqual(abs(model.display_anchors[hip.name][0]), .06131595, places=7)
            self.assertAlmostEqual(abs(model.display_anchors[thigh.name][1]), .04849499, places=7)
            self.assertEqual(audit['joints'][calf.name]['kind'], 'urdf_reference')
            for j in (hip, thigh):
                measured = audit['joints'][j.name]['measured']
                self.assertLess(measured['circle_residual_mm'], .001)
                self.assertLess(measured['axis_distance_mm'], .7)
        self.assertEqual(len(viewer.urdf_joint_groups(model, model.forward({}))), 12)

    def test_display_audit_missing_stale_or_off_axis_fails(self):
        path = urdf.MODEL_PATHS['aibo']
        real_read = type(path).read_text
        audit = json.loads(path.with_name('display_axis_audit.json').read_text())
        import copy
        for problem in ('missing', 'stale', 'off_axis', 'incomplete'):
            modified = copy.deepcopy(audit)
            if problem == 'stale':
                modified['urdf_sha256'] = 'invalid'
            elif problem == 'off_axis':
                modified['joints']['legRF1']['anchor_link_m'][1] = .01
            elif problem == 'incomplete':
                del modified['joints']['legRF1']
            def read(p, *args, **kwargs):
                if p.name == 'display_axis_audit.json':
                    if problem == 'missing':
                        raise FileNotFoundError(str(p))
                    return json.dumps(modified)
                return real_read(p, *args, **kwargs)
            with patch.object(type(path), 'read_text', read):
                with self.assertRaises((ValueError, FileNotFoundError)):
                    urdf.load_urdf(path)

    def test_aibo_original_angular_ranges_restored(self):
        model = urdf.load_urdf(urdf.MODEL_PATHS["aibo"])
        reference = json.loads((urdf.MODEL_PATHS["aibo"].parent / "original_limits.json").read_text())
        joints = {j.name: j for j in model.movable}
        self.assertEqual(len(reference["limits_rad"]), 17)
        for name, limits in reference["limits_rad"].items():
            j = joints[name]
            self.assertAlmostEqual(math.radians(j.lower_deg), limits["lower"])
            self.assertAlmostEqual(math.radians(j.upper_deg), limits["upper"])
            self.assertEqual(j.clamp(-1000), j.lower_deg)
            self.assertEqual(j.clamp(1000), j.upper_deg)

    def test_aibo_face_controls_limits_and_independent_motion(self):
        with patch("sys.argv", ["viewer"]):
            state = viewer.create_urdf_viewer("aibo", viewer.parse_args())
        self.assertEqual(len(state.command_overlay.controls), 20)
        model = state.model
        joints = {j.name: j for j in model.movable}
        zero = model.forward({})
        for name, lower, upper in (("mouth", -55, 0), ("earL", 0, 10), ("earR", 0, 10)):
            j = joints[name]
            self.assertEqual(j.parent, "head")
            self.assertAlmostEqual(j.lower_deg, lower)
            self.assertAlmostEqual(j.upper_deg, upper)
            self.assertEqual(j.clamp(-1000), j.lower_deg)
            self.assertEqual(j.clamp(1000), j.upper_deg)
            moved = model.forward({name: lower if lower else upper})
            self.assertFalse(np.allclose(moved[name][:3, :3], zero[name][:3, :3]))
            for other in {"mouth", "earL", "earR", "head"} - {name}:
                np.testing.assert_allclose(moved[other], zero[other])
        rows = {r.joint: r for r in viewer.make_urdf_telemetry(state, model.forward({})).torque_rows}
        for name in ("mouth", "earL", "earR"):
            self.assertIsNone(rows[name].max_margin)

    def test_aibo_matches_tekkotsu_forward_kinematics(self):
        model = urdf.load_urdf(urdf.MODEL_PATHS["aibo"])
        reference = json.loads((urdf.MODEL_PATHS["aibo"].parent / "kinematics_reference.json").read_text())
        joints = {j.name: j for j in model.joints}
        rng = np.random.default_rng(42)
        for _ in range(20):
            angles = {j.name: rng.uniform(j.lower_deg, j.upper_deg) for j in model.movable}
            poses = model.forward(angles)
            for chain in reference["chains"]:
                world = np.eye(4)
                world[:3, :3] = urdf.rotation(np.array([0., 0., 1.]), reference["base_yaw_rad"])
                world[:3, 3] = reference["base_xyz_m"]
                for row in chain:
                    j = joints[row["joint"]]
                    # Tekkotsu updateTo: Rz(theta) Tz(d) Tx(r) Rx(alpha), then Rz(q).
                    c, s = math.cos(row["theta"]), math.sin(row["theta"])
                    ca, sa = math.cos(row["alpha"]), math.sin(row["alpha"])
                    dh = np.array([[c, -s*ca, s*sa, row["r_mm"]*c/1000],
                                   [s, c*ca, -c*sa, row["r_mm"]*s/1000],
                                   [0, sa, ca, row["d_mm"]/1000], [0, 0, 0, 1.]])
                    world = world @ dh
                    np.testing.assert_allclose(poses[j.child][:3, 3], world[:3, 3], atol=1e-10)
                    if j.kind != "fixed":
                        actual_axis = (poses[j.parent] @ j.transform)[:3, :3] @ j.axis
                        np.testing.assert_allclose(actual_axis, world[:3, 2], atol=1e-10)
                        q = np.eye(4)
                        q[:3, :3] = urdf.rotation(np.array([0., 0., 1.]), math.radians(angles[j.name]))
                        world = world @ q
        poses = model.forward({})
        np.testing.assert_allclose(poses["cuello"][:3, 3], poses["pan"][:3, 3])
        self.assertAlmostEqual(np.linalg.norm(poses["head"][:3, 3]-poses["pan"][:3, 3]), .08)

    def test_bundled_topologies_and_resources(self):
        for key, expected in (("go2", 12), ("aibo", 20)):
            model = urdf.load_urdf(urdf.MODEL_PATHS[key])
            self.assertEqual(len(model.movable), expected)
            self.assertEqual(len(model.forward({})), len(model.links))
            self.assertTrue(all(np.isfinite(l.edges).all() for l in model.links.values()))
            self.assertGreater(sum(len(l.edges) for l in model.links.values()), 0)

    def test_joint_rotation_precedes_child_translation(self):
        # Analytic two-link chain: rotate +X bone 90 degrees around +Z.
        joint = urdf.Joint("turn", "revolute", "root", "arm", np.eye(4), np.array([0., 0., 1.]))
        offset = np.eye(4)
        offset[0, 3] = 2.
        fixed = urdf.Joint("tip", "fixed", "arm", "tip", offset, np.zeros(3))
        model = urdf.UrdfModel("test", "root", {}, (joint, fixed))
        np.testing.assert_allclose(model.forward({"turn": 90})["tip"][:3, 3], [0, 2, 0], atol=1e-12)
        np.testing.assert_allclose(model.forward({"turn": 810})["tip"][:3, 3], [0, 2, 0], atol=1e-12)

    def test_rpy_rotates_joint_axis_in_parent_frame(self):
        transform = urdf.origin(ET.fromstring(f'<origin xyz="1 2 3" rpy="0 0 {math.pi/2}"/>'))
        joint = urdf.Joint("turn", "revolute", "root", "arm", transform, np.array([1., 0., 0.]))
        model = urdf.UrdfModel("test", "root", {}, (joint,))
        pose = model.forward({"turn": 90})["arm"]
        np.testing.assert_allclose(pose[:3, 3], [1, 2, 3], atol=1e-12)
        np.testing.assert_allclose(pose[:3, :3] @ [0, 1, 0], [0, 0, 1], atol=1e-12)

    def test_out_of_range_angles_saturate_at_urdf_limits(self):
        rng = np.random.default_rng(7)
        for path in urdf.MODEL_PATHS.values():
            model = urdf.load_urdf(path)
            angles = {j.name: float(rng.uniform(-1440, 1440)) for j in model.movable}
            poses = model.forward(angles)
            wrapped = model.forward({j.name: j.clamp(angles[j.name]) for j in model.movable})
            for j in model.joints:
                np.testing.assert_allclose(poses[j.child], wrapped[j.child], atol=1e-12)
                relative = np.linalg.inv(poses[j.parent]) @ poses[j.child]
                np.testing.assert_allclose(relative[:3, 3], j.transform[:3, 3], atol=1e-12)
            with self.assertRaises(ValueError):
                model.forward({"nonexistent_joint": 0})
            with self.assertRaises(ValueError):
                model.forward({model.movable[0].name: float("nan")})

    def test_manual_and_solo_motion_respect_limits_and_other_joints(self):
        with patch("sys.argv", ["viewer"]):
            args = viewer.parse_args()
        for key in urdf.MODEL_PATHS:
            state = viewer.create_urdf_viewer(key, args)
            state.angles[state.active_joint.name] = 1080
            state.solo = True
            state.autoplay = True
            before = state.angles.copy()
            viewer.update_urdf_motion(state, 1., False)
            self.assertEqual(state.angles[state.active_joint.name], state.active_joint.upper_deg)
            for name in before.keys() - {state.active_joint.name}:
                self.assertEqual(before[name], state.angles[name])
            event = pygame.event.Event(pygame.KEYDOWN, key=pygame.K_RIGHT, mod=0)
            viewer.handle_urdf_key(event, viewer.ViewerState(), state)
            self.assertEqual(state.angles[state.active_joint.name], state.active_joint.upper_deg)
            self.assertFalse(state.autoplay)
            before = state.angles.copy()
            viewer.update_urdf_motion(state, 10., False)
            self.assertEqual(state.angles, before)

    def test_missing_mesh_bounds_fails_loudly(self):
        visual = ET.fromstring('<link name="bad"><visual><geometry><mesh filename="missing.stl"/></geometry></visual></link>')
        with self.assertRaisesRegex(ValueError, "Missing precomputed mesh bounds"):
            urdf.visual_edges(visual, {})

    def test_reach_known_targets_and_preserve_unrelated_joints(self):
        for path in urdf.MODEL_PATHS.values():
            model = urdf.load_urdf(path)
            angles = {j.name: j.clamp(0.) for j in model.movable}
            effector = next(n for n in model.links if n.endswith(("_foot", "_paw")))
            target = urdf.effector_point(model, model.forward({j.name: (j.lower_deg+j.upper_deg)/2 for j in model.movable}), effector)
            result, error = urdf.solve_reach(model, angles, np.eye(4), target, effector)
            self.assertLess(error, 1e-3)
            for joint in model.movable:
                self.assertGreaterEqual(result[joint.name], joint.lower_deg)
                self.assertLessEqual(result[joint.name], joint.upper_deg)
            parents = {j.child: j for j in model.joints}
            ancestors = set()
            link = effector
            while link in parents:
                ancestors.add(parents[link].name)
                link = parents[link].parent
            for name in angles.keys() - ancestors:
                self.assertEqual(result[name], angles[name])

    def test_density_updates_mass_torque_and_margin_for_both_models(self):
        with patch("sys.argv", ["viewer"]):
            args = viewer.parse_args()
        states = [viewer.create_urdf_viewer(key, args) for key in urdf.MODEL_PATHS]
        for state in states:
            poses = state.model.forward(state.angles, state.root_transform)
            baseline = viewer.make_urdf_telemetry(state, poses)
            viewer.layout_material_overlay(state.material_overlay, (1920, 1440), None)
            control = state.material_overlay.controls[0]
            event = pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1,
                                       pos=control.rect.center)
            self.assertTrue(viewer.handle_material_overlay_event(event, viewer.ViewerState(), state.material_overlay))
            drag = pygame.event.Event(pygame.MOUSEMOTION, pos=(control.rect.right, control.rect.centery))
            self.assertTrue(viewer.handle_material_overlay_event(drag, viewer.ViewerState(), state.material_overlay))
            release = pygame.event.Event(pygame.MOUSEBUTTONUP, button=1)
            self.assertTrue(viewer.handle_material_overlay_event(release, viewer.ViewerState(), state.material_overlay))
            self.assertEqual(control.value_si, 1.5)
            changed = viewer.make_urdf_telemetry(state, poses)
            self.assertAlmostEqual(changed.model.total_mass_kg, baseline.model.total_mass_kg*1.5)
            np.testing.assert_allclose(changed.model.com_m, baseline.model.com_m)
            self.assertGreater(baseline.largest_torque.required_torque_nm, 0)
            for a, b in zip(baseline.torque_rows, changed.torque_rows):
                self.assertAlmostEqual(b.required_torque_nm, a.required_torque_nm*1.5)
                self.assertIsNone(b.continuous_margin)
                if a.max_margin is not None:
                    self.assertAlmostEqual(b.max_margin, a.max_margin/1.5)
            state.material_overlay.controls[0].value_si = 1.
        self.assertIsNot(states[0].material_overlay, states[1].material_overlay)

    def test_torque_uses_only_descendants_and_transformed_axis(self):
        # 2 kg at radius 3 m -> I=18 kg m2 -> torque=180 Nm at alpha=10.
        empty = np.empty((0, 2, 3))
        links = {"root": urdf.Link("root", 100., np.array([30., 0, 0]), empty),
                 "arm": urdf.Link("arm", 2., np.array([3., 0, 0]), empty)}
        joint = urdf.Joint("turn", "revolute", "root", "arm", np.eye(4), np.array([0., 0., 1.]), 360.)
        model = urdf.UrdfModel("test", "root", links, (joint,))
        state = viewer.UrdfViewerState(model)
        telemetry = viewer.make_urdf_telemetry(state, model.forward({"turn": 810}))
        self.assertAlmostEqual(telemetry.largest_torque.required_torque_nm, 180.)
        self.assertAlmostEqual(telemetry.worst_margin.max_margin, 2.)
        missing = urdf.Joint("turn", "revolute", "root", "arm", np.eye(4), np.array([0., 0., 1.]))
        state.model = urdf.UrdfModel("test", "root", links, (missing,))
        self.assertIsNone(viewer.make_urdf_telemetry(state, state.model.forward({})).worst_margin.max_margin)

    def test_urdf_limits_apply_to_reset_drag_and_unreachable_ik(self):
        with patch("sys.argv", ["viewer"]):
            args = viewer.parse_args()
        for key in urdf.MODEL_PATHS:
            state = viewer.create_urdf_viewer(key, args)
            ui = viewer.ViewerState()
            viewer.handle_urdf_key(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_0, mod=0), ui, state)
            for joint in state.model.movable:
                self.assertGreaterEqual(state.angles[joint.name], joint.lower_deg)
                self.assertLessEqual(state.angles[joint.name], joint.upper_deg)
            viewer.layout_stage3_overlay(state.command_overlay, (1920, 1440), [])
            state.command_overlay.active_key = state.active_joint.name
            viewer.handle_urdf_mouse(pygame.event.Event(pygame.MOUSEMOTION, pos=(-100000, 0)), state, ui)
            self.assertEqual(state.angles[state.active_joint.name], state.active_joint.lower_deg)
            state.autoplay = True
            for _ in range(30):
                viewer.update_urdf_motion(state, .5, False)
                for joint in state.model.movable:
                    self.assertGreaterEqual(state.angles[joint.name], joint.lower_deg)
                    self.assertLessEqual(state.angles[joint.name], joint.upper_deg)
            effector = next(n for n in state.model.links if n.endswith(("_foot", "_paw")))
            result, error = urdf.solve_reach(state.model, state.angles, state.root_transform,
                                             np.array([100., 100., 100.]), effector)
            self.assertGreater(error, 100.)
            for joint in state.model.movable:
                self.assertGreaterEqual(result[joint.name], joint.lower_deg)
                self.assertLessEqual(result[joint.name], joint.upper_deg)

    def test_live_model_switching_and_rendering(self):
        with patch("sys.argv", ["viewer", "--headless", "--frames", "3", "--fps", "0", "--paused"]):
            args = viewer.parse_args()
        batches = [[pygame.event.Event(pygame.KEYDOWN, key=key, mod=0)]
                   for key in (pygame.K_2, pygame.K_3, pygame.K_1)]
        with patch.object(pygame.event, "get", side_effect=batches), \
             patch.object(viewer, "draw_urdf_viewer", wraps=viewer.draw_urdf_viewer) as draw, \
             patch.object(viewer, "make_stage1_telemetry", wraps=viewer.make_stage1_telemetry) as telemetry, \
             patch.object(viewer, "draw_stage3_overlay", wraps=viewer.draw_stage3_overlay) as panels, \
             patch.object(viewer, "draw_status_panel", wraps=viewer.draw_status_panel) as status:
            viewer.run_viewer(args)
        self.assertEqual([c.args[-1] for c in draw.call_args_list], ["go2", "aibo"])
        self.assertGreaterEqual(telemetry.call_count, 1)
        self.assertEqual(panels.call_count, 3)
        self.assertEqual(status.call_count, 3)
        self.assertEqual(len({call.args[4].panel_rect.x for call in panels.call_args_list}), 1)


if __name__ == "__main__":
    unittest.main()
