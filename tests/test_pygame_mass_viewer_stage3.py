import math
import unittest
from pathlib import Path

import numpy as np

import mass_model
import pygame_mass_viewer as viewer_model


REPO_ROOT = Path(__file__).resolve().parents[1]


class PygameMassViewerStage3Test(unittest.TestCase):
    def load_inputs(self):
        description = mass_model.load_dog_description(REPO_ROOT / "dog_description.yaml")
        catalog = mass_model.load_catalog(
            REPO_ROOT / "materials.yaml",
            REPO_ROOT / "actuators.yaml",
            REPO_ROOT / "batteries.yaml",
        )
        return description, catalog

    def test_stage3_joint_command_keys_cover_21_dofs(self):
        self.assertEqual(len(viewer_model.JOINT_COMMAND_KEYS), 21)
        self.assertEqual(viewer_model.JOINT_COMMAND_KEYS[:2], ("waist_yaw", "waist_pitch"))
        self.assertEqual(viewer_model.JOINT_COMMAND_KEYS[-3:], ("neck_yaw", "neck_pitch", "head_claw"))
        self.assertIn("front_left_hip_ab", viewer_model.JOINT_COMMAND_KEYS)
        self.assertIn("rear_right_toe_bend", viewer_model.JOINT_COMMAND_KEYS)

    def test_initialize_joint_commands_stays_inside_ranges(self):
        description, _catalog = self.load_inputs()
        state = viewer_model.ViewerState()
        viewer_model.initialize_joint_commands(state, description, waist_deg=12.0, waist_pitch_deg=-3.0)

        for key in viewer_model.JOINT_COMMAND_KEYS:
            spec = description.joint_ranges[viewer_model.command_spec_name(key)]
            value = state.joint_commands_deg[key]
            self.assertGreaterEqual(value, spec.min_deg)
            self.assertLessEqual(value, spec.max_deg)

    def test_stage3_reach_selects_best_toe_effector(self):
        description, catalog = self.load_inputs()
        state = viewer_model.ViewerState(joint_babble=False, waist_babble=False, motion_paused=True)
        viewer_model.initialize_joint_commands(state, description, waist_deg=0.0, waist_pitch_deg=0.0)
        telemetry = viewer_model.make_stage1_telemetry(description, state, catalog)
        state.stage3_target_m = telemetry.model.legs["front_left"].toe_endpoint + np.array([0.01, 0.0, 0.02])
        untouched = {
            key: value
            for key, value in state.joint_commands_deg.items()
            if not key.startswith("front_left_") and key not in viewer_model.BODY_COMMAND_KEYS
        }

        changed = viewer_model.update_stage3_reach_target(
            state,
            description,
            telemetry,
            np.random.default_rng(3),
            dt=0.1,
        )

        self.assertTrue(changed)
        self.assertIsNotNone(state.stage3_target_m)
        self.assertEqual(state.stage3_active_effector, "front_left_toe")
        self.assertLess(state.stage3_target_error_m, 1e-4)
        self.assertNotEqual(state.joint_commands_deg["waist_yaw"], 0.0)
        self.assertNotEqual(state.joint_commands_deg["waist_pitch"], 0.0)
        for key, value in untouched.items():
            self.assertEqual(state.joint_commands_deg[key], value, key)
        for joint in viewer_model.LEG_COMMAND_JOINTS:
            key = f"front_left_{joint}"
            spec = description.joint_ranges[joint]
            value = state.joint_commands_deg[key]
            self.assertGreaterEqual(value, spec.min_deg)
            self.assertLessEqual(value, spec.max_deg)

    def test_stage3_reach_selects_head_claw_when_best(self):
        description, catalog = self.load_inputs()
        state = viewer_model.ViewerState(joint_babble=False, waist_babble=False, motion_paused=True)
        viewer_model.initialize_joint_commands(state, description, waist_deg=0.0, waist_pitch_deg=0.0)
        telemetry = viewer_model.make_stage1_telemetry(description, state, catalog)
        head_midpoint = 0.5 * (telemetry.model.head.upper_tip + telemetry.model.head.lower_tip)
        state.stage3_target_m = head_midpoint + np.array([0.0, 0.0, 0.1])

        changed = viewer_model.update_stage3_reach_target(
            state,
            description,
            telemetry,
            np.random.default_rng(3),
            dt=0.1,
        )

        self.assertTrue(changed)
        self.assertEqual(state.stage3_active_effector, "head_claw")
        for key in (*viewer_model.BODY_COMMAND_KEYS, *viewer_model.HEAD_COMMAND_KEYS):
            spec = description.joint_ranges[key]
            value = state.joint_commands_deg[key]
            self.assertGreaterEqual(value, spec.min_deg)
            self.assertLessEqual(value, spec.max_deg)

    def test_make_live_stage1_model_uses_manual_joint_commands(self):
        description, catalog = self.load_inputs()
        state = viewer_model.ViewerState(joint_babble=False, waist_babble=False)
        viewer_model.initialize_joint_commands(state, description, waist_deg=0.0, waist_pitch_deg=0.0)
        viewer_model.set_joint_command_deg(state, description, "front_left_knee_bend", 88.0)

        model = viewer_model.make_live_stage1_model(description, state, catalog)
        pose = model.pose
        chain = model.legs["front_left"]
        upper = chain.knee - chain.hip
        lower = chain.toe_joint - chain.knee
        _forward, outward, _down = pose.bases["front_left"]
        axis = -outward / np.linalg.norm(outward)

        upper_perp = upper - axis * float(np.dot(upper, axis))
        lower_perp = lower - axis * float(np.dot(lower, axis))
        upper_perp = upper_perp / np.linalg.norm(upper_perp)
        lower_perp = lower_perp / np.linalg.norm(lower_perp)
        bend = math.degrees(math.atan2(float(np.dot(axis, np.cross(upper_perp, lower_perp))), float(np.dot(upper_perp, lower_perp))))

        self.assertAlmostEqual(bend, 88.0, delta=1e-6)


if __name__ == "__main__":
    unittest.main()
