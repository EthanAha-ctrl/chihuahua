import csv
import subprocess
import tempfile
import unittest
from pathlib import Path

import numpy as np

import mass_model
import stage3_ik_control as stage3


REPO_ROOT = Path(__file__).resolve().parents[1]
REQUIRED_OUTPUTS = {
    "joint_commands.csv",
    "foot_targets.csv",
    "trajectory_frames.csv",
    "control_safety_summary.csv",
    "stage3_ik_control_summary.yaml",
}


class Stage3IKControlTest(unittest.TestCase):
    def load_inputs(self):
        description = mass_model.load_dog_description(REPO_ROOT / "dog_description.yaml")
        catalog = mass_model.load_catalog(
            REPO_ROOT / "materials.yaml",
            REPO_ROOT / "actuators.yaml",
            REPO_ROOT / "batteries.yaml",
        )
        return description, catalog

    def test_cli_generates_required_outputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            out_dir = Path(tmp)
            result = subprocess.run(
                [
                    "uv",
                    "run",
                    "python",
                    "stage3_ik_control.py",
                    "--out-dir",
                    str(out_dir),
                    "--frame-count",
                    "2",
                ],
                cwd=REPO_ROOT,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            for filename in REQUIRED_OUTPUTS:
                self.assertTrue((out_dir / filename).is_file(), filename)

            with (out_dir / "trajectory_frames.csv").open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 2)
            self.assertTrue(all(row["safe_to_execute"] == "yes" for row in rows))

    def test_stand_ik_outputs_safe_joint_commands(self):
        description, catalog = self.load_inputs()
        case = stage3.build_stage3_case(
            description,
            catalog,
            out_dir=Path("/tmp/stage3_test_unused"),
            primitive="stand",
            frame_count=1,
        )
        frame = case.frames[0]

        self.assertTrue(frame.safety.safe_to_execute, frame.safety.failure_reasons)
        self.assertLess(frame.safety.max_ik_residual_m, 1e-4)
        self.assertGreater(frame.safety.min_continuous_torque_margin, 1.0)
        self.assertGreaterEqual(frame.safety.support_polygon_margin_m, 0.0)

        for solution in frame.ik_solutions.values():
            for joint_type, angle_rad in solution.angles_rad.items():
                spec = description.joint_ranges[joint_type]
                angle_deg = np.degrees(angle_rad)
                self.assertGreaterEqual(angle_deg, spec.min_deg - 1e-8)
                self.assertLessEqual(angle_deg, spec.max_deg + 1e-8)

    def test_unreachable_foot_target_reports_residual(self):
        description, _catalog = self.load_inputs()
        geometry = mass_model.robot_geometry(description)
        pose = mass_model.make_body_pose(geometry, description.viewer.body_z_m, 0.0, 0.0)
        targets = stage3.build_stand_targets(geometry)
        far_target = targets["front_left"] + np.array([1.0, 0.0, 0.0])

        solution = stage3.solve_leg_ik(
            description,
            pose,
            "front_left",
            far_target,
            tolerance_m=1e-5,
            max_iterations=35,
        )

        self.assertFalse(solution.reached)
        self.assertGreater(solution.residual_m, 0.2)

    def test_support_polygon_margin_detects_inside_and_outside_com(self):
        square = [
            np.array([0.0, 0.0]),
            np.array([1.0, 0.0]),
            np.array([1.0, 1.0]),
            np.array([0.0, 1.0]),
        ]

        self.assertGreater(stage3.support_polygon_margin(np.array([0.5, 0.5]), square), 0.0)
        self.assertLess(stage3.support_polygon_margin(np.array([1.5, 0.5]), square), 0.0)

    def test_crawl_step_marks_unsupported_frames_unsafe(self):
        description, catalog = self.load_inputs()
        case = stage3.build_stage3_case(
            description,
            catalog,
            out_dir=Path("/tmp/stage3_test_unused"),
            primitive="crawl_step",
            frame_count=5,
        )

        unsafe = [frame for frame in case.frames if not frame.safety.safe_to_execute]
        self.assertTrue(unsafe)
        reasons = {reason for frame in unsafe for reason in frame.safety.failure_reasons}
        self.assertIn("support_polygon", reasons)


if __name__ == "__main__":
    unittest.main()
