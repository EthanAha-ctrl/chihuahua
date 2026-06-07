import csv
import subprocess
import tempfile
import unittest
from pathlib import Path

import mass_model


REPO_ROOT = Path(__file__).resolve().parents[1]
REQUIRED_OUTPUTS = {
    "mass_elements.csv",
    "joint_torque_estimate.csv",
    "case_summary.csv",
    "leg_endpoint_summary.csv",
    "mass_summary.yaml",
}
CASE_SUMMARY_FIELDS = {
    "case_name",
    "total_mass_kg",
    "com_x_m",
    "com_y_m",
    "com_z_m",
    "max_ik_stretch_m",
    "max_target_residual_m",
    "max_required_torque_nm",
}
LEG_ENDPOINT_FIELDS = {
    "requested_toe_x_m",
    "requested_toe_y_m",
    "requested_toe_z_m",
    "solved_toe_x_m",
    "solved_toe_y_m",
    "solved_toe_z_m",
}
VALID_MOUNT_FRAMES = {"rear", "waist", "front"}


class Stage1AcceptanceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmpdir = tempfile.TemporaryDirectory()
        cls.out_dir = Path(cls.tmpdir.name)
        result = subprocess.run(
            ["uv", "run", "python", "mass_model.py", "--out-dir", str(cls.out_dir)],
            cwd=REPO_ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        cls.cli_result = result
        if result.returncode != 0:
            raise AssertionError(
                "mass_model.py CLI failed\n"
                f"stdout:\n{result.stdout}\n"
                f"stderr:\n{result.stderr}"
            )

    @classmethod
    def tearDownClass(cls):
        cls.tmpdir.cleanup()

    def read_csv(self, filename):
        with (self.out_dir / filename).open(newline="", encoding="utf-8") as handle:
            return list(csv.DictReader(handle))

    def test_cli_generates_required_outputs(self):
        self.assertEqual(self.cli_result.returncode, 0)
        for filename in REQUIRED_OUTPUTS:
            self.assertTrue((self.out_dir / filename).is_file(), filename)

    def test_case_summary_acceptance(self):
        rows = self.read_csv("case_summary.csv")
        self.assertTrue(rows)
        self.assertTrue(CASE_SUMMARY_FIELDS.issubset(rows[0].keys()))

        by_case = {row["case_name"]: row for row in rows}
        neutral = by_case["neutral_pose"]
        self.assertGreater(float(neutral["total_mass_kg"]), 0.0)
        self.assertLess(float(neutral["max_target_residual_m"]), 1e-6)
        self.assertGreater(float(neutral["max_required_torque_nm"]), 0.0)

        yaw_residuals = [
            float(by_case[name]["max_target_residual_m"])
            for name in ("yaw_left_pose", "yaw_right_pose")
            if name in by_case
        ]
        self.assertTrue(yaw_residuals)
        self.assertTrue(any(value > 0.01 for value in yaw_residuals))

    def test_leg_endpoint_summary_acceptance(self):
        rows = self.read_csv("leg_endpoint_summary.csv")
        self.assertGreaterEqual(len(rows), 20)
        self.assertTrue(LEG_ENDPOINT_FIELDS.issubset(rows[0].keys()))

        yaw_rows = [
            row
            for row in rows
            if row["case_name"] in {"yaw_left_pose", "yaw_right_pose"}
        ]
        self.assertTrue(yaw_rows)
        self.assertTrue(
            any(float(row["target_residual_m"]) > 0.01 for row in yaw_rows)
        )

    def test_joint_torque_estimate_acceptance(self):
        rows = self.read_csv("joint_torque_estimate.csv")
        joints = {row["joint"] for row in rows}
        self.assertIn("waist_yaw", joints)
        self.assertIn("waist_pitch", joints)
        self.assertIn("neck_yaw", joints)
        self.assertIn("neck_pitch", joints)
        self.assertIn("head_claw", joints)
        self.assertTrue(any("_hip_" in joint for joint in joints))
        self.assertTrue(any(joint.endswith("_knee_bend") for joint in joints))
        self.assertTrue(any(joint.endswith("_toe_bend") for joint in joints))

        for row in rows:
            self.assertGreaterEqual(float(row["required_torque_nm"]), 0.0)

    def test_head_neck_anchor_and_torque_acceptance(self):
        description = mass_model.load_dog_description(REPO_ROOT / "dog_description.yaml")
        catalog = mass_model.load_catalog(
            REPO_ROOT / "materials.yaml",
            REPO_ROOT / "actuators.yaml",
            REPO_ROOT / "batteries.yaml",
        )
        assumptions = mass_model.Stage1Assumptions()
        model = mass_model.build_mass_model(
            "neutral_pose",
            description,
            catalog,
            assumptions,
            0.0,
            0.0,
        )

        for actual, expected in zip(model.head.neck_origin, model.head.body_anchor):
            self.assertAlmostEqual(actual, expected, places=12)

        element_names = {element.name for element in model.elements}
        self.assertNotIn("neck_mount_stub", element_names)

        torque_joints = {row.joint for row in mass_model.estimate_torques(model, catalog, assumptions)}
        self.assertIn("neck_yaw", torque_joints)
        self.assertIn("neck_pitch", torque_joints)
        self.assertIn("head_claw", torque_joints)

    def test_payload_mount_frames_are_explicit_and_valid(self):
        data = mass_model.yaml_mapping(REPO_ROOT / "batteries.yaml", "batteries")

        for group_name in ("batteries", "electronics"):
            group = mass_model.required_mapping(data, group_name, "batteries")
            for name, raw_item in group.items():
                item = mass_model.as_mapping(raw_item, f"{group_name}.{name}")
                self.assertIn("mount_frame", item, f"{group_name}.{name}")
                self.assertIn(item["mount_frame"], VALID_MOUNT_FRAMES)

        catalog = mass_model.load_catalog(
            REPO_ROOT / "materials.yaml",
            REPO_ROOT / "actuators.yaml",
            REPO_ROOT / "batteries.yaml",
        )
        self.assertIn(catalog.battery.mount_frame, VALID_MOUNT_FRAMES)
        for item in catalog.electronics:
            self.assertIn(item.mount_frame, VALID_MOUNT_FRAMES)

        with self.assertRaises(ValueError):
            mass_model.mount_frame_value({"mount_frame": "world"}, "bad_payload")


if __name__ == "__main__":
    unittest.main()
