import csv
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import mass_model
import stage4_mujoco_contact as stage4


REPO_ROOT = Path(__file__).resolve().parents[1]
REQUIRED_OUTPUTS = {
    "mujoco_model.xml",
    "mujoco_model.mjcf",
    "contact_loadcases.csv",
    "quasi_static_contact_proxy.csv",
    "stage2_feedback_loadcases.csv",
    "stage4_mujoco_contact_summary.yaml",
}


class Stage4MujocoContactTest(unittest.TestCase):
    def load_inputs(self):
        description = mass_model.load_dog_description(REPO_ROOT / "dog_description.yaml")
        catalog = mass_model.load_catalog(
            REPO_ROOT / "materials.yaml",
            REPO_ROOT / "actuators.yaml",
            REPO_ROOT / "batteries.yaml",
        )
        return description, catalog

    def build_case(self, out_dir: Path, primitive: str = "stand", frame_count: int = 3):
        description, catalog = self.load_inputs()
        return stage4.build_stage4_case(
            description,
            catalog,
            out_dir=out_dir,
            primitive=primitive,
            frame_count=frame_count,
            duration_s=1.0,
            waist_yaw_deg=0.0,
            waist_pitch_deg=0.0,
            swing_leg="front_left",
            step_length_m=0.040,
            step_height_m=0.025,
            min_torque_margin=1.0,
            min_joint_limit_margin_deg=1.0,
            ik_tolerance_m=1e-4,
            friction_coeff=stage4.DEFAULT_FOOT_FRICTION,
            run_mujoco=False,
            mujoco_duration_s=0.02,
        )

    def test_cli_generates_required_outputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            out_dir = Path(tmp)
            result = subprocess.run(
                [
                    "uv",
                    "run",
                    "python",
                    "stage4_mujoco_contact.py",
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

    def test_mujoco_xml_exports_contact_geoms_and_joint_motors(self):
        with tempfile.TemporaryDirectory() as tmp:
            case = self.build_case(Path(tmp))
            root = ET.parse(case.xml_path).getroot()
            mjcf_root = ET.parse(case.mjcf_path).getroot()

            self.assertEqual(root.tag, "mujoco")
            self.assertEqual(mjcf_root.tag, "mujoco")
            self.assertFalse((Path(tmp) / "mujoco_model.mjsd").exists())
            self.assertIsNotNone(root.find("./worldbody/geom[@name='ground']"))

            foot_names = {
                item.attrib["name"]
                for item in root.findall(".//geom")
                if item.attrib.get("name", "").endswith("_foot_contact")
            }
            self.assertEqual(len(foot_names), 4)
            self.assertIn("front_left_foot_contact", foot_names)

            self.assertEqual(root.findall("./actuator/position"), [])
            motors = root.findall("./actuator/motor")
            self.assertEqual(len(motors), 21)
            motor_joints = {motor.attrib["joint"] for motor in motors}
            self.assertIn("waist_yaw", motor_joints)
            self.assertIn("rear_right_toe_bend", motor_joints)
            self.assertIn("head_claw", motor_joints)
            self.assertTrue(all("kp" not in motor.attrib for motor in motors))
            self.assertTrue(all("forcerange" in motor.attrib for motor in motors))

    def test_contact_loadcases_are_mujoco_contact_forces(self):
        with tempfile.TemporaryDirectory() as tmp:
            case = self.build_case(Path(tmp), frame_count=1)
            with case.contact_csv_path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))

            self.assertTrue(rows)
            self.assertTrue(all(row["source"] == "mujoco_contact_force" for row in rows))
            self.assertIn("normal_force_n", rows[0])
            self.assertIn("geom1", rows[0])
            self.assertIn("geom2", rows[0])
            self.assertTrue(any("ground" in {row["geom1"], row["geom2"]} for row in rows))
            self.assertTrue(case.contact_csv_result.completed)

    def test_stand_contact_proxy_balances_weight_and_com_moment(self):
        with tempfile.TemporaryDirectory() as tmp:
            case = self.build_case(Path(tmp), frame_count=1)
            frame = case.stage3_case.frames[0]
            rows = [row for row in case.contact_rows if row.frame_index == 0]

            weight = frame.model.total_mass_kg * stage4.GRAVITY_M_S2
            forces = np.array([row.normal_force_n for row in rows], dtype=float)
            xs = np.array([row.foot_position_m[0] for row in rows], dtype=float)
            ys = np.array([row.foot_position_m[1] for row in rows], dtype=float)

            self.assertTrue(all(row.static_solvable for row in rows))
            self.assertAlmostEqual(float(forces.sum()), weight, places=6)
            self.assertAlmostEqual(float(np.dot(forces, xs)), weight * frame.model.com_m[0], places=6)
            self.assertAlmostEqual(float(np.dot(forces, ys)), weight * frame.model.com_m[1], places=6)

    def test_stage2_feedback_rows_match_support_contacts(self):
        with tempfile.TemporaryDirectory() as tmp:
            case = self.build_case(Path(tmp), frame_count=1)
            with case.stage2_feedback_csv_path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))

            support_contacts = [row for row in case.contact_rows if row.normal_force_n > 1e-9]
            self.assertEqual(len(rows), len(support_contacts))
            self.assertTrue(all(row["source"] == "stage4_quasi_static_contact_proxy" for row in rows))
            self.assertTrue(all(row["stage2_node"].endswith("_toe_endpoint") for row in rows))

    def test_front_left_hip_control_moves_visible_foot_geom(self):
        with tempfile.TemporaryDirectory() as tmp:
            case = self.build_case(Path(tmp), frame_count=1)
            import mujoco

            model = mujoco.MjModel.from_xml_path(str(case.mjcf_path))
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)

            geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "front_left_foot_contact")
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "front_left_hip_ab")
            qpos_adr = model.jnt_qposadr[joint_id]
            before = data.geom_xpos[geom_id].copy()

            data.qpos[qpos_adr] = 0.20
            mujoco.mj_forward(model, data)
            after = data.geom_xpos[geom_id].copy()

            self.assertGreater(float(np.linalg.norm(after - before)), 0.005)

    def test_crawl_step_has_unsolved_contact_frames(self):
        with tempfile.TemporaryDirectory() as tmp:
            case = self.build_case(Path(tmp), primitive="crawl_step", frame_count=5)
            by_frame = {}
            for row in case.contact_rows:
                by_frame.setdefault(row.frame_index, []).append(row.static_solvable)
            self.assertTrue(any(not all(values) for values in by_frame.values()))


if __name__ == "__main__":
    unittest.main()
