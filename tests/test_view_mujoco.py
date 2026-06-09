import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml

import view_mujoco


class ViewMujocoLauncherTest(unittest.TestCase):
    FOOT_CONTACT_NAMES = {
        "front_left_foot_contact",
        "front_right_foot_contact",
        "rear_left_foot_contact",
        "rear_right_foot_contact",
    }

    def test_default_launcher_case_writes_loadable_mjcf(self):
        case = view_mujoco.build_default_case()
        self.assertTrue(case.mjcf_path.is_file())

        model = mujoco.MjModel.from_xml_path(str(case.mjcf_path))
        self.assertEqual(model.nu, 21)
        self.assertEqual(model.njnt, 23)
        self.assertEqual(model.nq, 29)
        self.assertEqual(model.neq, 1)
        self.assertAlmostEqual(float(model.opt.gravity[2]), -9.80665, places=5)
        self.assertGreaterEqual(model.ngeom, 4)

        root = ET.parse(case.mjcf_path).getroot()
        geom_names = {geom.attrib.get("name", "") for geom in root.findall(".//geom")}
        self.assertFalse(any(name.endswith("_marker") for name in geom_names))
        self.assertFalse(any(name.startswith("payload_") for name in geom_names))
        foot_names = {name for name in geom_names if name.endswith("_foot_contact")}
        self.assertEqual(foot_names, self.FOOT_CONTACT_NAMES)
        self.assertFalse(any("foot_anchor" in name for name in geom_names))

        foot_geoms = [geom for geom in root.findall(".//geom") if geom.attrib.get("name", "").endswith("_foot_contact")]
        self.assertTrue(all(geom.attrib.get("contype") == "1" for geom in foot_geoms))
        self.assertTrue(all(geom.attrib.get("conaffinity") == "1" for geom in foot_geoms))
        for foot_name in self.FOOT_CONTACT_NAMES:
            leg_name = foot_name.removesuffix("_foot_contact")
            self.assertIsNotNone(root.find(f".//body[@name='{leg_name}_toe_body']/geom[@name='{foot_name}']"))
        structure_geoms = [geom for geom in root.findall(".//geom") if geom.attrib.get("material") == "structure"]
        self.assertGreater(len(structure_geoms), 0)
        self.assertTrue(all(geom.attrib.get("contype") == "1" for geom in structure_geoms))
        self.assertTrue(all(geom.attrib.get("conaffinity") == "1" for geom in structure_geoms))

        equality_names = {item.attrib.get("name", "") for item in root.findall("./equality/*")}
        self.assertEqual(equality_names, {"head_claw_lower_follows_head_claw"})

        self.assertEqual(root.findall("./actuator/position"), [])
        motors = root.findall("./actuator/motor")
        motor_joints = {motor.attrib["joint"] for motor in motors}
        self.assertEqual(len(motors), 21)
        self.assertIn("waist_yaw", motor_joints)
        self.assertIn("waist_pitch", motor_joints)
        self.assertIn("neck_yaw", motor_joints)
        self.assertIn("neck_pitch", motor_joints)
        self.assertIn("head_claw", motor_joints)
        self.assertNotIn("head_claw_lower", motor_joints)
        self.assertTrue(all("kp" not in motor.attrib for motor in motors))
        self.assertTrue(all("forcerange" in motor.attrib for motor in motors))

        body_names = {body.attrib.get("name", "") for body in root.findall(".//body")}
        self.assertFalse(any(name.startswith("joint_stub_") for name in body_names))
        self.assertFalse(any(name.endswith("_foot_anchor") for name in body_names))
        for leg in ("front_left", "front_right", "rear_left", "rear_right"):
            self.assertNotIn(f"{leg}_hip_body", body_names)
            self.assertIn(f"{leg}_hip_ab_body", body_names)
            self.assertIn(f"{leg}_hip_pitch_body", body_names)
            for suffix in ("hip_ab", "hip_pitch", "knee", "toe"):
                body = root.find(f".//body[@name='{leg}_{suffix}_body']")
                self.assertIsNotNone(body)
                self.assertEqual(len(body.findall("joint")), 1)

    def test_default_launcher_case_uses_ground_contact_not_foot_pins(self):
        case = view_mujoco.build_default_case()
        model = mujoco.MjModel.from_xml_path(str(case.mjcf_path))
        data = mujoco.MjData(model)

        for _ in range(150):
            mujoco.mj_step(model, data)

        contact_geom_names = set()
        for index in range(data.ncon):
            contact = data.contact[index]
            contact_geom_names.add(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1))
            contact_geom_names.add(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2))

        self.assertIn("ground", contact_geom_names)
        self.assertTrue(self.FOOT_CONTACT_NAMES.issubset(contact_geom_names))
        self.assertFalse(any(name and "foot_anchor" in name for name in contact_geom_names))

    def test_default_launcher_case_reports_data_topology_tree(self):
        case = view_mujoco.build_default_case()
        with case.summary_path.open(encoding="utf-8") as handle:
            summary = yaml.safe_load(handle)

        self.assertTrue(summary["analysis_state"]["viewer_safe_uses_data_topology_tree"])
        self.assertEqual(summary["counts"]["viewer_safe_topology_bodies"], 23)

    def test_default_launcher_case_structure_geoms_collide_with_ground(self):
        case = view_mujoco.build_default_case()
        model = mujoco.MjModel.from_xml_path(str(case.mjcf_path))
        root = ET.parse(case.mjcf_path).getroot()
        structure_names = {
            geom.attrib["name"]
            for geom in root.findall(".//geom")
            if geom.attrib.get("material") == "structure"
        }
        data = mujoco.MjData(model)
        data.qpos[2] = -0.27
        mujoco.mj_forward(model, data)

        contact_geom_names = set()
        for index in range(data.ncon):
            contact = data.contact[index]
            contact_geom_names.add(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1))
            contact_geom_names.add(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2))

        self.assertIn("ground", contact_geom_names)
        self.assertTrue(structure_names & contact_geom_names)

    def test_default_launcher_case_moves_body_under_leg_control(self):
        case = view_mujoco.build_default_case()
        model = mujoco.MjModel.from_xml_path(str(case.mjcf_path))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "front_left_hip_ab_motor")
        before = data.qpos[:3].copy()
        data.ctrl[actuator_id] = 0.20
        for _ in range(250):
            mujoco.mj_step(model, data)
        after = data.qpos[:3].copy()

        self.assertGreater(float(((after - before) ** 2).sum() ** 0.5), 0.001)

    def test_default_launcher_case_distributes_stage1_mass_to_live_bodies(self):
        case = view_mujoco.build_default_case()
        expected_mass = case.stage3_case.frames[0].model.total_mass_kg
        expected_com = case.stage3_case.frames[0].model.com_m
        model = mujoco.MjModel.from_xml_path(str(case.mjcf_path))

        body_names = [
            "robot_free_root",
            "waist_yaw_body",
            "waist_pitch_body",
            "front_left_hip_ab_body",
            "front_left_hip_pitch_body",
            "front_left_knee_body",
            "front_left_toe_body",
            "rear_right_hip_ab_body",
            "rear_right_hip_pitch_body",
            "neck_pitch_body",
            "head_upper_jaw_body",
            "head_lower_jaw_body",
        ]
        body_masses = {
            name: float(model.body_mass[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)])
            for name in body_names
        }

        self.assertAlmostEqual(float(model.body_mass.sum()), expected_mass, places=8)
        self.assertLess(body_masses["robot_free_root"], expected_mass * 0.25)
        self.assertGreater(body_masses["waist_yaw_body"], 0.30)
        self.assertGreater(body_masses["waist_pitch_body"], 0.30)
        self.assertGreater(body_masses["front_left_hip_ab_body"], 0.10)
        self.assertGreater(body_masses["front_left_hip_pitch_body"], 0.11)
        self.assertGreater(body_masses["front_left_knee_body"], 0.10)
        self.assertGreater(body_masses["front_left_toe_body"], 0.03)
        self.assertGreater(body_masses["rear_right_hip_ab_body"], 0.10)
        self.assertGreater(body_masses["rear_right_hip_pitch_body"], 0.11)
        self.assertGreater(body_masses["neck_pitch_body"], 0.02)
        self.assertGreater(body_masses["head_upper_jaw_body"], 0.003)
        self.assertGreater(body_masses["head_lower_jaw_body"], 0.003)

        root = ET.parse(case.mjcf_path).getroot()
        viewer_inertial_masses = [
            float(inertial.attrib["mass"])
            for body in root.findall(".//body")
            if not body.attrib.get("name", "").startswith("joint_stub_")
            for inertial in body.findall("inertial")
        ]
        self.assertNotIn(1.0e-5, viewer_inertial_masses)

        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        root_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "robot_free_root")
        np.testing.assert_allclose(data.subtree_com[root_body_id], expected_com, atol=1.0e-8)

    def test_default_launcher_case_moves_front_body_under_waist_control(self):
        case = view_mujoco.build_default_case()
        model = mujoco.MjModel.from_xml_path(str(case.mjcf_path))
        front_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "front_body_spine")
        rear_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "rear_body_spine")

        for joint_name in ("waist_yaw", "waist_pitch"):
            with self.subTest(joint_name=joint_name):
                data = mujoco.MjData(model)
                mujoco.mj_forward(model, data)
                joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                qpos_adr = model.jnt_qposadr[joint_id]
                front_before = data.geom_xpos[front_geom_id].copy()
                rear_before = data.geom_xpos[rear_geom_id].copy()

                data.qpos[qpos_adr] = 0.20
                mujoco.mj_forward(model, data)
                front_after = data.geom_xpos[front_geom_id].copy()
                rear_after = data.geom_xpos[rear_geom_id].copy()

                front_delta = float(((front_after - front_before) ** 2).sum() ** 0.5)
                rear_delta = float(((rear_after - rear_before) ** 2).sum() ** 0.5)
                self.assertGreater(front_delta, 0.005)
                self.assertLess(rear_delta, 1.0e-9)

    def test_default_launcher_case_moves_head_under_neck_control(self):
        case = view_mujoco.build_default_case()
        model = mujoco.MjModel.from_xml_path(str(case.mjcf_path))
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "head_upper_jaw_live")

        for joint_name in ("neck_yaw", "neck_pitch", "head_claw"):
            with self.subTest(joint_name=joint_name):
                data = mujoco.MjData(model)
                mujoco.mj_forward(model, data)
                joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                qpos_adr = model.jnt_qposadr[joint_id]
                before = data.geom_xpos[geom_id].copy()

                data.qpos[qpos_adr] = 0.20
                mujoco.mj_forward(model, data)
                after = data.geom_xpos[geom_id].copy()

                self.assertGreater(float(((after - before) ** 2).sum() ** 0.5), 0.005)

    def test_default_launcher_case_opens_both_head_claws_under_one_control(self):
        case = view_mujoco.build_default_case()
        model = mujoco.MjModel.from_xml_path(str(case.mjcf_path))
        model.opt.gravity[:] = 0.0
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "head_claw_motor")
        upper_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "head_upper_jaw_live")
        lower_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "head_lower_jaw_live")
        upper_before = data.geom_xpos[upper_geom_id].copy()
        lower_before = data.geom_xpos[lower_geom_id].copy()

        data.ctrl[actuator_id] = 0.20
        for _ in range(500):
            mujoco.mj_step(model, data)
        upper_delta = data.geom_xpos[upper_geom_id] - upper_before
        lower_delta = data.geom_xpos[lower_geom_id] - lower_before

        self.assertGreater(float(np.linalg.norm(upper_delta)), 0.002)
        self.assertGreater(float(np.linalg.norm(lower_delta)), 0.002)
        self.assertLess(float(np.dot(upper_delta, lower_delta)), 0.0)


if __name__ == "__main__":
    unittest.main()
