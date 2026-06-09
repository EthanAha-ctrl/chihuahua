import re
import tempfile
import unittest
from pathlib import Path

import mass_model
import stage2_openradioss_periodic_motion as periodic_motion


REPO_ROOT = Path(__file__).resolve().parents[1]


class Stage2OpenRadiossPeriodicMotionTest(unittest.TestCase):
    def build_case(self, out_dir: Path, control_policy: str = "stage1-torque-replay"):
        description = mass_model.load_dog_description(REPO_ROOT / "dog_description.yaml")
        catalog = mass_model.load_catalog(
            REPO_ROOT / "materials.yaml",
            REPO_ROOT / "actuators.yaml",
            REPO_ROOT / "batteries.yaml",
        )
        case = periodic_motion.build_periodic_motion_case(
            description,
            catalog,
            out_dir=out_dir,
            run_name="periodic_motion_test",
            sample_count=5,
            solver_duration_ms=8.0,
            viewer_start_seconds=0.0,
            viewer_motion_seconds=0.005,
            babble_scale=0.5,
            motion_scale=1.0,
            target_element_length_mm=8.0,
            use_nominal_radius_for_massless_members=True,
            uniform_radius_mm=8.0,
            case_name="periodic_motion_test",
            control_policy=control_policy,
        )
        periodic_motion.write_case(case, make_preview_gif=False, preview_frames=2, preview_duration_ms=40)
        return case

    def test_generates_whole_body_periodic_motion_deck(self):
        with tempfile.TemporaryDirectory() as tmp:
            case = self.build_case(Path(tmp))
            starter = case.deck.starter_path.read_text(encoding="utf-8")
            engine = case.deck.engine_path.read_text(encoding="utf-8")

            self.assertIn("/BEAM/1", starter)
            self.assertIn("/ADMAS/5/1", starter)
            self.assertIn("/CLOAD/", starter)
            self.assertIn("waist_yaw_distal_waist_pitch_ZZ_moment", starter)
            self.assertNotIn("/PROP/TYPE8/900001", starter)
            self.assertNotIn("/SPRING/900001", starter)
            self.assertNotIn("/IMPDISP/", starter)
            self.assertIn("/TH/NODE/2", starter)
            self.assertIn("/TH/BEAM/4", starter)
            self.assertIn("F1        M2        M3", starter)
            self.assertIn("rod_beam_section_resultants", starter)
            self.assertIn("rod_motion_displacement", starter)
            self.assertIn("/ANIM/VECT/DISP", engine)
            self.assertEqual(
                len(re.findall(r"^/CLOAD/\d+", starter, flags=re.MULTILINE)),
                periodic_motion.torque_replay_component_count(case),
            )
            self.assertGreater(len(case.deck.members), len(case.deck.rod_model.members))
            self.assertGreater(len(case.deck.node_ids), len(case.deck.rod_model.nodes))
            self.assertEqual(len(case.control_node_names), len(case.deck.rod_model.nodes))
            self.assertGreater(len(case.deck.members), 250)
            areas = {round(member.area_mm2, 9) for member in case.deck.members}
            self.assertEqual(len(areas), 1)

    def test_motion_case_has_no_gravity_contact_or_fixed_feet(self):
        with tempfile.TemporaryDirectory() as tmp:
            case = self.build_case(Path(tmp))
            starter = case.deck.starter_path.read_text(encoding="utf-8")
            summary = periodic_motion.motion_summary(case)

            for forbidden in ("/BCS", "/GRAV", "/INTER", "/RWALL"):
                self.assertNotIn(forbidden, starter)

            state = summary["analysis_state"]
            self.assertFalse(state["kinematic_periodic_motion_applied"])
            self.assertTrue(state["stage1_torque_replay_applied"])
            self.assertFalse(state["gravity_applied"])
            self.assertFalse(state["fixed_feet_applied"])
            self.assertFalse(state["contact_applied"])
            self.assertEqual(summary["counts"]["rod_graph_nodes"], 25)
            self.assertEqual(summary["counts"]["uniform_guided_joint_nodes"], 0)
            self.assertEqual(summary["counts"]["hard_prescribed_robot_joint_nodes"], 0)
            self.assertGreater(summary["counts"]["stage1_torque_replay_joints"], 0)
            self.assertGreater(summary["counts"]["stage1_torque_replay_moment_couples"], 0)
            self.assertGreater(summary["counts"]["concentrated_moment_functions"], 0)
            self.assertEqual(summary["counts"]["beam_resultant_history_elements"], len(case.deck.members))
            self.assertEqual(summary["control_policy"], "stage1-torque-replay")
            self.assertEqual(summary["uniform_radius_mm"], 8.0)
            self.assertEqual(summary["beam_section_radius_policy"], "uniform circular radius 8 mm")
            self.assertTrue(case.torque_replay_csv_path.is_file())
            self.assertNotIn("motion_boundary_handle_nodes", summary["counts"])
            self.assertNotIn("passive_fem_rod_graph_nodes", summary["counts"])

    def test_motion_targets_are_nonzero_and_written(self):
        with tempfile.TemporaryDirectory() as tmp:
            case = self.build_case(Path(tmp))
            self.assertTrue(case.target_csv_path.is_file())
            max_disp = max(
                float((values**2).sum(axis=1).max() ** 0.5)
                for values in case.target_displacements_mm.values()
            )
            self.assertGreater(max_disp, 1.0)

    def test_uniform_joint_guide_policy_still_writes_soft_guides(self):
        with tempfile.TemporaryDirectory() as tmp:
            case = self.build_case(Path(tmp), control_policy="uniform-joint-guides")
            starter = case.deck.starter_path.read_text(encoding="utf-8")
            summary = periodic_motion.motion_summary(case)

            self.assertIn("/PROP/TYPE8/900001", starter)
            self.assertIn("/SPRING/900001", starter)
            self.assertIn("/IMPDISP/", starter)
            self.assertNotIn("/CLOAD/", starter)
            self.assertEqual(summary["counts"]["uniform_guided_joint_nodes"], 25)
            self.assertEqual(summary["counts"]["hard_prescribed_robot_joint_nodes"], 0)
            self.assertEqual(summary["counts"]["imposed_displacement_functions"], 75)

    def test_beam_resultants_convert_to_outer_fiber_stress_and_strain(self):
        with tempfile.TemporaryDirectory() as tmp:
            case = self.build_case(Path(tmp))
            first = case.deck.members[0]
            headers = ["time"]
            values = ["0"]
            for item in case.deck.members:
                for var_idx in range(1, 4):
                    headers.append(f"{periodic_motion.TH_BEAM_GROUP} {item.beam_id} var {var_idx}")
                if item is first:
                    values.extend([f"{item.area_mm2}", "0", "0"])
                else:
                    values.extend(["0", "0", "0"])
            csv_path = Path(tmp) / "beam_resultants.csv"
            csv_path.write_text(",".join(headers) + "\n" + ",".join(values) + "\n", encoding="utf-8")

            stresses_mpa, strains = periodic_motion.load_beam_resultant_strains(csv_path, case)
            first_key = periodic_motion.beam_element_key(first)

            self.assertAlmostEqual(float(stresses_mpa[first_key][0]), 1000.0, places=6)
            self.assertAlmostEqual(float(strains[first_key][0]), 1000.0 / periodic_motion.YOUNGS_MPA, places=6)


if __name__ == "__main__":
    unittest.main()
