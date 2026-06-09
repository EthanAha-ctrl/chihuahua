import csv
import subprocess
import tempfile
import unittest
from pathlib import Path

import numpy as np

import mass_model
import stage2_rod_model


REPO_ROOT = Path(__file__).resolve().parents[1]
REQUIRED_OUTPUTS = {
    "nodes.csv",
    "members.csv",
    "lumped_masses.csv",
    "rod_model_summary.yaml",
    "stage2_whole_body_rods_poster.png",
}


class Stage2RodModelTest(unittest.TestCase):
    def make_model(self):
        description = mass_model.load_dog_description(REPO_ROOT / "dog_description.yaml")
        catalog = mass_model.load_catalog(
            REPO_ROOT / "materials.yaml",
            REPO_ROOT / "actuators.yaml",
            REPO_ROOT / "batteries.yaml",
        )
        source = stage2_rod_model.build_stage1_model(description, catalog, "test_rods", 0.0, 0.0)
        return source, stage2_rod_model.build_whole_body_rod_model(source)

    def test_rod_graph_is_whole_body_and_connected(self):
        _source, rods = self.make_model()
        names = {member.name for member in rods.members}

        self.assertIn("waist_yaw_pitch", names)
        self.assertIn("front_body_spine", names)
        self.assertIn("rear_body_spine", names)
        self.assertIn("front_left_upper", names)
        self.assertIn("rear_right_toe", names)
        self.assertIn("head_neck", names)
        self.assertIn("head_upper_jaw", names)

        connected = stage2_rod_model.connected_node_names(rods)
        self.assertEqual(len(connected), len(rods.nodes))
        self.assertEqual(len([node for node in rods.nodes if node.contact_candidate]), 4)

    def test_mass_and_com_match_stage1_source(self):
        source, rods = self.make_model()

        self.assertAlmostEqual(rods.total_mass_kg, source.total_mass_kg, places=12)
        np.testing.assert_allclose(rods.com_m, source.com_m, atol=1e-12)
        self.assertGreater(rods.total_member_mass_kg, 0.0)
        self.assertGreater(rods.total_lumped_mass_kg, 0.0)

    def test_rod_graph_is_topology_only(self):
        _source, rods = self.make_model()
        state = stage2_rod_model.summary_dict(rods)["analysis_state"]

        self.assertTrue(state["topology_only"])
        self.assertFalse(state["gravity_applied"])
        self.assertFalse(state["fixed_boundary_conditions_applied"])
        self.assertFalse(state["support_reactions_applied"])
        self.assertFalse(state["load_cases_applied"])
        self.assertFalse(state["solved_deformation"])

    def test_cli_generates_required_outputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            out_dir = Path(tmp)
            result = subprocess.run(
                [
                    "uv",
                    "run",
                    "python",
                    "stage2_rod_model.py",
                    "--out-dir",
                    str(out_dir),
                    "--frames",
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

            with (out_dir / "members.csv").open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertTrue(rows)
            self.assertIn("length_m", rows[0])
            self.assertTrue(all(float(row["length_m"]) > 0.0 for row in rows))


if __name__ == "__main__":
    unittest.main()
