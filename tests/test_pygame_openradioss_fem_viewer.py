import tempfile
import unittest
from pathlib import Path

import numpy as np

import pygame_openradioss_fem_viewer as viewer


class PygameOpenRadiossFemViewerTest(unittest.TestCase):
    def test_loads_and_interpolates_torque_replay_overlay_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            csv_path = Path(tmp) / "stage1_torque_replay_loads.csv"
            csv_path.write_text(
                "\n".join(
                    [
                        "time_ms,joint,proximal_node,distal_node,axis_x,axis_y,axis_z,torque_nm",
                        "0,waist_yaw,waist_yaw,waist_pitch,1,0,0,1",
                        "10,waist_yaw,waist_yaw,waist_pitch,0,1,0,3",
                    ]
                ),
                encoding="utf-8",
            )

            series = viewer.load_torque_overlay_series(csv_path)
            self.assertEqual(len(series), 1)

            sample = viewer.sample_torque_overlay(series, 5.0)[0]
            self.assertEqual(sample.joint, "waist_yaw")
            self.assertEqual(sample.proximal_node, "waist_yaw")
            self.assertEqual(sample.distal_node, "waist_pitch")
            self.assertAlmostEqual(sample.torque_nm, 2.0)
            np.testing.assert_allclose(sample.axis, np.array([1.0, 1.0, 0.0]) / 2**0.5)

    def test_missing_torque_csv_means_no_overlay_data(self):
        series = viewer.load_torque_overlay_series(Path("/tmp/definitely_missing_stage2_torque_overlay.csv"))
        self.assertEqual(series, ())


if __name__ == "__main__":
    unittest.main()
