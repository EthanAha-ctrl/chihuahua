#!/usr/bin/env -S uv run python
"""One-click MuJoCo viewer launcher.

Run `python -m src.components.b_mujoco_sim.view_mujoco` with no arguments. It refreshes the default MuJoCo stand model
and opens MuJoCo's native GUI.
"""

from __future__ import annotations

from pathlib import Path

from src.components.z_common.dog_description import DEFAULT_DESCRIPTION_PATH, load_dog_description
from src.components.z_common import mass_model as basic_motion
from src.components.b_mujoco_sim import mujoco_contact as mujoco
OUT_DIR = Path("artifacts/mujoco/mujoco_viewer")


def build_default_case() -> mujoco.MujocoCase:
    description = load_dog_description(DEFAULT_DESCRIPTION_PATH)
    catalog = basic_motion.load_catalog(
        basic_motion.DEFAULT_MATERIALS_PATH,
        basic_motion.DEFAULT_ACTUATORS_PATH,
        basic_motion.DEFAULT_BATTERIES_PATH,
    )
    return mujoco.build_mujoco_case(
        description,
        catalog,
        out_dir=OUT_DIR,
        primitive="stand",
        frame_count=5,
        duration_s=1.0,
        waist_yaw_deg=0.0,
        waist_pitch_deg=0.0,
        swing_leg="front_left",
        step_length_m=0.040,
        step_height_m=0.025,
        min_torque_margin=1.0,
        min_joint_limit_margin_deg=1.0,
        ik_tolerance_m=1e-4,
        friction_coeff=mujoco.DEFAULT_FOOT_FRICTION,
        run_mujoco=False,
        mujoco_duration_s=0.02,
        viewer_safe=True,
    )


def launch_viewer(mjcf_path: Path) -> None:
    import mujoco.viewer

    mujoco.viewer.launch_from_path(str(mjcf_path))


def main() -> int:
    case = build_default_case()
    print(f"Opening MuJoCo viewer: {case.mjcf_path}")
    launch_viewer(case.mjcf_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
