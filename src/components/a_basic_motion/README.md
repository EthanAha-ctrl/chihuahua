减速机构采用 off-the-shelf。选厂家已经匹配并装配好的电机、减速器与轴承总成.
1.5 kg 整机、每腿四个主动自由度和躯干集中驱动的目标.


By the tasks defined in `e_task_reward_rl`, 可能的options 有：

```text
semi-flexible thoracolumbar spine
strong lumbosacral region
mobile scapula / shoulder girdle
pelvis-spine load transfer
limited torsion + useful flexion/extension
high stiffness when biting / pulling
high mobility when running / turning / denning
```


Run from the repository root. Source code: `src/components/z_common/tendon/`.
Manufacturer drawings: `vendor/components/motor/` (including `sources.json`).
Generated STEP, geometry, screenshots and review bundles: `artifacts/ri_tendon_module/`.

```bash
uv run --with cadquery --with trimesh --with fast-simplification python -m src.components.z_common.tendon.build_cad
uv run python -m src.components.z_common.tendon.publish_view --renderer /absolute/path/to/render.py
```

The publisher requires an explicit renderer and all listed validation artifacts;
missing inputs are errors. CAD rebuild was verified with the command above;
the obsolete local CAD environment has been removed.

# Basic motion Acceptance

Basic motion is accepted as a rough free-motion mass and torque body. It is a
structured checkpoint for early design iteration, not a controller, contact
model, FEM model, MuJoCo model, or printable CAD validation.

Basic motion currently covers:

- Read `vendor/components/material/lists.md`, `vendor/components/joint/lists.md`, and `vendor/components/battery/lists.md`.
- Build a rough rigid body mass model from the current endpoint geometry.
- Output total mass and center of mass.
- Output free-space inertial torque estimates for waist, legs, neck, and
  head-claw joints.
- Output fixed-world foot target residuals, including requested and solved toe
  endpoints per leg.
- Use the same Basic motion model for the mass viewer pose and telemetry.
- Place battery and electronics by explicit `mount_frame` values:
  `rear`, `waist`, or `front`.
- Keep the head/neck anchor on the upper body: Basic motion has no head/neck root
  offset, and `neck_origin == body_anchor` is intentional.

## Required Outputs

Running:

```bash
uv run python -m src.components.z_common.mass_model --out-dir /tmp/chihuahua_basic_motion_acceptance
```

must produce:

- `mass_elements.csv`
- `joint_torque_estimate.csv`
- `case_summary.csv`
- `leg_endpoint_summary.csv`
- `mass_summary.yaml`

`case_summary.csv` must expose mass, COM, max IK stretch, max fixed-world target
residual, and max required torque per representative pose.

`leg_endpoint_summary.csv` must expose each leg's requested toe endpoint, solved
toe endpoint, residual, hip, knee, and toe-joint coordinates.

## Explicitly Out Of Scope

Basic motion does not include:

- Gravity torque.
- Foot contact reaction.
- Static load.
- Ground collision.
- FEM.
- IK/control.
- MuJoCo.
- Printable CAD validation.

## Acceptance Commands

Both commands must pass:

```bash
uv run python -m unittest ci/test_basic_motion_acceptance.py
uv run python -m src.components.z_common.mass_model --out-dir /tmp/chihuahua_basic_motion_acceptance
```

