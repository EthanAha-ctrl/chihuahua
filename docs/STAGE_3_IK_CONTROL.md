# Stage 3: IK And Control First Light

目标：把 Stage 1/2 形成的几何、质量和力矩模型接到控制侧，先建立一个
可测试的 IK / trajectory / safety scaffold。

Stage 3 不做 MuJoCo，不做真实接触动力学，不求 ground reaction，不假设脚端
真的能稳定支撑整机。MuJoCo/contact 是 Stage 4。

## Scope

Stage 3 当前覆盖：

- 从脚端目标求 leg joint commands。
- 生成简单 trajectory primitives。
- 检查 joint limit margin。
- 复用 Stage 1 mass / torque model 检查 continuous torque margin。
- 用 COM 投影和支撑脚端多边形做几何级 support polygon awareness。
- 输出 simulation-ready command tables，供后续 MuJoCo/export 使用。

Stage 3 当前不覆盖：

- gravity/contact dynamics
- support reaction solve
- foot friction
- body translation controller
- closed-loop balance
- MuJoCo XML export
- real actuator current / thermal loop

## Entry Point

```bash
uv run python stage3_ik_control.py \
  --out-dir stage3_outputs/ik_control \
  --primitive stand \
  --frame-count 5
```

Expected outputs:

```text
joint_commands.csv
foot_targets.csv
trajectory_frames.csv
control_safety_summary.csv
stage3_ik_control_summary.yaml
```

## Pygame Visualization

`pygame_mass_viewer.py` now includes a Stage 3 command overlay:

```bash
uv run python pygame_mass_viewer.py
```

The overlay shows 21 joint command bars:

```text
waist yaw / pitch
4 legs * hip_ab / hip_pitch / knee_bend / toe_bend
neck yaw / pitch
head claw
```

When auto reach is enabled, a random target is sampled around the current COM
inside the configured radius range. The viewer evaluates five candidate
effectors:

```text
front_left toe
front_right toe
rear_left toe
rear_right toe
head claw
```

The candidate with the lowest predicted reach residual wins. Only the winning
effector's command chain is overwritten; the other four effectors keep their
current periodic/manual motion. For a front-toe winner, waist yaw/pitch and that
front leg's four joint bars are updated. For a rear-toe winner, only that rear
leg's four joint bars are updated. For a head-claw winner, waist yaw/pitch plus
neck yaw/pitch and head-claw bars are updated. This is still free-space IK
visualization: no gravity, no contact, no support reaction.

Useful options:

```bash
uv run python pygame_mass_viewer.py \
  --stage3-target-radius-min-m 0.18 \
  --stage3-target-radius-max-m 0.34 \
  --stage3-target-period-s 1.4 \
  --stage3-arbitration-period-s 0.30
```

The periodic baseline is recomputed every frame. Best-effector arbitration runs
at the configured arbitration period, then only the winning command chain is
overwritten until the next arbitration tick.

## Current Primitives

`stand` keeps all four feet on their neutral targets. It should be safe under
the current rough Stage 1 free-space torque model.

`crawl_step` lifts one swing leg through a simple foot arc while the body frame
stays fixed. It is intentionally a diagnostic primitive: without body shift, some
single-leg-swing support triangles can put the COM outside the support polygon.
Those frames should be marked unsafe rather than accepted silently.

## Safety Policy

Each frame records:

```text
max IK residual
minimum continuous torque margin
worst torque-margin joint
minimum joint-limit margin
support polygon margin
safe_to_execute
failure reasons
```

A frame is safe only if:

- IK residual is below the configured tolerance.
- Stage 1 continuous torque margin is above the configured threshold.
- All command angles retain the configured joint-limit margin.
- COM projection is inside the support polygon made by support feet.

This is still a control-side prefilter. A Stage 3 safe frame is not a proof that
the robot can stand or walk in the world; it only means the command survived the
current IK / torque / geometric support checks.

## Relationship To Stage 1 And Stage 2

Stage 3 reuses:

- `dog_description.yaml` as the geometry and joint-range source.
- `mass_model.py` for Stage 1 mass, COM and free-space torque rows.
- The same leg and head chain conventions used by `pygame_mass_viewer.py`.

Stage 3 should feed later iterations:

```text
IK/control unsafe pose
 -> update foot targets / gait primitive / joint limits
 -> update Stage 1 torque estimate
 -> update Stage 2 torque-replay load case if needed
 -> later validate in Stage 4 MuJoCo/contact
```

## Acceptance

The Stage 3 first-light acceptance command is:

```bash
uv run python -m unittest tests/test_stage3_ik_control.py
```

The full repo test suite should also pass:

```bash
uv run python -m unittest discover -s tests
```
