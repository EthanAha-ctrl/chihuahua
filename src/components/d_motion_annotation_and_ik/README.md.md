# Human motion annotation and IK

使用 Scuf Omega Wireless PlayStation joystick 进行人工控制与动作标注，为固定设计下的 controller 训练提供数据。任务与奖励由 [3K tasks component](../e_task_reward_rl/README.md) 定义。

# IK control: IK And Control First Light

目标：把 Basic motion/2 形成的几何、质量和力矩模型接到控制侧，先建立一个
可测试的 IK / trajectory / safety scaffold。

IK control 不做 MuJoCo，不做真实接触动力学，不求 ground reaction，不假设脚端
真的能稳定支撑整机。MuJoCo/contact 是 MuJoCo。

## Scope

IK control 当前覆盖：

- 从脚端目标求 leg joint commands。
- 生成简单 trajectory primitives。
- 检查 joint limit margin。
- 复用 Basic motion mass / torque model 检查 continuous torque margin。
- 用 COM 投影和支撑脚端多边形做几何级 support polygon awareness。
- 输出 simulation-ready command tables，供后续 MuJoCo/export 使用。

IK control 当前不覆盖：

- gravity/contact dynamics
- support reaction solve
- foot friction
- body translation controller
- closed-loop balance
- MuJoCo XML export
- real actuator current / thermal loop

## Entry Point

```bash
uv run python -m src.components.a_basic_motion.ik_control \
  --out-dir artifacts/ik/ik_control \
  --primitive stand \
  --frame-count 5
```

Expected outputs:

```text
joint_commands.csv
foot_targets.csv
trajectory_frames.csv
control_safety_summary.csv
ik_ik_control_summary.yaml
```

## Pygame Visualization

`pygame_mass_viewer.py` now includes a IK control command overlay:

```bash
uv run python -m src.components.a_basic_motion.pygame_mass_viewer
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
uv run python -m src.components.a_basic_motion.pygame_mass_viewer \
  --ik-target-radius-min-m 0.18 \
  --ik-target-radius-max-m 0.34 \
  --ik-target-period-s 1.4 \
  --ik-arbitration-period-s 0.30
```

The periodic baseline is recomputed every frame. Best-effector arbitration runs
at the configured arbitration period, then only the winning command chain is
overwritten until the next arbitration tick.

## Current Primitives

`stand` keeps all four feet on their neutral targets. It should be safe under
the current rough Basic motion free-space torque model.

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
- Basic motion continuous torque margin is above the configured threshold.
- All command angles retain the configured joint-limit margin.
- COM projection is inside the support polygon made by support feet.

This is still a control-side prefilter. A IK control safe frame is not a proof that
the robot can stand or walk in the world; it only means the command survived the
current IK / torque / geometric support checks.

## Relationship To Basic motion And FEM

IK control reuses:

- `vendor/reference/urdf/chihuahua/lists.md` as the geometry and joint-range source.
- `mass_model.py` for Basic motion mass, COM and free-space torque rows.
- The same leg and head chain conventions used by `pygame_mass_viewer.py`.

IK control should feed later iterations:

```text
IK/control unsafe pose
 -> update foot targets / gait primitive / joint limits
 -> update Basic motion torque estimate
 -> update FEM torque-replay load case if needed
 -> later validate in MuJoCo/contact
```

## Acceptance

The IK control first-light acceptance command is:

```bash
uv run python -m unittest ci/test_ik_control.py
```

The full repo test suite should also pass:

```bash
uv run python -m unittest discover -s ci
```


---

# Stage 5: Complex Terrain IK

目标：在 Basic motion-4 已经完成 first-light 后，定义复杂地形上的 IK 和运动决策层。

Stage 5 不是打印阶段，也不是继续堆 MuJoCo 功能。它把 Basic motion 的质量/力矩、
FEM 的结构限制、IK control 的 IK/control scaffold、MuJoCo 的 contact evidence
合并成复杂地形下的可执行运动约束。

## 1. Scope

Stage 5 覆盖：

- terrain-aware foot target selection
- terrain-aware body height selection
- terrain-aware body attitude selection
- uneven-ground per-leg IK
- support-foot scheduling
- COM / support polygon check
- torque margin check for terrain poses
- joint-limit avoidance
- terrain refusal and recovery rules
- MuJoCo replay cases for terrain IK commands

Stage 5 不覆盖：

- printable CAD implementation
- new actuator selection
- new FEM solver implementation
- full autonomous navigation
- vision / perception stack
- outdoor field validation
- manufacturing process validation

## 2. Inputs

Stage 5 consumes:

- `mass_model.py` outputs: mass, COM, torque margin, leg endpoint residuals
- FEM FEM limits: hot spots, deformation-sensitive poses, load-path limits
- IK/control outputs: joint commands, trajectory primitives, safety limits
- MuJoCo/contact outputs: contact candidates, support proxy, failure evidence
- terrain description: height samples, slope, step edges, friction assumption

Terrain may start as a synthetic local height field. It does not need camera or
perception integration at this stage.

## 3. Core Questions

Stage 5 must answer:

```text
Where can each foot safely land?
What body pose keeps IK, COM, and torque inside limits?
Which legs should stay in support?
Which terrain patches should be refused?
Which terrain poses must be replayed in MuJoCo?
```

## 4. Terrain IK Outputs

Expected outputs:

- terrain contact candidate table
- terrain foot target table
- terrain body pose table
- terrain IK command table
- terrain safety / refusal table
- MuJoCo terrain replay cases
- terrain failure-mode report

## 5. Acceptance

Stage 5 is accepted when:

- flat-ground commands remain compatible with IK control and MuJoCo
- uneven terrain commands produce bounded IK residuals
- every generated pose has torque margin evidence
- every generated pose has COM/support evidence
- unsafe terrain cases are refused instead of silently generating commands
- replay cases can be exported back to MuJoCo/contact review

Stage 5 does not need to prove outdoor survival. It only needs to prove that
complex terrain IK has explicit inputs, outputs, refusal rules, and replayable
evidence.
