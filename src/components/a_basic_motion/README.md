

## 模型对比 viewer

`uv run python -m src.components.a_basic_motion.pygame_mass_viewer --model go2`（也可选 `aibo`、`chihuahua`、`pupper`）。
窗口内按 `1/2/3/4` 切换 Chihuahua / Go2 / Aibo ERS-7 / Pupper v3；Go2 和 Aibo 保留 URDF 拓扑，关节角度遵守 URDF 限位。
`B` 全身转动，`M` 单关节，`N` 或 `[/]` 选关节，左右键或横拖关节行调角；`空格` 暂停，`0` 零姿态，`F` 居中，`V` 切换 mesh 包围盒。
模型显示为骨架及包围盒；质量/COM 取自 URDF。`R` 切换自动目标触达，`G` 切换关节控制条；自动选择脚端（Aibo 还包括头部）。`Y` 显示共用材料／力矩面板；URDF 用相对密度缩放质量，力矩裕度使用 `effort` 上限。[模型来源](vendor/reference/urdf/README.md)。

---



## Basic motion: Free-Motion Mass And Torque Body

把当前关节几何展开成带有 mass、COM、inertia 和 torque limit 的 free-motion rigid body model。

这一阶段仍然不做结构破坏分析，也不做完整控制。目标是让系统知道：

```text
每个 link 有多重
每个 link 的 COM 在哪里
每个 link 的 inertia tensor 是什么
每个 joint axis 在哪里
每个 joint 的 torque / speed limit 是多少
actuator 放在哪里
battery 和 electronics 放在哪里
```

核心输出：

```text
rigid body tree
link mass table
joint torque table
static pose torque estimate
worst-pose torque estimate
COM estimate
actuator margin estimate
```

阶段 1 要回答的问题：

```text
当前几何下，站立姿态需要多少 torque？
腰部 yaw/pitch 后 torque 是否爆掉？
单腿抬起时，支撑腿 torque 是否还有余量？
actuator mass 是否导致 torque 递归增大？
battery 放在哪里 COM 最合理？
```

这一阶段应该允许粗略 geometry placeholders，但 mass、COM 和 torque 数据必须结构化。

---



每个候选 actuator 需要至少包含：

```text
mass
dimensions
mounting pattern
max torque
continuous torque
stall torque
max speed
gear ratio
backlash
thermal limit
current draw
voltage
driver mass
```

actuator 提供 torque，它本身也是 mass，并且通常在离 COM 很远的位置。actuator placement 会直接改变 torque requirement。

### Battery And Electronics Model

需要记录：

```text
battery mass
battery volume
battery voltage
usable capacity
peak current
continuous current
BMS / protection assumptions
controller mass
driver mass
wire mass
connector mass
```

电池质量不能当作后期附加物。它会显著改变 COM、支撑多边形、关节 torque 和摔倒载荷。

### Fastener And Bearing Model

需要建立标准件库：

```text
screws
heat-set inserts
shafts
bearings
bushings
washers
spacers
pins
```

每个标准件都要有 mass、尺寸、安装约束和载荷假设。不要每个关节发明一种紧固方式。


---


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

Actuator selection

候选数据见 [joint list](vendor/components/joint/lists.md)。当前不要求连续松弛或梯度穿过内部优化。
中层负责 geometry / material，最内层在固定设计下训练 controller；这项外层决定不额外规定中层或 RL 的算法。
a. 给定 geometry 的基本运动是独立组件，不属于任何优化循环。
