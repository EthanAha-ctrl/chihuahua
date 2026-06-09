# Top-Level Design

## 终极目标

构建一只能够由 Bambu 系列 FDM 打印机制造主要结构件的机械四肢动物。

目标不是先打印一个看起来像狗的外壳，而是建立一个可以收敛的机电设计流程：

```text
geometry
 -> mass / inertia / torque model
 -> structural FEM iteration
 -> IK and control
 -> MuJoCo simulation
 -> printable CAD implementation
```

这个项目的核心判断是：

```text
actuator mass increases torque
torque increases structure size
structure size increases mass
mass increases battery size
battery size increases mass
mass shifts COM
COM changes torque and control
```

因此每个阶段都必须更新质量、重心、惯量、力矩裕度和结构强度，而不是只看静态几何。

## 当前仓库角色

当前仓库已经有 yaw/pitch-waist quadruped 的几何雏形：

```text
dog_description.yaml
endpoint_geometry.py
pygame_endpoint_viewer.py
```

这些代码目前主要回答：

```text
关节拓扑是什么？
hip / endpoint 的几何关系是什么？
腰部 yaw/pitch 后 endpoint reach 是否合理？
关节范围和 linkage 外观是否直觉上成立？
```

下一步要把它从 kinematic sketch 升级为 electromechanical design bench。

## Prerequisites

在正式做 CAD / IK / MuJoCo 之前，需要先建立物理参数表。没有这些参数，后面的 torque、FEM 和 control 都会漂。

### Material Model

需要记录每种候选打印材料的：

```text
density
Young's modulus
Poisson ratio
yield strength
ultimate strength
fatigue assumption
layer adhesion strength
anisotropy factor
recommended print orientation
```

FDM 打印件不能当作各向同性实体。XY 平面强度、Z 向层间强度、孔周围强度、薄壁强度都需要分开处理。

候选材料可以分级：

```text
PLA / PLA+
PETG
ASA / ABS
PA-CF / PAHT-CF
TPU for feet and pads
```

早期仿真可以用保守近似值，但阶段 2 前后需要打印 coupon / test pieces 来校准 FEM 参数。

### Actuator Model

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

actuator 不是只提供 torque，它本身也是 mass，并且通常在离 COM 很远的位置。actuator placement 会直接改变 torque requirement。

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

### Printability Model

需要记录打印约束：

```text
build volume
minimum wall thickness
minimum rib thickness
hole compensation
insert hole dimensions
overhang limits
support accessibility
print orientation
post-processing assumptions
```

这不是阶段 5 才考虑的东西。打印方向会影响强度，因此它必须参与 FEM 和 CAD 迭代。

## Stage 1: Free-Motion Mass And Torque Body

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

## Stage 2: Whole-Robot FEM

以整只机器人作为唯一 FEM 分析对象。

这一阶段不建立 single-link、bracket、shaft seat、insert region 或 subassembly 的独立 FEM 路线。局部区域可以在整机结果中表现为 stress / deformation hot spot，但判断、加强和复验都回到 whole-robot FEM 中完成。

Stage 2 FEM load case 应来自阶段 1 的质量、惯量、姿态和 joint torque
估计，但这一阶段不引入 gravity、ground contact、support reaction 或
fixed feet。重力、接触和支撑稳定性属于后续 MuJoCo/contact 阶段；MuJoCo
结果再回灌 Stage 2，生成新的无重力结构复验工况。

```text
max joint torque
continuous joint torque
worst-case pose
actuator / battery lumped mass inertia
waist yaw / pitch extreme pose
torque replay articulated bend
motion-slice inertial load from Stage 1
mesh / material / radius comparison under the same torque replay
```

FEM 要找：

```text
global body deformation
body torsion
waist load path weakness
hip frame deformation
joint axis misalignment
endpoint deflection under load
whole-body torque / inertia load distribution
torque reaction load path
stress hot spots
layer delamination risk
fatigue-sensitive regions
```

加强方式包括：

```text
wall thickness change
rib placement
fillet radius
hole relocation
boss geometry
print orientation change
material change
load path redesign
standard part substitution
full-frame stiffness redistribution
actuator / battery placement change
```

每次加强之后必须更新：

```text
mass
COM
inertia
joint torque estimate
battery estimate if needed
FEM result
```

阶段 2 不是一次通过，而是一个循环：

```text
whole-robot FEM stress / deformation hot spot
 -> full-model geometry reinforcement
 -> mass update
 -> torque update
 -> COM update
 -> new whole-robot FEM load case
 -> whole-robot FEM again
```

## Stage 3: IK And Control Development

当 mass、torque 和整机结构经过几轮 FEM 后，开始加入 IK 和控制。

早期可以先写 rough IK 来生成 load cases；正式 IK/control 在这一阶段系统化。

核心能力：

```text
foot target to joint angles
joint limit avoidance
torque-aware pose selection
body height control
foot trajectory generation
support polygon awareness
COM-aware stepping
simple gait state machine
fall / overload detection
```

控制开发要带着工程约束：

```text
不要生成 torque margin 不足的动作
不要让整机 FEM 中的高风险姿态长期处在高载荷或大变形状态
不要让脚端轨迹逼近 joint limit
不要让 body pose 把 COM 推出支撑区域
```

阶段 3 的输出：

```text
IK solver
joint trajectory generator
gait primitives
torque margin checker
control-side safety limits
simulation-ready actuator commands
```

## Stage 4: CAD And Control In MuJoCo

把经过阶段 1-3 迭代的 CAD / rigid body / control 放进 MuJoCo。

MuJoCo 模型需要包含：

```text
link geometry
realistic mass
realistic inertia
joint axes
joint limits
actuator torque limits
actuator velocity limits
contact geometry
foot friction
battery / electronics lumped mass
controller loop
```

这一阶段验证：

```text
能否站立
能否低速移动
能否抬腿
能否转身
摔倒模式是什么
哪些关节 torque 经常饱和
哪些姿态让 COM 变危险
脚端接触是否稳定
控制是否依赖不现实的 actuator performance
```

MuJoCo 结果可能会推翻前面的设计，因此阶段 4 之后还要回到阶段 1/2/3：

```text
MuJoCo failure
 -> update load case
 -> update FEM / CAD
 -> update mass / inertia
 -> update IK / control
 -> simulate again
```

## Stage 5: Printing Implementation

只有当 mass、torque、FEM、IK 和 MuJoCo 都达到最低可信度之后，才开始完整打印实施。

打印实施不是探索主设计，而是验证制造和装配：

```text
print orientation
support removal
surface quality
insert installation
shaft / bearing fit
assembly sequence
wire routing
serviceability
field repair
```

阶段 5 应该优先打印：

```text
material coupons
joint coupons
single link
single bracket
single leg
waist module
half body
full body
```

最终整机打印前，必须已有：

```text
known material parameters
known tolerance table
known fastener standard
known actuator placement
known battery placement
known COM estimate
known torque margin
known FEM margin
known MuJoCo behavior
```

## Development Loop

真实开发不是线性的。主循环应该是：

```text
1. update geometry / CAD
2. compute mass / COM / inertia
3. compute static and motion torque
4. run whole-robot FEM
5. update reinforcement
6. update IK / control limits
7. run MuJoCo
8. record failure modes
9. repeat
```

每一轮至少记录：

```text
design version
geometry parameters
material assumptions
actuator assumptions
battery assumptions
mass table
COM
joint torque margins
whole-robot FEM stress / deformation hot spots
control limitations
MuJoCo results
next design decision
```

## Immediate Repo Direction

下一步代码层面应该做这些事情：

```text
1. Add structured physical parameter files.
2. Extend RobotGeometry into mass / inertia / actuator-aware model.
3. Add link mass and COM calculation.
4. Add static torque estimation for representative poses.
5. Add versioned design reports.
6. Document external Stage 2 FEM dependencies.
7. Prepare whole-robot FEM export / load-case generation.
8. Later add IK and MuJoCo export.
```

建议新增文件结构：

```text
materials.yaml
actuators.yaml
batteries.yaml
fasteners.yaml
design_state.py
mass_model.py
torque_model.py
fem_loadcases.py
mujoco_export.py
```

当前 `endpoint_geometry.py` 可以保留为几何压力计，但它不应该再是项目的顶层真相。顶层真相应该变成：

```text
geometry + mass + torque + strength + control + simulation
```

## Design Philosophy

这个项目的重点不是快速打印一个玩具，而是建立一个能够让机械四肢动物收敛的设计系统。

最重要的约束是：

```text
mass is not metadata
torque is not optional
FEM is not post-processing
control is not separate from CAD
printing is not the first prototype
```

只有当这些约束一起闭环，最终的 printed quadruped 才有机会不是一次性展示品，而是一个可以迭代、可以修、可以运动的机械动物。
