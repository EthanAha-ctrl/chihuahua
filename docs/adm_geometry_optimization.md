# ADM 几何优化设想

## 核心命题

这个项目的目标不是找到一个通用意义上的“最优机器狗几何”。真正的 golden loss 是：

```text
behavior 像 Chihuahua
```

也就是说，几何不是最终目标，而是让某种行为分布更自然、更低代价、更高概率地出现的物理载体。

可以把问题写成：

```text
find geometry g
such that
ADM samples conditioned on g
match Chihuahua behavior distribution
```

其中 ADM 指 Action Diffusion Model：一个吃 Gaussian noise、几何信息和行为上下文，然后输出动作/姿态序列的扩散模型。

## 形式化

记：

```text
g: 机器人几何参数
z: Gaussian noise
c: 行为上下文或 command
a_1:T: 一段动作/姿态序列
phi(a_1:T): 行为 embedding
```

ADM 学习：

```text
p_theta(a_1:T | g, c, z)
```

Chihuahua 的真实行为分布记为：

```text
P_chihuahua(phi)
```

对一个候选几何 `g`，ADM 采样得到：

```text
a_1:T ~ p_theta(a_1:T | g, c, z)
```

然后投影到行为空间：

```text
phi(a_1:T)
```

几何的行为损失可以写成：

```text
L_behavior(g) =
  D(
    P_phi(ADM samples | g),
    P_chihuahua(phi)
  )
```

这个 divergence 可以是 KL，但早期不一定要执着于 KL。高维 KL 很脆，support 不重叠时会爆炸。更实用的候选包括：

```text
MMD
sliced Wasserstein
classifier / discriminator divergence
FID-style latent Gaussian distance
low-dimensional density model KL
```

## 为什么不直接比较 raw pose

不要直接在 raw joint angle 或 raw pose 上算 divergence。

不同动物、不同机器人、不同骨架比例的 raw pose 空间并不共享同一个语义。直接比较很容易把“骨架格式差异”误认为“行为不像”。

更稳的方式是先映射到行为 embedding：

```text
phi(action) =
  步频
  步幅 / 身长
  duty factor
  四足落脚相位
  身体上下弹跳
  头部和躯干 yaw scanning
  站距
  抬脚高度
  急停急起节奏
  转弯半径
  acceleration burst
  limb phase relationship
```

我们真正想比较的是：

```text
这个几何下自然生成的行为分布
是否落在 Chihuahua-like behavior manifold 上
```

而不是：

```text
某一帧关节角是否和某只狗完全一样
```

## 总损失

ADM 的行为相似度不能单独作为物理真理。模型可能生成很像 Chihuahua 的动作，但真实机器人做不到。

因此总损失应该包含行为项和物理/工程约束项：

```text
L_total(g) =
  L_chihuahua_behavior(g)
  + L_kinematic_feasibility(g)
  + L_torque_margin(g)
  + L_stability(g)
  + L_collision(g)
  + L_manufacturing_prior(g)
```

当前仓库里的 endpoint geometry scan 可以看作 `L_kinematic_feasibility` 的雏形：它检查 yaw/pitch-waist 拓扑里的 yaw 平面几何下，脚端目标是否吃光腿部 XY reach budget。

## Pipeline

### 1. 收集四肢哺乳动物动作数据

数据可以来自：

```text
mocap
视频姿态估计
多视角重建
已有动物运动数据集
```

训练集不必只包含 Chihuahua。更大的四肢哺乳动物数据可以帮助 ADM 学到通用行为先验，但 Chihuahua 数据需要用于定义目标行为分布。

### 2. Retarget 到共享表示

把不同动物的骨架映射到共享空间：

```text
body frame
normalized limb lengths
foot contact states
limb phase
torso/head orientation
COM-like body motion
```

这一步的目标不是抹掉形态差异，而是让模型能理解：

```text
几何不同，但行为语义可以比较
```

### 3. 训练 ADM

训练条件扩散模型：

```text
p_theta(a_1:T | g, c, z)
```

其中 `c` 很重要。ADM 最好不只吃 geometry，还要吃行为上下文：

```text
walk
trot
turn-left
turn-right
idle-alert
start-stop
approach
startle
```

否则 Chihuahua 的走路、转身、警觉、停顿、突然加速会混成一个大分布，divergence 会变脏。

### 4. 对候选几何疯狂采样

给定第一版机器人几何 `g`：

```text
sample z_i
sample context c_i
generate a_i ~ ADM(g, c_i, z_i)
```

得到这个几何诱导出的 action space。

这一步不是为了找单个最优动作，而是估计：

```text
这个几何下，哪些行为是自然的、高概率的
```

### 5. 过滤物理不可行动作

ADM 是行为先验，不是物理引擎。

采样动作需要经过 feasibility filter：

```text
IK reach
joint limits
self collision
body collision
foot clearance
support polygon
torque margin
motor velocity margin
contact plausibility
```

过滤后的分布才更接近真实机器人可能产生的行为分布。

### 6. 和 Chihuahua 行为分布比较

对 ADM 采样动作和 Chihuahua 真实动作都计算 `phi`：

```text
phi_generated = phi(ADM samples | g)
phi_chihuahua = phi(real Chihuahua motion)
```

然后计算 divergence：

```text
D(phi_generated, phi_chihuahua)
```

这个值就是几何优化里的核心风格损失。

### 7. 优化几何

几何优化可以先用无梯度方法：

```text
CMA-ES
Bayesian optimization
NSGA-II
random search + local refinement
```

原因是整个 pipeline 里会有采样、过滤、碰撞、仿真和统计距离，不一定适合端到端梯度。

优化变量可以从当前仓库已有的参数开始：

```text
body_length_total_m
front_body_fraction
waist_joint_spacing_m
hip_half_width_m
foot_x_offset_m
foot_lateral_outset_m
body_half_width_m
body_z_m
upper_m
lower_m
distal_endpoint_m
waist_yaw_range
waist_pitch_range
```

## 直觉解释

这套方法问的不是：

```text
什么几何在抽象意义上最优？
```

而是：

```text
什么几何会让 Chihuahua-like behavior
在控制器和物理约束下更自然地出现？
```

这和光学设计类似。没有单个 golden curvature，只有针对目标成像风格、像差预算、制造约束和成本的高维权衡。

在这个项目里：

```text
optical aberration budget
对应
robot geometry margin budget
```

几何的坏味道可以分解成：

```text
reach usage 太高
joint limit margin 太小
torque margin 太小
support polygon margin 太小
self-collision margin 太小
packaging margin 太小
gait clearance 太小
```

Chihuahua-like behavior 则是最上层的审美和任务定义。

## 主要风险

### ADM 幻觉

ADM 可能学到“像”的动作，但这些动作在真实机器人上不可执行。

缓解方式：

```text
加入 feasibility filter
加入物理仿真验证
把 torque/contact/stability margin 加入 loss
```

### 行为 embedding 选错

如果 `phi` 没有抓住 Chihuahua 的行为特征，优化出来的几何会对错误指标过拟合。

缓解方式：

```text
手工行为特征 + learned latent 并行
用人类偏好或分类器校验
做 ablation
```

### 数据分布不足

Chihuahua 数据可能很少，且视频姿态估计噪声大。

缓解方式：

```text
用更大的四肢哺乳动物数据训练通用 ADM
用 Chihuahua 数据定义目标分布或 finetune
用 normalized behavioral statistics 降低数据需求
```

### KL 不稳定

直接在高维动作空间算 KL 容易不稳定。

缓解方式：

```text
先降维到 behavior latent
使用 MMD / sliced Wasserstein
或训练 discriminator 估计分布差异
```

## 当前仓库的角色

当前代码是第一层几何压力计：

```text
yaw/pitch-waist geometry
endpoint reach usage
foot clearance
turn radius hint
simple endpoint loss
```

下一步可以把它扩展为 ADM pipeline 里的 feasibility module：

```text
geometry g
ADM sampled behavior
IK / endpoint feasibility check
margin metrics
behavior divergence
geometry optimizer
```

这样 repo 会从“画几何和扫角度”升级成：

```text
Chihuahua-like morphology search bench
```
