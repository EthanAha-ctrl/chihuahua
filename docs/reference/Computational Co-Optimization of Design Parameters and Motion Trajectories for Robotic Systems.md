
造一个机器人得决定：腿多长？actuator（驱动器）装在哪？关节怎么动？这些事互相牵制——腿长一厘米，可能之前能走的步态就走不了了。传统做法：老工程师凭经验试，不行再改，改了再试。或者用遗传算法瞎搜，搜到什么算什么，跑两次结果还不一样。所有合法的 design + motion 组合构成一张**曲面**（manifold）。你站在曲面上某个点（你的初始设计），想往一个方向走（比如"减小最大力"）。**隐函数定理**告诉你：在这个点附近，曲面可以近似成一个平面。你只要知道"我想往哪个方向推一小步"，它就能算出"其他所有参数该怎么跟着调，才能不掉下曲面"。就这么一步一步走，走到满意为止。
1. **有方向感**：每一步都知道为什么要这样改，不是瞎搜
2. **可以随时插嘴**：跑了一半你说"不行，第一条腿太短了，给我加长"，算法自己调整其他参数配合
3. **design 和 motion 一起改**：光改步态能省 17%，连 design 一起改能省 35%——差不多翻倍

### 为什么不完美

1. 只能看到鼻子尖——只做局部优化，万一全局最优在曲面上很远的地方，走不到
2. 每次只能动一个参数，效率不高
3. 不能处理离散选择（比如"装 3 个 actuator 还是 4 个"）
4. 没有 feedback control——仿真里最优的设计，真机上遇到扰动可能拉胯

### 一句话总结

把"设计机器人"这件事从"全局搜索一个大优化问题"变成了"在合法设计空间上做有方向的局部漫步"——优雅、可交互、可复现，但本质上还是局部方法。

---

## 论文概述

**标题**: *Computational Co-Optimization of Design Parameters and Motion Trajectories for Robotic Systems*

**作者**: Sehoon Ha, Stelian Coros, Alexander Alspach, Joohyung Kim, Katsu Yamane (Disney Research / ETH Zürich / Toyota Research Institute)

**发表于**: *The International Journal of Robotics Research (IJRR)*, 2018

**核心问题**: 机器人的 morphology（形态设计）和 motion（运动轨迹）之间存在深度耦合——改一个 link length 就可能影响整个运动是否可行。传统方法要么手动 trial-and-error，要么用 evolutionary algorithm（EA）做全局搜索但缺乏可复现性和收敛保证。这篇 paper 提出了一个基于 **Implicit Function Theorem（隐函数定理）** 的计算框架，在 design parameter 和 motion parameter 构成的 constraint manifold 上做**局部线性化的迭代编辑**，从而实现 design 和 motion 的**联合优化**。

---

## 第一性原理：为什么用 Implicit Function Theorem？

### 核心直觉

考虑一个最基本的事实：当机器人在执行某个运动时，它的 design parameters $\mathbf{l}$ (link lengths)、$\boldsymbol{\alpha}$ (actuator attachment points) 和 motion parameters $\mathbf{q}_i, \boldsymbol{\tau}_i, \mathbf{f}_{i,j}$ (joint positions, actuator forces, contact forces) 必须同时满足一组物理约束（equation of motion, kinematic loop closure, end-effector trajectory tracking 等）。这些约束定义了一个 implicit manifold：

$$\mathbf{f}(\mathbf{y}, \mathbf{z}) = \mathbf{0}$$

其中 $\mathbf{y} \in \mathbb{R}^n$ 是我们**想主动改变**的参数子集，$\mathbf{z} \in \mathbb{R}^m$ 是需要**被动调整**以维持约束的参数子集。

### Implicit Function Theorem 的作用

在当前点 $(\mathbf{y}_0, \mathbf{z}_0)$ 附近做一阶 Taylor 展开：

$$\Delta \mathbf{f} = 0 = \left[\frac{\partial \mathbf{f}}{\partial \mathbf{y}}\right] \Delta \mathbf{y} + \left[\frac{\partial \mathbf{f}}{\partial \mathbf{z}}\right] \Delta \mathbf{z}$$

如果 $\frac{\partial \mathbf{f}}{\partial \mathbf{z}}$ 可逆，则：

$$\Delta \mathbf{z} = -\left[\frac{\partial \mathbf{f}}{\partial \mathbf{z}}\right]^{-1} \left[\frac{\partial \mathbf{f}}{\partial \mathbf{y}}\right] \Delta \mathbf{y} \quad \text{(Eq. 4)}$$

**直觉**: 这告诉你——"如果我想把某个 design parameter 往某个方向推一小步 $\Delta \mathbf{y}$，那么所有其他参数应该怎么跟着变，才能仍然停留在 constraint manifold 上。" 这本质上是在 manifold 上做 tangent space 的投影。

### 与传统 optimization 的关键区别

传统 monolithic optimization 需要定义一个 scalar cost function 然后做全局 minimization，用户必须通过调 weight 来间接控制结果。而这里，用户可以直接说："把第三个 actuator 的最大力减小 1%"，算法自动计算 design 和 motion 该怎么协同变化。这是一种**交互式的、directed editing** 范式。

---

## 算法细节

### 参数定义

全部参数向量 $\mathbf{x}$：

$$\mathbf{x} = [\mathbf{l}^T, \boldsymbol{\alpha}^T, \mathbf{q}_1^T, \ldots, \mathbf{q}_N^T, \boldsymbol{\tau}_1^T, \ldots, \boldsymbol{\tau}_N^T, \mathbf{f}_{1,1}^T, \ldots, \mathbf{f}_{N,M}^T, \boldsymbol{\kappa}]^T$$

| 符号 | 含义 |
|------|------|
| $\mathbf{l}$ | link lengths（不随时间变化） |
| $\boldsymbol{\alpha}$ | linear actuator attachment points |
| $\mathbf{q}_i$ | 第 $i$ 帧 joint positions |
| $\boldsymbol{\tau}_i$ | 第 $i$ 帧 actuator forces |
| $\mathbf{f}_{i,j}$ | 第 $i$ 帧第 $j$ 个 end-effector 的 contact force |
| $\boldsymbol{\kappa} = [\kappa_x, \kappa_y, \kappa_z]^T$ | motion plan scale parameters（可缩放步长等） |
| $N$ | 离散帧数, $M$ | end-effector 数 |

### 约束

1. **Equation of Motion (Eq. 5)**:

$$w_{\text{EOM}}^i(\mathbf{x}) = \left\| M\ddot{\mathbf{q}}_i + \mathbf{c} - R\boldsymbol{\tau}_i - \sum_{j=1}^{M} J_j^T \mathbf{f}_{i,j} \right\|^2 = 0$$

其中：
- $M$：joint-space inertia matrix（依赖 $\mathbf{l}, \mathbf{q}_i$）
- $\mathbf{c}$：gravity + centrifugal + Coriolis forces
- $R$：moment arm matrix，将 actuator force 映射为 joint torque
- $J_j$：第 $j$ 个 end-effector position 对 joint positions 的 Jacobian

2. **End-effector tracking (Eq. 9)**:

$$w_{\text{EE}}^{i,j}(\mathbf{x}) = \left\| \mathbf{e}_j(\mathbf{l}, \mathbf{q}_i) - \bar{\mathbf{e}}_{i,j} \right\|^2 = 0$$

$\bar{\mathbf{e}}_{i,j}$ 是第 $i$ 帧第 $j$ 个 end-effector 的 desired position。

3. **Closed kinematic loop (Eq. 10)**: 闭环约束 $w_{\text{Closed}}^{i,j} = \|\mathbf{p}^A - \mathbf{p}^B\|^2 = 0$

4. **Cyclic motion (Eq. 11)**: $w_{\text{Loop}} = \|\mathbf{q}_1 - \mathbf{q}_N\|^2 = 0$

5. **Inequality constraints**: friction cone, actuator stroke limits ($l_k^{lo} \leq l_{i,k}(\mathbf{x}) \leq l_k^{hi}$), joint limits

### 关键技巧：用 Constraint Gradient 而非 Constraint 本身

这是一个非常巧妙的设计。constraint $w$ 本身是 quadratic form（$\|\cdot\|^2$），在 manifold 上 $w = 0$，其 gradient $\nabla w$ 在 manifold 上也恒为零。如果直接用 $w$ 做 implicit function theorem，Jacobian 全是零，无法得到任何信息。

所以作者改用 **constraint 的 gradient** $C_i = \nabla w_i$ 作为 implicit relationship，然后要求 gradient 的变化为零：

$$\Delta C_i = D_i \Delta \mathbf{x} = 0 \quad \text{(Eq. 13)}$$

其中 $D_i = \frac{\partial C_i}{\partial \mathbf{x}}$ 是 gradient 的 Jacobian（即 $w_i$ 的 Hessian）。这个 $D_i$ 用 **finite difference** 计算（因为解析二阶导数对 Coriolis 力等几乎不可能），但部分 gradient 用解析公式（如 Eq. 25, 26）以提高精度。

### 线性系统与 Null Space 求解

收集所有约束后得到：

$$A_1 \Delta \mathbf{x} = \mathbf{b}_1, \quad A_2 \Delta \mathbf{x} \geq \mathbf{b}_2 \quad \text{(Eq. 15)}$$

用户指定 $\Delta \bar{\mathbf{y}}$（想要改变的参数），剩余 $\Delta \mathbf{z}$ 需要求解。分解后：

$$\min_{\Delta \mathbf{z}} \Delta \mathbf{z}^T S \Delta \mathbf{z} \quad \text{s.t.} \quad A_{1z} \Delta \mathbf{z} = \mathbf{b}_1 - A_{1y} \Delta \bar{\mathbf{y}}, \quad A_{2z} \Delta \mathbf{z} \geq \mathbf{b}_2 - A_{2y} \Delta \bar{\mathbf{y}} \quad \text{(Eq. 21)}$$

$S$ 是 diagonal weight matrix（actuator forces: 0.01, contact forces: 10.0, others: 1.0）——权重的含义是让 actuator forces 最 "cheap" to change（因为你想让它变），contact forces 最 "expensive"（因为不想扰动接触力太多）。

**Null space trick**: 用 SVD 分解 $A_{1z}$，得到 $\Delta \mathbf{z} = \Delta \mathbf{z}_0 + N \mathbf{u}$，其中 $N$ 是 null space basis。这把等式约束消掉，降维成只含不等式约束的 QP 问题（Eq. 23），大大提高求解效率。

### 数值误差校正

因为线性近似会累积误差，每隔几步需要做一个 correction step：
1. 先 fix design parameters，只优化 joint angles 满足 end-effector constraints（Eq. 27）
2. 再 fix joint angles，优化 actuator/contact forces 满足 EOM（Eq. 28）

---

## 实验结果汇总

### 实验配置

| Robot | Actuation | # Actuators | # Frames | # Total Parameters | Time/Iteration |
|-------|-----------|------------|----------|-------------------|----------------|
| Manipulator (Linear) | Linear | 4 | 7 | 68 | 0.8s |
| Four-bar Arm | Rotary | 3 | 11 | 117 | 1.5s |
| Large Quadruped | Linear | 16 | 13 | 596 | 40.2s |
| Small Quadruped | Rotary | 14 | 13 | 489 | 32.1s |
| Tetrabot Rolling | Rotary | 16 | 8 | 391 | 4.3s |
| Tetrabot Walking | Rotary | 16 | 16 | 794 | 9.9s |
| Tetrabot Both | Rotary | 16 | 8 | 1176 | 15.1s |

### 关键结果对比

**1. Linear-actuated Manipulator** (Table 2):

| Design | Max Force (N) |
|--------|--------------|
| Initial | 589 |
| Motion-only optimized | 487 |
| **Design+Motion co-optimized** | **383** |
| Length-edited (Link1→24cm) | 480 |

Co-optimization 减少 35%，远优于 motion-only 的 17%。算法增大了第三 actuator 的 moment arm（从 5cm 到 8.3cm），同时调整 link lengths 保持 range of motion。

**2. Four-bar Linkage Manipulator** (Table 3):

| Design | Max Torque (Nm) |
|--------|----------------|
| Initial | 9.81 |
| Motion-only | 9.74 (< 1% reduction) |
| **Design+Motion** | **5.59 (44% reduction)** |

这里 motion-only 几乎无效（因为 four-bar 的 DOF 限制了 motion 调整空间），但 design optimization 通过缩短 first link（35→12.4cm）改变 velocity ratio $h_k$ 来增大 joint velocity，效果显著。

**3. Large Quadruped (Linear actuators)** (Table 4):

| Task | Initial Max Force (N) | Optimized | Reduction |
|------|----------------------|-----------|-----------|
| Slow Walking | 1300 | 680 | 48% |
| Fast Walking | 2150 | 1290 | 40% |

Slow walking 的优化增大了 knee moment arm；fast walking 则保持小 moment arm 以维持大 range of motion。两种任务产生了**截然不同的 optimal morphology**——这恰恰说明 design 和 task 是耦合的。

**4. Small Quadruped Hardware** (Table 5):

| | Max Torque (Nm, sim) | Max Current (mA, hardware) |
|---|---------------------|---------------------------|
| Initial | 2.08 | 766 |
| Optimized | 1.42 (32% ↓) | 535 (30% ↓) |

Simulation 预测的 32% torque reduction 在 hardware 上体现为 30% current reduction，correlation coefficient (Pearson) = 0.7。

**5. Tetrabot (Spherically-symmetric quadruped)**:

| Gait | Initial | Motion-opt | Design-opt | Both-opt |
|------|---------|-----------|------------|----------|
| Rolling | 2.30 Nm | 1.88 Nm | **1.21 Nm** (47% ↓) | 1.58 Nm |
| Walking | 2.49 Nm | 1.55 Nm (38% ↓) | **0.96 Nm** (61% ↓) | 1.58 Nm |

Multi-gait optimization 的结果 (1.58 Nm) 自然比 single-gait 专用设计差——因为 rolling gait 先到达 lower bound 并触发了 break condition。

---

## 深层技术分析与批判

### 优势

1. **Sensitivity-driven 而非 stochastic**: 相比 EA/GA，这种方法利用了 constraint manifold 的局部几何信息（通过 Hessian），每一步的更新方向是有物理意义的，不是随机变异。这意味着可复现性和收敛速度都更好。

2. **交互式编辑**: 用户可以在任何迭代步骤插入自己的偏好（如 "把 Link1 加长"），而不需要重新定义 cost function。这在实际工程设计中非常实用。

3. **General formulation**: 只要能算 gradient，任何 constraint 都可以加进去。Paper 中展示了 linear actuators、four-bar linkages、rotary servos 三种 actuation 方式都可以用统一的 moment arm matrix $R$ 来表示。

### 局限性

1. **Local optimization only**: 隐函数定理本质上是在 tangent space 上的线性化，只能做局部探索。如果 optimal design 在 manifold 上距离初始点很远，可能需要大量迭代甚至陷入 local minimum。Paper 自己也承认这一点（Section 6）。

2. **Single parameter editing**: 每次迭代只能选一个参数指定 $\Delta \bar{y}$。这在高维空间中效率较低——理想情况下应该能同时沿多个方向做 line search 或 trust region 优化。

3. **Continuous parameters only**: 无法处理离散变量（actuator 数量、footfall pattern、monoarticular vs biarticular 配置等）。这是 fundamental limitation，因为隐函数定理要求可微。

4. **No closed-loop control consideration**: 所有 motion plan 都是 open-loop replay，hardware validation 时甚至没有 balance controller。Real robot 需要 feedback control，而 design optimization 没有考虑 control robustness——一个在 nominal trajectory 上最优的设计，可能在实际执行时由于 disturbance 而表现很差。

5. **Scalability**: Large quadruped 596 个参数，每步 40.2 秒。对于更复杂的 robot（如 humanoid，30+ DOF，数百帧），这个方法可能需要 hours 级别的 computation time。

### 与其他方法的关联

- **与 trajectory optimization 的关系**: 传统 trajectory optimization（如 direct collocation）只优化 motion，fixing design。这篇 paper 的贡献是把它扩展到 design-motion co-optimization，通过 implicit function theorem 建立 design 变化如何 propagate 到 motion 的显式映射。

- **与 differentiable simulation 的关系**: 这个框架本质上是 differentiable dynamics 的早期形式——用 finite difference 计算 $\frac{\partial C}{\partial \mathbf{x}}$。现代 differentiable simulation（如 [Brax](https://github.com/google/brax), [NVIDIA Warp](https://developer.nvidia.com/nvidia-warp), [MuJoCo MJX](https://github.com/google-deepmind/mujoco_mjx)）可以做 end-to-end 的 gradient computation，但核心思想一脉相承。

- **与 co-design 文献**: 在 mechanism design 领域，simultaneous design and control optimization 一直是个经典难题。这篇 paper 的独特之处在于用 implicit function theorem 将其转化为了 manifold navigation 问题，而非传统的 nested optimization 或 bilevel optimization。

---

## 总结

这篇 paper 的核心贡献是一个 conceptual framework：**把 robot design optimization 问题重新表述为在 constraint manifold 上的 directed editing 问题**。隐函数定理提供了从 "我想改变什么" 到 "其他一切该怎么变" 的解析桥梁。虽然方法本身有 local optimization 的固有局限，但这个 formulation 非常优雅，且在实际的 robot design 场景中展现了 practical value（30-60% 的 force/torque reduction across 4 simulation + 2 hardware experiments）。

参考链接：
- [IJRR 期刊页面](https://journals.sagepub.com/home/ijr)
- [Sehoon Ha 主页](https://sehoonha.github.io/)
- [Disney Research](https://www.disneyresearch.com/)
- [PyDART (paper 中使用的 dynamics engine)](https://github.com/jviacavier/pydart)
- [不同iable simulation 综述 (2021)](https://arxiv.org/abs/2106.10259)
