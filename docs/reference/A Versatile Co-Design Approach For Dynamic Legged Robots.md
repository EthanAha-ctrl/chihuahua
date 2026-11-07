造一只机器狗（四足机器人）。两个决定要做: **设计层面**：腿多长？身体多宽多厚？电机多大？齿轮比多少？电子盒放哪里？**运动层面**：这只狗要 trot（小跑）或 jump（跳跃），它该怎么动？每条腿什么时候抬、什么时候落地、用多大力？这两个问题是**耦合的**。把 motion planner 当成一个**黑盒函数**，用并行有限差分算梯度，上层用一个通用 NLP solver 带硬约束地优化设计参数。下层你随便换什么 differentiable motion planner 都行，不用改一行代码。简单、modular、可扩展，比遗传算法更高效，比把所有东西塞进一个方程更可维护。
之前别人怎么做的:
**方法一：随机试（CMA-ES 遗传算法）**: 随机生成一堆设计方案（比如腿长 15cm、电机 50g），让 motion planner 跑一遍看效果好不好，然后根据好的方案生成下一代，迭代。
- 你不能说"两个电子盒不能重叠"这种硬约束，只能写成"如果重叠了就扣分"——结果 optimizer 可能还是给你一个重叠的设计
- 设计参数越多，搜索空间越大，**指数级变慢**

**方法二：全塞进一个大方程一起优化**: 把腿长、电机大小、运动轨迹、接触力……全部当成一个巨大的优化变量，扔给一个 NLP solver。方程太大太复杂，求解器跑不动；而且你想加一个新的设计约束，就得重写整个 motion planner，一点都不 modular。

**方法三：从 motion planner 里推导梯度**: 用数学技巧（隐函数定理、adjoint method）算出"设计参数变化对运动结果的影响"，然后用这个梯度去优化设计。问题：每次只能优化一个设计参数；motion planner 里不能用硬约束（只能用 penalty），可能跑出物理上不合理的运动；也不能加设计约束。

这篇论文怎么做的: **双层结构，各管各的：**

- **下层（motion planning）**：给你一个设计（腿多长、电机多大），我来算这只狗该怎么 trot/jump 最省力。用的是一个现成的、很强的 motion planner（CROCODDYL 库里的 BOX-FDDP），它天然保证物理合理性——摩擦锥约束、接触约束、关节限位都是硬约束。
- **上层（design optimization）**：我给你一个设计，你跑完 motion planning 回来一个"能耗是多少"。我要调设计参数让这个能耗最低。设计层面我可以用硬约束（比如两个盒子不能碰撞、所有零件必须在身体范围内），因为上层用的是通用 NLP solver (KNITRO)。

**关键问题：上层怎么知道"改了设计参数之后，能耗会怎么变"？**

也就是上层需要梯度 $\nabla_\rho J_{cd}$——"我把腿加长 1mm，能耗会降多少还是升多少？"

论文的做法非常简单粗暴：**有限差分 + 并行**。

具体来说，设计参数 ρ 有 17 个分量。我只需要把每个分量单独 perturb 一下（加个 ε），重新跑一次 motion planner，看能耗变化了多少，就算出了这个分量方向的梯度。17 个分量就跑 18 次 motion planner，但可以**并行跑**。

为什么这行得通？因为设计参数维度（17）远小于运动维度（9000+）。你不需要去解析地微分穿过整个 motion planner 内部（那样做既复杂又破坏 modularity），直接数值差分就够了。

## 能耗怎么算的

这是论文里最有意思的部分之一。他们不只是说"关节力矩越小越好"，而是建模了**电机真正的电能消耗**，分三块：

1. **机械功率**：克服摩擦做功
2. **Joule 损耗**：电机线圈电阻发热，正比于力矩的平方除以一个电机常数 $K_m$。$K_m$ 和电机质量有关——**小电机的 $K_m$ 很小，Joule 损耗急剧增大**。这就是为什么 optimizer 倾向于选大一点的电机。
3. **摩擦功率**：传动机构的摩擦，正比于电机转速

总能耗 = 运动过程中 12 个电机的电功率积分，只算正值（忽略能量回收）。

这个 cost function 把物理 reality 建模进去了，所以优化出来的设计不是"数学上好看"而是"真的省电"。

## 实验发现了什么

他们优化了一个叫 SOLO 的开源四足机器人（baseline：电机 53g，gear ratio 9，总重 2.4kg），分别针对 trot 和 jump 两个任务。

| | Trot 最优 | Jump 最优 |
|---|---|---|
| 电机质量 | 179g | 168g |
| Gear ratio | 16 | 17 |
| 总重 | 3.8kg | 3.6kg |

**Trot（小跑）**：因为小跑速度慢、电机转速低，friction loss 小，主要能耗是 Joule loss。所以 optimizer 选了**更大的电机**（Joule loss 降低）和**更高的 gear ratio**。虽然机器人变重了，但电能效率更高。还把身体做厚一点，让腿能装得更高，降低重心。

**Jump（跳跃）**：跳跃要把整个身体抛起来，重量直接增加能耗。所以 optimizer 选了**稍微小一点的电机**，腿和身体也缩小，总重比 trot 最优轻。但 gear ratio 差不多，因为跳跃需要瞬间大力的力。

一个一致的发现：**optimizer 总是选择降低电能损耗、容忍更高的摩擦损耗**。因为小电机的 Joule 损耗指数级恶化，trade-off 的天平倾向大电机。

## 跟 CMA-ES 比怎么样

在 trot 任务上，从 4 个设计参数到 17 个设计参数逐步对比：

- **最终成本**：两者达到相近的水平——说明这种局部梯度方法在这类问题上可能够用了
- **收敛速度**：这篇论文的方法 scalability 更好，参数越多优势越大
- **确定性**：这篇方法是 deterministic（同样的输入永远得到同样的输出）；CMA-ES 是随机的，17 维 jump 任务 worst case 600 秒 vs 本方法 252 秒
- **每轮开销**：本方法每次迭代只需要 dim(ρ)+1 = 18 次 motion planner 调用；CMA-ES 用 population=50 就要 50 次




---


## 论文概述：A Versatile Co-Design Approach For Dynamic Legged Robots

这篇论文来自 Edinburgh Centre for Robotics (University of Edinburgh)，作者是 Traiko Dinev, Carlos Mastall, Vladimir Ivan, Steve Tonneau, Sethu Vijayakumar。核心思想是用 **bilevel optimization (双层优化)** 来同时优化腿式机器人的**机械设计参数**和**运动规划**。

论文链接：[arXiv:2103.04660](https://arxiv.org/abs/2103.04660) | 项目视频：https://youtu.be/Yxn7K1HXt_I

---

### 1. 问题背景：为什么需要 Co-Design

传统机器人设计是一个**迭代手工过程**：机械工程师设计硬件，motion planning 专家调试运动，然后反复修改。Co-design 的目标是把这个过程**自动化**：给定设计参数空间、约束、任务描述、评估指标，算法自动找到最优的 design + motion 组合。

现有方法分两类，各有硬伤：

| 方法 | 代表工作 | 核心机制 | 缺陷 |
|------|---------|---------|------|
| **Sampling-based** (e.g. CMA-ES) | Wampler [5], Digumarti [6], Ha [7], Fadini [9] | Monte-Carlo 采样候选设计 → motion planner 评估 fitness | (1) 不支持 design 上的**硬约束**，只能用 soft penalty + 调参；(2) 复杂度随 design 参数维度**指数增长** (curse of dimensionality) |
| **Single monolithic NLP** | Mombaur [12], Spielberg [14] | 把 motion 和 design 全部塞进一个非线性规划 | (1) NLP 规模巨大，求解困难；(2) 不 modular——加新的 co-design 需求要重写 motion planner |
| **Sensitivity analysis / fixed-point** | Ha [16] (implicit function theorem), Desai [17] (adjoint method) | 从 motion planner 的最优性条件提取 design-motion 关系 | (1) 每次只能优化**一个** design 参数 [16]；(2) 不支持 motion 上的硬约束 (用 penalty) [17-19]；(3) 不支持 design 约束 |

本文提出的方案同时解决上述所有问题。

---

### 2. 核心方法：Bilevel Optimization with Differentiable Motion Planning

#### 2.1 整体架构

```
Upper Level (NLP, solved by KNITRO)          Lower Level (Motion Planning, solved by BOX-FDDP)
┌───────────────────────────┐                ┌──────────────────────────┐
│  min  J_cd(ρ, X, U)       │  ── ρ ──→     │  MP(ρ; TASK) → {X, U}    │
│  s.t. ρ ≤ ρ ≤ ρ̄          │                │  (hybrid optimal control)│
│       g(ρ) = 0            │  ← ∇ρ J_cd ──  │  DDP-style, hard constr. │
│       h(ρ) ≤ 0            │                └──────────────────────────┘
└───────────────────────────┘
```

- **Upper level**: 优化 design 向量 ρ，用 KNITRO (interior-point solver) 求解，支持任意 design 约束
- **Lower level**: 给定 ρ，用 BOX-FDDP (CROCODDYL 库) 求解带接触的 hybrid nonlinear optimal control 问题，输出轨迹 {X, U}

关键创新点：**upper level 不需要知道 lower level 内部结构**，只需要 ∇ρJ_cd。这使得任何 differentiable motion planner 都可以作为 lower level 的 plug-in。

#### 2.2 梯度计算

Co-design cost 对 design 的全导数 (Eq. 2)：

$$\nabla_\rho J_{cd} = \frac{dJ_{cd}(\rho, m)}{d\rho} = \frac{\partial J_{cd}}{\partial m}\frac{\partial m}{\partial \rho} + \frac{\partial J_{cd}}{\partial \rho}$$

其中 $m = \{X, U\}$ 是 motion planner 的输出轨迹。$\frac{\partial m}{\partial \rho} = \frac{\partial \text{MP}(\rho; \text{TASK})}{\partial \rho}$ 是 motion 对 design 的导数。

论文选择**不去解析地推导** $\frac{\partial \text{MP}}{\partial \rho}$（因为要微分穿过一个复杂的 NLP，依赖于 lower level 的具体算法，会破坏 modularity），而是用**并行 finite difference**：

$$\nabla_{\rho_i} J_{cd} \approx \frac{J_{cd}(\rho + \epsilon_i, \text{MP}(\rho + \epsilon_i; \text{TASK})) - J_{cd}(\rho, \text{MP}(\rho; \text{TASK}))}{\epsilon}$$

(Eq. 3)，其中 $\rho + \epsilon_i$ 表示对 ρ 的第 i 个分量加 ε。

这需要 dim(ρ) + 1 次 motion planner 调用，但可以**并行**执行。论文指出 ρ 的维度远小于 motion 的维度（trotting 实验中 dim(ρ) = 17，dim(m) = 9163），所以 finite difference 的开销是可接受的。

---

### 3. Lower Level：Motion Planning 细节

#### 3.1 Hybrid Nonlinear Optimal Control (Eq. 4)

$$\arg\min_{X,U} \sum_{k=0}^{N-1} \|q_k - q_{ref}\|_Q^2 + \|v_k\|_N^2 + \|u_k\|_R^2 + \|\lambda_k\|_K^2$$

各变量含义：
- $q_k$: configuration at step k，$q \in SE(3) \times \mathbb{R}^{n_j}$（6 DoF base + $n_j$ 个关节）
- $v_k$: tangent vector，$v \in \mathbb{R}^{n_x}$（广义速度）
- $u_k$: joint torques, $u \in \mathbb{R}^{n_j}$
- $\lambda_k$: contact forces
- $q_{ref}$: 参考站立姿态
- $Q, N, R, K$: 正定权重矩阵

约束分为 **contact phase** 和 **swing phase**：

**Contact phase** ($\Delta t_p \leq k \leq \bar{\Delta t}_p$)：
- $q_{k+1} = q_k \oplus \int_{t_k}^{t_k + \Delta t_k} v_k \, dt$ (积分)
- $v_{k+1} = v_k + \int_{t_k}^{t_k + \Delta t_k} \dot{v}_k \, dt$
- $(\dot{v}_k, \lambda_k) = f_p(q_k, v_k, u_k)$ (contact dynamics)

**Swing/impact phase**：
- $q_{k+1} = q_k$
- $(v_{k+1}, \lambda_k) = \Delta_p(q_k, v_k)$ (impulse dynamics, 接触切换的 autonomous system [23])

约束还包括：
- **Friction cone** (linearized): $A\lambda \leq r$，A 和 r 由预定义的 polyhedron edges 数量和法向力上下界计算
- **Contact placement** (swing phase): $\log(p_{G(k)}^{-1} \circ M_{p_{G(k)}}) = 0$（用 Lie algebra 的 log 算子约束 swing 足到参考位置）
- State bounds $\underline{x} \leq x_k \leq \bar{x}$，control bounds $\underline{u} \leq u_k \leq \bar{u}$

#### 3.2 Solver: BOX-FDDP

BOX-FDDP (Feasibility-Driven Control-limited DDP) [24] 是 DDP 的变体，采用 **direct-indirect hybridization**：
- Indirect 部分 (DDP): 利用问题结构（动力学方程的 bellman 递归）高效求解
- Direct 部分: 对 control limits 用 hard constraint，对 equality/inequality/state constraints 用 soft quadratic barrier

这保证了 **physical feasibility**：输出的轨迹满足 full-body dynamics + friction cone + contact constraints。

---

### 4. Upper Level：Design 参数与 Cost

#### 4.1 Design 向量 ρ

对于 12-DoF SOLO robot [20]，ρ 包含：
- 上下肢段长度 (upper/lower leg limb lengths)
- 腿的 x, z attachment points
- Trunk shape: width, height, depth
- 两个 electronics box 的 x, z 位置
- Actuator: motor mass $m_m$ + gear ratio $n$（所有电机统一）

对称性约束沿 x 方向（运动方向），dim(ρ) = 17。

#### 4.2 Actuator Model 与 Energy Cost

**Torque limits** (Eq. 5，来自 [9] 对 motor datasheet 的 exponential regression)：

$$\underline{u} = -\bar{u} = 5.48 \, m_m^{0.97}$$

注意 motor mass $m_m$ 增大时 torque limit 增大（近似线性）。

**Motor torque** (Eq. 6)：

$$\tau_t = \frac{u}{n} + \tau_f$$

其中 $n$ 是 gear ratio，$\tau_f$ 是 friction torque。

**Friction** (Eq. 7)：

$$\tau_f = \tau_\mu \, \text{sign}(\omega_m) + b \, \omega_m$$

Coulomb ($\tau_\mu$) + viscous ($b$)，$\omega_m = n \cdot \omega_{joint}$（motor 转速 = gear ratio × 关节转速）。

**三种功率损耗** (Eq. 8)：

$$P_{mech} = \tau_f \omega_m, \quad P_{joule} = \frac{1}{K_m} \tau_f^2, \quad P_{fric} = \tau_f \omega_m$$

其中 $K_m = 0.15 \, m_m^{1.39}$ 是 speed-torque gradient 的 regression（motor 越大 $K_m$ 越大，Joule 损耗越小）。

**总 co-design cost** (Eq. 9)：

$$J_{cd} = \int_{t_0}^{t_N} \sum_{\text{motor}} P_{elec} + \max(P_{fric}, 0) \, dt$$

其中 $P_{elec} = \max(P_{mech} + P_{joule}, 0)$（忽略再生制动，只计正功率）。这就是整个运动过程中 12 个电机的总电能消耗。

#### 4.3 Constraints

**Collision** (Eq. 10)：electronics boxes 与 IMU 之间不碰撞

$$(x_{mb} - x_{imu})^2 + (z_{mb} - z_{imu})^2 \leq (r_{mb} + r_{imu})^2$$

**Bounded placement** (Eq. 11)：所有组件必须在 base 范围内

$$-\frac{w_b}{2} \leq x_{imu} \leq \frac{w_b}{2}, \quad -\frac{d_b}{2} \leq z_{imu} \leq \frac{d_b}{2}, \quad \ldots$$

---

### 5. 实验结果

#### 5.1 两个任务

| Task | 描述 | Knots | Integrator |
|------|------|-------|------------|
| **Trotting** | 前进 2 步，每步 0.05m，步高 0.05m | 22 (swing) + 37 (double support) | Symplectic Euler, 10ms |
| **Jumping** | 前跳 0.1m，步高 0.15m | 20 (flight) + 40 (take-off/landing) | Symplectic Euler, 10ms |

#### 5.2 设计结果

Baseline SOLO: $m_m = 0.053$ kg, $N = 9$, total weight = 2.421 kg

| | Trotting optimal | Jumping optimal |
|---|---|---|
| Motor mass $m_m$ | 0.179 kg | 0.168 kg |
| Gear ratio $N$ | 16.062 | 17.325 |
| Total mass | 3.805 kg | 3.592 kg |

关键观察：

1. **Trotting** 偏好**更大的电机和更高的 gear ratio**。因为 trotting 是相对静态的运动，motor velocity 小，friction loss (velocity-dependent) 小，主导成本是 electro-mechanical energy。更大的 $m_m$ 使 $K_m$ 增大（Joule loss 降低），更高的 $N$ 降 低 electro-mechanical energy。Optimizer 还增大了 base depth，使 upper leg 可以附在更高位置 → 降低 CoM → 增加 stability。

2. **Jumping** 偏好**更轻的电机**。因为跳跃需要移动整个质量，重的 robot 代价更高。但 legs 和 base 更小。gear ratio 与 trotting 相近。

3. **Cost trade-off**：两种情况下 optimizer 都选择 **降低 electrical losses 而增加 friction losses**——因为小电机的 $K_m^{-1} = 0.15^{-1} m_m^{-1.39}$ 随 $m_m$ 减小指数级增长，Joule loss 急剧上升。

4. **Simulation 验证** (PyBullet)：cost improvement 在 simulation 中保持在 optimization 结果的 10% 以内，验证了 dynamics model 的一致性。

#### 5.3 与 CMA-ES 的 Scalability 对比

在 dim(ρ) = {4, 6, 9, 13, 17} 上对比：

- **Per-iteration cost**: 本方法每次迭代只需 dim(ρ) + 1 次 MP 调用；CMA-ES (N=50) 每次 50 次
- **Convergence time**: 本方法 scalability 更好，尤其在高维时
- **Determinism**: 本方法是 deterministic；CMA-ES 有大方差。Jumping 17-DoF 时 CMA-ES worst-case = 600s vs 本方法 252s
- **Final cost**: 两者达到**相近的最优 cost**，暗示本方法的 local gradient-based bilevel 在这类问题上可能找到全局最优

---

### 6. 第一性原理解读

从第一性原理看，这篇论文解决了一个 **coupled optimization** 的本质矛盾：

**Motion planning** (lower level) 本身已经是一个高维 NLP（dim(m) ~ 9000+），如果把 design 参数也放进去，NLP 规模和复杂度爆炸。**Bilevel** 的核心 insight 是：design 参数维度 (dim(ρ) ~ 17) 远小于 motion 维度，所以我们可以把 motion planner 当成一个 **black-box differentiable function** $m = \text{MP}(\rho)$，只通过其 output gradient $\nabla_\rho J_{cd}$ 与 upper level 通信。

这个 separation 带来三个根本性优势：

1. **Modularity**: lower level 可以是任何 differentiable MP——你甚至不需要知道 MP 内部用的是什么 solver（DDP, SQP, direct collocation 都行），只需要它能跑出 feasible motion
2. **Hard constraints on both levels**: lower level 有 friction cone, contact, state/control bounds 的 hard constraints（DDP 天然保证 physical feasibility）；upper level 有 design collision/placement 的 hard constraints（KNITRO 保证）。之前的方法要么在 design 上用 soft penalty (CMA-ES)，要么在 motion 上用 soft penalty (adjoint methods)
3. **Scalability**: gradient-based 方法的 per-iteration cost 正比于 dim(ρ)，而 sampling-based 正比于 population size × dim(ρ) 或者更差。当 dim(ρ) 增大时，finite difference 的并行开销增长是线性的

代价是：finite difference 引入 numerical error，且 gradient-based 方法只保证 local optimality。但实验证据表明，在这类 co-design 问题上 local gradient 足以达到与 global CMA-ES 相当的结果。

---

### 7. Limitations & Future Work

论文自身承认的局限：
1. **Finite difference** 引入 numerical error——future work 计划用 analytical derivatives（需要对 motion planner 做 sensitivity analysis，但这会牺牲一些 modularity）
2. 固定 contact sequence 和 timings——未来可能用 footstep planning 同时优化 contact locations
3. 当前 contact model 不支持 sliding/slipping contacts
4. Upper-lower level 之间是 **weak coupling**：lower level 用 torque squared regularization，upper level 用 Joule cost（也是 torque squared 相关），这种 coupling 的理论上界分析没有给出

一个潜在的根本性问题是 **bilevel optimization 的 non-smoothness**：如果 motion planner 在某些 design 参数下 fail to converge 或者 active set 切换，$\nabla_\rho J_{cd}$ 会变得 noisy 或 discontinuous，这在更复杂的场景下可能成为瓶颈。论文中 BOX-FDDP 的 guaranteed feasibility 在一定程度上缓解了这个问题，但没有正式讨论。

