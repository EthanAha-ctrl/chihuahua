你想给四足机器人设计腿长，但每改一次设计就得重新训一个 controller 去评估"这腿好不好用"——太贵了。这篇 paper 先花一次钱训一个"什么腿都能快速上手"的 meta-policy，之后随便你抛什么设计过来，它5步微调就能给出接近满分的发挥，于是 design optimization 变得飞快。

---

## 为什么这事难？

设计机器人的腿，本质上是两个纠缠在一起的问题：

1. **腿造多长？**（design problem）
2. **这条腿怎么走？**（control problem）

你不能只解决第一个不管第二个，因为"腿好不好"取决于"你能把它走成什么样"。一条腿配一个差的 controller 可能看起来很差，但其实是个好腿——你只是没走对。

所以你得**同时**搞定 design 和 control，这就是 bilevel optimization——外层搜 design，内层对每个 design 找最优 control。

内层是瓶颈。以前的做法是用 model-based control（trajectory optimization），但问题在于：

- trajectory optimization 需要你**预先定义 gait pattern 和 motion trajectory**
- 它基于简化模型（比如 centroidal dynamics，忽略腿的质量），和真实动力学有 gap
- 它会**拒绝**那些 motion template 容纳不了的设计（比如太长或太短的腿）

结果就是：你优化出来的 design 是被你预设的 motion 限定的，不是真正的最优。Garbage in, garbage out。

---

## 用 RL 当内层会怎样？

RL 不需要你预定义任何 motion。它在 simulator 里自己摸索，什么东西都能学，复杂地形、复杂 actuator 动力学，都不在话下。

但有个坑：**naive multi-task RL 不行**。

如果你把不同 design 参数当 random task 来训一个通用 policy，你会得到一个"什么腿都能走、但哪条腿都走不到最好"的策略。它是一个万金油，不是专家。

问题在于 design optimization 需要对**每个** design 给出**它自己的最佳表现**。如果 controller 只能给出"一般般"的评估，你就分不清"0.61 倍腿长"和"1.02 倍腿长"哪个更好——因为 controller 对两者都只是凑合。

---

## Meta-RL 怎么解决的

MAML 的核心直觉特别简单：

> 与其找一个"对所有 task 都还行"的参数，不如找一个**位置**——从这个位置出发，随便遇到什么 task，走几步 gradient descent 就能到那 task 的最优附近。

打个比方：naive multi-task training 像是在找一个山谷最低的湖，水位不高不低，哪儿都差不多深。MAML 不是在找最低点，它是在找**山口**——站在那儿往任何一个谷底方向走两步就到了。

所以这篇 paper 训一个 meta-policy：对随机采样的 design 参数做 MAML 训练。训练完之后，你随便给它一个新 design，它做 5 步 PPO 微调，就能逼近"专门为这个 design 从头训 12 小时"的 specialized policy 的水平。

Fig. 3(b) 验证了这一点：adapted meta-policy 的 reward 和 specialized policy 基本重叠。

---

## 然后外层就简单了

内层有了又快又准的 evaluator，外层就是个标准的 gradient-free optimization。因为 fitness 函数是通过 rollout 算出来的 Monte-Carlo 估计，不可微，所以用 CMA-ES（进化策略的一种）。

CMA-ES 每代采样一批 design，用 meta-policy 评估每个 design 的 fitness，然后根据 fitness 调整采样分布的均值和协方差，朝着好 design 的方向漂移。30 代 × 35 个 population，1.4 小时搞定。

---

## 实验结论也很直觉

把 cost function 换成不同目标，优化出来的腿就不一样：

| 你关心什么 | 优化出的腿 | 为什么 |
|-----------|-----------|--------|
| **跑得准**（$C_v$） | 平地≈nominal，越难的地形腿越长 | 长腿能跨越更大的障碍 |
| **扭矩小**（$C_\tau$） | 尽量短腿 | 短腿 → 小 moment arm → 小 torque → 小 inertia，省力 |
| **功耗低**（$C_p$） | thigh > shank，适中长度 | 短腿省 torque，长 thigh 让 end-effector 速度快而不需要太高 joint speed，$\text{power} = \tau \times \dot{\phi}$ 两头都要管 |

有个细节很关键——$w_t$ 这个权重因子。它是 $\exp(1.5 \cdot (\text{tracking error})^2)$，tracking 越差权重越大。这意味着：你想省 torque？可以，但先把路走稳了再说。如果你为了省 torque 把腿缩到走都走不动，$w_t$ 会指数爆炸把你的 fitness 拉回深渊。这就自动实现了 Pareto trade-off：**在保证基本能力的前提下**优化效率。

4D 实验更有意思——加入 gear ratio 后，optimizer 选了**大 hip gear + 小 knee gear**。直觉上：hip 承重大需要力矩（高 gear ratio = 低速大扭矩），knee 在 swing phase 需要快速运动（低 gear ratio = 高速小扭矩）。这和生物力学里 hip/knee 的角色分工一致。

---

## 和 Vitruvio 比，赢在哪

Vitruvio 是 model-based 的 design optimization toolbox。拿同样的任务（ANYmal-B，平地，forward 0.36 m/s）对比：

Vitruvio 的最优设计落在 cost map 的高 cost 区域，而且 thigh 和 shank 差异大。本文的设计落在低 cost 区域，thigh/shank 更均衡。

原因：Vitruvio 用 centroidal dynamics（忽略腿质量），用预定义的 trajectory optimization 生成 motion，这会 reject 掉不符合预设 motion 的 design，等于在 design space 上强加了一个 kinematic constraint。相当于你戴着有色眼镜选 design，选出来的不是真实最优，而是"在你的 motion template 能 cover 的范围内"的最优。

---

## 算一笔经济账

| 方案 | 时间 |
|------|------|
| 为每个 design 从头训 specialized policy | 12 h/design |
| Naive approach: 30 代 × 35 designs × 12 h | ~12,600 h |
| Meta-policy 训练（一次性） | 72 h |
| **Design optimization（用 meta-policy）** | **1.4 h** |

72 小时的 meta-training 是一次性成本。之后你换 objective、换地形、换 robot base，都不用重训，1.4 小时跑完一轮 optimization。相当于把昂贵的 control training 摊销了。

---

## 我的看法

这篇 paper 的真正贡献不是"用 RL 做 design optimization"这个 idea 本身——这个 idea 很自然——而是它指出了一个关键的工程结构：**把内层 control 的训练成本从 per-design 摊销成 one-time meta-training**。这个 insight 的价值不止在 legged robot design，任何 co-design 问题（design + control joint optimization）都能用这套结构。

但它的局限也明显：design space 才 2-4 维，离真实的机器人 design（几十上百个参数）差很远。MAML 在高维 task space 下 meta-gradient 的方差会爆炸。而且 cost function 还太简化——真实能耗远不止 positive mechanical power。Sim-to-real 也没有验证。这篇更像是 proof-of-concept，证明了这条路走得通，但离实用还有距离。

---

让我先读取这篇paper的剩余部分，以全面理解其内容。


好的，我已经通读了全文。下面从第一性原理出发，详细拆解这篇paper。

---

## 核心问题

四足机器人（quadrupedal robot）的设计本质上是一个 **bilevel optimization**（双层优化）问题：

- **外层（outer loop）**：优化设计参数 θ_design（如 thigh/shank link length、gear ratio）
- **内层（inner loop）**：对于每一个具体的设计实例，找到最优的控制策略 π*

传统做法的困境：

| 方法类别 | 代表工作 | 核心局限 |
|---------|---------|---------|
| Gradient-based | Ha et al. [7], De Vincenti et al. [8] | 需要可微的 design→performance 映射，依赖简化动力学（如 centroidal dynamics 忽略 limb mass） |
| Gradient-free + Model-based control | Vitruvio [1], Digumarti et al. [11] | 内层依赖 trajectory optimization + 预定义 gait pattern/motion template，无法 end-to-end，且 motion 不一定是该 design 的最优 |

关键矛盾在于：**model-based controller 受限于简化模型和手工 motion primitive，无法为任意 design 找到真正最优的运动策略，因此 design optimization 的结果被 motion template 限制了。**

---

## 这篇paper的方案：Meta-RL + CMA-ES

### 第一性原理推导

1. **内层用 model-free RL 替代 model-based control**：RL 不依赖简化模型，直接在 simulator 中学习，可以处理完整动力学（包括 actuator dynamics）和复杂地形，且不受限于预定义 gait。

2. **但 naive multi-task RL 不够用**：如果只是把不同 design 参数当 random task 来训练一个通用 policy，得到的 policy 对每个 design 只能"generally well"，无法达到 specialized policy 的性能。而 design optimization 需要对每个 design 实例给出**最佳**性能评估——否则你无法区分好设计和坏设计。

3. **Meta-RL (MAML) 解决 fast adaptation**：MAML 训练一个 meta-policy，使其在经过少量 gradient steps 后能快速适应到任意 design，逼近 specialized policy 的性能。

4. **外层用 CMA-ES**：design optimization 的 fitness 函数不可微（因为是通过 rollout 中的 Monte-Carlo 估计得到的），所以用 gradient-free 的 CMA-ES。

### 整体架构

```
┌─────────────────── PHASE 1: META-RL TRAINING ───────────────────┐
│                                                                  │
│  Design Space p(T)        Terrain Curriculum                     │
│  (lt, ls ∈ [0.6, 1.4])    (flat, hills, steps)                   │
│         │                        │                               │
│         ▼                        ▼                               │
│  ┌─────────────────────────────────────┐                        │
│  │     MAML Meta-RL Training Loop       │                        │
│  │                                      │                        │
│  │  for each meta-batch i=1..M:        │                        │
│  │    1. Sample design Ti ~ p(T)       │                        │
│  │    2. Collect K-step rollout D      │                        │
│  │    3. Inner update:                 │                        │
│  │       θ'i = θ - α∇_θ L_Ti(π_θ)     │  ← first-order MAML     │
│  │    4. Collect D' with π_θ'i in Ti   │                        │
│  │  Meta update:                        │                        │
│  │    θ ← θ - β∇_θ Σ L_Ti(π_θ'i)      │  ← second-order update │
│  └─────────────────────────────────────┘                        │
│                    │                                              │
│                    ▼                                              │
│          Trained Meta-Policy π_θ0                                │
└──────────────────────────────────────────────────────────────────┘

┌─────────────────── PHASE 2: DESIGN OPTIMIZATION ────────────────┐
│                                                                  │
│  CMA-ES (G=30 generations, pop=35)                               │
│         │                                                        │
│         ▼                                                        │
│  ┌──── for each design pi in population Pk ────┐                │
│  │  1. Set design ← pi                         │                │
│  │  2. Reset policy: θ ← θ0  (meta-init)       │                │
│  │  3. Adaptation: U=5 PPO gradient updates    │  ← fast adapt   │
│  │     θ ← θ - α∇_θ L_pi(π_θ)                  │                │
│  │  4. Collect 250 transitions → fitness score  │               │
│  │     f(T) = E_{s∼ξ(π_θT)}[-C(s)]            │                │
│  └─────────────────────────────────────────────┘                │
│         │                                                        │
│         ▼                                                        │
│  Update population P_{k+1} via CMA-ES                            │
│         │                                                        │
│         ▼                                                        │
│  Optimized Design                                                │
└──────────────────────────────────────────────────────────────────┘
```

### MAML 的数学核心

MAML 的目标函数是：

$$\min_{\theta} \mathbb{E}_{T_i \sim p(T)} \left[ L_{T_i}\left(\phi_i\right) \right]$$

其中 $\phi_i$ 是 inner-loop adaptation 后的参数：

$$\phi_i = \theta - \alpha \nabla_\theta L_{T_i}(\pi_\theta)$$

- $T_i$：一个 task（这里 = 一组 design 参数 $\langle l_t, l_s \rangle$）
- $L_{T_i}$：在 task $T_i$ 上的 loss（这里是 PPO 的 surrogate loss）
- $\alpha$：inner-loop learning rate
- $\beta$：outer-loop (meta) learning rate
- $\nabla_\theta$：对 meta 参数 $\theta$ 求梯度，这里涉及二阶导数（因为 $\phi_i$ 依赖 $\theta$）

Meta-gradient 展开为：

$$\nabla_\theta \sum_{T_i} L_{T_i}(\pi_{\phi_i}) = \sum_{T_i} \nabla_{\phi_i} L_{T_i}(\pi_{\phi_i}) \cdot \nabla_\theta \phi_i$$

其中 $\nabla_\theta \phi_i = I - \alpha \nabla^2_\theta L_{T_i}(\pi_\theta)$（Hessian 项），但如果用 **first-order MAML (FOMAML)**，则近似为 $I$，忽略二阶项。这篇 paper 采用了 Finn 的原始实现，使用完整 MAML。

**直觉**：MAML 不是在找一个对所有 task 都好的"平均"参数，而是在找一个参数空间的**位置**——从这个位置出发，无论遇到哪个 task，走一两步 gradient descent 就能到该 task 的最优附近。这就是"learn to learn"——meta-learn 的是一个好的 initialization。

---

## MDP 定义细节

**State** $s_t$ 包含：
- velocity command $\langle v_x^{target}, v_y^{target}, \omega_z^{target} \rangle$
- body 的线速度和角速度
- joint states（位置、速度）
- gait pattern generator 的 frequency 和 phase（per foot）
- 最近两步的 action history
- **design parameters**（作为 observation 的一部分，让 policy 知道当前机器人的 morphology）
- privileged observations：contact states、terrain height map（因为是在 simulation 中训练）

**Action** $a_t$：Gaussian policy $a_t \sim \mathcal{N}(m_\theta(s_t), \sigma_\theta)$，由 MLP 输出 mean，$\sigma$ 与状态无关。

**Reward**（完整定义在 Table IV）：

$$R(s) = 0.5 r_v + 0.2 r_\omega + 0.1 r_{v}^{stab} + 0.1 r_{\omega}^{stab} + 0.005 r_{fm} - 0.5 r_{bc} - 0.05 r_{ts} - 0.005 r_{ms} - 0.001 r_\tau$$

各项含义：

| Reward term | 定义 | 作用 |
|-------------|------|------|
| $r_v$ | $\exp\left(-1.5 \cdot \frac{\|v_{xy}^{target} - v_{xy}\|^2}{2}\right)$ | 线速度跟踪 |
| $r_\omega$ | $\exp(-2.0 \cdot (\omega_z^{target} - \omega_z)^2)$ | 角速度跟踪 |
| $r_v^{stab}$ | $\exp(-1.5 \cdot (v_z)^2)$ | 抑制 body 垂直方向晃动 |
| $r_\omega^{stab}$ | $\exp(-1.5 \cdot \|\omega_{xy}\|^2)$ | 抑制 body roll/pitch |
| $r_{fm}$ | $\sum_{i \in I_{swing}} \frac{1_{F_{clear}}(i)}{|I_{swing}|}$ | 鼓励 swing foot 离地（clear ground） |
| $r_{bc}$ | $\|I_{c,body} \setminus I_{c,foot}\|$ | 惩罚 body 碰撞（非脚的部分接触地面） |
| $r_{ts}$ | $\|(r_{f,d})_t - 2(r_{f,d})_{t-1} + (r_{f,d})_{t-2}\|$ | target smoothness（二阶差分） |
| $r_{ms}$ | $\|u_t - u_{t-1}\|$ | action 平滑性 |
| $r_\tau$ | $\sum_{i \in joints} |\tau_i|$ | torque 正则化 |

---

## Design Optimization 的 Cost Function

三个优化目标（在 design optimization 阶段用，与 reward 不同）：

1. **Velocity tracking**:
$$C_v = \sum_{t \in [0,T]} \left( (e_v)_t^2 + (e_\omega)_t^2 \right)$$

2. **Weighted joint torque**:
$$C_\tau = \sum_{t \in [0,T]} w_t \left( \sum_{i \in \{1..12\}} (\tau_i)_t^2 \right)$$

3. **Weighted positive mechanical power**:
$$C_p = \sum_{t \in [0,T]} w_t \left( \sum_{i \in \{1..12\}} \max((\dot{\phi}_i \tau_i)_t, 0) \right)$$

其中：
- $e_v = \|v_{xy}^{target} - v_{xy}\|_2$：线速度跟踪误差
- $e_\omega = \|\omega_z^{target} - \omega_z\|_2$：角速度跟踪误差
- $w_t = \exp\left(1.5 \left((e_v)_t^2 + (e_\omega)_t^2\right)\right)$：**权重因子**，tracking 误差越大，权重指数增长，clip 到 max 100
- $\tau_i$：第 $i$ 个关节的 torque
- $\dot{\phi}_i$：第 $i$ 个关节的角速度
- $\max(\cdot, 0)$：只计算正功率（即 motor 在做正功时才计入消耗，braking 时的能量回收不计）

$w_t$ 的设计很关键：它创建了一个 **Pareto 前沿**——不能为了省 torque/power 而牺牲太多 tracking performance。当 tracking error 大时，$w_t$ 指数爆炸，强制 optimizer 先保证基本的 locomotion 能力。

---

## 实验结果

### 1. Meta-learning 的有效性（Fig. 3）

对比三组 policy：

| Policy | 训练方式 | 结果 |
|--------|---------|------|
| Naive multi-task | 随机采样 design 参数训练 | 在所有 design 上都"一般"，达不到 specialized 水平 |
| Meta-policy (MAML) | MAML 训练 + few-step adaptation | **逼近 specialized policy 性能** |
| Specialized policy | 针对特定 design 单独训练 | upper bound |

Meta-policy 经过 5 步 PPO adaptation 后，reward 达到与 specialized policy 可比的水平。

### 2. 不同 objective × 不同地形（Table I & II）

**Table I — Optimized link scales**（相对于 nominal design 350mm 的 scale factor）：

| Objective | Flat (Thigh/Shank) | Easy Hills | Mid Hills | Hard Hills | Easy Steps | Mid Steps | Hard Steps |
|-----------|--------------------|------------|-----------|------------|------------|-----------|------------|
| $C_v$ | 1.02/0.99 | 1.01/1.01 | 1.06/1.03 | 1.23/1.18 | 1.05/1.0 | 1.07/1.06 | 1.21/1.17 |
| $C_\tau$ | 0.61/0.63 | 0.63/0.67 | 0.64/0.68 | 0.75/0.80 | 0.70/0.68 | 0.76/0.77 | 0.94/0.97 |
| $C_p$ | 1.05/0.94 | 1.07/0.95 | 1.06/0.93 | 1.10/0.97 | 1.04/0.93 | 1.07/0.96 | 1.17/1.13 |

**Table II — 相对 nominal design 的 improvement**：

| | $C_v$ | $C_\tau$ | $C_p$ |
|---|-------|---------|-------|
| Flat | 1.27% | **43.53%** | 4.30% |
| Easy Hills | 2.16% | **43.85%** | 5.07% |
| Mid Hills | 4.32% | 39.72% | 3.01% |
| **Hard Hills** | **27.85%** | 16.36% | **13.47%** |
| Easy Steps | 4.50% | 37.47% | 4.10% |
| Mid Steps | 6.45% | 28.98% | 5.47% |
| **Hard Steps** | **24.79%** | 4.13% | **16.01%** |

**关键 trend**：

- **$C_v$（性能优先）**：flat terrain 下和 nominal design 差不多（1.02/0.99），因为 nominal 本身就是工程师为平地设计的。但 terrain 越难，腿越长（Hard Hills 达到 1.23/1.18），因为长腿能跨越更大的障碍物。Improvement 从 1% 跳到 28%。

- **$C_\tau$（torque 优先）**：一律选最短的腿（0.61/0.63 on flat），因为短腿 → 小 moment arm → 小 torque → 小 inertia。但随着 terrain 变难，腿被迫变长（Hard Steps 0.94/0.97），因为太短无法 locomote。$w_t$ 在这里发挥了 trade-off 作用。Improvement 从 43% 降到 4%（Hard Steps），因为 terrain 难了就必须长腿。

- **$C_p$（power 优先）**：选择 thigh > shank 的比例（如 1.05/0.94），这是一个中间策略——缩短腿来降 torque，但保持较长 thigh 以获得更高的 end-effector velocity 而不需要太高的 joint speed。$\text{power} = \tau \times \dot{\phi}$，所以减小 $\tau$ 和控制 $\dot{\phi}$ 都重要。

### 3. 与 Model-based Baseline (Vitruvio) 对比（Fig. 6）

在 ANYmal-B 上做相同任务（forward locomotion on flat terrain at 0.36 m/s），构建 12×12 cost map：

- **Vitruvio 的最优设计**（blue dots）落在 cost map 的高 cost 区域，且 thigh/shank 差异大
- **本文的最优设计**（yellow stars）落在低 cost 区域，thigh/shank 比例更均衡

原因分析：
- Vitruvio 使用 centroidal dynamics（忽略 limb mass）+ trajectory optimization 的预定义 motion
- 预定义 motion 限制了某些 design 的可行性（太长或太短的腿被 reject）
- Vitruvio 无法建模 ANYdrive actuator 的复杂动力学（neural network dynamics model [29]），而本文可以
- 本文的结果与 Ha et al. [13] 的结论一致：torque minimization 的最优 shank/thigh ratio 在 1.0–1.5 之间

### 4. 4D Design Space（Table III）

加入 hip gear ratio 和 knee gear ratio（nominal: hip=5.6, knee=8.0）：

| $C$ | Terrain | Thigh | Shank | Hip Gear | Knee Gear | Improvement |
|-----|---------|-------|-------|----------|-----------|-------------|
| $C_v$ | Flat | 1.06 | 1.0 | 7.05 | 3.54 | 0.2% |
| $C_v$ | Mid Hills | 1.06 | 1.04 | 7.5 | 3.7 | 5.84% |
| $C_v$ | Mid Steps | 1.07 | 1.06 | 7.7 | 3.8 | 10.8% |
| $C_\tau$ | Flat | 0.60 | 0.65 | **9.42** | 3.0 | **48.8%** |
| $C_\tau$ | Mid Hills | 0.60 | 0.72 | 9.85 | 3.1 | 42.71% |
| $C_\tau$ | Mid Steps | 0.66 | 0.83 | 9.8 | 3.2 | 35.02% |
| $C_p$ | Flat | 0.75 | 0.65 | 9.0 | 3.2 | 18.3% |
| $C_p$ | Mid Hills | 0.81 | 0.62 | 9.63 | 3.0 | 15.88% |
| $C_p$ | Mid Steps | 0.94 | 0.71 | 9.8 | 3.2 | 9.32% |

**发现**：optimizer 倾向于 **higher hip gear ratio + lower knee gear ratio** → 更强的 hip（承重）+ 更快的 knee（swing phase 需要快速运动）。这和生物力学的观察一致：hip 需要力矩，knee 需要速度。

4D 比 2D 的 improvement 更高（如 $C_\tau$ flat 从 43.5% 到 48.8%），因为多了两个 free parameter 可以利用。

### 5. 计算效率（Section III-G）

| 方案 | 时间 |
|------|------|
| 训练 1 个 specialized policy | ~12 hours |
| 训练 meta-policy | ~72 hours |
| Naive approach（每代 35 designs × 12h = 420h/generation × 30 generations） | ~12,600 hours |
| **本文方法（meta-policy 一次性训练 + 每代 fast adaptation）** | **1.4 hours** |

Meta-policy 训练 72 小时是一次性成本。之后每次 design optimization 只需 1.4 小时（AMD Ryzen 7 4800h + GTX 1650Ti，单 GPU 级别的硬件）。这是因为 Algorithm 2 中每个 design 只需要 5 步 PPO update + 250 transitions 来计算 fitness，而非从头训练。

---

## 我的批判性评价

1. **Design space 太小**：只有 2D（后来扩到 4D），实际机器人设计有几十上百个参数（link attachment point、joint range、actuator placement、body shape/material distribution 等）。MAML 在高维 design space 下是否还能保持 fast adaptation 的效果，是一个 open question。MAML 本身在 high-dimensional task space 中的 meta-gradient 估计方差会很大。

2. **Cost function 的局限**（作者自己也承认）：$C_p$ 只计算 positive mechanical power，不包含 transmission loss、joule heating、friction 等实际能耗来源。真实物理系统中的 Cost of Transport (COT) 远比 simulation 中的简化 metrics 复杂。

3. **Sim-to-real gap**：虽然作者用 neural network actuator model [29] 增强了 simulation fidelity，但整个 design optimization 在 simulation 中完成，optimized design 的物理验证缺失（future work 中提到）。

4. **MAML vs 其他 meta-RL 方法**：这篇 paper 用的是经典 MAML（2017），后续有更多 meta-RL 方法（如 PEARL、VariBAD、RL²）可能在这个问题上表现更好或更稳定。MAML 的二阶梯度计算开销大，且对 hyperparameter 敏感。

5. **与 co-design literature 的关系**：这篇本质上做了 **co-design**（design + control joint optimization），但通过 meta-RL 把内层 control 解耦成了一个可复用的 meta-policy。这个 insight 很有启发性——把昂贵的 control training 摊销到一次 meta-training 中。

---

## 相关链接

- **Paper**: [arXiv:2210.02750](https://arxiv.org/abs/2210.02750)
- **MAML (Finn et al., 2017)**: [arXiv:1703.03400](https://arxiv.org/abs/1703.03400) | [Code](https://github.com/cbfinn/maml)
- **Lee et al., 2020 — Learning quadrupedal locomotion over challenging terrain**: [Science Robotics](https://robotics.sciencemag.org/content/5/47/eabc5986)
- **Miki et al., 2022 — Learning robust perceptive locomotion in the wild**: [Science Robotics](https://www.science.org/doi/10.1126/scirobotics.abk6222)
- **Vitruvio (Chadwick et al., 2020)**: [IEEE RA-L](https://ieeexplore.ieee.org/document/8995112)
- **CMA-ES (Hansen et al., 2001)**: [Paper](https://direct.mit.edu/evco/article-abstract/9/2/159/871/)
- **PPO (Schulman et al., 2017)**: [arXiv:1707.06347](https://arxiv.org/abs/1707.06347)
- **RaiSim simulator**: [raisim.com](https://raisim.com)
- **Won & Lee, 2019 — Learning body shape variation in physics-based characters**: [ACM TOG](https://doi.org/10.1145/3355089.3356499)
- **ANYdrive actuator model (Hwangbo et al., 2019)**: [Science Robotics](https://www.science.org/doi/10.1126/scirobotics.aau5872)
