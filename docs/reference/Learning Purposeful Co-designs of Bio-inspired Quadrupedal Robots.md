
让机器人自己"长"出最适合干活的腿——腿的形状和弹簧硬度一起调，比只调一个强得多，而且在真机器人上跑通了。造了一个四足机器人。传统做法是：
1. 先把腿的形状定死
2. 再训练一个 AI 控制它走路

这就像先给你定死一双鞋的尺码和硬度，再让你学跑步——鞋不合适，你跑姿再好也白费。更聪明的做法是**同时优化鞋和跑姿**，这就是所谓的 co-design。之前两个问题：- 要么只调弹簧硬度、不敢动腿的形状（design space 太小）- 要么只在电脑仿真里跑，没在真机器人上验证

### 1. 造了一个叫 Dogger 的机器人

它的腿有几个特点：
- **三段式腿**（大腿、小腿、脚），每段长度可以 3D 打印更换
- 膝盖处有个**弹簧**，换一个塑料轮子就能调硬度，不用买新弹簧
- 拆装方便，换一套腿几分钟搞定

所以它的"可调参数"有 5 个：三段腿的长度 + 弹簧轮子大小 + 弹簧起始角度。

### 2. 两步走的优化方法

**第一步：练一个"万能司机"**

在仿真里，给机器人随机分配各种腿的尺寸和弹簧参数，让它学走路。学了 1.5 亿次交互之后，得到一个 policy——不管你给它什么腿，它都能走。关键是，这个 policy **能看到自己当前腿的参数**，所以它知道"我现在腿长这样，该怎么走最好"。

**第二步：帮每个任务选最合适的腿**

把第一步的"万能司机"固定住，然后问：对于"巡检"这个任务，哪种腿最省电？对于"搬东西"这个任务，哪种腿最省电？

用 Bayesian optimization 来搜（就是一种很会"花小钱办大事"的搜索方法），搜完就得到每种任务的最优腿配置。

### 3. 两个任务，两种腿

**任务一：巡检**——直线走 0.5 m/s，走走停停
最优腿：短大腿 + 长脚 + 中等弹簧

**任务二：搬货**——背着 2kg 东西（占体重 30%）全向走
最优腿：长大腿 + 极短小腿 + 长脚 + 大弹簧轮

两种任务选出来的腿**完全不一样**。

### 4. 真机效果

和"不装弹簧的标准腿"比：
- 巡检任务省电 **18.6%**
- 搬货任务省电 **35.7%**

这个数字还偏保守，因为机器人的电脑等电子器件的静态功耗占了总能耗的一半，这部分跟腿无关、改不了。所以纯走路那部分的省电效果其实更大。

---

## 最关键的发现

他们做了一个消融实验，分别只调腿长、只调弹簧、两个一起调：

| | 只调腿长 | 只调弹簧 | 两个一起调 |
|---|---|---|---|
| 巡检 | 省了 1%（几乎没用）| 省了 7% | **省了 13%** |
| 搬货 | 省了 24% | 省了 16% | **省了 34%** |

两个结论：
1. **不同任务看重的东西不一样**——巡检主要靠弹簧，搬货主要靠腿的形状
2. **两个一起调永远最好**——只调一个都会错过最优解

这就像调吉他：只调弦的粗细不调琴颈长度，或者反过来，都不如两个一起调来得准。

---

## 弹簧为什么能省电？

看膝盖电机的力矩曲线：
- 不装弹簧时，电机力矩一直在零线一侧（一直使劲顶着），峰值很高
- 装了弹簧后，弹簧帮你顶着静态重量，电机力矩就在零线上下波动，峰值低很多

电机发热跟电流平方成正比（$I^2R$），峰值降下来，铜损就大幅下降。弹簧干的活就是把"持续出力"变成"来回出力"，后者对电机友好得多。

---

## 这篇论文的真正价值

不是发明了什么新算法（RL + BO 都不是新的），而是在**真硬件上证明了**：

> 腿的形状和弹簧硬度是互补的，不能只优化一个；而且不同任务确实需要不同的腿。把设计空间扩大，配合学习型控制，能省很多电。

之前的 co-design 工作大多停在仿真里，或者只敢动一个参数。这篇是第一个在真机器人上把几何和柔度同时优化、并且 zero-shot 部署的。

---

## 一句话直觉

**好机器人不是"脑子聪明"或"身体好"就行的，是脑子和身体配合着一起长出来的——而且干不同的活，需要长成不同的样子。**

---


让我先通读整篇论文，再为你做详细讲解。





好的，我已经通读了全文。下面为你做一个详细的技术讲解。

---

## 论文总览：Learning Purposeful Co-designs of Bio-inspired Quadrupedal Robots

**作者**：Fabio Boekel (TU Delft), Jiatao Ding (北京中关村学院/AI Core), Cosimo Della Santina (TU Delft / DLR)

**核心问题**：morphology（形态）和 control（控制）在 legged robotics 中长期被当作独立的设计问题。已有的 co-design 方法要么 design space 太窄（只优化弹簧刚度，不改变腿的几何），要么只在 simulation 中验证，缺少 hardware 证据。这篇论文问的是：**当你同时扩大物理设计空间（leg geometry + passive compliance）并配合 learned control 时，task-driven 的 co-design 能否在真实硬件上带来显著能效提升？**

参考链接：
- arXiv: https://arxiv.org/abs/2611.02462
- 相关前作 Bjelonic et al. (ANYmal PEA co-design): https://ieeexplore.ieee.org/document/10080456
- BirdBot (Badri-Spröwitz): https://www.science.org/doi/10.1126/scirobotics.abg4055
- RAIBO2 (Nature 2026): https://www.nature.com/articles/s41586-026-08383-0

---

## 一、硬件平台：Dogger

### 1.1 设计理念

Dogger 是一个模块化四足机器人，重约 6.5 kg，长 37 cm，宽 27 cm。它的关键特征是**三段式腿 (three-segment leg)** + **four-bar linkage** + **可调 passive knee compliance (PEA)**，全部 3D 打印（PLA/PETG/TPU），用 FDM 打印机即可制造。

### 1.2 设计参数空间

每个 design 向量 **d** 定义为：

$$\mathbf{d} = [\lambda_1, \lambda_2, \lambda_3, \phi, o]^T$$

各变量的含义：

| 参数 | 含义 | 范围 |
|------|------|------|
| $\lambda_1$ | thigh（大腿）长度缩放因子 | [0.70, 1.30] |
| $\lambda_2$ | shank（小腿）长度缩放因子 | [0.70, 1.30] |
| $\lambda_3$ | foot（脚段）长度缩放因子 | [0.70, 1.30] |
| $\phi$ | spring wheel 直径，控制 effective rotational stiffness | [0, 50] mm |
| $o$ | spring offset，控制 spring engagement angle | [0, 10] mm |

实际 link length 由 $l_i = \lambda_i \cdot l_i^n$ 计算，其中 $l_i^n$ 是 nominal 长度。

### 1.3 PEA (Parallel Elastic Actuator) 机制

这是一个 wheel-wire 机构：spring 放在 thigh 关节处，wire 绕过 knee 关节处的 wheel 连接到 shank。关键 insight 是：**更换 wheel 直径 $\phi$ 就能改变 rotational stiffness，无需购买新弹簧**。

Spring 的线刚度 $c_s = 4180$ N/m，通过 wheel 直径 $\phi$ 转换为关节 rotational stiffness，可达 $0.03 - 2.62$ Nm/rad。Standing 位置初始 knee torque 可达 $0 - 5.62$ Nm。

### 1.4 PEA Torque 模型

$$\tau_i(q_i(t), \phi, c_s) = \max\left(\frac{1}{4}\phi^2 c_s \cdot \Delta q_i(t) + \frac{1}{2}\phi \cdot c_s \cdot o,\ 0\right) \tag{2}$$

各变量：
- $q_i(t)$：关节 $i$ 的角度
- $\Delta q_i(t)$：spring 压缩量对应的角度偏移
- $\phi$：wheel 直径（m），它把 spring 的线位移转化为角位移，$\phi$ 越大 → 同样 spring 压缩对应更大角度变化 → effective stiffness 更高
- $c_s$：spring 线刚度 (N/m)
- $o$：spring offset，决定 spring 何时开始 engage（engagement angle）
- $\max(\cdot, 0)$：保证 spring 只在被压缩时产生 torque，不会拉

第一项 $\frac{1}{4}\phi^2 c_s \cdot \Delta q$ 是 position-dependent 的弹性 torque；第二项 $\frac{1}{2}\phi c_s \cdot o$ 是 offset 引入的 bias，使 spring 在特定角度才开始工作。

### 1.5 其他硬件

- **Actuator**：Xiaomi Cybergear 电机，24V，峰值 torque 12 Nm/joint
- **Onboard compute**：NVIDIA Jetson Orin Nano
- **通信**：PEAK M.2 CAN bus（每条腿独立 CAN bus）
- **IMU**：MPU-6050（DMP mode）
- **Power**：2Ah 锂电池 + MJbots power distribution board
- **Control frequency**：50 Hz

---

## 二、Co-design 方法论：两阶段框架

### 2.1 架构概览

```
Stage 1: Universal Policy Training (RL)
  - 在 design space D 中采样不同 design d
  - 训练一个 morphology-conditioned policy πu
  - 观测中包含 design 参数 d → policy "知道"当前腿的形态

Stage 2: Design Selection (BO)
  - 固定 πu
  - 对特定 task k，用 BO 搜索最优 design d*_k
  - 优化目标：minimize Cost of Transport (CoT)
```

### 2.2 Stage 1: RL 训练 Universal Policy

用 PPO 在数千个 parallel environment 中训练，每个 environment 分配不同的 design $d$。

**Observation space**：
$$o'_t = [\mathbf{d},\ \hat{o}'_t]^T$$

其中 $\mathbf{d}$ 是 design 参数（5 维），$\hat{o}'_t$ 包含：
- commanded velocities $(v_x, v_y, \omega_{yaw})$
- gravity vector
- actual yaw rate
- joint positions $q^{mea}_t$
- previous actions $a_t$

**Action space**：desired joint angles，加上 standing pose：
$$q^{des}_t = q^{stand} + a_t$$

通过内部 PD controller 执行。

**Reward 设计**（关键设计决策）：

Reward 由 tracking reward + regularization 组成，但**刻意避免 design-biasing 的 term**：
- ✅ 不包含 body height reward（否则会偏好特定 leg length）
- ✅ 不使用 motion primitives / foot trajectory templates（让 policy 自由探索每种 design 的最优 gait）
- ✅ reward 中包含与 design selection 相同的 CoT 目标，保证 training 和 selection objective 对齐

Tracking reward 用 exponential kernel：
$$r_{track} = \exp\left(-\frac{(v^{des,x} - v_x)^2 + (v^{des,y} - v_y)^2}{\sigma}\right)$$

$\sigma$ = 0.25 是 sensitivity 参数，控制 reward 对速度跟踪误差的敏感度。

Regularization terms 包括：torque penalty ($\sum \tau_i^2$)、mechanical power penalty ($\sum \tau_i \dot{q}_i$)、action rate penalty、foot air time、foot slip 等。

**关键 design choice vs. 前作**：
- vs. [33] (Meta-RL)：不用 meta-RL，用单一固定 policy，简化训练
- vs. [8] (Bjelonic)：不区分 design-aware policy 和 deployment policy，用同一个 policy（deployment policy 也知道 design）；不用 motion primitives

**Training 超参数**：

| 参数 | 值 |
|------|-----|
| Timesteps | 150M |
| Environments | 8192 (Brax) / 4096 (Isaac Lab) |
| Policy network | (128, 128, 128, 128) MLP |
| Learning rate | $3 \times 10^{-4}$ |
| Discount | 0.97 |
| Training time | 15 min – 2 h (RTX 4070) |

### 2.3 Stage 2: BO-based Design Selection

固定 $\pi_u$，对 task $k$ 搜索最优 design：

$$\mathbf{d}^*_k = \arg\min_{\mathbf{d} \in \mathcal{D}} J(\mathbf{d}; k, \pi_u)$$

目标函数是 **Cost of Transport (CoT)**：

$$\text{CoT} = \frac{E_{total}}{\Delta s} = \frac{\int_0^T P_{total}(t)\, dt}{\Delta s} \tag{1}$$

- $E_{total}$：总能量消耗
- $P_{total}(t)$：瞬时总功率
- $\Delta s$：总行进距离
- $T$：总时间

**为什么用 BO 而不是 evolutionary algorithm**：sample efficiency。CoT 对 $\mathbf{d}$ 不可微（需要跑完整个 episode），所以用 derivative-free optimization。具体用 TPE (Tree-structured Parzen Estimator) via Optuna。

**BO 设置**：
- Warm-up：100 个随机采样 design
- 45 iterations，每 iteration 20 个 design
- 每个 design 评估 6000 steps (task 1) 或 15000 steps (task 2)
- 总计算时间约 45 分钟

**关键细节**：Stage 2 中**不做 domain randomization**，确保 design 之间的性能差异归因于 design 本身而非随机环境变化。所有 design 共享 random seed，确保公平比较。

### 2.4 Comprehensive Energy Model

$$P_{total}(t) = P_{mech}(t) + P_{system} \tag{3}$$

- $P_{system}$：Jetson + 电路板等静态功耗，近似常数（约占总能耗 50%！）
- $P_{mech}(t)$：12 个电机总功率

电机功率模型：
$$P_{mech}(t) = \sum_{i=1}^{n} \left[\frac{\tau_i(t)^2}{K_T^2} R + \frac{K_v \cdot \tau_i(t) \cdot \dot{q}_i(t)}{K_T}\right] \tag{4}$$

- $\tau_i(t)$：motor $i$ 的 torque
- $\dot{q}_i(t)$：motor $i$ 的角速度
- $R$：motor 绕组电阻
- $K_T$：torque constant (Nm/A)
- $K_v$：velocity constant

第一项 $\frac{\tau_i^2}{K_T^2} R = I^2 R$ 是 **Joule heating（铜损）**，与电流平方成正比；第二项 $\frac{K_v \tau_i \dot{q}_i}{K_T}$ 是 **机械输出功率**（$P = \tau \cdot \dot{q}$），$K_v/K_T$ 比例修正来自 motor 的 back-EMF 特性。

### 2.5 Sim-to-real 策略

1. **Domain randomization**：body mass, inertia, PD gains, friction, joint damping 等随机化（Stage 1 only）
2. **Noise injection**：observation noise + random velocity perturbation
3. **Feedforward PEA compensation**：用公式 (2) 计算 spring torque $\tau_t^{ff}$，作为 feedforward 叠加到 policy 输出上，让 policy 专注优化 gait 而非补偿 spring
4. **Comprehensive energy model**：确保 simulation 中 CoT 计算包含铜损 + 机械功 + 静态功耗

---

## 三、实验结果

### 3.1 两个 Task

| | Task 1: Inspection | Task 2: Payload Transport |
|---|---|---|
| 场景 | 直线行走 0.5 m/s + 间歇站立 | 全向行走 + 2kg payload（~30% 体重）|
| 命令 | 简单前进 | 16 个不同方向/速度命令，持续 2 min |
| 评估步数 | 6000 steps | 15000 steps |

### 3.2 选出的最优 design

| 参数 | $d^n_1$ (nominal, no spring) | $d^n_2$ (nominal, mid spring) | $d^*_1$ (task 1 optimal) | $d^*_2$ (task 2 optimal) |
|------|-----|-----|-----|-----|
| $\lambda_1$ | 1.00 | 1.00 | **0.86** | **1.30** |
| $\lambda_2$ | 1.00 | 1.00 | **1.02** | **0.70** |
| $\lambda_3$ | 1.00 | 1.00 | **1.28** | **1.30** |
| $\phi$ | 0 | 25 | **33** | **38** |
| $o$ | 0 | 5 | **6** | **2** |

**关键观察**：两个 task 选出了截然不同的 morphology！
- Task 1 (inspection)：短 thigh + 长 foot + 中等 spring → 更适合平稳的低速巡检
- Task 2 (payload)：长 thigh + 极短 shank + 长 foot + 大 wheel + 小 offset → 更适合承载重物

### 3.3 CoT 降低

| Comparison | Sim (task 1) | Hardware (task 1) | Sim (task 2) | Hardware (task 2) |
|------------|-------------|-------------------|-------------|-------------------|
| $d^*_1$ vs $d^n_1$ | **-12.6% ± 0.4%** | **-18.6% ± 1.7%** | -14.4% ± 0.4% | -23.7% ± 2.3% |
| $d^*_2$ vs $d^n_1$ | -7.0% ± 0.4% | -13.5% ± 3.3% | **-34.0% ± 0.3%** | **-35.7% ± 3.1%** |
| $d^n_2$ vs $d^n_1$ | (small) | (small) | (moderate) | (moderate) |

**Hardware improvement 比 simulation 更大**——这是因为 static power（~50% 总能耗）在所有 design 间恒定不变，所以 locomotion-dependent energy 的实际改善远大于 system-level CoT 数字所显示的。

### 3.4 Torque Profile 分析

Fig. 4C 显示 front-right knee motor torque：
- $d^n_1$（无 spring）：peak torque 最高，torque 分布偏向零线一侧（单向加载）
- $d^n_2$, $d^*_1$, $d^*_2$（有 spring）：torque 分布更对称地围绕零线，peak 降低
- $d^*_1$ 和 $d^*_2$：最均衡的 torque 分布，最低 peak

这从第一性原理上解释了 PEA 为什么省能：**Spring 承担了 knee 的静态/准静态负载，使 motor torque 围绕零点波动而非持续偏置**，从而减少了 $I^2R$ 铜损。

### 3.5 Design Space Ablation（消融实验）

比较四种 design scope：

| Scope | Task 1 CoT reduction | Task 2 CoT reduction |
|-------|---------------------|---------------------|
| Fixed (no optimization) | 0% | 0% |
| Link lengths only | 1.1% ± 2.5% | **24.0% ± 0.9%** |
| Springs only | **7.3% ± 1.3%** | 16.3% ± 0.8% |
| **Full (link + spring)** | **12.6% ± 0.4%** | **34.0% ± 0.3%** |

**核心发现**：
- Task 1 (inspection)：passive compliance 贡献更大（7.3% vs 1.1%）
- Task 2 (payload)：leg geometry 贡献更大（24.0% vs 16.3%）
- 两者联合始终最优 → **geometry 和 compliance 是 complementary 的 design freedoms**

### 3.6 Cross-Task Validation

$d^*_2$ 拿去做 task 1 → 有改善但不如 $d^*_1$
$d^*_1$ 拿去做 task 2 → 有改善但不如 $d^*_2$

这说明 co-design 确实找到了 **task-specific optima**，而非一个 universal best。但 cross-task 性能也不差 → 暗示这些 design exploit 了部分共享的 mechanical trade-off。

### 3.7 与其他 Co-design 方法对比

| Method | Task 1 CoT (normalized) | Task 2 CoT (normalized) |
|--------|------------------------|------------------------|
| CPG + CMA-ES | **96.0%** | 103.2% |
| Privileged RL + BO | 112.8% | 103.7% |
| **Ours (Universal RL + BO)** | 100.0% | **100.0%** |

Task 1 中 CPG+CMA-ES 反而略好（低 4%），但 Task 2 中 ours 最优。三种方法选出截然不同的 morphology → **最优 morphology 依赖于 controller 和 optimization formulation**，不存在 controller-agnostic 的最优 design。

### 3.8 户外鲁棒性

在 gravel、tall grass、uneven soil、forest debris 上部署，universal policy $\pi_u$ 保持稳定。连续 17 分钟户外无跌倒。

---

## 四、从第一性原理理解这篇工作的核心 insight

### 4.1 为什么 morphology 和 control 不能分开优化？

Legged locomotion 的能量消耗本质上由两个耦合因素决定：
1. **运动学/动力学路径**（leg geometry 决定了 foot trajectory、moment arm、force direction）
2. **驱动力来源**（motor active torque + spring passive torque 的分配）

改变 $\lambda_1, \lambda_2, \lambda_3$ 会改变 moment arm 和 leg 的 lever ratio → 影响所需 motor torque
改变 $\phi, o$ 会改变 spring 何时/多强地介入 → 影响 motor 需要补偿多少

这两者 **不是独立可加的**：一个长 shank 的腿可能需要 spring 在不同角度 engage 才能获得最优 energy return。消融实验证明了这一点——restricting either group 会 exclude 掉最低能耗的 configuration。

### 4.2 为什么两阶段而非端到端？

端到端 co-design（jointly optimize policy + morphology in one loop）理论上更强，但实践中：
- **Sample efficiency**：morphology 是离散/连续混合的低维空间（5 维），policy 是高维空间（数万参数）。联合优化时 gradient signal 会被 policy 的 stochasticity 淹没。
- **Hardware realizability**：两阶段方法允许你在 Stage 1 用大量 simulation 获得一个 robust universal policy，然后在 Stage 2 用少量 evaluation 找到最优 design 并直接 3D 打印装上。
- **Limitation（作者自己承认）**：fixed policy 限制了可探索的 design-performance frontier。如果 policy 能随 morphology 共同 adapt，可能会发现 fixed policy 无法到达的有效 design-policy pair。

### 4.3 为什么 CoT 作为目标而非 raw energy？

CoT = $E_{total}/\Delta s$ 归一化了行进距离，使得不同速度命令下的 design 可以公平比较。一个走得慢但省能的 design 不会不公平地获胜。

### 4.4 Feedforward PEA compensation 的作用

Spring 产生的 torque $\tau_t^{ff}$ 是由公式 (2) 根据当前关节角度计算出来的 deterministic 量。把它作为 feedforward 加到 PD controller 上，意味着 policy 不需要学习 "如何对抗 spring"——它只需要学 "如何在 spring 已经帮我承重的前提下优化 gait"。这降低了 policy 的 learning burden，提高了 sim-to-real 一致性。

---

## 五、局限与未来方向

1. **Fixed policy 限制**：Stage 2 中 policy 固定，design search 只能在该 policy 能控制的 design 范围内有效。如果某个 design 需要完全不同的 control strategy，fixed policy 可能无法充分发挥其潜力。
2. **Design space 仍有限**：只有 5 个参数（3 个 link length + 2 个 spring 参数）。实际生物系统还有 tendon routing、variable stiffness、multi-joint coupling 等。
3. **CoT 是唯一目标**：没有考虑 agility、stability margin、payload capacity 等多目标 trade-off。
4. **Single policy generalization**：虽然叫 "universal"，但在极端 design（如 $\lambda_1=0.7, \lambda_2=1.3$）下可能 performance 退化。

---

## 六、总结评价

这篇工作的核心贡献在于：**在真实硬件上证明了同时优化 leg geometry 和 passive compliance 的 co-design 显著优于只优化其中之一**，并且不同 task 确实需要不同的 morphology。两阶段框架（universal RL policy + BO design selection）虽然不是理论上最优的 co-design 方案，但它在 sample efficiency、sim-to-real 可靠性和 hardware 可复现性之间取得了很好的平衡。Dogger 平台的模块化设计（换 wheel 就能改 stiffness、3D 打印就能改 link length）使 co-design 从 simulation 验证走向了可重复的 hardware 实验。

如果用一句话总结这篇论文的 first principle：**Robot 的能效不是 "better controller" 或 "better hardware" 能单独决定的，而是两者在 task-specific 条件下的耦合最优——而这个最优解在 design space 中的位置，依赖于 task 本身。**