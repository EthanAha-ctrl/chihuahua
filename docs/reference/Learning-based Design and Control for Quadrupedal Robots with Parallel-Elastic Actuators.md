ANYmal 是 ETH 做的四足机器人. 每个膝关节有一个电机驱动。电机的活儿就是：站立时顶着身体别塌下去，走路时把腿甩来甩去。电费最贵的地方是啥？不是平均功率，而是**峰值扭矩**。因为电机的发热 $P = \tau^2 / K_t^2 \cdot R$，扭矩是**平方**进去的。你站着一动不动，电机也在吃力、也在烧钱。所以作者想了个招：**给膝关节旁边加一根弹簧**，让它帮忙扛一部分重量，电机就可以轻松点。
为什么加弹簧这事不简单? 直觉上你可能会想：那就加一根弹簧，让它刚好在站立姿态的时候把重力抵消掉，不就完了？不行。因为走路的时候膝关节角度会大幅变化——站立时可能 1.3 rad，迈步摆腿时可能接近 0 rad。如果弹簧设计成了"站立时帮你顶"，那摆腿的时候弹簧就会反过来**拽你**，电机反而要额外出力去克服弹簧。白加了。所以弹簧的"形状"（扭矩随角度怎么变化）得仔细设计。不只是一根直弹簧那么简单。

他们搞了一个很巧的机构：一根线性弹簧，通过钢丝绕过一个**椭圆形的轮子（cam）**，连接大腿和小腿。膝关节转动时，钢丝在椭圆上的接触点会变，力臂会变，于是弹簧扭矩就成了一个**非线性函数**——可以在某些角度帮得多，某些角度帮得少。

椭圆的好处是只用 4 个参数（长短轴 $a, b$、初始角度 $\phi_0$、弹簧平衡位置 $\bar{q}$）就能描述一个丰富的非线性扭矩曲线。

### 怎么找到最优的 4 个参数

这是这篇paper的核心难点。

传统做法：工程师用物理直觉算一下，"站立时重力需要多少扭矩，弹簧应该多硬"——这叫 first-principles design。

他们的做法：**让机器自己学**。

分两步：

**第一步**：训练一个"万能"控制策略。用 RL（PPO）训练一个神经网络，这个网络输入里包含了弹簧参数 $\mathbf{d} = [\bar{q}, a, b, \phi_0]$。每个 episode 随机抽一组弹簧参数，策略就得学会"不管弹簧长啥样我都能走路"。训练完之后，你给这个策略任意一组弹簧参数，它都能在仿真里控制机器人走起来。

**第二步**：用 Bayesian Optimization（贝叶斯优化）搜索最优的弹簧参数。BO 每次提议一组参数，就用第一步的万能策略在仿真里跑 1000 个 episode，看平均表现好不好，然后 BO 根据结果更新自己的猜测，再提议下一组参数。大概 25 轮就收敛了。

### 从仿真到真实

万能策略知道自己用的是哪组弹簧参数，但真实机器人上你不会精确知道这些参数（制造误差）。所以他们又训练了一个 student 策略：不告诉它弹簧参数，只给它传感器的噪声数据（IMU、关节编码器、深度相机），让它自己从历史信息里推断。训练的时候故意把弹簧参数扰动 10%，模拟制造公差。这样部署到真实机器人上就不需要精确标定弹簧。

---

## 效果怎么样

在真实硬件上：

- 膝关节平均扭矩从 26 N·m 降到 7 N·m——弹簧扛了大部分活
- 能耗指标（扭矩平方积分）降低 **33%**
- 最大扭矩降低 30%
- 跑道实测电池续航从 59 分钟提升到 68 分钟，多走了 360 米，提升 **11%**

在苏黎世市中心的楼梯、森林、碎石路上都走了，没摔过。

---

## 几个有意思的点

**33% vs 11% 的差距**：理论上扭矩平方降了 33%，但实际电池只多了 11%。因为机器人不只有电机在耗电——计算机、传感器、电机驱动器的电子损耗都不是 $\tau^2$ 能描述的。这是一个诚实的 gap。

**直觉设计 vs 优化设计**：传统方法（补偿站立时重力）只能降 8% 的能耗，他们的优化方法降了 33%。差了 4 倍。说明工程师的"直觉"在动态运动中严重不够用——走路不是站着不动，弹簧和电机的交互很复杂，让算法去找比让人去猜强得多。

**万能策略的 trade-off**：训练一个"什么弹簧都能用"的策略，不保证对某一个特定弹簧是最优的。但对 4 维参数空间来说够用了，因为不需要对每组参数都重新训练一个策略，省了大量计算。

---

## 一句话总结

给机器人膝盖加根弹簧能省电，但弹簧长啥样得和控制器一起联合优化，靠直觉设计差很远，靠 RL + 贝叶斯优化能找到真正最优的方案，然后在真实世界里验证了确实能多走 11% 的路。


---

让我先阅读这篇paper的更多内容，了解完整的方法和实验细节。




好的，我已经通读了全文。下面从第一性原理出发，系统地讲解这篇paper。

---

## 论文概述

**标题**: *Learning-based Design and Control for Quadrupedal Robots with Parallel-Elastic Actuators*

**作者**: Filip Bjelonic, Joonho Lee, Philip Arm, Dhionis Sako, Davide Tateo, Jan Peters, Marco Hutter (ETH Zürich + TU Darmstadt)

**发表**: IEEE Robotics and Automation Letters (RA-L), 2022

**核心问题**: 如何为四足机器人(ANYmal)的膝关节系统设计**Parallel Elastic Actuator (PEA)**——即在电机旁并联一个弹簧——使得行走时能量效率最大化，同时不牺牲控制性能和鲁棒性？

**主要贡献**:
1. 提出了一个 **co-optimization 框架**：联合优化弹簧机械设计参数和 locomotion controller
2. Sim-to-real transfer：将学习到的策略部署到真实硬件上
3. **首次**在实验室外（户外真实环境）评估 PEA 在行走机器人上的效果

---

## 第一性原理：为什么需要 Parallel Elastic Actuator

### 电机能量损失的物理本质

电机的主要能量损失来自 **Joule heating（焦耳热）**：

$$P_{\text{loss}} = I^2 R = \frac{\tau^2}{K_t^2} R$$

其中：
- $I$ 是电流，$R$ 是电机绕组电阻，$\tau$ 是输出扭矩，$K_t$ 是电机扭矩常数

关键insight: 功率损失正比于**扭矩的平方**，而不是扭矩本身。这意味着峰值扭矩对能量效率的惩罚远大于平均扭矩。论文因此定义了 **Cost of Torque (CoTr)**：

$$\text{CoTr} = \frac{\int \tau^2 \, dt}{m \cdot g \cdot \Delta s}$$

其中：
- $m$ 是机器人总质量
- $g = 9.81 \, \text{m/s}^2$ 是重力加速度  
- $\Delta s$ 是行走距离
- 分子 $\int \tau^2 \, dt$ 是扭矩平方的时间积分（近似焦耳热）
- 分母 $m \cdot g \cdot \Delta s$ 做归一化，使得不同质量/距离的实验可以比较

这类似于 Cost of Transport (CoT)，但用 $\tau^2$ 代替了直接的能量测量。

### 为什么并联弹簧能帮忙

膝关节在行走过程中，由于重力作用，大部分时间需要保持一定的屈曲扭矩来支撑身体。如果你在膝关节旁边并联一个弹簧，让弹簧在"站立姿态"附近提供恰好抵消重力的扭矩，电机就只需要处理**动态变化的部分**，大幅降低 $\tau_{\text{motor}}$，从而降低 $\tau^2$。

但问题在于：弹簧的扭矩-角度关系是非线性的，且行走过程中膝关节角度会周期性变化（swing phase 和 stance phase 的需求完全不同）。如果弹簧设计不当，弹簧在 swing phase 反而会**对抗**电机运动，增加而非减少能耗。

这就是为什么需要**精心设计**弹簧参数，并且需要和控制器**联合优化**。

---

## 方法详解

### 1. Parallel Elastic Knee 机械设计

#### 弹簧-钢丝机构 (Spring-Wire Mechanism)

设计核心是一个**线性弹簧**通过**钢丝**绕过一个**椭圆凸轮(cam)** 连接大腿(thigh)和小腿(shank)：

- 弹簧产生力 $\mathbf{F}_s$，钢丝传递到 cam
- cam 固定在 shank 上，随膝关节旋转
- 弹簧扭矩：

$$\tau_s(q) = \mathbf{F}_s \times \hat{l}_r(\theta)$$

其中：
- $\mathbf{F}_s$ 是弹簧力（向量）
- $\hat{l}_r(\theta)$ 是力臂向量，取决于 cam 的形状和当前膝关节角度 $\theta$
- $\times$ 是叉积（产生扭矩标量）

弹簧力大小由 Hooke's law：$f_s = \|\mathbf{F}_s\| = k_s \cdot \Delta l_s$，其中 $k_s$ 是弹簧刚度，$\Delta l_s$ 是弹簧伸长量。

#### 椭圆凸轮参数化

cam 形状用椭圆参数化：

$$\mathbf{l}_r(\theta) = \mathbf{R}_\phi \begin{bmatrix} a \cos\theta \\ b \sin\theta \end{bmatrix}$$

$$\mathbf{R}_\phi = \begin{bmatrix} \cos\phi & -\sin\phi \\ \sin\phi & \cos\phi \end{bmatrix}, \quad \phi = \phi_0 + q_{\text{KFE}}$$

其中：
- $a, b$ 是椭圆的长轴和短轴半径
- $\theta \in [0, 2\pi)$ 是 cam 上的参数角
- $\mathbf{R}_\phi$ 是旋转矩阵
- $\phi_0$ 是膝关节角度 $q_{\text{KFE}} = 0$ 时椭圆相对于 shank 纵轴的初始角度
- $q_{\text{KFE}}$ 是膝关节当前角度

接触点（钢丝与 cam 的切点）通过求解切线条件确定：

$$0 = \mathbf{F}_s \times \frac{\partial \hat{l}_r}{\partial \theta}$$

这个方程说：当力臂方向与弹簧力方向平行时（叉积为零），就是切点。对于椭圆，这个方程有两个解，通过不等式 $\langle \mathbf{F}_s, \hat{l}_r(\theta) \rangle \geq 0$ 选择正确一侧。

当 $a \neq b$ 时，扭矩 $\tau_s$ 是膝关节角度的非线性函数，这让设计空间更丰富。

#### 设计参数空间

$$\mathbf{d} = [\bar{q}_{\text{KFE}}, a, b, \phi_0]^T \in \mathbb{R}^4$$

- $\bar{q}_{\text{KFE}}$：弹簧平衡位置（弹簧力为零时的膝关节角度）
- $a, b$：椭圆 cam 长短轴半径
- $\phi_0$：cam 初始旋转角

弹簧扭矩最终表示为：$\tau_s(\mathbf{d}) = \psi(q_{\text{KFE}}, \mathbf{d})$，其中 $\psi$ 是一个需要数值求解椭圆积分的非线性函数。

### 2. Co-optimization 框架

整个框架分两步：

#### Step 1: 训练 Design-Conditioned Policy (Teacher)

用 **model-free RL (PPO)** 训练一个策略 $\pi_\theta$，这个策略的观测中**包含设计参数 $\mathbf{d}$** 作为 privileged information。关键在于，每个 episode 开始时，设计参数从 $\mathcal{D}$ 中随机采样，这样策略学会了在**不同弹簧配置下都能控制机器人行走**。

观测 $\mathbf{o}_t \in \mathbb{R}^{133}$ 包含：
- base 目标速度指令
- base 姿态、线速度、角速度
- FTG (Foot Trajectory Generator) 参数
- 关节位置/速度的历史
- 上一步的关节位置目标

Privileged 信息 $\in \mathbb{R}^{46}$ 包含：
- 接触摩擦系数
- 每只脚的接触状态和力
- 作用在 base 上的外力/外力矩
- **设计参数 $\mathbf{d}$**
- 机器人各连杆质量

动作 $\mathbf{a}_t \in \mathbb{R}^{16}$：4个 FTG 频率 + 12个关节位置增量

Reward function 包括：
- 正向/侧向/偏航速度跟踪奖励
- 惩罚 z 方向速度、roll/pitch 角速度
- 惩罚关节位置目标加速度（平滑性）
- **惩罚扭矩 L2 范数**（效率）
- 惩罚超限关节速度和脚滑

网络架构：MLP + auto-encoder。encoder 将 privileged 信息压缩为 embedding $\bar{l}_t$，与 proprioceptive 观测一起输入 policy network。

#### Step 2: Bayesian Optimization 寻找最优设计

有了 design-conditioned policy 后，对任意给定的设计参数 $\mathbf{d}$，可以在仿真中rollout评估性能。优化目标：

$$\mathbf{d}^* = \arg\min_{\mathbf{d} \in \mathcal{D}} \mathbb{E}_{\mathbf{d} \in \mathcal{D}, t \in \mathcal{T}} [f(\mathbf{d} | t, \pi)]$$

$$\text{s.t.} \quad 0 \leq c(\mathbf{d})$$

其中 $f$ 是平均 reward（用 1000 个 episode 的 Monte Carlo 估计），$c(\mathbf{d})$ 是约束条件。

使用 **HEBO (Heteroscedastic Evolutionary Bayesian Optimization)** 算法——NeurIPS 2020 black-box optimization challenge 的冠军方法。BO 的优势在于：
- 目标函数 $f$ 对 $\mathbf{d}$ 不可微（legged locomotion 有大量离散的接触事件）
- 样本效率高（每次评估需要 1000 次 rollout，成本昂贵）

### 3. Sim-to-Real: Deployment Policy (Student)

Teacher 策略依赖 privileged 信息（包括设计参数），但真实机器人没有这些信息。Student 策略通过 **imitation learning** 从 Teacher 学习，关键设计：

- 使用 **RNN** 处理历史观测序列，隐式估计 privileged 信息
- 观测包含 IMU + 关节编码器（proprioceptive）和深度传感器（exteroceptive），均添加仿真噪声
- **设计参数在 student 训练时被扰动 10%**，以模拟制造精度限制——这消除了精确 spring 系统辨识的需求

---

## 实验结果

### 仿真验证

在简化的 2D 设计空间（圆形 cam，$a = b = r$，只有 $\bar{q}$ 和 $r$ 两个参数）中：

- 通过 320,000 次仿真 rollout 绘制了完整的 reward 等高线图
- 最优点在 $\bar{q} \approx 0.0$ rad, $r \approx 6.0$ cm
- **First-principles 设计**（传统方法：补偿平均关节构型的重力，$\bar{q}_{\text{avg}} = 1.3$ rad）只能降低 CoTr **8%**
- **BO 优化设计**降低 CoTr **33%**

这说明了核心 insight：传统的直觉设计方法严重次优，因为静态重力补偿忽略了动态运动过程中弹簧与电机的交互。

5个不同 random seed 的全参数优化，结果标准差：角度参数 < 3°，半径参数 < 1mm，说明方法**高度可重复**。

### 硬件实验

最终优化参数：

| 参数 | 值 |
|------|------|
| $k_s$ (弹簧刚度) | 4154 N/m |
| $a$ (长轴) | 8.1 cm |
| $b$ (短轴) | 6.0 cm |
| $\phi_0$ | 0.0 rad |
| $\bar{q}_{\text{KFE}}$ | 0.36 rad |

AoPS (ANYmal with Parallel Spring) 总质量 52.5 kg vs ANYmal baseline 51.3 kg（弹簧增加了 1.2 kg）。

| 实验 | 指标 | ANYmal (baseline) | AoPS (优化) | 改善 |
|------|------|-------------------|-------------|------|
| Forward walking (16m) | CoTr | ~12 N·m·s | ~8 N·m·s | **-33%** |
| Random command tracking | 效率 | baseline | - | **~30%** |
| Random command tracking | 最大 KFE 扭矩 | 73 N·m | 52 N·m | **-29%** |
| Random command tracking | 平均 KFE 扭矩 | ~26 N·m | ~7 N·m | **-73%** |
| 电池续航 (跑道) | 行走距离 | 2640 m | 3000 m | **+11%** |
| 电池续航 | 运行时间 | 59 min | 68 min | **+15%** |

关键观察：
- KFE（膝关节）平均扭矩从 26 N·m 降到 7 N·m——弹簧承担了大部分静态负载
- HFE（髋关节）扭矩分布也更集中在 0 附近，方差更小——因为膝关节被弹簧辅助后，整个腿的动力学更"顺畅"
- HAA（髋外展/内收关节）基本不受影响——符合预期，因为弹簧只在膝关节

效率指标计算（考虑电池初始电量差异）：

$$\text{Efficiency} = \frac{3000}{2640} \times \frac{0.89 - 0.10}{0.92 - 0.11} = 1.11$$

户外实验（苏黎世中央车站楼梯、森林、山坡等）中机器人零跌倒，证明鲁棒性未受影响。

---

## 关键 Limitations（论文自述 + 我的观察）

1. **CoTr ≠ 实际能耗**: 论文用 $\int \tau^2 dt$ 近似能耗，但忽略了传动损失、电子学损失等。实验中 CoTr 改善 33% 但电池续航只改善 11%，这个 gap 就是未建模损失。

2. **Design-conditioned policy 可能次优**: 一个 "universal" 策略覆盖整个设计空间 $\mathcal{D}$，不保证在某个特定 $\mathbf{d}^*$ 上表现和单独训练的策略一样好。论文承认了这一点，引用了他们之前的工作 [32] 做了专门研究。

3. **机械限制**: 椭圆 cam + 钢丝机构限制了膝关节工作空间，且钢丝的制造精度有限（不同腿的钢丝长度不完全一致）。这也正是为什么 student 训练时要对设计参数做 10% 扰动。

4. **只优化了膝关节**: 设计空间可以扩展到 hip joint 的弹簧、连杆长度、质量分布等，但参数维度增加会显著增加 BO 的样本复杂度。

---

## 我的评价

从 first principles 角度看，这篇工作的核心 contribution 是把一个经典的 **hardware-software co-design** 问题用 learning-based 方法端到端解决了。传统的 PEA 设计依赖工程师直觉（"弹簧应该补偿站立时的重力扭矩"），但这个直觉在动态运动中是**严重次优的**——BO 优化结果 vs first-principles 设计的 CoTr 改善是 33% vs 8%，差了 4 倍。

方法上最有意思的是 **design-conditioned policy** 的思路：不是对每个设计参数单独训练一个策略（成本爆炸），而是训练一个能适应整个设计空间的"万能"策略，然后用 BO 在这个策略上做黑盒优化。这本质上是在策略空间中做了一个 **conditioning**，让策略成为设计参数的函数 $\pi(\mathbf{a} | \mathbf{o}, \mathbf{d})$。

不过也有值得质疑的地方：
- 11% 的实际续航提升相对于 33% 的 CoTr 改善，说明 Joule heating 假设可能高估了实际能耗中电机损失的占比。ANYmal 的计算机、传感器等也有不可忽略的功耗。
- HEBO 在 4D 空间中需要 ~25 次迭代，每次 1000 个 episode 的 rollout，计算成本不低。但相对于传统"设计-制造-测试-迭代"的硬件循环，这已经快了很多。

参考链接：
- Paper: https://arxiv.org/abs/2301.03509
- ANYmal robot: https://www.anybotics.com/
- Teacher-student learning (Lee et al., Science Robotics 2020): https://www.science.org/doi/10.1126/scirobotics.abc5986
- HEBO (NeurIPS 2020 winner): https://github.com/huawei-noah/HEBO
- PPO (Schulman et al.): https://arxiv.org/abs/1707.06347
