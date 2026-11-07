
游戏当中, 有一堆小怪兽，它们长得奇形怪状——有的两条腿，有的四条腿，有的还长着手臂。你要让它们学会走路、爬山、推箱子。怪兽长什么样和怪兽怎么控制自己的身体，这两件事是耦合在一起的。你不知道是设计一个好身体重要，还是训练一个好大脑重要，还是两者都重要。让虚拟生物自己进化身体形状，同时学习控制身体，看看能发现什么规律. **外层（进化）**：随机生成一堆怪兽身体 → 看谁走得远 → 优胜者的身体做个小 mutation（加条腿、改改关节角度之类的）→ 生成下一代.  **内层（学习）**：每只怪兽从出生开始，大脑是空白的，靠 RL（强化学习）从零学会控制自己的身体走路
关键点：**子代只继承身体结构，大脑每次从零开始学**。跟现实中动物一样——你遗传了你爸的骨骼结构，但走路是你自己学会的。他们搞了三个难度递增的场地：
- **平地**：走直线就行
- **障碍地形**：随机生成山坡、台阶、碎石路
- **障碍地形 + 推箱子**：不光要走，还得把一个箱子推到指定位置

用 1152 个 CPU 核心并行跑，16 小时内进化+训练了大约 4000 种不同身体形态的怪兽。这是迄今最大规模的「进化+学习」同时进行的实验。
发现一：环境越复杂，进化出的身体越"聪明". 他们让在三种环境中进化出来的怪兽，去完成 8 个它们从没见过的新任务（比如巡逻、避障、推球等）。结果：**在复杂环境（障碍+推箱子）中进化出来的怪兽，学新任务又快又好**。复杂环境逼着进化去找到那些物理上天然就稳定、天然就省力的身体设计。这种身体不光在原始任务中好用，到了新任务里也好用——因为身体本身就在帮你省事，大脑不用那么辛苦。类比一下：你在一个坑坑洼洼的工地上干了一年活，你的身体自然变得更结实、更协调。换到一个新环境里，你也比一直在平地上混日子的人适应得快。

### 发现二：Baldwin 效应——学得慢的会被淘汰

这个很有意思。他们只看怪兽一辈子学完之后的最终成绩来决定谁留下（没有专门奖励"学得快的"），但仅仅过了 10 代，怪兽达到同样水平所需的学习时间就**减半了**。

就是说：**原本要花一辈子才学会的技能，到了后代身上，年轻时就会了**。

这不是 Lamarckian（后天技能直接遗传），而是纯 Darwinian 的：能更快学会走路的怪兽，因为在学习期间少摔跤、少浪费能量，所以最终成绩也更好，就自然被选出来了。技能没有写进基因，但**让技能学得更快的身体特征**被写进了基因。

### 发现三：为什么？——省能量 + 天生稳

他们找到了上面两个现象的物理原因：

- **省能量**：进化过程中没有任何人要求"省电"，但进化出来的身体越来越省力。量化的方式是 COW（Cost of Work）——做同样的活花了多少能量。越往后代走，COW 越低。而且越省力的怪兽，学新任务也越快。
- **天生稳定**：把怪兽放在那里不动不控制，看它会不会自己摔倒。越往后代，不会自己摔倒的比例越高。复杂环境中这个趋势更明显。

直觉上很好理解：如果你的身体站着不站就倒，走路的时候大脑就得花大量精力去维持平衡，学习就慢。如果你的身体天生就站得稳，大脑就可以专注学别的事情。

---

## 一句话总结

**身体形态本身就是智能的一部分**。在复杂环境中进化出来的身体，因为被迫变得稳定和高效，反而让大脑的学习变得更轻松——这解释了为什么环境复杂度能催生智能，以及为什么"学得快"这个特征即使不被直接选择也会自然出现。

论文链接：[https://doi.org/10.1038/s41467-021-25874-z](https://doi.org/10.1038/s41467-021-25874-z)  
代码：[https://github.com/agrimgupta/derl](https://github.com/agrimgupta/derl)


---


---

# Deep Evolutionary Reinforcement Learning (DERL): 通过学习与进化实现 Embodied Intelligence

**论文**: Gupta, Savarese, Ganguli & Li Fei-Fei (2021), *Nature Communications* 12:5721  
**链接**: [https://doi.org/10.1038/s41467-021-25874-z](https://doi.org/10.1038/s41467-021-25874-z)  
**代码**: [https://github.com/agrimgupta/derl](https://github.com/agrimgupta/derl)

---

## 1. 核心问题

传统的 AI 研究集中在 disembodied cognition（语言、视觉、游戏），而 embodied cognition 领域认为：**morphology 本身就是 intelligence 的一部分**。一个well-adapted的morphology可以大幅简化控制学习问题。

但此前的工作面临两个根本困难：
- **morphology 搜索空间组合爆炸**：可能的形式太多
- **fitness 评估代价高昂**：每个 morphology 都需要完整训练一个 controller 才能知道好不好

先前的工作为了绕开这些困难，不得不做出大量妥协（如固定 morphology 只优化参数、用手设计的小参数 controller、Lamarckian 遗传等）。本文的目标是**同时 scale 环境复杂度、morphological 复杂度、和控制复杂度**，不使用上述 heuristic。

---

## 2. DERL 框架架构

### 2.1 双层循环结构

DERL 的核心是一个**嵌套的双层优化过程**：

```
外层循环 (Evolution, 跨世代):
    - 通过 mutation 操作搜索 morphology 空间
    - Tournament-based steady-state evolution

内层循环 (Lifetime Learning, 单个个体生命周期内):
    - 从 tabula rasa (随机初始化) 开始
    - 用 PPO 从 raw sensory observation 学习控制策略
    - 最终 reward → fitness
```

关键设计：**子代的 controller 从零开始学习**，只有 morphology 信息从父代继承。这是纯 Darwinian evolution，而非 Lamarckian。

### 2.2 Distributed Asynchronous Evolution

这是 DERL 能 scale 的关键技术贡献。传统的 generational evolution 要求整个种群同步完成学习后才能进入下一代，这在每个个体需要 5M 步 RL 训练的情况下完全不可行。

DERL 借鉴了 Neural Architecture Search 中的 **tournament-based steady-state evolution**：

- 初始种群 P = 576 个具有不同 topology 的 agent
- 每个 worker 独立地从当前种群中随机抽取 T = 4 个 agent 进行 tournament
- 选出 fitness 最高的作为 parent，施加 mutation 生成 child
- Child 经过完整 lifetime learning (5M iterations) 评估 fitness 后加入种群
- **通过 aging 机制维持种群大小**：只保留最近的 P 个个体（而非淘汰最弱者）

为什么用 aging 而非淘汰最弱者？三个理由：
1. **Diversity**：弱个体有多次机会积累 beneficial mutation，避免 premature convergence
2. **Robustness**：非 elitist，lucky 个体最终会被淘汰，只有真正好的 genotype 能通过遗传延续
3. **Fault tolerance**：分布式计算中节点失败不影响种群维护，可以廉价使用 spot instances

**计算规模**：1152 CPUs (16 × Intel Xeon Scalable, 每颗 72 核)，288 个 tournament 并行执行，整个 evolution + learning 在 **<16 小时**内完成，搜索并训练约 4000 个 morphology。

### 2.3 UNIMAL 设计空间

**UNIMAL = UNIversal aniMAL**，是一个 direct encoding 的 morphology 搜索空间。

**Genotype 结构**：kinematic tree（有向无环图）
- Root node：sphere（head）
- 其他 nodes：cylinders（limbs）
- Limbs 之间通过 motor-actuated hinge joints 连接

**三类 mutation 操作**：

| Mutation 类型 | 操作 |
|---|---|
| 结构性 | Grow limb(s), Delete limb(s) |
| 物理属性 | Mutate limb params (length, radius), Mutate density |
| 关节属性 | Mutate DoF, gear ratio, joint angle limits |

**关键约束**：所有 mutation 保持**双侧对称**（bilateral symmetry），这是约 6 亿年前 bilateral animal body plan 的古老保守特征。物理后果是 center of mass 始终在 sagittal plane 上，降低了 left-right balancing 的控制难度。

设计空间包含约 **10¹⁸** 个 unique morphology（<10 limbs），参数全部离散化以避免浪费计算资源在几乎相同的 morphology 上。

---

## 3. 三个环境（复杂度递增）

| 环境 | 简称 | 任务 | 复杂度 |
|---|---|---|---|
| Flat Terrain | FT | 平地上最大化前向位移 | 最低 |
| Variable Terrain | VT | 随机生成的 hills/steps/rubble 上行走 | 中 |
| Manipulation in VT | MVT | VT 上将 box 推到目标位置 | 最高 |

VT 的障碍物：
- **Hills**: 正弦波地形，振幅 a ∈ [0.6, 1.2] m
- **Steps**: 8 级台阶，每级高 0.2 m，先上 4 级再下 4 级
- **Rubble**: 随机三角形锯齿波，裁剪高度 h ∈ [0.2, 0.3] m

每个 episode 随机生成新地形，这使得 VT 极其困难——此前工作对 9-DoF 2D walker 都需要 10⁷ 步训练。

---

## 4. RL 细节

### 4.1 观测空间

**Proprioceptive**（agent 自身状态）：joint angles, angular velocities, velocimeter, accelerometer, gyroscope（安装在 head 上）, touch sensors

**Exteroceptive**（环境信息）：2D heightmap（非均匀采样，近处密远处疏），goal location, object position/velocity

### 4.2 Reward 函数

**FT/VT**（公式 1）：

$$r_t = w_x v_x - w_c \|a\|_2^2$$

其中：
- $v_x$：x 方向（前进方向）速度
- $a$：actuator 输入向量
- $w_x = 1$，$w_c = 0.001$（极弱的 torque 惩罚）

**MVT**（公式 2）：

$$r_t = w_{ao} \Delta d_{ao} + w_{og} \Delta d_{og} - w_c \|a\|_2^2$$

其中：
- $\Delta d_{ao}$：agent 到 object 的测地距离变化（前一步减当前步），鼓励接近 box
- $\Delta d_{og}$：object 到 goal 的测地距离变化，鼓励将 box 推向 goal
- $w_{ao} = w_{og} = 100$，$w_c = 0.001$
- 额外 sparse reward：agent 距 box < 0.75m 时 +10，box 距 goal < 0.75m 时 +10

**关键设计**：tournament selection 中只比较前向位移 $w_x v_x$，**没有显式的 energy efficiency 选择压力**。

### 4.3 Policy 网络

- 每种观测类型经过独立的 2-layer MLP [64, 64] 编码
- 编码后 concat，再经一层 MLP 编码为 64 维向量
- Policy network 输出 Gaussian action 分布参数
- Critic network 输出 discounted future return
- ~250,000 参数，用 PPO 优化
- tanh 非线性（输出层除外）

### 4.4 PPO 目标函数

$$L = \mathbb{E}\left[\min\left(l_t(\theta)\hat{A}_t, \ \text{clip}(l_t(\theta), 1-\epsilon, 1+\epsilon)\hat{A}_t\right)\right]$$

其中：
- $l_t(\theta) = \frac{\pi_\theta(a_t|o_t)}{\pi_{old}(a_t|o_t)}$：新策略与旧策略的概率比
- $\hat{A}_t$：Generalized Advantage Estimation 估计的优势函数
- $\epsilon$：clip 范围参数，构成 approximate trust region

---

## 5. 三大科学发现

### 5.1 环境复杂度催生 Morphological Intelligence

**定义 morphological intelligence**：一个 morphology 促进 RL 在**全新任务**上学习速度和性能的能力（类比 transfer learning 中评估 latent representation 质量的方式）。

**8 个测试任务**，分 3 类：

| Domain | Tasks |
|---|---|
| Agility | Patrol, Point Navigation, Obstacle, Exploration |
| Stability | Escape, Incline |
| Manipulation | Push Box Incline, Manipulate Ball |

每个 morphology 对每个任务**从零训练**，5 个 random seeds。

**结果**：
- MVT agent 在 7/8 个任务上优于 FT agent
- VT agent 在 5/6 个 agility/stability 任务上优于 FT agent
- 当训练步数减少到 1/5（1M vs 5M）时，差异更加显著——说明复杂环境进化的 morphology **学得更快**

这从第一性原理来看是合理的：MVT 环境需要解决的子任务集合 ⊃ VT ⊃ FT。更多的子任务意味着更高的隐式选择压力，迫使 morphology 进化出更 general 的物理特性（如 stability 和 energy efficiency），这些特性在 novel task 中同样有用。

### 5.2 Morphological Baldwin Effect

**Baldwin Effect 背景**（1896, James Mark Baldwin）：

与 Lamarckian inheritance 不同（后者认为后天习得的行为直接遗传给后代），Baldwin effect 是纯 Darwinian 的：

> 如果某个行为在早期世代中需要整个生命周期才能学会，而学会该行为的速度本身具有 survival value（因为学得慢的个体更可能在学习完成前死亡），那么自然选择会偏好那些能**加速该行为习得**的基因型变异。经过若干代后，原本需要长时间学习的行为会在后代生命早期就表现出来。

**本文的强形式 Baldwin Effect**：

DERL 中**没有对学习速度施加任何显式选择压力**——fitness 只看 lifetime learning 结束时的最终 reward。然而：

- 在仅约 10 代 evolution 内，top 100 agent 达到 criterion fitness（初始种群 75th percentile）所需的 learning iterations **减半**
- 第 8 代 agent 不仅最终性能是第 1 代的 2 倍，还能在 1/5 的时间内达到第 1 代的最终 fitness

**机制解释**：energy efficiency 同时与 learning curve 的两个属性正相关——(a) lifetime 结束时的最终性能，(b) 学习初期的速度。因此，只对 (a) 施加选择压力，会隐式地也选择了 (b)。这就是为什么不需要显式的 learning speed selection pressure。

### 5.3 Mechanistic Basis: Energy Efficiency & Passive Stability

#### Cost of Work (COW)

基于 Cost of Transportation (COT) 改编：

$$\text{COT} = \frac{E}{MgD}$$

其中 $E$ = 总能耗，$M$ = 总质量，$g$ = 重力加速度，$D$ = 移动距离。

本文定义 Cost of Work：

$$\text{COW} = \frac{E}{Mgr}$$

其中 $r$ = 获得的 reward（替代距离 $D$），$E$ = 所有关节力矩的绝对值之和。

**发现**：
- 没有任何显式的 energy efficiency 选择压力，但 evolution 仍然选择了更 energy-efficient 的 morphology
- body mass 实际上增加了（不是通过减轻重量来省能）
- 更 energy-efficient 的 morphology 在任何固定 generation 都 perform better（负相关）且 learn faster（正相关）
- MVT/VT agent 比 FT agent 更 energy efficient

#### Passive Stability

**定义**：agent 在无任何控制输入下不摔倒的能力。形式上，当 center of mass 在 support polygon（所有地面接触点的凸包）内且面积 > 0 时，agent 是 passively stable。

**测量方法**：初始化 agent，施加零控制，400 步后检查 head 高度是否 > 原始高度的 50%。

**发现**：
- 所有三个环境中 passively stable 的 morphology 比例随 evolution 递增
- VT/MVT 中的比例高于 FT——更复杂的环境产生更强的 stability 选择压力

---

## 6. 实验数据汇总

| 指标 | FT | VT | MVT |
|---|---|---|---|
| Population fitness 提升 | ~3× | ~3× | ~2× |
| DoF (approx) | ~16 | ~16 | ~16 |
| Stable morphology 比例 | 较低 | 较高 | 较高 |
| Energy efficiency | 最低 | 中 | 最高 |
| Test task 性能 (8 tasks) | 基线 | 5/6 优于 FT | 7/8 优于 FT |
| Baldwin effect (学习时间减半) | ✓ | ✓ | ✓ |

**计算规模**：4000 morphologies × 5M iterations each = 2×10¹⁰ total agent-environment interactions，1152 CPUs，<16h。

---

## 7. 直觉构建

从第一性原理来看，这篇论文揭示的核心 insight 可以这样理解：

**控制问题的难度 ∝ morphology 与环境的物理 mismatch 程度**

当一个 morphology 的 passive dynamics（无控制时的自然物理行为）本身就已经接近目标行为时，controller 只需要做小幅修正——learning 变得容易。这就是 embodied cognition 的物理本质。

Evolution 在复杂环境中被迫发现这样的 morphology：它们的 passive dynamics 天然地提供了 stability 和 energy efficiency，因为这些物理属性在复杂环境中是生存的前提。而这些属性恰好也是在新任务中快速学习的 enabler——因为它们降低了 controller 需要补偿的 "physics mismatch"。

**Baldwin effect 的本质**：fitness 只选择最终性能，但最终性能和初期学习速度共享同一个物理基础（energy efficiency），所以选择前者隐式选择了后者。这不是 magical，而是因为 learning curve 的起点和终点通过 morphology 的物理属性耦合在一起。

---

## 8. 局限性

- **Sim-to-real gap**：MuJoCo 仿真与真实物理仍有差距
- **Sensory simplification**：heightmap 直接提供而非从 raw camera 推断；box 位置直接给定而非需要探索
- **Policy 深度有限**：~250K 参数的浅网络；scale 到更深网络需要更多计算
- **Environment 复杂度仍有限**：没有 multi-agent、没有真正 open-ended evolution
- **Bilateral symmetry 约束**：排除了不对称 morphology 的探索空间

这篇论文是 embodied AI 领域的一个重要 proof of concept，展示了如何通过 scaling computation 来同时探索 morphology 和 control，并从中提炼出关于 learning、evolution、environment 三者交互的科学规律。

