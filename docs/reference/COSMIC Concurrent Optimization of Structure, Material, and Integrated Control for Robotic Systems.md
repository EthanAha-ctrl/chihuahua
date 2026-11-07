让AI同时设计一个机器人的"身体骨架"、"材料分配"和"控制策略"，而不是像传统做法那样先定身体再学走路。想象你要造一个能走路的机器人。传统流程是：

1. 先有个工程师画好骨架（几条腿、怎么连）
2. 再选材料（哪根杆硬、哪根软）
3. 最后让RL或控制算法去学怎么让它走

他们搞了一个叫**COSMIC**的框架，针对一种**truss-lattice robot**（就是用一根根杆子连成网格的那种机器人）。每个杆子可以是三种状态之一：
- **Void（空的）**：相当于这根杆不存在
- **Skeleton（骨架）**：硬但不动的，负责撑住结构
- **Control（肌肉/致动器）**：软但能动，负责产生力

三个设计维度：

1. **Topology（拓扑）**：哪些位置有杆子、哪些没有 → 决定robot的"形状"
2. **Material（材料分配）**：有杆子的位置，是放骨架还是放致动器 → 决定"哪硬哪软"
3. **Control（控制）**：一个小神经网络，根据时间和身体状态输出每根致动器杆的收缩/拉伸指令 → 决定"怎么动"

难点在于：这三个东西是**离散的**（杆子要么有要么没有，要么是骨架要么是致动器），而gradient-based optimization需要连续变量。

他们的解法是**relaxation + 逐步binarize**：

- 开始时每根杆子都是"三分之一的void + 三分之一的skeleton + 三分之一的actuator"这种模糊状态
- 用softmax把这种模糊状态映射成物理属性（刚度、密度等），这样simulation能跑、gradient能算
- 优化过程中通过penalty逐步逼着每根杆子"表态"——最终收敛到某个确定的one-hot状态
- 最后hard snap一下，所有杆子锁定到离散状态，再单独fine-tune控制网络

---

### 几个聪明的工程细节

**1. Stability Pass**

这个特别重要。优化过程中，有些杆子卡在"半骨架半致动器"的中间态。这些中间态的物理属性是两种材料的混合体，可能恰好维持了一个脆弱的平衡。你一看"性能还行"就以为没问题，但一旦强制binarize，平衡崩了，机器人直接趴窝。

所以他们搞了一个**stability pass**：用一个非常aggressive的softmax把当前设计"预览"成几乎离散的状态，跑一遍simulation看看performance掉不掉。掉了就说明这个中间态是"虚高"的，需要调整。

**2. 为什么不用SIMP**

传统topology optimization用SIMP（Solid Isotropic Material with Penalization）来惩罚中间密度。但在这个dynamic系统里，SIMP会导致simulation不稳定——因为材料属性突然变化会让dynamics炸掉。所以他们改用Augmented Lagrangian + 渐进式binarization，允许中间态暂时存在，慢慢逼向离散。

**3. 控制网络非常小**

就一个hidden layer，32个neuron。作者是故意keep it small的——co-design的optimization landscape已经够nonconvex了，controller太复杂会让优化更难收敛。CPG（Central Pattern Generator）提供基础节奏信号，MLP只是在上面做modulation。

---

### 实验结果说了什么

**1. Co-design碾压sequential design**

手工设计的baseline（三条腿、垂直致动器）走了**1.14m**。Co-design自动evolve出来的设计走了**1.90m**，好66%。而且co-design发现了一个人想不出来的morphology：一条大的后腿负责推进，一条小的前腿负责抓地，用leaping gait前进。

即使用完全无信息的uniform初始化（每根杆子三种状态均分），也能找到一个hopping gait走0.98m。

**2. Ablation study的核心发现**

他们把三个维度（topology、material、control）排列组合，有的开有的关，跑了8组实验：

| 设计维度 | 贡献排名 |
|---|---|
| Material（actuator放哪） | **最大**，+781% |
| Topology（结构拓扑） | 中等，+116% |
| Control（控制策略） | 最小，单独用甚至可能变差 |

这跟直觉可能不太一样——很多人觉得control最重要。但实际是：**如果你把致动器放错位置，再聪明的控制也救不回来**。反过来，只要致动器位置放对了，哪怕控制策略很烂，机器人也能凑合走。

但关键的是三者**联合**的effect远超各自之和。比如control单独贡献小，但当它和material一起优化时，travel distance翻了87%。这说明control是一个"放大器"——它需要好的physical substrate才能发挥作用。

**3. Convergent evolution**

不同初始条件下的优化，竟然独立发现了相同的structural motif——在两根passive cross-brace之间插一根horizontal actuator来协调前后腿。这跟生物学里的convergent evolution一模一样：不同物种因为面对相同的physical constraint，独立演化出相同的结构。

**4. 适应不同环境**

- 上坡：自动evolve出更低的重心 + arching motion
- 更小的design space（5×5而不是6×6）：自动rediscover了腿的division策略
- 滑地面（有摩擦系数）：把payload贴地当锚，front leg折叠当刹车垫

---

### 我的看法

这篇paper的核心贡献不在某一个单独的技术点，而在于**把三件事拼在一起并且让它们work**：

1. 把discrete design variable relaxation成连续的，再逐步binarize
2. 在differentiable simulator里把structure-material-control耦合起来
3. 用Augmented Lagrangian + stability pass来navigate极度nonconvex的landscape

最valuable的其实是**ablation study的insight**：material distribution > topology > control这个hierarchy，以及三者的synergy effect大于各自之和。这个结论对robot design community有实际的指导意义——你在control上花再多功夫，如果physical design不对，天花板就在那。

Limitations也很明显：2D、simplified physics、对initialization敏感、只有3种material state。但这些不影响它作为一个proof-of-concept的价值。

**相关链接**：
- Taichi (他们用的differentiable simulation平台): https://github.com/taichi-dev/taichi
- DiffTaichi原始paper: https://arxiv.org/abs/1910.00935
- SoftZoo (类似的co-design但针对soft robot): https://arxiv.org/abs/2303.09555
- Kriegman的evolutionary co-design: https://arxiv.org/abs/2502.10862

---

我来阅读这篇paper的更多内容，以便给你做详细的技术讲解。





好的，我已经通读了全文。下面来详细讲解这篇paper。

---

## COSMIC: Concurrent Optimization of Structure, Material, and Integrated Control for Robotic Systems

**作者**: Qinsong Guo, Liwei Wang (Carnegie Mellon University, Mechanical Engineering)
**时间**: May 2026

---

### 1. 核心问题与动机

这篇paper解决的核心问题是：**机器人系统中的结构 (structure)、材料 (material) 和控制 (control) 通常被分开设计**，而自然界中这三者是co-evolve的。分开设计会导致suboptimal solution，错过更优的joint optimum。

第一性原理来看，这个问题的根源在于：系统动力学 (system dynamics) 中，结构决定force transmission path，材料决定stiffness/density/actuation capacity，控制决定time-varying input。三者通过Newton运动方程耦合在一起：

$$m_i \ddot{x}_i = \sum_{j \in \mathcal{N}(i)} F_{ij}(k_{ij}, \epsilon_{\text{target},ij}(t), l_{ij}(t)) + m_i g$$

其中 $k_{ij}$ 取决于材料选择，$F_{ij}$ 的方向取决于拓扑连接，$\epsilon_{\text{target}}$ 取决于控制策略。三者无法解耦，因此分开优化只会在各自的subspace里找local optimum。

**难点在于discrete truss-lattice robot**：与continuum soft robot不同，离散系统的design space是combinatorial的，optimization landscape高度非凸甚至不连续，gradient-based optimization很难直接应用。

---

### 2. 方法框架

COSMIC框架分三个模块：**Design Representation → Differentiable Simulation → Constrained Optimization**。

#### 2.1 Design Representation

**拓扑与材料布局**：每个truss edge $E_{ij}$ 关联一个3维one-hot向量 $z_{ij} \in \{0,1\}^3$，三entry分别对应void、skeleton、actuator三种状态。

为了使离散变量可微，作者用**scaled softmax** relaxation：

$$\tilde{z}_{ij,k} = \frac{\exp(\beta \cdot z_{ij,k})}{\sum_{l=1}^{3} \exp(\beta \cdot z_{ij,l})}$$

- $z_{ij,k}$：连续relax后的design variable（优化前 $\in [0,1]^3$，非one-hot）
- $\beta$：temperature/scaling factor，控制projection的sharpness。$\beta$ 越大，$\tilde{z}$ 越接近one-hot。论文中 $\beta = 20.0$。
- $\tilde{z}_{ij,k}$：归一化后的material state ratio，用于physics simulation

每种material state对应3个物理属性 $H=3$：stiffness $k$、linear density $\rho$、max actuation strain $a_{\max}$。effective property通过加权得到：

$$p_{ij} = \Psi^\top \tilde{z}_{ij}$$

其中 $\Psi \in \mathbb{R}^{N_e \times 3}$ 是material property library matrix。

| Material State | $k$ (N/m) | $\rho$ (g/m) | $a_{\max}$ |
|---|---|---|---|
| Void (V) | $1\times10^{-7}$ | $1\times10^{-5}$ | 0.0 |
| Skeleton (S) | $4\times10^{2}$ | 30.0 | 0.0 |
| Actuator (A) | $3\times10^{1}$ | 100.0 | 0.35 |

这里有一个关键的**physical trade-off**：skeleton刚但不能actuate，actuator能actuate但compliant且重（density是skeleton的3.3倍）。这mirror了现实中electromechanical component的重量惩罚。

Node mass通过lumped mass model计算：

$$m_i = m_\epsilon + \frac{1}{2}\sum_{j \in \mathcal{N}(i)} l_{ij} \sum_{k=1}^{3} \tilde{z}_{ij,k} \rho_k$$

- $m_\epsilon$：极小的baseline mass，防止singular matrix
- $l_{ij}$：edge的resting length
- $\rho_k$：material state $k$ 的linear density

**控制策略**：用一个single-hidden-layer MLP作为controller。输入包括：

$$I(t) = [C(t)^\top, x_{\text{goal}}, \Delta x_1(t)^\top, \ldots, \Delta x_{N_m}(t)^\top, v_1(t)^\top, \ldots, v_{N_m}(t)^\top]^\top$$

其中 $C(t)$ 是Central Pattern Generator (CPG)信号：

$$C_j(t) = \sin\left(\omega t + \frac{2\pi j}{N_{\text{CPG}}}\right), \quad j \in \{0, \ldots, N_{\text{CPG}}-1\}$$

- $\omega$：fundamental frequency（$\omega = 10$）
- $N_{\text{CPG}} = 10$：CPG signal数量（更多会导致desynchronization）
- $\Delta x_i(t)$：node $i$ 相对于robot几何中心的位移，提供spatial awareness
- $x_{\text{goal}} \in \mathbb{R}^2$：goal position，enabling steering

MLP hidden layer dim = 32，total架构 $(156, 32, 110)$。输出经tanh激活，得到 $u_{ij}(t) \in [-1, 1]$。

关键设计：control signal被material state modulate：

$$\epsilon_{\text{target},ij}(t) = \tilde{z}_{ij,3} \cdot a_{\max} \cdot u_{ij}(t)$$

- $\tilde{z}_{ij,3}$：actuator state的ratio。只有actuator edge才有非零target strain
- 这实现了**structure-material-control的耦合**：MLP对所有edge输出信号，但只有actuator edge物理响应

#### 2.2 Differentiable Simulation

仿真基于Taichi平台，使用symplectic Euler integration：

$$v_i(t+\Delta t) = \exp(-\gamma \Delta t) v_i(t) + \frac{1}{m_i}\left(\sum_{j \in \mathcal{N}(i)} J_{j \to i}(t) + g\Delta t\right)$$

$$x_i(t+\Delta t) = x_i(t) + v_i(t+\Delta t) \Delta t$$

- $\gamma$：damping coefficient（模拟atmospheric drag和internal energy dissipation）
- $J_{j \to i}(t) = F_{ij}(t) \cdot \Delta t \cdot \hat{u}_{ij}(t)$：nodal impulse，其中 $\hat{u}_{ij}(t) = \frac{x_j(t) - x_i(t)}{l_{ij}(t)}$ 是unit direction vector

Truss force遵循generalized Hooke's law：

$$F_{ij}(t) = k_{ij}[l_{ij}(t) - l_{\text{target},ij}(t)]$$

$$l_{\text{target},ij}(t) = l_{0,ij}[1 + \epsilon_{\text{target},ij}(t)]$$

**Ground contact**使用Time of Impact (TOI) scheme来减少gradient discontinuity。碰撞后速度修正：

- Normal方向（y）：$v^+_{i,y}(t) = 0$（perfectly inelastic）
- Tangential方向（x）：infinite friction时 $v^+_{i,x} = 0$；Coulomb friction时 $v^+_{i,x} = v^*_{i,x} - \mu|v^*_{i,y}|\text{sgn}(v^*_{i,x})$

整个forward simulation构成一个computational graph，automatic differentiation可以backpropagate gradient从final displacement到design variables $Z$ 和controller weights $\theta$。

**仿真参数**：$\Delta t = 0.002$ s，$T = 8192$ timesteps（约16.4秒rollout），GPU-parallel。

#### 2.3 Constrained Optimization

**Objective**：

$$\min_{Z, \theta} \mathcal{L}_{\text{disp}}(Z, \theta) = -x_0(T)$$

$x_0(T)$ 是payload head node在final time的水平位移。

**Constraints**：

1. **Volume fraction**：

$$V = \frac{1}{N_e}\sum_{(i,j) \in \mathcal{E}} (p_{ij,2} + p_{ij,3}), \quad V_{\text{act}} = \frac{1}{N_e}\sum_{(i,j) \in \mathcal{E}} p_{ij,3}$$

bounds: $V \in [0.50, 0.50]$（严格50%），$V_{\text{act}} \in [0.20, 0.22]$。Volume bound tolerance $\Delta V$ 从0线性增加到0.03（iteration 110-150），允许intermediate states有更多mobility。

2. **Binarization constraint** — 两部分：

**Global L2 binarization penalty**（驱动 $\tilde{z}$ 的L2 norm趋向1）：

$$g_{\text{bin}} = \frac{1}{N_e}\sum_{(i,j) \in \mathcal{E}} \left(1 - \sqrt{\sum_{k=1}^{3} \tilde{z}^2_{ij,k}}\right)$$

**Pairwise orthogonal penalty**（减少任意两种material之间的overlap）：

$$g_{\text{ortho}} = \frac{1}{N_e}\sum_{(i,j) \in \mathcal{E}} \sum_{k \neq l} \frac{\tilde{z}_{ij,k} \tilde{z}_{ij,l}}{0.5(\tilde{z}_{ij,k} + \tilde{z}_{ij,l}) + \delta}$$

- $\delta$：小常数避免singular

**Augmented Lagrangian**处理binarization constraints（允许偶尔violate以增加exploration）：

$$\mathcal{L}(Z, \lambda, \tau) = \mathcal{L}_{\text{disp}} + \sum_{k=1}^{4}\left(-\lambda_k v_k + \frac{1}{2}\tau_k v_k^2\right)$$

- $v_k$：第 $k$ 个binarization constraint的violation magnitude
- $\lambda_k$：Lagrangian multiplier，动态更新 $\lambda_k \leftarrow \lambda_k - \tau_k v_k$
- $\tau_k$：penalty weight，violation增加时乘以 $a=1.01$，减少时除以 $a$

**关键trick — Stability Pass**：有些edge卡在intermediate state，这些intermediate material可能maintain了一个脆弱的structural balance。一旦discretize，balance被破坏，performance急剧下降。为解决此问题，作者用**globally centered softmax**生成一个proxy geometry：

$$\bar{z}_k = \frac{1}{N_e}\sum_{(i,j) \in \mathcal{E}} z_{ij,k}$$

$$z^*_{ij,k} = \frac{\exp(\beta_{\text{stab}}(z_{ij,k} - \bar{z}_k))}{\sum_{l=1}^{3}\exp(\beta_{\text{stab}}(z_{ij,l} - \bar{z}_l))}$$

- $\beta_{\text{stab}} = 500 \gg \beta = 20$：极度sharpened proxy
- $\bar{z}_k$：global mean，作为centering reference
- 用Straight-Through Estimator (STE)来缓解vanishing gradient

这个stability pass和performance pass交替进行，gradient updates按预定比例blend。

**Multi-stage optimization** (Algorithm 1)：

| 阶段 | Iteration | 操作 |
|---|---|---|
| Co-design | 1-239 | 交替Controller/Performance/Stability pass，$Z$用MMA更新，$\theta$用Adam更新，比例3:2 |
| Iterative Attraction | 170+ | 对confident edge施加nudge $\gamma$，pull向dominant state |
| Hard Snap | 240 | $Z$ snap到one-hot，freeze |
| Control Fine-tune | 240-300 | 只更新 $\theta$ |

**为什么 $Z$ 用MMA而 $\theta$ 用Adam**：MMA (Method of Moving Asymptotes) 是topology optimization中经典的constrained optimizer，擅长处理少量design variable + 约束；Adam擅长large-scale neural network parameter optimization。

---

### 3. 实验结果

#### 3.1 Co-design vs. Sequential Design

| Design | Initialization | Displacement | Gait |
|---|---|---|---|
| 6a (Baseline) | 手工3-leg + vertical actuator | 1.14 m | Alternating gait (不稳定) |
| 6b (Co-design) | Stability heuristic | **1.90 m** (+66%) | Propulsive leaping |
| 6c (Co-design) | Three-legged heuristic | 1.87 m | Sweeping motion |
| 6d (Co-design) | No heuristic (uniform) | 0.98 m | Hopping |

Baseline的限制在于passive cross-bracing限制了limb stroke，导致inconsistent ground contact。Co-design自动发现了一个massive back leg + small front leg的morphology，用leaping gait获得更大stride。

**有趣发现**：Design 6d（完全无heuristic）虽然performance最低，但自动discover了与6b相似的结构特征（low center of mass），类似于自然界中的convergent evolution。

#### 3.2 Ablation Study — 个体与集体贡献

$2^3 = 8$种组合（enable/disable topology, material, control）：

| Entity | Average "with" vs "without" improvement |
|---|---|
| **Material (actuation layout)** | **+781%** — 最大贡献 |
| Topology | +116% |
| Control | 最小，偶尔negative |

关键发现：

1. **Actuation layout最重要**： redistribute actuator的位置可以reshaped整个robot dynamics，即使structure和control suboptimal也能有decent performance。
2. **Control单独贡献小**：当structure/material suboptimal时，control的gradient signal被ground contact的非线性污染，Adam optimizer收到的directional information不可靠。但当与material co-evolve时，control的作用被放大——Trial 7g→7h，加入control后displacement增加87%（1.02m→1.91m）。
3. **Topology是multiplier**：topology单独weak，但它放大了material design的效果。Topology和material之间有强耦合——发现optimized topology而没有compatible actuation layout（Trial 7d vs h），或者反过来（Trial 7f vs h），都远不如jointly designed。
4. **Convergent evolution**：Trial 7g和7h独立discover了相同的structural motif——在passive cross-brace之间插入horizontal actuator来协调前后腿。这mirror了自然界中相似body plan的物种演化出相同musculoskeletal motif。

#### 3.3 Generalization

| Scenario | Result |
|---|---|
| 15° incline (9a) | 1.36 m horizontal + 0.35 m vertical，arching gait |
| 5×5 grid (9b) | 1.10 m，rediscovered leg division strategy |
| Coulomb friction μ=2.5 (9c) | 保持payload贴地作为anchorage，front leg折叠作为friction pad |

每个scenario都evolve出了**不同的morphology和gait**，证明framework能适应不同physical environment和design space。

---

### 4. 我的评价

**Strengths**：
- 把discrete truss robot的mixed-variable co-design问题用continuous relaxation + binarization constraint巧妙地转化为gradient-based optimization可处理的形式
- Stability pass是一个很重要的工程insight：intermediate design的performance不反映discrete design的true performance
- Ablation study提供了有价值的design insight：material > topology > control的hierarchy

**Limitations**：
- 仍然是2D simulation，没有3D
- Simplified physics（只有truss axial force，没有bending、torsion、material nonlinearity）
- 对initialization敏感（nonconvex问题的通病）
- Material library只有3种state，扩展到更多material type时binary relaxation的维度会增加，softmax的sharpness control会更复杂

**与相关工作的关系**：
- 继承了DiffTaichi [35] 的differentiable simulation框架
- 与SoftZoo [30]、ChainQueen [32] 的区别：那些focus on continuum soft robot，COSMIC是第一个对discrete truss-lattice做full co-design的
- 与Kriegman et al. [19, 21] 的evolutionary approach互补：gradient-based需要好的initialization，evolutionary擅长exploration但sample efficiency低

**Reference links**:
- Taichi (DiffTaichi): https://github.com/taichi-dev/taichi
- SoftZoo: https://arxiv.org/abs/2303.09555
- Kriegman accelerated co-design: https://arxiv.org/abs/2502.10862
- MMA optimizer (Svanberg 1987): https://epubs.siam.org/doi/10.1137/0802043
