软体机器人进化沙盒: 同时设计机器人的身体和大脑，然后看哪种算法能把机器人进化得最厉害。造一个机器人，传统做法是：人先画好身体结构再去学怎么控制它。之前没人搭一个统一的测试平台。每个研究组自己搞自己的环境，没法比较谁的方法好。所以他们就做了Evolution Gym。机器人是用一块块小方块（voxel）拼出来的，就有点像Minecraft里搭东西。每块方块可以是5种材质之一：
- **Empty**：空的，啥都没有
- **Rigid**：硬的，像骨头
- **Soft**：软的，像肌肉
- **Horizontal Actuator**：横向肌肉，能左右伸缩
- **Vertical Actuator**：竖向肌肉，能上下伸缩

把这些方块拼在网格上，就是一个机器人。然后物理引擎用弹簧模型模拟它的运动——每条边就是一根弹簧，软的弹簧常数小，硬的大。机器人在每个时刻能看到自己的形状、速度、周围地形，然后输出一个action告诉每个肌肉"你该缩多紧或伸多长"，就这样往前跑或者搬东西。两层循环，外层进化身体，内层进化大脑：
1. **外层**先随机生成一批机器人身体（比如100个）
2. 对每个身体，**内层**用PPO（一种强化学习算法）训练一个神经网络来控制它——"教这个身体怎么动"
3. 看每个身体能拿多少分
4. **外层**根据成绩，保留好的、淘汰差的、加点随机变异，生成下一代身体
5. 回到第2步，重复几十代

外层试了三种策略：
- **GA（遗传算法）**：简单粗暴——留下成绩好的，剩下的随机变异，跟自然选择很像
- **BO（贝叶斯优化）**：用一个数学模型猜哪块区域可能有好结果，然后去试
- **CPPN-NEAT**：用一个神经网络来"画"机器人的身体结构，然后进化这个画图网络

30多个任务，从简单到难：
- **简单的**：平地上走路、过软桥、接住天上掉下来的方块然后搬运
- **中等的**：爬楼梯、攀岩壁、把方块扔远
- **困难的**：穿越坑洞、把一根梁推着滑、接住旋转下落的物体、从洞里把东西举出来

几个有意思的发现：

**1. 最简单的遗传算法反而最强。** 这很反直觉——没有花哨的设计，就是"好的留下+随机变异"，结果整体表现最好。说明这个design space可能更适合这种朴素的local search。

**2. 进化出了像动物的形态。** 比如Carrier任务里，机器人自己进化出了"腿+持物结构"，有点像一只小动物抱着东西在跑。Climber任务里进化出了腿状结构+平坦的背（贴墙增加摩擦力）+身体中间一个洞（帮助走路的运动模式）。这些都不是人能想出来的。

**3. 进化出来的机器人打败了人手设计的。** 在所有测试任务上，算法优化的机器人都不比人手动设计的差，大部分还更好。说明人直觉设计robot身体其实不太靠谱。

**4. 但最难的几个任务全部失败。** 不管用哪种算法，那些hard task没有一个能做成的。这说明现在的算法还远远不够。

搭了个标准化的测试场，大家以后可以在这个平台上公平比较谁的co-design算法更牛，不用各自为战。用现有的最好算法跑了一遍，发现简单任务能做，中等任务勉强，难任务全跪。这就相当于立了个flag——"这些硬骨头谁能啃下来，谁就牛"。


---

## Evolution Gym: A Large-Scale Benchmark for Evolving Soft Robots

**论文链接**: [NeurIPS 2021 Proceedings](https://papers.nips.cc/paper/2021/hash/e8ed0958a0f0d22d6c7a9c84f4ad582e-Abstract.html) | **项目主页**: [http://evogym.csail.mit.edu](http://evogym.csail.mit.edu) | **代码**: [GitHub](https://github.com/EvolutionGym/evogym)

---

### 1. 核心问题：为什么需要这个Benchmark？

这篇paper解决的核心问题是 **robot co-design**——即同时优化robot的**body（物理结构）**和**brain（控制器）**。

从第一性原理来看，一个robot的task performance取决于两个factor：

$$\text{Performance} = f(\text{Design } D, \text{Controller } C)$$

传统robotics研究大多固定 $D$，只优化 $C$（即control/RL问题）。但biological evolution告诉我们，body和brain是**co-evolved**的——鸟的翅膀和飞行神经回路是一起进化出来的。之前之所以没人做co-design benchmark，原因有二：

1. **Bilevel optimization的复杂度**：外层优化design、内层优化control，iteration cycle极长
2. **没有统一benchmark**：每个group自己搭environment，无法公平比较

Evolution Gym就是来填这个gap的。

---

### 2. 技术架构

#### 2.1 Voxel-based Representation（体素表示）

每个robot被表示为一个2D grid上的**material matrix** $M$，每个voxel entry $m \in M$ 从以下集合中取值：

$$m \in \{\text{Empty}, \text{Rigid}, \text{Soft}, \text{Horizontal Actuator}, \text{Vertical Actuator}\}$$

另有一个connection link list $C$ 存储相邻voxel之间的连接关系。

这个设计的关键insight在于：**combinatorial expressiveness**。即使每个robot只有<100个voxel，5种material type的组合空间已经是 $5^{100} \approx 10^{70}$ 量级，远超任何搜索算法能穷举的范围。同时，voxel-based representation天然适配mass-spring simulation，计算效率高。

#### 2.2 Simulation Engine（物理引擎）

模拟器采用 **2D mass-spring system**（参考 Nealen et al. [26]）。具体来说：

- 每个voxel初始化为一个**cross-braced square**（带对角线支撑的正方形），4个corner points作为mass node
- 每条edge是一个理想spring，遵循 **Hooke's law**：

$$F_{spring} = -k \cdot (|\mathbf{x}_{ij}| - L_0) \cdot \hat{\mathbf{x}}_{ij}$$

其中 $k$ 是spring constant（由material type决定，共5种），$|\mathbf{x}_{ij}|$ 是当前长度，$L_0$ 是rest length，$\hat{\mathbf{x}}_{ij}$ 是方向单位向量。

- 积分方法用 **Symplectic RK-4**（辛积分器，保守能量不发散）
- Collision detection用**bounding-box tree**（参考 Ericson [12]），contact force按penetration depth成比例计算penalty-based force + frictional force

整个引擎用C++写成，通过Python binding暴露API，接口设计模仿OpenAI Gym。

#### 2.3 Observation / Action / Reward

**Observation**（每个time step）：

$$\mathbf{o} = [\underbrace{\Delta\mathbf{p}_1, \Delta\mathbf{p}_2, \ldots, \Delta\mathbf{p}_N}_{\text{2N-D: 相对质心位置}}, \underbrace{\mathbf{v}_{com}, \theta}_{\text{3-D: 质心速度和朝向}}, \underbrace{\text{terrain}_{1..2W}}_{\text{地形高程}}, \underbrace{\text{goal info}}_{\text{task-specific}}]$$

其中 $N$ 是robot的voxel corner point总数，$\Delta\mathbf{p}_i$ 是第 $i$ 个corner point相对于质心的位置，$W$ 是terrain observation的half-window大小。

**Action**：每个actuator voxel对应一个action分量 $u \in [0.6, 1.6]$，表示该actuator目标变形到 $u$ 倍rest length。$u > 1$ 表示膨胀，$u < 1$ 表示收缩。

---

### 3. Co-Design Algorithm Framework

核心是一个**bilevel optimization**（Algorithm 1）：

```
外层循环 (Design Optimization):
  for generation i = 1 to n:
    for each design Dj in population:
      内层循环 (Control Optimization):
        Cj ← PPO(T, Dj)    # 给定design，用RL训练controller
      rj ← Evaluate(T, Dj, Cj)
    下一代designs ← OPTIMIZE_DESIGNS(S, p)  # 用design optimizer更新种群
```

外层和内层分别测试了多种方法：

#### 3.1 Design Optimization（外层）— 三种方法

| 方法 | 核心思想 | 关键细节 |
|------|---------|---------|
| **GA** (Genetic Algorithm) | Elitism selection + mutation | 保留top $x$% survivors，$x$ 从60%线性衰减到0；每个voxel以10%概率mutation（包括变Empty以改变topology）；无crossover |
| **BO** (Bayesian Optimization) | GP surrogate + Thompson sampling | Batch BO，batch size = population size；用GPyOpt库；L-BFGS优化acquisition function |
| **CPPN-NEAT** | 用CPPN生成design pattern | CPPN输入是voxel坐标 $(x,y)$，输出是voxel type；NEAT算法进化CPPN的网络结构和权重 |

**CPPN-NEAT**特别值得一提：CPPN（Compositional Pattern Producing Network）是Kenneth Stanley提出的，本质是一个神经网络 $g_\theta(x, y) \rightarrow \text{voxel type}$，其中 $\theta$ 通过NEAT（NeuroEvolution of Augmenting Topologies）进化——即同时改变网络拓扑和权重。CPPN能产生具有symmetry、repetition、modularity等自然图案特征的结构，因为网络中的activation function（如sin, cos, gaussian）天然encode了这些pattern。

#### 3.2 Control Optimization（内层）

统一使用 **PPO**（Proximal Policy Optimization）[Schulman et al. 2017]。PPO的clipped surrogate objective：

$$L^{CLIP}(\theta) = \mathbb{E}_t \left[ \min(r_t(\theta)\hat{A}_t, \; \text{clip}(r_t(\theta), 1-\epsilon, 1+\epsilon)\hat{A}_t) \right]$$

其中 $r_t(\theta) = \frac{\pi_\theta(a_t|s_t)}{\pi_{\theta_{old}}(a_t|s_t)}$ 是importance sampling ratio，$\hat{A}_t$ 是advantage estimate，$\epsilon$ 是clip范围（通常0.2）。

选择PPO而非之前工作常用的open-loop periodic actuation的原因：**periodic control无法处理非周期任务**（如uneven terrain walking, object catching）。

---

### 4. Benchmark Tasks

共30+个task，选取10个展示，分3个difficulty level：

| 难度 | Locomotion | Manipulation |
|------|-----------|--------------|
| **Easy** | Walker（平地行走）, Bridge Walker（软桥行走） | Carrier（接住+搬运物体） |
| **Medium** | Climber（攀爬两壁）, Up Stepper（爬阶梯） | Thrower（抛掷物体） |
| **Hard** | Traverser（穿越rigid block坑） | Beam Slider（滑梁）, Catcher（接旋转下落物）, Lifter（从洞中提升物体） |

---

### 5. 实验结果

#### 5.1 算法对比（Figure 3）

核心发现：

1. **GA整体最强**——这很反直觉，因为GA用了最简单的operator（仅mutation+elitism selection，无crossover）。这说明design space的结构可能更适合local search式的探索，而非CPPN-NEAT的pattern-biased生成或BO的全局surrogate建模。

2. **CPPN-NEAT在locomotion任务上表现不错**，但在manipulation任务上差。原因是NEAT偏好简单结构的CPPN → 生成regular pattern的robot → locomotion够用但manipulation需要irregular的agile substructure（如gripper-like shape）。

3. **BO在大多数任务上表现差**——因为design space是高维categorical space + RL evaluation有noise → GP surrogate model拟合不准。这是BO在high-dimensional categorical + noisy setting下的已知局限。

4. **所有algorithm都无法解决最难的task**（Traverser, Beam Slider, Catcher, Lifter）——这是paper最重要的结论之一，说明现有algorithm远远不够。

#### 5.2 Evolution Analysis（Figure 4）

以Carrier任务为例，evolution过程展现了emergent complexity：
- Gen 1（随机初始化）：偶尔出现有"持物结构"和"腿"的robot，reward ~4.2
- Gen 10：这些结构被select并refine，reward ~8.4
- Gen 20：robot既有高效的holding mechanism又有快速locomotion的腿，reward ~10.6

Lifter任务中，algorithm自主进化出了**parallel gripper-like shape**——这种结构在Gen 1的随机种群中并不明显，是evolution逐步discover的。

#### 5.3 vs Hand-Designed Robots（Figure 5）

**在所有测试task上，algorithm-optimized robot都优于或至少持平hand-designed robot。**

Climber任务最有说服力：人工设计无法找到"wall traction vs upward acceleration"的trade-off，而GA进化出了leg-like structure + flat back（最大化wall friction）+ body中心的hole（优化walking motion）。这种组合不是人类直觉能想到的。

---

### 6. 我的批判性分析

**优点**：
- 填补了co-design benchmark空白，30+ tasks的scale在soft robotics领域前所未有
- Voxel-based representation简洁但expressive，simulation效率高（C++ backend，分钟级训练）
- Open-source + MIT license + OpenAI Gym-compatible API，降低使用门槛

**局限**：
1. **2D simulation only**——mass-spring 2D system无法捕捉3D soft robot的真实物理（如torsion, out-of-plane bending）。这是最大的limitation。
2. **Bilevel optimization框架本身可能是次优的**——外层design optimization的每次evaluation需要完整run一遍PPO内层loop，计算开销巨大。Paper提到未来方向包括concurrent co-optimization，但baseline都用了sequential bilevel scheme。
3. **Design space是categorical + combinatorial**，这对gradient-based method不友好。Paper没有尝试differentiable design optimization（如PlasticineLab [16]的可微物理思路）。
4. **所有hardest task都失败**——这既是motivation也是当前framework的fundamental limitation。可能需要fundamentally不同的paradigm（如morphogenetic development, decentralized control）。
5. **No sim-to-real transfer**——paper完全在simulation中，没有验证evolved robot能否在physical world工作。Soft robot的sim-to-real gap比rigid robot更严重，因为soft material的非线性行为难以精确模拟。

**与相关工作的定位**：
- 相对于Sims (1994) [31]的rigid creature evolution：Evolution Gym引入了soft material，极大扩展了design space
- 相对于Cheney et al. [7,8]的voxel-based soft robot evolution：Evolution Gym把control从open-loop periodic升级为neural network policy（PPO），能处理非周期复杂任务
- 相对于PlasticineLab [16]：PlasticineLab是control-only benchmark for soft body manipulation，Evolution Gym是co-design benchmark

---

### 7. 关键Takeaway

从第一性原理看，这篇paper的深层insight是：**robot的intelligence不只存在于controller（brain）中，同样存在于morphology（body）中**。这就是embodied intelligence的核心thesis——body本身就是computation的一部分。Evolution Gym提供了一个系统化的testbed来验证这个thesis：当你同时优化body和brain时，emergent behavior远超只优化brain。

但paper也诚实地揭示了当前algorithm的不足——在hardest task上全面失败。这暗示design space的high-dimensionality和combinatorial nature仍然是open challenge，需要全新的algorithm paradigm。

**参考链接**：
- [Evolution Gym 项目主页](http://evogym.csail.mit.edu)
- [GitHub 代码库](https://github.com/EvolutionGym/evogym)
- [NeurIPS 2021 论文](https://papers.nips.cc/paper/2021/hash/e8ed0958a0f0d22d6c7a9c84f4ad582e-Abstract.html)
- [PPO 原始论文 (Schulman et al. 2017)](https://arxiv.org/abs/1707.06347)
- [CPPN-NEAT (Stanley 2007)](https://link.springer.com/article/10.1007/s10710-007-9028-8)
- [PlasticineLab (Huang et al. 2021)](https://arxiv.org/abs/2104.03311)

