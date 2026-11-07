让电脑自己造出会动的生物，而不是让人一个一个去设计。做一个会游泳的虚拟鱼，传统做法是你得亲手设计鱼的身体形状、鱼鳍位置、然后写一套控制代码告诉它怎么摆尾巴。换一个会走路的？从头再来一遍。整个过程就四步，不断循环：

第一步：随机造一批生物: 每个生物的"基因"是一个**有向图**——就是一堆圆圈（node）和箭头（connection）连成的网。
- 外层图描述**身体长什么样**：每个圆圈代表一个零件（比如一段身体、一条腿），箭头代表"这个零件挂在哪个零件上"
- 每个零件的圆圈里面又嵌着一张小图，描述**这个零件的局部神经回路**

这个图可以是循环的，比如 A 指向 B，B 又指回 A。这样就能用很少的基因描述重复结构——就像你写一个 for 循环，执行三次就长出三节身体，不用写三遍。

第二步：把基因"发育"成真家伙: 从图的起点开始走，走到哪个圆圈就生成一个 3D 零件，走到循环就重复生成。最后得到一个由刚性方块组成、有关节连接的 3D 身体，外加一套连好的神经网络。这套神经网络不是传统那种"全是加权和再过 sigmoid"的统一结构。Sims 给了 22 种**不同功能**的神经元，比如：
- `oscillate-wave`：给个常量输入，它就自己振荡起来——天生就是个节拍器
- `sin`、`cos`：三角函数
- `integrate`：积分，能记住历史
- `memory`：记忆

这个设计的精妙之处在于：要做出游泳这种周期性运动，你不需要进化出复杂的反馈回路，直接扔一个振荡器神经元进去就行。降低了搜索难度。

第三步：放到物理仿真世界里跑: 在 3D 物理引擎里跑 10 秒左右，看它表现怎么样。物理引擎用 Featherstone 的方法算刚体动力学，用 Runge-Kutta 积分，处理碰撞、摩擦、水的粘性阻力。该有的都有。然后根据不同的fitness 函数打分:
- **游泳**：跑 10 秒，看质心跑了多远，越远越好。水的环境是关掉重力、加上粘性阻力
- **走路**：放在有地面有重力的环境里，看水平移动速度
- **跳远**：看最低点离地多高
- **追光**：放一个光源在不同位置，看它能不能朝光源移动，平均速度越高越好

第四步：选好的，生孩子: 300 个生物，跑完打分，**前 20% 存活**，其余淘汰。存活的有资格繁殖，后代数量跟 fitness 成正比——越成功的生越多。繁殖方式三种：
1. **无性繁殖（40%）**：直接复制，加点随机突变
2. **交叉（30%）**：两个 parent 的基因各切一段拼一起，像基因重组
3. **嫁接（30%）**：把 parent 1 的某个零件整个换成 parent 2 的一坨。这个最狠——等于把另一个生物的一整条腿连带着它的神经控制器直接移植过来

突变就是随机改参数、加节点、加连线、删连线。改完之后从根节点开始检查连通性，连不上的垃圾回收掉。然后回到第一步，下一代继续跑。跑 100 代，大概 3 小时（1994 年的 32 处理器 CM-5 超算）。
结果跑出来一堆千奇百怪但确实能动的生物：
- **游泳的**：有的四条 flipper 交替划水，有的像水蛇一样扭身体，有的用专门的桨状附肢拉着自己走
- **走路的**：有的像蜥蜴一样用方块边角着地爬，有的像尺蠖一样伸缩前进，有的原地晃一个附肢把自己往前"摇"出去
- **跳的**：策略少一些，但确实有能蹦的
- **追光的**：有的长出了转向鳍，根据光感器信号调整方向

还有一个很酷的实验：把水里进化出的水蛇**搬到陆地上继续进化**，它居然学会了在地上爬。说明进化出来的神经控制器有一定泛化能力。

---

## 最有意思的一个细节

Sims 在论文里说了句大实话：

> 物理仿真里任何 bug——哪怕只是浮点数 round-off error 导致的能量不守恒——进化都会发现并利用它。

也就是说，如果你的物理引擎不完美（当然不可能完美），creature 会学会**从 bug 里白嫖能量**。比如如果碰撞响应给了一点多余的能量，creature 就会学会疯狂碰撞来加速。

这其实是进化算法的一个本质属性：**它在最 adversarial 的意义上优化你的 fitness 函数**。你以为你在优化"游得快"，实际它在优化"在你这个有 bug 的仿真器里游得快"。

---

## 为什么这篇论文牛

1994 年的论文，到现在还被引用，原因是它做了一个**同时进化身体和大脑**的完整 demo。之前的类似工作要么只进化控制器（身体是人设计的），要么只在 2D 里做。Sims 是第一个在 3D 物理仿真里让**形态和神经控制共同进化**的。

核心 insight 其实就一个：**用有向图当基因**，因为图可以循环（复用结构）、可以增长（加节点加连线）、可以嵌套（身体图里套神经图）。这个 representation 让进化可以从小到大、从简到繁地探索，而不是在一个固定大小的空间里转。

用一句话总结：**他证明了，给定一个好的基因表示方式和靠谱的物理仿真，进化能自动造出人类想不到但也造不出来的运动方式。**


---

让我先读取这篇论文的完整内容，以便全面了解其技术细节。


好的，我已经完整阅读了这篇论文。下面是详细的技术讲解。

---

## 论文概述

这是 **Karl Sims** 发表在 **SIGGRAPH 1994** 的经典论文 **"Evolving Virtual Creatures"**。核心思想是：用 genetic algorithm 同时进化虚拟生物的 **morphology（形态结构）** 和 **neural control system（神经控制系统）**，让它们在 3D 物理仿真世界中自主学会 swimming、walking、jumping、following light 等行为。

**参考链接：**
- ACM Digital Library: https://dl.acm.org/doi/10.1145/192161.192293
- Karl Sims 个人网站: https://www.karlsims.com/evolved-virtual-creatures.html
- 视频演示: https://www.youtube.com/watch?v=JBgG_lSP8vQ

---

## 1. 核心架构：Genotype → Phenotype → Fitness → Selection

整个系统遵循 Darwinian evolution 的闭环：

```
Genotype (directed graph) 
  → Development (synthesis) 
    → Phenotype (3D rigid body hierarchy + neural circuitry) 
      → Physical Simulation (in virtual world) 
        → Fitness Evaluation 
          → Selection & Reproduction (mutation + mating) 
            → New Genotypes ...
```

### 1.1 Genotype：Directed Graph（有向图）

**第一性原理思考：** 为什么用 directed graph 而不是固定长度的 bit-string？

固定长度的 genotype 定义的是固定维度的搜索空间——你永远搜不到结构上全新的解。Sims 需要的是一种**可以增长复杂度的语言**（hyperspace），directed graph 天然支持：
- **递归/循环**：node 可以指向自己或形成 cycle，产生 fractal-like 重复结构（类似 L-system）
- **复用**：同一个 node 可以被多个 parent 指向，生成对称的重复 appendage
- **可变长度**：mutation 可以添加/删除 node 和 connection

### 1.2 Morphology Graph（外层图）

每个 **node** 描述一个 rigid part，包含：
- **Dimensions**: 矩形体的长宽高，决定物理形状
- **Joint-type**: 与 parent 的连接方式，决定了自由度 (DOF) 数量和约束：
  - `rigid`（0 DOF）, `revolute`（1 DOF 旋转）, `twist`, `universal`（2 DOF）, `bend-twist`, `twist-bend`, `spherical`（3 DOF）
- **Joint-limits**: 超出限制后产生 restoring spring force
- **Recursive-limit**: 在递归 cycle 中该 node 生成 phenotype part 的次数
- **Local neurons**: 嵌套的 neural graph（见下文）

每个 **connection** 描述 child part 相对于 parent 的空间关系，分解为：
- **Position**: 限制在 parent part 的表面上
- **Orientation**: 旋转角度
- **Scale**: 缩放
- **Reflection**: 负缩放，产生镜像对称子树
- **Terminal-only flag**: 只在递归到底时生效，用于生成末端结构（如手、尾）

**关键设计直觉**：position、orientation、scale、reflection 分离编码，使得每种变换可以被独立 mutation。这大大降低了搜索空间的耦合度——改变一个 limb 的对称性不需要重新搜索它的位置。

### 1.3 Neural Graph（内层嵌套图）

每个 morphological node 内部嵌套一个 neural graph。这是一种 **nested directed graph** 结构。

**Neuron functions**（22 种）：
```
sum, product, divide, sum-threshold, greater-than, sign-of, 
min, max, abs, if, interpolate, sin, cos, atan, log, expt, 
sigmoid, integrate, differentiate, smooth, memory, oscillate-wave, oscillate-saw
```

**第一性原理分析**：Sims 选择了一个 **dataflow programming** 模型而非传统 neural network（sum + threshold）。原因在于：

传统 ANN 的 node function 是统一的（如 sigmoid(w·x+b)），要表达振荡行为需要通过 weight 调整实现 recurrent dynamics，搜索效率低。Sims 直接提供了 `oscillate-wave`、`oscillate-saw` 这样的 **带状态的振荡器 neuron**——给定常量输入就能产生周期性输出。这把"生成节律性运动"这个关键子问题从"搜索 recurrent weights"简化为"搜索一个 oscillator 的参数"，极大地降低了进化难度。

每个 neuron 最多 3 个输入，每个输入有：
- **Source**: 指向另一个 neuron、sensor，或 constant value
- **Weight**: 缩放因子

每个 effector 接收一个 neuron 输出，乘以 weight，作为 **joint force/torque** 施加到对应 DOF。

### 1.4 Sensors（3 种）

| Sensor | 功能 | 信号 |
|--------|------|------|
| **Joint angle** | 每个 DOF 的当前角度值 | 连续标量 |
| **Contact** | 每个 part 的每个面是否有碰撞 | +1.0（有接触）/ -1.0（无接触） |
| **Photosensor** | 光源相对于 part 朝向的归一化方向 | 3 个标量 |

**Effector strength scaling**：effector 最大力 ∝ 连接两个 part 的最大横截面积。这是一个关键的 **物理约束**：

$$F_{\max} \propto A = \min(w_1 \cdot h_1, w_2 \cdot h_2)$$

其中 $w_i, h_i$ 是 part $i$ 的宽和高。而质量 $m \propto V = w \cdot h \cdot l$（体积）。所以 $F_{\max}/m \propto A/V \propto 1/l$——越大的 creature 单位质量可施加的力越小，正如自然界中 square-cube law 所规定。这防止了 creature 无限增大来获得优势。

---

## 2. Physical Simulation

| 组件 | 方法 |
|------|------|
| **Articulated body dynamics** | Featherstone's recursive O(N) algorithm [3] |
| **Numerical integration** | Runge-Kutta-Fehlberg (4th order RK + error estimation for adaptive step size) |
| **Time step** | 每帧 1/30s，每帧 1-5 个 integration steps；每个 dynamics step 执行 2 个 brain time steps |
| **Collision detection** | Bounding box hierarchy，将 O(N²) 降到可管理范围 |
| **Collision response** | Hybrid: 高速时用 impulse，低速时用 penalty spring |
| **Fluid viscosity** | 对每个暴露运动表面，viscous force 正比于表面积 × 法向速度 |

**Sims 的关键观察（也是我特别喜欢的段落）**：

> "Any bugs that allow energy leaks from non-conservation, or even round-off errors, will inevitably be discovered and exploited by the evolving creatures."

这是进化算法的 **adversarial property**——它会找到你 simulator 里所有的 bug 并加以利用。这不是 feature，是 fundamental property。如果你的物理引擎不守恒能量，creature 会学会"从空气中提取能量"。这实际上成了物理引擎的一个极端 fuzz tester。

---

## 3. Fitness Functions（行为选择）

| 行为 | Fitness 度量 | 环境设置 |
|------|-------------|---------|
| **Swimming** | center of mass 移动距离/时间；直线优先（用 max distance from initial position）；末期速度权重更高 | 关闭重力，开启粘性水阻 |
| **Walking** | 水平速度（忽略垂直分量）；先无摩擦无肌肉力下落至稳定高度 | 重力 + 地面 + 摩擦 |
| **Jumping** | 最低 part 离地最大高度 | 重力 + 地面 |
| **Following** | 多次试验中朝向光源移动的平均速度 | photosensor 启用 |

**Swimming fitness 的细节公式**（从论文描述重建）：

$$\text{fitness} = \alpha \cdot d_{\max} + \beta \cdot \bar{v}_{\text{final}}$$

其中 $d_{\max}$ 是 center of mass 相对初始位置的最大距离（奖励直线运动而非原地转圈），$\bar{v}_{\text{final}}$ 是测试周期最后阶段的平均速度（奖励持续运动而非一次性推力），$\alpha, \beta$ 是权重。

**Walking 中防止"作弊"的设计**：creature 可能直接倒下来获得高速度。Sims 的方案是先在**无摩擦、无 effector force** 条件下让 creature 自由下落到稳定状态，然后再开始有摩擦的正式测试。这确保 fitness 测量的是真正的主动运动。

---

## 4. Evolution 算法

### 4.1 参数
- Population size: **300**
- Survival ratio: **1/5**（每代 60 个存活）
- Generations: 通常 50-100
- 计算时间: 32-processor CM-5 上约 3 小时

### 4.2 Mutation（5 步）

1. **Parameter mutation**: 对 node/connection 的内部参数施加扰动。标量用 Gaussian-like 扰动（多个随机数叠加），扰动幅度 ∝ 原始值（大值大调，小值精调）
2. **Add node**: 新 node 初始无连接，如果后续没有 connection 指向它则被 garbage collect
3. **Connection mutation**: 修改 connection 参数，或随机重定向指向不同 node
4. **Add/remove connections**: 仅适用于 morphology graph（neural graph 的输入数固定）
5. **Garbage collection**: 从 root node（morphology）或 effector nodes（neural）向外传播可达性，删除不可达元素

**Mutation rate 自适应**：mutation frequency 按 graph 大小反比缩放，保证无论 graph 多大，平均至少 1 个 mutation 发生：

$$p_{\text{mut}} \propto \frac{1}{|G|}$$

其中 $|G|$ 是 graph 中元素数量。

### 4.3 Mating（3 种繁殖方式）

| 方式 | 比例 | 机制 |
|------|------|------|
| **Asexual** | 40% | 仅 mutation |
| **Crossover** | 30% | 将两个 parent 的 node 序列对齐，在 crossover point 切换复制来源 |
| **Grafting** | 30% | 从 parent 1 复制，随机选一个 connection 重定向到 parent 2 的一个 node，将 parent 2 的该 node 及其后代嫁接进来 |

**Grafting 的直觉**：这比 crossover 更有生物学类比性。它允许一个 creature 的完整子结构（比如一整条腿及其控制器）被"移植"到另一个 creature 上。这实现了 **modular combination**——独立进化的模块可以合并。

### 4.4 Nested graph mutation 顺序

先 mutate 外层 morphology graph，再 mutate 内层 neural graphs。因为 neural graph 中某些参数（inter-node input source）的合法值依赖于外层 topology。

---

## 5. Results

### Swimming
- 大量简单的 **paddling** 和 **tail-wagging** creature
- 一些用专门的 **sculling appendages** 拉着自己前进
- 一些有大量对称 flippers 的 creature
- **Multi-segmented watersnake**：用正弦波运动穿过水面

### Walking
- 简单的 **shuffling/hobbling** creature
- **Lizard-like gait**：用 part 的边角着地行走
- **Inchworm**：伸缩式前进
- **Hopping**：通过挥动 arm-like 结构实现弹跳前进

### Jumping
- 策略较少，但涌现出了简单的 jumping creature

### Following
- 水中：一些进化出 **steering fins**，通过 photosensor 输入转向光源
- 陆地：一些调整 paddle 角度来导向
- 部分 creature 在某些光源方位下失败——这反映了 **局部最优** 的特征

**Cross-environment transfer**：将水中进化的 watersnake 放到陆地上继续进化，它学会了 crawl。这说明 neural controller 具有一定的 **transferability**。

---

## 6. 我的批判性分析

### 6.1 这篇论文的真正贡献

这篇论文的核心 insight 不是一个算法，而是一个 **representation**：用 nested directed graph 同时编码 morphology 和 control，并通过 developmental process（graph traversal → phenotype hierarchy）实现结构复用。这个 representation 的关键性质是：

1. **Closure under mutation/mating**：mutation 和 mating 作用在 graph 上，产出的仍然是合法 graph
2. **Expressive sufficiency**：能表达从简单到复杂的连续谱
3. **Modularity**：local neural circuitry 随 part 复制，类似于生物学中的 segment duplication

### 6.2 局限性

1. **Fitness landscape 非常稀疏**：随机生成的 graph 大概率产生不合法或零 fitness 的 phenotype。论文承认需要"补充 rounds of seed genotypes"来替换零 fitness 个体
2. **No co-evolution / competition**：fitness 是绝对度量，不是 relative。Future work 部分提到但没有实现
3. **Neural graph 的 input 数量固定**：这限制了 neural architecture 的探索——你无法进化出一个突然需要 5 个输入的 neuron
4. **Physical simulation 的简化**：矩形 part、简化流体模型、无真实流体动力学。某些策略可能只在这个简化物理下 work
5. **计算成本**：即便在 1994 年的 CM-5 上，300 个体 × 100 代需要 3 小时。这在当时是合理的，但搜索空间实际上只被极其稀疏地采样了

### 6.3 与现代工作的关联

- **Neuroevolution of Augmenting Topologies (NEAT)** [Stanley & Miikkulainen, 2002] 可以看作 Sims 的 neural graph 部分的后续发展——但它只进化 control，不进化 morphology
- **DeepMind 的 EnvLife / virtual creature** 系列工作继承了"morphology + control 共同进化"的思想
- **DIFFTA / Differentiable Physics** 方向用 gradient descent 替代 GA，但仍然面临 Sims 指出的 representation 问题

这篇论文在 1994 年提出的问题——"如何表示和搜索可以增长复杂度的结构"——在今天仍然是 open problem。

---

**总结一句话**：Sims 的这篇论文证明了，给定一个足够好的 genetic representation（nested directed graph）和一个物理可靠的仿真环境，evolution 可以同时搜索 morphology 和 control 的联合空间，涌现出人类难以设计但物理上有效的运动策略。它的核心贡献在于 representation 的设计直觉，而非某个具体的算法突破。
