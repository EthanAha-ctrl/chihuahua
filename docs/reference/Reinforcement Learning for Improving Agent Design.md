
让 AI agent 自己改自己的身体，身体和控制策略一起学，结果跑得更快、学得更轻松。做 RL 的人，默认 agent 的身体是固定死的——给你一个 Ant，四条腿，每条腿多长多粗，都是写死的。你的算法只负责学"怎么动腿"。这就像让你穿着冰鞋去跑马拉松，然后只允许你练跑步技巧，不能换鞋。David Ha 说：这不合理。生物不是这样的——运动员一辈子在 **同时** 练技术和练身体。形态和策略是共同进化的。把"身体参数"和"策略参数"拼成一个大向量，一起丢进优化器里搜。
- 策略参数 = neural network 的权重（控制怎么动）
- 身体参数 = 腿有多长、多粗、关节角度（控制长什么样）

每次 rollout 之前，先用身体参数把环境里的 agent "捏"成对应的样子，然后再让 policy 去跑。跑完拿到 reward，用这个 reward 同时更新身体参数和策略参数。就这么简单。没有什么新算法，用的就是 1992 年 Williams 的 REINFORCE，一个 population-based 的黑盒搜索方法（跟 OpenAI-ES 是一个家族）。不用 PPO/TRPO 因为 body 参数有个尴尬的特性：它在一个 episode 里是固定的。你开头把腿捏好，整个 episode 都是这个腿。不像 action 每步都在变。这意味着 body 参数的 reward signal 是 极度稀疏的 —— 跑了 1000 步，最后给你一个总分，你根本不知道这 1000 步里哪个动作是因为腿长好才做对的，哪个是因为腿短才做错的。PPO/TRPO 依赖每步的 dense reward 做梯度回传，面对这种稀疏信号会很痛苦。而 population-based 方法（ES/REINFORCE）天然就是干这个的——它不在乎你中间发生了什么，只看你最后的总分，然后让总分高的"身体+策略"组合多生孩子。

### Ant（四足机器人）

原始 Ant 跑分 3447。学了身体之后跑分 5789，提升 68%。

学出来的身体长什么样？**腿全部变长变细**，而且 **打破了左右对称**——后腿比前腿粗。

直觉：后腿需要蹬地发力（粗=力大），前腿只需要摆动和探路（细=轻便）。这个 asymmetry 是优化器自己发现的，没有人为编码。

### BipedalWalker（双足行走）

简单版本：学出来的腿变得很细很长，像两根弹簧，agent 从"蹦蹦跳跳"变成"弹弹弹弹"。

难版本（Hardcore，要过障碍、爬坡、跳坑）：这个 task 非常难，之前只有两篇 paper 解决过，最快的也要 40 小时。

**加了 morphology learning 之后，12 小时就解了——快了 3 倍。**

而且学出来的身体很有意思：前腿细长（兼做触觉 sensor 探路），后腿下半截粗（当稳定器）。一个好的身体让 policy 的搜索空间变小了——你不需要学复杂的平衡策略，身体结构本身就帮你稳定了一部分。

### 额外玩法：给 agent 发"减肥奖"

作者还给 reward 加了一项：**腿面积越小，bonus 越多**。

公式是：

$$
r_{\text{new}} = r \times \left(1 + \log\frac{A_{\text{orig}}}{A_{\text{new}}}\right)
$$

就是把原始腿面积除以当前腿面积取个 log，当 bonus 乘上去。腿越小，bonus 越大，但 log 让 bonus 增长递减（不会无脑缩到零）。

结果：简单版本里腿缩到原来的 8%，还是能跑。难版本里腿缩到 27%，也还能过——agent 学会了用最少材料完成任务。

---

## 为什么这件事重要？

三个层次：

1. **RL 社区几乎从不碰 morphology**。body 是给定的，是环境的一部分，是"上帝的设定"。这篇 paper 说：你把 body 也当 learnable parameter，效果更好。就这么简单的 idea，但没人系统地做过。

2. **好的身体让策略更容易学**。这不只是"最终性能高一点"，而是"训练快 3 倍"。body design 本身就在做一部分 computation——弹簧腿不需要 policy 去学怎么缓冲，物理结构自动完成了。这叫 **morphological computation**。

3. **桥接了两个社区**。ALife（人工生命）社区搞 morphology evolution 搞了 25 年（从 Karl Sims 1994 开始），RL 社区完全不知道也不关心。David Ha 把 ALife 的 idea 搬到了 RL 社区最熟悉的 OpenAI Gym benchmark 上，两边终于有了共同语言。

---

## 局限性（作者自己也承认）

- 只能 **调参数**，不能 **长新器官**。你不能从四条腿变成三条腿或者加个翅膀——只能在现有拓扑结构上缩放。
- 必须 **加约束**（±75%）。不约束的话 agent 会钻物理引擎的 bug，长出各种妖魔鬼怪形状（作者贴心地放在了项目网站的 "Bloopers" 里）。
- 计算贵。96 核 CPU 跑好几天，做 10 次实验就是 10 台机器跑好几天。
- 作者是 David Ha，他天生就擅长这种"把简单 idea 做得很优雅、可视化做得很好"的风格。

---

## 最终 takeaway

这篇 paper 的核心 insight 用一句话讲：

**你的身体不是你大脑的容器，你的身体就是你大脑的一部分。**

agent 的形态决定了哪些运动模式是"免费的"（被动物理就能完成），policy 只需要学会触发和利用这些模式。所以 **优化形态 = 缩小策略搜索空间 = 更快学到更好的策略。**

项目页面（有视频，值得看）：
🔗 [https://designrl.github.io/](https://designrl.github.io/)

代码和预训练模型：
🔗 [https://github.com/hardmaru/astool](https://github.com/hardmaru/astool)

---

让我先通读这篇 paper 的完整内容，然后给你做详细讲解。



# Reinforcement Learning for Improving Agent Design — 详解

这是 **David Ha**（Google Brain Tokyo）2019年发表在 *Artificial Life* 期刊上的工作。核心 idea 非常简洁：**在 RL 中，agent 的 body design 是固定的——能不能让它一起学？**

---

## 1. 第一性原理：为什么 morphology matters

传统 RL 的 setting 是：

$$
\max_\phi \; \mathbb{E}\left[ \sum_{t=0}^{T} \gamma^t r_t \;\middle|\; \pi_\phi, \; \mathcal{E}_{\text{fixed}} \right]
$$

- $\phi$：policy network 的权重参数
- $\mathcal{E}_{\text{fixed}}$：environment（包括 agent 的 body morphology），**完全固定**
- $\gamma$：discount factor
- $r_t$：时刻 $t$ 的 reward

这隐含了一个假设：agent 的 body 是上帝给定的，你的 job 只是学会控制它。但从 **embodied cognition** 的角度看，认知能力是被 body shape 塑造的——甚至脑死亡后，某些 organism 仍能 jump、swim，因为 morphology 本身就 encode 了 motor skills。这在 ALife 社区是常识（Karl Sims 1994 [59,60]），但 RL 社区几乎从未触及。

David Ha 的核心 insight 是：**把 body 参数也变成 learnable parameter，和 policy 一起 joint optimization。**

---

## 2. 方法：把 body 参数塞进同一个 optimization 里

### 2.1 参数化方式

定义一个 environment parameterization vector，它是 policy network 权重 $w_{\text{policy}}$ 和 body design 参数 $w_{\text{body}}$ 的拼接：

$$
\mathbf{w} = [w_{\text{policy}}, \; w_{\text{body}}]
$$

- $w_{\text{body}}$ 控制：每条腿的 **length**（$\Delta x$, $\Delta y$）、**radius**（size）等几何属性
- 在 rollout 之前，先用 $w_{\text{body}}$ 调用 `env.augment(env_params)` 修改 environment

Figure 2 中对比了标准 OpenAI Gym rollout（左）和 augmented 版本（右）：

```python
# 标准 Gym
obs = env.reset()
a = agent.action(obs)        # policy 只控制 action
obs, r, done = env.step(a)

# augmented 版本
env.augment(env_params)       # 先用 body 参数修改环境
obs = env.reset()
a = agent.action(obs)
obs, r, done = env.step(a)
r = augment_reward(r, env_params)  # reward 也可以被 body 参数 augment
```

关键点：**body 参数在每个 episode 开头设置一次，在整个 episode 内保持不变。** 这意味着 body 参数的 credit assignment 是 **sparse** 的——你只在 episode 结束时拿到 cumulative reward，没法在每一步分别归因到 body 和 policy。这也解释了为什么作者选了 population-based method 而非 PPO/TRPO（后面对比讨论）。

### 2.2 优化算法：Population-based REINFORCE

用的是 **Williams 1992 REINFORCE** 的 Section 6（population-based policy gradient），与 OpenAI-ES [51]、NES [71] 本质上是同一类方法。

核心推导：

**目标函数：**

$$
J(\theta) = \mathbb{E}_\theta [R(\mathbf{w})] = \int R(\mathbf{w}) \, \pi(\mathbf{w}, \theta) \, d\mathbf{w}
$$

- $\theta$：distribution 的参数（mean $\mu$ 和 std $\sigma$）
- $\pi(\mathbf{w}, \theta)$：参数化的 sampling distribution
- $R(\mathbf{w})$：parameter vector $\mathbf{w}$ 对应的 cumulative reward

**Log-likelihood trick（score function estimator）：**

$$
\nabla_\theta J(\theta) = \mathbb{E}_\theta \left[ R(\mathbf{w}) \, \nabla_\theta \log \pi(\mathbf{w}, \theta) \right]
$$

直觉：如果某个 $\mathbf{w}$ 得到了高 reward，就沿着 $\log \pi$ 的梯度方向把 distribution 往那个 $\mathbf{w}$ 推；低 reward 的则推远。reward 在这里充当 **加权信号**，而不是做 temporal credit assignment。

**Population 近似（population size $N$）：**

$$
\nabla_\theta J(\theta) \approx \frac{1}{N} \sum_{i=1}^{N} R(\mathbf{w}^i) \, \nabla_\theta \log \pi(\mathbf{w}^i, \theta)
$$

- $\mathbf{w}^i$：第 $i$ 个 population member（上标 $i$ 表示 population 索引）
- $N$：population size（论文中 $N=192$）

**Factored multivariate normal distribution：**

每个维度独立采样：$w_j \sim \mathcal{N}(\mu_j, \sigma_j^2)$，下标 $j$ 表示 parameter vector 的第 $j$ 维。

Williams 给出的 **closed-form gradients**：

$$
\nabla_{\mu_j} \log \mathcal{N}(w_j^i, \theta) = \frac{w_j^i - \mu_j}{\sigma_j^2}
$$

$$
\nabla_{\sigma_j} \log \mathcal{N}(w_j^i, \theta) = \frac{(w_j^i - \mu_j)^2 - \sigma_j^2}{\sigma_j^3}
$$

- 第一式：mean 的梯度就是标准化残差 $(w_j^i - \mu_j)/\sigma_j^2$，reward 高的 sample 会把 $\mu_j$ 拉向自己
- 第二式：std 的梯度取决于该 sample 偏离 mean 的平方是否超过 $\sigma_j^2$，自动调节探索强度

**更新规则（gradient ascent）：**

$$
\theta \leftarrow \theta + \alpha \, \nabla_\theta J(\theta)
$$

$\alpha$ 是 learning rate。

**与 CMA-ES 的联系：** Akimoto et al. [2] 证明了 CMA-ES 可以从 information geometry 角度理解为 natural gradient of NES。所以这套方法在数学上和 CMA-ES 是近亲。不同之处在于这里 $\mu$ 和 $\sigma$ 的更新用固定 learning rate，而 CMA-ES 用自适应的 covariance matrix 更新。

---

## 3. 实验

### 3.1 RoboschoolAnt-v1（4 足 Ant）

- **Body 参数：** 36 个（每条腿 3 个 part × 3 参数/part × 4 腿 = 36），每个 part 有 $\Delta x$, $\Delta y$, size(radius)
- **约束：** 每个参数可缩放到 original 的 ±75%（保持 sign 和 direction 以维持拓扑结构）
- **Policy network：** 3-layer MLP，28 inputs → 64 → 32 → 8 outputs，tanh activation

**学习结果（Table 1，单次 run）：**

| Leg Part | Top Left Length | Top Left Radius | Top Right Length | Top Right Radius | Bottom Left Length | Bottom Left Radius | Bottom Right Length | Bottom Right Radius |
|---|---|---|---|---|---|---|---|---|
| Top | 141% | 33% | 141% | 25% | 169% | 35% | 84% | 51% |
| Middle | 169% | 26% | 164% | 26% | 171% | 31% | 140% | 29% |
| Bottom | 174% | 26% | 168% | 50% | 173% | 29% | 133% | 38% |

关键发现：
- **所有腿变长变细**（length ↑, radius ↓）—— 减少转动惯量，更容易摆动
- **打破了对称性** —— rear legs 比 front legs 更大，说明不对称 body 对 forward navigation 有利
- 性能：original 3447±251 vs. learned 5789±479（**提升 ~68%**）

### 3.2 BipedalWalker-v2 & BipedalWalkerHardcore-v2

BipedalWalker 是 Box2D 环境，agent 有两条腿（每条腿 2 个 part），靠 LIDAR sensor 感知地形。

**Hardcore 版本特别难**——需要跨过障碍、上下坡、跳过坑。此前只有两篇报告 solve 了这个 task：
- Population-based training [28]：96-CPU, 40 小时
- A3C + LSTM [26,43]：72-CPU, 48 小时

**核心实验结果（Table 2, 单次 run）：**

| 环境配置 | Avg. Score | Leg Area (%) | 关键观察 |
|---|---|---|---|
| BipedalWalker original | 347±0.9 | 100% | joyful skip 步态 |
| BipedalWalker + Morphology | 359±0.2 | 33% | 腿变细变长，像弹簧一样 bounce |
| BipedalWalker + Morphology + Smaller Leg reward | 323±68 | 8% | 腿缩到 ~25% 下限 |
| BipedalWalkerHardcore original | 313±53 | 100% | baseline |
| BipedalWalkerHardcore + Morphology | 335±37 | 95% | 前腿细长（当 sensor），后腿粗（当稳定器）|
| BipedalWalkerHardcore + Morphology + Smaller Leg reward | 312±69 | 27% | 宽度压到下限，保留必要高度 |

**最重要的发现——morphology learning 加速 policy learning：**

在 BipedalWalkerHardcore-v2 上：
- **Original body**：4600 generations, 40 小时 solve
- **Learnable body**：1400 generations, 12 小时 solve（**3× 加速**）

这说明 body design 的优化不仅提升了最终性能，还 **降低了 policy 搜索的难度**。直觉上，一个 well-designed body 可以让某些有利的运动模式（如 spring-like bouncing）自然涌现，policy 只需要学会触发和维持这些模式。

### 3.3 Reward Augmentation：优化材料使用

当 agent 被额外 reward 鼓励使用更小的腿时，reward 被 augment 为：

$$
r_{\text{augmented}} = r \times \left(1 + \log\left(\frac{A_{\text{orig}}}{A_{\text{new}}}\right)\right)
$$

- $A_{\text{orig}}$：original leg area
- $A_{\text{new}}$：current leg area
- $\log(\cdot)$ 使得 reward gain 在 leg area 减小时递增但递增速率放缓（diminishing returns）

这展示了 **multi-objective optimization** 的可能性：可以 trade off performance vs. material cost，类似于 engineering design 中的 constraint optimization。

### 3.4 多次实验统计（Table 3, 10 independent runs）

| 实验 | Avg. Score (10 runs) |
|---|---|
| (a) Ant baseline | 3139 ± 189.3 |
| (b) Ant + Morphology | **5267 ± 631.4** |
| (c) Biped baseline | 345 ± 1.3 |
| (d) Biped + Morphology | **354 ± 2.2** |
| (e) Biped + Morph + Smaller Leg | 330 ± 3.9 |
| (f) Biped Hardcore baseline | 300 ± 11.9 |
| (g) Biped Hardcore + Morphology | **326 ± 12.7** |
| (h) Biped Hardcore + Morph + Smaller Leg | 312 ± 11.9 |

Morphology learning 在所有环境中 **一致地** 提升了性能。值得注意的是 Ant 的 variance 也显著增大（631.4 vs 189.3），说明不同 run 发现了不同 body design——morphology search space 比 policy space 更有多样性。

---

## 4. 训练细节

- **Population size：** 192
- **每个 candidate 评估 16 次**（不同 random seed），reward 取 16 次平均
- **Hardware：** 96-core CPU machine
- **Training generations：** Ant 3000, BipedalWalker 5000, BipedalWalkerHardcore 10000
- **保存策略：** 保留整个训练历史中 best average reward 的 agent（而非最后一代）

---

## 5. 关键 Insights 和局限性

### 涌现的设计原则
1. **打破对称性**：agent 一致性地发展出更大的后腿，这对 forward locomotion 有利
2. **形态即计算**（morphological computation）：前腿变细变长，兼做 LIDAR 的补充 sensor
3. **被动动力学**：legs 学会充当弹簧，部分运动由物理结构本身完成，减少 policy 的控制负担

### 局限性
1. **只能优化现有设计**，不能像 Karl Sims 那样 evolve 出全新 topology（没有 structural mutation）
2. **±75% 约束是必须的**——不约束会导致 agent exploit simulator 的物理 bug（见 [Bloopers](https://designrl.github.io/)）
3. **Sparse credit assignment**：body 参数只在 episode 开头设一次，所以本质上是 black-box optimization，不适合 PPO/TRPO 这类需要 dense gradient 的方法
4. **计算成本高**：96-core 跑数天，10 次 independent run 需要大量资源

### 与 RL 社区的关系

作者明确提到一个 **inner loop / outer loop** 的设想：
- **Inner loop：** 固定 body，用 PPO/TRPO 学 policy（dense reward, fast policy learning）
- **Outer loop：** 用 evolutionary method 搜索 body design（sparse reward, global search）

这本质上就是 **bilevel optimization**，也是后来许多 co-design 工作的方向。

---

## 6. 我的评价

从第一性原理看，这篇工作触及了一个 RL 领域长期被忽视但根本性的问题：**agent 的 body 不是中性的 container，morphology 本身就是 computation 的一部分。** 把 policy 和 morphology 放进同一个 optimization 框架，在数学上没有什么创新——就是把 parameter vector 拼长了一点，用 ES/REINFORCE 搜。但 idea 的价值在于它 **打通了 ALife 和 RL 两个社区**，用 RL 社区熟悉的 Gym benchmark 展示了 ALife 社区的 morphology learning 概念。

实验结果也很有说服力：最 striking 的是 BipedalWalkerHardcore 从 40 小时降到 12 小时 solve——这说明 **合适的 body design 比 better policy 更重要**，至少在某些任务上。这呼应了 Rodney Brooks 的经典观点：intelligence 不只在 brain 里，body 本身就在做 inference。

项目页面和代码：
- 🔗 [https://designrl.github.io/](https://designrl.github.io/)（视频结果）
- 🔗 [https://github.com/hardmaru/astool](https://github.com/hardmaru/astool)（代码 + pretrained models）

参考的关键 prior work：
- Karl Sims, *Evolving Virtual Creatures*, SIGGRAPH 1994 — [DOI](https://doi.org/10.1145/192161.192204)
- Williams, *REINFORCE*, 1992 — [DOI](https://doi.org/10.1007/BF00992696)
- Salimans et al., *Evolution Strategies as a Scalable Alternative to RL*, 2017 — [arXiv:1703.03864](https://arxiv.org/abs/1703.03864)
- Pfeifer & Bongard, *How the Body Shapes the Way We Think*, MIT Press 2006
