
要造一个会走路的机器人。机器人的身材和它怎么走路是互相影响的。不能先随便定个身材再学走路，也不能先定个走法再设计身材——得一起搞。传统做法是分开的：人凭经验画一个设计图，然后让 RL 去学控制策略。把机器人身材设计和控制策略训练塞进同一个 RL 优化循环里，用设计分布代替固定设计，让 policy 能适应不同身材，然后根据表现好坏逐步收紧设计分布——最终自动发现人类想不到的好设计。不固定一个设计，而是维护一个"设计分布"——就是一堆候选身材，有的腿长有的腿短，有的胖有的瘦。一开始铺得很开，什么都有。每次训练，从这堆候选里随机抽一个身材出来，让 policy 去试着控制它走路。关键 trick 是——policy network 的输入除了机器人当前状态，还把当前用的设计参数也喂进去。这样 policy 知道"哦这次腿比较长，我得迈大步"，能针对不同身材调整策略。走完一圈看 reward。走得好的身材，下次出现的概率调高一点；走得差的，概率调低。这跟 policy gradient 的逻辑一模一样——只不过这里"action"变成了"选什么身材"，"reward"变成了"这个身材走得好不好"。训练过程中，分布会越来越窄，集中在表现好的设计上。最后收敛到一个身材 + 一套步态，一起达到最优。

用 GMM（高斯混合模型）不用单个gaussian？好的设计可能有多种。比如 Hopper 就学出了两种风格：重身体+小碎步 vs 轻身体+大跳步。单个高斯会把这两种 average 掉，GMM 能同时保留多个 hypothesis。然后每隔一段时间，砍掉表现最差的一半 component，慢慢淘汰。

一开始 policy 什么都不会，reward 是噪声。如果这时候就开始调设计分布，会被噪声带偏。所以前 100M 步只练 policy、冻结设计分布，等 policy 至少能控制一些设计了，再开始用 reward 来引导设计搜索。

训练全程 policy 都是在一个分布上练的，对很多设计都能走，但不一定对最终那个设计做到最优。所以最后固定到分布的 mode（出现概率最高的那个设计），让 policy 专门为它再练一阵。

在三种机器人上测的——Hopper（单腿）、Walker（双腿）、Ant（四腿）。

**平地**：全部吊打 baseline。学出来的设计很有意思，Walker 学出了长细腿小脚的跑步步态，有一个 seed 甚至学出了"跪着走"——低姿态但步幅巨大，像短跑运动员起跑。Ant 虽然四条腿参数独立优化，但最终收敛到四条腿差不多大小，而且只用三条腿走路，前腿基本闲着偶尔平衡用。

**斜坡**：Hopper 和 Walker 还是赢。Walker 甚至发现了一种两只脚同时蹬地的走法，省力。但 Ant 翻车了——8 个 seed 有 7 个卡在"站着不动只保持平衡"的 local optimum。因为 warm-up 期间 policy 发现站着不动比尝试走路更"安全"（不会摔倒扣分），设计分布就往"容易站稳但难走"的大身材偏，恶性循环。后来把"保持直立"的奖励砍一半，强迫它往前走，才解决这个问题。



---

## 论文概述

**标题**: *Jointly Learning to Construct and Control Agents using Deep Reinforcement Learning*

**作者**: Charles Schaff, David Yunis, Ayan Chakrabarti, Matthew R. Walter (TTIC / UChicago / WashU)

**核心问题**: Robot 的 physical design（物理设计）和 control policy（控制策略）是 coupled（耦合的）——不同的 body design 对应不同的最优 gait。传统做法是 fix design → train policy，把 design 当 hyperparameter 来搜，但 deep RL training 本身就贵，design space 又是高维的，所以 brute-force 或 Bayesian optimization 都不 scalable。

**核心思路**: 不要 decouple design 和 policy 的优化。维护一个 **distribution over designs** $p_\phi(\omega)$，让 policy $\pi_\theta(a|s, \omega)$ 同时看到 state 和 design parameters，在 design distribution 上做 **expected reward maximization**，然后逐步 shift distribution 朝高 reward 的 design 靠拢，最终 converge 到 jointly optimal 的 design + policy。

---

## 方法详解

### 1. 标准 RL formulation（背景）

Standard MDP: $\{S, A, p, r, p_0\}$

- $S \subseteq \mathbb{R}^d$: state space（关节角度、速度、质心速度、torso高度等）
- $A \subseteq \mathbb{R}^n$: action space（joint torques）
- $p(s_{t+1}|s_t, a_t)$: transition dynamics（unknown）
- $r(s_t, a_t)$: reward function（forward progress + staying upright - torque penalty - joint limit penalty）
- $\gamma \in [0,1)$: discount factor

Policy $\pi_\theta(a_t|s_t)$ 用 neural network 参数化，通过 PPO 训练。PPO 的 clipped objective:

$$\hat{E}_t\left[\min\left(r_t(\theta)\hat{A}_t,\ \text{clip}(r_t(\theta), 1-\epsilon, 1+\epsilon)\hat{A}_t\right)\right]$$

其中 $r_t(\theta) = \frac{\pi_\theta(a_t|s_t)}{\pi_{\theta_{old}}(a_t|s_t)}$ 是新旧 policy 的概率比，$\hat{A}_t$ 是 advantage estimate，$\epsilon$ 是 clip range。这个 objective 限制每次 policy update 的步长，避免 catastrophic collapse。

### 2. Joint optimization formulation（核心创新）

引入 design parameter $\omega \in \Omega$（比如各 link 的 length 和 radius）。不同 $\omega$ 对应不同的 transition dynamics $p(s_{t+1}|s_t, a_t, \omega)$。关键假设：所有 design 共享同一个 $S, A, r, p_0$，只是 dynamics 不同——这意味着 morphology（拓扑结构）固定，只优化连续几何参数。

**目标函数**:

$$\phi^*, \theta^* = \arg\max_{\phi, \theta} \mathbb{E}_{\omega \sim p_\phi}\left[\mathbb{E}_{\pi_\theta}[R_t]\right]$$

- $\phi$: design distribution $p_\phi(\omega)$ 的参数
- $\theta$: control policy $\pi_\theta(a_t|s_t, \omega)$ 的参数
- $R_t = \sum_{i=0}^{\infty} \gamma^i r_{t+i}$: discounted return

注意 policy 的输入是 $(s_t, \omega)$——policy network 能看到当前的 design parameters，从而 **tailor** 其行为到具体 design。这是关键设计：如果 policy 不知道 $\omega$，它只能学一个对 design distribution 的 "average" 策略，而不是对每个 design 的 specialized 策略。

### 3. Design distribution 的 gradient update

对 $\phi$ 的 gradient 用了 **score function estimator**（aka REINFORCE trick）:

$$\nabla_\phi \mathbb{E}_{\omega \sim p_\phi}\left[\mathbb{E}_{\pi_\theta}[R_t]\right] = \mathbb{E}_{\omega \sim p_\phi}\left[\nabla \log p_\phi(\omega) \cdot \mathbb{E}_{\pi_\theta}[R_t]\right]$$

- $\nabla \log p_\phi(\omega)$: log-derivative of design distribution w.r.t. $\phi$，即 score function
- $\mathbb{E}_{\pi_\theta}[R_t]$: 在 design $\omega$ 下 policy 的 expected return，实践中用 episode reward $R_i$ 来 estimate

这个 gradient 的直觉：如果某个 $\omega$ 获得了高 reward，就增大 $p_\phi(\omega)$ 的密度（shift $\phi$ 朝那个方向）；低 reward 的 design 概率密度降低。这跟 policy gradient 的逻辑完全一致，只不过这里 "action" 是 design $\omega$，"reward" 是 policy 在该 design 下的 performance。

实际 update:

$$\nabla_\phi \approx \frac{1}{n} \sum_{i=1}^{n} \nabla \log p_\phi(\omega_i) R_i$$

其中 $n$ 个 design $\omega_i$ 从 $p_\phi$ 采样。

### 4. Design distribution 的参数化: Gaussian Mixture Model (GMM)

$p_\phi(\omega)$ 用 **8-component GMM with diagonal covariance** 参数化。为什么用 GMM 而非单 Gaussian？

- **Multi-modal exploration**: design space 可能是 multi-modal 的（比如 Hopper 的两种 gait: heavy body + short strides vs. light body + bounding）。单 Gaussian 会 average 掉这些 mode。
- **防止 premature convergence**: 高 entropy 初始化 → 逐步 prune 掉差的 component → 自然实现 exploration → exploitation 的过渡。

**Component pruning 机制**: 每 $N$ 步（$N = 100M$ timesteps，Ant 用 $200M$），从每个 component 采样 100 个 design，evaluate average reward，**砍掉 reward 最低的一半 component**。Mixing probabilities 始终在剩余 component 间 uniform。

这意味着 component 的 **means 和 variances** 通过 gradient 更新，而 **mixing weights** 通过离散的 elimination 来调整——不是 gradient-based 的，而是 tournament-style 的 hard pruning。

### 5. 训练流程（Algorithm 1）

```
1. 初始化 πθ(a|s,ω), pφ(ω) [8-component GMM, random means, high variance]
2. Warm-up: 只更新 θ（policy），冻结 φ [前 100M timesteps]
   — 让 policy 先学到能控制 diverse designs 的基础能力
3. 交替更新:
   a. 从 pφ 采样 n 个 design ωi
   b. 每个 design 用 πθ 跑 episode，收集 trajectories
   c. 用 PPO 更新 θ
   d. 每 C 步: 计算 episode returns Ri，用 score function gradient 更新 φ
   e. 每 N 步: prune 一半差的 GMM component
4. 最后 100M: 固定 ω* = mode(pφ)，fine-tune θ
```

**Warm-up 的直觉**: 如果一开始就更新 design distribution，policy 还没学到任何有用的东西，reward signal 是 noise——distribution 会被随机推向无意义方向。先让 policy 学到 "至少能控制一些 design" 的能力，再用 reward signal 来 guide design search。

**Final fine-tune 的直觉**: design distribution 最终会 converge 到一个 narrow distribution，但即使如此，policy 还是在一个 distribution 上训练的，不是在单一 design 上。最后固定到 mode 并 fine-tune，让 policy 完全 specialize 到最终 design。

### 6. Policy network 架构

- 3 层 fully-connected, 128 units/层, tanh activation
- Input: $(s_t, \omega)$ — state + design parameters 拼接
- Output: joint torques（连续 action space）

---

## 实验设置

### Morphologies

| Robot | Design params | 说明 |
|-------|--------------|------|
| Hopper | 8 | link lengths + radii |
| Walker | 8 | same, 但施加左右腿对称约束 |
| Ant | 25 | 4 条腿独立优化（无对称约束）+ body radius |

### Environments

- Level terrain
- Inclined terrain (5° slope)
- Built on Bullet Physics via Roboschool

### Baselines

1. **Default Designs**: 用 Roboschool 默认 hand-crafted design，只 train policy
2. **Random Sampling**: 均匀随机采样 design，每个 design 独立 train policy，给 3x 的 timesteps
3. **Bayesian Optimization**: BayesOpt 搜 design space，每个 candidate design train 到 convergence。同样 timestep budget 下只能 evaluate ~10 个 design

### 训练规模

- 1B timesteps（Ant 用 1.5B）
- 8 个 parallel samples per iteration
- Design distribution 在前 100M（Ant 200M）步冻结
- 每 100M（Ant 200M）步 prune 一次 component
- 最后 100M 步固定 design fine-tune

---

## 实验结果

### Level Terrain

所有三个 morphology 上，proposed method 的 **所有 seed** 都 outperform 所有 baseline。具体发现：

- **Hopper**: 学到的 design 有长 torso（提高稳定性），两种 gait：heavy body → short quick strides; light body → bounding gait
- **Walker**: 长细腿 + 小脚，running gait with long strides。一个 seed 学到了 "跪着走"（knee-walking），低 stance 但长 stride（类似 sprinter 起跑姿势）
- **Ant**: 小轻 body + 细腿。虽然 4 条腿独立优化，但 converge 到大致相同的 shape。Gait 用 3 条腿走，前腿基本离地偶尔用于平衡

### Inclined Terrain

- **Hopper & Walker**: 依然 outperform baselines。Walker 发现了一种两只脚同时 push 的 gait（不需要一只脚超过另一只），减少 torque penalty
- **Ant on incline — failure case**: 7/8 seed 卡在 local optimum（站着不动只保持平衡）。原因：warm-up 期间 policy 无法让足够多的 sampled design 向前移动，选择了 "safe" 的 standing 策略。Design distribution 随后 shift 到大 design（容易平衡但难移动），形成恶性循环

  **Fix**: 训练时把 "staying upright" reward 减半，进一步 incentivize forward motion。修改后 8/8 seed 都 escape 了 local optimum，且用 **原始** reward function 评估时也 outperform baselines。

### 训练动态（Fig. 4）

Histogram 显示：early training → high variance in both design params 和 reward → 逐步 narrow → component pruning 时有明显 discontinuous shift → 最终 converge 到 narrow distribution around optimal design。

---

## 我的批判性分析

### 亮点

1. **Idea 简洁且 principled**: 把 design optimization 和 policy optimization 统一到同一个 expected reward maximization 框架下，用 score function gradient 同时更新两者。从 first principles 看，这是最自然的 formulation——如果 design 影响 transition dynamics，而 transition dynamics 影响 reward，那 design 就应该出现在优化目标里。

2. **Conditioning policy on design** 是关键 insight: 这让 policy 能 generalize across designs，而不是 collapse 到某个 average behavior。这跟 conditional computation / multi-task learning 的思路一致。

3. **GMM + component pruning** 是一个实用的 multi-modal exploration 机制。比单纯 annealing variance 更 robust。

### 局限

1. **Morphology 必须固定**: action space 必须相同，所以不能搜 "几条腿" 这种离散结构选择。这是最大的限制——真正的 design space 是 discrete + structured 的。

2. **Score function gradient 的 variance**: $\nabla \log p_\phi(\omega) \cdot R_i$ 这个 estimator 的 variance 很高，尤其在高维 design space（Ant 的 25 维）。论文没有讨论 baseline subtraction 或 variance reduction 技术。

3. **Compute cost**: 1B timesteps，即使是 simulated robot，这也是相当大的计算量。跟 random sampling baseline 给了 3x budget 比，虽然赢了，但 cost-effectiveness 的 argument 不完全 clear。

4. **Ant on incline 的 failure**: 暴露了 warm-up period 的脆弱性——如果 policy 在 warm-up 期间没有学到 useful behavior，整个 pipeline 会 collapse。手动修改 reward function 来 fix 不够 general。

5. **Sim-to-real gap**: 论文没有做 real robot 验证。学到的 design（长细腿、小脚）在实际 fabrication 中可能有 structural integrity 问题。

---

## 与相关工作的关系

这篇 paper 的思路跟后来的几个方向有联系：

- **Environment Design / Unsupervised Environment Design** (e.g., PAIRED, Dennis et al. 2020): 也是维护一个 distribution over environments，但目的是 train robust agent，而非 optimize design。
- **Differentiable Simulation** (e.g., Brax, MuJoCo MJX): 如果 simulator 可微，可以直接对 design parameters 做 gradient descent，不需要 score function estimator。但当时（2018）不可微 simulator 是主流。
- **Meta-learning**: policy conditioning on design parameters 类似于 task conditioning in MAML / meta-RL，只不过 "task" 是 design choice。

论文链接: [arXiv:1801.01432](https://arxiv.org/abs/1801.01432)

项目页面: http://ttic.uchicago.edu/~cbschaff/nlimb （可能已失效）