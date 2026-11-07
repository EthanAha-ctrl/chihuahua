## Related work

- 形态和控制联合优化: Sims 1994 的进化虚拟生物; Schaff et al. 2019 和 Ha 2019 用 RL 同时优化结构和控制器; Gupta et al. 2021 (DERL, 发在 Nature Communications).
- 可微协同设计: Spielberg et al. 2019 在软体机器人上做 learning-in-the-loop, 梯度能穿过仿真; Evolution Gym (2021).
- 真实四足上的协同设计: Bjelonic et al. 2023 在四足上联合优化并联弹性执行器和 RL 控制器, 但只是少数几个参数.
- [Dogger](https://arxiv.org/abs/2611.02462) (arXiv 2611.02462, 2026 年 11 月刚出): 做法是先训练一个以形态为条件的 RL policy, 再用贝叶斯优化按任务选形态, 最后造了真机. 它的外层是离散/无梯度搜索, 和你的 Scope 思路一致; 但它不选执行器, 也不做 FEM.
- Hutter 组 Meta-RL legged design ([ResearchGate](https://www.researchgate.net/publication/364159223_Meta_Reinforcement_Learning_for_Optimal_Design_of_Legged_Robots)): 把减速比映射成执行器的质量, 效率和峰值扭矩, 用 CMA-ES 和 RL 一起优化. 在执行器选型这件事上, 它离你最近.
- [COSMIC](https://arxiv.org/html/2605.12654) (arXiv 2605.12654): 拓扑, 材料和控制器在同一个可微框架里联合优化, 相当于把 FEM 式结构优化放进了梯度链. 但对象只是桁架机器人, 没有造真机.
- 其他: [Dinev et al. 2021](https://arxiv.org/pdf/2103.04660) 做了四足减速比和杆长的双层优化; [ETH IJRR 2018](https://crl.ethz.ch/papers/IJRR_2018.pdf) 联合优化设计参数和运动; [并联弹性执行器 + RL](https://arxiv.org/pdf/2301.03509).