Boston Dynamics 有非常深的 robot locomotion know-how:

硬件集成、控制、动态平衡、执行器、可靠性、实机调参、跌倒恢复、量产工程，世界顶级。强在：

```text
robot performs tasks
robot remains stable
robot survives commercial use
robot carries payload
robot navigates human terrain
```

传统 robotics 的“task”通常是外部指定的：

```text
go there
inspect this
carry that
open door
avoid obstacle
return to dock
```

环境是被工程师定义的，奖励函数是被工程师写的，失败模式是被测试场景枚举的。

---

https://chiwuawua.com/

`SURVIVAL_TO_DESIGN_GOALS.md` 很关键, 它把工程拉到“恢复一个智能动物继续生活的能力”。
 **return-to-wild / return-to-hunt prosthetic animal architecture** 是 know-how。
它是一个 living agent, 目标是:

```text
stay alive
stay with pack / family
seek food
avoid danger
learn household rules
respond to social reward
adapt to humans
form attachment
recover after injury
keep behaving like itself
```

and

```text
生态 use case
-> 生物行为需求
-> 形态/脊柱/关节 architecture
-> mass / torque / material / FEM / IK / MuJoCo
-> 复杂地形 refusal / survival validation
```
and

```text
agency
social intrinsic behavior
learning loop
wild survival
human-life adaptation
embodied continuity
```


intelligence 是生态和社会性的。
`good boy / good girl` 是关系里的 reinforcement。
它会观察人怎么生活，学什么时候靠近、什么时候让开、谁需要陪伴、哪里安全、哪里不能碰。

life context -> embodied behavior -> social learning -> survival adaptation



| 厂商／型号（官方链接）                                                                                                 | 重量    | 额定／持续扭矩 | 峰值／最大扭矩      | 减速比        | 驱动器   | 与 AKE 的可比性／采购状态                                                                                   |
| ----------------------------------------------------------------------------------------------------------- | ----- | ------- | ------------ | ---------- | ----- | ------------------------------------------------------------------------------------------------- |
| **CubeMars AKE40-6**（基准）                                                                                    | 140 g | 1.8 N·m | 5.5 N·m      | 6:1        | 外配    | 厂标 QDD；[官方参数对比表](https://www.cubemars.com/product/ake50-8-kv110-quasi-direct-drive-actuator.html) |
| **[CubeMars AKE50-8](https://www.cubemars.com/product/ake50-8-kv110-quasi-direct-drive-actuator.html)**（基准） | 190 g | 2.5 N·m | 7.5 N·m      | 8:1        | 外配    | 厂标 QDD；官网可下单                                                                                      |
| **[CubeMars AKE60-8](https://www.cubemars.com/product/ake60-8-kv80-quasi-direct-drive-actuator.html)**（基准）  | 260 g | 5 N·m   | 12.5 N·m     | 8:1        | 外配    | 厂标 QDD；官网可下单                                                                                      |
| **[Unitree GO-M8010-6](https://www.unitree.com/cn/go1/motor/)**                                             | 530 g | 5 N·m   | **23.7 N·m** | **6.33:1** | 内置    | 低减速比腿足执行器；明显更重、扭矩更大；[官方购买](https://shop.unitree.com/products/go1-motor)                           |
| **[酷卓 CODROID CoDrive PA60](https://www.codroid.ai/download/huace/part-pa.pdf)**                            | 400 g | 3 N·m   | **9 N·m**²   | **6:1**    | 目录未说明 | 厂标准直驱；较 AKE50／60 更重；未核实直接下单渠道                                                                     |


mechanical tendon stop + 凹轮
