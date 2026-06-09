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
