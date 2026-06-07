# Survival To Design Goals

## Context

`yellowstone.md` 给出这一层意图：

```text
return-to-wild / return-to-hunt
```

原文定义：

```text
动物植入人工骨骼与 powered artificial joints 后，能在 Yellowstone 长期野外生存，并继续完成完整生态行为。
```

`TOP_LEVEL_DESIGN.md` 给出工程链路：

```text
geometry
 -> mass / inertia / torque model
 -> structural FEM iteration
 -> IK and control
 -> MuJoCo simulation
 -> printable CAD implementation
```

两份文档之间缺一层转换函数：

```text
Yellowstone result -> engineering design goal
```

## Translation Function

当前只能把转换函数写成迭代更新：

```text
D_t + C_t -> O_t
D_{t+1} = U(D_t, C_t, O_t, E_t)
```

含义：

```text
D_t = 第 t 版设计
C_t = Yellowstone 环境和释放条件
O_t = 野外结果
E_t = 证据：遥测、视频、尸检、足迹、环境记录、回收零件、行为记录
U   = 根据结果和证据生成下一版设计改动的更新函数
```

`U` 的内容当前缺失。它只能通过实际版本迭代逐步填充。

## Progress Criterion

转换层通过野外迭代积累内容：

```text
design version
 -> field outcome
 -> evidence
 -> design change
 -> next field outcome
```

一条 design goal 进入 top-level design，需要满足：

- 来源能追到某次野外结果，或明确标记为待验证假设。
- 影响的工程变量能落到 `TOP_LEVEL_DESIGN.md` 的工程链路里。
- 下一版野外结果能支持、削弱或推翻这条 design goal。

