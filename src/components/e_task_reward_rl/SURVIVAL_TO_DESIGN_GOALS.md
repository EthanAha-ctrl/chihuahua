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

---

生态位是：

```text
捕猎 elk / moose / coyote / bunny
被威胁时 run away
5-8 群族
洞居，地面挖洞
常规冬季生存
```

pack predator + denning mammal + cursorial runner + digging-capable animal

行为需求：

1. **追兔子需要急转和爆发**  
   bunny 是 zigzag prey。只靠四肢转向，身体像硬盒子，转弯半径会变大，前后肢相位也难快速重排。腰/脊柱能让前后半身在转向时分担角动量。

2. **追 elk / moose 需要步幅和耐力经济性**  
   不是 cheetah 式极限脊柱，但需要躯干参与 stride modulation。狼这种 pack cursorial predator 也不是 rigid body。脊柱活动能改善步态转换、坡地追踪、失足恢复。

3. **扑击/咬住大型猎物需要身体扭转和抗反扭**  
   Elk/moose 会踢、甩、冲。捕猎不是只跑到旁边，还要接触、避蹄、咬合、撤离。躯干完全刚性会让转身和卸力很差。

4. **coyote 是竞争者/猎物边界对象**  
   捕 coyote 不是单纯 prey capture，它会反击、绕圈、逃。需要低延迟转向、侧向跳步、快速姿态恢复。

5. **洞居和地面挖洞需要身体可弯曲**  
   不是说要像蛇，但进入洞、转身、倒退、护幼、巢内挪动，都需要躯干能配合肩带和骨盆。刚性长躯干会让洞穴半径变大，筑洞成本变高。

6. **5-8 群族 + 育幼需要姿态表达和近身接触**  
   社交动物需要身体姿态、压低、翻滚、蜷护、幼仔贴近。躯干太刚会损失很多行为带宽。

