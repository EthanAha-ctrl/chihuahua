**Design North Star**
目标是 **return-to-wild / return-to-hunt**：动物植入人工骨骼与 powered artificial joints 后，能在 Yellowstone 长期野外生存，并继续完成完整生态行为。

核心行为清单：

```text
追逐
捕猎 elk / moose / coyote / bunny
逃跑
5-8 个体群族社交
地面挖洞
洞居筑巢
哺育幼仔
冬季正常生存
```

**Biological Context**
目标动物经过基因编辑，面向 Yellowstone 环境。它的生态位接近：

```text
群体捕猎者
能追大型猎物
能追小型快速猎物
能挖洞和洞居
能育幼
能在野外受伤后回归功能
```

脊柱/腰部活动能力有价值，原因来自行为需求：

```text
追 bunny 需要急转
追 elk / moose 需要步幅调节和耐力
捕猎接触需要卸力
逃跑需要转向、跳跃、摔倒恢复
挖洞和进洞需要身体曲率变化
育幼需要蜷护、侧躺和姿态调整
社交需要身体姿态表达
```

**Implant Platform**
我们设计一套植入式人工骨骼和 powered artificial joint 系统。用途包括伤后重建、关节替换、骨骼结构恢复、运动能力恢复。

系统要恢复：

```text
load path
joint range
joint torque
impact survival
spine / limb coordination
wild behavior capacity
```

**Known Capabilities**
团队已经掌握或另行解决：

```text
神经接口
decoder
sensory feedback
power supply
heat rejection
infection barrier
long-term seal
bone-implant fixation
wire / flex fatigue
actuator service life
```

当前讨论集中在 actuator 和 joint architecture。

**Actuator Architecture**
主关节采用可力控旋转 actuator：

```text
BLDC motor
+ low/medium ratio reducer
+ output torque sensing
+ series elastic element
+ parallel elastic element when useful
+ mechanical hard stop
+ controllable clutch / brake
```

控制目标：

```text
torque control
impedance control
stiffness modulation
impact yielding
stance / swing behavior switching
```

**Joint Strategy**
This is this repo's engineering focus.

**Reducer Preference**
This is this repo's engineering focus.

**Torque Envelope**
每个关节需要定义：

```text
continuous torque
burst torque
impact survival torque
backdrivable / yielding threshold
thermal envelope
fatigue envelope
```

捕猎 elk/moose、追 coyote/bunny、跳落、急转、被撞击时，瞬时载荷会远高于正常步态载荷。actuator 必须用弹性、离合、阻尼、限位和控制策略处理这些峰值。

**Validation Standard**
实验室能走路还远远不够。放归级验证要覆盖：

```text
长距离奔跑
急转追逐
扑击和咬合接触
跳落和摔倒恢复
挖洞
进出巢穴
群体接触
育幼姿态
低温和野外地形
长期无护理运行
```

**Engineering Philosophy**
核心方向：

```text
proximal power
distal lightness
elastic impact survival
torque-controlled joints
stiffness modulation
distributed spine function
return-to-wild validation
```

---

上述文字给出这一层意图：

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


---

- Core tasks: ~10.

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