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