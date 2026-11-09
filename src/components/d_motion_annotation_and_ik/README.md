# Human motion annotation and IK

使用 Scuf Omega Wireless PlayStation joystick 进行人工控制与动作标注，为固定设计下的 controller 训练提供数据。任务与奖励由 [3K tasks component](../e_task_reward_rl/README.md) 定义。

把 b_mujoco_sim 形成的几何、质量和力矩模型接到控制侧，建立一个可测试的 IK / trajectory / safety scaffold。真的能稳定支撑整机。IK control 当前覆盖：

- 从脚端目标求 leg joint commands。
- 生成简单 trajectory primitives。
- 检查 joint limit margin。
- 复用 Basic motion mass / torque model 检查 continuous torque margin。
- 用 COM 投影和支撑脚端多边形做几何级 support polygon awareness。
- 输出 simulation-ready command tables，供后续 MuJoCo/export 使用。

IK control reuses:

- `vendor/reference/urdf/chihuahua/lists.md` as the geometry and joint-range source.
- `mass_model.py` for Basic motion mass, COM and free-space torque rows.
- The same leg and head chain conventions used by `pygame_mass_viewer.py`.

IK control should feed later iterations:

```text
IK/control unsafe pose
 -> update foot targets / gait primitive / joint limits
 -> update Basic motion torque estimate
 -> update FEM torque-replay load case if needed
 -> later validate in MuJoCo/contact
```


---


| 类型   | 输入          | 功能                 | 状态                    |
| ---- | ----------- | ------------------ | --------------------- |
| 左摇杆  | 前推／后拉       | 左后脚前移／后移           |                       |
| 左摇杆  | 左推／右推       | 左后脚左移／右移           |                       |
| 右摇杆  | 前推／后拉       | 右后脚前移／后移           |                       |
| 右摇杆  | 左推／右推       | 右后脚左移／右移           |                       |
| 方向键  | ↑／↓         | 左前脚前移／后移           |                       |
| 方向键  | ←／→         | 左前脚左移／右移           |                       |
| 正面键  | △／×         | 右前脚前移／后移           |                       |
| 正面键  | □／○         | 右前脚左移／右移           |                       |
| 左肩键  | L1          | 左侧最近操作腿：脚端上升       |                       |
| 左扳机  | L2          | 左侧最近操作腿：脚端下降       |                       |
| 右肩键  | R1          | 右侧最近操作腿：脚端上升       |                       |
| 右扳机  | R2          | 右侧最近操作腿：脚端下降       |                       |
| 背键   | 左外背键        | 腰部向左转（yaw）         |                       |
| 背键   | 右外背键        | 腰部向右转（yaw）         |                       |
| 背键   | 左内背键        | 腰部向上弯（pitch）       |                       |
| 背键   | 右内背键        | 腰部向下弯（pitch）       |                       |
| IMU  | 手柄左右转动      | 颈部 yaw，按相对校准姿态映射   | 待验证                   |
| IMU  | 手柄抬起／低下     | 颈部 pitch，按相对校准姿态映射 | 待验证                   |
| 侧键   | 左 SAX       | 按住：爪口打开；松开保持       | 按 comment 调整，待 review |
| 侧键   | 右 SAX       | 按住：爪口闭合；松开保持       | 按 comment 调整，待 review |
| G 键  | G1          | 未分配，取消颈部控制         | 按 comment 调整          |
| G 键  | G2          | 未分配，取消颈部控制         | 按 comment 调整          |
| G 键  | G3          | 未分配，取消爪口控制         | 按 comment 调整          |
| G 键  | G4          | 未分配，取消爪口控制         | 按 comment 调整          |
| G 键  | G5          | 撤销最近一次编辑           |                       |
| 摇杆按压 | L3：左摇杆垂直按下  | 最近操作腿：趾关节弯曲        | 待确认                   |
| 摇杆按压 | R3：右摇杆垂直按下  | 最近操作腿：趾关节伸展        | 待确认                   |
| 功能键  | **Options** | **暂停／继续仿真**        |                       |
| 功能键  | Create      | 开始／结束录制            |                       |
| 触控板  | 按下          | 保存当前全身关键帧          |                       |
| 触控板  | 左滑／右滑       | 暂停时：前一个／后一个关键帧     |                       |
| 系统键  | PS          | 系统保留               |                       |
| 配置键  | Profile     | 硬件配置；操作期间固定，不用于动作  |                       |


当前 open issues 如下：

|项目|尚待确定|
|---|---|
|**同模式输入兼容性**|IMU、独立 extra keys、触控板滑动能否在同一设备模式下同时读取；到货后测试 PC／PS5 模式及有线／接收器连接。|
|**运行平台**|最终在 Windows、macOS 还是 Linux 上运行；已查到的读取路径不能直接保证跨平台适用。|
|**额外键持续与并发输入**|按住／释放是否正确上报；多个额外键与摇杆同时操作时是否冲突、丢事件。|
|**触控板与 Create 独立性**|两者能否分别绑定“保存关键帧”和“录制”，同时读取滑动，避免固件默认映射造成重复触发。|
|**IMU 颈部控制**|数据可读取后，确定校准、角度比例、漂移处理，以及冻结／重新定中的操作。|
|**SAX 爪口控制**|左 SAX 打开、右 SAX 闭合已按您的 comment 纳入，尚待最终确认。|
|**趾关节控制**|L3／R3 分别弯曲／伸展最近操作腿的方案尚待确认；它们是左右摇杆的按压键。|
|**最近操作腿的边界规则**|同侧前后腿同时操作时如何判定；趾关节使用的全局最近腿如何判定。已确认的升降期间锁定规则保留。|
|**控制参数**|摇杆死区、移动速度、按键变化速度、扳机曲线，以及限位／不可达时的停止规则。|
|**动作数据与执行接口**|关键帧插值、录制内容、接触标注、IK 到扭矩控制及 RL 导出契约尚未完成详细设计与 sign-off。|

G1–G4 暂时空置，不承担运动操作。