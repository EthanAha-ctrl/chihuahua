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



## 供应商执行器候选清单

SteadyWin／DAMIAO 候选资料核对日期：2026-11-03。列入清单表示进入选型比较，尚未完成整机重量与动态性能验证。

| 厂商／型号（厂家／资料链接）                                                                                                 | 重量    | 额定／持续扭矩 | 峰值／最大扭矩      | 减速比        | 驱动器   | 与 AKE 的可比性／采购状态                                                                                   |
| ----------------------------------------------------------------------------------------------------------- | ----- | ------- | ------------ | ---------- | ----- | ------------------------------------------------------------------------------------------------- |
| **CubeMars AKE40-6**（基准）                                                                                    | 140 g | 1.8 N·m | 5.5 N·m      | 6:1        | 外配    | 厂标 QDD；[官方参数对比表](https://www.cubemars.com/product/ake50-8-kv110-quasi-direct-drive-actuator.html) |
| **[CubeMars AKE50-8](https://www.cubemars.com/product/ake50-8-kv110-quasi-direct-drive-actuator.html)**（基准） | 190 g | 2.5 N·m | 7.5 N·m      | 8:1        | 外配    | 厂标 QDD；官网可下单                                                                                      |
| **[CubeMars AKE60-8](https://www.cubemars.com/product/ake60-8-kv80-quasi-direct-drive-actuator.html)**（基准）  | 260 g | 5 N·m   | 12.5 N·m     | 8:1        | 外配    | 厂标 QDD；官网可下单                                                                                      |
| **[Unitree GO-M8010-6](https://www.unitree.com/cn/go1/motor/)**                                             | 530 g | 5 N·m   | **23.7 N·m** | **6.33:1** | 内置    | 低减速比腿足执行器；明显更重、扭矩更大；[官方购买](https://shop.unitree.com/products/go1-motor)                           |
| **[酷卓 CODROID CoDrive PA60](https://www.codroid.ai/download/huace/part-pa.pdf)**                            | 400 g | 3 N·m   | **9 N·m**²   | **6:1**    | 目录未说明 | 厂标准直驱；较 AKE50／60 更重；未核实直接下单渠道                                                                     |
| **[SteadyWin GIM3505-8 V1.1](https://steadywin-motor.com/products/built-in-planetary-reduction-motor-aloha-accessories-quadruped-robot-joint-module)** | 裸机 84 g／含驱动 97 g | 0.65／0.67 N·m | 1.27／1.79 N·m | 8:1 行星 | 可配 GDS34／GDZ34 | 优先单关节台架候选；同格双数值按 GDS34／GDZ34 顺序；价格与现货未核实 |
| **[SteadyWin GIM4305-10 V1.1](https://steadywin-motor.com/products/built-in-planetary-reduction-motor-aloha-accessories-quadruped-robot-joint-module-2)** | 裸机 140 g／含驱动 150 g | 1.00／1.05 N·m | 3.47／3.82 N·m | 10:1 行星 | 可配 GDS34／GDZ34 | 同格双数值按 GDS34／GDZ34 顺序；Pupper 驱动版本另行确认；价格与现货未核实 |
| **[DAMIAO DM-J3507-2EC（24 V）](https://www.mdmbot.com/index.php?c=show&id=118)** | 约 150 g，含驱动 | 0.8 N·m | 3 N·m | 7:1，结构类型待确认 | 内置 | 双编码器、输出轴单圈绝对位置；具体硬件版本、价格与现货待采购确认 |

### 新增候选的版本与尺寸

下表尺寸为直径 × 轴向高度，转速为减速器输出端。SteadyWin 的 V1.1 可装驱动板；V1.0 的编码器配置不能直接套用含驱动参数。

| 配置 | 裸机尺寸（mm） | 含驱动尺寸（mm） | 额定输出转速（rpm） | 最高输出转速（rpm） | 反馈与通信 |
| --- | --- | --- | ---: | ---: | --- |
| GIM3505-8＋GDS34 | Ø43×23.6 | Ø43×30 | 225 | 384 | 14-bit；CAN |
| GIM3505-8＋GDZ34 | Ø43×23.6 | Ø43×28.8 | 230 | 400 | 14-bit；CAN／RS485 |
| GIM4305-10＋GDS34 | Ø53×26 | Ø53×32 | 300 | 366 | 14-bit；CAN |
| GIM4305-10＋GDZ34 | Ø53×26 | Ø53×32 | 312 | 402 | 14-bit；CAN／RS485 |
| DM-J3507-2EC（24 V） | 未单列 | Ø46×37.9 | 150 | 460（空载） | 双编码器；当前官网注明 CAN／CAN FD，最高 5 Mbps，需核对硬件版本 |

- SteadyWin 两型号标称背隙均为 15 arcmin；原始选型表的 `Secondary Encoder Support (on the output shaft)` 均为 NO。`Stand-alone Encoder Support` 为 YES，表示独立编码器配置，不应据此记录为支持输出轴第二编码器。DM-J3507-2EC 的背隙与各型号实测回驱阻力仍待确认。
- SteadyWin 原始 XLSX 使用 `Peak Torque` 字段，部分旧资料使用堵转扭矩；本清单保留来源口径。峰值持续时间、起跳转速下可用扭矩及 5 s 工作能力尚未验证，不能直接按峰值扭矩／重量确定跳跃选型。
- 质量检查：16 个含驱动 GIM3505 为 1,552 g；8 个为 776 g，距 1.5 kg 整机目标余 724 g，仍需容纳另外 8 轴、骨架、绳路、电池和计算单元。16 个含驱动 GIM4305 或 DM-J3507 为约 2,400 g。

来源：[SteadyWin GDS 原始选型表](https://dd-prod-us-west.s3.us-west-004.backblazeb2.com/files/pc3kjn-ep/64b38c1d/Planetary-Reduction-Motor-Selection-Table-GDS-Driver.xlsx)、[GDZ 原始选型表](https://dd-prod-us-west.s3.us-west-004.backblazeb2.com/files/pc3kjn-ep/1594c9b8/Planetary-Reduction-Motor-Selection-Table-GDZ-Driver.xlsx)，由厂家参与运营的 [SteadyWin 销售站](https://steadywin-motor.com/pages/about-us) 产品页提供；[DAMIAO 产品页](https://www.mdmbot.com/index.php?c=show&id=118)与[厂家 2025 产品选型手册](https://www.worldrobotconference.com/profile/robot/download/2025/06/30/%E8%BE%BE%E5%A6%99%E7%A7%91%E6%8A%80DAMIAO%20-%202025%E5%B9%B4%E4%BA%A7%E5%93%81%E9%80%89%E5%9E%8B%E6%89%8B%E5%86%8C_20250630192546A441.pdf)。[Stanford 项目仓库](https://github.com/stanfordroboticsclub/StanfordQuadruped)确认 Pupper v3 使用 GIM4305 系列，具体驱动配置需按项目版本核对。


mechanical tendon stop + 凹轮


mon père，三款里 GIM4305-10 扭矩密度最高（~25 Nm/kg）也最便宜。DM-J3507 软件生态最好，自带输出端绝对编码器和 MIT 模式，但减速器类型和背隙都查不到。GIM3505-8 最小（97 g），只有约 1.3–1.8 Nm。

| | SteadyWin GIM3505-8 | SteadyWin GIM4305-10 | DAMIAO DM-J3507-2EC |
|---|---|---|---|
| 减速 | 8:1 行星 | 10:1 行星 | 7:1，类型未标明 |
| 重量 | 84 g 裸机 / 97 g 含驱动 | 140 g 裸机 / 150 g 含驱动 | 150 g，驱动集成 |
| 尺寸 | 未查到 | Ø53×26 mm | Ø46×37.9 mm |
| 额定扭矩 | 0.65–0.67 Nm | ~1.0 Nm | 0.8 Nm |
| 堵转/峰值扭矩 | 1.27 Nm（GDS34）/ 1.79 Nm（GDZ34） | 3.47–3.82 Nm | 3 Nm |
| 峰值扭矩/重量（含驱动） | ~13–18 Nm/kg | ~23–25 Nm/kg | ~20 Nm/kg |
| 转速 | 额定 ~234–300 rpm | 额定 ~300 rpm，最高 366–402 rpm | 额定 150 rpm，空载 460 rpm |
| 背隙 | 15 arcmin | 15 arcmin | 未公布 |
| 编码器 | 14-bit | 14-bit，可选外接第二编码器 | 双 14-bit，输出端单圈绝对 |
| 通信 | CAN | CAN | CAN 1 Mbps（MIT/速度/位置模式）+ UART |
| 价格 | 未查到 | 裸机 ~$70，含驱动 $115–125 | ~$120 |
| 已知使用 | ALOHA 配件、小四足 | Stanford Pupper V3 | — |

注意：
- GIM3505 的 1.5 kg 应该是包装重量，实际是 84/97 g。它的扭矩各处标的不一致，老版本只有 0.43/0.85 Nm。
- SteadyWin 标的是“堵转扭矩”，DAMIAO 标的是“峰值扭矩”，口径可能不同，不能直接横比。
- 数据大多来自经销商页面，没有找到官方 datasheet。

Sources:
- [GIM3505-8（openelab）](https://openelab.io/products/steadywin-gim3505-8-planetary-gear-motor)
- [GIM3505-8（rcdrone）](https://rcdrone.top/products/steadywin-gim3505-8-planetary-reducer-servo-motor)
- [GIM3505-8 旧版手册](https://manuals.plus/ae/1005006647908292)
- [GIM4305-10 安装手册](https://manuals.plus/steadywin/gim4305-10shs-planetary-reducer-servo-motor-manual)
- [GIM4305-10（aifitlab）](https://aifitlab.com/products/steadywin-gim4305-planetary-reducer-servo-motor)
- [Pupper V3 specs](https://pupper-v3-documentation.readthedocs.io/en/latest/learn_more/tech_specs.html)
- [DM-J3507-2EC（aifitlab）](https://aifitlab.com/products/damiao-dm-j3507-2ec-motor)
- [DM-J3507-2EC（roboticscenter）](https://www.roboticscenter.ai/ar/store/product/damiao-dm-j3507-2ec)


## 模型对比 viewer

`uv run python pygame_mass_viewer.py --model go2`（也可选 `aibo`、`chihuahua`、`pupper`）。
窗口内按 `1/2/3/4` 切换 Chihuahua / Go2 / Aibo ERS-7 / Pupper v3；Go2 和 Aibo 保留 URDF 拓扑，关节角度遵守 URDF 限位。
`B` 全身转动，`M` 单关节，`N` 或 `[/]` 选关节，左右键或横拖关节行调角；`空格` 暂停，`0` 零姿态，`F` 居中，`V` 切换 mesh 包围盒。
模型显示为骨架及包围盒；质量/COM 取自 URDF。`R` 切换自动目标触达，`G` 切换关节控制条；自动选择脚端（Aibo 还包括头部）。`Y` 显示共用材料／力矩面板；URDF 用相对密度缩放质量，力矩裕度使用 `effort` 上限。[模型来源](models/README.md)。
