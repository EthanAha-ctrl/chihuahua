# Stage 2: Whole-Robot FEM

目标：用 **OpenRadioss** 建立 whole-robot FEM 迭代系统，让整机结构强度、刚度、质量、COM、惯量和力矩进入同一个闭环。

这一阶段只做 whole-robot FEM。不建立 single-link、bracket、shaft seat、insert region 或 subassembly 的独立 FEM 分析路线。局部问题必须通过整机模型暴露、修改和复验。

外部工具依赖记录在 `docs/EXTERNAL_DEPENDENCIES.md`。Stage 2 依赖 OpenRadioss、OpenRadioss result conversion 和 ParaView，但仓库不记录本机安装路径、环境变量或自动探测逻辑。

OpenRadioss 的本地 first-light 复现实验记录在 `docs/STAGE_2_OPENRADIOSS_FIRST_LIGHT.md`，包括 release 校验、官方 tensile example、Starter/Engine 运行、T01 转 CSV 和 GIF 可视化。

当前 Stage 2 的第一份 Chihuahua-specific 输入模型来自 `pygame_mass_viewer.py`
同源拓扑：整机被抽象成 connected rod graph。生成入口是
`stage2_rod_model.py`，输出 `nodes.csv`、`members.csv`、
`lumped_masses.csv`、`rod_model_summary.yaml` 和杆系 GIF。

```bash
uv run python stage2_rod_model.py --out-dir stage2_outputs/rod_model
```

这个 rod model 仍然是整机模型：body、waist、hips、four legs、feet、neck
和 head claw 一起进入同一个结构图。hip cross-link 在 solver topology 中被
拆成 center-to-hip rods，以保证 body spine 和 hip frame 连通；这不是局部
subassembly 分析。

这一步只导出 topology、质量和 toe-node tags。toe 节点可以保留
contact-candidate 语义，方便后续 MuJoCo/contact 阶段复用，但 Stage 2
不施加 gravity、support reaction、ground contact 或 fixed boundary
condition。

OpenRadioss whole-body beam deck 由同一份 rod graph 生成：

```bash
uv run python stage2_openradioss_deck.py \
  --out-dir stage2_outputs/openradioss_beam_deck
```

这个 deck 使用 `/BEAM`、`/PROP/TYPE3`、`/PART`、`/MAT/LAW1` 和
`/ADMAS/5`。它仍然是 topology-only smoke deck：没有 gravity、没有 fixed
feet、没有 support reaction、没有 external load。Engine file 只用于验证
Starter/Engine 链路能接收 whole-body beam deck，不产生设计结论。

第一份 solved deformation FEM route 是 viewer-periodic whole-body rod
motion case。当前默认 load policy 是 `stage1-torque-replay`：从同一帧
`pygame_mass_viewer.py` / Stage 1 model 计算 joint torque rows，再把每个
joint torque 写成 OpenRadioss `/CLOAD` equal-and-opposite nodal moment
couple。

```bash
uv run python stage2_openradioss_periodic_motion.py \
  --out-dir stage2_outputs/openradioss_periodic_motion \
  --solver-duration-ms 8 \
  --viewer-motion-seconds 0.005 \
  --target-element-length-mm 8 \
  --uniform-radius-mm 8 \
  --control-policy stage1-torque-replay
```

这个 case 复用 `pygame_mass_viewer.py` 的 constrained periodic motion，
但 FEM GIF 不播放完整秒级 gait period。它只取周期中的一个短时间片
（默认 `0.005 s`），映射到毫秒级 FEM 窗口（默认 `8 ms`），因此视觉上应几乎
看不到整机转动，主要读取 element-by-element tension map。

同一套 whole-body rod graph 会被切成 elementized `/BEAM` mesh，默认目标
element length 是 `8 mm`，默认 beam section 是 uniform circular radius
`8 mm`。25 个原始 whole-body joint nodes 都保留在 FEM mesh 里，并且语义上
统一都是 joint nodes。`stage1-torque-replay` 不使用 `/IMPDISP`、不使用
ghost target nodes、不使用 guide springs；pose samples 只用于计算 Stage 1
torque rows 和输出 reference CSV，不作为运动边界强推给 robot nodes。
旧的 `uniform-joint-guides` 和 `all-joints-hard` 策略只作为对照/调试开关保留。

它是整机 torque-replay FEM load case：没有 gravity、没有 contact、没有 fixed feet。
结果通过
`/TH/NODE D`、`/TH/NODE REACX REACY REACZ`、`/TH/BEAM F1 M2 M3`、
`/ANIM/VECT/DISP` 和 `/ANIM/VECT/FREAC` 输出。后处理优先从 T01 beam
section resultants 还原 element-by-element outer-fiber stress/strain：
`sigma = F1/A +/- M*r/I`，再用弹性模量换算 strain。旧的 nodal-displacement
outer-fiber strain proxy 只作为没有 `/TH/BEAM` 的旧 run fallback。

Periodic route 默认使用 uniform radius，所以 tension/bend map 不再混入
body、waist、leg、toe 截面半径差异。若需要回到 mass-derived 或 nominal
radius，可显式改变 `--uniform-radius-mm` / radius policy，但那属于对照实验。

当前已验证的 whole-body torque replay run：

```text
run dir: /mnt/s8t/openradioss/runs/chihuahua_stage2_stage1_torque_replay_radius12_8mm_native_beam
control policy: stage1-torque-replay
solver nodes: 305
beam elements: 304
beam resultant history elements: 304
imposed displacement functions: 0
concentrated moment functions: 76
stage1 torque replay joints: 21
stage1 torque replay moment couples: 22
Starter: 0 errors, 0 warnings
Engine: normal termination, 3233 cycles, 8.001 ms
GIF: openradioss_whole_body_periodic_motion.gif
Poster: openradioss_whole_body_periodic_motion_poster.png
```

这一 run 的 hottest element 位于 `waist_yaw_pitch_elem_0001` 腰段；poster
frame 的最大 displacement 约 `0.175 mm`，最大 Radioss beam F/M-derived
outer-fiber strain 约 `2.20e-4`，最大 outer-fiber stress 约 `0.703 MPa`。
这符合当前阶段的预期：FEM 窗口是毫秒级，视觉上几乎不应看到
大幅整机转动，主要阅读 element-by-element tension/bend map。

## 1. FEM 输入模型

- 读取 `dog_description.yaml`
- 读取 `pygame_mass_viewer.py` / Stage 1 linkage 同源生成的 whole-body rod graph
- 读取材料、打印方向、rod radius / mesh / lumped-mass 参数
- 读取 actuator、battery、electronics 的 mass / COM / inertia / placement
- 建立整机 structural model
- 保留 whole-body joint nodes 作为拓扑语义点；不假设具体 joint hardware geometry
- 每个设计版本记录 mass / COM / inertia / torque / FEM 假设

## 2. 分析对象

- full robot body
- full leg structures
- waist yaw / pitch structure
- hip frame and leg attachment regions
- actuator and battery lumped masses
- whole-body joint nodes as topology/load-path markers, not bearing or fastener geometry
- toe nodes as geometry tags only, with no ground contact or support reaction

## 3. Load Case Schema

- Stage 2 load case 来自 Stage 1 的质量、COM、惯量、姿态和 joint torque
  估计
- Stage 2 不包含 gravity、ground contact、support reaction、fixed feet 或
  standing-static support solve
- 每个 load case 记录：来源、单位、方向、施加载体、边界条件和姿态
- 每个 load case 必须可回写 mass / COM / inertia / torque 更新后的新版本
- MuJoCo/contact 阶段产生的 gravity/contact/support 结果，后续只能作为新输入
  回灌 Stage 2；它们不属于当前 Stage 2 原生工况

核心 load cases：

- crouched worst-torque pose
- waist yaw extreme pose
- waist pitch extreme pose
- torque replay articulated bend
- actuator torque reaction
- battery / electronics lumped mass inertia in the same torque replay
- mesh / material / radius comparison under the same load

## 4. OpenRadioss Case Generation

- 生成 whole-robot OpenRadioss 输入文件
- 每个设计版本 / 姿态 / 工况单独目录
- 支持 torque-replay、moment-couple reaction、motion-slice inertia 和 transient case
- 保留 actuator、battery、electronics 的 lumped mass 和 inertia
- 保留 joint-node 拓扑标签和 rod connectivity 假设
- 输出可追溯的 model assumptions table

## 5. FEM 检查项

- global body deformation
- body torsion
- waist load path weakness
- hip frame deformation
- joint axis misalignment
- endpoint deflection under load
- whole-body torque / inertia load distribution
- torque reaction load path
- stress hot spots
- layer delamination risk
- fatigue-sensitive regions
- joint-node load-path consistency
- rod-to-joint load transfer hot spots

## 6. 强化策略

- wall thickness change
- rib placement
- fillet radius
- hole relocation
- boss geometry
- print orientation change
- material change
- load path redesign
- standard part substitution
- full-frame stiffness redistribution
- actuator / battery placement change
- joint spacing or axis placement change

所有强化都必须在 whole-robot model 中复验。不从整机 FEM 分叉出独立 local FEM 流程。

## 7. 闭环

```text
whole-robot FEM stress / deformation hot spot
-> full-model reinforcement
-> mass update
-> COM / inertia update
-> torque update
-> new whole-robot load case
-> whole-robot FEM again
```

## 8. 输出

- whole-robot OpenRadioss case files
- whole-robot load-case table
- VTK visualization files
- CSV time-series files
- ParaView screenshots
- material / print-orientation assumptions
- joint-node topology and rod connectivity assumptions
- global deformation report
- endpoint deflection report
- joint axis misalignment report
- stress / deformation / margin report
- hot-spot table
- joint-node load-path table
- mass delta
- torque impact
- design decision log
