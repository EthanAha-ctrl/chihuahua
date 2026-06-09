# Stage 4: MuJoCo Contact First Light

目标：把 Stage 1/2/3 的几何、质量、关节命令和支撑检查接到 MuJoCo/contact
侧，先建立可测试的 XML export、接触载荷表和 Stage 2 回灌接口。

Stage 4 first-light 不是完整行走仿真，也不是闭环 balance controller。它先回答：

```text
当前 Stage 3 姿态能不能导出为 MuJoCo-readable model？
脚端 contact geometry 和 friction 是否结构化？
准静态支撑力能否平衡 weight / COM moment？
接触载荷能否写成 Stage 2 后续复验可读的 load-case table？
```

## Scope

当前 Stage 4 覆盖：

- 从 Stage 3 trajectory frame 生成 rough MuJoCo XML。
- 从 Stage 2 rod graph 生成 whole-body capsule geometry。
- 默认 MuJoCo GUI 入口导出 21 个 actuator：waist yaw/pitch、四条腿的
  articulated chains，以及真实挂在前躯干上的 neck/head chain。每个默认
  control slider 都连接到可见 articulated geometry。
- 每条腿的四个 DOF 是串联 MuJoCo bodies：`hip_ab -> hip_pitch -> knee -> toe`。
  `hip_ab` 和 `hip_pitch` 不再堆在同一个 body 里。
- `head_claw` 遵循 pygame viewer / Stage 1 mass model 的语义：upper
  jaw 和 lower jaw 由同一个 control 反向联动，对称开合。
- 默认 actuator 是 MuJoCo torque motor，control slider 是 Nm 级扭矩命令，
  不是 position-servo angle target。
- 导出四个 foot contact geoms、collision-enabled structural capsule geoms
  和 ground plane。
- 默认 MuJoCo GUI 模型把 `mass_model.py` / `pygame_mass_viewer.py` 同源的
  `MassModel.elements` 分配到 live articulated bodies 上，而不是把总质量
  lump 到 root body。
- 用 COM 投影和支撑脚端位置求 quasi-static normal reaction proxy。
- `contact_loadcases.csv` 来自 MuJoCo `mj_contactForce` contact samples；
  准静态 proxy 另存为 `quasi_static_contact_proxy.csv`。
- 输出 Stage 2 feedback load-case CSV。
- 可选尝试 Python `mujoco` smoke simulation。

当前 Stage 4 不覆盖：

- closed-loop balance controller
- real gait dynamics
- actuator current / thermal state integration
- frictional tangential force solve
- collision-rich CAD geometry
- measured hip bearing spacing / servo housing geometry
- MuJoCo-to-FEM automatic load application
- proof that the robot can stand or walk

## Entry Point

Open the default Stage 4 stand model directly in MuJoCo's native GUI:

```bash
./view_mujoco.py
```

This launcher has no arguments. It refreshes the default Stage 4 MJCF in
`stage4_outputs/mujoco_viewer/` and opens MuJoCo's viewer. The launcher exports
a gravity-on GUI inspection model with four real foot contact geoms on the
ground plane. Visible structural capsules are also collision-enabled. It does
not use invisible foot-anchor bodies or foot-pin equalities. The control panel
intentionally contains only joints wired to visible articulated geometry; there
are no payload balls, joint marker balls, or detached fake sliders in the
default viewer model. Its inertials are distributed from Stage 1
`MassModel.elements`: body shells, payload/electronics, actuators, links, foot
pads, neck, and jaws are assigned to the corresponding live MuJoCo bodies.

Expected outputs:

```text
mujoco_model.xml
mujoco_model.mjcf
contact_loadcases.csv
quasi_static_contact_proxy.csv
stage2_feedback_loadcases.csv
stage4_mujoco_contact_summary.yaml
```

`mujoco_model.mjcf` is the MuJoCo description alias for the same MJCF XML
content. Stage 4 no longer writes a `.mjsd` alias because that extension was
only the same XML under a misleading name.

## Contact Proxy

`contact_loadcases.csv` is sampled from MuJoCo after loading the exported MJCF.
Each row is one MuJoCo contact point at one simulation step, with geom/body
names, contact position, contact-frame normal/tangential force components, and
the contact friction coefficient.

`quasi_static_contact_proxy.csv` is still written separately. For each Stage 3
frame, Stage 4 solves a small quasi-static support problem:

```text
sum(Fz_i)       = mass * gravity
sum(Fz_i * x_i) = mass * gravity * COM_x
sum(Fz_i * y_i) = mass * gravity * COM_y
Fz_i >= 0
```

This produces candidate normal reactions at support feet. It is deliberately
not a dynamic contact solve. Tangential force is currently zero, so friction
utilization is only a placeholder field.

Frames with COM outside the support polygon or high force-balance residual are
marked not statically solvable.

## Stage 2 Feedback

`stage2_feedback_loadcases.csv` writes one row per nonzero support reaction:

```text
stage2_node = <leg>_toe_endpoint
force_z_n  = contact normal force
source     = stage4_quasi_static_contact_proxy
```

These rows are candidate inputs for later Stage 2 no-gravity structural replay.
They are not automatically applied to Stage 2, and they should not be mixed into
the Stage 2 native torque-replay case without an explicit load-case policy.

## Relationship To Earlier Stages

Stage 4 reuses:

- `dog_description.yaml` for geometry and joint limits.
- `mass_model.py` for mass, COM and inertia.
- `stage2_rod_model.py` for whole-body rod/capsule topology.
- `stage3_ik_control.py` for trajectory frames and support-leg state.

The loop is:

```text
Stage 3 pose / command
 -> Stage 4 contact proxy / MuJoCo XML
 -> contact or simulation failure
 -> update Stage 1 mass / torque
 -> update Stage 2 structural load case
 -> update Stage 3 control limits
 -> Stage 4 again
```

## Acceptance

The Stage 4 first-light acceptance command is:

```bash
uv run python -m unittest tests/test_stage4_mujoco_contact.py
```

The full repo test suite should also pass:

```bash
uv run python -m unittest discover -s tests
```
