# FEM / FEA

## Current requirements

本组件读取 MuJoCo 的 episode 状态和载荷输出，负责选择用于 FEA 的 snapshot. 每个 episode 按以下规则选取：

1. **关节侧驱动力矩峰值**：读取 MuJoCo `qfrc_actuator` 中被驱动转动关节对应的值，在所有这些关节、所有采样时刻中比较绝对值，取全局最大值所在的一个时刻。该值包含模型中凹轮传动关系的作用，表示执行器对关节的驱动力矩，不等于完整关节反力。
2. **接触面合力峰值**：将同一接触面上各接触点的力转换到同一坐标系后向量求和，比较合力的大小；在整个 episode 的各接触面、各采样时刻中，取最大值所在的一个时刻。
3. **去重**：如果上述两个峰值对应同一采样时刻，只保留一个 snapshot；否则保留两个。

每个 snapshot 保留对应时刻的模型状态、MuJoCo 载荷以及峰值对应的关节或接触面，供后续载荷映射和传力路径分析使用。选择过程只使用 MuJoCo 输出，不额外进行失效风险预测或 FEA 预计算。

用户可以选择：

- 整机分析。
- 局部分析：覆盖峰值对应的整条传力路径。
- 两者都做。

局部分析不预先宣称某个部件最容易损坏。FEA 结果用于判断结构响应，并反馈 geometry / material 优化。

从同一 snapshot 的模型状态和载荷构造工况。需明确载荷坐标系、单位、作用位置，以及 MuJoCo bodies / joints / contacts 到 FEM 节点或区域的映射。
整机和局部模型分别需要明确边界条件；局部截取边界须表达被截去结构的作用。
重力、接触、驱动和惯性如何进入所选结构分析，应由明确的载荷映射实现处理，避免遗漏或重复计入；本文不指定尚未确认的求解近似。


AI coding agent 负责把凹轮传动关系正确建进 MuJoCo 模型。选 snapshot 只用 MuJoCo 输出，不增加 FEA 前的失效风险预测或外部载荷估算。

每个 episode：

1. 在所有被驱动的转动关节、所有采样时刻中，取关节侧驱动力矩绝对值最大的一个时刻。使用 `qfrc_actuator` 的对应转动自由度；它表示模型传动后的执行器贡献，不等于完整关节反力。
2. 将同一接触面的各接触点力转换到同一坐标系后向量求和，按合力大小取峰值时刻。多个接触面之间取整个 episode 的最大值。
3. 两个时刻相同只保留一个 snapshot，否则保留两个。
4. 用户选择整机 FEA、局部 FEA 或两者。局部分析覆盖峰值对应的整条传力路径。

这些峰值用于快速筛选工况，不宣称已经找到最大应力或最容易损坏的部件；由随后 FEA 检验。
接触面的模型分组、传力路径提取、载荷映射及边界条件需要在实现中明确表达，不能靠静默默认值补齐。

---

# FEM OpenRadioss First Light

整机结构，而是确认 FEM 的外部 solver 链路真的可用：

```text
download OpenRadioss
-> verify binary archive
-> run official Starter
-> run official Engine
-> convert time history to CSV
-> make a GIF from mesh + result history
```

## Scope

本实验使用官方 OpenRadioss release 和官方 tensile-test example。

不把 solver 二进制、官方 example zip、H3D、restart、CSV 或 GIF 结果提交进
repo。它们是本机 sandbox artifacts。

官方 example deck 头部声明模型许可证为 `CC BY-NC 4.0`。不要把该模型当成
本仓库自有资产。

## Sources

- OpenRadioss release:
  `https://github.com/OpenCourant/extlib/releases/tag/upstream-latest-20260728-linux64`
- Linux binary:
  `https://github.com/OpenCourant/extlib/releases/download/upstream-latest-20260728-linux64/OpenRadioss_linux64.zip`
- Official running docs:
  `https://openradioss.atlassian.net/wiki/spaces/OPENRADIOSS/pages/19628079/Running+OpenRadioss`
- Official tensile test page:
  `https://openradioss.atlassian.net/wiki/spaces/OPENRADIOSS/pages/11075620/Tensile+Test`
- Tensile test Section 1 zip:
  `https://openradioss.atlassian.net/wiki/download/attachments/11075620/Tensile_Test_Section1.zip?api=v2`

## Known-Good Versions

Solver package (tensile results below are from the original `latest-20260520` run):

```text
OpenRadioss tag: latest-20260728
archive: OpenRadioss_linux64.zip
sha256: 598ed7b2905a7bacc8d1781470c250ac79d7558c39ba962768edf3644644fe33
platform: Linux x86_64
example: Tensile_Test_Section1.zip / 1_LAW2
```

## Directory Layout

Use a sandbox outside this repository:

```bash
export OR_ROOT=/mnt/s8t/openradioss
export OR_TAG=latest-20260728
mkdir -p "$OR_ROOT/downloads" "$OR_ROOT/runs"
```

Expected layout after the steps below:

```text
/mnt/s8t/openradioss/
  downloads/
    OpenRadioss_linux64.zip
  latest-20260728/
    OpenRadioss/
      exec/
        starter_linux64_gf
        engine_linux64_gf
        th_to_csv_linux64_gf
        anim_to_vtk_linux64_gf
      hm_cfg_files/
      extlib/
  runs/
    tensile_section1/
      1_LAW2/
        tensile_LAW2_0000.rad
        tensile_LAW2_0001.rad
        tensile_LAW2.h3d
        tensile_LAW2T01
        tensile_LAW2T01.csv
        visualization/
          openradioss_tensile_LAW2.gif
```

## Install OpenRadioss

Download and verify the Linux binary package:

```bash
cd "$OR_ROOT/downloads"
curl -L --fail --show-error \
  -o OpenRadioss_linux64.zip \
  "https://github.com/OpenCourant/extlib/releases/download/upstream-$OR_TAG-linux64/OpenRadioss_linux64.zip"

sha256sum OpenRadioss_linux64.zip
```

Expected hash:

```text
598ed7b2905a7bacc8d1781470c250ac79d7558c39ba962768edf3644644fe33  OpenRadioss_linux64.zip
```

Unzip:

```bash
rm -rf "$OR_ROOT/$OR_TAG"
mkdir -p "$OR_ROOT/$OR_TAG"
unzip -q "$OR_ROOT/downloads/OpenRadioss_linux64.zip" -d "$OR_ROOT/$OR_TAG"
```

Check binaries:

```bash
find "$OR_ROOT/$OR_TAG/OpenRadioss/exec" -maxdepth 1 -type f \
  \( -name 'starter*' -o -name 'engine*' -o -name '*to_csv*' -o -name '*to_vtk*' \) \
  -print
```

## Environment

For the release package, `starter_linux64_gf` needs the bundled HyperMesh reader
library in `LD_LIBRARY_PATH`.

```bash
export OPENRADIOSS_PATH="$OR_ROOT/$OR_TAG/OpenRadioss"
export RAD_CFG_PATH="$OPENRADIOSS_PATH/hm_cfg_files"
export RAD_H3D_PATH="$OPENRADIOSS_PATH/extlib/h3d/lib/linux64"
export OMP_STACKSIZE=400m
export OMP_NUM_THREADS=2
export LD_LIBRARY_PATH="$OPENRADIOSS_PATH/extlib/hm_reader/linux64:$OPENRADIOSS_PATH/extlib/h3d/lib/linux64:${LD_LIBRARY_PATH:-}"
```

## Download Official Example

```bash
mkdir -p "$OR_ROOT/runs/tensile_section1"
cd "$OR_ROOT/runs/tensile_section1"

curl -L --fail --show-error \
  -o Tensile_Test_Section1.zip \
  "https://openradioss.atlassian.net/wiki/download/attachments/11075620/Tensile_Test_Section1.zip?api=v2"

unzip -q -o Tensile_Test_Section1.zip
find . -maxdepth 2 -type f | sort
```

The Section 1 package contains four cases. The first-light run uses:

```text
1_LAW2/tensile_LAW2_0000.rad
1_LAW2/tensile_LAW2_0001.rad
```

## Run Starter And Engine

```bash
cd "$OR_ROOT/runs/tensile_section1/1_LAW2"

"$OPENRADIOSS_PATH/exec/starter_linux64_gf" \
  -i tensile_LAW2_0000.rad \
  -np 1 | tee starter.log

"$OPENRADIOSS_PATH/exec/engine_linux64_gf" \
  -i tensile_LAW2_0001.rad | tee engine.log
```

Successful Starter output includes:

```text
NORMAL TERMINATION
0 ERROR(S)
0 WARNING(S)
```

Successful Engine output includes:

```text
NORMAL TERMINATION
TOTAL NUMBER OF CYCLES  :  235922
```

The first successful local run produced:

```text
Starter elapsed time: 0.34 s
Engine elapsed time: 117.46 s
Total memory used: 50 MB
Total disk space used: 4 MB
H3D frames: 81
```

## Convert Time History To CSV

```bash
cd "$OR_ROOT/runs/tensile_section1/1_LAW2"
"$OPENRADIOSS_PATH/exec/th_to_csv_linux64_gf" tensile_LAW2T01 | tee th_to_csv.log
```

Expected output file:

```text
tensile_LAW2T01.csv
```

The CSV exposes global energy/mass/time-step histories, measuring-node values,
and rigid-body reaction values. The first local run ended with approximately:

```text
time: 40.00013 ms
node 616 displacement x relative to node 102: 35.38375 mm
rigid-body reaction x: 508.4768 N
plastic work: 507.3165 Nmm
```

## Make The GIF

After the CSV exists, generate a visual first-light artifact from this repo:

```bash
cd /mnt/s8t/chihuahua
uv run python -m src.components.c_fem.fem_tools.openradioss_tensile_gif \
  --run-dir "$OR_ROOT/runs/tensile_section1/1_LAW2"
```

Expected outputs:

```text
$OR_ROOT/runs/tensile_section1/1_LAW2/visualization/openradioss_tensile_LAW2.gif
$OR_ROOT/runs/tensile_section1/1_LAW2/visualization/openradioss_tensile_LAW2_poster.png
```

Important limitation:

```text
The GIF uses the .rad mesh and T01 displacement/force/energy histories.
Its deformation is an interpolated visualization proxy, not a full H3D nodal
displacement reconstruction.
```

## Why This Matters

This run proves the FEM toolchain can already do the essential loop:

```text
OpenRadioss Starter
-> OpenRadioss Engine
-> H3D / T01 output
-> CSV conversion
-> scriptable visualization
```

The next Chihuahua-specific FEM milestone should be the viewer-derived
whole-body rod model, not a single-part or subassembly FEM route:

```text
pygame_mass_viewer.py / Basic motion MassModel topology
-> connected body + waist + hips + legs + feet + neck + head rods
-> lump actuator, battery, electronics, and foot-pad masses
-> OpenRadioss whole-robot beam / rod deck
-> FEM load cases replay Basic motion torque / inertia without gravity or contact
-> CSV and FEM visualization artifacts
```

Only after that should FEM graduate to whole-robot OpenRadioss case
generation.

## Chihuahua Whole-Body Beam Deck Smoke Run

The first Chihuahua-specific OpenRadioss deck generator is:

```bash
cd /mnt/s8t/chihuahua
uv run python -m src.components.c_fem.openradioss_deck \
  --out-dir "$OR_ROOT/runs/chihuahua_fem_whole_body_beam"
```

It exports a topology-only whole-body beam deck:

```text
fem_whole_body_beam_0000.rad
fem_whole_body_beam_0001.rad
openradioss_beam_deck_summary.yaml
openradioss_whole_body_beam_deck.gif
openradioss_whole_body_beam_deck_poster.png
```

The deck uses `/BEAM`, `/PROP/TYPE3`, `/PART`, `/MAT/LAW1`, and `/ADMAS/5`.
It has no gravity, no fixed feet, no support reaction, and no external load.

Run the smoke check:

```bash
cd "$OR_ROOT/runs/chihuahua_fem_whole_body_beam"

"$OPENRADIOSS_PATH/exec/starter_linux64_gf" \
  -i fem_whole_body_beam_0000.rad \
  -np 1 | tee starter.log

"$OPENRADIOSS_PATH/exec/engine_linux64_gf" \
  -i fem_whole_body_beam_0001.rad | tee engine.log

"$OPENRADIOSS_PATH/exec/th_to_csv_linux64_gf" \
  fem_whole_body_beamT01 | tee th_to_csv.log
```

The first successful smoke run produced:

```text
Starter: NORMAL TERMINATION, 0 ERROR(S), 0 WARNING(S)
Engine: NORMAL TERMINATION, TOTAL NUMBER OF CYCLES: 4
T01 CSV: fem_whole_body_beamT01.csv
OpenRadioss deck mass: 2.492834 kg
```

This smoke run proves deck generation and solver ingestion only. It is not a
gravity, support, contact, deformation, stress, or design-margin result.

## Chihuahua Periodic-Motion Rod FEM

The first solved whole-body rod deformation case is generated from the same
`pygame_mass_viewer.py` periodic motion sampler. The current default solved
route is Basic motion torque replay: per-frame Basic motion joint torque rows are written
as equal-and-opposite OpenRadioss `/CLOAD` nodal moment couples. The sampled
viewer pose is a torque/reference source, not a hard motion boundary.

```bash
cd /mnt/s8t/chihuahua
uv run python -m src.components.c_fem.openradioss_periodic_motion \
  --out-dir "$OR_ROOT/runs/chihuahua_fem_basic_motion_torque_replay_radius12_8mm_native_beam" \
  --samples 25 \
  --solver-duration-ms 8 \
  --viewer-start-seconds 0.2 \
  --viewer-motion-seconds 0.005 \
  --motion-scale 1.0 \
  --target-element-length-mm 8 \
  --uniform-radius-mm 12 \
  --control-policy basic_motion-torque-replay \
  --no-preview-gif
```

Run OpenRadioss from the case directory so result files stay with the case:

```bash
cd "$OR_ROOT/runs/chihuahua_fem_basic_motion_torque_replay_radius12_8mm_native_beam"

"$OPENRADIOSS_PATH/exec/starter_linux64_gf" \
  -i fem_whole_body_periodic_motion_0000.rad \
  | tee starter.log

"$OPENRADIOSS_PATH/exec/engine_linux64_gf" \
  -i fem_whole_body_periodic_motion_0001.rad \
  | tee engine.log

"$OPENRADIOSS_PATH/exec/th_to_csv_linux64_gf" \
  fem_whole_body_periodic_motionT01 \
  | tee th_to_csv.log
```

Render the native beam-resultant strain GIF from T01:

```bash
cd /mnt/s8t/chihuahua
uv run python -m src.components.c_fem.openradioss_periodic_motion \
  --out-dir "$OR_ROOT/runs/chihuahua_fem_basic_motion_torque_replay_radius12_8mm_native_beam" \
  --samples 25 \
  --solver-duration-ms 8 \
  --viewer-start-seconds 0.2 \
  --viewer-motion-seconds 0.005 \
  --motion-scale 1.0 \
  --target-element-length-mm 8 \
  --uniform-radius-mm 12 \
  --control-policy basic_motion-torque-replay \
  --no-preview-gif \
  --result-frames 36 \
  --result-duration-ms 90
```

The articulated-bend millisecond FEM run produced:

```text
Starter: NORMAL TERMINATION, 0 ERROR(S), 0 WARNING(S)
Engine: NORMAL TERMINATION, TOTAL NUMBER OF CYCLES: 3233
Rod graph nodes: 25
Solver nodes: 305
Beam elements: 304
Beam resultant history columns: 304 elements x F1/M2/M3
Prescribed displacement functions: 0
Control policy: basic_motion-torque-replay
Uniform guided joint nodes: 0
Hard prescribed robot joint nodes: 0
Basic motion torque replay samples: 550
Basic motion torque replay joints: 21
Basic motion torque replay moment couples: 22
Concentrated moment functions: 76
Uniform radius: 12 mm
Max target displacement: 9.47419 mm
Torque replay CSV: basic_motion_torque_replay_loads.csv
T01 CSV: fem_whole_body_periodic_motionT01.csv
H3D: fem_whole_body_periodic_motion.h3d
Animation A-files: fem_whole_body_periodic_motionA001..A019
GIF: openradioss_whole_body_periodic_motion.gif
Poster: openradioss_whole_body_periodic_motion_poster.png
```

This is still not a gravity/contact/support analysis. It is a whole-body
torque-replay FEM motion case: a second-scale viewer period is sampled at
`viewer_start_seconds=0.2`, but only a `0.005 s` articulated-bend slice is used
inside an `8 ms` FEM window. The deck has no `/IMPDISP`, no ghost target nodes,
and no guide springs in this policy. The native beam-resultant GIF should therefore show
almost no gross rotation; it should primarily expose the element tension/bend
map caused by Basic motion torque replay.

All 25 original whole-body joint nodes are preserved in the solver mesh. The
applied loads come from 21 Basic motion torque rows: waist yaw/pitch, four leg
chains, neck yaw/pitch, and head claw. Each torque row is mapped to a proximal
and distal node pair with equal-and-opposite moment components in global
`XX`, `YY`, and `ZZ`. The old `uniform-joint-guides` and `all-joints-hard`
policies remain available only as comparison/debug paths.

The current native result route writes `/TH/BEAM` history for every beam element
with `F1`, `M2`, and `M3`. Post-processing converts those Radioss beam section
resultants into outer-fiber stress and strain using the deck's circular section:
`sigma = F1/A +/- M*r/I`, then `strain = sigma/E`. The older displacement-based
outer-fiber strain proxy remains only as a fallback for old runs that do not
contain `/TH/BEAM`.

Each beam element is still drawn as a projected cylinder footprint using its
section radius, with visible element edges, so the 8 mm mesh can be read
visually instead of appearing as a 1D line. This run uses the same 12 mm circular
radius for every beam element; body, waist, leg, toe, and head members do not
have different section radii in this case. FEM result colors use the red half of
the same `seismic` colormap family as `pygame_mass_viewer.py` joint torque
colors: white is zero and red is hotter absolute outer-fiber strain. The pygame
viewer can additionally overlay the Basic motion torque replay CSV as applied
moment-couple glyphs; those glyphs use the full `seismic` joint-torque scale.

For this radius-12-mm native beam-resultant run, the poster frame was at about
`t=7.082 ms` with maximum displacement about `0.175 mm`. The hottest element was
`waist_yaw_pitch_elem_0001`, with maximum outer-fiber strain about `2.20e-4` and
maximum outer-fiber stress about `0.703 MPa`.

## Pygame FEM Viewer

For interactive inspection, use the pygame viewer instead of a pre-rendered GIF:

```bash
cd /mnt/s8t/chihuahua
uv run python -m src.components.c_fem.pygame_openradioss_fem_viewer \
  --run-dir "$OR_ROOT/runs/chihuahua_fem_basic_motion_torque_replay_radius12_8mm_native_beam"
```

The viewer uses the same T01 displacement history as the generated deck. It
draws each OpenRadioss `/BEAM` element as a colored 3D wireframe line on the
rod geometry; it does not draw per-element solid cylinder surfaces in pygame.
The beam radius remains recorded in the panel and in the deck, but the
interactive view is a mesh-element wireframe for readability. On new runs, its
FEM colors use Radioss beam `F1/M2/M3` outer-fiber strain. On older runs without
beam resultants, it falls back to displacement-derived strain proxy. By default,
the viewer also reads `basic_motion_torque_replay_loads.csv` and overlays the
current-frame applied joint torque as equal-and-opposite proximal/distal moment
glyphs. Press `t` to hide or show that overlay.

Controls:

```text
mouse drag: rotate camera
wheel: zoom
space: pause / play
left / right: step frame
home / end: first / last frame
m: mesh elements
n: whole-body joint nodes
s: solver nodes
t: applied joint torque replay overlay
g: grid
p: screenshot
h: help
q / esc: quit
```

---



# FEM: Whole-Robot FEM

目标：用 **OpenRadioss** 建立 whole-robot FEM 迭代系统，让整机结构强度、刚度、质量、COM、惯量和力矩进入同一个闭环。

这一阶段只做 whole-robot FEM。不建立 single-link、bracket、shaft seat、insert region 或 subassembly 的独立 FEM 分析路线。局部问题必须通过整机模型暴露、修改和复验。


OpenRadioss 的本地 first-light 复现实验记录在 `docs/STAGE_2_OPENRADIOSS_FIRST_LIGHT.md`，包括 release 校验、官方 tensile example、Starter/Engine 运行、T01 转 CSV 和 GIF 可视化。

当前 FEM 的第一份 Chihuahua-specific 输入模型来自 `pygame_mass_viewer.py`
同源拓扑：整机被抽象成 connected rod graph。生成入口是
`rod_model.py`，输出 `nodes.csv`、`members.csv`、
`lumped_masses.csv`、`rod_model_summary.yaml` 和杆系 GIF。

```bash
uv run python -m src.components.c_fem.rod_model --out-dir artifacts/fem/rod_model
```

这个 rod model 仍然是整机模型：body、waist、hips、four legs、feet、neck
和 head claw 一起进入同一个结构图。hip cross-link 在 solver topology 中被
拆成 center-to-hip rods，以保证 body spine 和 hip frame 连通；这不是局部
subassembly 分析。

这一步只导出 topology、质量和 toe-node tags。toe 节点可以保留
contact-candidate 语义，方便后续 MuJoCo/contact 阶段复用，但 FEM
不施加 gravity、support reaction、ground contact 或 fixed boundary
condition。

OpenRadioss whole-body beam deck 由同一份 rod graph 生成：

```bash
uv run python -m src.components.c_fem.openradioss_deck \
  --out-dir artifacts/fem/openradioss_beam_deck
```

这个 deck 使用 `/BEAM`、`/PROP/TYPE3`、`/PART`、`/MAT/LAW1` 和
`/ADMAS/5`。它仍然是 topology-only smoke deck：没有 gravity、没有 fixed
feet、没有 support reaction、没有 external load。Engine file 只用于验证
Starter/Engine 链路能接收 whole-body beam deck，不产生设计结论。

第一份 solved deformation FEM route 是 viewer-periodic whole-body rod
motion case。当前默认 load policy 是 `basic_motion-torque-replay`：从同一帧
`pygame_mass_viewer.py` / Basic motion model 计算 joint torque rows，再把每个
joint torque 写成 OpenRadioss `/CLOAD` equal-and-opposite nodal moment
couple。

```bash
uv run python -m src.components.c_fem.openradioss_periodic_motion \
  --out-dir artifacts/fem/openradioss_periodic_motion \
  --solver-duration-ms 8 \
  --viewer-motion-seconds 0.005 \
  --target-element-length-mm 8 \
  --uniform-radius-mm 8 \
  --control-policy basic_motion-torque-replay
```

这个 case 复用 `pygame_mass_viewer.py` 的 constrained periodic motion，
但 FEM GIF 不播放完整秒级 gait period。它只取周期中的一个短时间片
（默认 `0.005 s`），映射到毫秒级 FEM 窗口（默认 `8 ms`），因此视觉上应几乎
看不到整机转动，主要读取 element-by-element tension map。

同一套 whole-body rod graph 会被切成 elementized `/BEAM` mesh，默认目标
element length 是 `8 mm`，默认 beam section 是 uniform circular radius
`8 mm`。25 个原始 whole-body joint nodes 都保留在 FEM mesh 里，并且语义上
统一都是 joint nodes。`basic_motion-torque-replay` 不使用 `/IMPDISP`、不使用
ghost target nodes、不使用 guide springs；pose samples 只用于计算 Basic motion
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
run dir: /mnt/s8t/openradioss/runs/chihuahua_fem_basic_motion_torque_replay_radius12_8mm_native_beam
control policy: basic_motion-torque-replay
solver nodes: 305
beam elements: 304
beam resultant history elements: 304
imposed displacement functions: 0
concentrated moment functions: 76
basic_motion torque replay joints: 21
basic_motion torque replay moment couples: 22
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

- 读取 `vendor/reference/urdf/chihuahua/lists.md`
- 读取 `pygame_mass_viewer.py` / Basic motion linkage 同源生成的 whole-body rod graph
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

- FEM load case 来自 Basic motion 的质量、COM、惯量、姿态和 joint torque
  估计
- FEM 不包含 gravity、ground contact、support reaction、fixed feet 或
  standing-static support solve
- 每个 load case 记录：来源、单位、方向、施加载体、边界条件和姿态
- 每个 load case 必须可回写 mass / COM / inertia / torque 更新后的新版本
- MuJoCo/contact 阶段产生的 gravity/contact/support 结果，后续只能作为新输入
  回灌 FEM；它们不属于当前 FEM 原生工况

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

---

# ADM 几何优化设想

## 核心命题

这个项目的目标不是找到一个通用意义上的“最优机器狗几何”。真正的 golden loss 是：

```text
behavior 像 Chihuahua
```

也就是说，几何不是最终目标，而是让某种行为分布更自然、更低代价、更高概率地出现的物理载体。

可以把问题写成：

```text
find geometry g
such that
ADM samples conditioned on g
match Chihuahua behavior distribution
```

其中 ADM 指 Action Diffusion Model：一个吃 Gaussian noise、几何信息和行为上下文，然后输出动作/姿态序列的扩散模型。

## 形式化

记：

```text
g: 机器人几何参数
z: Gaussian noise
c: 行为上下文或 command
a_1:T: 一段动作/姿态序列
phi(a_1:T): 行为 embedding
```

ADM 学习：

```text
p_theta(a_1:T | g, c, z)
```

Chihuahua 的真实行为分布记为：

```text
P_chihuahua(phi)
```

对一个候选几何 `g`，ADM 采样得到：

```text
a_1:T ~ p_theta(a_1:T | g, c, z)
```

然后投影到行为空间：

```text
phi(a_1:T)
```

几何的行为损失可以写成：

```text
L_behavior(g) =
  D(
    P_phi(ADM samples | g),
    P_chihuahua(phi)
  )
```

这个 divergence 可以是 KL，但早期不一定要执着于 KL。高维 KL 很脆，support 不重叠时会爆炸。更实用的候选包括：

```text
MMD
sliced Wasserstein
classifier / discriminator divergence
FID-style latent Gaussian distance
low-dimensional density model KL
```

## 为什么不直接比较 raw pose

不要直接在 raw joint angle 或 raw pose 上算 divergence。

不同动物、不同机器人、不同骨架比例的 raw pose 空间并不共享同一个语义。直接比较很容易把“骨架格式差异”误认为“行为不像”。

更稳的方式是先映射到行为 embedding：

```text
phi(action) =
  步频
  步幅 / 身长
  duty factor
  四足落脚相位
  身体上下弹跳
  头部和躯干 yaw scanning
  站距
  抬脚高度
  急停急起节奏
  转弯半径
  acceleration burst
  limb phase relationship
```

我们真正想比较的是：

```text
这个几何下自然生成的行为分布
是否落在 Chihuahua-like behavior manifold 上
```

而不是：

```text
某一帧关节角是否和某只狗完全一样
```

## 总损失

ADM 的行为相似度不能单独作为物理真理。模型可能生成很像 Chihuahua 的动作，但真实机器人做不到。

因此总损失应该包含行为项和物理/工程约束项：

```text
L_total(g) =
  L_chihuahua_behavior(g)
  + L_kinematic_feasibility(g)
  + L_torque_margin(g)
  + L_stability(g)
  + L_collision(g)
  + L_manufacturing_prior(g)
```

当前仓库里的 endpoint geometry scan 可以看作 `L_kinematic_feasibility` 的雏形：它检查 yaw/pitch-waist 拓扑里的 yaw 平面几何下，脚端目标是否吃光腿部 XY reach budget。

## Pipeline

### 1. 收集四肢哺乳动物动作数据

数据可以来自：

```text
mocap
视频姿态估计
多视角重建
已有动物运动数据集
```

训练集不必只包含 Chihuahua。更大的四肢哺乳动物数据可以帮助 ADM 学到通用行为先验，但 Chihuahua 数据需要用于定义目标行为分布。

### 2. Retarget 到共享表示

把不同动物的骨架映射到共享空间：

```text
body frame
normalized limb lengths
foot contact states
limb phase
torso/head orientation
COM-like body motion
```

这一步的目标不是抹掉形态差异，而是让模型能理解：

```text
几何不同，但行为语义可以比较
```

### 3. 训练 ADM

训练条件扩散模型：

```text
p_theta(a_1:T | g, c, z)
```

其中 `c` 很重要。ADM 最好不只吃 geometry，还要吃行为上下文：

```text
walk
trot
turn-left
turn-right
idle-alert
start-stop
approach
startle
```

否则 Chihuahua 的走路、转身、警觉、停顿、突然加速会混成一个大分布，divergence 会变脏。

### 4. 对候选几何疯狂采样

给定第一版机器人几何 `g`：

```text
sample z_i
sample context c_i
generate a_i ~ ADM(g, c_i, z_i)
```

得到这个几何诱导出的 action space。

这一步不是为了找单个最优动作，而是估计：

```text
这个几何下，哪些行为是自然的、高概率的
```

### 5. 过滤物理不可行动作

ADM 是行为先验，不是物理引擎。

采样动作需要经过 feasibility filter：

```text
IK reach
joint limits
self collision
body collision
foot clearance
support polygon
torque margin
motor velocity margin
contact plausibility
```

过滤后的分布才更接近真实机器人可能产生的行为分布。

### 6. 和 Chihuahua 行为分布比较

对 ADM 采样动作和 Chihuahua 真实动作都计算 `phi`：

```text
phi_generated = phi(ADM samples | g)
phi_chihuahua = phi(real Chihuahua motion)
```

然后计算 divergence：

```text
D(phi_generated, phi_chihuahua)
```

这个值就是几何优化里的核心风格损失。

### 7. 优化几何

几何优化可以先用无梯度方法：

```text
CMA-ES
Bayesian optimization
NSGA-II
random search + local refinement
```

原因是整个 pipeline 里会有采样、过滤、碰撞、仿真和统计距离，不一定适合端到端梯度。

优化变量可以从当前仓库已有的参数开始：

```text
body_length_total_m
front_body_fraction
waist_joint_spacing_m
hip_half_width_m
foot_x_offset_m
foot_lateral_outset_m
body_half_width_m
body_z_m
upper_m
lower_m
distal_endpoint_m
waist_yaw_range
waist_pitch_range
```

## 直觉解释

这套方法问的不是：

```text
什么几何在抽象意义上最优？
```

而是：

```text
什么几何会让 Chihuahua-like behavior
在控制器和物理约束下更自然地出现？
```

这和光学设计类似。没有单个 golden curvature，只有针对目标成像风格、像差预算、制造约束和成本的高维权衡。

在这个项目里：

```text
optical aberration budget
对应
robot geometry margin budget
```

几何的坏味道可以分解成：

```text
reach usage 太高
joint limit margin 太小
torque margin 太小
support polygon margin 太小
self-collision margin 太小
packaging margin 太小
gait clearance 太小
```

Chihuahua-like behavior 则是最上层的审美和任务定义。

## 主要风险

### ADM 幻觉

ADM 可能学到“像”的动作，但这些动作在真实机器人上不可执行。

缓解方式：

```text
加入 feasibility filter
加入物理仿真验证
把 torque/contact/stability margin 加入 loss
```

### 行为 embedding 选错

如果 `phi` 没有抓住 Chihuahua 的行为特征，优化出来的几何会对错误指标过拟合。

缓解方式：

```text
手工行为特征 + learned latent 并行
用人类偏好或分类器校验
做 ablation
```

### 数据分布不足

Chihuahua 数据可能很少，且视频姿态估计噪声大。

缓解方式：

```text
用更大的四肢哺乳动物数据训练通用 ADM
用 Chihuahua 数据定义目标分布或 finetune
用 normalized behavioral statistics 降低数据需求
```

### KL 不稳定

直接在高维动作空间算 KL 容易不稳定。

缓解方式：

```text
先降维到 behavior latent
使用 MMD / sliced Wasserstein
或训练 discriminator 估计分布差异
```

## 当前仓库的角色

当前代码是第一层几何压力计：

```text
yaw/pitch-waist geometry
endpoint reach usage
foot clearance
turn radius hint
simple endpoint loss
```

下一步可以把它扩展为 ADM pipeline 里的 feasibility module：

```text
geometry g
ADM sampled behavior
IK / endpoint feasibility check
margin metrics
behavior divergence
geometry optimizer
```

这样 repo 会从“画几何和扫角度”升级成：

```text
Chihuahua-like morphology search bench
```
