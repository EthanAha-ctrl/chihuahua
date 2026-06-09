# Stage 2 OpenRadioss First Light

本文档记录第一次 OpenRadioss 本地复现实验。目标不是求解 Chihuahua
整机结构，而是确认 Stage 2 的外部 solver 链路真的可用：

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
  `https://github.com/OpenRadioss/OpenRadioss/releases/tag/latest-20260520`
- Linux binary:
  `https://github.com/OpenRadioss/OpenRadioss/releases/download/latest-20260520/OpenRadioss_linux64.zip`
- Official running docs:
  `https://openradioss.atlassian.net/wiki/spaces/OPENRADIOSS/pages/19628079/Running+OpenRadioss`
- Official tensile test page:
  `https://openradioss.atlassian.net/wiki/spaces/OPENRADIOSS/pages/11075620/Tensile+Test`
- Tensile test Section 1 zip:
  `https://openradioss.atlassian.net/wiki/download/attachments/11075620/Tensile_Test_Section1.zip?api=v2`

## Known-Good Versions

This run used:

```text
OpenRadioss tag: latest-20260520
archive: OpenRadioss_linux64.zip
sha256: 7405c003f59198b1edde118a5dbf8aacf55fbe1415bbc3ca73d9e8e82a400ee0
platform: Linux x86_64
example: Tensile_Test_Section1.zip / 1_LAW2
```

## Directory Layout

Use a sandbox outside this repository:

```bash
export OR_ROOT=/mnt/s8t/openradioss
export OR_TAG=latest-20260520
mkdir -p "$OR_ROOT/downloads" "$OR_ROOT/runs"
```

Expected layout after the steps below:

```text
/mnt/s8t/openradioss/
  downloads/
    OpenRadioss_linux64.zip
  latest-20260520/
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
  "https://github.com/OpenRadioss/OpenRadioss/releases/download/$OR_TAG/OpenRadioss_linux64.zip"

sha256sum OpenRadioss_linux64.zip
```

Expected hash:

```text
7405c003f59198b1edde118a5dbf8aacf55fbe1415bbc3ca73d9e8e82a400ee0  OpenRadioss_linux64.zip
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
uv run python tools/openradioss_tensile_gif.py \
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

This run proves the Stage 2 toolchain can already do the essential loop:

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
pygame_mass_viewer.py / Stage 1 MassModel topology
-> connected body + waist + hips + legs + feet + neck + head rods
-> lump actuator, battery, electronics, and foot-pad masses
-> OpenRadioss whole-robot beam / rod deck
-> Stage 2 load cases replay Stage 1 torque / inertia without gravity or contact
-> CSV and FEM visualization artifacts
```

Only after that should Stage 2 graduate to whole-robot OpenRadioss case
generation.

## Chihuahua Whole-Body Beam Deck Smoke Run

The first Chihuahua-specific OpenRadioss deck generator is:

```bash
cd /mnt/s8t/chihuahua
uv run python stage2_openradioss_deck.py \
  --out-dir "$OR_ROOT/runs/chihuahua_stage2_whole_body_beam"
```

It exports a topology-only whole-body beam deck:

```text
stage2_whole_body_beam_0000.rad
stage2_whole_body_beam_0001.rad
openradioss_beam_deck_summary.yaml
openradioss_whole_body_beam_deck.gif
openradioss_whole_body_beam_deck_poster.png
```

The deck uses `/BEAM`, `/PROP/TYPE3`, `/PART`, `/MAT/LAW1`, and `/ADMAS/5`.
It has no gravity, no fixed feet, no support reaction, and no external load.

Run the smoke check:

```bash
cd "$OR_ROOT/runs/chihuahua_stage2_whole_body_beam"

"$OPENRADIOSS_PATH/exec/starter_linux64_gf" \
  -i stage2_whole_body_beam_0000.rad \
  -np 1 | tee starter.log

"$OPENRADIOSS_PATH/exec/engine_linux64_gf" \
  -i stage2_whole_body_beam_0001.rad | tee engine.log

"$OPENRADIOSS_PATH/exec/th_to_csv_linux64_gf" \
  stage2_whole_body_beamT01 | tee th_to_csv.log
```

The first successful smoke run produced:

```text
Starter: NORMAL TERMINATION, 0 ERROR(S), 0 WARNING(S)
Engine: NORMAL TERMINATION, TOTAL NUMBER OF CYCLES: 4
T01 CSV: stage2_whole_body_beamT01.csv
OpenRadioss deck mass: 2.492834 kg
```

This smoke run proves deck generation and solver ingestion only. It is not a
gravity, support, contact, deformation, stress, or design-margin result.

## Chihuahua Periodic-Motion Rod FEM

The first solved whole-body rod deformation case is generated from the same
`pygame_mass_viewer.py` periodic motion sampler. The current default solved
route is Stage 1 torque replay: per-frame Stage 1 joint torque rows are written
as equal-and-opposite OpenRadioss `/CLOAD` nodal moment couples. The sampled
viewer pose is a torque/reference source, not a hard motion boundary.

```bash
cd /mnt/s8t/chihuahua
uv run python stage2_openradioss_periodic_motion.py \
  --out-dir "$OR_ROOT/runs/chihuahua_stage2_stage1_torque_replay_radius12_8mm_native_beam" \
  --samples 25 \
  --solver-duration-ms 8 \
  --viewer-start-seconds 0.2 \
  --viewer-motion-seconds 0.005 \
  --motion-scale 1.0 \
  --target-element-length-mm 8 \
  --uniform-radius-mm 12 \
  --control-policy stage1-torque-replay \
  --no-preview-gif
```

Run OpenRadioss from the case directory so result files stay with the case:

```bash
cd "$OR_ROOT/runs/chihuahua_stage2_stage1_torque_replay_radius12_8mm_native_beam"

"$OPENRADIOSS_PATH/exec/starter_linux64_gf" \
  -i stage2_whole_body_periodic_motion_0000.rad \
  | tee starter.log

"$OPENRADIOSS_PATH/exec/engine_linux64_gf" \
  -i stage2_whole_body_periodic_motion_0001.rad \
  | tee engine.log

"$OPENRADIOSS_PATH/exec/th_to_csv_linux64_gf" \
  stage2_whole_body_periodic_motionT01 \
  | tee th_to_csv.log
```

Render the native beam-resultant strain GIF from T01:

```bash
cd /mnt/s8t/chihuahua
uv run python stage2_openradioss_periodic_motion.py \
  --out-dir "$OR_ROOT/runs/chihuahua_stage2_stage1_torque_replay_radius12_8mm_native_beam" \
  --samples 25 \
  --solver-duration-ms 8 \
  --viewer-start-seconds 0.2 \
  --viewer-motion-seconds 0.005 \
  --motion-scale 1.0 \
  --target-element-length-mm 8 \
  --uniform-radius-mm 12 \
  --control-policy stage1-torque-replay \
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
Control policy: stage1-torque-replay
Uniform guided joint nodes: 0
Hard prescribed robot joint nodes: 0
Stage 1 torque replay samples: 550
Stage 1 torque replay joints: 21
Stage 1 torque replay moment couples: 22
Concentrated moment functions: 76
Uniform radius: 12 mm
Max target displacement: 9.47419 mm
Torque replay CSV: stage1_torque_replay_loads.csv
T01 CSV: stage2_whole_body_periodic_motionT01.csv
H3D: stage2_whole_body_periodic_motion.h3d
Animation A-files: stage2_whole_body_periodic_motionA001..A019
GIF: openradioss_whole_body_periodic_motion.gif
Poster: openradioss_whole_body_periodic_motion_poster.png
```

This is still not a gravity/contact/support analysis. It is a whole-body
torque-replay FEM motion case: a second-scale viewer period is sampled at
`viewer_start_seconds=0.2`, but only a `0.005 s` articulated-bend slice is used
inside an `8 ms` FEM window. The deck has no `/IMPDISP`, no ghost target nodes,
and no guide springs in this policy. The native beam-resultant GIF should therefore show
almost no gross rotation; it should primarily expose the element tension/bend
map caused by Stage 1 torque replay.

All 25 original whole-body joint nodes are preserved in the solver mesh. The
applied loads come from 21 Stage 1 torque rows: waist yaw/pitch, four leg
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
viewer can additionally overlay the Stage 1 torque replay CSV as applied
moment-couple glyphs; those glyphs use the full `seismic` joint-torque scale.

For this radius-12-mm native beam-resultant run, the poster frame was at about
`t=7.082 ms` with maximum displacement about `0.175 mm`. The hottest element was
`waist_yaw_pitch_elem_0001`, with maximum outer-fiber strain about `2.20e-4` and
maximum outer-fiber stress about `0.703 MPa`.

## Pygame FEM Viewer

For interactive inspection, use the pygame viewer instead of a pre-rendered GIF:

```bash
cd /mnt/s8t/chihuahua
uv run python pygame_openradioss_fem_viewer.py \
  --run-dir "$OR_ROOT/runs/chihuahua_stage2_stage1_torque_replay_radius12_8mm_native_beam"
```

The viewer uses the same T01 displacement history as the generated deck. It
draws each OpenRadioss `/BEAM` element as a colored 3D wireframe line on the
rod geometry; it does not draw per-element solid cylinder surfaces in pygame.
The beam radius remains recorded in the panel and in the deck, but the
interactive view is a mesh-element wireframe for readability. On new runs, its
FEM colors use Radioss beam `F1/M2/M3` outer-fiber strain. On older runs without
beam resultants, it falls back to displacement-derived strain proxy. By default,
the viewer also reads `stage1_torque_replay_loads.csv` and overlays the
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
