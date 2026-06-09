# External Dependencies

这个仓库用两种方式记录 dependencies：

- Python dependencies 写在 `pyproject.toml`
- 非 Python 工程工具写在本文档

本文档只记录项目需要哪些外部工具、这些工具服务哪个阶段、版本策略和期望产物。不记录本机安装路径、环境变量、自动探测逻辑或个人机器配置。

## Stage 2 Whole-Robot FEM

### OpenRadioss

- status: required
- role: whole-robot FEM solver
- project stage: Stage 2
- source: official OpenRadioss release
- expected inputs: whole-robot OpenRadioss case files
- expected outputs: animation / time-history result files
- repo policy: do not vendor solver binaries into this repository
- first-light reproduction: `docs/STAGE_2_OPENRADIOSS_FIRST_LIGHT.md`

### OpenRadioss Result Conversion

- status: required
- role: convert solver results into visualization-friendly files
- project stage: Stage 2
- expected inputs: OpenRadioss animation / time-history result files
- expected outputs: VTK files and CSV time-series files
- repo policy: generated result files are design artifacts, converter binaries are not vendored

### ParaView

- status: required
- role: default visualization tool for Stage 2 FEM results
- project stage: Stage 2
- expected inputs: VTK files and CSV time-series files
- expected outputs: visual inspection, screenshots, deformation/stress plots
- repo policy: do not depend on Altair HyperView / HyperGraph

## Stage 2 Visualization Artifacts

Stage 2 should produce repository-readable artifacts in this shape:

```text
fem/stage2/<design_version>/<load_case>/
  case/
    whole_robot.rad
    assumptions.yaml
  results/
    whole_robot.vtk
    timeseries.csv
  screenshots/
    deformation.png
    stress_hot_spots.png
```

These paths describe expected artifacts, not local tool installation paths.

## Stage 4 MuJoCo / Contact

### MuJoCo Python

- status: optional for first-light, required for real Stage 4 simulation
- role: load and run the exported MuJoCo XML model
- project stage: Stage 4
- source: official MuJoCo Python package
- expected inputs: `mujoco_model.xml`
- expected outputs: simulation state/contact history and failure observations
- repo policy: Stage 4 XML and contact CSV export must work without vendoring or requiring the solver package
- first-light reproduction: `docs/STAGE_4_MUJOCO_CONTACT.md`
