# Stage 2: Whole-Robot FEM

目标：用 **OpenRadioss** 建立 whole-robot FEM 迭代系统，让整机结构强度、刚度、质量、COM、惯量和力矩进入同一个闭环。

这一阶段只做 whole-robot FEM。不建立 single-link、bracket、shaft seat、insert region 或 subassembly 的独立 FEM 分析路线。局部问题必须通过整机模型暴露、修改和复验。

## 1. FEM 输入模型

- 读取 `dog_description.yaml`
- 读取材料、打印方向、壁厚、孔、boss、rib、insert、shaft、fastener 参数
- 读取 actuator、battery、electronics 的 mass / COM / inertia / placement
- 建立整机 structural model
- 建立 joint / shaft / bearing / fastener 的简化连接模型
- 每个设计版本记录 mass / COM / inertia / torque / FEM 假设

## 2. 分析对象

- full robot body
- full leg structures
- waist yaw / pitch structure
- hip frame and leg attachment regions
- actuator and battery lumped masses
- simplified joints, shafts, bearings, inserts, and fasteners
- feet / ground contact support conditions

## 3. Load Case Schema

- load case 来自 Stage 1 的质量、COM、惯量、力矩、姿态和反力估计
- 每个 load case 记录：来源、单位、方向、施加载体、边界条件、姿态、支撑脚集合
- 每个 load case 必须可回写 mass / COM / inertia / torque 更新后的新版本

核心 load cases：

- four-leg standing
- single-leg lifted / three-leg support
- diagonal support
- crouched worst-torque pose
- waist yaw extreme pose
- waist pitch extreme pose
- side load
- landing / stumble impulse approximation
- actuator torque reaction
- battery / electronics inertia load
- assembly preload

## 4. OpenRadioss Case Generation

- 生成 whole-robot OpenRadioss 输入文件
- 每个设计版本 / 姿态 / 工况单独目录
- 支持 static-equivalent、torque reaction、side load、preload、impulse approximation、transient case
- 保留 actuator、battery、electronics 的 lumped mass 和 inertia
- 保留 joint / shaft / bearing / fastener 的连接假设
- 输出可追溯的 model assumptions table

## 5. FEM 检查项

- global body deformation
- body torsion
- waist load path weakness
- hip frame deformation
- joint axis misalignment
- endpoint deflection under load
- support-leg load distribution
- torque reaction load path
- stress hot spots
- layer delamination risk
- fatigue-sensitive regions
- connector reaction force
- ground contact reaction consistency

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
- material / print-orientation assumptions
- joint / shaft / bearing / fastener connection assumptions
- global deformation report
- endpoint deflection report
- joint axis misalignment report
- stress / deformation / margin report
- hot-spot table
- connector reaction force table
- mass delta
- torque impact
- design decision log
