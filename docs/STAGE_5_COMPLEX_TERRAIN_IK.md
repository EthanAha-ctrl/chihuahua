# Stage 5: Complex Terrain IK

目标：在 Stage 1-4 已经完成 first-light 后，定义复杂地形上的 IK 和运动决策层。

Stage 5 不是打印阶段，也不是继续堆 MuJoCo 功能。它把 Stage 1 的质量/力矩、
Stage 2 的结构限制、Stage 3 的 IK/control scaffold、Stage 4 的 contact evidence
合并成复杂地形下的可执行运动约束。

## 1. Scope

Stage 5 覆盖：

- terrain-aware foot target selection
- terrain-aware body height selection
- terrain-aware body attitude selection
- uneven-ground per-leg IK
- support-foot scheduling
- COM / support polygon check
- torque margin check for terrain poses
- joint-limit avoidance
- terrain refusal and recovery rules
- MuJoCo replay cases for terrain IK commands

Stage 5 不覆盖：

- printable CAD implementation
- new actuator selection
- new FEM solver implementation
- full autonomous navigation
- vision / perception stack
- outdoor field validation
- manufacturing process validation

## 2. Inputs

Stage 5 consumes:

- `mass_model.py` outputs: mass, COM, torque margin, leg endpoint residuals
- Stage 2 FEM limits: hot spots, deformation-sensitive poses, load-path limits
- Stage 3 IK/control outputs: joint commands, trajectory primitives, safety limits
- Stage 4 MuJoCo/contact outputs: contact candidates, support proxy, failure evidence
- terrain description: height samples, slope, step edges, friction assumption

Terrain may start as a synthetic local height field. It does not need camera or
perception integration at this stage.

## 3. Core Questions

Stage 5 must answer:

```text
Where can each foot safely land?
What body pose keeps IK, COM, and torque inside limits?
Which legs should stay in support?
Which terrain patches should be refused?
Which terrain poses must be replayed in MuJoCo?
```

## 4. Terrain IK Outputs

Expected outputs:

- terrain contact candidate table
- terrain foot target table
- terrain body pose table
- terrain IK command table
- terrain safety / refusal table
- MuJoCo terrain replay cases
- terrain failure-mode report

## 5. Acceptance

Stage 5 is accepted when:

- flat-ground commands remain compatible with Stage 3 and Stage 4
- uneven terrain commands produce bounded IK residuals
- every generated pose has torque margin evidence
- every generated pose has COM/support evidence
- unsafe terrain cases are refused instead of silently generating commands
- replay cases can be exported back to MuJoCo/contact review

Stage 5 does not need to prove outdoor survival. It only needs to prove that
complex terrain IK has explicit inputs, outputs, refusal rules, and replayable
evidence.
