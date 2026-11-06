# Reference robots

| Robot | Local files | Source revision | License |
| --- | --- | --- | --- |
| Unitree Go2 | [urdf/go2/](urdf/go2/) | [unitreerobotics/unitree_ros @ 3ae893d](https://github.com/unitreerobotics/unitree_ros/tree/3ae893d905d11795ddf1075825ee5010364a1109/robots/go2_description) | BSD-3-Clause |
| Aibo ERS-7 | [urdf/aibo_ers7/](urdf/aibo_ers7/) | [dkotfis/aibo_ros @ eec7a9d](https://github.com/dkotfis/aibo_ros/tree/eec7a9dc21779a289eb3bfa48d66cedd830176d3/aibo_description) | MIT |
| Chihuahua | [Geometry and joint ranges](urdf/chihuahua/lists.md) | Repository design configuration, including original references | Internal configuration |
| Pupper v3 | [urdf/pupper_v3/](urdf/pupper_v3/) | [G-Levine/pupper_v3_description @ 1a3c859](https://github.com/G-Levine/pupper_v3_description/tree/1a3c8590189d3a406219d155f4177afc491a7d77) | MIT |

[Model adaptations, kinematic provenance and audit details](urdf/README.md).
URDFs, source meshes, licenses and runtime audit metadata are retained together.
Generated review images live in `artifacts/model_review/<robot>/`.
Audit and rendering code lives in `src/components/model_tools/`.
