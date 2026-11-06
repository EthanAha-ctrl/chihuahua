| Migration notes |
| --- |
| Link-only dog/robot description for the endpoint geometry viewer.<br>No physics. No contact model. No controller. No solid body rendering.<br>These are drawing ranges for constrained periodic motion.<br>Goniometry sources report included joint angles; this viewer uses simple<br>drawing bend angles. The distal endpoint joint is a robot digitigrade<br>toe/paw-style link, not a sourced canine phalange ROM. |

| field | value | type |
| --- | --- | --- |
| name | digitigrade_yaw_pitch_waist_dog | string |
| sources.mixed_breed_limb_bones.url | https://pmc.ncbi.nlm.nih.gov/articles/PMC8280978/ | string |
| sources.mixed_breed_limb_bones.summary | Mixed breed dog long-bone data, not Chihuahua-specific. Reported left/right lengths: humerus 16.93/17.55 cm, radius 18.30/18.15 cm, femur 19.02/19.40 cm, tibia 18.90/20.03 cm. No toe/metacarpal/ metatarsal/phalange segment length is provided.<br> | string |
| sources.chihuahua_joint_motion.url | https://papers.ssrn.com/sol3/papers.cfm?abstract_id=5098946 | string |
| sources.chihuahua_joint_motion.summary | Chihuahua study found, but it is Grade III medial patellar luxation, not a complete healthy Chihuahua baseline for all leg joints.<br> | string |
| sources.general_canine_rom.url | https://catalogimages.wiley.com/images/db/pdf/9781118711361.excerpt.pdf | string |
| sources.general_canine_rom.summary | General canine passive ROM fallback. Table 1.2 reports hip, stifle, tarsus, shoulder, elbow, and carpus flexion/extension values.<br> | string |
| sources.small_breed_rom_cross_check.url | https://www.frontiersin.org/articles/10.3389/fvets.2019.00424/full | string |
| sources.small_breed_rom_cross_check.summary | Healthy French Bulldog goniometry cross-check: hip ROM about 121-123 deg, stifle ROM about 114-115 deg, tarsus ROM about 149 deg.<br> | string |
| sources.unitree_go2_fallback.url | https://github.com/unitreerobotics/unitree_ros/blob/master/robots/go2_description/urdf/go2_description.urdf | string |
| sources.unitree_go2_fallback.summary | Robot fallback only. Go2 has hip/thigh/calf revolute limits, but its foot joint is fixed, so it is not a source for our distal endpoint joint.<br> | string |
| sources.canine_cervical_note.url | https://pubmed.ncbi.nlm.nih.gov/2263970/ | string |
| sources.canine_cervical_note.summary | Canine cervical cadaver study reports intact C4-C5 flexion-extension ROM of 24.5 +/- 6.6 deg at 1 N-m. This is one vertebral motion unit, not a complete healthy Chihuahua whole-neck ROM.<br> | string |
| sources.digitigrade_endpoint_note.url | https://www.britannica.com/science/digitigrade-posture | string |
| sources.digitigrade_endpoint_note.summary | Dogs and cats are digitigrade: only the phalanges touch while ankle/wrist are elevated. This is the intended analogy for the robot distal endpoint joint, not a tarsus/hock joint.<br> | string |
| geometry.body_length_total_m.visual_tuning | true | boolean |
| geometry.body_length_total_m.value | 0.64 | number |
| geometry.front_body_fraction.visual_tuning | true | boolean |
| geometry.front_body_fraction.value | 0.5 | number |
| geometry.waist_joint_spacing_m.visual_tuning | true | boolean |
| geometry.waist_joint_spacing_m.value | 0.08 | number |
| geometry.body_half_width_m.visual_tuning | false | boolean |
| geometry.body_half_width_m.value | 0.075 | number |
| geometry.hip_half_width_m.visual_tuning | false | boolean |
| geometry.hip_half_width_m.value | 0.105 | number |
| geometry.foot_x_offset_m.visual_tuning | false | boolean |
| geometry.foot_x_offset_m.value | 0.0 | number |
| geometry.foot_lateral_outset_m.visual_tuning | false | boolean |
| geometry.foot_lateral_outset_m.value | 0.045 | number |
| geometry.min_reach_xy_m.visual_tuning | false | boolean |
| geometry.min_reach_xy_m.value | 0.015 | number |
| geometry.max_reach_xy_m.visual_tuning | false | boolean |
| geometry.max_reach_xy_m.value | 0.165 | number |
| geometry.min_foot_clearance_m.visual_tuning | false | boolean |
| geometry.min_foot_clearance_m.value | 0.08 | number |
| viewer.body_z_m.visual_tuning | false | boolean |
| viewer.body_z_m.value | 0.245 | number |
| viewer.links_m.upper.visual_tuning | false | boolean |
| viewer.links_m.upper.value | 0.105 | number |
| viewer.links_m.lower.visual_tuning | false | boolean |
| viewer.links_m.lower.value | 0.112 | number |
| viewer.links_m.distal_endpoint.visual_tuning | false | boolean |
| viewer.links_m.distal_endpoint.value | 0.045 | number |
| viewer.head_claw.neck_length_m.visual_tuning | false | boolean |
| viewer.head_claw.neck_length_m.value | 0.078 | number |
| viewer.head_claw.hinge_half_gap_m.visual_tuning | false | boolean |
| viewer.head_claw.hinge_half_gap_m.value | 0.012 | number |
| viewer.head_claw.jaw_length_m.visual_tuning | false | boolean |
| viewer.head_claw.jaw_length_m.value | 0.058 | number |
| joint_ranges_deg.waist_yaw.visual_tuning | false | boolean |
| joint_ranges_deg.waist_yaw.bias_deg | 0.0 | number |
| joint_ranges_deg.waist_yaw.amp_deg | 24.0 | number |
| joint_ranges_deg.waist_yaw.min_deg | -45.0 | number |
| joint_ranges_deg.waist_yaw.max_deg | 45.0 | number |
| joint_ranges_deg.waist_pitch.visual_tuning | false | boolean |
| joint_ranges_deg.waist_pitch.bias_deg | 0.0 | number |
| joint_ranges_deg.waist_pitch.amp_deg | 14.0 | number |
| joint_ranges_deg.waist_pitch.min_deg | -25.0 | number |
| joint_ranges_deg.waist_pitch.max_deg | 25.0 | number |
| joint_ranges_deg.hip_ab.visual_tuning | false | boolean |
| joint_ranges_deg.hip_ab.bias_deg | 0.0 | number |
| joint_ranges_deg.hip_ab.amp_deg | 24.0 | number |
| joint_ranges_deg.hip_ab.min_deg | -32.0 | number |
| joint_ranges_deg.hip_ab.max_deg | 32.0 | number |
| joint_ranges_deg.hip_pitch.visual_tuning | false | boolean |
| joint_ranges_deg.hip_pitch.bias_deg | 0.0 | number |
| joint_ranges_deg.hip_pitch.amp_deg | 42.0 | number |
| joint_ranges_deg.hip_pitch.min_deg | -55.0 | number |
| joint_ranges_deg.hip_pitch.max_deg | 55.0 | number |
| joint_ranges_deg.knee_bend.visual_tuning | false | boolean |
| joint_ranges_deg.knee_bend.bias_deg | 68.0 | number |
| joint_ranges_deg.knee_bend.amp_deg | 52.0 | number |
| joint_ranges_deg.knee_bend.min_deg | 10.0 | number |
| joint_ranges_deg.knee_bend.max_deg | 135.0 | number |
| joint_ranges_deg.toe_bend.visual_tuning | false | boolean |
| joint_ranges_deg.toe_bend.bias_deg | 0.0 | number |
| joint_ranges_deg.toe_bend.amp_deg | 105.0 | number |
| joint_ranges_deg.toe_bend.min_deg | -110.0 | number |
| joint_ranges_deg.toe_bend.max_deg | 110.0 | number |
| joint_ranges_deg.neck_yaw.visual_tuning | false | boolean |
| joint_ranges_deg.neck_yaw.bias_deg | 0.0 | number |
| joint_ranges_deg.neck_yaw.amp_deg | 30.0 | number |
| joint_ranges_deg.neck_yaw.min_deg | -55.0 | number |
| joint_ranges_deg.neck_yaw.max_deg | 55.0 | number |
| joint_ranges_deg.neck_pitch.visual_tuning | false | boolean |
| joint_ranges_deg.neck_pitch.bias_deg | 0.0 | number |
| joint_ranges_deg.neck_pitch.amp_deg | 22.0 | number |
| joint_ranges_deg.neck_pitch.min_deg | -35.0 | number |
| joint_ranges_deg.neck_pitch.max_deg | 35.0 | number |
| joint_ranges_deg.head_claw.visual_tuning | false | boolean |
| joint_ranges_deg.head_claw.bias_deg | 8.0 | number |
| joint_ranges_deg.head_claw.amp_deg | 18.0 | number |
| joint_ranges_deg.head_claw.min_deg | -22.0 | number |
| joint_ranges_deg.head_claw.max_deg | 28.0 | number |
