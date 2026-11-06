| Migration notes |
| --- |
| Stage 1 actuator catalog placeholders.<br>Torque and thermal numbers must be replaced with real datasheets before CAD lock-in. |

| field | value | type |
| --- | --- | --- |
| default_actuator_set.waist_yaw | compact_servo_2nm | string |
| default_actuator_set.waist_pitch | compact_servo_2nm | string |
| default_actuator_set.hip_ab | compact_servo_2nm | string |
| default_actuator_set.hip_pitch | compact_servo_2nm | string |
| default_actuator_set.knee_bend | compact_servo_2nm | string |
| default_actuator_set.toe_bend | micro_servo_0_4nm | string |
| default_actuator_set.neck_yaw | micro_servo_0_4nm | string |
| default_actuator_set.neck_pitch | micro_servo_0_4nm | string |
| default_actuator_set.head_claw | micro_servo_0_4nm | string |
| actuators.micro_servo_0_4nm.mass_kg | 0.02 | number |
| actuators.micro_servo_0_4nm.dimensions_m.x | 0.023 | number |
| actuators.micro_servo_0_4nm.dimensions_m.y | 0.012 | number |
| actuators.micro_servo_0_4nm.dimensions_m.z | 0.025 | number |
| actuators.micro_servo_0_4nm.mounting_pattern | two-lug micro servo, placeholder | string |
| actuators.micro_servo_0_4nm.max_torque_nm | 0.45 | number |
| actuators.micro_servo_0_4nm.continuous_torque_nm | 0.16 | number |
| actuators.micro_servo_0_4nm.stall_torque_nm | 0.55 | number |
| actuators.micro_servo_0_4nm.max_speed_rad_s | 7.0 | number |
| actuators.micro_servo_0_4nm.gear_ratio | 180.0 | number |
| actuators.micro_servo_0_4nm.backlash_deg | 2.0 | number |
| actuators.micro_servo_0_4nm.thermal_limit_c | 65.0 | number |
| actuators.micro_servo_0_4nm.current_draw_a.idle | 0.05 | number |
| actuators.micro_servo_0_4nm.current_draw_a.continuous | 0.35 | number |
| actuators.micro_servo_0_4nm.current_draw_a.stall | 1.2 | number |
| actuators.micro_servo_0_4nm.voltage_v.min | 6.0 | number |
| actuators.micro_servo_0_4nm.voltage_v.nominal | 7.4 | number |
| actuators.micro_servo_0_4nm.voltage_v.max | 8.4 | number |
| actuators.micro_servo_0_4nm.driver_mass_kg | 0.003 | number |
| actuators.micro_servo_0_4nm.notes | distal, neck, and head-claw placeholder only | string |
| actuators.compact_servo_2nm.mass_kg | 0.095 | number |
| actuators.compact_servo_2nm.dimensions_m.x | 0.04 | number |
| actuators.compact_servo_2nm.dimensions_m.y | 0.04 | number |
| actuators.compact_servo_2nm.dimensions_m.z | 0.03 | number |
| actuators.compact_servo_2nm.mounting_pattern | square face mount with radial output horn, placeholder | string |
| actuators.compact_servo_2nm.max_torque_nm | 2.4 | number |
| actuators.compact_servo_2nm.continuous_torque_nm | 0.95 | number |
| actuators.compact_servo_2nm.stall_torque_nm | 3.0 | number |
| actuators.compact_servo_2nm.max_speed_rad_s | 5.5 | number |
| actuators.compact_servo_2nm.gear_ratio | 280.0 | number |
| actuators.compact_servo_2nm.backlash_deg | 1.0 | number |
| actuators.compact_servo_2nm.thermal_limit_c | 70.0 | number |
| actuators.compact_servo_2nm.current_draw_a.idle | 0.08 | number |
| actuators.compact_servo_2nm.current_draw_a.continuous | 1.2 | number |
| actuators.compact_servo_2nm.current_draw_a.stall | 4.5 | number |
| actuators.compact_servo_2nm.voltage_v.min | 7.4 | number |
| actuators.compact_servo_2nm.voltage_v.nominal | 11.1 | number |
| actuators.compact_servo_2nm.voltage_v.max | 12.6 | number |
| actuators.compact_servo_2nm.driver_mass_kg | 0.008 | number |
| actuators.compact_servo_2nm.notes | first-pass hip, knee, and waist actuator placeholder | string |
| actuators.compact_actuator_5nm.mass_kg | 0.18 | number |
| actuators.compact_actuator_5nm.dimensions_m.x | 0.052 | number |
| actuators.compact_actuator_5nm.dimensions_m.y | 0.052 | number |
| actuators.compact_actuator_5nm.dimensions_m.z | 0.038 | number |
| actuators.compact_actuator_5nm.mounting_pattern | coaxial output bearing mount, placeholder | string |
| actuators.compact_actuator_5nm.max_torque_nm | 5.5 | number |
| actuators.compact_actuator_5nm.continuous_torque_nm | 2.2 | number |
| actuators.compact_actuator_5nm.stall_torque_nm | 7.0 | number |
| actuators.compact_actuator_5nm.max_speed_rad_s | 4.5 | number |
| actuators.compact_actuator_5nm.gear_ratio | 360.0 | number |
| actuators.compact_actuator_5nm.backlash_deg | 0.6 | number |
| actuators.compact_actuator_5nm.thermal_limit_c | 75.0 | number |
| actuators.compact_actuator_5nm.current_draw_a.idle | 0.12 | number |
| actuators.compact_actuator_5nm.current_draw_a.continuous | 2.0 | number |
| actuators.compact_actuator_5nm.current_draw_a.stall | 8.0 | number |
| actuators.compact_actuator_5nm.voltage_v.min | 11.1 | number |
| actuators.compact_actuator_5nm.voltage_v.nominal | 14.8 | number |
| actuators.compact_actuator_5nm.voltage_v.max | 16.8 | number |
| actuators.compact_actuator_5nm.driver_mass_kg | 0.012 | number |
| actuators.compact_actuator_5nm.notes | heavier fallback for joints that fail Stage 1 torque margin | string |
