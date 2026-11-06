| Migration notes |
| --- |
| Stage 1 physical material placeholders.<br>Values are conservative early-design estimates, not calibrated coupon data. |

| field | value | type |
| --- | --- | --- |
| default_structural_material | pla_plus | string |
| default_elastomer_material | tpu_95a | string |
| materials.pla_plus.class | fdm_printed_polymer | string |
| materials.pla_plus.density_kg_m3 | 1240.0 | number |
| materials.pla_plus.youngs_modulus_pa | 3200000000.0 | number |
| materials.pla_plus.poisson_ratio | 0.36 | number |
| materials.pla_plus.yield_strength_pa | 45000000.0 | number |
| materials.pla_plus.ultimate_strength_pa | 60000000.0 | number |
| materials.pla_plus.fatigue_assumption | low confidence; avoid cyclic tensile layer loading | string |
| materials.pla_plus.layer_adhesion_strength_pa | 18000000.0 | number |
| materials.pla_plus.anisotropy_factor | 0.55 | number |
| materials.pla_plus.recommended_print_orientation | keep primary bending and tension loads in XY when possible | string |
| materials.petg.class | fdm_printed_polymer | string |
| materials.petg.density_kg_m3 | 1270.0 | number |
| materials.petg.youngs_modulus_pa | 2100000000.0 | number |
| materials.petg.poisson_ratio | 0.38 | number |
| materials.petg.yield_strength_pa | 38000000.0 | number |
| materials.petg.ultimate_strength_pa | 50000000.0 | number |
| materials.petg.fatigue_assumption | better ductility than PLA; still derate layer tension heavily | string |
| materials.petg.layer_adhesion_strength_pa | 16000000.0 | number |
| materials.petg.anisotropy_factor | 0.5 | number |
| materials.petg.recommended_print_orientation | avoid Z-layer peel at brackets and bearing seats | string |
| materials.asa_abs.class | fdm_printed_polymer | string |
| materials.asa_abs.density_kg_m3 | 1050.0 | number |
| materials.asa_abs.youngs_modulus_pa | 1900000000.0 | number |
| materials.asa_abs.poisson_ratio | 0.35 | number |
| materials.asa_abs.yield_strength_pa | 30000000.0 | number |
| materials.asa_abs.ultimate_strength_pa | 42000000.0 | number |
| materials.asa_abs.fatigue_assumption | ductile thermal-resistant option; needs print quality calibration | string |
| materials.asa_abs.layer_adhesion_strength_pa | 14000000.0 | number |
| materials.asa_abs.anisotropy_factor | 0.48 | number |
| materials.asa_abs.recommended_print_orientation | use enclosed printing; avoid unsupported thin vertical tabs | string |
| materials.paht_cf.class | fdm_printed_fiber_filled_polyamide | string |
| materials.paht_cf.density_kg_m3 | 1180.0 | number |
| materials.paht_cf.youngs_modulus_pa | 6500000000.0 | number |
| materials.paht_cf.poisson_ratio | 0.32 | number |
| materials.paht_cf.yield_strength_pa | 70000000.0 | number |
| materials.paht_cf.ultimate_strength_pa | 95000000.0 | number |
| materials.paht_cf.fatigue_assumption | preferred high-stiffness candidate after drying and coupon testing | string |
| materials.paht_cf.layer_adhesion_strength_pa | 28000000.0 | number |
| materials.paht_cf.anisotropy_factor | 0.62 | number |
| materials.paht_cf.recommended_print_orientation | align fiber-rich XY raster with link length and bracket load path | string |
| materials.tpu_95a.class | fdm_printed_elastomer | string |
| materials.tpu_95a.density_kg_m3 | 1200.0 | number |
| materials.tpu_95a.youngs_modulus_pa | 40000000.0 | number |
| materials.tpu_95a.poisson_ratio | 0.49 | number |
| materials.tpu_95a.yield_strength_pa | 7000000.0 | number |
| materials.tpu_95a.ultimate_strength_pa | 25000000.0 | number |
| materials.tpu_95a.fatigue_assumption | useful for pads and compliant feet, not structural links | string |
| materials.tpu_95a.layer_adhesion_strength_pa | 6000000.0 | number |
| materials.tpu_95a.anisotropy_factor | 0.45 | number |
| materials.tpu_95a.recommended_print_orientation | print pads flat; avoid relying on thin Z-direction features | string |
