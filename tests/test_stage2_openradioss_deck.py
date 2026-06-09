import tempfile
import unittest
import re
from pathlib import Path

import mass_model
import stage2_openradioss_deck
import stage2_rod_model


REPO_ROOT = Path(__file__).resolve().parents[1]


class Stage2OpenRadiossDeckTest(unittest.TestCase):
    def build_deck(self, out_dir: Path):
        description = mass_model.load_dog_description(REPO_ROOT / "dog_description.yaml")
        catalog = mass_model.load_catalog(
            REPO_ROOT / "materials.yaml",
            REPO_ROOT / "actuators.yaml",
            REPO_ROOT / "batteries.yaml",
        )
        source = stage2_rod_model.build_stage1_model(description, catalog, "deck_test", 0.0, 0.0)
        rods = stage2_rod_model.build_whole_body_rod_model(source)
        deck = stage2_openradioss_deck.build_beam_deck(rods, out_dir, "deck_test_beam")
        stage2_openradioss_deck.write_deck(deck, make_gif=False, frames=1, duration_ms=65)
        return deck

    def test_generates_whole_body_beam_deck(self):
        with tempfile.TemporaryDirectory() as tmp:
            deck = self.build_deck(Path(tmp))
            starter = deck.starter_path.read_text(encoding="utf-8")
            engine = deck.engine_path.read_text(encoding="utf-8")

            self.assertIn("/MAT/ELAST/1", starter)
            self.assertIn("/PROP/TYPE3/1", starter)
            self.assertIn("/PART/1", starter)
            self.assertIn("/BEAM/1", starter)
            self.assertIn("/ADMAS/5/1", starter)
            self.assertIn("/H3D/NODA/VEL", engine)
            self.assertEqual(len(re.findall(r"^/BEAM/\d+", starter, flags=re.MULTILINE)), len(deck.members))

    def test_deck_is_topology_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            deck = self.build_deck(Path(tmp))
            starter = deck.starter_path.read_text(encoding="utf-8")
            summary = stage2_openradioss_deck.summary_dict(deck)

            for forbidden in ("/BCS", "/GRAV", "/CLOAD", "/LOAD", "/IMPDISP"):
                self.assertNotIn(forbidden, starter)

            state = summary["analysis_state"]
            self.assertTrue(state["topology_only"])
            self.assertFalse(state["gravity_applied"])
            self.assertFalse(state["fixed_boundary_conditions_applied"])
            self.assertFalse(state["support_reactions_applied"])
            self.assertFalse(state["load_cases_applied"])

    def test_deck_mass_matches_stage1_closely(self):
        with tempfile.TemporaryDirectory() as tmp:
            deck = self.build_deck(Path(tmp))
            source_mass = deck.rod_model.source_model.total_mass_kg
            deck_mass = stage2_openradioss_deck.deck_mass_kg(deck)
            self.assertLess(abs(deck_mass - source_mass), 0.001)


if __name__ == "__main__":
    unittest.main()
