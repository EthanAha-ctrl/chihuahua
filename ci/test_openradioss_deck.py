import tempfile
import unittest
import re
from pathlib import Path

from src.components.z_common import mass_model
from src.components.c_fem import openradioss_deck as openradioss_deck
from src.components.c_fem import rod_model as rod_model
REPO_ROOT = Path(__file__).resolve().parents[1]


class FemOpenRadiossDeckTest(unittest.TestCase):
    def build_deck(self, out_dir: Path):
        description = mass_model.load_dog_description(REPO_ROOT / "vendor/reference/urdf/chihuahua/lists.md")
        catalog = mass_model.load_catalog(
            REPO_ROOT / "vendor/components/material/lists.md",
            REPO_ROOT / "vendor/components/joint/lists.md",
            REPO_ROOT / "vendor/components/battery/lists.md",
        )
        source = rod_model.build_basic_motion_model(description, catalog, "deck_test", 0.0, 0.0)
        rods = rod_model.build_whole_body_rod_model(source)
        deck = openradioss_deck.build_beam_deck(rods, out_dir, "deck_test_beam")
        openradioss_deck.write_deck(deck, make_gif=False, frames=1, duration_ms=65)
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
            summary = openradioss_deck.summary_dict(deck)

            for forbidden in ("/BCS", "/GRAV", "/CLOAD", "/LOAD", "/IMPDISP"):
                self.assertNotIn(forbidden, starter)

            state = summary["analysis_state"]
            self.assertTrue(state["topology_only"])
            self.assertFalse(state["gravity_applied"])
            self.assertFalse(state["fixed_boundary_conditions_applied"])
            self.assertFalse(state["support_reactions_applied"])
            self.assertFalse(state["load_cases_applied"])

    def test_deck_mass_matches_basic_motion_closely(self):
        with tempfile.TemporaryDirectory() as tmp:
            deck = self.build_deck(Path(tmp))
            source_mass = deck.rod_model.source_model.total_mass_kg
            deck_mass = openradioss_deck.deck_mass_kg(deck)
            self.assertLess(abs(deck_mass - source_mass), 0.001)


if __name__ == "__main__":
    unittest.main()
