#!/usr/bin/env python3

import pathlib
import unittest

import yaml


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[2]
VARIANTS_PATH = PACKAGE_ROOT / "config" / "planner" / "variants.yaml"


class ShortHorizonMatchedReleaseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with VARIANTS_PATH.open("r", encoding="utf-8") as stream:
            cls.variants = yaml.safe_load(stream)["variants"]

    def test_pair_differs_only_in_liquid_weight(self):
        matched0 = dict(self.variants["B_slosh_matched0"])
        matched5 = dict(self.variants["B_slosh_matched5"])
        self.assertEqual(matched0.pop("w_slosh"), 0.0)
        self.assertEqual(matched5.pop("w_slosh"), 5.0)
        self.assertEqual(matched0, matched5)

    def test_release_uses_short_trusted_window_and_same_10d_solver(self):
        for name in ("B_slosh_matched0", "B_slosh_matched5"):
            config = self.variants[name]
            self.assertIs(config["slosh_enable"], True)
            self.assertEqual(config["slosh_cost_horizon_steps"], 3)
            self.assertEqual(config["slosh_cost_tail_discount"], 0.0)
            self.assertEqual(config["w_control"], 0.3)
            self.assertEqual(config["w_smooth"], 1.0)
            self.assertEqual(config["w_alpha"], 1.0)
            self.assertEqual(config["w_du_a"], 1.0)
            self.assertEqual(config["w_du_vs"], 1.0)

    def test_literal_short100_differs_from_historical_bslosh_only_by_window(self):
        historical = dict(self.variants["B_slosh"])
        short100 = dict(self.variants["B_slosh_short100"])
        self.assertEqual(short100.pop("slosh_cost_horizon_steps"), 3)
        self.assertEqual(short100.pop("slosh_cost_tail_discount"), 0.0)
        self.assertEqual(short100, historical)
        self.assertNotIn("slosh_cost_horizon_steps", historical)
        self.assertNotIn("slosh_cost_tail_discount", historical)


if __name__ == "__main__":
    unittest.main()
