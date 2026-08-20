#!/usr/bin/env python3

import csv
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
ANALYSIS_DIR = PACKAGE_ROOT / "tools" / "analysis"
sys.path.insert(0, str(ANALYSIS_DIR))
MODULE_PATH = ANALYSIS_DIR / "analyze_g2s_raw_rgb_three_trial.py"
SPEC = importlib.util.spec_from_file_location("analyze_g2s_raw_rgb_three_trial", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class AnalyzeG2sRawRgbThreeTrialTest(unittest.TestCase):
    def write_csv(self, path, clipped_index=None):
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=[
                    "frame_index",
                    "stamp_sec",
                    "any_clipped",
                    "h_mm_max_lcr_smooth_corr",
                ],
            )
            writer.writeheader()
            for index in range(40):
                writer.writerow(
                    {
                        "frame_index": index * 5,
                        "stamp_sec": 10.0 + index * 0.1,
                        "any_clipped": int(index == clipped_index),
                        "h_mm_max_lcr_smooth_corr": -1.0 if index == 0 else 0.25,
                    }
                )

    def test_load_offline_rgb_applies_motion_window_and_nonnegative_floor(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.csv"
            self.write_csv(path)
            records, audit = MODULE.load_offline_rgb(path, 10.0, 13.9)
        self.assertEqual(len(records), 40)
        self.assertEqual(records[0][1], 0.0)
        self.assertEqual(audit["clipped_frame_count"], 0)
        self.assertEqual(audit["frame_stride"], 5)

    def test_load_offline_rgb_rejects_any_clipping(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.csv"
            self.write_csv(path, clipped_index=4)
            with self.assertRaises(RuntimeError):
                MODULE.load_offline_rgb(path, 10.0, 13.9)


if __name__ == "__main__":
    unittest.main()
