#!/usr/bin/env python3

import json
import pathlib
import subprocess
import sys
import tempfile
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[2]
ANALYZER = (
    PACKAGE_ROOT / "tools" / "simulation" /
    "analyze_smpcc_bt_go_no_go.py")
SEEDS = (9911, 9912, 9913, 9914, 9915)
DISTURBANCES = ("D1_INITIAL_POSE", "D2_SHORT_LINEAR_SPEED_CAP")


def summary(method, disturbance, seed):
    smpcc = method == "SMPCC_BT"
    return {
        "mode": "smpcc_bt_direct" if smpcc else "bounded_tracking",
        "seed": seed,
        "go_no_go_disturbance": {"id": disturbance},
        "trial": {
            "sequence_completed": True,
            "task_success": True,
            "first_settled_goal_sec": 31.0 if smpcc else 30.0,
        },
        "controller_audit": {"controlled_stops": 0},
        "smpcc_bt_go_no_go_audit": {
            "candidate_failures": 0,
            "effective_correction_fraction": 0.20 if smpcc else None,
        },
        "primary_metric": {"value_m": 0.001005 if smpcc else 0.001},
        "secondary_metrics": {
            "tracking_q95_m": 0.85 if smpcc else 1.0,
            "external_measured_height_fixed_tail_q95_m":
                0.001010 if smpcc else 0.001,
        },
    }


class AnalyzeSmpccBtGoNoGoTest(unittest.TestCase):
    def write_campaign(self, root, mutate=None):
        for method in ("BT", "SMPCC_BT"):
            for disturbance in DISTURBANCES:
                for seed in SEEDS:
                    payload = summary(method, disturbance, seed)
                    if mutate is not None:
                        mutate(method, disturbance, seed, payload)
                    trial = root / f"{method}_{disturbance}_{seed}"
                    trial.mkdir()
                    (trial / "summary.json").write_text(
                        json.dumps(payload), encoding="utf-8")

    def run_analyzer(self, root):
        output = root / "decision.json"
        completed = subprocess.run(
            [sys.executable, str(ANALYZER),
             "--input-dir", str(root), "--output", str(output)],
            text=True, capture_output=True, check=False)
        return completed, json.loads(output.read_text(encoding="utf-8"))

    def test_all_frozen_thresholds_pass(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            self.write_campaign(root)
            completed, decision = self.run_analyzer(root)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertEqual(decision["decision"], "GO_ROUTE_A")
        self.assertEqual(decision["reasons"], [])

    def test_missing_settled_time_is_no_go_not_invalid_evidence(self):
        def mutate(method, disturbance, seed, payload):
            if (method, disturbance, seed) == (
                    "SMPCC_BT", "D1_INITIAL_POSE", 9911):
                payload["trial"]["first_settled_goal_sec"] = None

        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            self.write_campaign(root, mutate)
            completed, decision = self.run_analyzer(root)
        self.assertEqual(completed.returncode, 4, completed.stderr)
        self.assertEqual(decision["decision"], "NO_GO_ROUTE_B")
        self.assertIn(
            "D1_INITIAL_POSE:COMPLETION_TIME", decision["reasons"])


if __name__ == "__main__":
    unittest.main()
