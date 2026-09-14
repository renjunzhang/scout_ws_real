#!/usr/bin/env python3
"""Check that screening cannot silently become a different validation chain."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))
import freeze_internal_slosh_evaluation as freeze


class EvaluationFreezeTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.chain = {"sha256": "test-chain", "files": {"model": "model-sha"}}
        self.full = self.candidate("full")
        self.smooth = self.candidate("smooth")
        self.output = self.root / "lock.json"

    def candidate(self, condition, weight=1.0, protocol=None, jerk_max=0.6,
                  weights=None):
        protocol = protocol or freeze.PROTOCOL
        prefix = "/spmpc_local_planner/"
        params = {prefix + k: v for k, v in {
            "planner_variant": "B_slosh", "slosh_observer/source": "processed_imu",
            "ablation/jerk_limit_enable": True, "ablation/jerk_max": jerk_max,
            "ablation/zero_liquid_initial_state": False, "variants/B_slosh/v_ref": 0.2,
            "variants/B_slosh/slosh_enable": condition == "full",
            "variants/B_slosh/w_slosh": weight if condition == "full" else 0,
            "variants/B_slosh/w_smooth": 0.1,
            "slosh/natural_frequency": 32.4, "processed_imu/filter_hz": 20,
        }.items()}
        if protocol == freeze.PARAMETER_PROTOCOL:
            for key, value in (weights or {"w_v": 1.0, "w_contour": 1.0, "w_lag": 0.2}).items():
                params[prefix + "variants/B_slosh/" + key] = value
        launch = self.root / (condition + "_launch_params.yaml")
        launch.write_text(yaml.safe_dump(params))
        report = {
            "bag": str(self.root / (condition + ".bag")),
            "status": "PASS", "eligible_for_comparison": True,
            "prereg": {"protocol": protocol, "phase": "screening", "condition": condition,
                       "w_slosh": str(weight if condition == "full" else 0),
                       "jerk_max": str(jerk_max),
                       "evaluation_chain_sha256": self.chain["sha256"],
                       "launch_params_sha256": freeze.sha256(launch),
                       "path_sha256": "path-sha", "map_sha256": "map-sha"},
            "launch_params_file": str(launch), "launch_params_sha256": freeze.sha256(launch),
        }
        if protocol == freeze.PARAMETER_PROTOCOL:
            report["prereg"]["evaluation_primary_monitor"] = "imu"
            report["prereg"].update({key: str(value) for key, value in (weights or {"w_v": 1.0, "w_contour": 1.0, "w_lag": 0.2}).items()})
            report["effective_config_first"] = dict(
                (weights or {"w_v": 1.0, "w_contour": 1.0, "w_lag": 0.2}),
                jerk_max=jerk_max)
        target = self.root / (condition + ".json")
        target.write_text(json.dumps(report))
        return target

    def create(self):
        return freeze.create_lock(self.full, self.smooth, "imu", "development candidate", self.output, self.chain)

    def test_freeze_and_all_four_rows_then_refuse_overwrite(self):
        lock = self.create()
        for row, condition in freeze.ROWS.items():
            launch = (self.root / (condition + "_launch_params.yaml")).read_text()
            self.assertEqual(freeze.check_lock(lock, condition, row, launch,
                             "path-sha", "map-sha", self.chain), "imu")
        with self.assertRaisesRegex(ValueError, "preserve"):
            self.create()

    def test_wrong_row_monitor_model_build_params_or_path_rejected(self):
        lock = self.create()
        launch = (self.root / "full_launch_params.yaml").read_text()
        for change in (
            {"row": "02"}, {"chain": {"sha256": "new-build"}},
            {"path_sha": "new-path"}, {"map_sha": "new-map"},
            {"launch_text": launch.replace("w_slosh: 1.0", "w_slosh: 0.5")},
            {"launch_text": launch.replace("filter_hz: 20", "filter_hz: 10")},
            {"lock": dict(lock, primary_monitor="nokov")},
        ):
            args = dict(lock=lock, condition="full", row="01", launch_text=launch,
                        path_sha="path-sha", map_sha="map-sha", chain=self.chain)
            args.update(change)
            with self.assertRaises(ValueError, msg=str(change)):
                freeze.check_lock(**args)

    def test_failed_old_protocol_validation_or_stale_candidate_rejected(self):
        base = json.loads(self.full.read_text())
        for field, value in (("phase", "validation"), ("protocol", "V1"),
                             ("condition", "b0"), ("evaluation_chain_sha256", "old-code")):
            report = copy.deepcopy(base)
            report["prereg"][field] = value
            self.full.write_text(json.dumps(report))
            with self.assertRaises(ValueError):
                self.create()
        self.full.write_text(json.dumps(dict(base, eligible_for_comparison=False)))
        with self.assertRaises(ValueError):
            self.create()

    def test_nonliquid_cost_difference_and_tampered_dump_rejected(self):
        launch = self.root / "smooth_launch_params.yaml"
        launch.write_text(launch.read_text().replace("w_smooth: 0.1", "w_smooth: 1.0"))
        with self.assertRaisesRegex(ValueError, "SHA mismatch"):
            self.create()
        report = json.loads(self.smooth.read_text())
        report["launch_params_sha256"] = report["prereg"]["launch_params_sha256"] = freeze.sha256(launch)
        self.smooth.write_text(json.dumps(report))
        with self.assertRaisesRegex(ValueError, "non-liquid"):
            self.create()

    def test_v3_lock_and_protocol_monitor_and_parameter_rejections(self):
        weights = {"w_v": 2.0, "w_contour": 1.5, "w_lag": 0.3}
        self.full = self.candidate("full", protocol=freeze.PARAMETER_PROTOCOL,
                                   jerk_max=1.0, weights=weights)
        self.smooth = self.candidate("smooth", protocol=freeze.PARAMETER_PROTOCOL,
                                     jerk_max=1.0, weights=weights)
        lock = freeze.create_lock(self.full, self.smooth, "imu", "v3", self.output,
                                  self.chain, freeze.PARAMETER_PROTOCOL)
        launch = (self.root / "full_launch_params.yaml").read_text()
        self.assertEqual(freeze.check_lock(lock, "full", "01", launch,
                                           "path-sha", "map-sha", self.chain,
                                           freeze.PARAMETER_PROTOCOL), "imu")
        with self.assertRaises(ValueError):
            freeze.check_lock(lock, "full", "01", launch, "path-sha", "map-sha",
                              self.chain, freeze.PROTOCOL)
        with self.assertRaises(ValueError):
            freeze.create_lock(self.full, self.smooth, "odom", "v3", self.root / "odom.json",
                               self.chain, freeze.PARAMETER_PROTOCOL)
        for key, value in (("w_v", 0), ("w_contour", 21), ("w_lag", float("nan"))):
            bad = dict(weights, **{key: value})
            with self.assertRaises(ValueError):
                freeze.check_params(yaml.safe_load(launch), "full", 1.0,
                                    freeze.PARAMETER_PROTOCOL, 1.0, bad)
        with self.assertRaises(ValueError):
            freeze.check_params(yaml.safe_load(launch), "full", 1.0,
                                freeze.PARAMETER_PROTOCOL, 0.7, weights)

        full_text = self.full.read_text()
        for source, key, value in (("prereg", "w_v", "0.5"),
                                   ("effective_config_first", "w_contour", 0.5),
                                   ("prereg", "evaluation_primary_monitor", "odom")):
            report = json.loads(full_text)
            report[source][key] = value
            self.full.write_text(json.dumps(report))
            with self.assertRaises(ValueError):
                freeze.read_candidate(self.full, "full", self.chain, freeze.PARAMETER_PROTOCOL)
        self.full.write_text(full_text)
        # Individually valid reports with different common parameters must
        # not become a paired candidate; V2/V3 cannot share a lock either.
        self.smooth = self.candidate("smooth", protocol=freeze.PARAMETER_PROTOCOL,
                                     jerk_max=1.0, weights=dict(weights, w_v=0.5))
        with self.assertRaisesRegex(ValueError, "non-liquid"):
            freeze.create_lock(self.full, self.smooth, "imu", "different common weights",
                               self.root / "mismatch.json", self.chain, freeze.PARAMETER_PROTOCOL)
        self.smooth = self.candidate("smooth")
        with self.assertRaisesRegex(ValueError, "screening"):
            freeze.create_lock(self.full, self.smooth, "imu", "mixed protocols",
                               self.root / "mixed.json", self.chain, freeze.PARAMETER_PROTOCOL)

if __name__ == "__main__":
    unittest.main()
