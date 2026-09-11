import hashlib
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "analysis" / "analyze_internal_slosh_pair.py"
SPEC = importlib.util.spec_from_file_location("internal_slosh_metrics", SCRIPT)
METRICS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(METRICS)


class InternalSloshMetricsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def _topics(self, invalid=False):
        audits = [
            {"command_was_published": True, "command_publish_stamp": 10.0,
             "published_cmd_v": 0.2, "published_cmd_omega": 0.0, "status": "RUNNING"},
            {"command_was_published": True, "command_publish_stamp": 20.0,
             "published_cmd_v": 0.0, "published_cmd_omega": 0.0, "status": "GOAL_REACHED"},
        ]
        snapshots = []
        for stamp, x in ((0.0, 0.0), (5.0, 0.0), (10.0, 0.0), (12.0, 0.2),
                         (15.0, 0.5), (18.0, 0.8), (20.0, 1.0), (25.0, 1.0)):
            snapshots.append({
                "valid": True, "robot_state_stamp": stamp, "robot_x": x,
                "robot_y": 0.01, "robot_yaw": 0.0,
            })
        monitor_rows = []
        for index in range(1281):
            stamp = 9.0 + index * (16.0 / 1280.0)
            row = {"state_stamp": float(stamp), "valid": True, "configured": True,
                   "modal_height_m": 0.001 + (stamp % 3) * 0.0005}
            monitor_rows.append(row)
        if invalid:
            monitor_rows[80] = dict(monitor_rows[80], valid=False)
        return {
            "audit": [{"value": row} for row in audits],
            "snapshot": [{"value": row} for row in snapshots],
            "imu": [{"value": row} for row in monitor_rows],
            "odom": [{"value": row} for row in monitor_rows],
            "config": [{"value": {"w_slosh": 1.0, "v_ref": 0.2, "horizon_steps": 20.0}}],
            "path": [{"value": {"frame_id": "map", "xy": [[0.0, 0.0], [1.0, 0.0]]}}],
        }

    def _pass_report(self, name="run", invalid=False, condition="full", start_offset=0.0):
        bag = self.root / (name + ".bag")
        bag.touch()
        for suffix in METRICS.POSTFLIGHT_SUFFIXES:
            (self.root / (name + "_" + suffix + ".json")).write_text(
                json.dumps({"status": "PASS"}), encoding="utf-8"
            )
        prereg = self.root / (name + "_prereg.env")
        prereg.write_text(
            "phase=validation\nprotocol={}\nrunner_exit_code=0\ndiagnostics_exit_code=0\ncondition={}\nevaluation_row={}\n".format(
                METRICS.PROTOCOL,
                condition, {"full": "01", "smooth": "02"}.get(condition, "01")
            ), encoding="utf-8"
        )
        report = METRICS.analyze_topics(self._topics(invalid), bag, prereg_path=prereg)
        report["windows"]["task"][0] += start_offset
        report["eligible_for_comparison"] = report["status"] == "PASS"
        return report

    def test_window_and_monitor_statistics(self):
        bag = self.root / "metrics.bag"
        bag.touch()
        for suffix in METRICS.POSTFLIGHT_SUFFIXES:
            (self.root / ("metrics_" + suffix + ".json")).write_text('{"status":"PASS"}')
        prereg = self.root / "metrics_prereg.env"
        prereg.write_text("condition=full\nphase=validation\nprotocol={}\nrunner_exit_code=0\ndiagnostics_exit_code=0\n".format(METRICS.PROTOCOL))
        report = METRICS.analyze_topics(self._topics(), bag, prereg_path=prereg)
        self.assertEqual(report["windows"]["task"], [10.0, 20.0])
        self.assertAlmostEqual(report["windows"]["path10_90"][0], 11.0)
        self.assertEqual(report["task_duration_sec"], 10.0)
        self.assertGreater(report["monitor_height_mm"]["task"]["imu"]["n"], 100)
        self.assertAlmostEqual(report["monitor_height_mm"]["task"]["imu"]["peak_mm"], 2.5, delta=0.01)
        self.assertTrue(report["monitor_height_mm"]["task"]["imu"]["coverage_valid"])
        self.assertTrue(report["tracking"]["snapshot_time_order_valid"])

    def test_invalid_monitor_is_fail_closed(self):
        bag = self.root / "invalid.bag"
        bag.touch()
        for suffix in METRICS.POSTFLIGHT_SUFFIXES:
            (self.root / ("invalid_" + suffix + ".json")).write_text('{"status":"PASS"}')
        prereg = self.root / "invalid_prereg.env"
        prereg.write_text("condition=full\nphase=validation\nprotocol={}\nrunner_exit_code=0\ndiagnostics_exit_code=0\n".format(METRICS.PROTOCOL))
        report = METRICS.analyze_topics(self._topics(invalid=True), bag, prereg_path=prereg)
        self.assertFalse(report["monitor_height_mm"]["task"]["imu"]["continuous_valid"])
        self.assertFalse(report["eligible_for_comparison"])
        self.assertIn("monitor_continuity", report["gate_failures"])

    def test_single_report_rejects_unstable_config_and_keeps_identity_on_path_failure(self):
        bag = self.root / "bad-config.bag"
        bag.touch()
        launch = self.root / "bad-config_launch_params.yaml"
        launch.write_text("condition: full\n")
        for suffix in METRICS.POSTFLIGHT_SUFFIXES:
            (self.root / ("bad-config_" + suffix + ".json")).write_text('{"status":"PASS"}')
        prereg = self.root / "bad-config_prereg.env"
        launch_sha = hashlib.sha256(launch.read_bytes()).hexdigest()
        prereg.write_text(
            "condition=full\nphase=validation\nprotocol={}\nrunner_exit_code=0\n"
            "diagnostics_exit_code=0\nlaunch_params_sha256={}\nevaluation_chain_sha256={}\n".format(
                METRICS.PROTOCOL, launch_sha, METRICS._current_evaluation_chain_sha()
            )
        )
        topics = self._topics()
        topics["config"].append({"value": dict(topics["config"][0]["value"], v_ref=0.3)})
        report = METRICS.analyze_topics(topics, bag, prereg_path=prereg)
        self.assertEqual(report["status"], "FAIL")
        self.assertIn("effective_config", report["gate_failures"])
        self.assertEqual(report["prereg"]["protocol"], METRICS.PROTOCOL)
        self.assertEqual(report["postflights"]["recording_postflight"], "PASS")

        topics["path"][0]["value"]["xy"] = [[0.0, 0.0], [0.0, 0.0]]
        report = METRICS.analyze_topics(topics, bag, prereg_path=prereg)
        self.assertEqual(report["prereg"]["protocol"], METRICS.PROTOCOL)
        self.assertEqual(report["postflights"]["recording_postflight"], "PASS")
        self.assertIn("error", report)

    def test_nonzero_runner_or_diagnostics_exit_is_ineligible(self):
        bag = self.root / "exit-failure.bag"
        bag.touch()
        for suffix in METRICS.POSTFLIGHT_SUFFIXES:
            (self.root / ("exit-failure_" + suffix + ".json")).write_text('{"status":"PASS"}')
        prereg = self.root / "exit-failure_prereg.env"
        prereg.write_text(
            "condition=full\nphase=validation\nrunner_exit_code=0\ndiagnostics_exit_code=7\n"
        )
        report = METRICS.analyze_topics(self._topics(), bag, prereg_path=prereg)
        self.assertFalse(report["eligible_for_comparison"])
        self.assertIn("diagnostics_exit_code", report["gate_failures"])

    def test_screening_preserves_reports_without_selecting_winner(self):
        first = self.root / "first.json"
        second = self.root / "second.json"
        first.write_text(json.dumps({"monitor_height_mm": {"task": {"imu": {"rms_mm": 1.0}}}}))
        second.write_text(json.dumps({"monitor_height_mm": {"task": {"imu": {"rms_mm": 2.0}}}}))
        output = self.root / "screen.json"
        result = METRICS.compare_reports([first, second], output)
        self.assertEqual(result["status"], "SCREENING")
        self.assertFalse(result["winner_selected"])
        self.assertEqual(len(result["reports"]), 2)

    def _write_locked_reports(self):
        reports = []
        files = []
        for index, (row, condition) in enumerate((("01", "full"), ("02", "smooth"), ("03", "smooth"), ("04", "full"))):
            bag = self.root / ("row" + row + ".bag")
            bag.touch()
            launch = self.root / ("row" + row + "_launch_params.yaml")
            launch.write_text("condition: {}\n".format(condition))
            for suffix in METRICS.POSTFLIGHT_SUFFIXES:
                (self.root / ("row" + row + "_" + suffix + ".json")).write_text('{"status":"PASS"}')
            prereg = self.root / ("row" + row + "_prereg.env")
            prereg.write_text(
                "phase=validation\nprotocol={}\nrunner_exit_code=0\ndiagnostics_exit_code=0\ncondition={}\nevaluation_row={}\n".format(METRICS.PROTOCOL, condition, row)
            )
            report = METRICS.analyze_topics(self._topics(), bag, prereg_path=prereg)
            report["effective_config_first"]["w_slosh"] = 1.0 if condition == "full" else 0.0
            report["windows"]["task"][0] = 100.0 + index * 10.0
            report["eligible_for_comparison"] = True
            report["launch_params_sha256"] = hashlib.sha256(launch.read_bytes()).hexdigest()
            reports.append(report)
            files.append(self.root / ("report" + row + ".json"))
        lock = self.root / "lock.json"
        chain = METRICS._current_evaluation_chain_sha()
        lock.write_text(json.dumps({
            "schema_version": 1, "protocol": METRICS.PROTOCOL, "primary_monitor": "imu",
            "rows": {"01": "full", "02": "smooth", "03": "smooth", "04": "full"},
            "full_w_slosh": 1.0, "smooth_w_slosh": 0.0, "created_at_epoch_sec": 1.0,
            "launch_params_sha256": {"full": reports[0]["launch_params_sha256"], "smooth": reports[1]["launch_params_sha256"]},
            "evaluation_chain_sha256": chain,
        }, sort_keys=True))
        lock_sha = hashlib.sha256(lock.read_bytes()).hexdigest()
        for report in reports:
            report["prereg"]["evaluation_lock_sha256"] = lock_sha
            report["prereg"]["evaluation_primary_monitor"] = "imu"
            report["prereg"]["evaluation_chain_sha256"] = chain
        for path, report in zip(files, reports):
            path.write_text(json.dumps(report))
        return files, lock

    def test_locked_compare_computes_both_full_to_smooth_pairs(self):
        reports, lock = self._write_locked_reports()
        output = self.root / "comparison.json"
        result = METRICS.compare_reports(reports, output, lock)
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(set(result["pair_changes"]), {"01_vs_02", "04_vs_03"})
        value = result["pair_changes"]["01_vs_02"]["task"]["rms_mm_percent_change_full_vs_smooth"]
        self.assertAlmostEqual(value, 0.0)

    def test_locked_compare_rejects_wrong_primary_monitor_and_lock_sha(self):
        reports, lock = self._write_locked_reports()
        payload = json.loads(lock.read_text())
        payload["primary_monitor"] = "odom"
        lock.write_text(json.dumps(payload, sort_keys=True))
        result = METRICS.compare_reports(reports, self.root / "bad.json", lock)
        self.assertEqual(result["status"], "FAIL")
        self.assertTrue(any("primary monitor mismatch" in failure for failure in result["gate_failures"]))
        self.assertTrue(any("evaluation lock SHA mismatch" in failure for failure in result["gate_failures"]))

    def test_locked_compare_preserves_fail_summary_when_metrics_are_missing(self):
        reports, lock = self._write_locked_reports()
        payload = json.loads(reports[1].read_text())
        payload["monitor_height_mm"] = {}
        reports[1].write_text(json.dumps(payload))
        output = self.root / "missing-metrics.json"
        result = METRICS.compare_reports(reports, output, lock)
        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["pair_changes_role"], "diagnostic_only")
        self.assertFalse(result["effect_claim_allowed"])
        self.assertTrue(output.is_file())

    def test_locked_compare_rejects_old_out_of_order_and_screening_bags(self):
        reports, lock = self._write_locked_reports()
        original = reports[0].read_text()
        for field, value in (("task_start", 0.5), ("task_start", 125.0),
                             ("phase", "screening"), ("evaluation_row", "02")):
            payload = json.loads(original)
            if field == "task_start":
                payload["windows"]["task"][0] = value
            else:
                payload["prereg"][field] = value
            reports[0].write_text(json.dumps(payload))
            result = METRICS.compare_reports(reports, self.root / "rejected.json", lock)
            self.assertEqual(result["status"], "FAIL", (field, value))

    def test_percentage_sign_and_float32_weights(self):
        reports, lock = self._write_locked_reports()
        full = json.loads(reports[0].read_text())
        smooth = json.loads(reports[1].read_text())
        full["monitor_height_mm"]["task"]["imu"]["rms_mm"] = 0.8
        smooth["monitor_height_mm"]["task"]["imu"]["rms_mm"] = 1.0
        reports[0].write_text(json.dumps(full))
        reports[1].write_text(json.dumps(smooth))
        result = METRICS.compare_reports(reports, self.root / "sign.json", lock)
        self.assertAlmostEqual(result["pair_changes"]["01_vs_02"]["task"]
                               ["rms_mm_percent_change_full_vs_smooth"], -20.0)
        self.assertTrue(METRICS._close_number(0.10000000149011612, 0.1))
        self.assertFalse(METRICS._close_number(0.2, 0.1))


if __name__ == "__main__":
    unittest.main()
