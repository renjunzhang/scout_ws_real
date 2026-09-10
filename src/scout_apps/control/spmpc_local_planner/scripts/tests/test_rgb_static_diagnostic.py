#!/usr/bin/env python3
"""Regression cases that would produce incorrect acquisition/causal conclusions."""
import importlib.util
from pathlib import Path
import struct
import tempfile
import threading
import unittest

spec = importlib.util.spec_from_file_location("rgb_diag", Path(__file__).resolve().parents[1] /
                                            "diagnose_spmpc_rgb_recording.py")
diag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diag)


def row(t, seq, counter=None):
    return {"stamp_ns": round(t * 1e9), "seq": seq, "receive": t + .02,
            "frame_counter": counter}


class RGBDiagnosticTests(unittest.TestCase):
    def test_short_initial_burst_cannot_pass_long_window(self):
        rows = [row(1 + i / 30, i) for i in range(3)]
        self.assertEqual(diag.summarize(rows, .1, 30, 1, 21)["status"], "FAIL")

    def test_empty_stream_fails(self):
        self.assertEqual(diag.summarize([], .1, 30, 1, 21)["status"], "FAIL")

    def test_complete_window_passes(self):
        rows = [row(1 + i / 30, i) for i in range(600)]
        self.assertEqual(diag.summarize(rows, .1, 30, 1, 21)["status"], "PASS")

    def test_timestamp_reset_not_hidden_by_sorting(self):
        rows = [row(t, i) for i, t in enumerate([1, 1.033, 1.01, 1.066])]
        self.assertIn("source timestamp regression/duplicate", diag.summarize(rows, .1, 30)["failures"])

    def test_counter_skip_with_contiguous_driver_sequence_is_upstream(self):
        info = [row(1, 80), row(1.066, 81)]
        meta = [row(1, 1, 100), row(1.066, 2, 102)]
        result = diag.evidence({diag.INFO: info, diag.META: meta}, {})
        self.assertEqual(result["counter_skips_with_contiguous_driver_seq"], 1)

    def test_monitor_sequence_skip_cannot_prove_upstream_loss(self):
        info = [row(1, 80), row(1.066, 82)]
        meta = [row(1, 1, 100), row(1.066, 3, 102)]
        result = diag.evidence({diag.INFO: info, diag.META: meta}, {})
        self.assertEqual(result["counter_skips_with_contiguous_driver_seq"], 0)
        self.assertEqual(result["driver_seq_skips_seen_live"], 1)

    def test_compare_only_overlap_while_preserving_internal_missing_frame(self):
        live = [row(1 + i / 30, i) for i in range(10)]
        bag = [live[i] for i in [2, 3, 5, 6, 7]]
        result = diag.evidence({diag.INFO: live}, {diag.INFO: bag})
        self.assertEqual(result["live_bag_comparison"][diag.INFO]["live_present_bag_missing"], 1)

    def test_raw_header_preserves_nanosecond_identity(self):
        value = diag.header_row(struct.pack("<III", 123, 1789031420, 409221001), 1)
        self.assertEqual(value["stamp_ns"], 1789031420409221001)
        self.assertEqual(value["seq"], 123)

    def test_camera_clock_offset_does_not_shift_bag_receive_window(self):
        rows = [dict(row(103 + i / 30, i), receive=101 + i / 30) for i in range(90)]
        lo, hi = diag.source_window({diag.INFO: rows}, 101.5, 102.5)
        self.assertAlmostEqual(lo, 103.5)
        self.assertAlmostEqual(hi, 104.5)

    def test_missing_live_window_is_not_replaced_with_previous_samples(self):
        with self.assertRaises(RuntimeError):
            diag.source_window({diag.INFO: [row(1, 1), row(2, 2)]}, 10, 20)

    def test_camera_reentering_auto_mode_fails_even_with_stable_config_snapshot(self):
        cfg = {"exposure": 100, "gain": 64, "white_balance": 4600}
        records = [{"metadata": {"auto_exposure": 0, "auto_white_balance_temperature": 0}},
                   {"metadata": {"auto_exposure": 1, "auto_white_balance_temperature": 0}}]
        self.assertTrue(diag.camera_lock_failures(records, cfg))

    def test_waits_for_cpp_recorder_to_rename_after_launcher_exits(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "test.bag"
            active = Path(str(path) + ".active")
            active.write_bytes(b"closing")
            worker = threading.Timer(.05, lambda: active.rename(path))
            worker.start()
            try:
                diag.wait_for_bag_close(path, timeout_sec=1)
                self.assertFalse(active.exists())
            finally:
                worker.join()

    def test_frame_gap_alone_is_not_reported_as_clock_offset(self):
        gate = {"status": "FAIL", "failures": ["source max gap 0.13s exceeds 0.1s"]}
        self.assertEqual(diag.clock_only_health(gate)["status"], "PASS")
        self.assertEqual(gate["status"], "FAIL")

    def test_future_clock_skew_is_preserved(self):
        gate = {"status": "FAIL", "failures": ["source stamp future skew 1.95s exceeds 0.05s"]}
        self.assertEqual(diag.clock_only_health(gate)["status"], "FAIL")

    def test_missing_exposure_metadata_cannot_pass_lock(self):
        cfg = {"exposure": 100, "gain": 64, "white_balance": 4600}
        records = [{"metadata": {"auto_exposure": 0, "auto_white_balance_temperature": 0,
                                  "gain_level": 64, "manual_white_balance": 4600}}]
        self.assertIn("actual_exposure unavailable", diag.camera_lock_failures(records, cfg))

    def test_full_load_missing_tracker_cannot_pass(self):
        self.assertEqual(diag.context_coverage([], 1, 61)["status"], "FAIL")
        times = [1 + i / 50 for i in range(3000) if not 1000 < i < 1100]
        self.assertEqual(diag.context_coverage(times, 1, 61)["status"], "FAIL")

    def test_full_load_coverage_uses_receive_time(self):
        times = [i / 50 for i in range(3200)]
        self.assertEqual(diag.context_coverage(times, 1, 61)["status"], "PASS")


if __name__ == "__main__":
    unittest.main()
