#!/usr/bin/env python3

import importlib.util
import unittest
from pathlib import Path

import numpy as np


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "analyze_mocap_imu_relative_latency.py"
)
SPEC = importlib.util.spec_from_file_location("relative_latency", str(SCRIPT_PATH))
LATENCY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LATENCY)


class RelativeLatencyAnalysisTest(unittest.TestCase):
    DT_SEC = 0.005

    @classmethod
    def reference_signal(cls):
        time = np.arange(0.0, 30.0, cls.DT_SEC)
        return (
            np.sin(2.0 * np.pi * 0.6 * time)
            + 0.45 * np.sin(2.0 * np.pi * 1.1 * time)
        )

    @classmethod
    def shifted_signal(cls, reference, lag_sec):
        count = int(round(abs(lag_sec) / cls.DT_SEC))
        if lag_sec > 0.0:
            return np.concatenate((np.zeros(count), reference[:-count]))
        if lag_sec < 0.0:
            return np.concatenate((reference[count:], np.zeros(count)))
        return reference.copy()

    def assert_recovers_lag(self, expected_lag_sec, polarity=1):
        reference = self.reference_signal()
        response = polarity * self.shifted_signal(reference, expected_lag_sec)
        result = LATENCY.estimate_lag(reference, response, self.DT_SEC, 0.20)
        self.assertAlmostEqual(result["lag_sec"], expected_lag_sec, places=12)
        self.assertGreater(result["peak_correlation_abs"], 0.99)
        self.assertEqual(result["polarity"], polarity)
        self.assertFalse(result["at_search_boundary"])

    def test_positive_means_nokov_response_is_later(self):
        self.assert_recovers_lag(0.080)

    def test_negative_means_nokov_response_is_earlier(self):
        self.assert_recovers_lag(-0.045)

    def test_axis_polarity_does_not_change_lag_sign(self):
        self.assert_recovers_lag(0.060, polarity=-1)


if __name__ == "__main__":
    unittest.main()
