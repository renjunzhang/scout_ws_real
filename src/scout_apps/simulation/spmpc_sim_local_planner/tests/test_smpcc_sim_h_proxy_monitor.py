#!/usr/bin/env python3
"""Pure-Python contract tests for the source-separated H_proxy monitor."""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts/smpcc_sim_h_proxy_monitor.py"
LAUNCH_PATH = ROOT / "launch/smpcc_sim_h_proxy_monitor.launch"


def load_monitor():
    spec = importlib.util.spec_from_file_location("smpcc_sim_h_proxy_monitor_test", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class HProxyMonitorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.monitor = load_monitor()

    def test_modal_proxy_is_deterministic_and_odom_excited(self):
        first = self.monitor.ModalProxy(0.0185, 0.058, 1000.0, 1, 0.05, 0.02, True, False)
        second = self.monitor.ModalProxy(0.0185, 0.058, 1000.0, 1, 0.05, 0.02, True, False)
        for _ in range(20):
            first.update(0.6, 0.15, 0.25, 0.0)
            second.update(0.6, 0.15, 0.25, 0.0)
        self.assertGreater(first.height(0.25), 0.0)
        self.assertEqual(first.x, second.x)
        self.assertEqual(first.height(0.25), second.height(0.25))

    def test_invalid_geometry_or_mode_fails_closed(self):
        with self.assertRaises(self.monitor.ConfigurationError):
            self.monitor.ModalProxy(0.0, 0.058, 1000.0, 1, 0.05, 0.02, True, False)
        with self.assertRaises(self.monitor.ConfigurationError):
            self.monitor.ModalProxy(0.0185, 0.058, 1000.0, 8, 0.05, 0.02, True, False)

    def test_launch_is_sim_owned_and_has_no_command_input(self):
        launch = LAUNCH_PATH.read_text(encoding="utf-8")
        source = MODULE_PATH.read_text(encoding="utf-8")
        self.assertIn('pkg="spmpc_sim_local_planner"', launch)
        self.assertIn("smpcc_sim_h_proxy_monitor.py", launch)
        self.assertIn('subscribe_cmd_vel_debug" default="false"', launch)
        self.assertNotIn("slosh_models", launch)
        self.assertNotIn("Subscriber(\"/cmd_vel\"", source)
        self.assertIn("one input, executed ``nav_msgs/Odometry``", source)
        self.assertIn("never liquid plant truth", source)


if __name__ == "__main__":
    unittest.main()
