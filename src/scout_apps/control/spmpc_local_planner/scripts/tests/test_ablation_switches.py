#!/usr/bin/env python3
"""Resolve the real launch includes without starting ROS nodes or publishing motion."""

import subprocess
import unittest
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[2]
PREFIX = "/spmpc_local_planner/"


def resolve(launch, *overrides):
    result = subprocess.run(
        ["roslaunch", "--dump-params", str(PACKAGE / "launch" / launch),
         "planner_variant:=B_slosh", "publish_cmd_vel:=false", *overrides],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True,
        timeout=30,
    )
    return yaml.safe_load(result.stdout)


class AblationSwitchLaunchTest(unittest.TestCase):
    def test_conditions_preserve_common_motion_and_observer_settings(self):
        conditions = {
            "b0": (False, False, False),
            "smooth": (False, False, True),
            "nostate": (True, True, True),
            "full": (True, False, True),
            "no_jerk": (True, False, False),
        }
        common = [
            "v_ref:=0.20", "w_accel:=0.3", "w_alpha:=0.1", "w_du_a:=0.1",
            "observer_source:=processed_imu", "observer_fallback_policy:=fail_closed",
            "terminal_mpc_stop_handoff_enable:=true",
            "jerk_max:=0.8",
        ]
        previous = None
        varying = {
            PREFIX + "variants/B_slosh/slosh_enable",
            PREFIX + "variants/B_slosh/w_slosh",
            PREFIX + "ablation/zero_liquid_initial_state",
            PREFIX + "ablation/jerk_limit_enable",
        }
        for condition, (liquid, zero, jerk) in conditions.items():
            with self.subTest(condition=condition):
                params = resolve(
                    "spmpc_fixed_path.launch", *common,
                    "slosh_enable:=" + str(liquid).lower(),
                    "zero_liquid_initial_state:=" + str(zero).lower(),
                    "jerk_limit_enable:=" + str(jerk).lower(),
                    "w_slosh:=" + ("1.0" if liquid else "0.0"),
                )
                self.assertEqual(params[PREFIX + "variants/B_slosh/slosh_enable"], liquid)
                self.assertEqual(params[PREFIX + "ablation/zero_liquid_initial_state"], zero)
                self.assertEqual(params[PREFIX + "ablation/jerk_limit_enable"], jerk)
                self.assertEqual(params[PREFIX + "ablation/jerk_max"], 0.8)
                self.assertEqual(params[PREFIX + "slosh_observer/source"], "processed_imu")
                self.assertFalse(params[PREFIX + "publish_cmd_vel"])
                frozen = {k: v for k, v in params.items() if k not in varying}
                if previous is not None:
                    self.assertEqual(frozen, previous)
                previous = frozen

    def test_defaults_and_point_to_point_forwarding(self):
        defaults = resolve("spmpc_experiment.launch")
        self.assertTrue(defaults[PREFIX + "variants/B_slosh/slosh_enable"])
        self.assertFalse(defaults[PREFIX + "ablation/zero_liquid_initial_state"])
        self.assertFalse(defaults[PREFIX + "ablation/jerk_limit_enable"])
        changed = resolve(
            "spmpc_point_to_point.launch", "zero_liquid_initial_state:=true",
            "jerk_limit_enable:=true", "jerk_max:=0.7",
        )
        self.assertTrue(changed[PREFIX + "ablation/zero_liquid_initial_state"])
        self.assertTrue(changed[PREFIX + "ablation/jerk_limit_enable"])
        self.assertEqual(changed[PREFIX + "ablation/jerk_max"], 0.7)


if __name__ == "__main__":
    unittest.main()
