#!/usr/bin/env python3
"""Exercise schema-5 postflight on real ROS messages/bags, without robot nodes."""
import copy
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS / "analysis"))
import validate_spmpc_ablation_smoke as validator
import genpy
import rosbag
from spmpc_local_planner.msg import PreSolveSnapshot, PredictedHorizon
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


def pair(zero=False, jerk=True, liquid=True):
    snapshot, horizon = PreSolveSnapshot(), PredictedHorizon()
    for msg in (snapshot, horizon):
        msg.valid = True
        msg.schema_version = 5
        msg.cycle_id = 1
        msg.solver_input_epoch = genpy.Time.from_sec(100.0)
        msg.slosh_enabled = liquid
        msg.zero_liquid_initial_state = zero
        msg.jerk_limit_enable = jerk
        msg.jerk_max = 1.0
        msg.delta_a_max = 1.0 / 30.0 if jerk else 1e15
        msg.dt = 1.0 / 30.0
        msg.horizon_steps = 60
    snapshot.actuator_a_cmd_memory = 0.2
    horizon.a = [0.2] * 60
    horizon.a_cmd_memory = [0.2] * 61
    for field in validator.LIQUID_FIELDS:
        setattr(snapshot, "observed_" + field, 0.001)
        setattr(snapshot, field, 0.0 if zero else 0.001)
        # NoState still predicts a nonzero future response.
        setattr(horizon, field, [getattr(snapshot, field)] + [0.002] * 60)
    return snapshot, horizon


class AblationPostflightTest(unittest.TestCase):
    def check_pair(self, snapshot, horizon, **overrides):
        expected = dict(liquid=True, zero=False, jerk=True, jerk_max=1.0)
        expected.update(overrides)
        return validator.validate_pair(snapshot, horizon, **expected)[0]

    def test_all_conditions(self):
        for liquid, zero, jerk in ((True, False, True), (True, True, True),
                                  (False, False, True), (False, False, False),
                                  (True, False, False)):
            self.assertEqual(self.check_pair(*pair(zero=zero, liquid=liquid, jerk=jerk),
                                            zero=zero, liquid=liquid, jerk=jerk), [])

    def test_first_and_last_stage_violations(self):
        for stage in (0, 59):
            snapshot, horizon = pair()
            horizon.a[stage] += 0.05
            horizon.a_cmd_memory[stage + 1] = horizon.a[stage]
            failures = self.check_pair(snapshot, horizon)
            self.assertTrue(any("bound violated at stage " + str(stage) in x for x in failures))

    def test_disabled_constraint_accepts_large_delta(self):
        snapshot, horizon = pair(jerk=False)
        horizon.a[0] = -0.2
        horizon.a_cmd_memory[1] = -0.2
        self.assertEqual(self.check_pair(snapshot, horizon, jerk=False), [])

    def test_wrong_switch_history_and_epoch_fail(self):
        original = pair()
        for field, value in (("jerk_limit_enable", False),
                             ("a_cmd_memory", [0.0] + [0.2] * 60),
                             ("solver_input_epoch", genpy.Time.from_sec(100.02))):
            snapshot, horizon = copy.deepcopy(original)
            setattr(horizon, field, value)
            self.assertTrue(self.check_pair(snapshot, horizon), field)

    def test_nostate_cannot_keep_observed_initial_state(self):
        snapshot, horizon = pair(zero=True)
        snapshot.eta_x = 0.001
        self.assertTrue(self.check_pair(snapshot, horizon, zero=True))

    def test_bag_topics_and_missing_config(self):
        snapshot, horizon = pair()
        config = Float32MultiArray()
        config.layout.dim = [MultiArrayDimension(
            label="slosh_enable,zero_liquid_initial_state,jerk_limit_enable,jerk_max")]
        config.data = [1.0, 0.0, 1.0, 1.0]
        with tempfile.TemporaryDirectory() as directory:
            for include_config in (True, False):
                path = str(Path(directory) / (str(include_config) + ".bag"))
                with rosbag.Bag(path, "w") as bag:
                    if include_config:
                        bag.write("/spmpc/debug/effective_config", config, genpy.Time(90))
                    for cycle in range(10):
                        snapshot.cycle_id = horizon.cycle_id = cycle
                        bag.write("/spmpc/debug/pre_solve_snapshot", snapshot, genpy.Time(100 + cycle))
                        bag.write("/spmpc/debug/predicted_horizon", horizon, genpy.Time(100 + cycle))
                report = validator.validate_bag(SimpleNamespace(
                    bag=path, slosh_enable=True, zero_liquid_initial_state=False,
                    jerk_limit_enable=True, jerk_max=1.0))
                self.assertEqual(report["checked_pairs"], 10)
                self.assertEqual(report["status"], "PASS" if include_config else "FAIL")


class AblationEntryTest(unittest.TestCase):
    def test_invalid_arguments_fail_before_acquisition(self):
        entry = str(SCRIPTS / "run_spmpc_ablation_smoke.sh")
        for args in (("--condition", "typo"), ("--jerk-max", "nan"),
                     ("--jerk-max", "0"), ("--jerk-max", "-1"), ("--jerk-max",)):
            result = subprocess.run(["bash", entry, *args], capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertNotIn("motion NOT started", result.stdout)

    def test_real_runner_forwards_switches_to_launch(self):
        # Evaluate only its command-array expression, never the acquisition body.
        source = (SCRIPTS / "run_spmpc_real_fixed_path_trial.sh").read_text()
        command_array = source[source.index("planner_cmd=("):source.index("planner_command_string=")]
        for liquid, zero, jerk in (("true", "false", "true"), ("true", "true", "true"),
                                  ("false", "false", "true"), ("false", "false", "false"),
                                  ("true", "false", "false")):
            script = ('VARIANT=B_slosh; SLOSH_ENABLE=$1; ZERO_LIQUID_INITIAL_STATE=$2; '
                      'JERK_LIMIT_ENABLE=$3; JERK_MAX=0.8; TERMINAL_MPC_STOP_HANDOFF_ENABLE=true\n' + command_array
                      + '\nprintf "%s\\n" "${planner_cmd[@]}"')
            result = subprocess.run(["bash", "-c", script, "test", liquid, zero, jerk],
                                    check=True, capture_output=True, text=True)
            args = result.stdout.splitlines()
            self.assertIn("slosh_enable:=" + liquid, args)
            self.assertIn("zero_liquid_initial_state:=" + zero, args)
            self.assertIn("jerk_limit_enable:=" + jerk, args)
            self.assertIn("jerk_max:=0.8", args)
            self.assertIn("terminal_mpc_stop_handoff_enable:=true", args)


if __name__ == "__main__":
    unittest.main()
