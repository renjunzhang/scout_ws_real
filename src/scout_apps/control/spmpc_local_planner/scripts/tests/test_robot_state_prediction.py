#!/usr/bin/env python3
"""Scientific/IO contracts for the offline initial-state vs model diagnosis."""

import copy
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS / "analysis"))
sys.path.insert(0, str(SCRIPTS / "acados"))
import analyze_robot_state_prediction as analysis
import robot_state_prediction_core as core
import spmpc_acados_model as model


def checked_reference():
    config = json.loads(analysis.DEFAULT_REFERENCE.read_text())
    config.update(status="verified_independent", provenance="Synthetic independent coordinate/time truth",
                  calibration_bag_sha256=["calibration_only"])
    config["uncertainty"] = dict(time_sec=0.001, position_m=0.0001, yaw_rad=0.001,
                                 v_mps=0.001, omega_radps=0.001)
    return config


def true_state(epoch):
    t, v, omega = epoch - 100.0, 0.2, 0.15
    return np.array([v / omega * math.sin(omega*t),
                     v / omega * (1 - math.cos(omega*t)), omega*t, v, omega])


def make_snapshot(epoch=101.0, cycle_id=50):
    record = dict(schema_version=5, valid=True, backend="continuous_mpcc_acados_explicit_actuator",
                  variant="B_slosh", solver_status="ACADOS_SUCCESS", control_semantics="a_cmd_alpha_cmd",
                  state_width=model.NX_SLOSH, control_width=3, parameter_width=model.NP_SLOSH,
                  dt=1/30, horizon_steps=60, actuator_state_valid=True, frame="map", cycle_id=cycle_id,
                  solver_input_epoch=epoch, robot_state_stamp=epoch, raw_robot_state_stamp=epoch - 0.02,
                  parameter_names=model.PARAM_NAMES_SLOSH, s0=0.5,
                  eta_x=0.001, eta_x_dot=-0.01, eta_y=-0.002, eta_y_dot=0.02,
                  actuator_v_cmd=0.2, actuator_omega_cmd=0.15, actuator_a_cmd_memory=0.0,
                  actuator_delayed_v_cmd=0.2, actuator_delayed_omega_cmd=0.15,
                  actuator_linear_delay_queue=[0.2]*5, actuator_angular_delay_queue=[0.15]*10,
                  jerk_limit_enable=True, jerk_max=1.0, zero_liquid_initial_state=False)
    for name, value in zip(core.ROBOT_FIELDS, true_state(epoch)):
        record["robot_" + name] = value
    params = np.ones(model.NP_SLOSH)
    for name, value in dict(actuator_dt=1/30, actuator_tau_v=0.112, actuator_tau_omega=0.119,
                             actuator_gain_v=1.0, actuator_gain_omega=1.0).items():
        params[model.PIDX_SLOSH[name]] = value
    record["stage_parameters"] = np.tile(params, 61).tolist()
    return record


def synthetic_data(initial_error=False, model_error=False):
    audits = []
    for i, t in enumerate(np.arange(100.0, 104.0, 0.02)):
        audit = {flag: False for flag in analysis.AUDIT_FLAGS}
        audit.update(cycle_id=i, epoch=t, stamp=t, bag_time=t+0.005, v=0.2, omega=0.15,
                     solve_attempted=True, solve_success=True, command_was_published=True,
                     command_accepted=True)
        audits.append(audit)
    snapshots = [make_snapshot(audits[i]["epoch"], i) for i in range(50, 81, 10)]
    for s in snapshots:
        if initial_error:
            for name, perturbation in zip(core.ROBOT_FIELDS, (0.03, -0.02, 0.1, 0.05, -0.04)):
                s["robot_" + name] += perturbation
        if model_error:
            params = np.array(s["stage_parameters"]).reshape((61, -1))
            params[:, model.PIDX_SLOSH["actuator_gain_v"]] = 1.3
            s["stage_parameters"] = params.ravel().tolist()
    time = np.arange(100.0, 104.0, 0.01)
    mocap = [[t, *true_state(t)[:3]] for t in time]
    return dict(snapshots=snapshots, audits=audits,
                cmd=[[a["bag_time"]-0.001, a["v"], a["omega"]] for a in audits],
                mocap=mocap, odom=[], imu=[], imu_debug=[], effective_config=["synthetic full/J1"],
                counts={analysis.SNAPSHOT_TOPIC: len(snapshots), analysis.AUDIT_TOPIC: len(audits)},
                invalid={}, timing={"/vrpn_client_node/Tracker0/pose": [[t, 0.005] for t in time]},
                frames={"solver": {"map"}, "mocap": {"world"}})


def options():
    return analysis.parse_args(["synthetic.bag", "--leads", "0.2", "0.5"])


class NumericalTest(unittest.TestCase):
    def test_delayed_zoh_step_matches_analytic_response_and_position(self):
        params = core.snapshot_contract(make_snapshot())
        params.update(delay_v=0.2, delay_omega=0.3)
        commands = np.array([[9., 0., 0.], [10.1, 0.2, 0.], [10.5, 0., 0.]])
        time = np.array([10.2, 10.3, 10.4, 10.65, 10.8])
        states = core.replay_robot(np.zeros(5), 10., time, commands, params)
        tau = params["actuator_tau_v"]
        for t, state in zip(time, states):
            elapsed = max(0, t-10.3)
            off = max(0, t-10.7)
            expected_v = 0.2 * (1-math.exp(-elapsed/tau)) - 0.2*(1-math.exp(-off/tau))
            expected_x = 0.2 * (elapsed-tau*(1-math.exp(-elapsed/tau))) - 0.2*(off-tau*(1-math.exp(-off/tau)))
            self.assertAlmostEqual(state[3], expected_v, delta=1e-12)
            self.assertAlmostEqual(state[0], expected_x, delta=1e-10)

    def test_replay_time_translation_and_substep_convergence(self):
        params = core.snapshot_contract(make_snapshot())
        commands = np.array([[9., 0.1, 0.2], [10.12, 0.3, -0.1], [10.35, 0.15, 0.3]])
        times = np.array([10.2, 10.4, 10.7])
        a = core.replay_robot(np.zeros(5), 10, times, commands, params)
        b = core.replay_robot(np.zeros(5), 10, times, commands, params, max_step_sec=0.001)
        shifted = commands.copy()
        shifted[:, 0] += 1000
        c = core.replay_robot(np.zeros(5), 1010, times+1000, shifted, params)
        np.testing.assert_allclose(a, b, atol=1e-10)
        np.testing.assert_allclose(a, c, atol=1e-10)

    def test_constant_command_matches_existing_casadi_ocp_robot_step(self):
        import casadi as ca
        symbols = model.export_spmpc_slosh_symbols()
        step = ca.Function("robot_step_test", [symbols["x"], symbols["u"], symbols["p"]], [symbols["disc_dyn"]])
        snapshot = make_snapshot()
        params = core.snapshot_contract(snapshot)
        x = np.zeros(model.NX_SLOSH)
        x[:4] = [0.1, -0.2, 0.3, 0.12]
        x[5:8] = [0.07, 0.2, 0.15]
        x[8:13], x[13:23] = 0.2, 0.15
        p = snapshot["stage_parameters"][:model.NP_SLOSH]
        discrete = np.array(step(x, np.zeros(3), p)).ravel()[[0, 1, 2, 3, 5]]
        continuous = core.constant_input_step(x[[0, 1, 2, 3, 5]], 0.2, 0.15, 0.112, 0.119, 1/30)
        # The existing one-stage RK4 and exact continuous response differ slightly.
        np.testing.assert_allclose(discrete, continuous, atol=2e-6)

    def test_marker_lever_arm_rotation_and_yaw_wrap(self):
        reference = checked_reference()
        reference.update(tracker_to_base_yaw_rad=0.4, base_to_tracker_m=[0.12, -0.03],
                         mocap_stamp_offset_sec=-0.015)
        reference["mocap_to_solver"] = dict(x_m=2., y_m=-1., yaw_rad=0.2)
        base_yaw = np.linspace(3.0, 3.8, 30)
        q = base_yaw-0.4
        lever = reference["base_to_tracker_m"]
        marker_x = 0.3+np.cos(base_yaw)*lever[0]-np.sin(base_yaw)*lever[1]
        marker_y = 0.5+np.sin(base_yaw)*lever[0]+np.cos(base_yaw)*lever[1]
        rows = np.column_stack([np.arange(30)*0.01+100, marker_x, marker_y, core.wrap_angle(q)])
        transformed = core.transform_mocap(rows, reference)
        np.testing.assert_allclose(transformed[:, 1], 2+0.3*math.cos(.2)-.5*math.sin(.2), atol=1e-12)
        np.testing.assert_allclose(transformed[:, 2], -1+.3*math.sin(.2)+.5*math.cos(.2), atol=1e-12)
        np.testing.assert_allclose(transformed[:, 3], base_yaw+.2, atol=1e-12)
        np.testing.assert_allclose(transformed[:, 0], rows[:, 0]-.015)

    def test_corrected_initial_state_has_no_future_sample_leakage(self):
        time = np.linspace(100, 101, 101)
        poses = np.array([true_state(t)[:3] for t in time])
        support = core.causal_support(time, 100.6, 0.12, 0.035)
        a = core.fit_state(time[support], poses[support], 100.6, 0.12)[0]
        poses[time > 100.6] += 999
        b = core.fit_state(time[support], poses[support], 100.6, 0.12)[0]
        np.testing.assert_array_equal(a, b)
        np.testing.assert_allclose(a, true_state(100.6), atol=1e-6)
        with self.assertRaisesRegex(core.ContractError, "NONCAUSAL"):
            core.fit_state(time, poses, 100.6, 0.12)

    def test_only_robot_initial_state_is_replaced(self):
        original = make_snapshot()
        saved = copy.deepcopy(original)
        result = core.replace_robot_initial_state(original, np.arange(5))
        self.assertEqual(original, saved)
        for key in original:
            if key not in {"robot_" + name for name in core.ROBOT_FIELDS}:
                self.assertEqual(original[key], result[key])

    def test_gaps_and_invalid_commands_are_not_filled_or_sorted(self):
        with self.assertRaisesRegex(core.ContractError, "regressing"):
            core.strict_rows([[100, 0, 0], [99, 0, 0]], 3, "commands")
        with self.assertRaisesRegex(core.ContractError, "repeated"):
            core.strict_rows([[100, 0, 0], [100, 0, 0]], 3, "commands")
        with self.assertRaisesRegex(core.ContractError, "MISSING"):
            core.check_command_support(np.array([[100, 0, 0], [101, 0, 0]]), 99, 101, .12)
        with self.assertRaisesRegex(core.ContractError, "GAP"):
            core.check_command_support(np.array([[100, 0, 0], [101, 0, 0]]), 100, 101, .12)
        with self.assertRaisesRegex(core.ContractError, "GAPPED"):
            core.causal_support(np.array([100, 100.01, 100.02, 100.03, 100.1, 100.11]), 100.12, .12, .035)

    def test_stage_parameter_and_queue_contracts(self):
        snapshot = make_snapshot()
        snapshot["stage_parameters"][model.NP_SLOSH+model.PIDX_SLOSH["actuator_tau_v"]] = 99
        with self.assertRaisesRegex(core.ContractError, "CHANGE_WITH_STAGE"):
            core.snapshot_contract(snapshot)
        snapshot = make_snapshot()
        snapshot["actuator_linear_delay_queue"][0] = 0.7
        commands = np.array([[100, .2, .15], [102, .2, .15]])
        with self.assertRaisesRegex(core.ContractError, "QUEUE_HISTORY"):
            core.check_snapshot_history(snapshot, commands)


class AttributionAndReportTest(unittest.TestCase):
    def test_correcting_initial_state_removes_synthetic_initial_error(self):
        report, initial, _, _ = analysis.evaluate(synthetic_data(initial_error=True), checked_reference(), options(), "evaluation")
        self.assertEqual(report["issues"], [])
        self.assertEqual(len(initial), 4)
        for metric in report["future_errors"]:
            for channel in ("position", "yaw", "v", "omega"):
                self.assertGreater(metric[channel]["rmse_reduction_pct"], 99.)

    def test_correct_initial_state_cannot_remove_wrong_dynamics(self):
        report, _, _, _ = analysis.evaluate(synthetic_data(model_error=True), checked_reference(), options(), "evaluation")
        self.assertLess(report["initial_errors"]["v"]["rmse"], 1e-6)
        for metric in report["future_errors"]:
            self.assertGreater(metric["v"]["B"]["rmse"], .03)
            self.assertAlmostEqual(metric["v"]["B"]["rmse"] / metric["v"]["A"]["rmse"], 1., delta=.001)

    def test_no_initial_error_is_a_b_symmetric(self):
        report, _, _, _ = analysis.evaluate(synthetic_data(), checked_reference(), options(), "evaluation")
        for metric in report["future_errors"]:
            for channel in ("position", "yaw", "v", "omega"):
                self.assertLess(metric[channel]["A"]["rmse"], 1e-6)
                self.assertLess(metric[channel]["B"]["rmse"], 1e-6)

    def test_unverified_same_bag_calibration_and_bad_frame_block_inference(self):
        unverified = json.loads(analysis.DEFAULT_REFERENCE.read_text())
        report, initial, _, _ = analysis.evaluate(synthetic_data(), unverified, options(), "evaluation")
        self.assertEqual(report["status"], "INCONCLUSIVE")
        self.assertEqual(len(initial), 4)
        config = checked_reference()
        config["calibration_bag_sha256"] = ["evaluation"]
        report, _, _, _ = analysis.evaluate(synthetic_data(), config, options(), "evaluation")
        self.assertIn("REFERENCE_FITTED_ON_EVALUATION_BAG", report["issues"])
        data = synthetic_data()
        data["frames"]["mocap"] = {"different_world"}
        report, initial, _, _ = analysis.evaluate(data, checked_reference(), options(), "evaluation")
        self.assertEqual(initial, [])
        self.assertIn("MOCAP_FRAME_MISMATCH", report["issues"])

    def test_missing_command_ledger_and_robot_epoch_are_visible(self):
        data = synthetic_data()
        del data["cmd"][55]
        report, _, _, _ = analysis.evaluate(data, checked_reference(), options(), "evaluation")
        self.assertIn("COMMAND_LEDGER_INCOMPLETE", report["issues"])
        data = synthetic_data()
        data["snapshots"][0]["robot_state_stamp"] -= .01
        report, _, _, _ = analysis.evaluate(data, checked_reference(), options(), "evaluation")
        self.assertEqual(report["excluded_anchors"]["ROBOT_STATE_EPOCH_MISMATCH"], 1)

    def test_safety_commands_retained_and_future_windows_stratified(self):
        data = synthetic_data()
        data["audits"][66]["safety_gate_intervened"] = True
        data["audits"][66]["solve_success"] = False
        report, _, future, _ = analysis.evaluate(data, checked_reference(), options(), "evaluation")
        self.assertEqual(report["command_ledger"]["audit_published"], len(data["audits"]))
        self.assertEqual(report["whole_bag_runtime"]["solve_failures"], 1)
        self.assertTrue(any(r["future_intervention"] for r in future))

    def test_reader_uses_source_stamps_and_keeps_final_safety_commands(self):
        def stamp(value):
            return NS(to_sec=lambda: value)
        data = synthetic_data()
        s = data["snapshots"][0]
        snapshot = NS(**{**s, "solver_input_epoch": stamp(s["solver_input_epoch"]),
                         "robot_state_stamp": stamp(s["robot_state_stamp"]),
                         "raw_robot_state_stamp": stamp(s["raw_robot_state_stamp"]),
                         "header": NS(frame_id="map")})
        audit = data["audits"][50]
        msg = NS(**audit, solver_input_epoch=stamp(audit["epoch"]),
                 command_publish_stamp=stamp(audit["stamp"]), published_cmd_v=.2, published_cmd_omega=.15)
        msg.safety_gate_intervened = True
        pose = NS(header=NS(stamp=stamp(100.1), frame_id="world"),
                  pose=NS(position=NS(x=0., y=0.), orientation=NS(x=0., y=0., z=0., w=1.)))
        bad_pose = copy.deepcopy(pose)
        bad_pose.header.stamp = stamp(0)
        records = analysis.read_records([(analysis.SNAPSHOT_TOPIC, snapshot, stamp(200)),
            (analysis.AUDIT_TOPIC, msg, stamp(200)),
            ("/vrpn_client_node/Tracker0/pose", pose, stamp(200)),
            ("/vrpn_client_node/Tracker0/pose", bad_pose, stamp(200))], "Tracker0")
        self.assertEqual(records["snapshots"][0]["solver_input_epoch"], s["solver_input_epoch"])
        self.assertEqual(records["audits"][0]["stamp"], audit["stamp"])
        self.assertTrue(records["audits"][0]["safety_gate_intervened"])
        self.assertEqual(records["mocap"][0][0], 100.1)
        self.assertEqual(len(records["mocap"]), 1)
        self.assertEqual(sum(records["invalid"].values()), 1)

    def test_report_rendering_json_and_non_overwrite_cli(self):
        data = synthetic_data(initial_error=True)
        report, initial, future, anchors = analysis.evaluate(data, checked_reference(), options(), "evaluation")
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)
            analysis.write_report(output, report, initial, future, anchors, data)
            self.assertEqual(json.loads((output / "summary.json").read_text())["paired_anchors"], 4)
            self.assertIn("同包滚动窗口", (output / "summary.md").read_text())
            for name in ("01_timing.png", "02_current_state.png", "03_future_error.png"):
                self.assertGreater((output / name).stat().st_size, 10000)
            bag = output / "existing.bag"
            bag.write_text("not read: existing output should be rejected first")
            with patch.object(analysis, "read_bag", side_effect=AssertionError("must not read")):
                self.assertEqual(analysis.main([str(bag), "--output-dir", tmp]), 2)
            reference_file = output / "reference.json"
            reference_file.write_text(json.dumps(checked_reference()))
            cli_output = output / "cli_report"
            with patch.object(analysis, "read_bag", return_value=data):
                self.assertEqual(analysis.main([str(bag), "--reference-config", str(reference_file),
                    "--output-dir", str(cli_output), "--leads", "0.2", "0.5"]), 0)
            written = json.loads((cli_output / "summary.json").read_text())
            self.assertEqual(written["provenance"]["bag_sha256"], analysis.file_sha256(bag))
            self.assertIn("robot_state_prediction_core.py", written["provenance"]["analysis_files_sha256"])

    def test_current_cli_rejects_leads_that_share_initial_measurements(self):
        with self.assertRaises(SystemExit):
            analysis.parse_args(["test.bag", "--leads", "0.033", "0.1"])


if __name__ == "__main__":
    unittest.main()
