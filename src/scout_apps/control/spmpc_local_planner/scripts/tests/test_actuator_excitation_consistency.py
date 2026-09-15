#!/usr/bin/env python3
"""Analytic excitation, causality and cached CLI checks for three-way replay."""
import contextlib
import copy
from dataclasses import asdict
import gzip
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import yaml

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS / "analysis"))
sys.path.insert(0, str(SCRIPTS / "acados"))
import actuator_excitation_consistency_core as core
import analyze_actuator_excitation_consistency as cli
import spmpc_acados_model as model
from analyze_ocp_imu_forecast import HORIZON_TOPIC, sha256
from robot_state_prediction_core import command_at


PROCESSING = core.ImuProcessing(10.0, 12.0, .015, .006834, .005020, .015001, -.1, .045)


def fixture(changed_commands=False, true_tau=.112):
    epoch, dt = 100.0, 1/30
    stamps = 99.006 + np.arange(130)*dt
    change = float(stamps[37])
    commands = np.array([[t, .3 if changed_commands and t >= change else .2,
                          .18 if changed_commands and t >= change else .1] for t in stamps])
    audits = [dict(cycle_id=i+1, solver_input_epoch=float(t-.006), command_publish_stamp=float(t),
                   solve_attempted=True, solve_success=True, command_accepted=True,
                   command_was_published=True, observer_source=2,
                   published_cmd_v=float(v), published_cmd_omega=float(w))
              for i, (t, v, w) in enumerate(commands)]
    ident = 31
    h = dict(schema_version=5, valid=True, backend="continuous_mpcc_acados_explicit_actuator",
             variant="B_slosh", solver_status="B_slosh_ACADOS_OK", cycle_id=ident,
             solver_input_epoch=epoch, solve_start_stamp=epoch+.001, solve_end_stamp=epoch+.004,
             robot_state_stamp=epoch, liquid_state_stamp=epoch,
             horizon_available_stamp=epoch+.004, control_semantics="a_cmd_alpha_cmd",
             slosh_enabled=True, zero_liquid_initial_state=False, dt=dt, horizon_steps=60,
             t=(np.arange(61)*dt).tolist(), v=[.2]*61, omega=[.1]*61,
             delayed_v_cmd=[.2]*61, delayed_omega_cmd=[.1]*61,
             a_actual=[0.0]*61, alpha_actual=[0.0]*61,
             eta_x=[.002]*61, eta_x_dot=[0.0]*61, eta_y=[0.0]*61, eta_y_dot=[0.0]*61,
             h_modal=[.004]*61, header_stamp=9999)
    s = {k: v for k, v in h.items() if not isinstance(v, list)}
    s.update(state_width=model.NX_SLOSH, control_width=3, parameter_width=model.NP_SLOSH,
             actuator_state_valid=True, robot_x=0.0, robot_y=0.0, robot_yaw=0.0, robot_v=.2,
             robot_omega=.1, robot_state_stamp=epoch, eta_x=.002, eta_x_dot=0.0, eta_y=0.0,
             eta_y_dot=0.0, actuator_v_cmd=.2, actuator_omega_cmd=.1,
             actuator_delayed_v_cmd=.2, actuator_delayed_omega_cmd=.1,
             actuator_a_cmd_memory=0.0, parameter_names=model.PARAM_NAMES_SLOSH,
             actuator_linear_delay_queue=[float(command_at(commands, epoch-(5-i)*dt)[0]) for i in range(5)],
             actuator_angular_delay_queue=[float(command_at(commands, epoch-(10-i)*dt)[1]) for i in range(10)])
    p = np.ones(model.NP_SLOSH)
    for name, value in dict(actuator_dt=dt, actuator_tau_v=.112, actuator_tau_omega=.119,
                             actuator_gain_v=1.0, actuator_gain_omega=1.0, eta_ref=.005).items():
        p[model.PIDX_SLOSH[name]] = value
    s["stage_parameters"] = np.tile(p, 61).tolist()
    times = 99.901 + np.arange(113)*.02
    rx, ry = PROCESSING.lever_arm_imu_to_target_x_m, PROCESSING.lever_arm_imu_to_target_y_m
    # Known steady target ax=0, ay=.02; initial filtered acceleration is at IMU.
    fx, fy, fw = .1**2*rx, .02+.1**2*ry, .1
    rows = []
    for i, t in enumerate(times):
        def response(before, after, delay, tau):
            elapsed = float(t)-change-delay
            if not changed_commands or elapsed <= 0:
                return before, 0.0
            value = after+(before-after)*math.exp(-elapsed/tau)
            return value, (after-value)/tau
        v, ax = response(.2, .3, 5*dt, true_tau)
        omega, alpha = response(.1, .18, 10*dt, .119)
        ay = v*omega
        # Independently apply the documented digital IMU recurrence to truth.
        qa = math.exp(-2*math.pi*10*.02)
        qw = math.exp(-2*math.pi*12*.02)
        fx = qa*fx + (1-qa)*(ax+alpha*ry+omega**2*rx)
        fy = qa*fy + (1-qa)*(ay-alpha*rx+omega**2*ry)
        old_w = fw
        fw = qw*fw + (1-qw)*omega
        fa = (fw-old_w)/.02
        rows.append(dict(valid=True, configured=True, filter_ready=True, bias_ready=True,
                         source=2, measurement_stamp=float(t), source_stamp=float(t+.015),
                         receive_stamp=float(t+.016), accel_effective_stamp=float(t-.006834),
                         gyro_effective_stamp=float(t-.005020), alpha_effective_stamp=float(t-.015001),
                         sample_dt_sec=.02, ax_mps2=fx-fa*ry-fw**2*rx,
                         ay_mps2=fy+fa*rx-fw**2*ry, omega_z_radps=fw, alpha_z_radps2=fa,
                         accel_filtered_base_x_mps2=fx, accel_filtered_base_y_mps2=fy,
                         excitation_axes_frame="base_link",
                         excitation_reference_point="liquid_observer_target_icr_proxy",
                         observer_update_count=i+1, reset_epoch=0))
    return [h], [s], audits, rows, PROCESSING, {"task": [99.9, 103.0]}, 2.0


class ExcitationTest(unittest.TestCase):
    def test_steady_motion_matches_and_ignores_header_time(self):
        report, rows = core.analyze(*fixture())
        self.assertEqual(report["status"], "DIAGNOSTIC_COMPLETE")
        self.assertGreater(len(rows), 80)
        self.assertTrue(all(100 < r["measurement_stamp"] < 102.01 for r in rows))
        for pair in ("ocp_vs_imu", "replay_vs_imu", "ocp_vs_replay"):
            for axis in core.AXES:
                self.assertLess(report["groups"]["task"][pair][axis]["rmse"], 1e-10)
        self.assertFalse(report["method"]["parameters_fitted"])

    def test_actual_command_change_is_separated_from_model_error(self):
        report, _ = core.analyze(*fixture(changed_commands=True))
        group = report["groups"]["task"]
        self.assertGreater(group["ocp_vs_imu"]["ax"]["rmse"], .1)
        self.assertGreater(group["ocp_vs_replay"]["ax"]["rmse"], .1)
        for axis in core.AXES:
            self.assertLess(group["replay_vs_imu"][axis]["rmse"], 1e-8)

    def test_wrong_time_constant_remains_a_replay_residual(self):
        report, _ = core.analyze(*fixture(changed_commands=True, true_tau=.25))
        self.assertGreater(report["groups"]["task"]["replay_vs_imu"]["ax"]["rmse"], .05)

    def test_recorded_plan_holds_queue_head_until_next_node(self):
        horizons, snapshots, *_ = fixture()
        h = horizons[0]
        params = core.snapshot_contract(snapshots[0])
        dt, tau = h["dt"], params["actuator_tau_v"]
        h["delayed_v_cmd"][0] = .3
        h["delayed_v_cmd"][1] = .1
        v1 = .3 + (.2-.3)*math.exp(-dt/tau)
        h["v"][1] = v1
        samples = core.recorded_plan_motion(h, params, np.array([100+dt/2, 100+dt, 100+1.5*dt]))
        expected = [(.3-(.3+(.2-.3)*math.exp(-dt/2/tau)))/tau,
                    (.1-v1)/tau,
                    (.1-(.1+(v1-.1)*math.exp(-dt/2/tau)))/tau]
        self.assertTrue(np.allclose(samples[:, 0], expected, atol=1e-10, rtol=0))

    def test_angular_motion_requires_matching_lever_arm_and_filter(self):
        args = fixture(changed_commands=True)
        h, s, _, imu, processing, _, _ = args
        monitor = core.ImuSamples(imu, processing, .035)
        seed, rows = monitor.window(100, 102)
        raw = core.recorded_plan_motion(h[0], core.snapshot_contract(s[0]),
                                        np.array([r["measurement_stamp"] for r in rows]))
        filtered = core.process_model(raw, rows, seed, processing)
        self.assertTrue(np.allclose(filtered[:, 0], 0, atol=1e-10))
        altered = asdict(processing)
        altered["lever_arm_imu_to_target_x_m"] = 0
        with self.assertRaisesRegex(ValueError, "lever-arm"):
            core.ImuSamples(imu, core.ImuProcessing(**altered), .035)

    def test_measurement_metadata_rejects_frame_or_time_mismatch(self):
        for field, value in (("excitation_axes_frame", "map"),
                             ("excitation_reference_point", "tracker"),
                             ("accel_effective_stamp", 98.0)):
            args = fixture()
            args[3][8][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                core.analyze(*args)

    def test_gap_reset_and_invalid_samples_exclude_window(self):
        for kind in ("gap", "reset", "invalid"):
            args = fixture()
            if kind == "gap":
                del args[3][25:28]
            elif kind == "reset":
                for row in args[3][25:]:
                    row["reset_epoch"] = 1
            else:
                args[3][25]["valid"] = False
            report, rows = core.analyze(*args)
            with self.subTest(kind=kind):
                self.assertEqual(rows, [])
                self.assertIn("gap or reset", report["rejected_origins"][0]["reason"])

    def test_filter_seed_must_be_available_at_forecast_origin(self):
        args = fixture()
        for row in args[3]:
            if row["measurement_stamp"] <= 100:
                row["receive_stamp"] = 100.1
        report, rows = core.analyze(*args)
        self.assertFalse(rows)
        self.assertIn("causal", report["rejected_origins"][0]["reason"])

    def test_command_gap_and_bad_queue_are_not_silently_filled(self):
        for kind in ("gap", "queue"):
            args = fixture()
            if kind == "gap":
                del args[2][48:53]
            else:
                args[1][0]["actuator_linear_delay_queue"][2] = .7
            report, rows = core.analyze(*args)
            with self.subTest(kind=kind):
                self.assertFalse(rows)
                self.assertTrue(report["rejected_origins"])

    def test_duplicate_cycles_and_failed_origin_are_rejected(self):
        for kind in ("duplicate", "failed", "wrong_epoch", "state_epoch"):
            args = fixture()
            if kind == "duplicate":
                args[0].append(copy.deepcopy(args[0][0]))
            elif kind == "failed":
                args[2][30]["solve_success"] = False
            elif kind == "wrong_epoch":
                args[1][0]["solver_input_epoch"] += .1
            else:
                args[1][0]["robot_state_stamp"] -= .01
            report, rows = core.analyze(*args)
            with self.subTest(kind=kind):
                self.assertFalse(rows)
                self.assertEqual(len(report["rejected_origins"]), 1)

    def test_future_intervention_and_cross_goal_are_reported_separately(self):
        args = fixture()
        args[2][40]["terminal_controller_intervened"] = True
        args[5]["task"][1] = 100.8
        report, rows = core.analyze(*args)
        self.assertTrue(rows)
        self.assertGreater(report["groups"]["task"]["count"], 0)
        self.assertGreater(report["groups"]["cross_goal"]["count"], 0)
        self.assertEqual(report["groups"]["task_no_future_intervention"]["count"], 0)

    def test_invalid_parameters_and_no_requested_cycle_are_explicit(self):
        for kwargs in (dict(duration=float("nan")), dict(max_imu_gap=float("inf")),
                       dict(max_command_gap=-1)):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                core.analyze(*fixture(), **kwargs)
        report, rows = core.analyze(*fixture(), cycle_id=999)
        self.assertEqual(report["status"], "INCONCLUSIVE")
        self.assertFalse(rows)

    def test_cached_cli_writes_diagnostic_artifacts_without_reading_rosbag(self):
        args = fixture(changed_commands=True)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bag = root / "synthetic.bag"
            bag.write_bytes(b"Synthetic identity only; all topics are cached.")
            identity = dict(bag=str(bag.resolve()), size=bag.stat().st_size, mtime_ns=bag.stat().st_mtime_ns)
            cache = root / "internal_cache.json"
            topics = dict(snapshot=args[1], audit=args[2], imu=args[3])
            cache.write_text(json.dumps(dict(identity=identity,
                                             topics={k: [{"value": r} for r in v] for k, v in topics.items()})))
            horizon_cache = root / "synthetic_ocp_horizon_cache.json.gz"
            with gzip.open(str(horizon_cache), "wt") as stream:
                json.dump(dict(identity=dict(identity, topics=[HORIZON_TOPIC]), topics={"horizon": args[0]}), stream)
            launch = root / "launch.yaml"
            config = {"/spmpc_local_planner/imu_shadow/"+k: v for k, v in asdict(PROCESSING).items()}
            config["/spmpc_local_planner/slosh/slosh_height_ref"] = .01
            launch.write_text(yaml.safe_dump(config))
            source = root / "internal_slosh.json"
            source.write_text(json.dumps(dict(status="PASS", bag=str(bag), cache={"path": str(cache)},
                                             prereg=dict(condition="full", observer="processed_imu"),
                                             windows=args[5], launch_params_file=str(launch),
                                             launch_params_sha256=sha256(launch))))
            output = root / "result"
            with contextlib.redirect_stdout(io.StringIO()):
                code = cli.main(["--report", str(source), "--output", str(output)])
            self.assertEqual(code, 0)
            report = json.loads((output / "report.json").read_text())
            self.assertEqual(report["status"], "DIAGNOSTIC_COMPLETE")
            self.assertFalse(report["method"]["time_shift_fitted"])
            self.assertGreater((output / "samples.csv").stat().st_size, 100)
            self.assertEqual((output / "excitation_comparison.png").read_bytes()[:8], b"\x89PNG\r\n\x1a\n")
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(cli.main(["--report", str(source), "--output", str(output)]), 2)
            launch.write_text("{}")
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(cli.main(["--report", str(source), "--output", str(root / "stale")]), 2)


if __name__ == "__main__":
    unittest.main()
