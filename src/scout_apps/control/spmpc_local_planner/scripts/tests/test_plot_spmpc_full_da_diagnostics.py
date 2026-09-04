#!/usr/bin/env python3
"""Synthetic contract tests for the full-Delta-a offline diagnostic plotter."""

import importlib.util
import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np

MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "analysis"
    / "plot_spmpc_full_da_diagnostics.py"
)
SMOKE_ENGINE = (
    Path(__file__).resolve().parents[1]
    / "run_spmpc_i0_failclosed_explicit_actuator_runtime_smoke.sh"
)
SPEC = importlib.util.spec_from_file_location(
    "plot_spmpc_full_da_diagnostics", MODULE_PATH
)
PLOTTER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = PLOTTER
SPEC.loader.exec_module(PLOTTER)


class FakeStamp:
    def __init__(self, value):
        self.value = value

    def to_sec(self):
        return self.value


def synthetic_data():
    data = PLOTTER.DiagnosticData(bag_path=Path("synthetic.bag"))
    start = 1_000.0
    control_dt = 1.0 / 30.0
    horizon_t = np.arange(61, dtype=float) * control_dt

    previous_a0 = 0.0
    for index in range(75):
        decision = start + index * control_dt
        a0 = 0.22 * math.sin(2.0 * math.pi * 5.0 * index * control_dt)
        if index == 35:
            a0 = 0.65
        elif index == 36:
            a0 = -0.65
        omega = 0.12 * math.sin(0.7 * index * control_dt)
        solver_v = 0.16 + 0.02 * math.sin(0.4 * index * control_dt)
        published_v = 0.0 if index == 50 else solver_v
        audit = PLOTTER.AuditRecord(
            cycle_id=index + 1,
            cycle_start_sec=decision,
            solver_input_sec=decision + 0.001,
            solve_start_sec=decision + 0.002,
            solve_end_sec=decision + 0.009,
            publish_sec=decision + 0.012,
            solve_attempted=True,
            solve_success=index != 45,
            command_accepted=index != 45,
            command_was_published=True,
            status="WAITING_FOR_SLOSH_OBSERVER" if index == 45 else "OK",
            solver_status="ACADOS_FAILED" if index == 45 else "ACADOS_OK",
            a0=a0,
            previous_a1_available=index > 0,
            previous_a1=previous_a0,
            delta_a0=a0 - previous_a0,
            solver_v=solver_v,
            solver_omega=omega,
            post_gate_v=published_v,
            post_gate_omega=omega,
            published_v=published_v,
            published_omega=omega,
            safety_intervened=index == 50,
        )
        data.audits.append(audit)
        data.snapshots.append(
            PLOTTER.SnapshotRecord(
                cycle_id=index + 1,
                solver_input_sec=decision + 0.001,
                valid=True,
                actuator_state_valid=True,
                a_cmd_memory=previous_a0,
                eta_x=0.0002 * math.sin(index * control_dt),
                eta_x_dot=0.0002 * math.cos(index * control_dt),
                eta_y=0.00015 * math.cos(index * control_dt),
                eta_y_dot=-0.00015 * math.sin(index * control_dt),
            )
        )
        phase = index * control_dt + horizon_t
        eta_x = 0.00025 * np.sin(2.0 * math.pi * 1.2 * phase)
        eta_y = 0.00018 * np.cos(2.0 * math.pi * 1.2 * phase)
        data.horizons.append(
            PLOTTER.HorizonRecord(
                cycle_id=index + 1,
                solver_input_sec=decision + 0.001,
                available_sec=decision + 0.010,
                valid=True,
                dt=control_dt,
                t=horizon_t.copy(),
                fields={
                    "v": solver_v + 0.01 * np.sin(0.5 * horizon_t),
                    "omega": omega + 0.02 * np.sin(0.8 * horizon_t),
                    "eta_x": eta_x,
                    "eta_x_dot": np.gradient(eta_x, control_dt),
                    "eta_y": eta_y,
                    "eta_y_dot": np.gradient(eta_y, control_dt),
                    "h_modal": np.hypot(eta_x, eta_y),
                    "a_cmd_memory": np.full(61, previous_a0),
                    "a_actual": np.full(61, a0),
                    "alpha_actual": np.zeros(61),
                    "a": np.full(60, a0),
                },
            )
        )
        values = {
            "total": 1.0 + 0.1 * math.sin(index * control_dt),
            "J_contour": 0.15,
            "J_lag": 0.08,
            "J_progress": -0.25,
            "J_v": 0.12,
            "J_control": 0.08 + 0.02 * abs(a0),
            "J_smooth": 0.05,
            "J_terminal": 0.04,
            "J_corridor": 0.02,
            "J_obstacle": 0.01,
            "J_slosh_eta": 0.12,
            "J_slosh_eta_dot": 0.08,
        }
        data.costs.append(PLOTTER.CostRecord(index + 1, decision + 0.001, values))
        data.raw_commands.append(
            PLOTTER.RawCommandRecord(index + 1, decision + 0.012, published_v, omega)
        )
        previous_a0 = a0

    for index in range(140):
        stamp = start + index * 0.02
        data.odom.append(
            PLOTTER.OdomRecord(
                stamp,
                0.16 + 0.018 * math.sin(0.4 * (stamp - start)),
                0.12 * math.sin(0.7 * (stamp - start)),
            )
        )
        data.imu.append(
            PLOTTER.ImuRecord(
                stamp,
                0.2 * math.sin(1.4 * (stamp - start)),
                0.1 * math.cos(1.4 * (stamp - start)),
            )
        )
        data.observers.append(
            PLOTTER.ObserverRecord(
                stamp_sec=stamp,
                valid=True,
                source=2,
                input_status="READY",
                ax=0.2 * math.sin(1.4 * (stamp - start)),
                ay=0.1 * math.cos(1.4 * (stamp - start)),
                eta_x=0.00025 * math.sin(2.0 * math.pi * 1.2 * (stamp - start)),
                eta_x_dot=0.00025
                * 2.0
                * math.pi
                * 1.2
                * math.cos(2.0 * math.pi * 1.2 * (stamp - start)),
                eta_y=0.00018 * math.cos(2.0 * math.pi * 1.2 * (stamp - start)),
                eta_y_dot=-0.00018
                * 2.0
                * math.pi
                * 1.2
                * math.sin(2.0 * math.pi * 1.2 * (stamp - start)),
                modal_height_m=0.0003
                + 0.00008 * math.sin(2.0 * math.pi * 1.2 * (stamp - start)),
            )
        )
        if index % 5 == 0:
            data.rgb.append(
                PLOTTER.RgbRecord(
                    stamp,
                    0.31 + 0.07 * math.sin(2.0 * math.pi * 1.2 * (stamp - start)),
                )
            )
    return data


class NumericalContractTest(unittest.TestCase):
    def test_bounded_interpolation_never_crosses_a_large_sensor_gap(self):
        self.assertAlmostEqual(
            PLOTTER.bounded_interpolate(0.5, [0.0, 1.0], [2.0, 4.0], 1.0),
            3.0,
        )
        self.assertTrue(
            math.isnan(PLOTTER.bounded_interpolate(0.5, [0.0, 1.0], [2.0, 4.0], 0.99))
        )

    def test_horizon_interpolation_and_frequency_peak(self):
        record = PLOTTER.HorizonRecord(
            cycle_id=1,
            solver_input_sec=1.0,
            available_sec=1.1,
            valid=True,
            dt=0.1,
            t=np.asarray([0.0, 0.1, 0.2]),
            fields={"v": np.asarray([0.0, 1.0, 2.0])},
        )
        self.assertAlmostEqual(PLOTTER.sample_horizon(record, "v", 0.15), 1.5)

        times = np.arange(300, dtype=float) / 30.0
        values = np.sin(2.0 * math.pi * 5.0 * times)
        frequencies, amplitudes = PLOTTER.compute_spectrum(times, values)
        peak = int(np.argmax(amplitudes[1:]) + 1)
        self.assertAlmostEqual(frequencies[peak], 5.0, delta=0.11)

    def test_event_markers_cover_all_required_classes(self):
        events = PLOTTER.build_event_markers(synthetic_data().audits)
        self.assertEqual(
            {event.kind for event in events},
            {"strong_flip", "fail_closed", "solver_failure", "intervention"},
        )

    def test_headerless_values_receive_cycle_effective_time_not_bag_time(self):
        horizon = SimpleNamespace(
            cycle_id=17,
            solver_input_epoch=FakeStamp(101.0),
            cycle_start_stamp=FakeStamp(100.9),
            horizon_available_stamp=FakeStamp(101.01),
            valid=True,
            dt=0.1,
            t=[0.0, 0.1],
            v=[0.1, 0.2],
            omega=[0.0, 0.1],
        )
        cost = SimpleNamespace(
            layout=SimpleNamespace(dim=[SimpleNamespace(label="total,J_control")]),
            data=[2.0, 0.4],
        )
        audit = SimpleNamespace(
            cycle_id=17,
            cycle_start_stamp=FakeStamp(100.9),
            solver_input_epoch=FakeStamp(101.0),
            solve_start_stamp=FakeStamp(101.001),
            solve_end_stamp=FakeStamp(101.008),
            command_publish_stamp=FakeStamp(101.012),
            solve_attempted=True,
            solve_success=True,
            command_accepted=True,
            command_was_published=True,
            status="OK",
            solver_status="ACADOS_OK",
            solver_u0_a=0.1,
            solver_cmd_v=0.2,
            solver_cmd_omega=0.0,
            post_gate_cmd_v=0.2,
            post_gate_cmd_omega=0.0,
            published_cmd_v=0.2,
            published_cmd_omega=0.0,
        )
        command = SimpleNamespace(
            linear=SimpleNamespace(x=0.2), angular=SimpleNamespace(z=0.0)
        )
        odom = SimpleNamespace(
            header=SimpleNamespace(stamp=FakeStamp(100.95)),
            twist=SimpleNamespace(
                twist=SimpleNamespace(
                    linear=SimpleNamespace(x=0.18),
                    angular=SimpleNamespace(z=0.01),
                )
            ),
        )
        messages = [
            (PLOTTER.HORIZON_TOPIC, horizon, FakeStamp(9_001.0)),
            (PLOTTER.COST_TOPIC, cost, FakeStamp(9_002.0)),
            (PLOTTER.AUDIT_TOPIC, audit, FakeStamp(9_003.0)),
            (PLOTTER.CMD_TOPIC, command, FakeStamp(9_004.0)),
            (PLOTTER.ODOM_TOPIC, odom, FakeStamp(9_005.0)),
        ]

        class FakeBag:
            def __init__(self, *_args, **_kwargs):
                pass

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def read_messages(self, topics):
                selected = set(topics)
                return (row for row in messages if row[0] in selected)

        fake_rosbag = SimpleNamespace(Bag=FakeBag)
        with mock.patch.dict(sys.modules, {"rosbag": fake_rosbag}):
            loaded = PLOTTER.load_bag(Path("fake.bag"))

        self.assertEqual(loaded.costs[0].cycle_id, 17)
        self.assertEqual(loaded.costs[0].stamp_sec, 101.0)
        self.assertEqual(loaded.raw_commands[0].cycle_id, 17)
        self.assertEqual(loaded.raw_commands[0].stamp_sec, 101.012)
        self.assertEqual(loaded.odom[0].stamp_sec, 100.95)
        for effective in (
            loaded.costs[0].stamp_sec,
            loaded.raw_commands[0].stamp_sec,
            loaded.odom[0].stamp_sec,
        ):
            self.assertLess(effective, 1_000.0)


class RenderContractTest(unittest.TestCase):
    def test_synthetic_run_writes_exactly_six_nonempty_pngs(self):
        data = synthetic_data()
        with tempfile.TemporaryDirectory() as raw_directory:
            output_dir = Path(raw_directory)
            paths = PLOTTER.render_diagnostics(data, output_dir, dpi=80)
            self.assertEqual([path.name for path in paths], list(PLOTTER.FIGURE_NAMES))
            self.assertEqual(
                sorted(path.name for path in output_dir.iterdir()),
                sorted(PLOTTER.FIGURE_NAMES),
            )
            for path in paths:
                with self.subTest(path=path.name):
                    self.assertGreater(path.stat().st_size, 2_000)
                    image = PLOTTER.plt.imread(path)
                    self.assertGreater(image.shape[0], 400)
                    self.assertGreater(image.shape[1], 700)
                    self.assertGreater(float(np.std(image)), 0.01)


class SmokeIntegrationContractTest(unittest.TestCase):
    def test_plotting_precedes_postflight_and_all_return_codes_are_aggregated(self):
        script = SMOKE_ENGINE.read_text(encoding="utf-8")
        self.assertLess(
            script.index('python3 "${FULL_DA_PLOTTER}"'),
            script.index('python3 "${EXACT_POSTFLIGHT}" "${BAG_PATH}"'),
        )
        for token in (
            'bash "${RUNNER}" || runner_rc=$?',
            '"${BAG_PATH}" "${BAG_PATH}.active" "${EXACT_REPORT}"',
            '"${DIAGNOSTIC_PLOT_DIR}"',
            "runner_rc != 0",
            "bag_rc != 0",
            "plot_rc != 0",
            "exact_rc != 0",
            "runtime_rc != 0",
            "preserve the bag, plots, and reports",
        ):
            self.assertIn(token, script)


if __name__ == "__main__":
    unittest.main()
