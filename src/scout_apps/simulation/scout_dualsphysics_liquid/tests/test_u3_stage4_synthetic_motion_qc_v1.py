#!/usr/bin/env python3
"""Static, unit, and fail-closed tests for synthetic-motion QC v1."""

from __future__ import annotations

import copy
import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from jsonschema import Draft202012Validator


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))
import r8_liquid_u3_stage4_synthetic_motion_qc_v1 as qc  # noqa: E402


def fit_frames(amplitude: float, *, yaw: bool = False) -> list[dict[str, float]]:
    name = "yaw_solver_deg" if yaw else "translation_x_m"
    return [
        {
            "relative_time_s": index * 0.05,
            "boundary_observed": amplitude * math.sin(2 * math.pi * index * 0.05),
            "boundary_signal_name": name,
        }
        for index in range(41)
    ]


def adjudication_fixture() -> tuple[dict[str, object], dict[str, object]]:
    runs = {}
    for name in ("zero", "translation", "yaw"):
        runs[name] = {
            "pass": True,
            "boundary": {
                "zero_displacement_max_m": 0.0,
                "position_error_max_m": 0.0,
                "yaw_angle_error_max_deg": 0.0,
            },
            "time_alignment_max_abs_s": 0.0,
            "surface": {"minimum_valid_fraction_observed": 1.0},
            "frequency_fit": {"frequency_abs_error_hz": 0.0},
        }
    comparisons = {
        "zero_stability": {"pass": True},
        "translation_response": {
            "dynamic_over_zero_ratio": 1.1,
            "peak_surface_first_harmonic_m": 0.00001,
            "pass": True,
        },
        "yaw_response": {"dynamic_over_zero_ratio": 1.1, "pass": True},
        "free_decay": {
            "translation": {"final_to_initial_peak_ratio": 0.95},
            "yaw": {"final_to_initial_peak_ratio": 0.95},
            "pass": True,
        },
        "input_reconstruction": {"pass": True},
    }
    return runs, comparisons


def csv_frame() -> dict[str, object]:
    surface = {
        "crest_above_nominal_m": 0.001,
        "absolute_deviation_max_m": 0.001,
        "peak_to_peak_m": 0.0002,
        "peak_height_m": 0.059,
        "p95_height_m": 0.0589,
        "rms_deviation_m": 0.0005,
        "first_azimuthal_harmonic_m": 0.0001,
        "first_azimuthal_harmonic_phase_rad": 0.0,
        "sector_heights_m": [0.058 + index * 1e-6 for index in range(16)],
    }
    return {
        "part": 901,
        "time_s": qc.START_TIME_S,
        "relative_time_s": 0.0,
        "boundary_signal_name": "zero",
        "boundary_observed": 0.0,
        "boundary_expected": 0.0,
        "boundary_position_error_max_m": 0.0,
        "fluid_speed_rms_lab_m_s": 0.001,
        "fluid_speed_rms_container_m_s": 0.001,
        "fluid_tangential_speed_rms_container_m_s": 0.0001,
        "specific_kinetic_energy_lab_j_kg": 5e-7,
        "specific_kinetic_energy_container_j_kg": 5e-7,
        "surface": surface,
    }


class SyntheticMotionQcV1Tests(unittest.TestCase):
    def test_schema_is_valid_and_every_object_is_closed(self) -> None:
        schema = json.loads(qc.SCHEMA_PATH.read_text(encoding="utf-8"))
        Draft202012Validator.check_schema(schema)
        self.assertIs(schema["additionalProperties"], False)
        for name, definition in schema["$defs"].items():
            if definition.get("type") == "object":
                self.assertIs(definition.get("additionalProperties"), False, name)

    def test_frozen_thresholds_are_exact_in_schema(self) -> None:
        schema = json.loads(qc.SCHEMA_PATH.read_text(encoding="utf-8"))
        props = schema["$defs"]["acceptance"]["properties"]
        self.assertEqual(props["dynamic_fluid_speed_rms_over_zero_ratio_min"]["const"], 1.1)
        self.assertEqual(props["free_decay_final_to_initial_peak_ke_ratio_max"]["const"], 0.95)
        self.assertEqual(props["translation_surface_first_harmonic_min_m"]["const"], 1e-5)
        self.assertEqual(schema["$defs"]["surfaceProxy"]["properties"]["sector_count"]["const"], 16)
        self.assertEqual(schema["$defs"]["surfaceProxy"]["properties"]["minimum_particles_per_sector"]["const"], 128)
        self.assertFalse(schema["$defs"]["contract"]["properties"]["threshold_overrides_allowed"]["const"])

    def test_expected_motion_is_two_cycles_then_zero(self) -> None:
        translation = qc.RUNS["translation"]
        self.assertAlmostEqual(qc.expected_signal(translation, 0.25), 0.002)
        self.assertAlmostEqual(qc.expected_signal(translation, 0.75), -0.002)
        self.assertAlmostEqual(qc.expected_signal(translation, 1.25), 0.002)
        self.assertAlmostEqual(qc.expected_signal(translation, 2.0), 0.0, places=14)
        self.assertEqual(qc.expected_signal(translation, 2.01), 0.0)
        self.assertAlmostEqual(qc.expected_signal(qc.RUNS["yaw"], 0.25), 2.0)
        self.assertEqual(qc.expected_signal(qc.RUNS["zero"], 0.25), 0.0)

    def test_quantile_is_linear_and_rejects_invalid_data(self) -> None:
        self.assertEqual(qc.quantile([0.0, 10.0], 0.25), 2.5)
        self.assertEqual(qc.quantile([3.0, 1.0, 2.0], 0.5), 2.0)
        for values, probability in (([], 0.5), ([1.0], -0.1), ([float("nan")], 0.5)):
            with self.assertRaises(qc.SyntheticMotionQcError):
                qc.quantile(values, probability)

    def test_frequency_fit_recovers_translation_and_solver_signed_yaw(self) -> None:
        translation = qc.frequency_fit(fit_frames(0.002), qc.RUNS["translation"])
        yaw = qc.frequency_fit(fit_frames(2.0, yaw=True), qc.RUNS["yaw"])
        for result, amplitude in ((translation, 0.002), (yaw, 2.0)):
            self.assertTrue(result["pass"])
            self.assertAlmostEqual(result["observed_frequency_hz"], 1.0, places=6)
            self.assertAlmostEqual(result["observed_amplitude"], amplitude, places=9)
            self.assertLess(result["phase_abs_error_rad"], 1e-9)
            self.assertGreater(result["fit_r_squared"], 0.999999)

    def test_adjudication_accepts_values_at_all_frozen_limits(self) -> None:
        runs, comparisons = adjudication_fixture()
        self.assertTrue(qc.adjudicate(runs, comparisons))

    def test_each_core_threshold_exceedance_fails_closed(self) -> None:
        mutations = [
            ("zero displacement", lambda r, c: r["zero"]["boundary"].__setitem__("zero_displacement_max_m", 1.0001e-10)),
            ("translation position", lambda r, c: r["translation"]["boundary"].__setitem__("position_error_max_m", 0.000050001)),
            ("yaw angle", lambda r, c: r["yaw"]["boundary"].__setitem__("yaw_angle_error_max_deg", 0.050001)),
            ("time", lambda r, c: r["yaw"].__setitem__("time_alignment_max_abs_s", 0.00010001)),
            ("sector validity", lambda r, c: r["translation"]["surface"].__setitem__("minimum_valid_fraction_observed", 0.949)),
            ("frequency", lambda r, c: r["yaw"]["frequency_fit"].__setitem__("frequency_abs_error_hz", 0.050001)),
            ("translation ratio", lambda r, c: c["translation_response"].__setitem__("dynamic_over_zero_ratio", 1.0999)),
            ("surface harmonic", lambda r, c: c["translation_response"].__setitem__("peak_surface_first_harmonic_m", 0.000009999)),
            ("yaw ratio", lambda r, c: c["yaw_response"].__setitem__("dynamic_over_zero_ratio", 1.0999)),
            ("translation decay", lambda r, c: c["free_decay"]["translation"].__setitem__("final_to_initial_peak_ratio", 0.950001)),
            ("yaw decay", lambda r, c: c["free_decay"]["yaw"].__setitem__("final_to_initial_peak_ratio", 0.950001)),
        ]
        for name, mutation in mutations:
            with self.subTest(name=name):
                runs, comparisons = adjudication_fixture()
                mutation(runs, comparisons)
                self.assertFalse(qc.adjudicate(runs, comparisons))

    def test_yaw_does_not_require_surface_harmonic(self) -> None:
        runs, comparisons = adjudication_fixture()
        comparisons["yaw_response"].update({
            "peak_surface_first_harmonic_m": 0.0,
            "surface_harmonic_required": False,
            "surface_harmonic_pass": True,
        })
        self.assertTrue(qc.adjudicate(runs, comparisons))

    def test_csv_series_is_deterministic_and_has_16_sector_columns(self) -> None:
        runs = {name: {"frames": [csv_frame()]} for name in ("zero", "translation", "yaw")}
        first = qc.series_csv_bytes(runs)
        second = qc.series_csv_bytes(runs)
        self.assertEqual(first, second)
        header = first.decode("utf-8").splitlines()[0].split(",")
        self.assertEqual(sum(name.startswith("surface_sector_") for name in header), 16)
        self.assertEqual(len(first.decode("utf-8").splitlines()), 4)

    def test_exclusive_writer_handles_short_writes_and_rejects_reuse(self) -> None:
        data = b"frozen-evidence\n" * 20
        original_write = os.write

        def short_write(descriptor: int, source: bytes | memoryview) -> int:
            return original_write(descriptor, bytes(source[:7]))

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "receipt.json"
            with mock.patch.object(qc.os, "write", side_effect=short_write):
                identity = qc.write_exclusive(path, data, allowed={path})
                with self.assertRaises(FileExistsError):
                    qc.write_exclusive(path, data, allowed={path})
            self.assertEqual(path.read_bytes(), data)
            self.assertEqual(identity["mode"], "0640")

    def test_writer_rejects_an_unfrozen_path(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            allowed = Path(directory) / "allowed"
            other = Path(directory) / "other"
            with self.assertRaises(qc.SyntheticMotionQcError):
                qc.write_exclusive(other, b"x", allowed={allowed})
            self.assertFalse(other.exists())

    def test_static_self_check_is_read_only_and_complete(self) -> None:
        value = qc.static_self_check()
        self.assertEqual(value["run_count"], 3)
        self.assertEqual(value["frame_count"], 143)
        self.assertEqual(value["surface_sector_count"], 16)
        self.assertEqual(value["minimum_surface_particles_per_sector"], 128)
        self.assertFalse(value["threshold_overrides_allowed"])
        self.assertFalse(value["solver_executed"])
        self.assertFalse(value["gpu_exposed"])
        self.assertFalse(value["network_used"])

    def test_report_finite_guard_rejects_nan_and_infinity(self) -> None:
        for value in (float("nan"), float("inf"), -float("inf")):
            self.assertFalse(qc._all_finite({"nested": [0.0, value]}))
        self.assertTrue(qc._all_finite({"nested": [0.0, 1.0]}))


if __name__ == "__main__":
    unittest.main()
