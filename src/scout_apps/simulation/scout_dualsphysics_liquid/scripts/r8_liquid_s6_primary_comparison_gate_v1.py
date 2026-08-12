#!/usr/bin/env python3
"""Static-only S6 primary replay reader, comparator, and delivery planner.

Only synthetic in-memory fixtures are admitted in v1.  The frozen real S5B0
parent is NOT_MATERIALIZED, so this module cannot publish a final figure,
encode media, append a ledger, or claim a Stage-6 PASS.
"""

from __future__ import annotations

import argparse
import bisect
import hashlib
import io
import json
import math
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s6_primary_static_analysis_v1.json"
POLICY_SCHEMA_PATH = ROOT / "schema/target_host_s6_primary_static_policy_v1.json"
RESULT_SCHEMA_PATH = ROOT / "schema/target_host_s6_primary_comparison_result_v1.json"
MAXIMUM_REPLAY_MANIFEST_BYTES = 4 * 1024 * 1024
EXPECTED_REPLAY_KEYS = {
    "schema_version", "document_type", "status", "finalized", "synthetic_fixture",
    "attempt_id", "planned_denominator", "time_origin_ns", "numeric_sources",
    "series", "claims",
}


class S6ComparisonError(ValueError):
    """Closed-contract, alignment, metric, claim, or parent failure."""


def canonical_json(value: Any) -> bytes:
    return (json.dumps(value, allow_nan=False, ensure_ascii=False, sort_keys=True,
                       separators=(",", ":")) + "\n").encode("utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise S6ComparisonError(f"JSON root is not an object: {path}")
    return value


def assert_deep_closed(value: Any, location: str = "$") -> None:
    if isinstance(value, dict):
        if value.get("type") == "object" and value.get("additionalProperties") is not False:
            raise S6ComparisonError(f"schema object is open at {location}")
        for key, child in value.items():
            assert_deep_closed(child, f"{location}/{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            assert_deep_closed(child, f"{location}/{index}")


def load_contracts() -> tuple[dict[str, Any], dict[str, Any]]:
    policy = _read_json(POLICY_PATH)
    policy_schema = _read_json(POLICY_SCHEMA_PATH)
    result_schema = _read_json(RESULT_SCHEMA_PATH)
    for schema in (policy_schema, result_schema):
        Draft202012Validator.check_schema(schema)
        assert_deep_closed(schema)
    Draft202012Validator(policy_schema).validate(policy)
    if policy["parents"]["s5b0_finalized_replay_manifest"]["state"] != "NOT_MATERIALIZED":
        raise S6ComparisonError("real replay parent unexpectedly materialized")
    return policy, result_schema


def read_finalized_replay_bytes(raw: bytes, *, synthetic_fixture: bool) -> dict[str, Any]:
    if not isinstance(raw, bytes) or not 0 < len(raw) <= MAXIMUM_REPLAY_MANIFEST_BYTES:
        raise S6ComparisonError("replay manifest bytes exceed the bound")
    try:
        value = json.loads(raw, parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
    except (json.JSONDecodeError, ValueError) as exc:
        raise S6ComparisonError("replay manifest JSON is invalid or non-finite") from exc
    if not isinstance(value, dict) or set(value) != EXPECTED_REPLAY_KEYS:
        raise S6ComparisonError("replay manifest is not closed")
    if not synthetic_fixture or value["synthetic_fixture"] is not True:
        raise S6ComparisonError("REAL_REPLAY_NOT_ADMITTED_PARENT_NOT_MATERIALIZED")
    if value["schema_version"] != "smpcc-r8-liquid-s5b0-finalized-replay-fixture-v1":
        raise S6ComparisonError("synthetic replay schema differs")
    if value["status"] != "S5B0_PRIMARY_R7_EXECUTED_MOTION_REPLAY_PASS_DEVELOPMENT_ONLY" or value["finalized"] is not True:
        raise S6ComparisonError("replay is not finalized with the frozen status")
    if value["attempt_id"] != "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01" or value["planned_denominator"] != 1:
        raise S6ComparisonError("replay row identity or denominator differs")
    if value["numeric_sources"] != ["finalized_bi4", "gauge_zsurf_csv", "slosh_height_csv"]:
        raise S6ComparisonError("replay numeric sources differ")
    if value["claims"] != {"cpu_selected_trajectory_comparison": False, "paired_ranking": False,
                            "physical_fidelity_validated": False, "physical_reference_pending": True}:
        raise S6ComparisonError("replay claims exceed the ceiling")
    rows = value["series"]
    if not isinstance(rows, list) or len(rows) < 4:
        raise S6ComparisonError("replay series is too short")
    expected = {"t_s", "H_crest_mm", "H_abs_mm", "H_peak_to_peak_mm"}
    previous: float | None = None
    for row in rows:
        if not isinstance(row, dict) or set(row) != expected:
            raise S6ComparisonError("replay series row is not closed")
        values = [float(row[name]) for name in expected]
        if any(not math.isfinite(item) for item in values):
            raise S6ComparisonError("replay series contains non-finite values")
        stamp = float(row["t_s"])
        if previous is not None and stamp <= previous:
            raise S6ComparisonError("replay time is not strictly increasing")
        if float(row["H_abs_mm"]) < 0 or float(row["H_peak_to_peak_mm"]) < 0:
            raise S6ComparisonError("absolute/peak-to-peak metric is negative")
        previous = stamp
    return value


def profile_replay(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    times = [float(row["t_s"]) for row in rows]
    return {"row_count": len(rows), "missing_values": 0, "time_min_s": min(times),
            "time_max_s": max(times), "strict_time": True}


def _uniform_dt(times: Sequence[float]) -> float:
    deltas = [right - left for left, right in zip(times, times[1:])]
    dt = sum(deltas) / len(deltas)
    if dt <= 0 or max(abs(delta - dt) for delta in deltas) > max(1e-9, dt * 1e-6):
        raise S6ComparisonError("metric grid is not uniformly spaced")
    return dt


def series_metrics(times: Sequence[float], values: Sequence[float]) -> dict[str, float]:
    if len(times) != len(values) or len(times) < 4:
        raise S6ComparisonError("metric series length differs")
    dt = _uniform_dt(times)
    mean = sum(values) / len(values)
    centered = [value - mean for value in values]
    best_k, best_real, best_imag, best_power = 0, 0.0, 0.0, -1.0
    for k in range(1, len(values) // 2 + 1):
        real = sum(value * math.cos(2 * math.pi * k * index / len(values)) for index, value in enumerate(centered))
        imag = -sum(value * math.sin(2 * math.pi * k * index / len(values)) for index, value in enumerate(centered))
        power = real * real + imag * imag
        if power > best_power:
            best_k, best_real, best_imag, best_power = k, real, imag, power
    peaks = [(times[index], abs(values[index])) for index in range(1, len(values) - 1)
             if abs(values[index]) >= abs(values[index - 1]) and abs(values[index]) > abs(values[index + 1]) and abs(values[index]) > 0]
    damping = 0.0
    if len(peaks) >= 2:
        xs, ys = [row[0] for row in peaks], [math.log(row[1]) for row in peaks]
        xmean, ymean = sum(xs) / len(xs), sum(ys) / len(ys)
        denominator = sum((x - xmean) ** 2 for x in xs)
        damping = -sum((x - xmean) * (y - ymean) for x, y in zip(xs, ys)) / denominator if denominator else 0.0
    return {
        "amplitude_peak_mm": max(abs(value) for value in values),
        "dominant_frequency_hz": best_k / (len(values) * dt),
        "damping_rate_per_s": damping,
        "phase_rad": math.atan2(best_imag, best_real),
    }


def _interpolate(times: Sequence[float], values: Sequence[float], query: float) -> float:
    if query < times[0] or query > times[-1]:
        raise S6ComparisonError("query requires forbidden extrapolation")
    index = bisect.bisect_left(times, query)
    if index < len(times) and abs(times[index] - query) <= 1e-12:
        return values[index]
    left, right = index - 1, index
    fraction = (query - times[left]) / (times[right] - times[left])
    return values[left] + fraction * (values[right] - values[left])


def _wrap_phase(value: float) -> float:
    return (value + math.pi) % (2 * math.pi) - math.pi


def compare(replay: Mapping[str, Any], selected: Mapping[str, Any]) -> tuple[dict[str, Any], list[dict[str, float]]]:
    if selected.get("planned_denominator") != 1 or selected.get("claims") != {
        "stage6_pass": False, "physical_reference_pending": True,
        "physical_fidelity_validated": False, "paired_ranking": False,
        "cpu_selected_trajectory_comparison": False,
    }:
        raise S6ComparisonError("selected signal claims or denominator differ")
    replay_rows = replay["series"]
    grid = [float(row["t_s"]) for row in replay_rows]
    origin = int(replay["time_origin_ns"])
    aligned: list[dict[str, float]] = []
    model_values: dict[str, list[float]] = {}
    for name in ("H_proxy", "H_modal"):
        samples = selected["series"][name]["samples"]
        times = [(int(row["bag_record_t_ns"]) - origin) / 1e9 for row in samples]
        values = [float(row["value_comparison_mm"]) for row in samples]
        model_values[name] = [_interpolate(times, values, query) for query in grid]
    for index, row in enumerate(replay_rows):
        aligned.append({"t_s": grid[index], "H_crest_mm": float(row["H_crest_mm"]),
                        "H_abs_mm": float(row["H_abs_mm"]), "H_peak_to_peak_mm": float(row["H_peak_to_peak_mm"]),
                        "H_proxy_mm": model_values["H_proxy"][index], "H_modal_mm": model_values["H_modal"][index]})
    crest = [row["H_crest_mm"] for row in aligned]
    crest_metrics = series_metrics(grid, crest)
    surface = {
        "H_crest": crest_metrics,
        "H_abs": series_metrics(grid, [row["H_abs_mm"] for row in aligned]),
        "H_peak_to_peak": series_metrics(grid, [row["H_peak_to_peak_mm"] for row in aligned]),
    }
    comparisons: dict[str, Any] = {}
    for name in ("H_proxy", "H_modal"):
        values = model_values[name]
        metrics = series_metrics(grid, values)
        xmean, ymean = sum(crest) / len(crest), sum(values) / len(values)
        covariance = sum((x - xmean) * (y - ymean) for x, y in zip(crest, values))
        xvar = sum((x - xmean) ** 2 for x in crest)
        yvar = sum((y - ymean) ** 2 for y in values)
        correlation = covariance / math.sqrt(xvar * yvar) if xvar and yvar else 0.0
        comparisons[name] = {
            "sample_count": len(grid),
            "amplitude_error_mm": metrics["amplitude_peak_mm"] - crest_metrics["amplitude_peak_mm"],
            "frequency_error_hz": metrics["dominant_frequency_hz"] - crest_metrics["dominant_frequency_hz"],
            "damping_error_per_s": metrics["damping_rate_per_s"] - crest_metrics["damping_rate_per_s"],
            "phase_error_rad": _wrap_phase(metrics["phase_rad"] - crest_metrics["phase_rad"]),
            "rmse_mm": math.sqrt(sum((x - y) ** 2 for x, y in zip(crest, values)) / len(grid)),
            "correlation": max(-1.0, min(1.0, correlation)),
        }
    return {"surface": surface, "comparisons": comparisons}, aligned


def build_result(policy: Mapping[str, Any], replay_raw: bytes, replay: Mapping[str, Any], selected: Mapping[str, Any]) -> tuple[dict[str, Any], list[dict[str, float]]]:
    metrics, aligned = compare(replay, selected)
    result = {
        "schema_version": "smpcc-r8-liquid-s6-primary-comparison-result-v1",
        "document_type": "SMPCC_R8_LIQUID_S6_PRIMARY_COMPARISON_RESULT_V1",
        "status": "S6_PRIMARY_SYNTHETIC_STATIC_COMPARISON_VALIDATED_NOT_FINAL",
        "attempt_id": policy["selection"]["attempt_id"], "planned_denominator": 1,
        "parents": {
            "selected_signals": {"kind": "SYNTHETIC_ROS1_BAG_V2_BYTES", "sha256": selected["source"]["sha256"], "real_parent_materialized": False},
            "finalized_replay": {"kind": "SYNTHETIC_FINALIZED_REPLAY_FIXTURE", "sha256": sha256_bytes(replay_raw), "real_parent_materialized": False},
        },
        "profile": profile_replay(replay["series"]),
        "surface_metrics": metrics["surface"], "model_comparisons": metrics["comparisons"],
        "alignment": {"x_axis": policy["alignment"]["x_axis"],
                      "grid_source": policy["alignment"]["grid_source"],
                      "interpolation": policy["alignment"]["interpolation"],
                      "extrapolation": policy["alignment"]["extrapolation"],
                      "smoothing": policy["alignment"]["smoothing"],
                      "sample_count": len(aligned)},
        "visualization": {"layout": "THREE_VERTICAL_SHARED_X_PANELS", "shared_x": True, "axes_count": 3,
                          "dual_y_axes": False, "palette": "OKABE_ITO_COLORBLIND_SAFE", "redundant_line_styles": True,
                          "rainbow_colormap": False, "grayscale_review": True,
                          "layout_audit": "REQUIRED_BEFORE_MATERIALIZATION", "final_figure_generated": False},
        "delivery": {"write_semantics": "CREATE_NEW_ONLY", "artifacts_materialized": False,
                     "planned_inventory": policy["delivery"]["planned_inventory"],
                     "animation_interface": {"source": "FINALIZED_SOLVER_FRAMES_ONLY", "mp4": "animation/primary.mp4",
                                             "gif": "animation/primary_preview.gif", "keyframes": ["keyframes/primary_t000.png"],
                                             "encoder_executed": False}},
        "evidence_index": {"status": "PLANNED_NOT_MATERIALIZED", "entries": [], "checksums_path": "checksums.sha256"},
        "secondary_ledger": {"status": "PLANNED_NOT_APPENDED", "path": "secondary_ledger_entry.json", "append_performed": False},
        "claims": {"stage6_pass": False, "paired_ranking": False, "cpu_selected_trajectory_comparison": False,
                   "physical_reference_pending": True, "physical_fidelity_validated": False, "formal": False, "production": False},
    }
    return result, aligned


def build_planned_bundle(result: Mapping[str, Any]) -> dict[str, bytes]:
    manifest = canonical_json(result)
    evidence = canonical_json(result["evidence_index"])
    ledger = canonical_json(result["secondary_ledger"])
    files = {"comparison_manifest.json": manifest, "evidence_index.json": evidence,
             "secondary_ledger_entry.json": ledger}
    files["checksums.sha256"] = "".join(f"{sha256_bytes(files[name])}  {name}\n" for name in sorted(files)).encode("ascii")
    return files


def render_synthetic_preview(aligned: Sequence[Mapping[str, float]], output: Path) -> dict[str, Any]:
    if output.exists() or "scout_liquid_lab" in output.parts:
        raise S6ComparisonError("preview target is not a fresh test-only path")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {"blue": "#0072B2", "orange": "#E69F00", "green": "#009E73", "purple": "#CC79A7", "black": "#000000"}
    times = [row["t_s"] for row in aligned]
    fig, axes = plt.subplots(3, 1, figsize=(7.2, 6.6), sharex=True, constrained_layout=True)
    plt.rcParams.update({"font.size": 8, "pdf.fonttype": 42, "ps.fonttype": 42})
    axes[0].plot(times, [row["H_crest_mm"] for row in aligned], color=colors["blue"], linestyle="-", label="H_crest")
    axes[0].plot(times, [row["H_abs_mm"] for row in aligned], color=colors["orange"], linestyle="--", label="H_abs")
    axes[0].plot(times, [row["H_peak_to_peak_mm"] for row in aligned], color=colors["green"], linestyle=":", label="H_peak_to_peak")
    axes[1].plot(times, [row["H_proxy_mm"] for row in aligned], color=colors["blue"], linestyle="-", label="H_proxy")
    axes[1].plot(times, [row["H_modal_mm"] for row in aligned], color=colors["purple"], linestyle="--", label="H_modal")
    axes[2].plot(times, [row["H_crest_mm"] - row["H_proxy_mm"] for row in aligned], color=colors["black"], linestyle="-", label="H_crest - H_proxy")
    axes[2].plot(times, [row["H_crest_mm"] - row["H_modal_mm"] for row in aligned], color=colors["orange"], linestyle="--", label="H_crest - H_modal")
    for index, axis in enumerate(axes):
        axis.set_ylabel("Height (mm)" if index < 2 else "Residual (mm)")
        axis.legend(frameon=False, ncol=3 if index == 0 else 2, loc="upper right")
        axis.grid(True, color="#d9d9d9", linewidth=0.45, alpha=0.65)
        axis.spines[["top", "right"]].set_visible(False)
    axes[-1].set_xlabel("Time since odom header origin (s)")
    axes[0].set_title("Synthetic fixture only — not a finalized S6 result")
    fig.canvas.draw()
    shared = all(axes[0].get_shared_x_axes().joined(axes[0], axis) for axis in axes[1:])
    if len(fig.axes) != 3 or not shared:
        plt.close(fig)
        raise S6ComparisonError("layout audit failed")
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=300)
    plt.close(fig)
    return {"path": str(output), "sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
            "axes_count": 3, "shared_x": True, "dual_y_axes": False, "layout_audit": "PASS_SYNTHETIC_PREVIEW"}


def self_check() -> dict[str, Any]:
    policy, result_schema = load_contracts()
    if policy["selection"]["planned_denominator"] != 1 or policy["delivery"]["final_outputs_allowed"]:
        raise S6ComparisonError("policy admission ceiling differs")
    return {"status": "S6_STATIC_SELF_CHECK_OK_NOT_ADMITTED", "planned_denominator": 1,
            "result_schema_deep_closed": True, "real_bag_read": False, "external_replay_read": False,
            "solver_or_gpu_executed": False, "final_outputs_generated": False,
            "result_schema_id": result_schema["$id"]}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        print(json.dumps(self_check(), ensure_ascii=False, sort_keys=True))
    except (S6ComparisonError, OSError, ValueError, KeyError) as exc:
        print(json.dumps({"status": "S6_STATIC_SELF_CHECK_FAILED", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
