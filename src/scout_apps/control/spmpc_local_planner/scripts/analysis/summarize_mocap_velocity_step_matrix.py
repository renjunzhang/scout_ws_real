#!/usr/bin/env python3
"""Aggregate V2 chassis-step reports without mixing sensors or metrics in plots."""

import argparse
import csv
import json
import math
import statistics
import sys
from collections import defaultdict
from pathlib import Path


EXPECTED_PROTOCOL = "MOCAP_VELOCITY_STEP_V2"


def finite_or_none(value):
    if value is None:
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def direction_name(axis, command):
    if axis == "linear":
        return "forward" if command > 0.0 else "reverse"
    return "left" if command > 0.0 else "right"


def extract_row(path, payload):
    if payload.get("protocol_id") != EXPECTED_PROTOCOL:
        raise ValueError(
            "{} protocol is {}, expected {}".format(
                path, payload.get("protocol_id"), EXPECTED_PROTOCOL
            )
        )
    axis = payload["axis"]
    command = float(payload["bag_time"]["command"]["command_value"])
    primary = payload["bag_time"]["sensors"]["mocap"]
    fit = primary.get("fopdt", {})
    fit_valid = bool(fit.get("valid", False))
    fit_identifiable = bool(fit_valid and fit.get("identifiable", False))
    capability = primary.get("acceleration_capability", {})
    return {
        "report": str(path.resolve()),
        "bag": payload.get("bag"),
        "run_label": payload.get("run_label"),
        "data_split": payload.get("data_split"),
        "matrix_row": payload.get("matrix_row"),
        "attempt": payload.get("attempt"),
        "axis": axis,
        "direction": direction_name(axis, command),
        "command_value": command,
        "command_magnitude": abs(command),
        "onset_delay_ms": 1000.0 * float(primary["onset_delay_sec"]),
        "t90_ms": 1000.0 * float(primary["t90_sec"]),
        "fopdt_delay_ms": (
            1000.0 * float(fit["delay_sec"]) if fit_identifiable else None
        ),
        "fopdt_tau_ms": (
            1000.0 * float(fit["tau_sec"]) if fit_identifiable else None
        ),
        "fopdt_gain": finite_or_none(fit.get("gain")) if fit_identifiable else None,
        "fopdt_r2": finite_or_none(fit.get("r2")) if fit_identifiable else None,
        "fopdt_identifiable": fit_identifiable,
        "fopdt_quality_reasons": "; ".join(fit.get("quality_reasons", [])),
        "fopdt_diagnostic_delay_ms": (
            1000.0 * float(fit["delay_sec"]) if fit_valid else None
        ),
        "fopdt_diagnostic_tau_ms": (
            1000.0 * float(fit["tau_sec"]) if fit_valid else None
        ),
        "fopdt_diagnostic_gain": (
            finite_or_none(fit.get("gain")) if fit_valid else None
        ),
        "fopdt_diagnostic_r2": (
            finite_or_none(fit.get("r2")) if fit_valid else None
        ),
        "effective_acceleration": finite_or_none(primary.get("effective_acceleration")),
        "effective_deceleration": finite_or_none(primary.get("effective_deceleration")),
        "acceleration_p95": finite_or_none(capability.get("acceleration_p95")),
        "deceleration_p95": finite_or_none(capability.get("deceleration_p95")),
        "stopping_distance_m": finite_or_none(
            primary.get("stopping_distance_after_command_m")
        ),
        "rotation_after_command_deg": finite_or_none(
            primary.get("rotation_after_command_deg")
        ),
    }


def finite_summary(values):
    values = [float(value) for value in values if value is not None and math.isfinite(value)]
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "median": statistics.median(values),
        "minimum": min(values),
        "maximum": max(values),
    }


def aggregate_rows(rows):
    groups = defaultdict(list)
    for row in rows:
        key = (row["axis"], row["direction"], row["command_magnitude"])
        groups[key].append(row)
    metrics = (
        "onset_delay_ms",
        "t90_ms",
        "fopdt_delay_ms",
        "fopdt_tau_ms",
        "fopdt_gain",
        "fopdt_r2",
        "effective_acceleration",
        "effective_deceleration",
        "acceleration_p95",
        "deceleration_p95",
        "stopping_distance_m",
        "rotation_after_command_deg",
    )
    output = []
    for (axis, direction, magnitude), group in sorted(groups.items()):
        output.append(
            {
                "axis": axis,
                "direction": direction,
                "command_magnitude": magnitude,
                "trial_count": len(group),
                "metrics": {
                    metric: finite_summary(row.get(metric) for row in group)
                    for metric in metrics
                },
            }
        )
    return output


def make_metric_plot(path, rows, axis_name, direction, metric, title, unit):
    selected = [
        row
        for row in rows
        if row["axis"] == axis_name
        and row["direction"] == direction
        and row.get(metric) is not None
    ]
    if not selected:
        return False
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    by_magnitude = defaultdict(list)
    for row in selected:
        by_magnitude[row["command_magnitude"]].append(float(row[metric]))
    magnitude = sorted(by_magnitude)
    median = [statistics.median(by_magnitude[value]) for value in magnitude]
    figure, plot_axis = plt.subplots(figsize=(7.4, 4.6), constrained_layout=True)
    for value in magnitude:
        plot_axis.scatter(
            [value] * len(by_magnitude[value]),
            by_magnitude[value],
            color="#88AADD",
            s=26,
            alpha=0.75,
        )
    plot_axis.plot(magnitude, median, color="#004488", marker="o", linewidth=1.5)
    command_unit = "m/s" if axis_name == "linear" else "rad/s"
    plot_axis.set_xlabel("Command magnitude [{}]".format(command_unit))
    plot_axis.set_ylabel("{}{}".format(title, " [{}]".format(unit) if unit else ""))
    plot_axis.set_title("{} {}: {} (NOKOV only)".format(axis_name, direction, title))
    plot_axis.grid(True, alpha=0.25)
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(str(path), dpi=170)
    plt.close(figure)
    return True


def make_plots(plot_dir, rows):
    specifications = (
        ("onset_delay_ms", "Observed onset delay", "ms"),
        ("fopdt_delay_ms", "FOPDT dead time", "ms"),
        ("fopdt_tau_ms", "FOPDT time constant", "ms"),
        ("fopdt_gain", "FOPDT steady gain", ""),
        ("effective_acceleration", "Effective acceleration", None),
        ("effective_deceleration", "Effective deceleration", None),
        ("acceleration_p95", "Acceleration P95", None),
        ("deceleration_p95", "Deceleration P95", None),
    )
    output = []
    for axis_name, direction in (
        ("angular", "left"),
        ("angular", "right"),
        ("linear", "forward"),
        ("linear", "reverse"),
    ):
        acceleration_unit = "m/s^2" if axis_name == "linear" else "rad/s^2"
        for metric, title, unit in specifications:
            actual_unit = acceleration_unit if unit is None else unit
            path = plot_dir / "{}_{}_{}.png".format(axis_name, direction, metric)
            if make_metric_plot(
                path, rows, axis_name, direction, metric, title, actual_unit
            ):
                output.append(path)
    return output


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reports", type=Path, nargs="+")
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--plot-dir", type=Path, required=True)
    return parser


def main():
    args = build_parser().parse_args()
    try:
        rows = [
            extract_row(path, json.loads(path.read_text(encoding="utf-8")))
            for path in args.reports
        ]
    except Exception as exc:
        print("[summarize_mocap_velocity_step_matrix][ERR] {}".format(exc), file=sys.stderr)
        return 2
    if len({row["report"] for row in rows}) != len(rows):
        print("[summarize_mocap_velocity_step_matrix][ERR] duplicate report", file=sys.stderr)
        return 2
    data_splits = {row["data_split"] for row in rows}
    if len(data_splits) != 1:
        print(
            "[summarize_mocap_velocity_step_matrix][ERR] mixed data splits: {}".format(
                sorted(data_splits)
            ),
            file=sys.stderr,
        )
        return 2
    aggregate = aggregate_rows(rows)
    plot_paths = make_plots(args.plot_dir, rows)
    expected_rows = ["{:02d}".format(value) for value in range(1, 13)]
    observed_rows = sorted(
        {row["matrix_row"] for row in rows if row["matrix_row"] in expected_rows}
    )
    payload = {
        "protocol_id": EXPECTED_PROTOCOL,
        "report_type": "MOCAP_VELOCITY_STEP_MATRIX_SUMMARY_V2",
        "primary_sensor": "NOKOV",
        "trial_count": len(rows),
        "data_split": next(iter(data_splits)),
        "matrix_coverage": {
            "expected_rows": expected_rows,
            "observed_rows": observed_rows,
            "missing_rows": [row for row in expected_rows if row not in observed_rows],
            "complete": observed_rows == expected_rows,
        },
        "trials": rows,
        "groups": aggregate,
        "plots": [str(path.resolve()) for path in plot_paths],
        "plot_contract": "one axis + one direction + one metric + NOKOV only per image",
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0])
    with args.output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print("[summarize_mocap_velocity_step_matrix] PASS")
    print("  trials : {}".format(len(rows)))
    print("  json   : {}".format(args.output_json))
    print("  csv    : {}".format(args.output_csv))
    print("  plots  : {} ({} files)".format(args.plot_dir, len(plot_paths)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
