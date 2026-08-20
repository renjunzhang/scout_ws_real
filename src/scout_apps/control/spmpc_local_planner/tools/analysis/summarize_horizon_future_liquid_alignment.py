#!/usr/bin/env python3
"""Build the descriptive, per-run horizon/liquid alignment summary.

The primary RGB statistics are recomputed from native RGB-event rows.  A
strictly limited fallback can rebuild sample-count-weighted RMSE from the
fine-bin metric table when the event table is unavailable; Pearson
correlation is deliberately not reconstructed in that case.
"""

import argparse
import csv
import hashlib
import json
import math
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, MutableMapping, Optional, Sequence, Tuple


STATUS = "DESCRIPTIVE ONLY"
MIN_N = 100
GRID_HZ = 30.0
PRIMARY_LAG_MODE = "primary_delta0"
CAUSAL_SCOPE = "causal"

# The labels are the requested rounded millisecond intervals.  Membership is
# defined on the analyzer's exact 30 Hz floor bins so the native-event and
# fine-bin fallback paths have identical boundaries.
WIDE_LEAD_BINS = (
    ("0–33 ms", 0, 33, 0, 1),
    ("33–67 ms", 33, 67, 1, 2),
    ("67–100 ms", 67, 100, 2, 3),
    ("100–167 ms", 100, 167, 3, 5),
    ("167–333 ms", 167, 333, 5, 10),
    ("333–500 ms", 333, 500, 10, 15),
    ("500–1000 ms", 500, 1000, 15, 30),
)

# display name, native-event prediction field, analyzer metric model
METHODS = (
    ("OCP", "pred_projection_mm", "mpc_horizon"),
    ("planned_high_accuracy", "planned_replay_projection_mm", "planned_high_accuracy"),
    ("zero_input", "zero_input_projection_mm", "zero_input"),
    ("actual_input_replay", "actual_replay_projection_mm", "actual_input_replay"),
    ("future_observer", "future_observer_projection_mm", "future_observer"),
)
METHOD_ORDER = tuple(item[0] for item in METHODS)
EVENT_FIELD_BY_METHOD = {item[0]: item[1] for item in METHODS}
MODEL_BY_METHOD = {item[0]: item[2] for item in METHODS}
METHOD_BY_MODEL = {item[2]: item[0] for item in METHODS}

SELECTED_FIELDS = (
    "status",
    "metric_family",
    "source_file",
    "aggregation",
    "run_id",
    "block",
    "row",
    "target",
    "lag_mode",
    "scope",
    "method",
    "source_model",
    "lead_interval",
    "lead_start_ms",
    "lead_end_ms",
    "bin_j_start",
    "bin_j_end_exclusive",
    "grid_step",
    "grid_time_ms",
    "n",
    "correlation",
    "rmse_mm",
    "note",
)


@dataclass
class RunningPairMetric:
    """Numerically stable online Pearson/RMSE accumulator."""

    n: int = 0
    mean_prediction: float = 0.0
    mean_target: float = 0.0
    prediction_m2: float = 0.0
    target_m2: float = 0.0
    covariance_sum: float = 0.0
    squared_error_sum: float = 0.0

    def add(self, prediction: float, target: float) -> None:
        old_n = self.n
        self.n += 1
        prediction_delta = prediction - self.mean_prediction
        target_delta = target - self.mean_target
        self.mean_prediction += prediction_delta / self.n
        self.mean_target += target_delta / self.n
        self.prediction_m2 += prediction_delta * (prediction - self.mean_prediction)
        self.target_m2 += target_delta * (target - self.mean_target)
        if old_n:
            self.covariance_sum += (
                prediction_delta * target_delta * float(old_n) / float(self.n)
            )
        error = prediction - target
        self.squared_error_sum += error * error

    def correlation(self) -> Optional[float]:
        denominator = self.prediction_m2 * self.target_m2
        if self.n < 3 or denominator <= 0.0:
            return None
        value = self.covariance_sum / math.sqrt(denominator)
        # Roundoff can put a mathematically bounded value a few ulps outside.
        return max(-1.0, min(1.0, value))

    def rmse(self) -> Optional[float]:
        if self.n <= 0:
            return None
        return math.sqrt(self.squared_error_sum / float(self.n))


@dataclass
class WeightedRmseMetric:
    n: int = 0
    weighted_squared_rmse_sum: float = 0.0

    def add(self, n: int, rmse: float) -> None:
        self.n += n
        self.weighted_squared_rmse_sum += float(n) * rmse * rmse

    def rmse(self) -> Optional[float]:
        if self.n <= 0:
            return None
        return math.sqrt(self.weighted_squared_rmse_sum / float(self.n))


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Summarize per-run horizon/future-liquid alignment outputs "
            "(descriptive only)."
        )
    )
    parser.add_argument(
        "--analysis-dir",
        required=True,
        type=Path,
        help="Directory produced by analyze_horizon_future_liquid_alignment.py",
    )
    return parser.parse_args(argv)


def finite_float(value: Any) -> Optional[float]:
    if value is None or str(value).strip() == "":
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def integer_value(value: Any) -> Optional[int]:
    parsed = finite_float(value)
    if parsed is None or abs(parsed - round(parsed)) > 1e-9:
        return None
    return int(round(parsed))


def load_json(path: Path) -> Mapping[str, Any]:
    if not path.is_file():
        raise RuntimeError("missing required input: {}".format(path))
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise RuntimeError("cannot read JSON {}: {}".format(path, exc))
    if not isinstance(payload, dict):
        raise RuntimeError("JSON root must be an object: {}".format(path))
    return payload


def file_provenance(path: Path) -> Dict[str, Any]:
    """Return a streaming SHA-256 record for one immutable input/output."""
    if not path.is_file():
        raise RuntimeError("cannot hash missing file: {}".format(path))
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return {
        "path": str(path.resolve()),
        "size_bytes": path.stat().st_size,
        "sha256": digest.hexdigest(),
    }


def csv_reader(path: Path, required_fields: Iterable[str]):
    if not path.is_file():
        raise RuntimeError("missing required input: {}".format(path))
    stream = path.open("r", encoding="utf-8", newline="")
    reader = csv.DictReader(stream)
    fieldnames = set(reader.fieldnames or ())
    missing = sorted(set(required_fields) - fieldnames)
    if missing:
        stream.close()
        raise RuntimeError(
            "{} is missing required columns: {}".format(path, ", ".join(missing))
        )
    return stream, reader


def lead_bin_for_j(bin_j: int) -> Optional[Tuple[str, int, int, int, int]]:
    for definition in WIDE_LEAD_BINS:
        if definition[3] <= bin_j < definition[4]:
            return definition
    return None


def load_run_metadata(path: Path) -> Tuple[List[str], Dict[str, Dict[str, str]]]:
    required = ("run_id", "block", "row")
    stream, reader = csv_reader(path, required)
    run_order: List[str] = []
    metadata: Dict[str, Dict[str, str]] = {}
    try:
        for record in reader:
            run_id = str(record.get("run_id", "")).strip()
            if not run_id:
                raise RuntimeError("empty run_id in {}".format(path))
            if run_id in metadata:
                raise RuntimeError("duplicate run_id in {}: {}".format(path, run_id))
            run_order.append(run_id)
            metadata[run_id] = {
                "block": str(record.get("block", "")),
                "row": str(record.get("row", "")),
                "bag": str(record.get("bag", "")),
            }
    finally:
        stream.close()
    if not run_order:
        raise RuntimeError("no runs in {}".format(path))
    return run_order, metadata


def ordered_runs(run_order: Sequence[str], rows: Sequence[Mapping[str, Any]]) -> List[str]:
    present = {str(row["run_id"]) for row in rows}
    ordered = [run_id for run_id in run_order if run_id in present]
    ordered.extend(sorted(present - set(ordered)))
    return ordered


def base_selected_row(
    run_id: str,
    run_metadata: Mapping[str, Mapping[str, str]],
    method: str,
) -> Dict[str, Any]:
    metadata = run_metadata.get(run_id, {})
    return {
        "status": STATUS,
        "run_id": run_id,
        "block": metadata.get("block", ""),
        "row": metadata.get("row", ""),
        "method": method,
        "source_model": MODEL_BY_METHOD[method],
    }


def aggregate_native_events(
    path: Path,
    run_metadata: Mapping[str, Mapping[str, str]],
) -> List[Dict[str, Any]]:
    required = {
        "run_id",
        "lag_mode",
        "model_availability",
        "causal_lead_bin_j",
        "rgb_centered_signed_slope_mm",
    }
    required.update(EVENT_FIELD_BY_METHOD.values())
    stream, reader = csv_reader(path, required)
    accumulators: MutableMapping[
        Tuple[str, str, str], RunningPairMetric
    ] = {}
    matching_event_rows = 0
    try:
        for record in reader:
            if (
                str(record.get("lag_mode", "")) != PRIMARY_LAG_MODE
                or str(record.get("model_availability", "")) != CAUSAL_SCOPE
            ):
                continue
            bin_j = integer_value(record.get("causal_lead_bin_j"))
            target = finite_float(record.get("rgb_centered_signed_slope_mm"))
            if bin_j is None or target is None:
                continue
            lead_bin = lead_bin_for_j(bin_j)
            if lead_bin is None:
                continue
            run_id = str(record.get("run_id", "")).strip()
            if not run_id:
                raise RuntimeError("empty run_id in {}".format(path))
            matching_event_rows += 1
            interval_label = lead_bin[0]
            for method in METHOD_ORDER:
                prediction = finite_float(record.get(EVENT_FIELD_BY_METHOD[method]))
                if prediction is None:
                    continue
                key = (run_id, method, interval_label)
                accumulators.setdefault(key, RunningPairMetric()).add(prediction, target)
    finally:
        stream.close()
    if matching_event_rows == 0:
        raise RuntimeError(
            "{} has no primary_delta0 causal native RGB rows in 0–1000 ms".format(path)
        )

    rows: List[Dict[str, Any]] = []
    for lead_bin in WIDE_LEAD_BINS:
        label, start_ms, end_ms, start_j, end_j = lead_bin
        for method in METHOD_ORDER:
            run_ids = sorted(
                key[0]
                for key in accumulators
                if key[1] == method and key[2] == label
            )
            for run_id in run_ids:
                metric = accumulators[(run_id, method, label)]
                if metric.n < MIN_N:
                    continue
                row = base_selected_row(run_id, run_metadata, method)
                row.update(
                    {
                        "metric_family": "native_rgb_vs_causal_lead",
                        "source_file": path.name,
                        "aggregation": "exact_native_event_reaggregation",
                        "target": "rgb_centered_signed_slope_mm",
                        "lag_mode": PRIMARY_LAG_MODE,
                        "scope": CAUSAL_SCOPE,
                        "lead_interval": label,
                        "lead_start_ms": start_ms,
                        "lead_end_ms": end_ms,
                        "bin_j_start": start_j,
                        "bin_j_end_exclusive": end_j,
                        "grid_step": "",
                        "grid_time_ms": "",
                        "n": metric.n,
                        "correlation": metric.correlation(),
                        "rmse_mm": metric.rmse(),
                        "note": (
                            "Exact Pearson correlation and RMSE from native event-cycle "
                            "pairs; overlapping cycles are not independent."
                        ),
                    }
                )
                rows.append(row)
    return rows


def aggregate_native_fine_bins(
    path: Path,
    run_metadata: Mapping[str, Mapping[str, str]],
) -> List[Dict[str, Any]]:
    required = (
        "run_id",
        "target_type",
        "lag_mode",
        "bin_j",
        "scope",
        "model",
        "n",
        "rmse",
    )
    stream, reader = csv_reader(path, required)
    accumulators: MutableMapping[
        Tuple[str, str, str], WeightedRmseMetric
    ] = {}
    seen_fine_keys = set()
    try:
        for record in reader:
            model = str(record.get("model", ""))
            if (
                str(record.get("target_type", "")) != "native_rgb_event"
                or str(record.get("lag_mode", "")) != PRIMARY_LAG_MODE
                or str(record.get("scope", "")) != CAUSAL_SCOPE
                or model not in METHOD_BY_MODEL
            ):
                continue
            run_id = str(record.get("run_id", "")).strip()
            if not run_id or run_id == "ALL_POOLED":
                continue
            bin_j = integer_value(record.get("bin_j"))
            n = integer_value(record.get("n"))
            rmse = finite_float(record.get("rmse"))
            if bin_j is None or n is None or n <= 0 or rmse is None:
                continue
            lead_bin = lead_bin_for_j(bin_j)
            if lead_bin is None:
                continue
            fine_key = (run_id, model, bin_j)
            if fine_key in seen_fine_keys:
                raise RuntimeError("duplicate fine-bin metric row: {}".format(fine_key))
            seen_fine_keys.add(fine_key)
            method = METHOD_BY_MODEL[model]
            key = (run_id, method, lead_bin[0])
            accumulators.setdefault(key, WeightedRmseMetric()).add(n, rmse)
    finally:
        stream.close()
    if not accumulators:
        raise RuntimeError(
            "{} has no primary_delta0 causal native RGB fine-bin metrics".format(path)
        )

    rows: List[Dict[str, Any]] = []
    for lead_bin in WIDE_LEAD_BINS:
        label, start_ms, end_ms, start_j, end_j = lead_bin
        for method in METHOD_ORDER:
            run_ids = sorted(
                key[0]
                for key in accumulators
                if key[1] == method and key[2] == label
            )
            for run_id in run_ids:
                metric = accumulators[(run_id, method, label)]
                if metric.n < MIN_N:
                    continue
                row = base_selected_row(run_id, run_metadata, method)
                row.update(
                    {
                        "metric_family": "native_rgb_vs_causal_lead",
                        "source_file": path.name,
                        "aggregation": "sample_count_weighted_rmse_from_fine_bins",
                        "target": "rgb_centered_signed_slope_mm",
                        "lag_mode": PRIMARY_LAG_MODE,
                        "scope": CAUSAL_SCOPE,
                        "lead_interval": label,
                        "lead_start_ms": start_ms,
                        "lead_end_ms": end_ms,
                        "bin_j_start": start_j,
                        "bin_j_end_exclusive": end_j,
                        "grid_step": "",
                        "grid_time_ms": "",
                        "n": metric.n,
                        "correlation": None,
                        "rmse_mm": metric.rmse(),
                        "note": "cannot reconstruct correlation from bin summaries",
                    }
                )
                rows.append(row)
    return rows


def load_observer_grid_metrics(
    path: Path,
    run_metadata: Mapping[str, Mapping[str, str]],
) -> List[Dict[str, Any]]:
    required = (
        "run_id",
        "target_type",
        "lag_mode",
        "bin_j",
        "scope",
        "model",
        "n",
        "correlation",
        "rmse",
    )
    stream, reader = csv_reader(path, required)
    rows: List[Dict[str, Any]] = []
    seen_keys = set()
    try:
        for record in reader:
            model = str(record.get("model", ""))
            if (
                str(record.get("target_type", "")) != "observer_projection"
                or str(record.get("lag_mode", "")) != "state_time"
                or str(record.get("scope", "")) != CAUSAL_SCOPE
                or model not in METHOD_BY_MODEL
                or model == "future_observer"
            ):
                continue
            run_id = str(record.get("run_id", "")).strip()
            if not run_id or run_id == "ALL_POOLED":
                continue
            grid_step = integer_value(record.get("bin_j"))
            n = integer_value(record.get("n"))
            rmse = finite_float(record.get("rmse"))
            if grid_step is None or grid_step < 0 or n is None or n < MIN_N or rmse is None:
                continue
            key = (run_id, model, grid_step)
            if key in seen_keys:
                raise RuntimeError("duplicate observer grid metric row: {}".format(key))
            seen_keys.add(key)
            method = METHOD_BY_MODEL[model]
            row = base_selected_row(run_id, run_metadata, method)
            row.update(
                {
                    "metric_family": "observer_projection_vs_grid_step",
                    "source_file": path.name,
                    "aggregation": "direct_per_run_grid_bin_metric",
                    "target": "future_observer_projection_mm",
                    "lag_mode": "state_time",
                    "scope": CAUSAL_SCOPE,
                    "lead_interval": "",
                    "lead_start_ms": "",
                    "lead_end_ms": "",
                    "bin_j_start": "",
                    "bin_j_end_exclusive": "",
                    "grid_step": grid_step,
                    "grid_time_ms": 1000.0 * float(grid_step) / GRID_HZ,
                    "n": n,
                    "correlation": finite_float(record.get("correlation")),
                    "rmse_mm": rmse,
                    "note": (
                        "Direct per-run metric; future_observer is the target and is "
                        "not drawn as a tautological identity curve."
                    ),
                }
            )
            rows.append(row)
    finally:
        stream.close()
    if not rows:
        raise RuntimeError(
            "{} has no causal observer-projection grid metrics with n >= {}".format(
                path, MIN_N
            )
        )
    return rows


def format_csv_value(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return "{:.12g}".format(value)
    return value


def write_selected_metrics(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(SELECTED_FIELDS))
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {field: format_csv_value(row.get(field)) for field in SELECTED_FIELDS}
            )


def plot_setup():
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("matplotlib is required to write PNG summaries: {}".format(exc))
    return plt


def run_label(run_id: str, metadata: Mapping[str, Mapping[str, str]]) -> str:
    values = metadata.get(run_id, {})
    block = values.get("block", "")
    row = values.get("row", "")
    if block or row:
        return "block {} / row {}".format(block or "?", row or "?")
    return run_id


def native_metric_plot(
    path: Path,
    rows: Sequence[Mapping[str, Any]],
    run_order: Sequence[str],
    run_metadata: Mapping[str, Mapping[str, str]],
    metric_field: str,
    ylabel: str,
    title: str,
    fallback: bool,
) -> None:
    plt = plot_setup()
    native_rows = [row for row in rows if row["metric_family"] == "native_rgb_vs_causal_lead"]
    runs = ordered_runs(run_order, native_rows)
    if not runs:
        raise RuntimeError("no native RGB rows available for plotting")
    colors = {
        "OCP": "#1f77b4",
        "planned_high_accuracy": "#ff7f0e",
        "zero_input": "#7f7f7f",
        "actual_input_replay": "#2ca02c",
        "future_observer": "#d62728",
    }
    line_styles = {"future_observer": "--"}
    markers = {"actual_input_replay": "s", "future_observer": "x"}
    centers = {
        definition[0]: 500.0 * float(definition[3] + definition[4]) / GRID_HZ
        for definition in WIDE_LEAD_BINS
    }
    figure, axes = plt.subplots(
        len(runs), 1, figsize=(13.0, max(4.2, 3.8 * len(runs))), sharex=True, squeeze=False
    )
    for axis, run_id in zip(axes[:, 0], runs):
        plotted = 0
        for method in METHOD_ORDER:
            method_rows = sorted(
                (
                    row
                    for row in native_rows
                    if row["run_id"] == run_id
                    and row["method"] == method
                    and row.get(metric_field) is not None
                ),
                key=lambda row: float(row["bin_j_start"]),
            )
            if not method_rows:
                continue
            axis.plot(
                [centers[str(row["lead_interval"])] for row in method_rows],
                [float(row[metric_field]) for row in method_rows],
                linewidth=1.8,
                markersize=4.5,
                color=colors[method],
                linestyle=line_styles.get(method, "-"),
                marker=markers.get(method, "o"),
                label=method,
            )
            plotted += 1
        if not plotted:
            explanation = "No eligible values with n >= {}".format(MIN_N)
            if metric_field == "correlation" and fallback:
                explanation += "\ncannot reconstruct correlation from bin summaries"
            axis.text(0.5, 0.5, explanation, ha="center", va="center", transform=axis.transAxes)
        axis.set_title("{}  ({})".format(run_label(run_id, run_metadata), run_id), fontsize=10)
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.25)
        if metric_field == "correlation":
            axis.axhline(0.0, color="black", linewidth=0.8, alpha=0.35)
            axis.set_ylim(-1.05, 1.05)
        if plotted:
            axis.legend(loc="best", fontsize=8, ncol=3)
    axes[-1, 0].set_xlabel("causal lead interval (rounded ms labels; exact 30 Hz bin membership)")
    axes[-1, 0].set_xticks([centers[item[0]] for item in WIDE_LEAD_BINS])
    axes[-1, 0].set_xticklabels(
        [item[0].replace(" ms", "\nms") for item in WIDE_LEAD_BINS], fontsize=9
    )
    figure.suptitle("{} — {}".format(title, STATUS), fontsize=14, fontweight="bold")
    source_note = (
        "Fine-bin fallback: weighted RMSE only; correlation is NA."
        if fallback
        else "Exact reaggregation from native RGB event-cycle pairs."
    )
    figure.text(
        0.5,
        0.008,
        "primary_delta0 · causal only · n >= {} · per-run lines · no pooled CI. {}".format(
            MIN_N, source_note
        ),
        ha="center",
        fontsize=9,
    )
    figure.tight_layout(rect=(0.0, 0.035, 1.0, 0.96))
    figure.savefig(
        str(path),
        dpi=180,
        bbox_inches="tight",
        metadata={"Title": title, "Description": STATUS},
    )
    plt.close(figure)


def observer_grid_plot(
    path: Path,
    rows: Sequence[Mapping[str, Any]],
    run_order: Sequence[str],
    run_metadata: Mapping[str, Mapping[str, str]],
) -> None:
    plt = plot_setup()
    observer_rows = [
        row for row in rows if row["metric_family"] == "observer_projection_vs_grid_step"
    ]
    runs = ordered_runs(run_order, observer_rows)
    if not runs:
        raise RuntimeError("no observer grid rows available for plotting")
    colors = {
        "OCP": "#1f77b4",
        "planned_high_accuracy": "#ff7f0e",
        "zero_input": "#7f7f7f",
        "actual_input_replay": "#2ca02c",
    }
    figure, axes = plt.subplots(
        len(runs), 2, figsize=(14.0, max(4.5, 3.8 * len(runs))), squeeze=False, sharex="col"
    )
    for row_index, run_id in enumerate(runs):
        for column, metric_field, ylabel in (
            (0, "correlation", "Pearson correlation"),
            (1, "rmse_mm", "RMSE (mm)"),
        ):
            axis = axes[row_index, column]
            plotted = 0
            for method in METHOD_ORDER:
                if method == "future_observer":
                    continue
                method_rows = sorted(
                    (
                        row
                        for row in observer_rows
                        if row["run_id"] == run_id
                        and row["method"] == method
                        and row.get(metric_field) is not None
                    ),
                    key=lambda row: int(row["grid_step"]),
                )
                if not method_rows:
                    continue
                axis.plot(
                    [int(row["grid_step"]) for row in method_rows],
                    [float(row[metric_field]) for row in method_rows],
                    linewidth=1.7,
                    color=colors[method],
                    label=method,
                )
                plotted += 1
            axis.set_ylabel(ylabel)
            axis.grid(True, alpha=0.25)
            if metric_field == "correlation":
                axis.axhline(0.0, color="black", linewidth=0.8, alpha=0.35)
                axis.set_ylim(-1.05, 1.05)
            if plotted:
                axis.legend(loc="best", fontsize=8)
            if column == 0:
                axis.set_title(
                    "{} ({})\ncorrelation".format(run_label(run_id, run_metadata), run_id),
                    fontsize=9,
                )
            else:
                axis.set_title("RMSE", fontsize=9)
        axes[row_index, 0].set_xlabel("horizon grid step j")
        axes[row_index, 1].set_xlabel("horizon grid step j")
    figure.suptitle(
        "Projection versus future observer by grid step — {}".format(STATUS),
        fontsize=14,
        fontweight="bold",
    )
    figure.text(
        0.5,
        0.008,
        (
            "state_time · causal only · n >= {} · per-run lines · no pooled CI. "
            "future_observer is the target, not an identity curve."
        ).format(MIN_N),
        ha="center",
        fontsize=9,
    )
    figure.tight_layout(rect=(0.0, 0.035, 1.0, 0.96))
    figure.savefig(
        str(path),
        dpi=180,
        bbox_inches="tight",
        metadata={
            "Title": "Observer projection versus grid step",
            "Description": STATUS,
        },
    )
    plt.close(figure)


def markdown_float(value: Any, digits: int = 3) -> str:
    parsed = finite_float(value)
    return "NA" if parsed is None else ("{:.%df}" % digits).format(parsed)


def markdown_native_tables(
    rows: Sequence[Mapping[str, Any]], run_order: Sequence[str]
) -> List[str]:
    output: List[str] = []
    native_rows = [row for row in rows if row["metric_family"] == "native_rgb_vs_causal_lead"]
    for run_id in ordered_runs(run_order, native_rows):
        output.extend(
            [
                "### `{}`".format(run_id),
                "",
                "| causal lead | method | n | correlation | RMSE (mm) |",
                "|---|---:|---:|---:|---:|",
            ]
        )
        run_rows = sorted(
            (row for row in native_rows if row["run_id"] == run_id),
            key=lambda row: (
                int(row["bin_j_start"]),
                METHOD_ORDER.index(str(row["method"])),
            ),
        )
        for row in run_rows:
            output.append(
                "| {} | {} | {} | {} | {} |".format(
                    row["lead_interval"],
                    row["method"],
                    row["n"],
                    markdown_float(row.get("correlation")),
                    markdown_float(row.get("rmse_mm")),
                )
            )
        output.append("")
    return output


def build_report(
    path: Path,
    analysis_dir: Path,
    manifest: Mapping[str, Any],
    projection_fit: Mapping[str, Any],
    rows: Sequence[Mapping[str, Any]],
    run_order: Sequence[str],
    run_metadata: Mapping[str, Mapping[str, str]],
    fallback: bool,
) -> None:
    fits = projection_fit.get("fits")
    if not isinstance(fits, dict) or not isinstance(fits.get(PRIMARY_LAG_MODE), dict):
        raise RuntimeError("projection_fit.json lacks fits.primary_delta0")
    primary_fit = fits[PRIMARY_LAG_MODE]
    coefficients = primary_fit.get("coefficients", {})
    if not isinstance(coefficients, dict):
        coefficients = {}
    native_rows = [row for row in rows if row["metric_family"] == "native_rgb_vs_causal_lead"]
    observer_rows = [
        row for row in rows if row["metric_family"] == "observer_projection_vs_grid_step"
    ]
    intervals_present = {
        str(row["lead_interval"]) for row in native_rows if row.get("lead_interval")
    }
    missing_intervals = [item[0] for item in WIDE_LEAD_BINS if item[0] not in intervals_present]
    report: List[str] = [
        "# Horizon / future-liquid alignment summary — DESCRIPTIVE ONLY",
        "",
        "> **DESCRIPTIVE ONLY.** These overlapping event-cycle pairs are not independent "
        "experimental replicates. No pooled confidence interval or inferential claim is made.",
        "",
        "## Frozen selection contract",
        "",
        "- Primary RGB alignment only: `lag_mode=primary_delta0` (`delta=0`).",
        "- Causal rows only; hindcast, boundary, `all`, and pooled rows are excluded.",
        "- Every displayed/selected row has `n >= {}`.".format(MIN_N),
        "- Methods: `OCP`, `planned_high_accuracy`, `zero_input`, "
        "`actual_input_replay`, and `future_observer`.",
        "- Runs are drawn separately. There is no pooled curve and no confidence band.",
        "- Rounded wide lead labels use exact 30 Hz floor-bin membership: "
        "`j=[0,1), [1,2), [2,3), [3,5), [5,10), [10,15), [15,30)`.",
        "",
        "## Inputs and provenance",
        "",
        "- Analysis directory: `{}`".format(analysis_dir),
        "- Manifest report type: `{}`".format(manifest.get("report_type", "unknown")),
        "- Manifest status: `{}`".format(manifest.get("status", "unknown")),
        "- Workspace revision recorded by analyzer: `{}`".format(
            manifest.get("workspace_git_revision", "unknown")
        ),
        "- RGB metric source: `{}`.".format(
            "per_run_bin_metrics.csv (fallback)"
            if fallback
            else "per_cycle_rgb_event.csv (exact native-event reaggregation)"
        ),
        "- Observer-grid source: `per_run_bin_metrics.csv`.",
        "- Exact input/output SHA-256 records: `summary_manifest.json`.",
        "",
        "| run | block | row |",
        "|---|---:|---:|",
    ]
    for run_id in run_order:
        metadata = run_metadata.get(run_id, {})
        report.append(
            "| `{}` | {} | {} |".format(
                run_id, metadata.get("block", ""), metadata.get("row", "")
            )
        )
    report.extend(
        [
            "",
            "## Projection fit used by `primary_delta0`",
            "",
            "This is reported for provenance, not treated as an independent test result.",
            "",
            "| item | value |",
            "|---|---:|",
            "| fit scope | `{}` |".format(projection_fit.get("fit_scope", "unknown")),
            "| native training events | {} |".format(primary_fit.get("native_event_count", "NA")),
            "| fit correlation | {} |".format(markdown_float(primary_fit.get("correlation"))),
            "| fit RMSE (mm) | {} |".format(markdown_float(primary_fit.get("rmse_mm"))),
            "| raw design condition number | {} |".format(
                markdown_float(primary_fit.get("raw_design_condition_number"), 1)
            ),
            "| `b0_mm` | {} |".format(markdown_float(coefficients.get("b0_mm"), 6)),
            "| `eta_x_mm` | {} |".format(markdown_float(coefficients.get("eta_x_mm"), 6)),
            "| `eta_y_mm` | {} |".format(markdown_float(coefficients.get("eta_y_mm"), 6)),
            "",
            "## Native RGB versus causal lead",
            "",
        ]
    )
    if fallback:
        report.extend(
            [
                "`per_cycle_rgb_event.csv` was unavailable. RMSE below is reconstructed only as "
                "`sqrt(sum(n_i * rmse_i^2) / sum(n_i))`.",
                "",
                "**cannot reconstruct correlation from bin summaries**; correlation is therefore "
                "`NA`, never averaged or interpolated.",
                "",
            ]
        )
    else:
        report.extend(
            [
                "Pearson correlation and RMSE were recomputed exactly from finite prediction/target "
                "pairs in the native RGB-event table.",
                "",
            ]
        )
    if missing_intervals:
        report.extend(
            [
                "Intervals with no method/run meeting `n >= {}`: {}.".format(
                    MIN_N, ", ".join(missing_intervals)
                ),
                "",
            ]
        )
    report.extend(markdown_native_tables(native_rows, run_order))
    report.extend(
        [
            "## Observer projection versus grid step",
            "",
            "The observer plot uses the direct per-run `observer_projection/state_time/causal` "
            "fine-grid metrics. `future_observer` is the target here, so it is not plotted as a "
            "tautological correlation-1/RMSE-0 identity curve. It remains a predictor in both "
            "native-RGB plots.",
            "",
            "Selected observer-grid rows: {} (all `n >= {}`).".format(
                len(observer_rows), MIN_N
            ),
            "",
            "## Outputs",
            "",
            "- `native_rgb_correlation_vs_causal_lead.png`",
            "- `native_rgb_rmse_vs_causal_lead.png`",
            "- `observer_projection_vs_grid_step.png`",
            "- `selected_metrics.csv`",
            "- `report.md`",
            "- `summary_manifest.json`",
            "",
            "## Interpretation limits",
            "",
            "- DESCRIPTIVE ONLY: repeated, overlapping control cycles do not supply independent n.",
            "- Correlation does not establish correct amplitude, phase, calibration, or full-state "
            "observability.",
            "- The camera projection is a fitted one-dimensional liquid measurement; it does not "
            "validate all four modal-state components.",
            "- `actual_input_replay` and future-observer comparisons are retrospective diagnostics, "
            "not online forecast availability claims.",
            "- No trust horizon or causal effect is selected from these descriptive curves.",
            "",
        ]
    )
    path.write_text("\n".join(report), encoding="utf-8")


def write_summary_manifest(
    path: Path,
    analysis_dir: Path,
    input_paths: Sequence[Path],
    output_paths: Sequence[Path],
    fallback: bool,
) -> None:
    """Bind the derived summary to its exact script, inputs, and outputs."""
    payload = {
        "schema_version": 1,
        "report_type": "G3R2_HORIZON_FUTURE_LIQUID_ALIGNMENT_SUMMARY",
        "status": STATUS,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "analysis_directory": str(analysis_dir),
        "selection_contract": {
            "lag_mode": PRIMARY_LAG_MODE,
            "scope": CAUSAL_SCOPE,
            "minimum_n": MIN_N,
            "pooled_rows_allowed": False,
            "native_rgb_source": (
                "per_run_bin_metrics.csv_weighted_rmse_only"
                if fallback
                else "per_cycle_rgb_event.csv_exact_reaggregation"
            ),
        },
        "summarizer": file_provenance(Path(__file__).resolve()),
        "inputs": [file_provenance(item) for item in input_paths],
        "outputs": [file_provenance(item) for item in output_paths],
    }
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def validate_manifest(manifest: Mapping[str, Any]) -> None:
    report_type = manifest.get("report_type")
    if report_type not in (None, "G3R2_HORIZON_FUTURE_LIQUID_ALIGNMENT"):
        raise RuntimeError("unexpected manifest report_type: {}".format(report_type))
    status = str(manifest.get("status", ""))
    if status and "DESCRIPTIVE" not in status.upper():
        raise RuntimeError("manifest is not marked descriptive: {}".format(status))


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    analysis_dir = args.analysis_dir.expanduser().resolve()
    if not analysis_dir.is_dir():
        raise RuntimeError("--analysis-dir is not a directory: {}".format(analysis_dir))

    manifest_path = analysis_dir / "manifest.json"
    projection_fit_path = analysis_dir / "projection_fit.json"
    run_summary_path = analysis_dir / "run_summary.csv"
    manifest = load_json(manifest_path)
    validate_manifest(manifest)
    projection_fit = load_json(projection_fit_path)
    run_order, run_metadata = load_run_metadata(run_summary_path)
    metric_path = analysis_dir / "per_run_bin_metrics.csv"

    event_path = analysis_dir / "per_cycle_rgb_event.csv"
    fallback = not event_path.is_file()
    if fallback:
        native_rows = aggregate_native_fine_bins(metric_path, run_metadata)
    else:
        native_rows = aggregate_native_events(event_path, run_metadata)
    observer_rows = load_observer_grid_metrics(metric_path, run_metadata)
    selected_rows = native_rows + observer_rows

    if not native_rows:
        raise RuntimeError("no native RGB wide-bin rows meet n >= {}".format(MIN_N))
    if any(int(row["n"]) < MIN_N for row in selected_rows):
        raise RuntimeError("internal error: selected row below frozen n threshold")
    if any(str(row["run_id"]) == "ALL_POOLED" for row in selected_rows):
        raise RuntimeError("internal error: pooled row entered selected output")

    write_selected_metrics(analysis_dir / "selected_metrics.csv", selected_rows)
    native_metric_plot(
        analysis_dir / "native_rgb_correlation_vs_causal_lead.png",
        native_rows,
        run_order,
        run_metadata,
        "correlation",
        "Pearson correlation",
        "Native RGB correlation versus causal lead",
        fallback,
    )
    native_metric_plot(
        analysis_dir / "native_rgb_rmse_vs_causal_lead.png",
        native_rows,
        run_order,
        run_metadata,
        "rmse_mm",
        "RMSE (mm)",
        "Native RGB RMSE versus causal lead",
        fallback,
    )
    observer_grid_plot(
        analysis_dir / "observer_projection_vs_grid_step.png",
        observer_rows,
        run_order,
        run_metadata,
    )
    build_report(
        analysis_dir / "report.md",
        analysis_dir,
        manifest,
        projection_fit,
        selected_rows,
        run_order,
        run_metadata,
        fallback,
    )
    output_paths = (
        analysis_dir / "native_rgb_correlation_vs_causal_lead.png",
        analysis_dir / "native_rgb_rmse_vs_causal_lead.png",
        analysis_dir / "observer_projection_vs_grid_step.png",
        analysis_dir / "selected_metrics.csv",
        analysis_dir / "report.md",
    )
    input_paths = [manifest_path, projection_fit_path, run_summary_path, metric_path]
    if not fallback:
        input_paths.append(event_path)
    write_summary_manifest(
        analysis_dir / "summary_manifest.json",
        analysis_dir,
        input_paths,
        output_paths,
        fallback,
    )
    print(
        "[PASS] wrote descriptive per-run summary to {} (RGB source: {})".format(
            analysis_dir, "fine-bin RMSE fallback" if fallback else "native RGB events"
        )
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # noqa: BLE001 - fail closed with a concise CLI error.
        print("[FAIL] {}".format(exc), file=sys.stderr)
        sys.exit(1)
