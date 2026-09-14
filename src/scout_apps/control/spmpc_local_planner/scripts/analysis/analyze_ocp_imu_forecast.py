#!/usr/bin/env python3
"""Compare recorded Full OCP nodes with future IMU model states, offline only.

This is forecast consistency against a motion-driven model, not liquid ground
truth or an executed-task performance metric. No solver or ROS node is run.
"""
import argparse
from collections import Counter
import csv
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from analyze_internal_slosh_pair import scalar_message
from horizon_liquid_replay import ModalState, cubic_hermite_q


LEADS = (0.1, 0.3, 0.5, 1.0, 2.0)
FIELDS = ("eta_x", "eta_x_dot", "eta_y", "eta_y_dot")
HORIZON_TOPIC = "/spmpc/debug/predicted_horizon"
BACKEND = "continuous_mpcc_acados_explicit_actuator"


def require(ok, reason):
    if not ok:
        raise ValueError(reason)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def check_identity(identity, bag):
    stat = bag.stat()
    require(identity.get("bag") == str(bag.resolve()) and
            identity.get("size") == stat.st_size and
            identity.get("mtime_ns") == stat.st_mtime_ns, "stale cache identity")


def load_horizons(bag):
    """Reuse cost cache if present; otherwise read only the missing small topic."""
    for suffix in ("_exact_cost_cache.json.gz", "_ocp_horizon_cache.json.gz"):
        cache = bag.with_name(bag.stem + suffix)
        if cache.exists():
            with gzip.open(str(cache), "rt") as stream:
                data = json.load(stream)
            check_identity(data["identity"], bag)
            require(HORIZON_TOPIC in data["identity"]["topics"], "cache lacks horizon")
            return data["topics"]["horizon"], {"path": str(cache), "hit": True}
    import rosbag  # file reader only; deliberately no rospy
    rows = []
    with rosbag.Bag(str(bag), "r") as stream:
        require(HORIZON_TOPIC in stream.get_type_and_topic_info().topics,
                "bag lacks recorded horizon")
        for _, msg, _ in stream.read_messages(topics=[HORIZON_TOPIC]):
            rows.append(scalar_message(msg))
    identity = {"schema_version": 1, "bag": str(bag.resolve()),
                "size": bag.stat().st_size, "mtime_ns": bag.stat().st_mtime_ns,
                "topics": [HORIZON_TOPIC]}
    with gzip.open(str(cache), "wt") as stream:
        json.dump({"identity": identity, "topics": {"horizon": rows}}, stream)
    return rows, {"path": str(cache), "hit": False}


class Monitor:
    """Hermite interpolation only within a continuous valid observer segment."""
    def __init__(self, rows, height_coeff, max_gap):
        self.rows, self.segments = [], []
        self.counts = Counter()
        self.height_coeff = height_coeff
        previous, segment = None, 0
        for row in rows:
            good = (row.get("valid") and row.get("configured") and
                    row.get("source") == 2 and row.get("bias_ready") and
                    row.get("filter_ready") and row.get("state_stamp", 0) > 0 and
                    all(np.isfinite(row.get(k, np.nan)) for k in (*FIELDS, "modal_height_m")))
            if not good:
                self.counts["invalid"] += 1
                previous = None
                segment += 1
                continue
            q = [row[k] for k in FIELDS]
            require(np.isclose(row["modal_height_m"], height_coeff * np.hypot(q[0], q[2]),
                               rtol=2e-6, atol=1e-10), "monitor height coefficient differs")
            if self.rows and row["state_stamp"] == self.rows[-1]["state_stamp"]:
                keys = (*FIELDS, "modal_height_m", "reset_epoch", "observer_update_count")
                require(all(row[k] == self.rows[-1][k] for k in keys),
                        "conflicting monitor duplicate")
                self.counts["identical_duplicate"] += 1
                continue
            if self.rows:
                require(row["state_stamp"] > self.rows[-1]["state_stamp"],
                        "monitor clock moved backwards")
            if previous is not None:
                if (row["reset_epoch"] != previous["reset_epoch"] or
                        row["observer_update_count"] != previous["observer_update_count"] + 1 or
                        row["state_stamp"] - previous["state_stamp"] > max_gap):
                    segment += 1
                    self.counts["continuity_break"] += 1
            self.rows.append(row)
            self.segments.append(segment)
            previous = row
        require(len(self.rows) >= 2, "insufficient valid IMU states")
        self.times = np.array([r["state_stamp"] for r in self.rows])
        self.counts["valid_unique"] = len(self.rows)
        self.counts["segments"] = len(set(self.segments))

    def sample(self, stamp):
        index = int(np.searchsorted(self.times, stamp))
        if index < len(self.times) and self.times[index] == stamp:
            row = self.rows[index]
            return ModalState(*(row[k] for k in FIELDS)), self.segments[index]
        require(0 < index < len(self.times), "monitor_extrapolation")
        require(self.segments[index-1] == self.segments[index], "monitor_gap_or_reset")
        left, right = self.rows[index-1], self.rows[index]
        q = cubic_hermite_q(ModalState(*(left[k] for k in FIELDS)),
                            ModalState(*(right[k] for k in FIELDS)),
                            stamp - self.times[index-1], self.times[index] - self.times[index-1])
        return q, self.segments[index]

    def height(self, stamp):
        q, segment = self.sample(stamp)
        return self.height_coeff * np.hypot(q.eta_x, q.eta_y), segment


def unique_cycles(rows):
    result, duplicates = {}, set()
    for row in rows:
        cycle = row["cycle_id"]
        if cycle in result:
            duplicates.add(cycle)
        result[cycle] = row
    return result, duplicates


def validate_cycle(h, s, a, coeff):
    require(h.get("valid") and s.get("valid"), "invalid_snapshot_or_horizon")
    require(h.get("slosh_enabled") and s.get("slosh_enabled") and
            not h.get("zero_liquid_initial_state") and not s.get("zero_liquid_initial_state"),
            "requires_full_liquid_state")
    require(h["schema_version"] == s["schema_version"] == 5 and
            h["backend"] == s["backend"] == BACKEND and
            h["variant"] == s["variant"] == "B_slosh" and
            h["solver_status"] == s["solver_status"] == "B_slosh_ACADOS_OK",
            "unsupported_or_failed_solver")
    require(all(a.get(k) for k in ("solve_attempted", "solve_success", "command_accepted",
                                   "command_was_published")) and a.get("observer_source") == 2 and
            not any(a.get(k) for k in ("command_contract_violation", "terminal_controller_intervened",
                                       "safety_gate_intervened")), "origin_command_not_solver")
    epoch = h["solver_input_epoch"]
    require(h["cycle_id"] == s["cycle_id"] == a["cycle_id"] and
            all(abs(epoch - row["solver_input_epoch"]) < 1e-6 for row in (s, a)),
            "cycle_or_epoch_mismatch")
    require(epoch <= h["solve_start_stamp"] <= h["solve_end_stamp"] <=
            h["horizon_available_stamp"] <= a["command_publish_stamp"], "invalid_clock_order")
    require(h["horizon_steps"] == s["horizon_steps"] == 60 and s["state_width"] == 28 and
            np.isclose(h["dt"], 1/30, atol=1e-9, rtol=0), "unsupported_horizon")
    for k in ("t", "h_modal", *FIELDS):
        require(len(h[k]) == 61 and np.all(np.isfinite(h[k])), "malformed_nodes")
    require(np.allclose(h["t"], np.arange(61) * h["dt"], rtol=0, atol=1e-8),
            "invalid_relative_node_times")
    require(all(abs(h[k][0] - s[k]) < 1e-7 for k in FIELDS), "x0_snapshot_mismatch")
    require(np.allclose(h["h_modal"], coeff*np.hypot(h["eta_x"], h["eta_y"]),
                        rtol=2e-6, atol=1e-10), "horizon_height_coefficient_differs")


def metrics(rows):
    if not rows:
        return {"count": 0}
    pred = np.array([r["predicted_mm"] for r in rows])
    obs = np.array([r["imu_mm"] for r in rows])
    diff = pred - obs
    rms = lambda x: float(np.sqrt(np.mean(x*x)))
    p, o = int(np.argmax(pred)), int(np.argmax(obs))
    result = {"count": len(rows), "bias_mm": float(np.mean(diff)),
              "mae_mm": float(np.mean(abs(diff))), "rmse_mm": rms(diff),
              "predicted_rms_mm": rms(pred), "imu_rms_mm": rms(obs),
              "rms_ratio": rms(pred)/rms(obs) if rms(obs) > 1e-12 else None,
              "correlation": float(np.corrcoef(pred, obs)[0, 1])
              if min(np.std(pred), np.std(obs)) > 1e-12 else None,
              "strongest_peak_predicted_mm": float(pred[p]),
              "strongest_peak_imu_mm": float(obs[o]),
              "strongest_peak_target_delta_sec": rows[p]["target_sec"]-rows[o]["target_sec"]}
    if "future_replans" in rows[0]:
        for name in ("future_replans", "future_interventions", "usable_lead_sec"):
            values = [r[name] for r in rows]
            result[name + "_median"] = float(np.median(values))
            result[name + "_max"] = float(max(values))
    return result


def analyze(horizons, snapshots, audits, monitor, windows, coeff):
    start, end = windows["task"]
    post_end = windows["goal_post5s"][1]
    ss, sd = unique_cycles(snapshots)
    aa, ad = unique_cycles(audits)
    hh, hd = unique_cycles(horizons)
    rejected, pair_rejected = Counter(), Counter()
    matches, initial, peaks, ledger = [], [], [], []
    future = sorted((r for r in audits if r.get("command_was_published")),
                    key=lambda r: r["command_publish_stamp"])
    times = np.array([r["command_publish_stamp"] for r in future])
    replans = np.array([bool(r.get("solve_success") and r.get("command_accepted")) for r in future])
    interventions = np.array([bool(r.get("terminal_controller_intervened") or
                                  r.get("safety_gate_intervened") or
                                  r.get("command_contract_violation")) for r in future])
    for cycle, h in sorted(hh.items()):
        epoch = h.get("solver_input_epoch", 0)
        if not start <= epoch < end:
            rejected["origin_outside_task"] += 1
            continue
        try:
            require(cycle not in sd | ad | hd, "duplicate_cycle")
            require(cycle in ss and cycle in aa, "missing_snapshot_or_audit")
            validate_cycle(h, ss[cycle], aa[cycle], coeff)
            h0, origin_segment = monitor.height(epoch)
        except ValueError as exc:
            rejected[str(exc)] += 1
            ledger.append({"cycle_id": cycle, "reason": str(exc)})
            continue
        initial.append({"cycle_id": cycle, "target_sec": epoch-start,
                        "predicted_mm": h["h_modal"][0]*1000, "imu_mm": h0*1000})
        origin = aa[cycle]["command_publish_stamp"]
        available = h["horizon_available_stamp"]
        for lead in LEADS:
            k = int(round(lead/h["dt"]))
            target = epoch + h["t"][k]
            try:
                require(target > max(available, origin), "target_not_future_when_available")
                require(target <= post_end, "target_outside_windows")
                obs, segment = monitor.height(target)
                require(segment == origin_segment, "future_crosses_monitor_gap_or_reset")
            except ValueError as exc:
                pair_rejected[str(exc)] += 1
                ledger.append({"cycle_id": cycle, "lead_sec": lead, "reason": str(exc)})
                continue
            lo, hi = np.searchsorted(times, [origin, target], side="right")
            matches.append({"cycle_id": cycle, "lead_sec": lead, "origin_epoch": epoch,
                            "target_epoch": target, "target_sec": target-start,
                            "window": "task" if target <= end else "cross_goal",
                            "usable_lead_sec": target-available,
                            "predicted_mm": h["h_modal"][k]*1000, "imu_mm": obs*1000,
                            "error_mm": (h["h_modal"][k]-obs)*1000,
                            "future_replans": int(sum(replans[lo:hi])),
                            "future_interventions": int(sum(interventions[lo:hi]))})
        # Peak timing in each complete, task-only 2 s forecast on identical node times.
        if epoch + h["t"][-1] <= end:
            grid = [(epoch+t, height) for t, height in zip(h["t"][1:], h["h_modal"][1:])
                    if epoch+t > max(available, origin)]
            try:
                actual = [monitor.height(t) for t, _ in grid]
                require(all(segment == origin_segment for _, segment in actual), "peak_gap")
                pred = np.array([v for _, v in grid]) * 1000
                obs = np.array([v for v, _ in actual]) * 1000
                ip, io = int(np.argmax(pred)), int(np.argmax(obs))
                peaks.append({"cycle_id": cycle, "origin_sec": epoch-start,
                              "predicted_peak_mm": float(pred[ip]), "imu_peak_mm": float(obs[io]),
                              "peak_delta_sec": grid[ip][0]-grid[io][0]})
            except ValueError:
                pair_rejected["peak_window_gap"] += 1
    groups = {window: {str(lead): metrics([r for r in matches if r["lead_sec"] == lead and
                                         r["window"] == window]) for lead in LEADS}
              for window in ("task", "cross_goal")}
    common = set.intersection(*[{r["cycle_id"] for r in matches if r["lead_sec"] == lead and
                                 r["window"] == "task"} for lead in LEADS])
    groups["task_common_origins"] = {str(lead): metrics([r for r in matches if
                                    r["cycle_id"] in common and r["lead_sec"] == lead])
                                    for lead in LEADS}
    intervened = {r["cycle_id"] for r in matches if r["future_interventions"] > 0}
    groups["task_common_origins_no_intervention"] = {
        str(lead): metrics([r for r in matches if r["cycle_id"] in common-intervened and
                           r["lead_sec"] == lead]) for lead in LEADS}
    shifted = [r for r in audits if r.get("previous_shifted_plan_available") and
               r.get("solve_success") and r.get("command_was_published") and
               start <= r["command_publish_stamp"] <= end]
    shifted_stats = {"count": len(shifted)}
    for key in ("replanned_minus_shifted_a", "replanned_minus_shifted_alpha"):
        values = np.array([r[key] for r in shifted])
        if len(values):
            shifted_stats[key] = {"rms": float(np.sqrt(np.mean(values**2))),
                                  "absolute_p95": float(np.percentile(abs(values), 95))}
    peak_stats = {"count": len(peaks)}
    if peaks:
        delta = np.array([r["peak_delta_sec"] for r in peaks])
        peak_stats.update(absolute_time_delta_median_sec=float(np.median(abs(delta))),
                          signed_time_delta_median_sec=float(np.median(delta)),
                          absolute_time_delta_p90_sec=float(np.percentile(abs(delta), 90)),
                          median_peak_ratio=float(np.median([r["predicted_peak_mm"]/r["imu_peak_mm"]
                                                             for r in peaks if r["imu_peak_mm"] > 1e-12])))
    report = {"groups": groups, "initial_alignment_only": metrics(initial),
              "complete_2s_peak_windows": peak_stats, "replanning": shifted_stats,
              "horizon_count": len(horizons), "accepted_origin_count": len(initial),
              "rejected_origins": dict(rejected), "rejected_pairs": dict(pair_rejected),
              "monitor": dict(monitor.counts), "rejections": ledger}
    return report, matches, peaks


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot(output, matches, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(LEADS), 1, figsize=(12, 10), sharex=True)
    for ax, lead in zip(axes, LEADS):
        rows = sorted((r for r in matches if r["lead_sec"] == lead and r["window"] == "task"),
                      key=lambda r: r["target_sec"])
        ax.plot([r["target_sec"] for r in rows], [r["imu_mm"] for r in rows],
                label="Future IMU-driven model", color="black", lw=1)
        ax.plot([r["target_sec"] for r in rows], [r["predicted_mm"] for r in rows],
                label="Recorded OCP forecast", color="tab:blue", lw=1, alpha=.85)
        ax.set_ylabel(f"{lead:g} s lead\nheight (mm)")
        ax.grid(alpha=.2)
    axes[0].legend(loc="upper right", ncol=2)
    axes[0].set_title(title + " | target-time alignment; model reference, not surface truth")
    axes[-1].set_xlabel("Target time since task start (s)")
    fig.tight_layout()
    fig.savefig(str(output / "forecast_alignment.png"), dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True, help="existing _internal_slosh.json")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    internal = json.loads(args.report.read_text())
    require(internal.get("status") == "PASS", "requires quality-passed source report")
    require(internal["prereg"]["condition"] == "full" and
            internal["prereg"]["observer"] == "processed_imu", "requires Full with IMU state")
    bag = Path(internal["bag"]).resolve()
    cache = Path(internal["cache"]["path"])
    cached = json.loads(cache.read_text())
    check_identity(cached["identity"], bag)
    topics = {k: [r["value"] for r in v] for k, v in cached["topics"].items()}
    launch = Path(internal["launch_params_file"])
    require(sha256(launch) == internal["launch_params_sha256"], "launch identity mismatch")
    params = yaml.safe_load(launch.read_text())
    href = params["/spmpc_local_planner/slosh/slosh_height_ref"]
    snapshots = [s for s in topics["snapshot"] if s.get("valid") and s.get("slosh_enabled")]
    require(bool(snapshots), "no valid liquid snapshots")
    coefficients = [max(1e-4, href)/dict(zip(s["parameter_names"], s["stage_parameters"]))["eta_ref"]
                    for s in snapshots]
    coeff = coefficients[0]
    require(np.allclose(coefficients, coeff, rtol=1e-9, atol=0), "changing height scale")
    max_gap = float(internal["prereg"]["max_interpolation_gap_sec"])
    monitor = Monitor(topics["imu"], coeff, max_gap)
    horizons, horizon_cache = load_horizons(bag)
    report, matches, peaks = analyze(horizons, topics["snapshot"], topics["audit"],
                                      monitor, internal["windows"], coeff)
    require(bool(matches), "no valid forecast matches")
    report.update(schema_version=1, bag=str(bag), trial_id=internal["prereg"]["trial_id"],
                  prereg=internal["prereg"], windows=internal["windows"],
                  source_report=str(args.report.resolve()), source_report_sha256=sha256(args.report),
                  internal_cache=str(cache), internal_cache_sha256=sha256(cache),
                  horizon_cache=horizon_cache, horizon_cache_sha256=sha256(horizon_cache["path"]),
                  height_coeff=coeff, max_interpolation_gap_sec=max_gap,
                  script_sha256=sha256(__file__),
                  dependencies_sha256={name: sha256(Path(__file__).with_name(name)) for name in
                                       ("horizon_liquid_replay.py", "analyze_internal_slosh_pair.py")},
                  method={"time": "solver_input_epoch + recorded t[k]; never bag arrival/header",
                          "reference": "Hermite eta/eta_dot at state_stamp, then c_h*hypot(eta_x,eta_y)",
                          "leads_sec": LEADS, "time_shift_fitted": False,
                          "scope": "Full forecast consistency against future IMU-driven model",
                          "limits": "overlapping rolling forecasts, subsequent replanning; not independent samples or liquid truth",
                          "peak": "strongest maxima on the same sampled window; not matched physical wave crests or fitted delay"})
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    write_csv(args.output / "matches.csv", matches)
    write_csv(args.output / "peak_windows.csv", peaks)
    plot(args.output, matches, report["trial_id"])
    print(json.dumps({"trial": report["trial_id"], "origins": report["accepted_origin_count"],
                      "common": report["groups"]["task_common_origins"],
                      "rejected": report["rejected_origins"]}))


if __name__ == "__main__":
    main()
