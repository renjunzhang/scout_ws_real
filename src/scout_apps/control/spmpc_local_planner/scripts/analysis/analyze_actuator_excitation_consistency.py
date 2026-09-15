#!/usr/bin/env python3
"""Compare recorded OCP, actual-command replay and processed IMU excitation.

Reads an existing quality-passed Full/IMU internal-slosh report and its caches.
Only a missing horizon cache requires rosbag; no ROS node or command is sent.
Produces JSON, native-sample CSV and a three-curve PNG. Does not fit parameters.
"""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np
import yaml

from actuator_excitation_consistency_core import AXES, ImuProcessing, analyze
from analyze_ocp_imu_forecast import check_identity, load_horizons, require, sha256


def load_inputs(source):
    internal = json.loads(source.read_text())
    require(internal.get("status") == "PASS", "requires a quality-passed internal-slosh report")
    require(internal["prereg"].get("condition") == "full" and
            internal["prereg"].get("observer") == "processed_imu", "requires Full with IMU source")
    bag = Path(internal["bag"]).resolve()
    cache = Path(internal["cache"]["path"])
    cached = json.loads(cache.read_text())
    check_identity(cached["identity"], bag)
    topics = {key: [r["value"] for r in values] for key, values in cached["topics"].items()}
    launch = Path(internal["launch_params_file"])
    require(sha256(launch) == internal["launch_params_sha256"], "launch identity mismatch")
    params = yaml.safe_load(launch.read_text())
    processing = ImuProcessing.from_launch(params)
    snapshots = [s for s in topics["snapshot"] if s.get("valid") and s.get("slosh_enabled")]
    require(bool(snapshots), "no valid Full snapshots")
    href = float(params["/spmpc_local_planner/slosh/slosh_height_ref"])
    coefficients = [max(1e-4, href)/dict(zip(s["parameter_names"], s["stage_parameters"]))["eta_ref"]
                    for s in snapshots]
    require(np.isfinite(coefficients).all() and min(coefficients) > 0 and
            np.allclose(coefficients, coefficients[0], rtol=1e-9, atol=0), "changing/invalid height scale")
    horizons, horizon_cache = load_horizons(bag)
    identity = dict(bag=str(bag), source_report=str(source.resolve()), source_report_sha256=sha256(source),
                    internal_cache=str(cache), internal_cache_sha256=sha256(cache),
                    launch_params_file=str(launch), launch_params_sha256=sha256(launch),
                    horizon_cache=horizon_cache, horizon_cache_sha256=sha256(horizon_cache["path"]),
                    windows=internal["windows"], prereg=internal["prereg"])
    return horizons, topics, processing, internal["windows"], coefficients[0], identity


def plot(output, rows):
    if not rows:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    first = min(rows, key=lambda r: r["solver_input_epoch"])["cycle_id"]
    selected = [r for r in rows if r["cycle_id"] == first]
    epoch = selected[0]["solver_input_epoch"]
    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    for ax, name in zip(axes, AXES):
        stamp = ("accel" if name in ("ax", "ay") else "gyro" if name == "omega" else "alpha")+"_effective_stamp"
        time = [r[stamp]-epoch for r in selected]
        for key, label, color in (("ocp", "Recorded plan + IMU operator", "tab:blue"),
                                  ("replay", "Published-command replay + IMU operator", "tab:orange"),
                                  ("imu", "Processed IMU", "black")):
            ax.plot(time, [r[key+"_"+name] for r in selected], label=label, color=color, lw=1.2)
        ax.set_ylabel(name + (" (m/s2)" if name in ("ax", "ay") else " (rad/s)" if name == "omega" else " (rad/s2)"))
        ax.grid(alpha=.25)
    axes[0].legend(fontsize=8)
    axes[0].set_title("First accepted origin, cycle {} | nominal geometry; no parameter fitting".format(first))
    axes[-1].set_xlabel("Effective time relative to OCP epoch (s)")
    fig.tight_layout()
    fig.savefig(str(output / "excitation_comparison.png"), dpi=150)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True, help="existing Full _internal_slosh.json")
    parser.add_argument("--output", type=Path, required=True, help="new or empty output directory")
    parser.add_argument("--cycle-id", type=int, help="limit comparison to one recorded origin")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--min-lead", type=float, default=0.1)
    parser.add_argument("--max-imu-gap", type=float, default=0.035)
    parser.add_argument("--max-command-gap", type=float, default=0.1)
    args = parser.parse_args(argv)
    try:
        require(not args.output.exists() or (args.output.is_dir() and not any(args.output.iterdir())),
                "output directory must be new or empty")
        horizons, topics, processing, windows, coeff, identity = load_inputs(args.report)
        report, rows = analyze(horizons, topics["snapshot"], topics["audit"], topics["imu"],
                               processing, windows, coeff, args.duration, args.min_lead,
                               args.max_imu_gap, args.max_command_gap, args.cycle_id)
        report.update(schema_version=1, **identity, script_sha256=sha256(__file__),
                      dependencies_sha256={name: sha256(Path(__file__).with_name(name)) for name in
                                           ("actuator_excitation_consistency_core.py", "robot_state_prediction_core.py",
                                            "analyze_ocp_imu_forecast.py", "analyze_internal_slosh_pair.py")})
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
        if rows:
            with (args.output / "samples.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            plot(args.output, rows)
        print(json.dumps({"status": report["status"], "accepted_origins": len(report["origins"]),
                          "samples": len(rows), "rejected_origins": len(report["rejected_origins"]),
                          "output": str(args.output)}))
        return 0 if rows else 2
    except (ValueError, KeyError, TypeError, OSError, ImportError) as exc:
        print("excitation comparison: {}".format(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
