#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import os
from types import SimpleNamespace

from evaluate_anti_slosh_path_candidates import evaluate_path, load_config


def parse_float_list(text):
    return [float(item) for item in text.split(",") if item.strip()]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Sweep matched-time timing candidates for anti-slosh path geometry candidates."
    )
    parser.add_argument("--csv", required=True, help="CSV output path")
    parser.add_argument(
        "--original",
        action="append",
        required=True,
        help="original fixed path JSON; repeat for multiple paths",
    )
    parser.add_argument(
        "--candidate",
        action="append",
        required=True,
        help="candidate fixed path JSON; repeat for multiple candidates",
    )
    parser.add_argument(
        "--config",
        default="src/scout_apps/control/scout_local_planner/config/mpc_params_sim.yaml",
        help="Planner YAML used for slosh parameters",
    )
    parser.add_argument("--baseline-v-ref", type=float, default=1.2)
    parser.add_argument("--v-refs", default="1.35,1.40,1.45")
    parser.add_argument("--ramp-lengths", default="0.45,0.50,0.60,0.65")
    parser.add_argument("--max-time-ratio", type=float, default=1.15)
    parser.add_argument("--ds", type=float, default=0.05)
    parser.add_argument("--dt-max", type=float, default=0.02)
    parser.add_argument("--v-floor", type=float, default=0.15)
    return parser.parse_args()


def make_eval_args(v_ref, ramp_length, args):
    return SimpleNamespace(
        v_ref=v_ref,
        ramp_length=ramp_length,
        ds=args.ds,
        dt_max=args.dt_max,
        v_floor=args.v_floor,
    )


def delta_pct(value, baseline):
    return 100.0 * (value - baseline) / max(1e-12, baseline)


def write_csv(path, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main():
    args = parse_args()
    cfg = load_config(args.config)

    baselines = {}
    for path in args.original:
        row = evaluate_path(path, cfg, make_eval_args(args.baseline_v_ref, 0.0, args), "constant")
        baselines[row["path_id"]] = row

    rows = []
    for path in args.candidate:
        for v_ref in parse_float_list(args.v_refs):
            for ramp_length in parse_float_list(args.ramp_lengths):
                row = evaluate_path(path, cfg, make_eval_args(v_ref, ramp_length, args), "low_jerk")
                base = baselines.get(row["path_id"])
                if not base:
                    raise RuntimeError(f"No original baseline for path_id={row['path_id']}")
                out = dict(row)
                out["baseline_time_s"] = base["time_proxy_s"]
                out["time_delta_pct"] = delta_pct(row["time_proxy_s"], base["time_proxy_s"])
                out["h_delta_pct"] = delta_pct(row["h_p95"], base["h_p95"])
                out["energy_delta_pct"] = delta_pct(row["energy_p95"], base["energy_p95"])
                out["eta_dot_delta_pct"] = delta_pct(row["eta_dot_p95"], base["eta_dot_p95"])
                out["ay_delta_pct"] = delta_pct(row["ay_p95"], base["ay_p95"])
                out["pass_time"] = out["time_delta_pct"] <= (args.max_time_ratio - 1.0) * 100.0
                out["pass_slosh"] = (
                    out["h_delta_pct"] < 0.0
                    and out["energy_delta_pct"] < 0.0
                    and out["eta_dot_delta_pct"] < 0.0
                )
                out["pass_ay"] = out["ay_delta_pct"] <= 0.0
                out["pass_all"] = out["pass_time"] and out["pass_slosh"] and out["pass_ay"]
                rows.append(out)

    rows.sort(
        key=lambda item: (
            not item["pass_all"],
            item["path_id"],
            item["candidate"],
            item["jerk_max"],
            item["ax_max"],
        )
    )
    write_csv(args.csv, rows)

    for row in rows:
        if not row["pass_all"]:
            continue
        print(
            f"{row['path_id']} {row['candidate']} "
            f"v={row['v_ref']:.2f} ramp={row['ramp_length_m']:.2f} "
            f"time={row['time_delta_pct']:.1f}% "
            f"h={row['h_delta_pct']:.1f}% "
            f"E={row['energy_delta_pct']:.1f}% "
            f"eta_dot={row['eta_dot_delta_pct']:.1f}% "
            f"ay={row['ay_delta_pct']:.1f}% "
            f"ax_max={row['ax_max']:.2f} "
            f"jerk_max={row['jerk_max']:.1f}"
        )
    print(f"csv: {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
