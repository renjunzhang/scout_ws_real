#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Step 2 verdict aggregator.

Reads `oscrs_model_consistency_*.csv` and `oscrs_candidates_*.csv` produced by
`check_oscrs_model_consistency.py` and `analyze_oscrs_candidates.py`, then
applies the two-layer judgement of §3.4 of `2026-05-07_OSCRS具体执行方案.md`:

  Layer 1 (model consistency, FAIL fast):
    > 30% rows with spearman_rho_pred_vs_obs < rho_min  -> FAIL fast.

  Layer 2 (candidate feasibility, three branches):
    Per (bag, eta_lim_mm):
      (a) >=1 non-original candidate with geom_pass & height_pass & residual_pass
      (b) min h_p95_pred among (a)-candidates < geometry-only baseline h_p95_pred
          by improvement_min (default 5%)
      (c) >=2 of eta_lim in {20, 25, 30} satisfy (a) and (b)
    Bag passes layer-2 iff (c) holds. Scenario verdict per §3.4.

Output: markdown report + per-bag CSV summary.
"""

import argparse
import csv
import os
import re
import statistics
import sys
from collections import OrderedDict, defaultdict
from pathlib import Path


SCENARIOS = (
    ("open_user_goal", re.compile(r"open_user_goal")),
    ("open_goal_b", re.compile(r"open_goal_b(?!cd)")),
    ("open_goal_c", re.compile(r"open_goal_c")),
    ("open_goal_d", re.compile(r"open_goal_d")),
    ("open_long_path", re.compile(r"open_long_path")),
    ("open_with_obstacle_pair", re.compile(r"open_with_obstacle_pair")),
)
CONDITIONS = ("GEOREF_TUNED", "RAW_TUNED", "GEOREF_OSCRS_SHADOW", "GEOREF_OSCRS_ACTIVE",
              "GEOREF_SLOSH_SCORE", "RAW_SLOW", "GEOREF_ORIGINAL")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--consistency", nargs="+", required=True,
                        help="oscrs_model_consistency_*.csv files")
    parser.add_argument("--candidates", nargs="+", required=True,
                        help="oscrs_candidates_*.csv files")
    parser.add_argument("--condition", default="GEOREF_TUNED",
                        help="Primary condition for layer-2 verdict (default GEOREF_TUNED)")
    parser.add_argument("--rho-min", type=float, default=0.5,
                        help="Spearman rho threshold (§1.5 default 0.5)")
    parser.add_argument("--rho-fail-rate", type=float, default=0.30,
                        help="Layer-1 FAIL fast threshold for fraction of rows below rho-min")
    parser.add_argument("--improvement-min", type=float, default=0.05,
                        help="Required relative h_p95_pred improvement vs baseline (default 5%%)")
    parser.add_argument("--eta-lim-set", default="20,25,30",
                        help="Comma list of eta_lim_mm to apply criterion (c)")
    parser.add_argument("--min-eta-passes", type=int, default=2,
                        help="Criterion (c): minimum eta_lim values satisfying (a)(b)")
    parser.add_argument("--user-goal-min-pass", type=int, default=4,
                        help="open_user_goal pass threshold (§3.4 default 4 of 5)")
    parser.add_argument("--user-goal-min-bags", type=int, default=5,
                        help="open_user_goal expected bag count (§5.1 default 5)")
    parser.add_argument("--open-b-min-pass", type=int, default=2,
                        help="open_goal_b pass threshold (§3.4 default 2)")
    parser.add_argument("--report-md", default="docs/Claude/分析数据/oscrs_step2_verdict.md",
                        help="Output markdown report path")
    parser.add_argument("--report-csv", default="docs/Claude/分析数据/oscrs_step2_verdict.csv",
                        help="Per-bag CSV summary path")
    return parser.parse_args()


def detect_scenario(bag_path):
    name = os.path.basename(bag_path)
    for label, pattern in SCENARIOS:
        if pattern.search(name):
            return label
    return "unknown"


def detect_condition(bag_path):
    name = os.path.basename(bag_path)
    for cond in CONDITIONS:
        if cond in name:
            return cond
    return "UNKNOWN"


def read_csv(paths):
    rows = []
    for path in paths:
        if not os.path.isfile(path):
            sys.stderr.write(f"warn: missing csv {path}\n")
            continue
        with open(path, "r", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                row["_source"] = path
                rows.append(row)
    return rows


def parse_float(value, default=float("nan")):
    try:
        if value is None or value == "":
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def parse_int(value, default=0):
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def layer1(consistency_rows, rho_min, fail_rate_max):
    by_scenario = defaultdict(list)
    for row in consistency_rows:
        scenario = detect_scenario(row.get("bag", ""))
        condition = detect_condition(row.get("bag", ""))
        rho = parse_float(row.get("spearman_rho_pred_vs_obs"))
        by_scenario[scenario].append({
            "bag": os.path.basename(row.get("bag", "")),
            "condition": condition,
            "rho": rho,
            "below": rho < rho_min if not _isnan(rho) else True,
        })
    summary = OrderedDict()
    overall_total = 0
    overall_below = 0
    for scenario, entries in by_scenario.items():
        rhos = [e["rho"] for e in entries if not _isnan(e["rho"])]
        below = sum(1 for e in entries if e["below"])
        summary[scenario] = {
            "rows": entries,
            "median_rho": statistics.median(rhos) if rhos else float("nan"),
            "min_rho": min(rhos) if rhos else float("nan"),
            "below_count": below,
            "total": len(entries),
        }
        overall_total += len(entries)
        overall_below += below
    fail_fast = overall_total > 0 and overall_below / overall_total > fail_rate_max
    return summary, {
        "overall_total": overall_total,
        "overall_below": overall_below,
        "fail_rate": (overall_below / overall_total) if overall_total else 0.0,
        "fail_fast": fail_fast,
        "rho_min": rho_min,
        "fail_rate_max": fail_rate_max,
    }


def _isnan(x):
    return x != x


def index_candidates(candidate_rows):
    by_bag_eta = defaultdict(list)
    for row in candidate_rows:
        bag = row.get("bag", "")
        eta = parse_float(row.get("eta_lim_mm"))
        by_bag_eta[(bag, eta)].append(row)
    return by_bag_eta


def evaluate_bag_eta(rows, improvement_min):
    """Return dict with (a),(b) flags + diagnostic numbers for one bag at one eta_lim."""
    baseline_h_p95 = float("nan")
    baseline_name = ""
    for row in rows:
        if parse_int(row.get("selected_by_existing")) == 1:
            baseline_h_p95 = parse_float(row.get("h_p95_pred"))
            baseline_name = row.get("candidate", "")
            break
    if _isnan(baseline_h_p95):
        for row in rows:
            if row.get("candidate") == "original":
                baseline_h_p95 = parse_float(row.get("h_p95_pred"))
                baseline_name = "original_fallback"
                break

    feasible = []
    for row in rows:
        if row.get("candidate") == "original":
            continue
        if parse_int(row.get("geom_pass")) != 1:
            continue
        if parse_int(row.get("height_pass")) != 1:
            continue
        if parse_int(row.get("residual_pass")) != 1:
            continue
        feasible.append(row)
    a_pass = len(feasible) > 0

    best = None
    best_h_p95 = float("inf")
    for row in feasible:
        h = parse_float(row.get("h_p95_pred"))
        if not _isnan(h) and h < best_h_p95:
            best_h_p95 = h
            best = row
    b_pass = False
    rel_improvement = float("nan")
    if a_pass and not _isnan(baseline_h_p95) and baseline_h_p95 > 1e-12:
        rel_improvement = (baseline_h_p95 - best_h_p95) / baseline_h_p95
        b_pass = rel_improvement >= improvement_min

    return {
        "a_pass": a_pass,
        "b_pass": b_pass,
        "baseline_candidate": baseline_name,
        "baseline_h_p95": baseline_h_p95,
        "best_candidate": best.get("candidate") if best else "",
        "best_h_p95": best_h_p95 if best else float("nan"),
        "rel_improvement": rel_improvement,
        "feasible_count": len(feasible),
    }


def layer2(candidate_rows, condition, eta_set, min_eta_passes, improvement_min,
           user_goal_min_pass, open_b_min_pass, user_goal_min_bags):
    bags_by_scenario = defaultdict(OrderedDict)
    indexed = index_candidates(candidate_rows)
    for (bag, eta), rows in indexed.items():
        cond = detect_condition(bag)
        if cond != condition:
            continue
        scenario = detect_scenario(bag)
        if scenario == "unknown":
            continue
        if eta not in eta_set:
            continue
        result = evaluate_bag_eta(rows, improvement_min)
        bags_by_scenario[scenario].setdefault(bag, {})[eta] = result

    scenario_summaries = OrderedDict()
    for scenario, bag_map in bags_by_scenario.items():
        bag_summary = []
        for bag, eta_results in bag_map.items():
            ab_eta_count = sum(1 for r in eta_results.values() if r["a_pass"] and r["b_pass"])
            a_only_eta_count = sum(1 for r in eta_results.values() if r["a_pass"])
            c_pass = ab_eta_count >= min_eta_passes
            ab30_only = (
                eta_results.get(30.0, {}).get("a_pass", False)
                and eta_results.get(30.0, {}).get("b_pass", False)
                and ab_eta_count == 1
            )
            bag_summary.append({
                "bag": os.path.basename(bag),
                "ab_eta_count": ab_eta_count,
                "a_only_eta_count": a_only_eta_count,
                "c_pass": c_pass,
                "ab30_only": ab30_only,
                "details": eta_results,
            })
        scenario_summaries[scenario] = bag_summary

    user_goal = scenario_summaries.get("open_user_goal", [])
    open_b = scenario_summaries.get("open_goal_b", [])
    scenario_rollup = OrderedDict()
    for scenario, bags in scenario_summaries.items():
        scenario_rollup[scenario] = {
            "total": len(bags),
            "pass_bags": sum(1 for b in bags if b["c_pass"]),
            "a_bags": sum(1 for b in bags if b["a_only_eta_count"] >= 1),
            "ab30_only_bags": sum(1 for b in bags if b["ab30_only"]),
        }

    user_goal_pass_bags = sum(1 for b in user_goal if b["c_pass"])
    user_goal_a_bags = sum(1 for b in user_goal if b["a_only_eta_count"] >= 1)
    user_goal_ab30_only_bags = sum(1 for b in user_goal if b["ab30_only"])
    open_b_pass_bags = sum(1 for b in open_b if b["c_pass"] or b["a_only_eta_count"] >= 1)

    user_goal_data_complete = len(user_goal) >= user_goal_min_bags
    open_b_data_complete = len(open_b) >= open_b_min_pass

    pass_user_goal = user_goal_pass_bags >= user_goal_min_pass
    pass_open_b = open_b_pass_bags >= open_b_min_pass

    if pass_user_goal and pass_open_b and user_goal_data_complete and open_b_data_complete:
        verdict = "PASS"
    elif user_goal_a_bags >= user_goal_min_pass and not pass_user_goal:
        verdict = "SATURATED_NO_IMPROVEMENT"
    elif user_goal_ab30_only_bags >= user_goal_min_pass:
        verdict = "SATURATED_ETA30_ONLY"
    elif user_goal_a_bags <= max(0, len(user_goal) - user_goal_min_pass):
        verdict = "FAIL"
    else:
        verdict = "INCONCLUSIVE"

    return {
        "scenarios": scenario_summaries,
        "user_goal_pass_bags": user_goal_pass_bags,
        "user_goal_a_bags": user_goal_a_bags,
        "user_goal_ab30_only_bags": user_goal_ab30_only_bags,
        "user_goal_total": len(user_goal),
        "user_goal_data_complete": user_goal_data_complete,
        "open_b_pass_bags": open_b_pass_bags,
        "open_b_total": len(open_b),
        "open_b_data_complete": open_b_data_complete,
        "scenario_rollup": scenario_rollup,
        "verdict": verdict,
        "min_eta_passes": min_eta_passes,
        "improvement_min": improvement_min,
        "condition": condition,
        "eta_set": eta_set,
    }


def fmt(value, spec=".3f"):
    if value is None:
        return "-"
    if isinstance(value, float) and (_isnan(value) or value in (float("inf"), float("-inf"))):
        return "-"
    return format(value, spec)


def write_csv_summary(path, layer2_result):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fields = ["scenario", "bag", "eta_lim_mm", "a_pass", "b_pass",
              "feasible_count", "baseline_candidate", "baseline_h_p95",
              "best_candidate", "best_h_p95", "rel_improvement"]
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for scenario, bags in layer2_result["scenarios"].items():
            for bag in bags:
                for eta, detail in sorted(bag["details"].items()):
                    writer.writerow({
                        "scenario": scenario,
                        "bag": bag["bag"],
                        "eta_lim_mm": eta,
                        "a_pass": int(detail["a_pass"]),
                        "b_pass": int(detail["b_pass"]),
                        "feasible_count": detail["feasible_count"],
                        "baseline_candidate": detail["baseline_candidate"],
                        "baseline_h_p95": fmt(detail["baseline_h_p95"], ".6g"),
                        "best_candidate": detail["best_candidate"],
                        "best_h_p95": fmt(detail["best_h_p95"], ".6g"),
                        "rel_improvement": fmt(detail["rel_improvement"], ".4f"),
                    })


def render_report(layer1_summary, layer1_meta, layer2_result, args):
    lines = []
    lines.append("# OSCRS Step 2 verdict")
    lines.append("")
    lines.append(f"- Condition: `{layer2_result['condition']}`")
    lines.append(f"- eta_lim set: {sorted(layer2_result['eta_set'])} (min eta passes for (c) = {layer2_result['min_eta_passes']})")
    lines.append(f"- improvement_min: {layer2_result['improvement_min']:.0%}")
    lines.append(f"- rho_min: {layer1_meta['rho_min']:.2f}, layer-1 fail-fast threshold: > {layer1_meta['fail_rate_max']:.0%} rows below")
    lines.append("")

    lines.append("## Layer 1 — model consistency (§1.5)")
    lines.append("")
    lines.append("| scenario | bags | median rho | min rho | rows below rho_min |")
    lines.append("|---|---|---|---|---|")
    for scenario, info in layer1_summary.items():
        lines.append("| {} | {} | {} | {} | {}/{} |".format(
            scenario, info["total"], fmt(info["median_rho"]), fmt(info["min_rho"]),
            info["below_count"], info["total"]))
    lines.append("")
    lines.append("Per-bag rho:")
    lines.append("")
    lines.append("| scenario | bag | condition | rho | pass |")
    lines.append("|---|---|---|---|---|")
    for scenario, info in layer1_summary.items():
        for entry in info["rows"]:
            lines.append("| {} | {} | {} | {} | {} |".format(
                scenario, entry["bag"], entry["condition"], fmt(entry["rho"]),
                "no" if entry["below"] else "yes"))
    lines.append("")
    lines.append("Layer-1 verdict: **{}** (overall {}/{} below rho_min, fail rate {:.1%})".format(
        "FAIL_FAST" if layer1_meta["fail_fast"] else "PASS",
        layer1_meta["overall_below"], layer1_meta["overall_total"], layer1_meta["fail_rate"]))
    lines.append("")

    if layer1_meta["fail_fast"]:
        lines.append("> Layer-1 FAIL_FAST: skip layer-2 and revisit §1.5 calibration.")
        lines.append("")
        return "\n".join(lines)

    lines.append("## Layer 2 — candidate feasibility (§3.4)")
    lines.append("")
    for scenario, bags in layer2_result["scenarios"].items():
        lines.append(f"### {scenario}  ({len(bags)} bag{'s' if len(bags)!=1 else ''})")
        lines.append("")
        lines.append("| bag | (a) eta count | (a)+(b) eta count | (c) pass | eta30-only |")
        lines.append("|---|---|---|---|---|")
        for bag in bags:
            lines.append("| {} | {} | {} | {} | {} |".format(
                bag["bag"], bag["a_only_eta_count"], bag["ab_eta_count"],
                "yes" if bag["c_pass"] else "no",
                "yes" if bag["ab30_only"] else "no"))
        lines.append("")
        lines.append("Per-bag x eta detail:")
        lines.append("")
        lines.append("| bag | eta_lim_mm | feasible | baseline cand | baseline h_p95 | best cand | best h_p95 | rel improvement | (a) | (b) |")
        lines.append("|---|---|---|---|---|---|---|---|---|---|")
        for bag in bags:
            for eta, detail in sorted(bag["details"].items()):
                lines.append("| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |".format(
                    bag["bag"], int(eta), detail["feasible_count"],
                    detail["baseline_candidate"], fmt(detail["baseline_h_p95"], ".6g"),
                    detail["best_candidate"] or "-", fmt(detail["best_h_p95"], ".6g"),
                    fmt(detail["rel_improvement"], ".1%") if not _isnan(detail["rel_improvement"]) else "-",
                    "yes" if detail["a_pass"] else "no",
                    "yes" if detail["b_pass"] else "no"))
        lines.append("")

    lines.append("## Aggregate")
    lines.append("")
    lines.append("Scenario rollup:")
    lines.append("")
    lines.append("| scenario | bags | pass (c) | only (a) | eta30-only |")
    lines.append("|---|---|---|---|---|")
    for scenario, info in layer2_result.get("scenario_rollup", {}).items():
        lines.append("| {} | {} | {} | {} | {} |".format(
            scenario, info["total"], info["pass_bags"], info["a_bags"],
            info["ab30_only_bags"]))
    lines.append("")
    lines.append("Legacy §3.4 gates:")
    lines.append("")
    lines.append("- open_user_goal bags passing (c): {}/{} (need >= {})".format(
        layer2_result["user_goal_pass_bags"], layer2_result["user_goal_total"],
        args.user_goal_min_pass))
    lines.append("- open_user_goal bags satisfying only (a): {}/{}".format(
        layer2_result["user_goal_a_bags"], layer2_result["user_goal_total"]))
    lines.append("- open_user_goal bags whose (a)+(b) hold only at eta=30: {}/{}".format(
        layer2_result["user_goal_ab30_only_bags"], layer2_result["user_goal_total"]))
    lines.append("- open_goal_b bags with at least (a) or (c): {}/{} (need >= {})".format(
        layer2_result["open_b_pass_bags"], layer2_result["open_b_total"],
        args.open_b_min_pass))
    lines.append("- open_user_goal data complete (>= {} bags): {}".format(
        args.user_goal_min_bags, layer2_result["user_goal_data_complete"]))
    lines.append("- open_goal_b data complete (>= {} bags): {}".format(
        args.open_b_min_pass, layer2_result["open_b_data_complete"]))
    lines.append("")
    lines.append("**Verdict: {}**".format(layer2_result["verdict"]))
    lines.append("")
    lines.append("## Next-step actions per §3.4")
    lines.append("")
    actions = {
        "PASS": [
            "Lock the eta_lim values that satisfied (b) into `oscrs_container.yaml` and launch defaults.",
            "Run GEOREF_OSCRS_SHADOW x1 smoke and confirm shadow does not change GEOREF_TUNED behavior.",
            "Run GEOREF_OSCRS_ACTIVE x1 smoke; verify fallback path on a marginal bag.",
            "Run GEOREF_OSCRS_ACTIVE x5 closed-loop sims (Step 4 / §5).",
            "Open Step 4 evaluation table vs RAW x5 / GEOREF_TUNED x5 / SLOSH_SCORE x3.",
        ],
        "SATURATED_NO_IMPROVEMENT": [
            "Reorder paper contributions: C1 (eta_lim safety) primary, C3 (failure-informed) secondary.",
            "Extend simulation to >=6 open scenarios (long_path / obstacle_pair).",
            "Develop Step 1 only after GEOREF_OSCRS_SHADOW confirms no false-trigger.",
            "Keep GEOREF_OSCRS_ACTIVE as parallel line, not replacement of GEOREF_TUNED.",
            "Downgrade venue assessment (IROS / ICRA viable; RA-L weakened).",
        ],
        "SATURATED_ETA30_ONLY": [
            "Document that OSCRS only beats geometry-only at the loosest eta_lim=30 mm gate.",
            "Run an ablation showing eta_lim sensitivity (15/20/25/30) in the paper.",
            "Re-tune candidate generator (mild/medium/strong) to push more candidates below eta=25.",
            "Treat current data as preliminary; rerun Step 0 after generator tuning.",
        ],
        "FAIL": [
            "Inspect candidate generator diversity (kappa coverage of mild/medium/strong).",
            "Try Ferrari eq. (3) zeta_physics vs default 0.05 as ablation.",
            "If zeta correction does not lift rho, re-check observer height_coeff branch.",
            "Consider width-inflated B-spline candidates to enrich generator output.",
            "If SATURATED is unreachable within 6 weeks, demote OSCRS to offline analysis.",
        ],
        "INCONCLUSIVE": [
            "Data shape does not fit any branch cleanly. Re-verify --condition and --eta-lim-set.",
            "Confirm bag count meets §5.1 (5 bags for open_user_goal, >=2 for open_goal_b).",
        ],
    }
    for line in actions.get(layer2_result["verdict"], ["(no action set)"]):
        lines.append(f"- {line}")
    lines.append("")
    return "\n".join(lines)


def main():
    args = parse_args()
    eta_set = set(float(x.strip()) for x in args.eta_lim_set.split(",") if x.strip())
    consistency_rows = read_csv(args.consistency)
    candidate_rows = read_csv(args.candidates)
    if not consistency_rows:
        sys.stderr.write("error: no consistency rows loaded\n")
        sys.exit(2)
    if not candidate_rows:
        sys.stderr.write("error: no candidate rows loaded\n")
        sys.exit(2)
    layer1_summary, layer1_meta = layer1(consistency_rows, args.rho_min, args.rho_fail_rate)
    if layer1_meta["fail_fast"]:
        layer2_result = {
            "scenarios": OrderedDict(),
            "user_goal_pass_bags": 0,
            "user_goal_a_bags": 0,
            "user_goal_ab30_only_bags": 0,
            "user_goal_total": 0,
            "user_goal_data_complete": False,
            "open_b_pass_bags": 0,
            "open_b_total": 0,
            "open_b_data_complete": False,
            "scenario_rollup": OrderedDict(),
            "verdict": "LAYER1_FAIL_FAST",
            "min_eta_passes": args.min_eta_passes,
            "improvement_min": args.improvement_min,
            "condition": args.condition,
            "eta_set": eta_set,
        }
    else:
        layer2_result = layer2(
            candidate_rows, args.condition, eta_set, args.min_eta_passes,
            args.improvement_min, args.user_goal_min_pass, args.open_b_min_pass,
            args.user_goal_min_bags,
        )
    report = render_report(layer1_summary, layer1_meta, layer2_result, args)
    os.makedirs(os.path.dirname(args.report_md) or ".", exist_ok=True)
    with open(args.report_md, "w", encoding="utf-8") as handle:
        handle.write(report)
    write_csv_summary(args.report_csv, layer2_result)
    print(report)
    print(f"\nreport: {args.report_md}\nper-bag csv: {args.report_csv}")


if __name__ == "__main__":
    main()
