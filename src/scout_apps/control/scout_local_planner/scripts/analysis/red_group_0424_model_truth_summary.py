#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Check 2026-04-24 red-liquid summaries for model-vs-RGB rank agreement.

This script uses existing outputs under `.tmp_red_group_compare_0424`.
It does not re-process bags and it does not evaluate OSCRS effectiveness.
"""

import argparse
import csv
import math
from pathlib import Path


METRIC_PAIRS = [
    ("peak", "height_peak_mm", "smooth_peak_mm"),
    ("p95", "height_p95_mm", "smooth_p95_mm"),
    ("rms", "height_rms_mm", "smooth_rms_mm"),
]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default=".tmp_red_group_compare_0424")
    parser.add_argument(
        "--out-dir",
        default="docs/Claude/分析数据/0424_model_truth_20260513",
    )
    parser.add_argument("--eps", type=float, default=1e-9)
    return parser.parse_args()


def read_csv(path):
    with open(path, "r", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path, rows):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys()) if rows else []
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def f(row, key):
    try:
        value = float(row.get(key, "nan"))
    except (TypeError, ValueError):
        return math.nan
    return value


def sign(value, eps):
    if not math.isfinite(value) or abs(value) <= eps:
        return 0
    return 1 if value > 0 else -1


def main():
    args = parse_args()
    root = Path(args.root)
    visual_rows = read_csv(root / "visual_metric_summary.csv")
    model_rows = read_csv(root / "slosh_internal_metric_summary.csv")

    model_by_key = {
        (row["group"], row["label"]): row
        for row in model_rows
    }
    joined = []
    for vrow in visual_rows:
        key = (vrow["group"], vrow["condition"])
        mrow = model_by_key.get(key)
        if mrow is None:
            continue
        out = {
            "group": vrow["group"],
            "block": vrow["block"],
            "condition": vrow["condition"],
            "visual_csv": vrow.get("csv", ""),
        }
        for _, model_key, visual_key in METRIC_PAIRS:
            out[model_key] = mrow.get(model_key, "")
            out[visual_key] = vrow.get(visual_key, "")
        joined.append(out)

    groups = {}
    for row in joined:
        groups.setdefault(row["group"], []).append(row)

    pair_rows = []
    summary_acc = {}
    for group, rows in sorted(groups.items()):
        for metric, model_key, visual_key in METRIC_PAIRS:
            token = (group, metric)
            summary_acc.setdefault(token, {"n": 0, "agree": 0, "disagree": 0, "ties": 0})
            for i in range(len(rows)):
                for j in range(i + 1, len(rows)):
                    a = rows[i]
                    b = rows[j]
                    dm = f(a, model_key) - f(b, model_key)
                    dv = f(a, visual_key) - f(b, visual_key)
                    sm = sign(dm, args.eps)
                    sv = sign(dv, args.eps)
                    if sm == 0 or sv == 0:
                        verdict = "tie"
                        summary_acc[token]["ties"] += 1
                    elif sm == sv:
                        verdict = "agree"
                        summary_acc[token]["agree"] += 1
                        summary_acc[token]["n"] += 1
                    else:
                        verdict = "disagree"
                        summary_acc[token]["disagree"] += 1
                        summary_acc[token]["n"] += 1
                    pair_rows.append({
                        "group": group,
                        "metric": metric,
                        "a": a["condition"],
                        "b": b["condition"],
                        "model_delta_mm": "{:.6g}".format(dm),
                        "visual_delta_mm": "{:.6g}".format(dv),
                        "verdict": verdict,
                    })

    summary_rows = []
    for (group, metric), acc in sorted(summary_acc.items()):
        n = acc["n"]
        summary_rows.append({
            "group": group,
            "metric": metric,
            "rank_pairs": n,
            "agree": acc["agree"],
            "disagree": acc["disagree"],
            "ties": acc["ties"],
            "A_rank": "{:.6g}".format(acc["agree"] / n) if n else "nan",
        })

    overall_rows = []
    for metric, _, _ in METRIC_PAIRS:
        rows = [row for row in summary_rows if row["metric"] == metric]
        agree = sum(int(row["agree"]) for row in rows)
        n = sum(int(row["rank_pairs"]) for row in rows)
        disagree = sum(int(row["disagree"]) for row in rows)
        ties = sum(int(row["ties"]) for row in rows)
        overall_rows.append({
            "metric": metric,
            "rank_pairs": n,
            "agree": agree,
            "disagree": disagree,
            "ties": ties,
            "A_rank": "{:.6g}".format(agree / n) if n else "nan",
        })

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    write_csv(out_dir / "0424_joined_model_visual_metrics.csv", joined)
    write_csv(out_dir / "0424_model_visual_pairwise_rank.csv", pair_rows)
    write_csv(out_dir / "0424_model_visual_A_rank_by_group.csv", summary_rows)
    write_csv(out_dir / "0424_model_visual_A_rank_overall.csv", overall_rows)

    md = ["# 2026-04-24 Model vs RGB Rank Agreement", ""]
    md.append("| metric | rank_pairs | agree | disagree | ties | A_rank |")
    md.append("|---|---:|---:|---:|---:|---:|")
    for row in overall_rows:
        md.append(
            "| {metric} | {rank_pairs} | {agree} | {disagree} | {ties} | {A_rank} |".format(**row)
        )
    md.extend([
        "",
        "Notes:",
        "- RGB visual metrics come from `.tmp_red_group_compare_0424/visual_metric_summary.csv`.",
        "- Model metrics come from recorded `/slosh/height` summaries in `slosh_internal_metric_summary.csv`.",
        "- This is model-fidelity evidence only; the bags used old MPC/controller variants, not OSCRS.",
    ])
    (out_dir / "0424_model_visual_A_rank_overall.md").write_text("\n".join(md) + "\n", encoding="utf-8")

    print("wrote", out_dir)
    for row in overall_rows:
        print(row)


if __name__ == "__main__":
    main()
