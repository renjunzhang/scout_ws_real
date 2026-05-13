#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate a bagging checklist for slosh-model validation against RGB truth.

The script intentionally does not drive the robot.  It prints recording
commands and operator actions so the motion remains explicit and observable.
"""

import argparse
from datetime import datetime
from pathlib import Path


DEFAULT_TOPICS = [
    "/tf",
    "/tf_static",
    "/odom",
    "/scout/odom",
    "/cmd_vel",
    "/scout/cmd_vel",
    "/slosh/ax_est",
    "/slosh/ay_est",
    "/slosh/height",
    "/anti_slosh_path/candidate_report",
    "/anti_slosh_path/safety_alarm",
    "/scout/global_path",
    "/scout/global_path_anti_slosh",
]

MANUAL_TRIALS = [
    ("static_01", "Keep the robot still for 30 s. Record RGB zero level and sensor bias."),
    ("straight_ax_low_01", "Straight line: 0 -> 0.5 m/s -> stop, then hold still for 6 s."),
    ("straight_ax_mid_01", "Straight line: 0 -> 1.0 m/s -> stop, then hold still for 6 s."),
    ("straight_ax_high_01", "Straight line: 0 -> 1.5-2.0 m/s -> stop, then hold still for 6 s."),
    ("turn_left_mid_01", "Left constant-radius turn at mid speed, then hold still for 6 s."),
    ("turn_right_mid_01", "Right constant-radius turn at mid speed, then hold still for 6 s."),
]

PATH_TRIALS = [
    ("P2_s_curve", "RAW_TUNED", "truth_p2_raw01"),
    ("P2_s_curve", "GEOREF_TUNED", "truth_p2_geo01"),
    ("P2_s_curve", "GEOREF_OSCRS_ACTIVE", "truth_p2_oscrs01"),
    ("P3_mixed", "RAW_TUNED", "truth_p3_raw01"),
    ("P3_mixed", "GEOREF_TUNED", "truth_p3_geo01"),
    ("P3_mixed", "GEOREF_OSCRS_ACTIVE", "truth_p3_oscrs01"),
]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--date", default=datetime.now().strftime("%Y%m%d"))
    parser.add_argument("--bag-root", default="/data/a/slosh_bags/real")
    parser.add_argument("--workspace", default="/home/a/scout_ws")
    parser.add_argument("--record-duration", default="0",
                        help="rosbag/run_sim duration. 0 means Ctrl+C stop.")
    parser.add_argument("--repeat", type=int, default=3,
                        help="Suggested repeat count for manual excitation trials.")
    parser.add_argument("--format", choices=("markdown", "shell"), default="markdown")
    parser.add_argument("--out", default="", help="Optional output file.")
    return parser.parse_args()


def q(text):
    return "'" + text.replace("'", "'\"'\"'") + "'"


def record_command(bag_dir, label, duration, topics):
    output = f"{bag_dir}/{label}.bag"
    base = f"rosbag record -O {q(output)} " + " ".join(topics)
    if duration not in ("0", "0.0"):
        return f"timeout {duration}s {base}"
    return base


def record_script_command(bag_dir, label):
    return (
        "cd $(rospack find scout_local_planner)\n"
        f"SLOSH_BAG_DIR={q(bag_dir)} \\\n"
        f"./scripts/record_slosh_experiment.sh 0 {label}"
    )


def run_sim_command(path_id, condition, run_id, duration):
    return (
        "PATH_MODE=replay \\\n"
        f"PATH_ID={path_id} \\\n"
        f"CONDITION={condition} \\\n"
        f"RUN_ID={run_id} \\\n"
        "START_DELAY=10 \\\n"
        f"RECORD_DURATION={duration} \\\n"
        "rosrun scout_local_planner run_sim_fixed_path_bag.sh"
    )


def markdown(args):
    bag_dir = f"{args.bag_root}/{args.date}_model_truth"
    lines = [
        "# Slosh Model Truth-Validation Bagging Protocol",
        "",
        "## Prerequisites",
        "",
        "```bash",
        f"source {args.workspace}/devel/setup.bash",
        f"mkdir -p {q(bag_dir)}",
        "```",
        "",
        "RGB visual liquid height is the ground truth.  `/slosh/height` is recorded",
        "only as a model/observer signal and must not replace RGB metrics.",
        "",
        "## Manual Real-Robot Excitation Bags",
        "",
        "Run one command at a time, start RGB recording before the command, then",
        "perform the listed operator action.  Repeat each non-static trial",
        f"{args.repeat} times with distinct suffixes.",
        "",
    ]
    for label, action in MANUAL_TRIALS:
        lines.extend([
            f"### {label}",
            "",
            f"Action: {action}",
            "",
            "```bash",
            record_script_command(bag_dir, f"modeltruth_{label}"),
            "```",
            "",
        ])
    lines.extend([
        "## Fixed-Path Strategy Comparison",
        "",
        "Use these after the single-axis checks look sane.  They test whether the",
        "model ranking agrees with RGB truth under RAW / GeoRef / OSCRS strategies.",
        "",
    ])
    for path_id, condition, run_id in PATH_TRIALS:
        lines.extend([
            f"### {path_id} {condition}",
            "",
            "```bash",
            run_sim_command(path_id, condition, run_id, args.record_duration),
            "```",
            "",
        ])
    return "\n".join(lines)


def shell(args):
    bag_dir = f"{args.bag_root}/{args.date}_model_truth"
    lines = [
        "#!/usr/bin/env bash",
        "set -euo pipefail",
        f"source {args.workspace}/devel/setup.bash",
        f"mkdir -p {q(bag_dir)}",
        "",
        "# Manual excitation commands. Run one at a time while recording RGB.",
    ]
    for label, action in MANUAL_TRIALS:
        lines.extend([
            "",
            f"# {label}: {action}",
            record_script_command(bag_dir, f"modeltruth_{label}"),
        ])
    lines.extend(["", "# Fixed-path strategy comparison commands."])
    for path_id, condition, run_id in PATH_TRIALS:
        lines.extend([
            "",
            f"# {path_id} {condition}",
            run_sim_command(path_id, condition, run_id, args.record_duration),
        ])
    return "\n".join(lines) + "\n"


def main():
    args = parse_args()
    text = markdown(args) if args.format == "markdown" else shell(args)
    if args.out:
        path = Path(args.out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
    else:
        print(text)


if __name__ == "__main__":
    main()
