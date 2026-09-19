#!/usr/bin/env python3
"""Generate an external baseline reference offline; never publishes commands."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from baselines.zvd import generate_plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("task", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--base-duration", type=float, required=True)
    parser.add_argument("--ramp-duration", type=float, default=2.)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists; preserve previous evidence")
    plan = generate_plan(args.task, base_duration=args.base_duration, ramp_duration=args.ramp_duration)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(plan, indent=2, allow_nan=False)+"\n")
    print(json.dumps({"baseline": plan["baseline"], "validation": plan["validation"]}, indent=2))


if __name__ == "__main__":
    main()
