#!/usr/bin/env python3
"""Generate/validate full-task geometry-speed-liquid plans; no ROS or commands."""
import argparse
import json
from pathlib import Path
import sys
from planning.optimizer import solve_task
from planning.validation import validate_plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("task", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--warm-plan", type=Path)
    parser.add_argument("--validate-plan", action="store_true", help="input is an existing plan, skip optimization")
    args = parser.parse_args()
    try:
        if args.validate_plan:
            result = validate_plan(json.loads(args.task.read_text()))
        else:
            warm = json.loads(args.warm_plan.read_text()) if args.warm_plan else None
            result = solve_task(args.task, warm)
        encoded = json.dumps(result, indent=2, allow_nan=False)+"\n"
        args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.output.with_suffix(args.output.suffix+".tmp")
        temporary.write_text(encoded)
        temporary.replace(args.output)
        print(json.dumps(result.get("validation", result), ensure_ascii=False))
    except (RuntimeError, ValueError, KeyError) as error:
        # An infeasible task is an explicit failure, never a saved executable plan.
        print(f"planning failed: {error}", file=sys.stderr)
        return 2
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
