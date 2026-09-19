#!/usr/bin/env python3
"""Generate an external baseline reference offline; never publishes commands."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from planning.diagnostics import ProgressLog
from planning.optimizer import PlanValidationError


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("task", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--method", choices=("zvd", "fp-as"), default="zvd")
    parser.add_argument("--base-duration", type=float)
    parser.add_argument("--ramp-duration", type=float, default=2.)
    parser.add_argument("--warm-plan", type=Path)
    parser.add_argument("--diagnostics", type=Path)
    parser.add_argument("--rejected-plan", type=Path)
    parser.add_argument("--solver-verbosity", type=int, choices=range(13), default=0)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists; preserve previous evidence")
    if args.rejected_plan and args.rejected_plan.exists():
        parser.error("rejected-plan diagnostic already exists")
    if args.method == "zvd" and (args.base_duration is None or args.warm_plan):
        parser.error("ZVD needs --base-duration and does not consume a warm plan")
    if args.method == "fp-as" and args.base_duration is not None:
        parser.error("FP-AS duration comes only from the task")
    progress = ProgressLog(args.diagnostics) if args.diagnostics else None
    try:
        if args.method == "zvd":
            from baselines.zvd import generate_plan
            plan = generate_plan(args.task, base_duration=args.base_duration, ramp_duration=args.ramp_duration)
        else:
            from baselines.fixed_path import solve_fixed_path
            warm = json.loads(args.warm_plan.read_text()) if args.warm_plan else None
            plan = solve_fixed_path(args.task, warm, progress=progress, solver_verbosity=args.solver_verbosity)
    except (RuntimeError, ValueError, KeyError) as error:
        if progress:
            progress("attempt_failed", error=str(error))
        if isinstance(error, PlanValidationError) and args.rejected_plan:
            args.rejected_plan.parent.mkdir(parents=True, exist_ok=True)
            args.rejected_plan.write_text(json.dumps(dict(executable=False,
                rejection=str(error), candidate=error.candidate), indent=2, allow_nan=False)+"\n")
        print(f"baseline generation failed: {error}", file=sys.stderr)
        return 2
    finally:
        if progress:
            progress.close()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(plan, indent=2, allow_nan=False)+"\n")
    print(json.dumps({"baseline": plan["baseline"], "validation": plan["validation"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
