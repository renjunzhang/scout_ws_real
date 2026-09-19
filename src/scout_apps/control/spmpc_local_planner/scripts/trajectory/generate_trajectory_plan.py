#!/usr/bin/env python3
"""Generate/validate full-task geometry-speed-liquid plans; no ROS or commands."""
import argparse
import json
from pathlib import Path
import sys
# Import shared planning modules when invoked as a standalone CLI.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from planning.optimizer import solve_task, PlanValidationError
from planning.validation import validate_plan
from planning.diagnostics import ProgressLog


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("task", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--warm-plan", type=Path)
    parser.add_argument("--validate-plan", action="store_true", help="input is an existing plan, skip optimization")
    parser.add_argument("--diagnostics", type=Path, help="new JSONL file for stage/solver diagnostics")
    parser.add_argument("--solver-verbosity", type=int, choices=range(13), default=0)
    parser.add_argument("--rejected-plan", type=Path,
                        help="new diagnostic file for a solved candidate rejected by validation; not a plan")
    args = parser.parse_args()
    if args.rejected_plan and args.rejected_plan.exists():
        parser.error("rejected-plan diagnostic already exists")
    progress = ProgressLog(args.diagnostics) if args.diagnostics else None
    try:
        if args.validate_plan:
            result = validate_plan(json.loads(args.task.read_text()))
        else:
            warm = json.loads(args.warm_plan.read_text()) if args.warm_plan else None
            result = solve_task(args.task, warm, progress=progress, solver_verbosity=args.solver_verbosity)
        encoded = json.dumps(result, indent=2, allow_nan=False)+"\n"
        args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.output.with_suffix(args.output.suffix+".tmp")
        temporary.write_text(encoded)
        temporary.replace(args.output)
        print(json.dumps(result.get("validation", result), ensure_ascii=False))
    except (RuntimeError, ValueError, KeyError) as error:
        # An infeasible task is an explicit failure, never a saved executable plan.
        if progress is not None:
            progress("attempt_failed", error=str(error))
        if isinstance(error, PlanValidationError) and args.rejected_plan:
            args.rejected_plan.parent.mkdir(parents=True, exist_ok=True)
            args.rejected_plan.write_text(json.dumps(dict(executable=False,
                rejection=str(error), candidate=error.candidate), indent=2, allow_nan=False)+"\n")
        print(f"planning failed: {error}", file=sys.stderr)
        return 2
    finally:
        if progress is not None:
            progress.close()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
