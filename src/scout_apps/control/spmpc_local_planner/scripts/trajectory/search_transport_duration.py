#!/usr/bin/env python3
"""Qualify a fixed-duration slosh plan; search shorter times only with --optimize-time."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from planning.duration_search import run_search
from planning.task import load_task


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('task', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--warm-plan', type=Path, required=True)
    parser.add_argument('--cpp-validator', type=Path, required=True)
    parser.add_argument('--slosh-limits', type=Path, required=True,
                        help='JSON upper limits in metres for transport_peak_m, transport_p95_m, transport_rms_m')
    parser.add_argument('--optimize-time', action='store_true',
                        help='after slosh qualification, select the shortest verified candidate')
    parser.add_argument('--times', nargs='+', type=float,
                        help='explicit descending durations, starting at the task duration; requires --optimize-time')
    parser.add_argument('--process-seconds', type=float, default=600.)
    parser.add_argument('--total-seconds', type=float, default=2400.)
    args = parser.parse_args()
    if args.optimize_time != bool(args.times):
        parser.error('--optimize-time and --times must be supplied together')
    candidates = args.times if args.optimize_time else [load_task(args.task)['transport_duration']]
    result = run_search(args.task, args.warm_plan, args.output.resolve(), candidates,
                        args.process_seconds, args.total_seconds, args.cpp_validator.resolve(),
                        optimize_time=args.optimize_time, slosh_limits=json.loads(args.slosh_limits.read_text()))
    print(json.dumps({k:result[k] for k in ('status', 'optimize_time', 'selected_plan', 'wall_seconds')}))
    return 0 if result['selected_plan'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
