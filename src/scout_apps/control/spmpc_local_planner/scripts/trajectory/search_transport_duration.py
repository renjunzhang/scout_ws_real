#!/usr/bin/env python3
"""Finite offline time search; each candidate is solved in a bounded process."""
import argparse
from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import time

# Import shared planning modules when invoked as a standalone CLI.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from planning.process_runner import run_process
from planning.task import load_task
from planning.validation import validate_plan

SUCCESS = {'Solve_Succeeded', 'Solved_To_Acceptable_Level'}


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def failure_details(events_path):
    if not events_path.exists():
        return {'failure_kind': 'process_or_setup'}
    events = [json.loads(line) for line in events_path.read_text().splitlines()]
    failed = next((e for e in reversed(events) if e['stage'] == 'solve_failed'), None)
    if failed:
        return {'failure_kind': 'solver', 'solver': failed}
    return {'failure_kind': ('postflight_validation' if any(e['stage'] == 'solve_finished' for e in events)
                             else 'input_or_setup'), 'last_event': events[-1] if events else None}


def run_search(task_source, baseline_path, output, candidates, process_seconds, total_seconds, cpp_validator):
    started = time.monotonic()
    task = load_task(task_source)
    if task.get('liquid_policy') is None:
        raise ValueError('time search requires an explicit liquid_policy')
    if any(not math.isfinite(v) or v <= 0 for v in (process_seconds, total_seconds)):
        raise ValueError('process and total budgets must be positive and finite')
    candidates = list(candidates)
    if (not candidates or any(not math.isfinite(t) or t <= 0 for t in candidates)
            or candidates != sorted(set(candidates), reverse=True)
            or candidates[0] != task['transport_duration']):
        raise ValueError('candidates must descend uniquely from the baseline transport duration')
    baseline = json.loads(baseline_path.read_text())
    validate_plan(baseline)
    if baseline.get('optimization', {}).get('status') not in SUCCESS:
        raise ValueError('baseline must be a genuinely reoptimized and validated plan')
    base_task = load_task(baseline['task'])
    for key in set(task) | set(base_task):
        if key not in ('task_id', 'liquid_policy') and task.get(key) != base_task.get(key):
            raise ValueError('baseline/task mismatch: ' + key)
    if not cpp_validator.is_file() or not os.access(cpp_validator, os.X_OK):
        raise ValueError('an executable C++ plan validator is required')
    tasks = []
    for duration in candidates:
        candidate = deepcopy(task)
        candidate['transport_duration'] = duration
        tasks.append(load_task(candidate))
    output.mkdir(parents=True, exist_ok=False)
    write_json(output/'source_task.json', task)
    manifest = dict(status='running', candidate_times_sec=candidates,
                    process_wall_limit_sec=process_seconds, total_wall_limit_sec=total_seconds,
                    retries_per_candidate=0, local_refinements=0,
                    baseline_file=str(baseline_path.resolve()),
                    baseline_sha256=hashlib.sha256(baseline_path.read_bytes()).hexdigest(),
                    liquid_policy=task['liquid_policy'], selected_plan=None,
                    attempts=[dict(transport_duration=t, status='pending') for t in candidates])
    def save():
        manifest['wall_seconds'] = time.monotonic()-started
        write_json(output/'manifest.json', manifest)
    def remaining():
        return total_seconds-(time.monotonic()-started)
    save()
    warm = baseline_path.resolve()
    feasible = []
    for name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
        os.environ[name] = '1'
    try:
        for index, (candidate, attempt) in enumerate(zip(tasks, manifest['attempts'])):
            if remaining() <= 0:
                manifest['status'] = 'total_budget_exhausted'
                break
            directory = output/('T_%04d' % round(candidate['transport_duration']/candidate['dt']))
            directory.mkdir()
            task_path, plan_path = directory/'task.json', directory/'plan.json'
            write_json(task_path, candidate)
            events_path = directory/'events.jsonl'
            command = [sys.executable, '-u', str(Path(__file__).with_name('generate_trajectory_plan.py')),
                       str(task_path), str(plan_path), '--warm-plan', str(warm),
                       '--diagnostics', str(events_path), '--solver-verbosity', '5']
            attempt.update(status='running', task_file=str(task_path), warm_plan=str(warm),
                           task_sha256=hashlib.sha256(task_path.read_bytes()).hexdigest())
            save()
            print('SOLVE T=' + str(candidate['transport_duration']), flush=True)
            result = run_process(command, directory/'solve.log', min(process_seconds, remaining()))
            attempt['process'] = asdict(result)
            attempt['status'] = 'failed'
            if result.status == 'timeout':
                attempt['failure_kind'] = 'process_timeout'
            elif result.returncode != 0:
                attempt.update(failure_details(events_path))
            else:
                try:
                    plan = json.loads(plan_path.read_text())
                    if plan['task'] != candidate or plan.get('liquid_policy') != task['liquid_policy']:
                        raise ValueError('candidate output task/policy identity mismatch')
                    if plan['optimization']['status'] not in SUCCESS:
                        raise ValueError('solver did not report convergence')
                    attempt['validation'] = validate_plan(plan)
                    attempt['optimization'] = plan['optimization']
                    attempt['plan_file'] = str(plan_path)
                    attempt['plan_sha256'] = hashlib.sha256(plan_path.read_bytes()).hexdigest()
                    attempt['status'] = 'python_verified'
                    feasible.append(attempt)
                    warm = plan_path
                except (ValueError, KeyError, RuntimeError) as error:
                    attempt.update(failure_kind='postflight_validation', error=str(error))
            save()
            print('RESULT T=%s %s' % (candidate['transport_duration'], attempt['status']), flush=True)
            if index == 0 and attempt['status'] != 'python_verified':
                manifest['status'] = 'constrained_baseline_failed'
                break
        if feasible:
            selected = min(feasible, key=lambda a: (a['transport_duration'], a['optimization']['objective']))
            if remaining() > 0:
                result = run_process([str(cpp_validator), selected['plan_file']], output/'cpp_validation.log',
                                     min(60., remaining()))
                selected['cpp_validation'] = asdict(result)
                if result.returncode == 0:
                    shutil.copy2(selected['plan_file'], output/'selected_plan.json')
                    manifest.update(status='selected', selected_plan='selected_plan.json',
                                    selected_transport_duration=selected['transport_duration'],
                                    selection_reason='smallest tested T passing Python dense and C++ physical loading checks')
                else:
                    manifest['status'] = 'selected_cpp_validation_failed'
            else:
                manifest['status'] = 'no_budget_for_cpp_validation'
        elif manifest['status'] == 'running':
            manifest['status'] = 'no_valid_candidate'
    except BaseException as error:
        manifest.update(status='interrupted', error=str(error))
        for attempt in manifest['attempts']:
            if attempt['status'] == 'running':
                attempt['status'] = 'interrupted'
        raise
    finally:
        for attempt in manifest['attempts']:
            if attempt['status'] == 'pending':
                attempt.update(status='not_attempted', reason=manifest['status'])
        save()
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('task', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--warm-plan', type=Path, required=True)
    parser.add_argument('--cpp-validator', type=Path, required=True)
    parser.add_argument('--times', nargs='+', type=float, default=[40., 36., 33., 31.5])
    parser.add_argument('--process-seconds', type=float, default=600.)
    parser.add_argument('--total-seconds', type=float, default=2400.)
    args = parser.parse_args()
    result = run_search(args.task, args.warm_plan, args.output.resolve(), args.times,
                        args.process_seconds, args.total_seconds, args.cpp_validator.resolve())
    print(json.dumps({k:result[k] for k in ('status', 'selected_plan', 'wall_seconds')}))
    return 0 if result['selected_plan'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
