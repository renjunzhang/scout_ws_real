import copy
import json
from pathlib import Path
import sys
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from planning_fixture import make_plan
from planning.process_runner import ProcessResult
from planning.duration_search import run_search
from planning.transport_objective import METRICS

LIMITS = {key: .1 for key in METRICS}


def inputs(tmp_path):
    plan = make_plan()
    # The fixture stands in for a converged baseline; it remains fully replayed.
    plan['optimization']['status'] = 'Solve_Succeeded'
    baseline = tmp_path/'baseline.json'
    baseline.write_text(json.dumps(plan))
    task = copy.deepcopy(plan['task'])
    task['liquid_policy'] = dict(mode='transport_tail_v1', move_height_m=.01,
                                 tail_height_m=.001, height_tolerance_m=1e-6)
    return task, baseline


def test_failed_constrained_baseline_preserves_attempt_and_never_selects_old_plan(tmp_path):
    task, baseline = inputs(tmp_path)
    task_path = tmp_path/'task.json'
    task_path.write_text(json.dumps(task))
    def failed(command, log, timeout):
        log.write_text('infeasible')
        return ProcessResult(command, 2, 'failed', .01, timeout)
    out = tmp_path/'search'
    with patch('planning.duration_search.run_process', side_effect=failed):
        result = run_search(task_path, baseline, out, [4., 3.5], 1., 5., Path(sys.executable), optimize_time=True, slosh_limits=LIMITS)
    assert result['status'] == 'constrained_baseline_failed'
    assert [a['status'] for a in result['attempts']] == ['failed', 'not_attempted']
    assert not (out/'selected_plan.json').exists()
    assert json.loads((out/'manifest.json').read_text())['selected_plan'] is None


def test_interrupt_keeps_a_complete_attempt_list_without_executable_output(tmp_path):
    task, baseline = inputs(tmp_path)
    out = tmp_path/'search'
    with patch('planning.duration_search.run_process', side_effect=KeyboardInterrupt):
        with pytest.raises(KeyboardInterrupt):
            run_search(task, baseline, out, [4., 3.5], 1., 5., Path(sys.executable), optimize_time=True, slosh_limits=LIMITS)
    result = json.loads((out/'manifest.json').read_text())
    assert result['status'] == 'interrupted'
    assert [a['status'] for a in result['attempts']] == ['interrupted', 'not_attempted']
    assert not (out/'selected_plan.json').exists()


def test_archived_replayed_seed_cannot_bypass_reoptimization_gate(tmp_path):
    task, baseline = inputs(tmp_path)
    plan = json.loads(baseline.read_text())
    plan['optimization']['status'] = 'REPLAYED_FEASIBLE_SEED_NOT_REOPTIMIZED'
    baseline.write_text(json.dumps(plan))
    with pytest.raises(ValueError, match='genuinely reoptimized'):
        run_search(task, baseline, tmp_path/'search', [4., 3.5], 1., 5., Path(sys.executable), optimize_time=True, slosh_limits=LIMITS)


def constrained_baseline(tmp_path):
    task, baseline = inputs(tmp_path)
    plan = json.loads(baseline.read_text())
    plan['task'] = task
    plan['liquid_policy'] = task['liquid_policy']
    plan['optimization']['objective'] = 1.
    baseline.write_text(json.dumps(plan))
    return task, baseline


def test_default_reuses_identical_qualified_baseline_without_numerical_resolve(tmp_path):
    task, baseline = constrained_baseline(tmp_path)
    def cpp_only(command, log, timeout):
        assert command[0] == sys.executable
        assert len(command) == 2
        log.write_text('loaded')
        return ProcessResult(command, 0, 'completed', .01, timeout)
    with patch('planning.duration_search.run_process', side_effect=cpp_only) as process:
        result = run_search(task, baseline, tmp_path/'search', [4.], 1., 5.,
                            Path(sys.executable), slosh_limits=LIMITS)
    assert process.call_count == 1
    assert result['status'] == 'selected'
    assert result['optimize_time'] is False
    assert result['closed_loop_verified'] is False
    assert result['attempts'][0]['baseline_reused'] is True
    assert result['selected_transport_duration'] == 4.


def test_shorter_candidates_require_explicit_time_switch(tmp_path):
    task, baseline = inputs(tmp_path)
    with pytest.raises(ValueError, match='explicit optimize_time'):
        run_search(task, baseline, tmp_path/'search', [4., 3.5], 1., 5.,
                   Path(sys.executable), slosh_limits=LIMITS)
    assert not (tmp_path/'search').exists()


def test_faster_candidate_cannot_replace_baseline_when_p95_or_rms_fails(tmp_path):
    task, baseline = constrained_baseline(tmp_path)
    from planning.validation import validate_plan
    validation = validate_plan(json.loads(baseline.read_text()))
    faster_validation = copy.deepcopy(validation)
    faster_validation['stopping_evaluation']['windows'].update(
        transport_peak_m=.05, transport_p95_m=.11, transport_rms_m=.11)
    def process(command, log, timeout):
        if len(command) > 2:
            plan = json.loads(baseline.read_text())
            plan['task'] = json.loads(Path(command[3]).read_text())
            plan['transport_duration'] = plan['task']['transport_duration']
            Path(command[4]).write_text(json.dumps(plan))
        log.write_text('ok')
        return ProcessResult(command, 0, 'completed', .01, timeout)
    # Isolate selection from OCP convergence; real baseline validation is above.
    with patch('planning.duration_search.validate_plan', side_effect=[validation, faster_validation]), \
         patch('planning.duration_search.run_process', side_effect=process):
        result = run_search(task, baseline, tmp_path/'search', [4., 3.5], 1., 5.,
                            Path(sys.executable), optimize_time=True, slosh_limits=LIMITS)
    assert result['selected_transport_duration'] == 4.
    assert result['attempts'][1]['slosh_evaluation']['passed'] is False
    assert result['attempts'][1]['status'] == 'failed'
