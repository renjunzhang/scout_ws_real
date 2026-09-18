import copy
import json
from pathlib import Path
import sys
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from planning_fixture import make_plan
from planning.process_runner import ProcessResult
from search_transport_duration import run_search


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
    def failed(command, log, timeout):
        log.write_text('infeasible')
        return ProcessResult(command, 2, 'failed', .01, timeout)
    out = tmp_path/'search'
    with patch('search_transport_duration.run_process', side_effect=failed):
        result = run_search(task, baseline, out, [4., 3.5], 1., 5., Path(sys.executable))
    assert result['status'] == 'constrained_baseline_failed'
    assert [a['status'] for a in result['attempts']] == ['failed', 'not_attempted']
    assert not (out/'selected_plan.json').exists()
    assert json.loads((out/'manifest.json').read_text())['selected_plan'] is None


def test_interrupt_keeps_a_complete_attempt_list_without_executable_output(tmp_path):
    task, baseline = inputs(tmp_path)
    out = tmp_path/'search'
    with patch('search_transport_duration.run_process', side_effect=KeyboardInterrupt):
        with pytest.raises(KeyboardInterrupt):
            run_search(task, baseline, out, [4., 3.5], 1., 5., Path(sys.executable))
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
        run_search(task, baseline, tmp_path/'search', [4., 3.5], 1., 5., Path(sys.executable))
