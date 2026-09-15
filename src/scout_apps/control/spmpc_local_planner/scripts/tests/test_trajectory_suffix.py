import copy
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from planning import suffix
from planning.task import load_task
from planning.optimizer import dynamics
from planning.validation import validate_plan


from planning_fixture import make_plan


def test_remaining_task_preserves_full_state_and_remaining_clock():
    plan = make_plan(); state = np.asarray(plan["samples"][9]["state"]).copy(); state[25]+=.05
    task = suffix.remaining_task(plan, state, .3)
    assert task["start_state"] == state.tolist()
    assert task["deadline"] == pytest.approx(3.7)
    assert task["transport_duration"] == pytest.approx(3.7)
    assert task["origin_task_elapsed_sec"] == pytest.approx(.3)


def test_nominal_suffix_keeps_fifo_and_acceleration_memory_without_fake_metadata():
    plan = make_plan(); initial = np.asarray(plan["samples"][15]["state"])
    task = suffix.remaining_task(plan, initial, 15*plan["task"]["dt"])
    candidate = suffix._nominal_suffix(plan, task, 15*plan["task"]["dt"])
    assert len(candidate["samples"]) == 136
    assert np.max(np.abs(initial[8:24])) > .01
    assert abs(initial[23]) > .01
    np.testing.assert_allclose(candidate["samples"][0]["state"], initial)
    np.testing.assert_allclose(candidate["samples"][1]["state"][8:24], plan["samples"][16]["state"][8:24], atol=1e-10)
    assert "optimization" not in candidate
    assert candidate["validation"]["status"] == "SOFTWARE_VERIFIED"


def test_compare_rejects_invalid_epoch_and_retains_failures(monkeypatch):
    plan = make_plan()
    with pytest.raises(ValueError): suffix.compare_suffixes(plan, np.zeros(27), 0.)
    with pytest.raises(ValueError): suffix.compare_suffixes(plan, np.zeros(28), .01)
    monkeypatch.setattr(suffix, "_nominal_suffix", lambda *args: (_ for _ in ()).throw(ValueError("nominal failed")))
    monkeypatch.setattr(suffix, "solve_task", lambda *args: (_ for _ in ()).throw(RuntimeError("ipopt failed")))
    result = suffix.compare_suffixes(plan, np.zeros(28), 0.)
    assert [r["feasible"] for r in result["candidates"]] == [False, False]
    assert "nominal failed" in result["candidates"][0]["failure"]
    assert "ipopt failed" in result["candidates"][1]["failure"]
    assert result["best_feasible_candidate"] is None


def test_nominal_suffix_accepts_rollout_roundoff_without_rewriting_progress():
    plan = make_plan()
    state = np.asarray(plan["samples"][15]["state"]).copy()
    state[4] += 2e-8
    elapsed = 15*plan["task"]["dt"]
    candidate = suffix._nominal_suffix(plan, suffix.remaining_task(plan, state, elapsed), elapsed)
    assert candidate["samples"][0]["state"][4] == state[4]
    assert candidate["validation"]["status"] == "SOFTWARE_VERIFIED"
    state[4] += .01
    result = suffix.compare_suffixes(plan, state, elapsed, reoptimize=False)
    assert not result["candidates"][0]["feasible"]
