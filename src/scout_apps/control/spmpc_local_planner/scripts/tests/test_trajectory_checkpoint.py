import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from planning.checkpoint import from_horizon


def test_exact_terminal_state_and_clock_are_preserved():
    rows = [float(i)/100 for i in range(84)]
    record = dict(schema_version=8, cost_model_version=3, liquid_model_version=1,
                  valid=True, model_state_width=28, model_states=rows, horizon_steps=2,
                  dt=1/30, task_elapsed_sec=3., cycle_id=10)
    result = from_horizon(record)
    assert result["state"] == rows[56:84]
    assert result["task_elapsed_sec"] == pytest.approx(3+2/30)
    assert from_horizon(record, 0)["state"] == rows[:28]
    record["model_states"] = rows[:-1]
    with pytest.raises(ValueError):
        from_horizon(record)
    record["model_states"] = rows
    record["model_state_width"] = 24
    with pytest.raises(ValueError):
        from_horizon(record)
