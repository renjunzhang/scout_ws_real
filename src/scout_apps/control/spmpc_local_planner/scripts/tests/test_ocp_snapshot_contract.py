import pytest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))
from ocp_snapshot_contract import upgrade_parameter_row, validate_snapshot  # noqa: E402


from spmpc_acados_model import PARAM_NAMES, PARAM_NAMES_SLOSH  # noqa: E402


def record(schema=8, cost=3, nx=24, width=None):
    if width is None:
        width = len(PARAM_NAMES)
    names = list(PARAM_NAMES if schema == 8 else PARAM_NAMES[:34])
    if nx == 28:
        width = len(PARAM_NAMES_SLOSH) if schema == 8 else 46
        names = list(PARAM_NAMES_SLOSH if schema == 8 else PARAM_NAMES[:34] + PARAM_NAMES_SLOSH[len(PARAM_NAMES):])
    return {"schema_version": schema, "cost_model_version": cost,
            "liquid_model_version": 1, "horizon_steps": 60,
            "state_width": nx, "control_width": 3, "parameter_width": width,
            "parameter_names": names,
            "stage_parameters": [0.0] * (width * 61)}


def test_new_and_old_contracts():
    assert validate_snapshot(record())['parameter_width'] == len(PARAM_NAMES)
    assert validate_snapshot(record(7, 2, 28, 46))['parameter_width'] == 46


@pytest.mark.parametrize("kwargs", [dict(schema=7, cost=3), dict(schema=8, cost=2), dict(width=79), dict(nx=23)])
def test_rejects_mixed_or_bad_layout(kwargs):
    with pytest.raises(ValueError):
        validate_snapshot(record(**kwargs))


def test_old_to_new_preserves_common_and_moves_liquid_tail():
    row = list(range(46))
    out = upgrade_parameter_row(row, slosh=True)
    assert out['parameters'][:34] == row[:34]
    assert out['parameters'][len(PARAM_NAMES):len(PARAM_NAMES) + 12] == row[34:46]
    assert out['source_schema'] == 7 and out['target_schema'] == 8
    assert out['purpose'] == 'dynamics_replay_only'
    assert out['cost_reconstruction_supported'] is False


def test_rejects_parameter_name_reordering():
    bad = record()
    bad['parameter_names'][0], bad['parameter_names'][1] = bad['parameter_names'][1], bad['parameter_names'][0]
    with pytest.raises(ValueError):
        validate_snapshot(bad)
