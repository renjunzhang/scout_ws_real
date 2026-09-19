import copy
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "acados"))
from rotating_liquid_replay import (  # noqa: E402
    evaluate_prediction_liquid,
    replay_ocp,
)
from spmpc_acados_model import PARAM_NAMES, PARAM_NAMES_SLOSH, PIDX, PIDX_SLOSH  # noqa: E402


EPOCH_NS = 1_000_000_000
LIQUID = [0.7, 3.2, 1.0, 1.0]
INITIAL = [0.002, 0.015, -0.001, 0.006]


def make_case(width, model_states, *, n=3):
    names = PARAM_NAMES_SLOSH if width == 28 else PARAM_NAMES
    index = PIDX_SLOSH if width == 28 else PIDX
    rows = np.zeros((n + 1, len(names)))
    rows[:, index["actuator_dt"]] = 1.0 / 30.0
    rows[:, index["actuator_tau_v"]] = 0.112
    rows[:, index["actuator_tau_omega"]] = 0.119
    rows[:, index["actuator_gain_v"]] = 1.018
    rows[:, index["actuator_gain_omega"]] = 1.096
    if width == 28:
        rows[:, index["two_zeta_omega_n"]] = LIQUID[0]
        rows[:, index["omega_n_sq"]] = LIQUID[1]
        rows[:, index["kappa_x"]] = LIQUID[2]
        rows[:, index["kappa_y"]] = LIQUID[3]
    snapshot = {
        "schema_version": 8,
        "cost_model_version": 3,
        "liquid_model_version": 1,
        "cycle_id": 42,
        "solver_input_epoch_ns": EPOCH_NS,
        "dt": 1.0 / 30.0,
        "horizon_steps": n,
        "state_width": width,
        "control_width": 3,
        "parameter_width": len(names),
        "parameter_names": list(names),
        "slosh_enabled": width == 28,
        "valid": True,
        "stage_parameters": rows.reshape(-1).tolist(),
    }
    horizon = {
        "schema_version": 8,
        "cost_model_version": 3,
        "liquid_model_version": 1,
        "cycle_id": 42,
        "solver_input_epoch_ns": EPOCH_NS,
        "dt": 1.0 / 30.0,
        "horizon_steps": n,
        "model_state_width": width,
        "model_states": np.asarray(model_states).reshape(-1).tolist(),
        "valid": True,
    }
    return {"snapshot": snapshot, "horizon": horizon}


def evaluate(case):
    return evaluate_prediction_liquid(
        case["snapshot"], case["horizon"], INITIAL, EPOCH_NS,
        LIQUID, 0.18, "observer", True)


def full_replay_case():
    n = 3
    params = np.zeros(len(PARAM_NAMES_SLOSH))
    params[PIDX_SLOSH["actuator_dt"]] = 1.0 / 30.0
    params[PIDX_SLOSH["actuator_tau_v"]] = 0.112
    params[PIDX_SLOSH["actuator_tau_omega"]] = 0.119
    params[PIDX_SLOSH["actuator_gain_v"]] = 1.018
    params[PIDX_SLOSH["actuator_gain_omega"]] = 1.096
    for key, value in zip(("two_zeta_omega_n", "omega_n_sq", "kappa_x", "kappa_y"), LIQUID):
        params[PIDX_SLOSH[key]] = value
    x0 = np.zeros(28)
    x0[3], x0[5] = 0.2, 0.45
    x0[8], x0[13] = 0.12, 0.20
    x0[24:28] = INITIAL
    controls = np.asarray([[0.3, 0.4, 0.2], [0.1, -0.3, 0.2], [-0.2, 0.1, 0.2]])
    states = replay_ocp(x0, controls, np.tile(params, (n + 1, 1)), liquid_model_version=1)
    return make_case(28, states, n=n), states


def test_b0_placeholder_does_not_zero_explicit_observer_initial_state():
    case = make_case(24, np.zeros((4, 24)))
    # B0 has no liquid tail; this must not become the independent initial q.
    assert all(value == 0.0 for value in case["horizon"]["model_states"])
    result = evaluate(case)
    assert len(result["h_modal"]) == case["horizon"]["horizon_steps"] + 1
    assert result["h_modal"][0] == pytest.approx(0.18 * np.hypot(INITIAL[0], INITIAL[2]))
    assert result["h_modal"][0] > 0.0
    assert not np.allclose(result["liquid_states"][1], np.zeros(4))


def test_b0_and_full_same_actual_inputs_reconstruct_identical_liquid_memory():
    full, states = full_replay_case()
    b0 = make_case(24, states[:, :24])
    full_result = evaluate(full)
    b0_result = evaluate(b0)
    np.testing.assert_allclose(
        b0_result["liquid_states"], full_result["liquid_states"], atol=0.0, rtol=0.0)
    np.testing.assert_allclose(full_result["liquid_states"], states[:, 24:28], atol=1e-13, rtol=0.0)


def test_command_acceleration_and_input_records_are_ignored_and_unchanged():
    case = make_case(24, np.zeros((4, 24)))
    original = copy.deepcopy(case)
    baseline = evaluate(case)
    altered = copy.deepcopy(case)
    altered["horizon"]["model_states"] = np.asarray(altered["horizon"]["model_states"]) + 0.0
    altered["horizon"]["a"] = [-1e6] * 3
    altered["horizon"]["alpha_or_omega"] = [1e6] * 3
    changed = evaluate(altered)
    np.testing.assert_array_equal(changed["liquid_states"], baseline["liquid_states"])
    assert case == original


@pytest.mark.parametrize(
    "source,epoch,match",
    [("b0_placeholder", EPOCH_NS, "aligned observer"),
     ("observer", EPOCH_NS + 1, "epoch")],
)
def test_initial_state_source_and_epoch_are_fail_closed(source, epoch, match):
    case = make_case(24, np.zeros((4, 24)))
    with pytest.raises(ValueError, match=match):
        evaluate_prediction_liquid(
            case["snapshot"], case["horizon"], INITIAL, epoch,
            LIQUID, 0.18, source, True)


@pytest.mark.parametrize("field,value", [("valid", False), ("cycle_id", 43)])
def test_snapshot_horizon_pair_must_be_valid_and_same_cycle(field, value):
    case = make_case(24, np.zeros((4, 24)))
    case["horizon"][field] = value
    with pytest.raises(ValueError, match="valid=true|cycle_id"):
        evaluate(case)


def test_initial_state_validity_is_required_even_for_observer_source():
    case = make_case(24, np.zeros((4, 24)))
    with pytest.raises(ValueError, match="liquid_initial_valid"):
        evaluate_prediction_liquid(
            case["snapshot"], case["horizon"], INITIAL, EPOCH_NS,
            LIQUID, 0.18, "observer", False)


@pytest.mark.parametrize("field,value", [("schema_version", 7), ("cost_model_version", 2),
                                          ("liquid_model_version", 0)])
def test_historical_horizon_is_not_silently_reinterpreted(field, value):
    case = make_case(24, np.zeros((4, 24)))
    case["horizon"][field] = value
    with pytest.raises(ValueError, match="schema8/cost3/liquid1"):
        evaluate(case)


def test_schema9_records_actual_jerk_and_keeps_liquid_replay_unchanged():
    case = make_case(24, np.zeros((4, 24)))
    baseline = evaluate(case)
    for name in ("snapshot", "horizon"):
        case[name]["schema_version"] = 9
        case[name]["actual_jerk_max"] = 1.
    np.testing.assert_array_equal(evaluate(case)["liquid_states"], baseline["liquid_states"])
    del case["snapshot"]["actual_jerk_max"]
    with pytest.raises(ValueError, match="actual_jerk_max"):
        evaluate(case)
