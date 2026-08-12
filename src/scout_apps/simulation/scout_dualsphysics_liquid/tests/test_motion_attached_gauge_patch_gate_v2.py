"""Static/negative tests for the exact S5B0 motion-attached Gauge contract."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/r8_liquid_motion_attached_gauge_patch_gate_v2.py"
SPEC = importlib.util.spec_from_file_location("motion_gauge_patch_v2_test", SCRIPT)
gate = importlib.util.module_from_spec(SPEC)
assert SPEC.loader
SPEC.loader.exec_module(gate)


def policy():
    return gate.read_json(gate.POLICY_PATH)


def fixture():
    return gate.read_json(gate.FIXTURE_PATH)


def test_golden_binds_exact_s5b0_v4_sixteen_probe_contract_and_patch_bytes():
    result = gate.audit()
    assert result["status"] == "PASS_MOTION_ATTACHED_GAUGE_PATCH_V2_STATIC_CROSS_LAYER_CONTRACT"
    assert result["runtime_status"] == "NOT_RUN_FRESH_RUNTIME_SMOKE_REQUIRED"
    assert result["probe_names"] == [f"s5b0_p{i:02d}" for i in range(16)]
    assert result["probe_radius_m"] == 0.0145
    assert result["pointdp_m"] == 0.001
    assert result["h0_m"] == 0.058
    assert result["maximum_invalid_ratio"] == 0.001
    assert result["proces_time_call_count_before"] == result["proces_time_call_count_after"]
    assert result["solver_order"]["initial_gauge_after_absolute_restart_sync"] is True
    assert result["compiler_run"] is result["solver_run"] is result["gpu_exposed"] is False


@pytest.mark.parametrize("field,value", [
    ("probe_radius_m", 0.0148), ("pointdp_m", 0.002), ("h0_m", 0.1),
    ("maximum_invalid_ratio", 0.25), ("motionref", 1), ("mkbound", 1),
])
def test_fixture_rejects_cross_layer_scalar_drift(field, value):
    mutated = copy.deepcopy(fixture())
    mutated[field] = value
    with pytest.raises(gate.PatchV2Error):
        gate.validate_fixture(mutated, gate.read_json(gate.S5B0_POLICY_PATH)["gauge_contract"])


def test_fixture_rejects_probe_name_order_coordinate_and_extra_drift():
    gauge = gate.read_json(gate.S5B0_POLICY_PATH)["gauge_contract"]
    mutations = []
    value = copy.deepcopy(fixture()); value["probes"][0]["probe_id"] = "swl_00"; mutations.append(value)
    value = copy.deepcopy(fixture()); value["probes"][0], value["probes"][1] = value["probes"][1], value["probes"][0]; mutations.append(value)
    value = copy.deepcopy(fixture()); value["probes"][0]["point0_m"][0] += 1e-4; mutations.append(value)
    value = copy.deepcopy(fixture()); value["probes"][0]["unexpected"] = True; mutations.append(value)
    for mutated in mutations:
        with pytest.raises(gate.PatchV2Error):
            gate.validate_fixture(mutated, gauge)


def test_policy_rejects_runtime_pass_or_patch_parent_drift():
    original = policy()
    value = copy.deepcopy(original)
    value["attachment_contract"]["runtime_order_requires_fresh_smoke_before_s5b0"] = False
    with pytest.raises(Exception):
        gate.audit(value, fixture())
    value = copy.deepcopy(original)
    value["parent_v1"]["unified_patch_sha256"] = "0" * 64
    with pytest.raises(gate.PatchV2Error):
        gate.audit(value, fixture())


def test_rendered_patch_is_byte_identical_to_v1_and_cli_has_no_action_surface():
    current = policy()
    v1 = gate.load_v1(current)
    source = v1.load_upstream()
    rendered = v1.render_unified(source, v1.apply_in_memory(source))
    assert gate.sha256_bytes(rendered) == current["parent_v1"]["unified_patch_sha256"]
    text = SCRIPT.read_text(encoding="utf-8")
    assert 'choices=("self-check", "render-unified")' in text
    for forbidden in ("/usr/bin/sudo", "apparmor_parser", "nvcc", '"make"', "capture.bag"):
        assert forbidden not in text


def test_schema_is_deep_closed_and_static_claims_are_fail_closed():
    schema = json.loads(gate.SCHEMA_PATH.read_text(encoding="utf-8"))
    gate.assert_deep_closed(schema)
    result = gate.audit()
    for key in ("compiler_run","solver_run","gpu_exposed","sudo_used","apparmor_loaded","network_used","optional_bag_read","build_authorized","s5b0_replay_authorized"):
        assert result[key] is False
