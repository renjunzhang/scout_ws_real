import copy
import importlib.util
import json
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator, ValidationError

ROOT = Path(__file__).resolve().parents[1]


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


gate = load("motion_build_execution_v2", "scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v2.py")
profiles = load("motion_profiles_v2", "scripts/r8_liquid_motion_gauge_gpu_build_profile_validator_v2.py")


def policy():
    return json.loads(gate.POLICY_PATH.read_bytes())


def good_evidence(phase):
    return {"authorized":True, "create_new":True, "o_excl":True, "o_nofollow":True,
            "fd_rechecked":True, "sealed_source_unchanged":True,
            "profile_zero_residue":True, "candidate_executed":False,
            "gpu_exposed":False, "network_used":False,
            "make_count":1 if phase == "build" else 0}


def test_closed_schemas_and_static_self_check():
    report = gate.self_check()
    assert report["status"] == "PASS_STATIC_EXECUTION_V2_NOT_AUTHORIZED_NOT_EXECUTABLE"
    assert report["counts"] == {"parents":14, "profiles":4, "source_entries":352,
                                 "changed":6, "unchanged":346, "wrapper_bytes":84,
                                 "objects":131, "static_commands":557}
    assert report["root_absent"] and report["receipt_prefix_absent"]
    assert not report["real_phase_runner_implemented"]
    for path in (gate.POLICY_SCHEMA, gate.RECEIPT_SCHEMA):
        schema = json.loads(path.read_bytes())
        gate.assert_deep_closed(schema)
        Draft202012Validator.check_schema(schema)
    Draft202012Validator(json.loads(gate.POLICY_SCHEMA.read_bytes())).validate(policy())


def test_policy_is_default_fail_closed_and_runtime_separate():
    p = policy()
    assert p["authorization"]["default_authorized"] is False
    assert p["authorization"]["all_real_phases_authorized"] is False
    assert not any(value for key, value in p["execution_implementation"].items()
                   if key.endswith("_implemented"))
    assert p["runtime_smoke"] == {"included":False,
        "candidate_execution_authorized":False, "gpu_exposure_authorized":False,
        "next_contract":"SEPARATE_CREATE_NEW_RUNTIME_SMOKE_V1"}
    mutated = copy.deepcopy(p)
    mutated["authorization"]["default_authorized"] = True
    with pytest.raises(ValidationError):
        Draft202012Validator(json.loads(gate.POLICY_SCHEMA.read_bytes())).validate(mutated)


def test_exact_profiles_are_identity_pinned_and_minimal():
    report = profiles.validate()
    assert report["status"] == "PASS_EXACT_PROFILES_V2_READ_ONLY_VALIDATION"
    assert len(report["profiles"]) == 4
    assert report["query_argv_template"] == ["/usr/sbin/apparmor_parser", "-Q", "-K", "-T", "<EXACT_PROFILE_PATH>"]


@pytest.mark.parametrize("phase", gate.REAL_PHASES)
def test_mock_phase_contract_accepts_only_complete_safe_evidence(phase):
    gate.validate_mock_phase_evidence(policy(), phase, good_evidence(phase))
    bad = good_evidence(phase)
    bad["fd_rechecked"] = False
    with pytest.raises(gate.GateError):
        gate.validate_mock_phase_evidence(policy(), phase, bad)


@pytest.mark.parametrize("field", ["candidate_executed", "gpu_exposed", "network_used"])
def test_mock_phase_rejects_forbidden_surface(field):
    evidence = good_evidence("static-audit")
    evidence[field] = True
    with pytest.raises(gate.GateError):
        gate.validate_mock_phase_evidence(policy(), "static-audit", evidence)


def test_real_phase_cli_hard_stops_even_with_token_flags(tmp_path):
    token = tmp_path / "authorization.json"
    token.write_text("{}")
    assert gate.main(["source-copy", "--execute", "--authorization-token-file", str(token)]) == 2


def test_receipt_schema_is_closed_and_forbids_gpu_execution():
    schema = json.loads(gate.RECEIPT_SCHEMA.read_bytes())
    receipt = {"schema_version":"smpcc-r8-liquid-motion-gauge-gpu-build-execution-receipt-v2",
      "document_type":"SMPCC_R8_LIQUID_MOTION_GAUGE_GPU_BUILD_EXECUTION_RECEIPT_V2",
      "campaign_id":"motion_gauge_gpu_build_sm120_20260812T034446Z_v1",
      "build_id":"motion_gauge_gpu_build_sm120_20260812T034446Z_v1_a",
      "phase":"SOURCE_COPY","kind":"FAILURE","status":"STOP_NOT_AUTHORIZED",
      "policy_sha256":"a"*64,"authorization":{"execute_flag_present":False,"token_valid":False,"policy_hash_match":False},
      "input_identity":None,"output_identity":None,
      "execution":{"exact_argv":[],"return_code":None,"timed_out":False,"make_count":0,"resource_samples":0},
      "safety":{"create_new_only":True,"o_nofollow":True,"fd_identity_rechecked":False,
        "sealed_source_unchanged":True,"candidate_mode_octal":None,"candidate_executed":False,
        "gpu_exposed":False,"network_used":False,"profile_zero_residue":True,"failure_preserved":True},
      "next_allowed_stage":"STOP_NOT_AUTHORIZED"}
    Draft202012Validator(schema).validate(receipt)
    receipt["safety"]["candidate_executed"] = True
    with pytest.raises(ValidationError):
        Draft202012Validator(schema).validate(receipt)
