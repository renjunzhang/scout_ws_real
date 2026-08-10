"""Static and non-executing checks for the corrected U3 C1 seed v2 gate."""

from __future__ import annotations

import ast
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "scripts/r8_liquid_target_u3_gencase_seed_gate_v2.py"
POLICY = PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_u3_gencase_seed_materialization_policy_v2.json"
SCHEMA = PACKAGE / "schema/target_host_u3_gencase_seed_materialization_policy_v2.json"


def load_gate():
    spec = importlib.util.spec_from_file_location("u3_gencase_seed_gate_v2_test", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_review_artifacts_template_and_rejected_v1_evidence_pass() -> None:
    gate = load_gate()
    review = gate.verify_review_artifacts()
    assert review["template"]["sha256"] == gate.CASE_TEMPLATE_SHA256
    assert review["template"]["boundary_mask"] == "2"
    assert review["template"]["min_fluid_stop"] == "1"
    assert review["template"]["deprecated_parts_out_max_present"] is False
    assert review["rejected_predecessor"]["receipt_sha256"] == gate.V1_RECEIPT_SHA256
    assert review["rejected_predecessor"]["disposition"] == "PERMANENT_NO_GO_DO_NOT_EXECUTE_OR_REUSE"
    assert review["rejected_predecessor"]["used_as_v2_source"] is False


def test_policy_and_schema_are_closed_exact_v2_contracts() -> None:
    gate = load_gate()
    policy = json.loads(POLICY.read_text(encoding="utf-8"))
    schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
    assert policy == gate.expected_policy()
    assert schema["additionalProperties"] is False
    assert set(policy) == set(schema["required"])
    gate.validate_schema_instance(policy, schema)
    assert policy["allowed_gate_commands"] == ["self-check", "preflight", "materialize"]
    assert policy["source_provenance"]["gencase"]["host_execution"] == "forbidden"
    assert policy["materialization_contract"]["predecessor_seed_as_source"] == "forbidden"
    assert policy["invariants"]["no_predecessor_seed_input_reuse"] is True


def test_v2_attempt_paths_do_not_alias_any_rejected_v1_path() -> None:
    gate = load_gate()
    v2_paths = {
        gate.SEED_ROOT,
        gate.INPUT_ROOT,
        gate.RECEIPT,
        gate.PARTIAL_RECEIPT,
    }
    v1_paths = {
        gate.V1_SEED_ROOT,
        gate.V1_INPUT_ROOT,
        gate.V1_RECEIPT,
        gate.V1_PARTIAL_RECEIPT,
    }
    assert v2_paths.isdisjoint(v1_paths)
    assert gate.V1_SEED_ID not in gate.SEED_ID
    assert gate.expected_rejected_predecessor()["is_v2_materialization_source"] is False


def test_schema_rejects_missing_predecessor_and_changed_seed_identity() -> None:
    gate = load_gate()
    schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
    policy = gate.expected_policy()
    missing = dict(policy)
    missing.pop("rejected_predecessor")
    with pytest.raises(gate.GateError):
        gate.validate_schema_instance(missing, schema)
    changed = json.loads(json.dumps(policy))
    changed["frozen_attempt"]["seed_id"] = gate.V1_SEED_ID
    with pytest.raises(gate.GateError):
        gate.validate_schema_instance(changed, schema)


@pytest.mark.parametrize(
    "old,new,error_fragment",
    [
        (b'mask="2"', b'mask="1 | 2"', "boundary contract"),
        (b'key="MinFluidStop"', b'key="PartsOutMax"', "lost-fluid stop"),
    ],
)
def test_template_semantics_reject_v1_geometry_or_deprecated_parameter(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    old: bytes,
    new: bytes,
    error_fragment: str,
) -> None:
    gate = load_gate()
    raw = gate.CASE_TEMPLATE.read_bytes()
    assert old in raw
    changed = raw.replace(old, new, 1)
    candidate = tmp_path / "C1_changed.xml"
    candidate.write_bytes(changed)
    monkeypatch.setattr(gate, "CASE_TEMPLATE", candidate)
    monkeypatch.setattr(gate, "CASE_TEMPLATE_SHA256", hashlib.sha256(changed).hexdigest())
    monkeypatch.setattr(gate, "CASE_TEMPLATE_SIZE_BYTES", len(changed))
    with pytest.raises(gate.GateError, match=error_fragment):
        gate.template_facts()


def test_gate_subprocess_surface_is_only_fixed_git_cat_file() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    tree = ast.parse(source)
    subprocess_calls: list[ast.Call] = []
    forbidden_calls: list[str] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if isinstance(node.func, ast.Attribute) and isinstance(node.func.value, ast.Name):
            qualified = f"{node.func.value.id}.{node.func.attr}"
            if qualified.startswith("subprocess."):
                subprocess_calls.append(node)
            if qualified in {"os.system", "os.execv", "os.execve", "os.spawnv", "os.spawnve"}:
                forbidden_calls.append(qualified)
    assert len(subprocess_calls) == 1
    call = subprocess_calls[0]
    assert isinstance(call.func, ast.Attribute)
    assert call.func.attr == "run"
    keyword_names = {keyword.arg for keyword in call.keywords}
    assert "shell" not in keyword_names
    assert forbidden_calls == []
    assert '"cat-file"' in source
    assert '"blob"' in source
    assert "--no-replace-objects" in source
    assert "predecessor_seed_used_as_source" in source
    assert "precompiled_binary_executed" in source


def test_preflight_is_read_only_and_new_attempt_is_currently_absent() -> None:
    gate = load_gate()
    before = {
        path: path.exists()
        for path in (gate.SEED_ROOT, gate.RECEIPT, gate.PARTIAL_RECEIPT)
    }
    assert before == {gate.SEED_ROOT: False, gate.RECEIPT: False, gate.PARTIAL_RECEIPT: False}
    result = gate.preflight()
    after = {
        path: path.exists()
        for path in (gate.SEED_ROOT, gate.RECEIPT, gate.PARTIAL_RECEIPT)
    }
    assert after == before
    assert result["predecessor_used_as_materialization_source"] is False
    assert result["trusted_tools"]["git"]["sha256"] == gate.GIT_SHA256
