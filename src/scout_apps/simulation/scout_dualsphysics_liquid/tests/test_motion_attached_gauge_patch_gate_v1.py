"""Static and mock-only tests for the motion-attached SWL patch contract."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/r8_liquid_motion_attached_gauge_patch_gate_v1.py"
spec = importlib.util.spec_from_file_location("motion_gauge_patch_v1_test", SCRIPT)
gate = importlib.util.module_from_spec(spec)
assert spec.loader
spec.loader.exec_module(gate)


def _policy():
    return json.loads(gate.POLICY_PATH.read_text(encoding="utf-8"))


def _fixture():
    return json.loads(gate.FIXTURE_PATH.read_text(encoding="utf-8"))


def test_golden_exact_hashes_ordering_and_no_second_proces_time():
    report = gate.audit(_policy(), _fixture())
    assert report["status"] == "PASS_MOTION_ATTACHED_GAUGE_PATCH_V1_STATIC_CONTRACT"
    assert report["probe_count"] == 16
    assert report["motionref"] == report["mkbound"] == 0
    assert report["ray_axis"] == "GLOBAL_Z"
    assert report["proces_time_call_count_before"] == report["proces_time_call_count_after"]
    assert report["compiler_run"] is report["solver_run"] is report["gpu_exposed"] is False
    assert len(report["source_files"]) == 6


def test_render_is_deterministic_and_modifies_exactly_six_files():
    before = gate.load_upstream()
    first = gate.apply_in_memory(before)
    second = gate.apply_in_memory(before)
    assert first == second
    assert set(first) == set(gate.BEFORE_SHA256)
    assert gate.render_unified(before, first) == gate.render_unified(before, second)
    assert all(first[name] != before[name] for name in first)


def test_rejects_before_byte_drift_and_missing_file():
    source = gate.load_upstream()
    drift = dict(source)
    drift["JSph.cpp"] += b"\n"
    with pytest.raises(gate.PatchContractError, match="before hash"):
        gate.apply_in_memory(drift)
    missing = dict(source)
    missing.pop("JSph.h")
    with pytest.raises(gate.PatchContractError, match="inventory"):
        gate.apply_in_memory(missing)


def test_rejects_hash_contract_drift():
    policy = _policy()
    policy["source_files"][0]["after_sha256"] = "0" * 64
    with pytest.raises(gate.PatchContractError, match="source hashes"):
        gate.audit(policy, _fixture())
    policy = _policy()
    policy["patch_identity"]["unified_patch_sha256"] = "0" * 64
    with pytest.raises(gate.PatchContractError, match="patch hash"):
        gate.audit(policy, _fixture())


@pytest.mark.parametrize("mutation", ["count", "identity", "axis", "ray", "extra"])
def test_fixture_fail_closed_negative_cases(mutation):
    value = copy.deepcopy(_fixture())
    if mutation == "count": value["probes"].pop()
    if mutation == "identity": value["probes"][0]["motionref"] = 1
    if mutation == "axis": value["ray_axis"] = "BODY_Z"
    if mutation == "ray": value["probes"][0]["point2_m"][0] += 0.001
    if mutation == "extra": value["unexpected"] = True
    with pytest.raises(gate.PatchContractError):
        gate.validate_fixture(value)


def test_patch_semantics_are_narrow_and_fail_closed():
    before = gate.load_upstream()
    after = gate.apply_in_memory(before)
    corpus = b"\n".join(after.values()).decode()
    assert "Exactly 16 motion-attached JGaugeSwl probes are required." in corpus
    assert "ConfigDomMCel(false);" in corpus
    assert "motionref" in corpus and "mkbound" in corpus
    assert "SyncMotionAttachedGauges(false);" in corpus
    assert "SyncMotionAttachedGauges(true);" in corpus
    assert "\n+//==============================================================================" not in corpus
    assert "network" not in corpus.lower()
    jsph = after["JSph.cpp"].decode()
    assert jsph.count("DsMotion->ProcesTime(") == before["JSph.cpp"].decode().count("DsMotion->ProcesTime(")
