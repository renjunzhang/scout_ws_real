#!/usr/bin/env python3
"""Static v2 cross-layer contract for the motion-attached native SWL patch.

No source tree is modified and no compiler, solver, GPU, AppArmor, sudo, bag,
or network command is reachable from this CLI.  The C++ patch bytes remain the
byte-pinned v1 rendering; v2 binds those bytes to the exact S5B0 16-probe
contract and proves the relevant upstream solver call order.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Any, Iterable, Mapping

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_motion_attached_gauge_patch_policy_v2.json"
SCHEMA_PATH = ROOT / "schema/target_host_motion_attached_gauge_patch_policy_v2.json"
FIXTURE_PATH = ROOT / "tests/fixtures/motion_attached_gauge_patch_fixture_v2.json"
S5B0_POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5b0_replay_admission_policy_v4.json"
S5B0_SCHEMA_PATH = ROOT / "schema/target_host_s5b0_replay_admission_policy_v4.json"
V1_POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_motion_attached_gauge_patch_policy_v1.json"
V1_SCHEMA_PATH = ROOT / "schema/target_host_motion_attached_gauge_patch_policy_v1.json"
V1_GATE_PATH = ROOT / "scripts/r8_liquid_motion_attached_gauge_patch_gate_v1.py"
V1_TESTS_PATH = ROOT / "tests/test_motion_attached_gauge_patch_gate_v1.py"
V1_FIXTURE_PATH = ROOT / "tests/fixtures/motion_attached_gauge_patch_fixture_v1.json"
UPSTREAM_COMMIT = "ef3721a861fda961f0e2f9ec4cd317b19de99086"
BARE_REPOSITORY = Path("/home/zrj/scout_liquid_lab/dependency/source/DualSPHysics_ef3721a861fda961f0e2f9ec4cd317b19de99086.full_attempt_3.git")


class PatchV2Error(RuntimeError):
    """A byte identity or cross-layer invariant differs."""


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def canonical_json(value: Any) -> bytes:
    return (json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True,
                       separators=(",", ":")) + "\n").encode("utf-8")


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"),
                       parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
    if not isinstance(value, dict):
        raise PatchV2Error(f"JSON root is not an object: {path}")
    return value


def assert_deep_closed(value: Any, location: str = "$") -> None:
    if isinstance(value, dict):
        if value.get("type") == "object" and value.get("additionalProperties") is not False:
            raise PatchV2Error(f"schema object open at {location}")
        for key, child in value.items():
            assert_deep_closed(child, f"{location}/{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            assert_deep_closed(child, f"{location}/{index}")


def git_show(relative: str) -> bytes:
    proc = subprocess.run(
        ["/usr/bin/git", f"--git-dir={BARE_REPOSITORY}", "show", f"{UPSTREAM_COMMIT}:{relative}"],
        stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        check=False, timeout=30, env={"PATH":"/usr/bin", "LC_ALL":"C.UTF-8"},
    )
    if proc.returncode or len(proc.stderr) > 65536:
        raise PatchV2Error(f"pinned git show failed for {relative}: rc={proc.returncode}")
    return proc.stdout


def load_v1(policy: Mapping[str, Any]):
    paths = {
        "policy_sha256": V1_POLICY_PATH, "schema_sha256": V1_SCHEMA_PATH,
        "gate_sha256": V1_GATE_PATH, "tests_sha256": V1_TESTS_PATH,
    }
    for key, path in paths.items():
        if sha256_bytes(path.read_bytes()) != policy["parent_v1"][key]:
            raise PatchV2Error(f"frozen v1 parent drift: {path.name}")
    spec = importlib.util.spec_from_file_location("r8_motion_attached_gauge_v1_pinned", V1_GATE_PATH)
    if spec is None or spec.loader is None:
        raise PatchV2Error("cannot load byte-pinned v1 gate")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def validate_fixture(fixture: Mapping[str, Any], gauge: Mapping[str, Any]) -> None:
    required = {"schema_version","attempt_id","motionref","mkbound","ray_axis","probe_radius_m",
                "point0_z_m","point2_z_m","pointdp_m","h0_m","maximum_invalid_ratio","probes"}
    if set(fixture) != required or fixture["schema_version"] != "smpcc-r8-liquid-motion-attached-gauge-fixture-v2":
        raise PatchV2Error("fixture v2 is not closed")
    scalar_pairs = {
        "probe_radius_m":"probe_radius_m", "point0_z_m":"point0_z_m",
        "point2_z_m":"point2_z_m", "pointdp_m":"pointdp_m", "h0_m":"h0_m",
        "maximum_invalid_ratio":"maximum_invalid_ratio",
    }
    if fixture["attempt_id"] != "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01" or fixture["motionref"] != 0 or fixture["mkbound"] != 0 or fixture["ray_axis"] != "GLOBAL_Z":
        raise PatchV2Error("fixture selection/attachment identity drift")
    for fixture_key, gauge_key in scalar_pairs.items():
        if fixture[fixture_key] != gauge[gauge_key]:
            raise PatchV2Error(f"fixture differs from S5B0 Gauge contract: {fixture_key}")
    probes = fixture["probes"]
    source_probes = gauge["probes"]
    if not isinstance(probes, list) or len(probes) != 16 or len(source_probes) != 16:
        raise PatchV2Error("fixture must contain exactly sixteen probes")
    expected_keys = {"probe_id","angle_deg","point0_m","point2_m","pointdp_m","motionref","mkbound"}
    for index, (probe, source) in enumerate(zip(probes, source_probes)):
        if set(probe) != expected_keys or probe["probe_id"] != source["name"] or probe["angle_deg"] != source["angle_deg"]:
            raise PatchV2Error(f"probe identity/order drift at {index}")
        expected0 = [source["x_m"], source["y_m"], gauge["point0_z_m"]]
        expected2 = [source["x_m"], source["y_m"], gauge["point2_z_m"]]
        if probe["point0_m"] != expected0 or probe["point2_m"] != expected2 or probe["pointdp_m"] != gauge["pointdp_m"] or probe["motionref"] != 0 or probe["mkbound"] != 0:
            raise PatchV2Error(f"probe geometry/binding drift at {index}")
        radius = math.hypot(float(probe["point0_m"][0]), float(probe["point0_m"][1]))
        if abs(radius - gauge["probe_radius_m"]) > 2e-15:
            raise PatchV2Error(f"probe radius drift at {index}")


def verify_solver_order(policy: Mapping[str, Any], patched: Mapping[str, bytes]) -> dict[str, Any]:
    observed: dict[str, str] = {}
    for item in policy["solver_order_sources"]:
        raw = git_show(item["path"])
        digest = sha256_bytes(raw)
        if digest != item["sha256"]:
            raise PatchV2Error(f"solver-order source drift: {item['path']}")
        text = raw.decode("utf-8")
        compute = text.find("const double stepdt=ComputeStep();")
        gauge = text.find("RunGaugeSystem(TimeStep+stepdt);", compute)
        motion = text.find("if(CaseNmoving)RunMotion(stepdt);", gauge)
        if min(compute, gauge, motion) < 0 or not compute < gauge < motion:
            raise PatchV2Error(f"solver step/gauge/boundary order drift: {item['path']}")
        initial = text.find("RunFirstGaugeSystem(TimeStep);")
        if initial < 0:
            raise PatchV2Error(f"initial Gauge call absent: {item['path']}")
        observed[item["path"]] = digest
    jsph = patched["JSph.cpp"].decode("utf-8")
    process = jsph.find("DsMotion->ProcesTime(mode,TimeStep,stepdt);")
    sync = jsph.find("SyncMotionAttachedGauges(false);", process)
    active = jsph.find("const bool active=DsMotion->GetActiveMotion();", sync)
    restart_sync = jsph.find("SyncMotionAttachedGauges(true);")
    if min(process, sync, active, restart_sync) < 0 or not process < sync < active:
        raise PatchV2Error("patched CalcMotion/restart synchronization order drift")
    return {"sources": observed, "step_order":"COMPUTE_STEP_THEN_GAUGE_T_PLUS_DT_THEN_BOUNDARY_RUNMOTION",
            "initial_gauge_after_absolute_restart_sync": True}


def audit(policy: Mapping[str, Any] | None = None, fixture: Mapping[str, Any] | None = None) -> dict[str, Any]:
    policy_obj = dict(policy or read_json(POLICY_PATH))
    fixture_obj = dict(fixture or read_json(FIXTURE_PATH))
    schema = read_json(SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    assert_deep_closed(schema)
    Draft202012Validator(schema).validate(policy_obj)
    bindings = policy_obj["s5b0_v4_binding"]
    if sha256_bytes(S5B0_POLICY_PATH.read_bytes()) != bindings["policy_sha256"] or sha256_bytes(S5B0_SCHEMA_PATH.read_bytes()) != bindings["policy_schema_sha256"] or sha256_bytes(FIXTURE_PATH.read_bytes()) != bindings["fixture_v2_sha256"]:
        raise PatchV2Error("S5B0/fixture parent byte identity drift")
    s5b0 = read_json(S5B0_POLICY_PATH)
    gauge = s5b0["gauge_contract"]
    if sha256_bytes(canonical_json(gauge)) != bindings["gauge_contract_sha256"]:
        raise PatchV2Error("S5B0 Gauge canonical identity drift")
    validate_fixture(fixture_obj, gauge)
    attachment = policy_obj["attachment_contract"]
    if attachment["probe_names"] != [probe["name"] for probe in gauge["probes"]]:
        raise PatchV2Error("policy probe identities differ from S5B0")
    v1 = load_v1(policy_obj)
    before = v1.load_upstream()
    patched = v1.apply_in_memory(before)
    rendered = v1.render_unified(before, patched)
    if sha256_bytes(rendered) != policy_obj["parent_v1"]["unified_patch_sha256"] or v1.aggregate_source_hash(patched) != policy_obj["parent_v1"]["patched_six_file_aggregate_sha256"]:
        raise PatchV2Error("v1 rendered patch byte identity drift")
    expected_files = {entry["path"].split("/")[-1]: (entry["before_sha256"], entry["after_sha256"]) for entry in policy_obj["source_files"]}
    observed_files = {name: (sha256_bytes(before[name]), sha256_bytes(patched[name])) for name in sorted(before)}
    if observed_files != expected_files:
        raise PatchV2Error("six-file before/after identity drift")
    before_calls = before["JSph.cpp"].count(b"DsMotion->ProcesTime(")
    after_calls = patched["JSph.cpp"].count(b"DsMotion->ProcesTime(")
    if before_calls != after_calls or attachment["additional_proces_time_calls"] != 0:
        raise PatchV2Error("motion clock advancement count drift")
    order = verify_solver_order(policy_obj, patched)
    return {
        "status":"PASS_MOTION_ATTACHED_GAUGE_PATCH_V2_STATIC_CROSS_LAYER_CONTRACT",
        "runtime_status":"NOT_RUN_FRESH_RUNTIME_SMOKE_REQUIRED",
        "probe_count":16, "probe_names":attachment["probe_names"],
        "probe_radius_m":attachment["probe_radius_m"], "pointdp_m":attachment["pointdp_m"],
        "h0_m":attachment["h0_m"], "maximum_invalid_ratio":attachment["maximum_invalid_ratio"],
        "motionref":0, "mkbound":0, "unified_patch_sha256":sha256_bytes(rendered),
        "patched_six_file_aggregate_sha256":v1.aggregate_source_hash(patched),
        "proces_time_call_count_before":before_calls, "proces_time_call_count_after":after_calls,
        "solver_order":order, "compiler_run":False, "solver_run":False, "gpu_exposed":False,
        "sudo_used":False, "apparmor_loaded":False, "network_used":False, "optional_bag_read":False,
        "build_authorized":False, "s5b0_replay_authorized":False,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", "render-unified"))
    args = parser.parse_args(argv)
    report = audit()
    if args.command == "render-unified":
        v1 = load_v1(read_json(POLICY_PATH))
        source = v1.load_upstream()
        sys.stdout.buffer.write(v1.render_unified(source, v1.apply_in_memory(source)))
    else:
        print(json.dumps(report, ensure_ascii=False, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (PatchV2Error, OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as exc:
        print(f"FAIL_MOTION_ATTACHED_GAUGE_PATCH_V2: {exc}", file=sys.stderr)
        raise SystemExit(1)
