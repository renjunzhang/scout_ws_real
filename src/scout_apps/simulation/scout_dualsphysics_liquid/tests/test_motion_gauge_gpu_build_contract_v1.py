import copy
import importlib.util
import json
import sys
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator, ValidationError


ROOT = Path(__file__).resolve().parents[1]


def load(name: str, relative: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


gate = load("motion_gauge_build_gate_v1", "scripts/r8_liquid_motion_gauge_gpu_build_gate_v1.py")
profiles = load("motion_gauge_profile_v1", "scripts/r8_liquid_motion_gauge_gpu_build_profile_generator_v1.py")
sys.path.insert(0, str(ROOT / "scripts"))
supervisor = load("motion_gauge_supervisor_v1", "scripts/r8_liquid_motion_gauge_gpu_build_supervisor_v1.py")


def policy():
    return json.loads(gate.POLICY_PATH.read_bytes())


def inventory_pair():
    p = policy()
    before = {item["path"]: item["before_sha256"] for item in p["patch_transition"]["files"]}
    after = {item["path"]: item["after_sha256"] for item in p["patch_transition"]["files"]}
    for index in range(346):
        name = f"unchanged_{index:03d}.cpp"
        digest = f"{index + 1:064x}"
        before[name] = after[name] = digest
    return before, after


def test_self_check_is_static_fail_closed():
    report = gate.self_check()
    assert report["status"] == "PASS_MOTION_GAUGE_GPU_BUILD_V1_STATIC_CONTRACT_NOT_ADMITTED"
    assert report["counts"] == {"changed": 6, "unchanged": 346, "source_entries": 352,
                                 "post_wrapper_entries": 353, "objects": 131,
                                 "audit_commands": 142}
    assert report["motion_patch_v2_parent_sha256"] is None
    for key in ("external_root_created", "files_written", "sudo_used", "apparmor_loaded",
                "compiler_run", "make_run", "candidate_executed", "gpu_exposed"):
        assert report[key] is False


def test_policy_and_receipt_schemas_are_closed():
    p = policy()
    for path in (gate.POLICY_SCHEMA, gate.RECEIPT_SCHEMA):
        schema = json.loads(path.read_bytes())
        gate.assert_deep_closed(schema)
        Draft202012Validator.check_schema(schema)
    Draft202012Validator(json.loads(gate.POLICY_SCHEMA.read_bytes())).validate(p)
    receipt = gate.build_not_admitted_receipt(p, "a" * 64)
    Draft202012Validator(json.loads(gate.RECEIPT_SCHEMA.read_bytes())).validate(receipt)
    bad = copy.deepcopy(p)
    bad["extra"] = True
    with pytest.raises(ValidationError):
        Draft202012Validator(json.loads(gate.POLICY_SCHEMA.read_bytes())).validate(bad)


def test_exact_patch_transition_accepts_only_six_changes():
    before, after = inventory_pair()
    report = gate.validate_patch_inventories(policy(), before, after)
    assert report["changed_count"] == 6 and report["unchanged_count"] == 346
    extra = dict(after)
    extra["unchanged_000.cpp"] = "f" * 64
    with pytest.raises(gate.BuildContractError, match="exact six"):
        gate.validate_patch_inventories(policy(), before, extra)


@pytest.mark.parametrize("mutation", ["missing", "wrong_before", "wrong_after", "extra_entry"])
def test_patch_inventory_negative_cases(mutation):
    before, after = inventory_pair()
    if mutation == "missing":
        before.pop("unchanged_000.cpp")
    elif mutation == "wrong_before":
        before["JSph.cpp"] = "0" * 64
    elif mutation == "wrong_after":
        after["JSph.h"] = "0" * 64
    else:
        before["extra"] = after["extra"] = "e" * 64
    with pytest.raises(gate.BuildContractError):
        gate.validate_patch_inventories(policy(), before, after)


def test_exact_84_byte_wrapper_and_353_inventory():
    p = policy()
    wrapper = p["wrapper"]["content_utf8"].encode()
    before, after = inventory_pair()
    inventory = dict(after)
    inventory["U3GpuBuild.mk"] = p["wrapper"]["sha256"]
    assert gate.validate_post_wrapper_inventory(p, inventory, wrapper, 0o600)["entry_count"] == 353
    with pytest.raises(gate.BuildContractError):
        gate.validate_post_wrapper_inventory(p, inventory, wrapper + b"x", 0o600)
    with pytest.raises(gate.BuildContractError):
        gate.validate_post_wrapper_inventory(p, inventory, wrapper, 0o700)


def test_build_evidence_is_one_make_gxx11_sm120_and_131_objects():
    p = policy()
    g1 = json.loads((ROOT / p["parents"]["g1"]["policy"]["path"]).read_bytes())
    objects = g1["object_contract"]["object_names"]
    gate.validate_build_evidence(p, make_count=1, make_argv=p["build"]["make_argv"],
                                 object_names=objects, candidate_mode=0o400,
                                 candidate_regular=True, candidate_nlink=1,
                                 candidate_size=1)
    with pytest.raises(gate.BuildContractError):
        gate.validate_build_evidence(p, make_count=2, make_argv=p["build"]["make_argv"],
                                     object_names=objects, candidate_mode=0o400,
                                     candidate_regular=True, candidate_nlink=1,
                                     candidate_size=1)
    argv = list(p["build"]["make_argv"])
    argv[argv.index("CC=/usr/bin/x86_64-linux-gnu-g++-11")] = "CC=/usr/bin/x86_64-linux-gnu-g++-13"
    with pytest.raises(gate.BuildContractError):
        gate.validate_build_evidence(p, make_count=1, make_argv=argv, object_names=objects,
                                     candidate_mode=0o400, candidate_regular=True,
                                     candidate_nlink=1, candidate_size=1)


def test_static_audit_is_142_read_only_results_and_identity_preserving():
    p = policy()
    g1 = json.loads((ROOT / p["parents"]["g1"]["policy"]["path"]).read_bytes())
    objects = {name: f"{index + 1:064x}" for index, name in enumerate(g1["object_contract"]["object_names"])}
    ids = list(p["static_audit"]["candidate_command_ids"])
    ids += [f"object_readelf_header_{name}" for name in objects]
    candidate = {"sha256": "c" * 64, "mode": "0400", "size": 1}
    gate.validate_static_audit_evidence(p, command_ids=ids,
                                        candidate_before=candidate, candidate_after=candidate,
                                        objects_before=objects, objects_after=objects)
    mutated = dict(objects)
    mutated[next(iter(mutated))] = "f" * 64
    with pytest.raises(gate.BuildContractError, match="mutated"):
        gate.validate_static_audit_evidence(p, command_ids=ids,
                                            candidate_before=candidate, candidate_after=candidate,
                                            objects_before=objects, objects_after=mutated)


def test_profiles_are_render_only_minimal_surfaces():
    report = profiles.self_check()
    assert report["object_rule_count"] == 131
    assert not report["files_written"] and not report["profile_loaded"]
    p, objects = profiles.load_inputs()
    rendered = profiles.render_profiles(p, objects)
    for item in rendered.values():
        text = item["bytes"].decode()
        assert "/dev/nvidia" not in text
        assert "network inet " not in text and "network inet6 " not in text
    patch_text = rendered["patch"]["bytes"].decode()
    host_patch_writes = [line.strip() for line in patch_text.splitlines()
                         if line.strip().startswith(p["campaign"]["source_root"] + "/")
                         and line.strip().endswith(" rw,")]
    assert len(host_patch_writes) == 6
    assert sum(".cpp rw," in line for line in host_patch_writes) == 3
    assert sum(".h rw," in line for line in host_patch_writes) == 3
    audit_text = rendered["static_audit"]["bytes"].decode()
    assert audit_text.count(".o r,") == 131
    assert "/usr/bin/make rix" not in audit_text


def test_admission_and_supervisor_execution_hard_stop():
    p = policy()
    plan = supervisor.build_plan(p)
    assert plan["runtime_attempted"] is False
    assert plan["status"] == "NOT_ADMITTED_MOTION_PATCH_V2_PARENT_HASH_REQUIRED"
    with pytest.raises(supervisor.SupervisorError, match="NOT_ADMITTED"):
        supervisor.run_one_shot()
    assert gate.main(["admission"]) == 2


def test_parent_or_authorization_mutation_fails_closed():
    p = policy()
    p["parents"]["motion_patch_v2"]["sha256"] = "a" * 64
    with pytest.raises(gate.BuildContractError):
        gate.validate_static_contract(p)
    p = policy()
    p["authorization"]["make_compiler_authorized"] = True
    with pytest.raises(gate.BuildContractError):
        gate.validate_static_contract(p)
