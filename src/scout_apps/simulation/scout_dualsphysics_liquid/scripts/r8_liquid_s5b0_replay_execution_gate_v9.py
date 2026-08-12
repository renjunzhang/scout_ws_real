#!/usr/bin/env python3
"""Static-first fail-closed gate for one exact primary S5B0 replay v9."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import re
import stat
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5b0_replay_execution_policy_v9.json"
POLICY_SCHEMA_PATH = ROOT / "schema/target_host_s5b0_replay_execution_policy_v9.json"
TOKEN_SCHEMA_PATH = ROOT / "schema/target_host_s5b0_replay_authorization_token_v9.json"
RECEIPT_SCHEMA_PATH = ROOT / "schema/target_host_s5b0_replay_execution_receipt_v9.json"
RESULT_SCHEMA_PATH = ROOT / "schema/target_host_s5b0_replay_result_package_v9.json"
PROFILE_PATH = ROOT / "config/apparmor_drafts/r8-liquid-s5b0-primary-bsmooth-b01-20260812t110635z-v9.profile"
PROFILE_GENERATOR_PATH = ROOT / "scripts/r8_liquid_s5b0_replay_profile_generator_v9.py"
RUNTIME_PATH = ROOT / "scripts/r8_liquid_s5b0_replay_runtime_supervisor_v9.py"
LIFECYCLE_PATH = ROOT / "scripts/r8_liquid_s5b0_replay_lifecycle_supervisor_v9.py"
TOKEN_PRODUCER_PATH = ROOT / "scripts/r8_liquid_s5b0_replay_authorization_token_v9.py"
FRAME_READER_PATH = ROOT / "scripts/r8_liquid_s5b0_finalized_frame_reader_v2.py"
GAUGE_NORMALIZER_PATH = ROOT / "scripts/r8_liquid_s5b0_native_gauge_normalizer_v1.py"
OUTPUT_QC_PATH = ROOT / "scripts/r8_liquid_s5b0_replay_output_qc_v9.py"
PACKAGE_ROOT = ROOT


class GateV9Error(ValueError):
    pass


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def canonical_json(value: Any) -> bytes:
    return (json.dumps(value, allow_nan=False, ensure_ascii=False, sort_keys=True,
                       separators=(",", ":")) + "\n").encode("utf-8")


def assert_deep_closed(value: Any, location: str = "$") -> None:
    if isinstance(value, dict):
        kind = value.get("type")
        if (kind == "object" or isinstance(kind, list) and "object" in kind) and value.get("additionalProperties") is not False:
            raise GateV9Error(f"open schema object: {location}")
        for key, child in value.items(): assert_deep_closed(child, f"{location}/{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value): assert_deep_closed(child, f"{location}/{index}")


def _assert_required_equals_properties(value: Any, location: str = "$") -> None:
    if isinstance(value, dict):
        kind = value.get("type")
        if kind == "object" or isinstance(kind, list) and "object" in kind:
            properties = value.get("properties")
            if not isinstance(properties, dict) or set(value.get("required", ())) != set(properties):
                raise GateV9Error(f"schema required/properties differ: {location}")
        for key, child in value.items():
            _assert_required_equals_properties(child, f"{location}/{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _assert_required_equals_properties(child, f"{location}/{index}")


def _resolve(path_text: str) -> Path:
    path = Path(path_text)
    if path.is_absolute(): return path
    if path.parts and path.parts[0] in ("docs", "src"):
        return ROOT.parents[3] / path
    return PACKAGE_ROOT / path


def _load_profile_generator():
    spec = importlib.util.spec_from_file_location("s5b0_profile_generator_v9", PROFILE_GENERATOR_PATH); assert spec and spec.loader
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module); return module


def read_policy() -> tuple[dict[str, Any], str]:
    raw = POLICY_PATH.read_bytes(); value = json.loads(raw)
    for path in (POLICY_SCHEMA_PATH, TOKEN_SCHEMA_PATH, RECEIPT_SCHEMA_PATH, RESULT_SCHEMA_PATH,
                 ROOT / "schema/target_host_s5b0_finalized_solver_frames_manifest_v2.json"):
        schema = json.loads(path.read_bytes()); Draft202012Validator.check_schema(schema)
        assert_deep_closed(schema); _assert_required_equals_properties(schema)
    Draft202012Validator(json.loads(POLICY_SCHEMA_PATH.read_bytes())).validate(value)
    return value, hashlib.sha256(raw).hexdigest()


def validate_contract(policy: Mapping[str, Any]) -> None:
    if policy["status"] != "STATIC_FROZEN_BUILD_AND_EXACT_REPLAY_AUTHORIZATION_REQUIRED": raise GateV9Error("status drift")
    selection = policy["selection"]
    if selection["planned_denominator"] != 1 or any(selection[key] for key in ("optional_bag_read", "optional_authorized", "c2_authorized", "second_execution_authorized", "cross_method_ranking")):
        raise GateV9Error("single-primary selection drift")
    if policy["devices"] != ["/dev/nvidia0", "/dev/nvidiactl", "/dev/nvidia-uvm", "/dev/nvidia-uvm-tools"]:
        raise GateV9Error("four-device set drift")
    if policy["resources"] != {"wall_timeout_seconds": 5400, "kill_after_seconds": 30,
        "sample_min_seconds": 10, "sample_max_seconds": 30,
        "minimum_available_memory_bytes": 4294967296, "minimum_free_vram_bytes": 6442450944,
        "maximum_output_bytes": 1073741824, "xid_zero_required": True}:
        raise GateV9Error("resource contract drift")
    if policy["solver"]["candidate_execution_count"] != 1 or policy["solver"]["parallel_jobs"] != 1:
        raise GateV9Error("execution cardinality drift")
    expected_gauges = [f"GaugesSwl_s5b0_p{index:02d}.csv" for index in range(16)]
    output = policy["output_contract"]
    if output["raw_gauge_names"] != expected_gauges or output["qc_pipeline"] != [
        "frame_reader.read_finalized", "frame_reader.parse_runparts",
        "output_qc.parse_motion_ref", "output_qc.boundary_qc", "output_qc.normalize_gauges",
    ]:
        raise GateV9Error("closed output/QC invocation contract drift")
    expected_reader_parent = {
        "role": "FINALIZED_FRAME_READER_V2",
        "path": "scripts/r8_liquid_s5b0_finalized_frame_reader_v2.py",
        "sha256": "aecd5125625ce4da91b9782f6a28eed017ff5e163056fc648b414aca96e2af4c",
    }
    reader_parents = [parent for parent in policy["parents"]
                      if parent["role"].startswith("FINALIZED_FRAME_READER_")]
    if reader_parents != [expected_reader_parent] or output["frame_reader_revision"] != 2:
        raise GateV9Error("finalized-frame v2 parent binding drift")
    if policy["publication"] != {
        "partial_root": policy["replay"]["partial_root"],
        "final_root": policy["replay"]["final_root"],
        "rename_syscall": "renameat2", "rename_flag": "RENAME_NOREPLACE",
        "rename_flag_value": 1, "parent_fsync_required": True,
        "checksums_relative_path": "checksums.sha256",
        "checksums_scope": "ALL_REGULAR_PACKAGE_FILES_EXCEPT_CHECKSUMS_MANIFEST",
        "partial_preserved_on_failure": True,
    }:
        raise GateV9Error("publication transaction contract drift")
    if policy["journal"]["boot_id_path"] != "/proc/sys/kernel/random/boot_id" or not policy["journal"]["same_boot_required"] or not policy["journal"]["zero_xid_required"]:
        raise GateV9Error("journal boot/cursor/Xid contract drift")
    generator = _load_profile_generator()
    rendered = generator.render_profile(generator.TEMPLATE_PATH.read_text(encoding="utf-8"), generator.exact_replacements()).encode()
    if rendered != PROFILE_PATH.read_bytes() or hashlib.sha256(rendered).hexdigest() != "df6cad9059a84797d819a70c491e2d16dc7a4b65308bfc818697b01e3f402a13":
        raise GateV9Error("exact profile bytes drift")
    if policy["authorization"]["default_authorized"] or policy["authorization"]["profile_sha256"] is not None or policy["authorization"]["candidate_sha256"] is not None:
        raise GateV9Error("premature replay authorization")


def validate_frozen_parents(policy: Mapping[str, Any], *, include_external: bool = False) -> None:
    for parent in policy["parents"]:
        resolved = _resolve(parent["path"])
        if not include_external and not resolved.is_relative_to(ROOT.parents[3]):
            continue
        if sha256(resolved) != parent["sha256"]:
            raise GateV9Error(f"parent drift: {parent['role']}")


def validate_dynamic_fresh(policy: Mapping[str, Any]) -> None:
    build = policy["build_dependency"]
    if build["materialized"] or os.path.lexists(build["candidate_path"]) or os.path.lexists(build["campaign_final_receipt"]):
        raise GateV9Error("fresh v11 dependency is unexpectedly materialized")
    replay = policy["replay"]
    for key in ("stage_root", "partial_root", "final_root", "audit_root", "token_path", "start_receipt", "attempt_receipt", "final_receipt", "failure_receipt", "lifecycle_receipt", "output_inventory_receipt", "result_package_receipt"):
        if os.path.lexists(replay[key]):
            raise GateV9Error(f"fresh replay target already exists: {key}")
    for role, identity in policy["sources"].items():
        if role == "transfer_root":
            continue
        if _file_identity(Path(identity["path"])) != identity:
            raise GateV9Error(f"source drift: {role}")


def validate_static(policy: Mapping[str, Any], *, verify_external: bool = False) -> None:
    validate_contract(policy)
    validate_frozen_parents(policy)
    if verify_external:
        validate_dynamic_fresh(policy)


def _file_identity(path: Path, maximum: int = 512 * 1024 * 1024) -> dict[str, Any]:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1 or not 0 < before.st_size <= maximum: raise GateV9Error("unsafe exact file")
        digest = hashlib.sha256(); total = 0
        while total < before.st_size:
            block = os.read(descriptor, min(1 << 20, before.st_size - total))
            if not block: raise GateV9Error("short exact file read")
            total += len(block); digest.update(block)
        after = os.fstat(descriptor)
    finally: os.close(descriptor)
    if (before.st_dev, before.st_ino, before.st_mode, before.st_nlink, before.st_size, before.st_mtime_ns) != (after.st_dev, after.st_ino, after.st_mode, after.st_nlink, after.st_size, after.st_mtime_ns):
        raise GateV9Error("exact file TOCTOU")
    return {"path": str(path), "sha256": digest.hexdigest(), "size_bytes": total,
            "mode": f"{stat.S_IMODE(after.st_mode):04o}"}


def validate_token(path: Path) -> dict[str, Any]:
    policy, policy_sha = read_policy(); validate_static_allow_built(policy)
    identity = _file_identity(path, 64 * 1024)
    if identity["mode"] != "0600": raise GateV9Error("token mode differs")
    value = json.loads(path.read_bytes()); schema = json.loads(TOKEN_SCHEMA_PATH.read_bytes())
    Draft202012Validator(schema).validate(value)
    if value["policy"]["sha256"] != policy_sha or value["policy"]["path"] != str(POLICY_PATH): raise GateV9Error("token policy binding differs")
    for key, target in (("gate", Path(__file__).resolve()), ("runtime_supervisor", RUNTIME_PATH),
                        ("lifecycle_supervisor", LIFECYCLE_PATH),
                        ("token_producer", TOKEN_PRODUCER_PATH), ("profile", PROFILE_PATH)):
        if value[key] != _file_identity(target): raise GateV9Error(f"token {key} binding differs")
    for key, target in (("policy", POLICY_SCHEMA_PATH), ("authorization_token", TOKEN_SCHEMA_PATH),
                        ("execution_receipt", RECEIPT_SCHEMA_PATH), ("result_package", RESULT_SCHEMA_PATH)):
        if value["schemas"][key] != _file_identity(target):
            raise GateV9Error(f"token schema binding differs: {key}")
    candidate = _file_identity(Path(policy["build_dependency"]["candidate_path"]))
    if candidate["mode"] != "0400" or value["candidate"] != candidate: raise GateV9Error("token candidate binding differs")
    build_receipt = _file_identity(Path(policy["build_dependency"]["campaign_final_receipt"]), 16 * 1024 * 1024)
    receipt = json.loads(Path(build_receipt["path"]).read_bytes())
    if receipt.get("status") != policy["build_dependency"]["required_status"] or value["build_final_receipt"] != build_receipt:
        raise GateV9Error("token build/static receipt binding differs")
    expected_roots = {key: policy["replay"][key] for key in ("stage_root", "partial_root", "final_root", "audit_root")}
    if value["roots"] != expected_roots or value["devices"] != policy["devices"]:
        raise GateV9Error("token root/device binding differs")
    return value


def validate_static_allow_built(policy: Mapping[str, Any]) -> None:
    """Static parent/source checks after authorized v11 has materialized."""
    validate_contract(policy); validate_frozen_parents(policy, include_external=True)
    for role, identity in policy["sources"].items():
        if role != "transfer_root" and _file_identity(Path(identity["path"])) != identity:
            raise GateV9Error(f"source drift: {role}")
    if PROFILE_PATH.read_bytes() != _load_profile_generator().render_profile(_load_profile_generator().TEMPLATE_PATH.read_text(), _load_profile_generator().exact_replacements()).encode():
        raise GateV9Error("profile drift")


def solver_argv(policy: Mapping[str, Any]) -> list[str]:
    replay = policy["replay"]
    return ["/runtime/candidate", "/case/C1M_case", "/output", *policy["solver"]["argv_suffix"]]


def sandbox_argv(policy: Mapping[str, Any]) -> list[str]:
    stage = policy["replay"]["stage_root"]
    argv = ["/usr/bin/aa-exec", "-p", policy["replay"]["profile_name"], "--",
            "/usr/bin/bwrap", "--die-with-parent", "--new-session", "--unshare-user", "--unshare-pid", "--unshare-net", "--unshare-ipc", "--unshare-uts", "--unshare-cgroup-try", "--uid", "1000", "--gid", "1000", "--cap-drop", "ALL", "--hostname", "r8-liquid-s5b0-v9", "--clearenv"]
    for key, value in policy["solver"]["environment"].items(): argv.extend(("--setenv", key, value))
    argv.extend(("--ro-bind", "/usr", "/usr", "--symlink", "usr/lib", "/lib", "--symlink", "usr/lib64", "/lib64", "--dir", "/etc", "--ro-bind", "/etc/ld.so.cache", "/etc/ld.so.cache", "--proc", "/proc", "--dev", "/dev"))
    for device in policy["devices"]: argv.extend(("--dev-bind", device, device))
    argv.extend(("--ro-bind", "/sys", "/sys", "--tmpfs", "/tmp", "--ro-bind", stage + "/runtime", "/runtime", "--ro-bind", stage + "/case", "/case", "--ro-bind", stage + "/restart", "/restart", "--bind", stage + "/guest-output", "/output", "--chdir", "/runtime", "--", *solver_argv(policy)))
    writable = [argv[index + 1:index + 3] for index, token in enumerate(argv) if token == "--bind"]
    if writable != [[stage + "/guest-output", "/output"]] or "--unshare-net" not in argv or argv.count("--dev-bind") != 4:
        raise GateV9Error("sandbox writable/network/device grammar differs")
    return argv


def self_check() -> dict[str, Any]:
    policy, digest = read_policy(); validate_static(policy); argv = sandbox_argv(policy)
    return {"status": "PASS_S5B0_REPLAY_EXECUTION_GATE_V9_STATIC_NOT_AUTHORIZED",
            "policy_sha256": digest, "profile_sha256": sha256(PROFILE_PATH),
            "planned_denominator": 1, "sandbox_argv": argv, "device_count": 4,
            "wall_timeout_seconds": 5400, "minimum_available_memory_bytes": 4294967296,
            "minimum_free_vram_bytes": 6442450944, "files_written": False,
            "build_started": False, "replay_started": False, "candidate_executed": False,
            "profile_loaded": False, "sudo_used": False, "gpu_exposed": False,
            "optional_bag_read": False, "external_source_read": False,
            "output_qc_pipeline_frozen": True, "token_full_binding_required": True,
            "renameat2_noreplace_required": True, "journal_boot_cursor_anchor_required": True,
            "stage6_pass": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument("command", choices=("self-check",), nargs="?", default="self-check"); parser.parse_args(argv)
    try: print(json.dumps(self_check(), ensure_ascii=False, sort_keys=True, separators=(",", ":"))); return 0
    except Exception as exc: print(json.dumps({"status": "FAIL_S5B0_REPLAY_EXECUTION_GATE_V9", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=sys.stderr); return 2


if __name__ == "__main__": raise SystemExit(main())
