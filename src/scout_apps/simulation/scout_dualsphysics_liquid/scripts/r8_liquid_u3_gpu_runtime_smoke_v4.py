#!/usr/bin/env python3
"""Guest-readable staged-input revision for the RTX 5080 C1M GPU smoke.

V3 proved that the root-owned /run transport and isolated GPU sandbox work,
but its root-owned 0400/0500 files were not readable/executable by guest
uid 1000.  V4 keeps the host stage root root:root/0700 and every guest bind
read-only, while descriptor-changing only the four staged files to uid/gid
1000 with 0400 data and 0500 candidate modes.  The candidate is restored to
0400 after the one-shot attempt.
"""

from __future__ import annotations

import argparse
import copy
import importlib.util
import json
import os
import stat
import subprocess
import sys
from pathlib import Path
from typing import Any

import jsonschema


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
V3_RUNNER_PATH = SCRIPT_PATH.with_name("r8_liquid_u3_gpu_runtime_smoke_v3.py")
REVISION_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_runtime_smoke_v4.json"
REVISION_SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_gpu_runtime_smoke_revision_v4.json"


def load_v3():
    spec = importlib.util.spec_from_file_location("r8_gpu_smoke_v3_for_v4", V3_RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load v3 GPU smoke runner")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


v3 = load_v3()
base = v3.base
ORIGINAL_OS_OPEN = v3.ORIGINAL_OS_OPEN
V3_EFFECTIVE_POLICY, _V3_SCHEMA = v3.load_contract()
REVISION = base.read_json(REVISION_PATH)
MAIN_STAGE_ROOT = Path(REVISION["transport"]["stage_root"])
PROBE_STAGE_ROOT = Path(REVISION["transport"]["probe_stage_root"])
PROBE_RECEIPT = Path(REVISION["transport"]["probe_receipt"])
PROBE_OUTPUT_ROOT = Path(REVISION["transport"]["probe_output_root"])


def load_contract() -> tuple[dict[str, Any], dict[str, Any]]:
    revision = base.read_json(REVISION_PATH)
    schema = base.read_json(REVISION_SCHEMA_PATH)
    try:
        jsonschema.Draft202012Validator.check_schema(schema)
        jsonschema.Draft202012Validator(schema).validate(revision)
    except jsonschema.exceptions.SchemaError as exc:
        raise base.SmokeError(f"invalid v4 closed schema: {exc.message}") from exc
    except jsonschema.exceptions.ValidationError as exc:
        raise base.SmokeError(
            f"v4 revision fails closed schema at {list(exc.absolute_path)}: {exc.message}"
        ) from exc
    for parent in revision["parents"].values():
        if base.sha256_file(Path(parent["path"])) != parent["sha256"]:
            raise base.SmokeError(f"v4 parent drift: {parent['path']}")
    effective = copy.deepcopy(V3_EFFECTIVE_POLICY)
    effective["status"] = "STAGE3_RUNTIME_CONTRACT_V4_GUEST_STAGED_INPUTS_PENDING_ONE_SHOT"
    effective["run"] = copy.deepcopy(revision["run"])
    effective["sandbox"]["model"] = (
        "root_owned_run_transport_guest_owned_readonly_files_then_uid1000_bwrap_then_o_excl_export"
    )
    return effective, schema


def copy_guest_file(source_spec: dict[str, Any], destination: Path, mode: int) -> dict[str, Any]:
    root_identity = v3.copy_exclusive(source_spec, destination, mode)
    descriptor = ORIGINAL_OS_OPEN(destination, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if metadata.st_ino != root_identity["inode"] or not stat.S_ISREG(metadata.st_mode):
            raise base.SmokeError(f"v4 staged descriptor identity changed: {destination}")
        os.fchown(descriptor, 1000, 1000)
        os.fchmod(descriptor, mode)
    finally:
        os.close(descriptor)
    result = base.identity(destination)
    expected_mode = f"0{mode:03o}"
    if (
        result["uid"] != 1000
        or result["gid"] != 1000
        or result["mode"] != expected_mode
        or result["sha256"] != source_spec["sha256"]
        or result["size_bytes"] != source_spec["size_bytes"]
    ):
        raise base.SmokeError(f"v4 staged guest file identity differs: {destination}")
    return result


def prepare_stage(policy: dict[str, Any], stage_root: Path) -> dict[str, dict[str, Any]]:
    expected = PROBE_STAGE_ROOT if v3.PROBE_MODE else MAIN_STAGE_ROOT
    if stage_root != expected:
        raise base.SmokeError(f"v4 stage identity changed: {stage_root}")
    if v3.STAGE_ROOT is not None:
        if v3.STAGE_ROOT != stage_root:
            raise base.SmokeError("v4 stage root changed within one process")
        return v3.STAGE_INPUTS
    if os.path.lexists(stage_root):
        raise base.SmokeError(f"v4 stage root is not fresh: {stage_root}")
    v3.mkdir_root(stage_root)
    v3.mkdir_root(stage_root / "runtime")
    v3.mkdir_root(stage_root / "case")
    v3.mkdir_root(stage_root / "output")
    staged = {
        "candidate": copy_guest_file(
            policy["inputs"]["candidate"],
            stage_root / "runtime/DualSPHysics5.4_linux64",
            0o500,
        ),
        "dsph_config": copy_guest_file(
            policy["inputs"]["dsph_config"], stage_root / "runtime/DsphConfig.xml", 0o400
        ),
        "case_bi4": copy_guest_file(
            policy["inputs"]["case_bi4"], stage_root / "case/C1M_zero.bi4", 0o400
        ),
        "case_xml": copy_guest_file(
            policy["inputs"]["case_xml"], stage_root / "case/C1M_zero.xml", 0o400
        ),
    }
    v3.STAGE_INPUTS = staged
    v3.STAGE_ROOT = stage_root
    return staged


def stage_probe() -> dict[str, Any]:
    policy, _ = load_contract()
    preflight = base.preflight(require_fresh=True)
    if os.path.lexists(PROBE_STAGE_ROOT) or os.path.lexists(PROBE_RECEIPT):
        raise base.SmokeError("v4 access-probe identity is not fresh")
    if os.path.lexists(PROBE_OUTPUT_ROOT.parent):
        raise base.SmokeError("v4 access-probe lab root is not fresh")
    base.ensure_directory(PROBE_OUTPUT_ROOT.parent.parent)
    base.ensure_directory(PROBE_OUTPUT_ROOT.parent)
    base.ensure_directory(PROBE_OUTPUT_ROOT)
    probe_policy = copy.deepcopy(policy)
    probe_policy["run"]["output_root"] = str(PROBE_OUTPUT_ROOT)
    v3.PROBE_MODE = True
    code = (
        "import json,os; "
        "p=['/runtime/DualSPHysics5.4_linux64','/runtime/DsphConfig.xml','/case/C1M_zero.bi4','/case/C1M_zero.xml']; "
        "heads={x:open(x,'rb').read(1).hex() for x in p}; "
        "r={'uid':os.getuid(),'gid':os.getgid(),'groups':os.getgroups(),"
        "'readable':{x:os.access(x,os.R_OK) for x in p},"
        "'candidate_executable':os.access(p[0],os.X_OK),'heads':heads}; "
        "open('/output/probe.json','x').write(json.dumps(r,sort_keys=True)); print(json.dumps(r,sort_keys=True))"
    )
    command = ["/usr/bin/python3.12", "-I", "-B", "-S", "-c", code]
    argv = v3.bwrap_argv(probe_policy, command=command, include_run=True)
    completed = subprocess.run(
        argv,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=30,
        check=False,
    )
    if completed.returncode != 0:
        raise base.SmokeError(
            f"v4 access probe failed rc={completed.returncode}: "
            f"{completed.stderr.decode('utf-8', 'replace')!r}"
        )
    report = base.read_json(PROBE_STAGE_ROOT / "output/probe.json")
    if (
        report.get("uid") != 1000
        or report.get("gid") != 1000
        or report.get("groups") != [1000]
        or report.get("candidate_executable") is not True
        or set(report.get("readable", {}).values()) != {True}
        or len(report.get("heads", {})) != 4
    ):
        raise base.SmokeError(f"v4 access probe result differs: {report}")
    receipt = {
        "schema_version": "r8-liquid-u3-gpu-smoke-stage-probe-v4",
        "status": "PASS_ROOT_ONLY_TRANSPORT_GUEST_INPUT_ACCESS_PROBE",
        "captured_at_utc": base.utc_now(),
        "preflight": preflight,
        "argv": argv,
        "returncode": completed.returncode,
        "stdout": completed.stdout.decode("utf-8", "replace"),
        "stderr": completed.stderr.decode("utf-8", "replace"),
        "report": report,
        "candidate_executed": False,
        "network_used": False,
    }
    receipt_identity = base.write_json_exclusive(PROBE_RECEIPT, receipt)
    receipt["receipt"] = receipt_identity
    return receipt


def self_check() -> dict[str, Any]:
    policy, _ = load_contract()
    base.static_semantic_check(policy)
    result = base.self_check()
    result["status"] = "PASS_U3_GPU_RUNTIME_SMOKE_V4_GUEST_STAGED_INPUT_SELF_CHECK"
    result["semantic_delta"] = REVISION["semantic_delta"]
    return result


v3.REVISION_PATH = REVISION_PATH
v3.REVISION_SCHEMA_PATH = REVISION_SCHEMA_PATH
v3.REVISION = REVISION
v3.MAIN_STAGE_ROOT = MAIN_STAGE_ROOT
v3.PROBE_STAGE_ROOT = PROBE_STAGE_ROOT
v3.PROBE_RECEIPT = PROBE_RECEIPT
v3.STAGE_ROOT = None
v3.STAGE_INPUTS = {}
v3.STAGE_EXPORTED = False
v3.PROBE_MODE = False
v3.AUDIT_PATHS = {
    REVISION["run"]["stdout_log"],
    REVISION["run"]["stderr_log"],
    REVISION["run"]["resource_log"],
    REVISION["run"]["gpu_lock"],
}
v3.load_contract = load_contract
v3.prepare_stage = prepare_stage
base.POLICY_PATH = REVISION_PATH
base.SCHEMA_PATH = REVISION_SCHEMA_PATH
base.SCRIPT_PATH = SCRIPT_PATH
base.load_contract = load_contract
base.bwrap_argv = v3.bwrap_argv
base.verify_bwrap_argv = v3.verify_bwrap_argv


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("command", choices=("self-check", "preflight", "sandbox-probe", "stage-probe", "run"))
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    original_open = os.open
    try:
        if args.command == "self-check":
            result = self_check()
        elif args.command == "preflight":
            result = {"status": "PASS_U3_GPU_RUNTIME_SMOKE_V4_PREFLIGHT", "evidence": base.preflight(require_fresh=True)}
        elif args.command == "sandbox-probe":
            result = base.sandbox_probe()
            result["status"] = "PASS_U3_GPU_RUNTIME_SMOKE_V4_SANDBOX_PROBE"
        elif args.command == "stage-probe":
            result = stage_probe()
        else:
            os.open = v3.audit_open
            base.os.open = v3.audit_open
            result = base.execute_one_shot()
            result["runtime_revision"] = "v4_guest_owned_staged_inputs"
    except (
        base.SmokeError,
        OSError,
        ValueError,
        subprocess.SubprocessError,
        jsonschema.exceptions.ValidationError,
        jsonschema.exceptions.SchemaError,
    ) as exc:
        print(
            base.canonical_bytes(
                {"status": "FAIL_U3_GPU_RUNTIME_SMOKE_V4_GATE", "error_type": type(exc).__name__, "error": str(exc)}
            ).decode("utf-8"),
            end="",
            file=sys.stderr,
        )
        return 1
    finally:
        os.open = original_open
        base.os.open = original_open
        stage_identity = v3.secure_stage_postcondition()
        if isinstance(locals().get("result"), dict) and stage_identity is not None:
            result["stage_candidate_after"] = stage_identity
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
