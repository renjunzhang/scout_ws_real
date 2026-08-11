#!/usr/bin/env python3
"""Exec-capable root-only transport revision for the RTX 5080 C1M GPU smoke."""

from __future__ import annotations

import argparse
import copy
import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

import jsonschema


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
V5_RUNNER_PATH = SCRIPT_PATH.with_name("r8_liquid_u3_gpu_runtime_smoke_v5.py")
REVISION_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_runtime_smoke_v6.json"
REVISION_SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_gpu_runtime_smoke_revision_v6.json"


def load_v5():
    spec = importlib.util.spec_from_file_location("r8_gpu_smoke_v5_for_v6", V5_RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load v5 GPU smoke runner")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


v5 = load_v5()
v4 = v5.v4
v3 = v5.v3
base = v5.base
ORIGINAL_V5_PREPARE_STAGE = v5.prepare_stage
V5_EFFECTIVE_POLICY, _V5_SCHEMA = v5.load_contract()
REVISION = base.read_json(REVISION_PATH)
MAIN_STAGE_ROOT = Path(REVISION["transport"]["stage_root"])
PROBE_STAGE_ROOT = Path(REVISION["transport"]["probe_stage_root"])
PROBE_RECEIPT = Path(REVISION["transport"]["probe_receipt"])
PROBE_OUTPUT_ROOT = Path(REVISION["transport"]["probe_output_root"])


def transport_mount_evidence() -> dict[str, Any]:
    target = Path("/var/tmp")
    matches: list[tuple[int, str, str, set[str]]] = []
    for line in Path("/proc/self/mountinfo").read_text(encoding="utf-8").splitlines():
        fields = line.split()
        separator = fields.index("-")
        mountpoint = fields[4].replace("\\040", " ")
        if str(target) == mountpoint or str(target).startswith(mountpoint.rstrip("/") + "/"):
            options = set(fields[5].split(",")) | set(fields[separator + 3].split(","))
            matches.append((len(mountpoint), mountpoint, fields[separator + 1], options))
    if not matches:
        raise base.SmokeError("cannot resolve /var/tmp mount")
    _, mountpoint, fstype, options = max(matches)
    if fstype != "ext4" or "noexec" in options:
        raise base.SmokeError(
            f"v6 transport mount is not exec-capable ext4: mount={mountpoint} fstype={fstype} options={sorted(options)}"
        )
    metadata = os.lstat(target)
    if not target.is_dir() or metadata.st_uid != 0 or (metadata.st_mode & 0o7777) != 0o1777:
        raise base.SmokeError("/var/tmp identity differs from root-owned sticky directory")
    return {
        "target": str(target),
        "mountpoint": mountpoint,
        "fstype": fstype,
        "noexec": False,
        "directory_uid": metadata.st_uid,
        "directory_gid": metadata.st_gid,
        "directory_mode": "01777",
    }


def load_contract() -> tuple[dict[str, Any], dict[str, Any]]:
    revision = base.read_json(REVISION_PATH)
    schema = base.read_json(REVISION_SCHEMA_PATH)
    try:
        jsonschema.Draft202012Validator.check_schema(schema)
        jsonschema.Draft202012Validator(schema).validate(revision)
    except jsonschema.exceptions.SchemaError as exc:
        raise base.SmokeError(f"invalid v6 closed schema: {exc.message}") from exc
    except jsonschema.exceptions.ValidationError as exc:
        raise base.SmokeError(
            f"v6 revision fails closed schema at {list(exc.absolute_path)}: {exc.message}"
        ) from exc
    for parent in revision["parents"].values():
        if base.sha256_file(Path(parent["path"])) != parent["sha256"]:
            raise base.SmokeError(f"v6 parent drift: {parent['path']}")
    transport_mount_evidence()
    effective = copy.deepcopy(V5_EFFECTIVE_POLICY)
    effective["status"] = "STAGE3_RUNTIME_CONTRACT_V6_EXEC_CAPABLE_TRANSPORT_PENDING_ONE_SHOT"
    effective["run"] = copy.deepcopy(revision["run"])
    effective["sandbox"]["model"] = (
        "root_only_exec_capable_var_tmp_stage_then_uid1000_bwrap_then_o_excl_export"
    )
    return effective, schema


def prepare_stage(policy: dict[str, Any], stage_root: Path) -> dict[str, dict[str, Any]]:
    return ORIGINAL_V5_PREPARE_STAGE(policy, stage_root)


def stage_probe() -> dict[str, Any]:
    policy, _ = load_contract()
    mount = transport_mount_evidence()
    preflight = base.preflight(require_fresh=True)
    if os.path.lexists(PROBE_STAGE_ROOT) or os.path.lexists(PROBE_RECEIPT):
        raise base.SmokeError("v6 access-probe identity is not fresh")
    if os.path.lexists(PROBE_OUTPUT_ROOT.parent):
        raise base.SmokeError("v6 access-probe lab root is not fresh")
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
        "s=dict(line.split(':',1) for line in open('/proc/self/status') if ':' in line); "
        "r={'uid':os.getuid(),'gid':os.getgid(),'groups':os.getgroups(),'cap_eff':s['CapEff'].strip(),"
        "'readable':{x:os.access(x,os.R_OK) for x in p},"
        "'candidate_executable':os.access(p[0],os.X_OK),'heads':heads}; "
        "open('/output/probe.json','x').write(json.dumps(r,sort_keys=True)); print(json.dumps(r,sort_keys=True))"
    )
    command = ["/usr/bin/python3.12", "-I", "-B", "-S", "-c", code]
    argv = v3.bwrap_argv(probe_policy, command=command, include_run=True)
    completed = subprocess.run(
        argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        timeout=30, check=False,
    )
    if completed.returncode != 0:
        raise base.SmokeError(
            f"v6 access probe failed rc={completed.returncode}: {completed.stderr.decode('utf-8', 'replace')!r}"
        )
    report = base.read_json(PROBE_STAGE_ROOT / "output/probe.json")
    if (
        report.get("uid") != 1000 or report.get("gid") != 1000 or report.get("groups") != [1000]
        or report.get("cap_eff") != "0000000000000000"
        or report.get("candidate_executable") is not True
        or set(report.get("readable", {}).values()) != {True}
        or len(report.get("heads", {})) != 4
    ):
        raise base.SmokeError(f"v6 access probe result differs: {report}")
    receipt = {
        "schema_version": "r8-liquid-u3-gpu-smoke-stage-probe-v6",
        "status": "PASS_EXEC_CAPABLE_ROOT_ONLY_TRANSPORT_GUEST_ACCESS_PROBE",
        "captured_at_utc": base.utc_now(),
        "transport_mount": mount,
        "preflight": preflight,
        "argv": argv,
        "returncode": completed.returncode,
        "stdout": completed.stdout.decode("utf-8", "replace"),
        "stderr": completed.stderr.decode("utf-8", "replace"),
        "report": report,
        "candidate_executed": False,
        "network_used": False,
        "project_workspace_writable_in_guest": False,
    }
    receipt_identity = base.write_json_exclusive(PROBE_RECEIPT, receipt)
    receipt["receipt"] = receipt_identity
    return receipt


def self_check() -> dict[str, Any]:
    policy, _ = load_contract()
    base.static_semantic_check(policy)
    result = base.self_check()
    result["status"] = "PASS_U3_GPU_RUNTIME_SMOKE_V6_EXEC_CAPABLE_TRANSPORT_SELF_CHECK"
    result["semantic_delta"] = REVISION["semantic_delta"]
    result["transport_mount"] = transport_mount_evidence()
    return result


for module in (v5, v4):
    module.REVISION_PATH = REVISION_PATH
    module.REVISION_SCHEMA_PATH = REVISION_SCHEMA_PATH
    module.REVISION = REVISION
    module.MAIN_STAGE_ROOT = MAIN_STAGE_ROOT
    module.PROBE_STAGE_ROOT = PROBE_STAGE_ROOT
    module.PROBE_RECEIPT = PROBE_RECEIPT
    module.PROBE_OUTPUT_ROOT = PROBE_OUTPUT_ROOT
    module.load_contract = load_contract
    module.prepare_stage = prepare_stage
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
    REVISION["run"]["stdout_log"], REVISION["run"]["stderr_log"],
    REVISION["run"]["resource_log"], REVISION["run"]["gpu_lock"],
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
            result = {"status": "PASS_U3_GPU_RUNTIME_SMOKE_V6_PREFLIGHT", "evidence": base.preflight(require_fresh=True)}
        elif args.command == "sandbox-probe":
            result = base.sandbox_probe()
            result["status"] = "PASS_U3_GPU_RUNTIME_SMOKE_V6_SANDBOX_PROBE"
        elif args.command == "stage-probe":
            result = stage_probe()
        else:
            os.open = v3.audit_open
            base.os.open = v3.audit_open
            result = base.execute_one_shot()
            result["runtime_revision"] = "v6_exec_capable_root_only_transport"
    except (
        base.SmokeError, OSError, ValueError, subprocess.SubprocessError,
        jsonschema.exceptions.ValidationError, jsonschema.exceptions.SchemaError,
    ) as exc:
        print(
            base.canonical_bytes(
                {"status": "FAIL_U3_GPU_RUNTIME_SMOKE_V6_GATE", "error_type": type(exc).__name__, "error": str(exc)}
            ).decode("utf-8"),
            end="", file=sys.stderr,
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
