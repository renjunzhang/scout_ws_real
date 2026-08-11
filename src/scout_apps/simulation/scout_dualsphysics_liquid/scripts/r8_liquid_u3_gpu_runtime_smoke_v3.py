#!/usr/bin/env python3
"""Root-owned /run transport revision for the RTX 5080 C1M GPU smoke."""

from __future__ import annotations

import argparse
import copy
import hashlib
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
BASE_RUNNER_PATH = SCRIPT_PATH.with_name("r8_liquid_u3_gpu_runtime_smoke_v1.py")
BASE_POLICY_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_runtime_smoke_v1.json"
BASE_SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_gpu_runtime_smoke_policy_v1.json"
REVISION_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_runtime_smoke_v3.json"
REVISION_SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_gpu_runtime_smoke_revision_v3.json"
PROBE_STAGE_ROOT = Path("/run/r8-liquid-u3-gpu-smoke-v3-stage-probe-20260810t184500z")
PROBE_RECEIPT = Path("/home/zrj/scout_liquid_lab/audits/u3_gpu_smoke_v3_stage_probe_20260810T184500Z.json")


def load_base():
    spec = importlib.util.spec_from_file_location("r8_gpu_smoke_v1_for_v3", BASE_RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load v1 base runner")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


base = load_base()
ORIGINAL_POPEN = subprocess.Popen
ORIGINAL_OS_OPEN = os.open
REVISION = base.read_json(REVISION_PATH)
MAIN_STAGE_ROOT = Path(REVISION["transport"]["stage_root"])
STAGE_ROOT: Path | None = None
STAGE_INPUTS: dict[str, dict[str, Any]] = {}
STAGE_EXPORTED = False
PROBE_MODE = False
AUDIT_PATHS = {
    REVISION["run"]["stdout_log"], REVISION["run"]["stderr_log"],
    REVISION["run"]["resource_log"], REVISION["run"]["gpu_lock"],
}


def load_contract() -> tuple[dict[str, Any], dict[str, Any]]:
    base_policy = base.read_json(BASE_POLICY_PATH)
    base_schema = base.read_json(BASE_SCHEMA_PATH)
    jsonschema.Draft202012Validator.check_schema(base_schema)
    jsonschema.Draft202012Validator(base_schema).validate(base_policy)
    revision = base.read_json(REVISION_PATH)
    schema = base.read_json(REVISION_SCHEMA_PATH)
    jsonschema.Draft202012Validator.check_schema(schema)
    jsonschema.Draft202012Validator(schema).validate(revision)
    for parent in revision["parents"].values():
        if base.sha256_file(Path(parent["path"])) != parent["sha256"]:
            raise base.SmokeError(f"v3 parent drift: {parent['path']}")
    effective = copy.deepcopy(base_policy)
    effective["status"] = "STAGE3_RUNTIME_CONTRACT_V3_RUN_TRANSPORT_PENDING_ONE_SHOT"
    effective["run"] = copy.deepcopy(revision["run"])
    effective["sandbox"]["model"] = "root_owned_run_transport_then_uid1000_bwrap_then_o_excl_export"
    return effective, schema


def mkdir_root(path: Path, mode: int = 0o700) -> None:
    os.mkdir(path, mode)
    os.chown(path, 0, 0)


def copy_exclusive(source_spec: dict[str, Any], destination: Path, mode: int) -> dict[str, Any]:
    source_fd = ORIGINAL_OS_OPEN(source_spec["path"], os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    destination_fd = ORIGINAL_OS_OPEN(
        destination,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        mode,
    )
    digest = hashlib.sha256()
    total = 0
    try:
        source_meta = os.fstat(source_fd)
        if (
            not stat.S_ISREG(source_meta.st_mode) or source_meta.st_nlink != 1
            or source_meta.st_ino != source_spec["inode"] or source_meta.st_size != source_spec["size_bytes"]
        ):
            raise base.SmokeError(f"v3 source identity drift: {source_spec['path']}")
        os.fchown(destination_fd, 0, 0)
        while True:
            block = os.read(source_fd, 1 << 20)
            if not block:
                break
            digest.update(block)
            offset = 0
            while offset < len(block):
                offset += os.write(destination_fd, block[offset:])
            total += len(block)
        os.fsync(destination_fd)
    finally:
        os.close(source_fd)
        os.close(destination_fd)
    if total != source_spec["size_bytes"] or digest.hexdigest() != source_spec["sha256"]:
        raise base.SmokeError(f"v3 staged byte identity differs: {destination}")
    result = base.identity(destination)
    if result["uid"] != 0 or result["gid"] != 0 or result["mode"] != f"0{mode:03o}":
        raise base.SmokeError(f"v3 staged metadata differs: {destination}")
    return result


def prepare_stage(policy: dict[str, Any], stage_root: Path) -> dict[str, dict[str, Any]]:
    global STAGE_ROOT, STAGE_INPUTS
    if STAGE_ROOT is not None:
        if STAGE_ROOT != stage_root:
            raise base.SmokeError("v3 stage identity changed")
        return STAGE_INPUTS
    if os.path.lexists(stage_root):
        raise base.SmokeError(f"v3 stage root is not fresh: {stage_root}")
    mkdir_root(stage_root)
    mkdir_root(stage_root / "runtime")
    mkdir_root(stage_root / "case")
    mkdir_root(stage_root / "output")
    STAGE_INPUTS = {
        "candidate": copy_exclusive(policy["inputs"]["candidate"], stage_root / "runtime/DualSPHysics5.4_linux64", 0o500),
        "dsph_config": copy_exclusive(policy["inputs"]["dsph_config"], stage_root / "runtime/DsphConfig.xml", 0o400),
        "case_bi4": copy_exclusive(policy["inputs"]["case_bi4"], stage_root / "case/C1M_zero.bi4", 0o400),
        "case_xml": copy_exclusive(policy["inputs"]["case_xml"], stage_root / "case/C1M_zero.xml", 0o400),
    }
    STAGE_ROOT = stage_root
    return STAGE_INPUTS


def pairs(argv: list[str], flag: str) -> list[list[str]]:
    return [argv[index + 1:index + 3] for index, value in enumerate(argv) if value == flag]


def verify_bwrap_argv(policy: dict[str, Any], argv: list[str], *, command: list[str], include_run: bool) -> None:
    prefix = [policy["tools"]["bwrap"]["path"]] if os.geteuid() == 0 else [policy["tools"]["sudo"]["path"], "-n", "--", policy["tools"]["bwrap"]["path"]]
    if argv[:len(prefix)] != prefix:
        raise base.SmokeError("v3 bwrap prefix differs")
    for flag in ("--unshare-user", "--unshare-pid", "--unshare-net", "--unshare-ipc", "--unshare-uts", "--cap-drop"):
        if argv.count(flag) != 1:
            raise base.SmokeError(f"v3 isolation option differs: {flag}")
    if "--share-net" in argv or argv[argv.index("--cap-drop") + 1] != "ALL":
        raise base.SmokeError("v3 network/capability boundary differs")
    if pairs(argv, "--dev-bind") != [[path, path] for path in policy["gpu"]["device_paths"]]:
        raise base.SmokeError("v3 GPU device set differs")
    if include_run:
        expected_stage = PROBE_STAGE_ROOT if PROBE_MODE else MAIN_STAGE_ROOT
        if pairs(argv, "--bind") != [[str(expected_stage / "output"), "/output"]]:
            raise base.SmokeError("v3 guest writable bind differs")
        expected_read = [
            ["/usr", "/usr"], ["/etc/ld.so.cache", "/etc/ld.so.cache"], ["/sys", "/sys"],
            [str(expected_stage / "runtime"), "/runtime"], [str(expected_stage / "case"), "/case"],
        ]
        if pairs(argv, "--ro-bind") != expected_read:
            raise base.SmokeError("v3 read-only bind set differs")
    elif pairs(argv, "--bind"):
        raise base.SmokeError("v3 probe unexpectedly has writable bind")
    if argv[-len(command):] != command:
        raise base.SmokeError("v3 guest command differs")


def bwrap_argv(policy: dict[str, Any], *, command: list[str], include_run: bool) -> list[str]:
    prefix = [policy["tools"]["bwrap"]["path"]] if os.geteuid() == 0 else [policy["tools"]["sudo"]["path"], "-n", "--", policy["tools"]["bwrap"]["path"]]
    argv = [
        *prefix, "--die-with-parent", "--new-session", "--unshare-user", "--unshare-pid",
        "--unshare-net", "--unshare-ipc", "--unshare-uts", "--unshare-cgroup-try",
        "--uid", "1000", "--gid", "1000", "--cap-drop", "ALL",
        "--hostname", "r8-liquid-gpu-smoke-v3", "--clearenv",
    ]
    for key, value in policy["solver"]["environment"].items():
        argv.extend(("--setenv", key, value))
    argv.extend((
        "--ro-bind", "/usr", "/usr", "--symlink", "usr/lib", "/lib", "--symlink", "usr/lib64", "/lib64",
        "--dir", "/etc", "--ro-bind", "/etc/ld.so.cache", "/etc/ld.so.cache", "--proc", "/proc", "--dev", "/dev",
    ))
    for device in policy["gpu"]["device_paths"]:
        argv.extend(("--dev-bind", device, device))
    argv.extend(("--ro-bind", "/sys", "/sys", "--tmpfs", "/tmp"))
    if include_run:
        stage_root = PROBE_STAGE_ROOT if PROBE_MODE else MAIN_STAGE_ROOT
        if os.path.isdir(policy["run"]["output_root"]):
            prepare_stage(policy, stage_root)
        argv.extend((
            "--ro-bind", str(stage_root / "runtime"), "/runtime",
            "--ro-bind", str(stage_root / "case"), "/case",
            "--bind", str(stage_root / "output"), "/output",
            "--chdir", "/runtime",
        ))
    else:
        argv.extend(("--chdir", "/tmp"))
    argv.extend(("--", *command))
    verify_bwrap_argv(policy, argv, command=command, include_run=include_run)
    return argv


def copy_output_file(source: Path, destination: Path) -> dict[str, Any]:
    source_fd = ORIGINAL_OS_OPEN(source, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    destination_fd = ORIGINAL_OS_OPEN(destination, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
    digest = hashlib.sha256()
    total = 0
    try:
        metadata = os.fstat(source_fd)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or not 0 < metadata.st_size <= 32 * 1024 * 1024:
            raise base.SmokeError(f"unsafe v3 stage output: {source}")
        os.fchown(destination_fd, 1000, 1000)
        while True:
            block = os.read(source_fd, 1 << 20)
            if not block:
                break
            digest.update(block)
            offset = 0
            while offset < len(block):
                offset += os.write(destination_fd, block[offset:])
            total += len(block)
        os.fsync(destination_fd)
    finally:
        os.close(source_fd)
        os.close(destination_fd)
    return {"path": str(destination), "size_bytes": total, "sha256": digest.hexdigest()}


def export_stage_outputs(policy: dict[str, Any]) -> None:
    global STAGE_EXPORTED
    if STAGE_EXPORTED:
        return
    if STAGE_ROOT != MAIN_STAGE_ROOT:
        raise base.SmokeError("v3 main stage identity is absent at export")
    stage_output = MAIN_STAGE_ROOT / "output"
    root_names = {entry.name for entry in os.scandir(stage_output)}
    if root_names != base.EXPECTED_ROOT_FILES | {"data"}:
        raise base.SmokeError(f"v3 stage output root inventory differs: {sorted(root_names)}")
    data_names = {entry.name for entry in os.scandir(stage_output / "data")}
    if data_names != base.EXPECTED_DATA_FILES:
        raise base.SmokeError(f"v3 stage output data inventory differs: {sorted(data_names)}")
    destination = Path(policy["run"]["output_root"])
    if list(os.scandir(destination)):
        raise base.SmokeError("v3 lab output is not empty before export")
    data_destination = destination / "data"
    os.mkdir(data_destination, 0o700)
    os.chown(data_destination, 1000, 1000)
    for name in sorted(base.EXPECTED_ROOT_FILES):
        copy_output_file(stage_output / name, destination / name)
    for name in sorted(base.EXPECTED_DATA_FILES):
        copy_output_file(stage_output / "data" / name, data_destination / name)
    STAGE_EXPORTED = True


class ExportingProcess:
    def __init__(self, process: subprocess.Popen[bytes], policy: dict[str, Any]):
        self._process = process
        self._policy = policy
        self.pid = process.pid
        self._exported = False

    def poll(self):
        return self._process.poll()

    def wait(self, *args, **kwargs):
        returncode = self._process.wait(*args, **kwargs)
        if returncode == 0 and not self._exported:
            export_stage_outputs(self._policy)
            self._exported = True
        return returncode


def transport_popen(args, *pargs, **kwargs):
    process = ORIGINAL_POPEN(args, *pargs, **kwargs)
    if (
        not PROBE_MODE and isinstance(args, (list, tuple))
        and str(MAIN_STAGE_ROOT / "output") in args and "--bind" in args
    ):
        policy, _ = load_contract()
        return ExportingProcess(process, policy)
    return process


def audit_open(path, flags, mode=0o777, *, dir_fd=None):
    descriptor = ORIGINAL_OS_OPEN(path, flags, mode, dir_fd=dir_fd)
    if os.geteuid() == 0 and flags & os.O_CREAT and str(path) in AUDIT_PATHS:
        os.fchown(descriptor, 1000, 1000)
    return descriptor


def secure_stage_postcondition() -> dict[str, Any] | None:
    if STAGE_ROOT is None or not STAGE_ROOT.exists():
        return None
    candidate = STAGE_ROOT / "runtime/DualSPHysics5.4_linux64"
    if candidate.exists():
        descriptor = ORIGINAL_OS_OPEN(candidate, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
        try:
            os.fchmod(descriptor, 0o400)
        finally:
            os.close(descriptor)
    for base_dir, dirs, files in os.walk(STAGE_ROOT, topdown=False, followlinks=False):
        for name in files:
            path = Path(base_dir) / name
            if path != candidate:
                os.chmod(path, 0o400, follow_symlinks=False)
            os.chown(path, 1000, 1000, follow_symlinks=False)
        for name in dirs:
            path = Path(base_dir) / name
            os.chmod(path, 0o700, follow_symlinks=False)
            os.chown(path, 1000, 1000, follow_symlinks=False)
    os.chmod(STAGE_ROOT, 0o700)
    os.chown(STAGE_ROOT, 1000, 1000)
    return base.identity(candidate) if candidate.exists() else None


base.POLICY_PATH = REVISION_PATH
base.SCHEMA_PATH = REVISION_SCHEMA_PATH
base.SCRIPT_PATH = SCRIPT_PATH
base.load_contract = load_contract
base.bwrap_argv = bwrap_argv
base.verify_bwrap_argv = verify_bwrap_argv
base.subprocess.Popen = transport_popen


def stage_probe() -> dict[str, Any]:
    global PROBE_MODE
    policy, _ = load_contract()
    preflight = base.preflight(require_fresh=True)
    if os.path.lexists(PROBE_STAGE_ROOT) or os.path.lexists(PROBE_RECEIPT):
        raise base.SmokeError("v3 stage probe identity is not fresh")
    probe_output = Path("/home/zrj/scout_liquid_lab/probes/u3_gpu_smoke_v3_stage_probe_20260810T184500Z.partial/output")
    if os.path.lexists(probe_output.parent):
        raise base.SmokeError("v3 stage probe lab root is not fresh")
    base.ensure_directory(probe_output.parent.parent)
    base.ensure_directory(probe_output.parent)
    base.ensure_directory(probe_output)
    probe_policy = copy.deepcopy(policy)
    probe_policy["run"]["output_root"] = str(probe_output)
    PROBE_MODE = True
    code = (
        "import json,os; p=['/runtime/DualSPHysics5.4_linux64','/runtime/DsphConfig.xml','/case/C1M_zero.bi4','/case/C1M_zero.xml']; "
        "r={'uid':os.getuid(),'gid':os.getgid(),'sizes':{x:os.stat(x).st_size for x in p}}; "
        "open('/output/probe.json','x').write(json.dumps(r,sort_keys=True)); print(json.dumps(r,sort_keys=True))"
    )
    command = ["/usr/bin/python3.12", "-I", "-B", "-S", "-c", code]
    argv = bwrap_argv(probe_policy, command=command, include_run=True)
    completed = subprocess.run(argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=30, check=False)
    if completed.returncode != 0:
        raise base.SmokeError(f"v3 stage probe failed rc={completed.returncode}: {completed.stderr.decode('utf-8','replace')!r}")
    stage_report = base.read_json(PROBE_STAGE_ROOT / "output/probe.json")
    if stage_report.get("uid") != 1000 or stage_report.get("gid") != 1000:
        raise base.SmokeError(f"v3 stage probe identity differs: {stage_report}")
    receipt = {
        "schema_version": "r8-liquid-u3-gpu-smoke-stage-probe-v3", "status": "PASS_ROOT_OWNED_RUN_TRANSPORT_PROBE",
        "captured_at_utc": base.utc_now(), "preflight": preflight, "argv": argv,
        "returncode": completed.returncode, "stdout": completed.stdout.decode("utf-8","replace"),
        "stderr": completed.stderr.decode("utf-8","replace"), "stage_report": stage_report,
        "candidate_executed": False, "network_used": False,
    }
    publication = base.write_json_exclusive(PROBE_RECEIPT, receipt)
    receipt["receipt"] = publication
    return receipt


def self_check() -> dict[str, Any]:
    policy, _ = load_contract()
    base.static_semantic_check(policy)
    result = base.self_check()
    result["status"] = "PASS_U3_GPU_RUNTIME_SMOKE_V3_RUN_TRANSPORT_SELF_CHECK"
    return result


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
            result = {"status": "PASS_U3_GPU_RUNTIME_SMOKE_V3_PREFLIGHT", "evidence": base.preflight(require_fresh=True)}
        elif args.command == "sandbox-probe":
            result = base.sandbox_probe()
        elif args.command == "stage-probe":
            result = stage_probe()
        else:
            os.open = audit_open
            base.os.open = audit_open
            result = base.execute_one_shot()
            result["runtime_revision"] = "v3_root_owned_run_transport"
    except (base.SmokeError, OSError, ValueError, subprocess.SubprocessError, jsonschema.exceptions.ValidationError, jsonschema.exceptions.SchemaError) as exc:
        print(base.canonical_bytes({"status": "FAIL_U3_GPU_RUNTIME_SMOKE_V3_GATE", "error_type": type(exc).__name__, "error": str(exc)}).decode("utf-8"), end="", file=sys.stderr)
        return 1
    finally:
        os.open = original_open
        base.os.open = original_open
        stage_identity = secure_stage_postcondition()
        if isinstance(locals().get("result"), dict) and stage_identity is not None:
            result["stage_candidate_after"] = stage_identity
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
