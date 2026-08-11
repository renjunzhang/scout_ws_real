#!/usr/bin/env python3
"""FD-pinned revision of the U3 RTX 5080 GPU smoke runner.

V1 reached bwrap but its post-userns source-path lookup could not traverse the
mode-0700 build hierarchy.  V2 preserves every guest mount, device, flag and
limit, while replacing four input path binds and one output path bind with
O_NOFOLLOW-opened descriptors and bwrap's --ro-bind-fd/--bind-fd interface.
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
BASE_RUNNER_PATH = SCRIPT_PATH.with_name("r8_liquid_u3_gpu_runtime_smoke_v1.py")
BASE_POLICY_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_runtime_smoke_v1.json"
BASE_SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_gpu_runtime_smoke_policy_v1.json"
REVISION_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_runtime_smoke_v2.json"
REVISION_SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_gpu_runtime_smoke_revision_v2.json"


def load_base_module():
    spec = importlib.util.spec_from_file_location("r8_gpu_smoke_v1_frozen", BASE_RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load frozen v1 runner")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


base = load_base_module()
ORIGINAL_POPEN = subprocess.Popen
ORIGINAL_OS_OPEN = os.open
ACTIVE_FDS: list[int] = []
REVISION_RUN = base.read_json(REVISION_PATH)["run"]
AUDIT_CREATE_PATHS = {
    REVISION_RUN["stdout_log"], REVISION_RUN["stderr_log"],
    REVISION_RUN["resource_log"], REVISION_RUN["gpu_lock"],
}
FD_PROBE_ROOT = Path("/home/zrj/scout_liquid_lab/probes/u3_gpu_smoke_v2_fd_probe_20260810T183800Z.partial")
FD_PROBE_OUTPUT = FD_PROBE_ROOT / "output"
FD_PROBE_RECEIPT = Path("/home/zrj/scout_liquid_lab/audits/u3_gpu_smoke_v2_fd_probe_20260810T183800Z.json")


def load_contract() -> tuple[dict[str, Any], dict[str, Any]]:
    base_policy = base.read_json(BASE_POLICY_PATH)
    base_schema = base.read_json(BASE_SCHEMA_PATH)
    jsonschema.Draft202012Validator.check_schema(base_schema)
    jsonschema.Draft202012Validator(base_schema).validate(base_policy)

    revision = base.read_json(REVISION_PATH)
    revision_schema = base.read_json(REVISION_SCHEMA_PATH)
    jsonschema.Draft202012Validator.check_schema(revision_schema)
    jsonschema.Draft202012Validator(revision_schema).validate(revision)
    for parent in revision["parents"].values():
        observed = base.sha256_file(Path(parent["path"]), maximum=base.MAX_IDENTITY_BYTES)
        if observed != parent["sha256"]:
            raise base.SmokeError(f"v2 parent drift: {parent['path']}")

    effective = copy.deepcopy(base_policy)
    effective["status"] = "STAGE3_RUNTIME_CONTRACT_V2_FD_PINNED_PENDING_ONE_SHOT"
    effective["run"] = copy.deepcopy(revision["run"])
    effective["sandbox"]["model"] = "sudo_root_bwrap_fd_pinned_mount_namespace_drop_to_uid1000"
    return effective, revision_schema


def open_regular_fd(spec: dict[str, Any]) -> int:
    descriptor = ORIGINAL_OS_OPEN(spec["path"], os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if (
            not stat.S_ISREG(metadata.st_mode)
            or metadata.st_nlink != 1
            or metadata.st_ino != spec["inode"]
            or metadata.st_size != spec["size_bytes"]
        ):
            raise base.SmokeError(f"FD-pinned input identity differs: {spec['path']}")
        return descriptor
    except Exception:
        os.close(descriptor)
        raise


def open_bind_fds(policy: dict[str, Any]) -> dict[str, int]:
    descriptors = {
        "candidate": open_regular_fd(policy["inputs"]["candidate"]),
        "dsph_config": open_regular_fd(policy["inputs"]["dsph_config"]),
        "case_bi4": open_regular_fd(policy["inputs"]["case_bi4"]),
        "case_xml": open_regular_fd(policy["inputs"]["case_xml"]),
    }
    try:
        output_fd = ORIGINAL_OS_OPEN(
            policy["run"]["output_root"],
            os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
        )
        metadata = os.fstat(output_fd)
        if not stat.S_ISDIR(metadata.st_mode):
            raise base.SmokeError("FD-pinned output is not a directory")
        descriptors["output"] = output_fd
        return descriptors
    except Exception:
        for descriptor in descriptors.values():
            os.close(descriptor)
        raise


def _pairs(argv: list[str], flag: str) -> list[list[str]]:
    return [argv[index + 1:index + 3] for index, value in enumerate(argv) if value == flag]


def verify_bwrap_argv(policy: dict[str, Any], argv: list[str], *, command: list[str], include_run: bool) -> None:
    expected_prefix = (
        [policy["tools"]["bwrap"]["path"]]
        if os.geteuid() == 0
        else [policy["tools"]["sudo"]["path"], "-n", "--", policy["tools"]["bwrap"]["path"]]
    )
    if argv[:len(expected_prefix)] != expected_prefix:
        raise base.SmokeError("v2 bwrap prefix differs")
    for required in ("--unshare-user", "--unshare-pid", "--unshare-net", "--unshare-ipc", "--unshare-uts", "--cap-drop"):
        if argv.count(required) != 1:
            raise base.SmokeError(f"v2 sandbox option count differs: {required}")
    if "--share-net" in argv or argv[argv.index("--cap-drop") + 1] != "ALL":
        raise base.SmokeError("v2 network/capability isolation differs")
    if _pairs(argv, "--dev-bind") != [[path, path] for path in policy["gpu"]["device_paths"]]:
        raise base.SmokeError("v2 GPU device set differs")
    if _pairs(argv, "--bind"):
        raise base.SmokeError("v2 retains a host path writable bind")
    if include_run:
        if len(_pairs(argv, "--ro-bind-fd")) != 4 or len(_pairs(argv, "--bind-fd")) != 1:
            raise base.SmokeError("v2 FD bind count differs")
        if [item[1] for item in _pairs(argv, "--ro-bind-fd")] != [
            "/runtime/DualSPHysics5.4_linux64", "/runtime/DsphConfig.xml",
            "/case/C1M_zero.bi4", "/case/C1M_zero.xml",
        ]:
            raise base.SmokeError("v2 input FD destinations differ")
        if _pairs(argv, "--bind-fd")[0][1] != "/output":
            raise base.SmokeError("v2 output FD destination differs")
    elif _pairs(argv, "--ro-bind-fd") or _pairs(argv, "--bind-fd"):
        raise base.SmokeError("v2 probe unexpectedly contains input/output FDs")
    if argv[-len(command):] != command:
        raise base.SmokeError("v2 guest command differs")


def bwrap_argv(policy: dict[str, Any], *, command: list[str], include_run: bool) -> list[str]:
    prefix = (
        [policy["tools"]["bwrap"]["path"]]
        if os.geteuid() == 0
        else [policy["tools"]["sudo"]["path"], "-n", "--", policy["tools"]["bwrap"]["path"]]
    )
    argv = [
        *prefix,
        "--die-with-parent", "--new-session", "--unshare-user", "--unshare-pid",
        "--unshare-net", "--unshare-ipc", "--unshare-uts", "--unshare-cgroup-try",
        "--uid", "1000", "--gid", "1000", "--cap-drop", "ALL",
        "--hostname", "r8-liquid-gpu-smoke-v2", "--clearenv",
    ]
    for key, value in policy["solver"]["environment"].items():
        argv.extend(("--setenv", key, value))
    argv.extend((
        "--ro-bind", "/usr", "/usr", "--symlink", "usr/lib", "/lib",
        "--symlink", "usr/lib64", "/lib64", "--dir", "/etc",
        "--ro-bind", "/etc/ld.so.cache", "/etc/ld.so.cache", "--proc", "/proc",
        "--dev", "/dev",
    ))
    for device in policy["gpu"]["device_paths"]:
        argv.extend(("--dev-bind", device, device))
    argv.extend(("--ro-bind", "/sys", "/sys", "--tmpfs", "/tmp"))
    if include_run:
        output_exists = os.path.isdir(policy["run"]["output_root"])
        fd_map = open_bind_fds(policy) if output_exists else {
            "candidate": 200, "dsph_config": 201, "case_bi4": 202, "case_xml": 203, "output": 204
        }
        if output_exists:
            if ACTIVE_FDS:
                raise base.SmokeError("v2 FD set was already active")
            ACTIVE_FDS.extend(fd_map.values())
        argv.extend((
            "--dir", "/runtime",
            "--ro-bind-fd", str(fd_map["candidate"]), "/runtime/DualSPHysics5.4_linux64",
            "--ro-bind-fd", str(fd_map["dsph_config"]), "/runtime/DsphConfig.xml",
            "--dir", "/case",
            "--ro-bind-fd", str(fd_map["case_bi4"]), "/case/C1M_zero.bi4",
            "--ro-bind-fd", str(fd_map["case_xml"]), "/case/C1M_zero.xml",
            "--dir", "/output",
            "--bind-fd", str(fd_map["output"]), "/output",
            "--chdir", "/runtime",
        ))
    else:
        argv.extend(("--chdir", "/tmp"))
    argv.extend(("--", *command))
    verify_bwrap_argv(policy, argv, command=command, include_run=include_run)
    return argv


def pinned_popen(args, *pargs, **kwargs):
    is_fd_bwrap = isinstance(args, (list, tuple)) and "--ro-bind-fd" in args
    if not is_fd_bwrap:
        return ORIGINAL_POPEN(args, *pargs, **kwargs)
    descriptors = tuple(ACTIVE_FDS)
    if len(descriptors) != 5:
        raise base.SmokeError(f"v2 active FD count differs: {len(descriptors)}")
    kwargs["pass_fds"] = descriptors
    kwargs["close_fds"] = True
    try:
        return ORIGINAL_POPEN(args, *pargs, **kwargs)
    finally:
        for descriptor in descriptors:
            try:
                os.close(descriptor)
            except OSError:
                pass
        ACTIVE_FDS.clear()


def audit_open(path, flags, mode=0o777, *, dir_fd=None):
    descriptor = ORIGINAL_OS_OPEN(path, flags, mode, dir_fd=dir_fd)
    if os.geteuid() == 0 and flags & os.O_CREAT and str(path) in AUDIT_CREATE_PATHS:
        os.fchown(descriptor, 1000, 1000)
    return descriptor


def close_active_fds() -> None:
    for descriptor in tuple(ACTIVE_FDS):
        try:
            os.close(descriptor)
        except OSError:
            pass
    ACTIVE_FDS.clear()


base.POLICY_PATH = REVISION_PATH
base.SCHEMA_PATH = REVISION_SCHEMA_PATH
base.SCRIPT_PATH = SCRIPT_PATH
base.load_contract = load_contract
base.bwrap_argv = bwrap_argv
base.verify_bwrap_argv = verify_bwrap_argv
base.subprocess.Popen = pinned_popen


def self_check() -> dict[str, Any]:
    policy, _ = load_contract()
    base.static_semantic_check(policy)
    result = base.self_check()
    result["status"] = "PASS_U3_GPU_RUNTIME_SMOKE_V2_FD_PINNED_SELF_CHECK"
    result["revision"] = base.identity(REVISION_PATH, maximum=4 * 1024 * 1024)
    return result


def fd_probe() -> dict[str, Any]:
    preflight = base.preflight(require_fresh=True)
    if any(os.path.lexists(path) for path in (FD_PROBE_ROOT, FD_PROBE_RECEIPT)):
        raise base.SmokeError("v2 FD probe identity is not fresh")
    policy, _ = load_contract()
    probe_policy = copy.deepcopy(policy)
    probe_policy["run"]["attempt_root"] = str(FD_PROBE_ROOT)
    probe_policy["run"]["output_root"] = str(FD_PROBE_OUTPUT)
    base.ensure_directory(FD_PROBE_ROOT.parent)
    base.ensure_directory(FD_PROBE_ROOT)
    base.ensure_directory(FD_PROBE_OUTPUT)
    code = (
        "import json,os; "
        "paths=['/runtime/DualSPHysics5.4_linux64','/runtime/DsphConfig.xml','/case/C1M_zero.bi4','/case/C1M_zero.xml']; "
        "r={'uid':os.getuid(),'gid':os.getgid(),'groups':os.getgroups(),'files':{p:{'size':os.stat(p).st_size,'head':open(p,'rb').read(16).hex()} for p in paths}}; "
        "open('/output/fd_probe.json','x').write(json.dumps(r,sort_keys=True)); print(json.dumps(r,sort_keys=True))"
    )
    command = ["/usr/bin/python3.12", "-I", "-B", "-S", "-c", code]
    argv = bwrap_argv(probe_policy, command=command, include_run=True)
    completed = subprocess.run(
        argv,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=30,
        check=False,
    )
    base.normalize_output_ownership(FD_PROBE_OUTPUT)
    output_path = FD_PROBE_OUTPUT / "fd_probe.json"
    if completed.returncode != 0 or not output_path.exists():
        raise base.SmokeError(
            f"v2 FD probe failed rc={completed.returncode}: {completed.stderr.decode('utf-8', 'replace')!r}"
        )
    output = base.read_json(output_path)
    if output.get("uid") != 1000 or output.get("gid") != 1000 or len(output.get("files", {})) != 4:
        raise base.SmokeError(f"v2 FD probe output differs: {output!r}")
    expected_sizes = {
        "/runtime/DualSPHysics5.4_linux64": policy["inputs"]["candidate"]["size_bytes"],
        "/runtime/DsphConfig.xml": policy["inputs"]["dsph_config"]["size_bytes"],
        "/case/C1M_zero.bi4": policy["inputs"]["case_bi4"]["size_bytes"],
        "/case/C1M_zero.xml": policy["inputs"]["case_xml"]["size_bytes"],
    }
    if {name: value["size"] for name, value in output["files"].items()} != expected_sizes:
        raise base.SmokeError("v2 FD probe input sizes differ")
    receipt = {
        "schema_version": "r8-liquid-u3-gpu-runtime-smoke-fd-probe-v2",
        "document_type": "SMPCC_R8_LIQUID_U3_GPU_RUNTIME_SMOKE_FD_PROBE_V2",
        "status": "PASS_FD_PINNED_INPUT_OUTPUT_SANDBOX_PROBE",
        "captured_at_utc": base.utc_now(),
        "preflight": preflight,
        "argv": argv,
        "returncode": completed.returncode,
        "stdout": completed.stdout.decode("utf-8", "replace"),
        "stderr": completed.stderr.decode("utf-8", "replace"),
        "output": output,
        "output_identity": base.identity(output_path, maximum=1024 * 1024),
        "candidate_executed": False,
        "network_used": False,
    }
    publication = base.write_json_exclusive(FD_PROBE_RECEIPT, receipt)
    receipt["receipt"] = publication
    return receipt


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("command", choices=("self-check", "preflight", "sandbox-probe", "fd-probe", "run"))
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    original_open = os.open
    try:
        if args.command == "self-check":
            result = self_check()
        elif args.command == "preflight":
            result = {"status": "PASS_U3_GPU_RUNTIME_SMOKE_V2_PREFLIGHT", "evidence": base.preflight(require_fresh=True)}
        elif args.command == "sandbox-probe":
            result = base.sandbox_probe()
            result["status"] = "PASS_EPHEMERAL_GPU_SANDBOX_V2_FD_CONTRACT_PROBE"
        elif args.command == "fd-probe":
            result = fd_probe()
        else:
            os.open = audit_open
            base.os.open = audit_open
            result = base.execute_one_shot()
            result["runtime_revision"] = "v2_fd_pinned"
    except (base.SmokeError, OSError, ValueError, subprocess.SubprocessError, jsonschema.exceptions.ValidationError, jsonschema.exceptions.SchemaError) as exc:
        print(base.canonical_bytes({"status": "FAIL_U3_GPU_RUNTIME_SMOKE_V2_GATE", "error_type": type(exc).__name__, "error": str(exc)}).decode("utf-8"), end="", file=sys.stderr)
        return 1
    finally:
        os.open = original_open
        base.os.open = original_open
        close_active_fds()
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
