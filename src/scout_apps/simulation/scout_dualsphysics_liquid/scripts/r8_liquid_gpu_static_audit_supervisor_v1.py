#!/usr/bin/env python3
"""Root-only, byte-pinned lifecycle wrapper for one static-audit run.

Root is used solely to add/remove the exact AppArmor profile and to drop to
uid/gid 1000 with zero supplementary groups.  ELF/CUDA parsers run only after
the privilege drop and only through aa-exec+bwrap as constructed by the gate.
"""

from __future__ import annotations

import hashlib
import json
import os
import stat
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence


WORKSPACE = Path("/home/zrj/scout_ws")
PACKAGE = WORKSPACE / "src/scout_apps/simulation/scout_dualsphysics_liquid"
POLICY_PATH = PACKAGE / (
    "config/target_hosts/"
    "liquid_zrj_msi_u2404_gpu_static_audit_20260810T170339Z_v1.json"
)
POLICY_SHA256 = "e81b735acb74d3a73fd81a384576f65ffb1835e18f2caecbbad7a93de0d7e347"
GATE_PATH = PACKAGE / "scripts/r8_liquid_gpu_static_audit_gate_v1.py"
GATE_SHA256 = "c21ef005e4375ec24c92fd19bec4fabd3e11ba630b6a5b34a6c2981a4e7cbe28"
PROFILE_PATH = PACKAGE / (
    "config/apparmor_drafts/"
    "r8-liquid-u3-gpu-static-audit-20260810T170339Z.profile"
)
PROFILE_NAME = "r8-liquid-u3-gpu-static-audit-20260810T170339Z"
PROFILE_SHA256 = "d38092826e9b17d59c20d9ef1be3b30517c5cc106e316c394877c45c72f693ed"
APPARMOR_PARSER = Path("/usr/sbin/apparmor_parser")
APPARMOR_PARSER_SHA256 = "6bc852b37807961c14976be9a227ae96bd817f73b5189cb0e0ff5eca4448c01c"
AA_STATUS = Path("/usr/sbin/aa-status")
AA_STATUS_SHA256 = "af9f579ad039f0e669bfb15735c55efd17896f838d684e7f9be15b09fa1e2dd4"
SETPRIV = Path("/usr/bin/setpriv")
SETPRIV_SHA256 = "96b083b79c32fd2f0c29657e88e20c7495839349fc64ad5d0503f32d26bf8733"


class SupervisorError(RuntimeError):
    pass


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def sha256_regular(path: Path) -> str:
    info = path.lstat()
    if path.is_symlink() or not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise SupervisorError(f"unsafe pinned file: {path}")
    digest = hashlib.sha256()
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        while True:
            block = os.read(descriptor, 1024 * 1024)
            if not block:
                break
            digest.update(block)
    finally:
        os.close(descriptor)
    return digest.hexdigest()


def write_all(descriptor: int, data: bytes) -> None:
    offset = 0
    while offset < len(data):
        written = os.write(descriptor, data[offset:])
        if written <= 0:
            raise SupervisorError("short lifecycle receipt write")
        offset += written


def write_json_new(path: Path, value: Mapping[str, Any]) -> None:
    payload = json.dumps(value, sort_keys=True, indent=2, ensure_ascii=False).encode("utf-8") + b"\n"
    descriptor = os.open(
        path,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        0o640,
    )
    try:
        write_all(descriptor, payload)
        os.fsync(descriptor)
        os.fchmod(descriptor, 0o640)
        os.fchown(descriptor, 1000, 1000)
    finally:
        os.close(descriptor)


def run(argv: Sequence[str], *, timeout: int, capture_limit: int = 65536) -> dict[str, Any]:
    started = time.monotonic()
    completed = subprocess.run(
        argv,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        check=False,
        cwd=WORKSPACE,
    )
    return {
        "argv": list(argv),
        "returncode": completed.returncode,
        "elapsed_seconds": round(time.monotonic() - started, 6),
        "stdout_tail_utf8": completed.stdout[-capture_limit:].decode("utf-8", "replace"),
        "stderr_tail_utf8": completed.stderr[-capture_limit:].decode("utf-8", "replace"),
        "stdout_bytes": len(completed.stdout),
        "stderr_bytes": len(completed.stderr),
        "stdout_sha256": hashlib.sha256(completed.stdout).hexdigest(),
        "stderr_sha256": hashlib.sha256(completed.stderr).hexdigest(),
    }


def profile_loaded(name: str) -> bool:
    result = run([str(AA_STATUS)], timeout=30)
    if result["returncode"] != 0:
        raise SupervisorError(f"aa-status failed: {result['stderr_tail_utf8']}")
    return any(line.strip() == name for line in result["stdout_tail_utf8"].splitlines())


def verify_pins() -> dict[str, str]:
    pins = {
        str(POLICY_PATH): POLICY_SHA256,
        str(GATE_PATH): GATE_SHA256,
        str(PROFILE_PATH): PROFILE_SHA256,
        str(APPARMOR_PARSER): APPARMOR_PARSER_SHA256,
        str(AA_STATUS): AA_STATUS_SHA256,
        str(SETPRIV): SETPRIV_SHA256,
    }
    for path_text, digest in pins.items():
        if sha256_regular(Path(path_text)) != digest:
            raise SupervisorError(f"pinned lifecycle input drift: {path_text}")
    return pins


def main() -> int:
    if (
        os.geteuid() != 0
        or os.environ.get("SUDO_UID") != "1000"
        or os.environ.get("SUDO_GID") != "1000"
    ):
        print("must be invoked once through sudo by uid/gid 1000", file=sys.stderr)
        return 2
    try:
        pins = verify_pins()
        policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
        if policy["profile"] != {
            "name": PROFILE_NAME,
            "path": "src/scout_apps/simulation/scout_dualsphysics_liquid/config/apparmor_drafts/r8-liquid-u3-gpu-static-audit-20260810T170339Z.profile",
            "mode_octal": "0600",
            "size_bytes": 61078,
            "sha256": PROFILE_SHA256,
        }:
            raise SupervisorError("policy exact-profile identity drift")
        lifecycle_path = Path(policy["outputs"]["lifecycle_receipt"])
        if os.path.lexists(lifecycle_path):
            raise SupervisorError("create-new lifecycle receipt already exists")
        if profile_loaded(PROFILE_NAME):
            raise SupervisorError("static-audit profile already loaded")
    except (OSError, KeyError, ValueError, json.JSONDecodeError, SupervisorError) as exc:
        print(f"static-audit supervisor preflight failed: {exc}", file=sys.stderr)
        return 2

    events: list[dict[str, Any]] = []
    child_returncode: int | None = None
    unload_failure: str | None = None
    load = run(
        [str(APPARMOR_PARSER), "-K", "-T", "-a", "--", str(PROFILE_PATH)],
        timeout=60,
    )
    events.append({"action": "load", **load})
    if load["returncode"] != 0:
        write_json_new(
            lifecycle_path,
            {
                "status": "FAIL_PROFILE_LOAD",
                "created_at": utc_now(),
                "profile_name": PROFILE_NAME,
                "profile_path": str(PROFILE_PATH),
                "profile_sha256": PROFILE_SHA256,
                "pins": pins,
                "events": events,
                "child_started": False,
                "zero_residue": not profile_loaded(PROFILE_NAME),
            },
        )
        return 1

    try:
        if not profile_loaded(PROFILE_NAME):
            raise SupervisorError("profile absent immediately after load")
        events.append({"action": "verify_loaded", "returncode": 0, "loaded": True})
        child_argv = [
            str(SETPRIV),
            "--reuid=1000",
            "--regid=1000",
            "--clear-groups",
            "--",
            "/usr/bin/env",
            "PYTHONDONTWRITEBYTECODE=1",
            f"R8_EXACT_PROFILE_PRELOADED_SHA256={PROFILE_SHA256}",
            "/usr/bin/python3",
            str(GATE_PATH),
            "run",
        ]
        child = run(child_argv, timeout=1200)
        child_returncode = child["returncode"]
        events.append({"action": "uid1000_zero_group_static_audit_gate", **child})
    except (OSError, subprocess.SubprocessError, SupervisorError) as exc:
        child_returncode = 1
        events.append({"action": "supervisor_exception", "returncode": 1, "error": str(exc)})
    finally:
        unload = run(
            [str(APPARMOR_PARSER), "-K", "-T", "-R", "--", str(PROFILE_PATH)],
            timeout=60,
        )
        events.append({"action": "unload", **unload})
        if unload["returncode"] != 0:
            unload_failure = unload["stderr_tail_utf8"] or "apparmor_parser remove failed"
        residue = profile_loaded(PROFILE_NAME)
        events.append({"action": "verify_zero_residue", "returncode": 0 if not residue else 1, "loaded": residue})
        if residue:
            unload_failure = "static-audit profile residue remains loaded"

    status = (
        "PASS_STATIC_AUDIT_PROFILE_LIFECYCLE_AND_GATE"
        if child_returncode == 0 and unload_failure is None
        else "FAIL_STATIC_AUDIT_PROFILE_LIFECYCLE_OR_GATE"
    )
    write_json_new(
        lifecycle_path,
        {
            "status": status,
            "created_at": utc_now(),
            "profile_name": PROFILE_NAME,
            "profile_path": str(PROFILE_PATH),
            "profile_sha256": PROFILE_SHA256,
            "pins": pins,
            "events": events,
            "child_returncode": child_returncode,
            "zero_residue": unload_failure is None,
            "unload_failure": unload_failure,
            "root_binary_content_parser_run": False,
            "candidate_executed": False,
        },
    )
    return 0 if status == "PASS_STATIC_AUDIT_PROFILE_LIFECYCLE_AND_GATE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
