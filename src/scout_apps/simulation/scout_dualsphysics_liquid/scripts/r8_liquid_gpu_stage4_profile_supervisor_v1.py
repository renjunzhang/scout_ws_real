#!/usr/bin/env python3
"""Root-only exact AppArmor lifecycle supervisor for the fresh GPU build.

It performs no build itself.  It loads one byte-pinned profile, invokes the
campaign gate through setpriv as uid/gid 1000 with zero supplementary groups,
and removes the profile in a finally block.  No root shell or root Make exists.
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
POLICY_PATH = PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_gpu_stage4_campaign_20260810T162710Z_v1.json"
GATE_PATH = PACKAGE / "scripts/r8_liquid_gpu_stage4_campaign_gate_v1.py"


class SupervisorError(RuntimeError):
    pass


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def sha256_file(path: Path) -> str:
    info = path.lstat()
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise SupervisorError(f"unsafe file: {path}")
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
            raise SupervisorError("short write")
        offset += written


def write_json_new(path: Path, value: Mapping[str, Any]) -> None:
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
    try:
        write_all(descriptor, json.dumps(value, sort_keys=True, indent=2).encode("utf-8") + b"\n")
        os.fsync(descriptor)
        os.fchmod(descriptor, 0o640)
        os.fchown(descriptor, 1000, 1000)
    finally:
        os.close(descriptor)


def run(argv: Sequence[str], *, timeout: int) -> dict[str, Any]:
    started = time.monotonic()
    completed = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout, check=False)
    return {
        "argv": list(argv),
        "returncode": completed.returncode,
        "elapsed_seconds": round(time.monotonic() - started, 6),
        "stdout": completed.stdout.decode("utf-8", "replace")[-32768:],
        "stderr": completed.stderr.decode("utf-8", "replace")[-32768:],
    }


def loaded(name: str) -> bool:
    result = run(["/usr/sbin/aa-status"], timeout=30)
    if result["returncode"] != 0:
        raise SupervisorError(f"aa-status failed: {result['stderr']}")
    return any(line.strip() == name for line in result["stdout"].splitlines())


def main() -> int:
    if os.geteuid() != 0 or os.environ.get("SUDO_UID") != "1000" or os.environ.get("SUDO_GID") != "1000":
        print("must be invoked once via sudo by uid/gid 1000", file=sys.stderr)
        return 2
    policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
    profile = (WORKSPACE / policy["profile"]["path"]).resolve()
    if sha256_file(profile) != policy["profile"]["sha256"]:
        print("profile hash drift", file=sys.stderr)
        return 2
    lifecycle_path = Path(policy["receipts"]["build_lifecycle"])
    if lifecycle_path.exists():
        print("lifecycle receipt already exists", file=sys.stderr)
        return 2
    name = policy["profile"]["name"]
    if loaded(name):
        print("profile already loaded before lifecycle", file=sys.stderr)
        return 2

    events: list[dict[str, Any]] = []
    child_rc: int | None = None
    unload_failure: str | None = None
    load = run(["/usr/sbin/apparmor_parser", "-K", "-T", "-a", "--", str(profile)], timeout=60)
    events.append({"action": "load", **load})
    if load["returncode"] != 0:
        write_json_new(lifecycle_path, {
            "status": "FAIL_PROFILE_LOAD",
            "created_at": utc_now(),
            "profile_name": name,
            "events": events,
            "child_started": False,
        })
        return 1
    try:
        if not loaded(name):
            raise SupervisorError("profile absent immediately after load")
        events.append({"action": "verify_loaded", "returncode": 0, "loaded": True})
        child_argv = [
            "/usr/bin/setpriv", "--reuid=1000", "--regid=1000", "--clear-groups", "--",
            "/usr/bin/env", "PYTHONDONTWRITEBYTECODE=1",
            f"R8_EXACT_PROFILE_PRELOADED_SHA256={policy['profile']['sha256']}",
            "/usr/bin/python3", str(GATE_PATH), "build",
        ]
        child = subprocess.run(child_argv, cwd=WORKSPACE, check=False)
        child_rc = child.returncode
        events.append({"action": "uid1000_zero_group_gate", "argv": child_argv, "returncode": child_rc})
    except Exception as exc:
        events.append({"action": "supervisor_exception", "error": str(exc), "returncode": 1})
        child_rc = 1
    finally:
        unload = run(["/usr/sbin/apparmor_parser", "-K", "-T", "-R", "--", str(profile)], timeout=60)
        events.append({"action": "unload", **unload})
        if unload["returncode"] != 0:
            unload_failure = unload["stderr"] or "apparmor_parser remove failed"
        residue = loaded(name)
        events.append({"action": "verify_zero_residue", "returncode": 0 if not residue else 1, "loaded": residue})
        if residue:
            unload_failure = "profile residue remains loaded"

    status = "PASS_PROFILE_LIFECYCLE_AND_GATE" if child_rc == 0 and unload_failure is None else "FAIL_PROFILE_LIFECYCLE_OR_GATE"
    write_json_new(lifecycle_path, {
        "status": status,
        "created_at": utc_now(),
        "profile_name": name,
        "profile_path": str(profile),
        "profile_sha256": policy["profile"]["sha256"],
        "events": events,
        "child_returncode": child_rc,
        "zero_residue": unload_failure is None,
        "unload_failure": unload_failure,
        "root_make": False,
    })
    return 0 if status == "PASS_PROFILE_LIFECYCLE_AND_GATE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
