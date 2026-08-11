#!/usr/bin/env python3
"""Exact lifecycle supervisor for bounded static-audit revision v2."""

from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path
from types import ModuleType
from typing import Any


WORKSPACE = Path("/home/zrj/scout_ws")
PACKAGE = WORKSPACE / "src/scout_apps/simulation/scout_dualsphysics_liquid"
BASE_SUPERVISOR = PACKAGE / "scripts/r8_liquid_gpu_static_audit_supervisor_v1.py"
BASE_SUPERVISOR_SHA256 = "e63fb6502b2521e6d6edd08bd655b28199c08d09cb66c7dadb1bc9ac1aa3be77"
POLICY_PATH = PACKAGE / (
    "config/target_hosts/"
    "liquid_zrj_msi_u2404_gpu_static_audit_20260810T170339Z_v2.json"
)
POLICY_SHA256 = "18e3869bcf0ad280ce06f1ca9803d6b9420f5df45066c3dae6048c28fba600d5"
GATE_PATH = PACKAGE / "scripts/r8_liquid_gpu_static_audit_gate_v2.py"
GATE_SHA256 = "c6a88721257a792860bf93a7821872e0cd1e28fca318a2d1e43f8de6da7c6b86"
PROFILE_PATH = PACKAGE / (
    "config/apparmor_drafts/"
    "r8-liquid-u3-gpu-static-audit-20260810T170339Z-v2.profile"
)
PROFILE_NAME = "r8-liquid-u3-gpu-static-audit-20260810T170339Z-v2"
PROFILE_SHA256 = "b3a2225ed5d988932387cbc487dab637bd88a565de219d74a7cf5afd9a60647f"


class SupervisorV2Error(RuntimeError):
    pass


def load_base() -> ModuleType:
    spec = importlib.util.spec_from_file_location("r8_gpu_static_audit_supervisor_v1_for_v2", BASE_SUPERVISOR)
    if spec is None or spec.loader is None:
        raise SupervisorV2Error("cannot import frozen v1 lifecycle supervisor")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    if module.sha256_regular(BASE_SUPERVISOR) != BASE_SUPERVISOR_SHA256:
        raise SupervisorV2Error("frozen v1 lifecycle supervisor hash drift")
    return module


def main() -> int:
    if (
        os.geteuid() != 0
        or os.environ.get("SUDO_UID") != "1000"
        or os.environ.get("SUDO_GID") != "1000"
    ):
        print("must be invoked once through sudo by uid/gid 1000", file=sys.stderr)
        return 2
    try:
        base = load_base()
        pins = {
            str(BASE_SUPERVISOR): BASE_SUPERVISOR_SHA256,
            str(POLICY_PATH): POLICY_SHA256,
            str(GATE_PATH): GATE_SHA256,
            str(PROFILE_PATH): PROFILE_SHA256,
            str(base.APPARMOR_PARSER): base.APPARMOR_PARSER_SHA256,
            str(base.AA_STATUS): base.AA_STATUS_SHA256,
            str(base.SETPRIV): base.SETPRIV_SHA256,
        }
        for path_text, digest in pins.items():
            if base.sha256_regular(Path(path_text)) != digest:
                raise SupervisorV2Error(f"pinned v2 lifecycle input drift: {path_text}")
        policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
        expected_profile = {
            "name": PROFILE_NAME,
            "path": "src/scout_apps/simulation/scout_dualsphysics_liquid/config/apparmor_drafts/r8-liquid-u3-gpu-static-audit-20260810T170339Z-v2.profile",
            "mode_octal": "0600",
            "size_bytes": 61138,
            "sha256": PROFILE_SHA256,
        }
        if policy.get("profile") != expected_profile:
            raise SupervisorV2Error("v2 exact profile identity drift")
        lifecycle_path = Path(policy["outputs"]["lifecycle_receipt"])
        if os.path.lexists(lifecycle_path):
            raise SupervisorV2Error("create-new v2 lifecycle receipt already exists")
        if base.profile_loaded(PROFILE_NAME):
            raise SupervisorV2Error("v2 static-audit profile already loaded")
    except (OSError, KeyError, ValueError, json.JSONDecodeError, RuntimeError) as exc:
        print(f"static-audit v2 supervisor preflight failed: {exc}", file=sys.stderr)
        return 2

    events: list[dict[str, Any]] = []
    child_returncode: int | None = None
    unload_failure: str | None = None
    load = base.run(
        [str(base.APPARMOR_PARSER), "-K", "-T", "-a", "--", str(PROFILE_PATH)],
        timeout=60,
    )
    events.append({"action": "load", **load})
    if load["returncode"] != 0:
        base.write_json_new(
            lifecycle_path,
            {
                "status": "FAIL_PROFILE_LOAD",
                "created_at": base.utc_now(),
                "profile_name": PROFILE_NAME,
                "profile_path": str(PROFILE_PATH),
                "profile_sha256": PROFILE_SHA256,
                "pins": pins,
                "events": events,
                "child_started": False,
                "zero_residue": not base.profile_loaded(PROFILE_NAME),
            },
        )
        return 1
    try:
        if not base.profile_loaded(PROFILE_NAME):
            raise SupervisorV2Error("v2 profile absent immediately after load")
        events.append({"action": "verify_loaded", "returncode": 0, "loaded": True})
        child_argv = [
            str(base.SETPRIV),
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
        child = base.run(child_argv, timeout=1200)
        child_returncode = child["returncode"]
        events.append({"action": "uid1000_zero_group_static_audit_gate_v2", **child})
    except (OSError, subprocess.SubprocessError, SupervisorV2Error) as exc:
        child_returncode = 1
        events.append({"action": "supervisor_exception", "returncode": 1, "error": str(exc)})
    finally:
        unload = base.run(
            [str(base.APPARMOR_PARSER), "-K", "-T", "-R", "--", str(PROFILE_PATH)],
            timeout=60,
        )
        events.append({"action": "unload", **unload})
        if unload["returncode"] != 0:
            unload_failure = unload["stderr_tail_utf8"] or "apparmor_parser remove failed"
        residue = base.profile_loaded(PROFILE_NAME)
        events.append({"action": "verify_zero_residue", "returncode": 0 if not residue else 1, "loaded": residue})
        if residue:
            unload_failure = "v2 static-audit profile residue remains loaded"

    status = (
        "PASS_STATIC_AUDIT_PROFILE_LIFECYCLE_AND_GATE"
        if child_returncode == 0 and unload_failure is None
        else "FAIL_STATIC_AUDIT_PROFILE_LIFECYCLE_OR_GATE"
    )
    base.write_json_new(
        lifecycle_path,
        {
            "status": status,
            "created_at": base.utc_now(),
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
            "revision": "BOUNDED_GUEST_TMPFS_CUBIN_METADATA_V2",
        },
    )
    return 0 if status == "PASS_STATIC_AUDIT_PROFILE_LIFECYCLE_AND_GATE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
