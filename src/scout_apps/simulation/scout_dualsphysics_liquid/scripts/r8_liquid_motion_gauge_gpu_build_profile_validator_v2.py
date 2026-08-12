#!/usr/bin/env python3
"""Read-only validator for four exact v2 AppArmor profile instances."""

from __future__ import annotations

import hashlib
import json
import os
import stat
import sys
from pathlib import Path

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_execution_policy_v2.json"


class ProfileError(ValueError):
    pass


def validate() -> dict[str, object]:
    policy = json.loads(POLICY.read_bytes())
    results = []
    for item in policy["profiles"]:
        path = (ROOT / item["path"]).resolve(strict=True)
        info = os.lstat(path)
        raw = path.read_bytes()
        identity = {"mode_octal":format(stat.S_IMODE(info.st_mode), "04o"),
                    "size_bytes":len(raw), "sha256":hashlib.sha256(raw).hexdigest()}
        if not stat.S_ISREG(info.st_mode) or stat.S_ISLNK(info.st_mode) or info.st_nlink != 1:
            raise ProfileError(f"unsafe profile: {item['role']}")
        if identity != {key:item[key] for key in identity}:
            raise ProfileError(f"profile identity drift: {item['role']}")
        text = raw.decode("utf-8")
        if f"profile {item['name']} " not in text or "@@" in text:
            raise ProfileError(f"profile name/marker drift: {item['role']}")
        if any(token in text for token in ("/dev/nvidia", "network inet", "network inet6")):
            raise ProfileError(f"GPU/network permission present: {item['role']}")
        results.append({"role":item["role"], **identity})
    return {"status":"PASS_EXACT_PROFILES_V2_READ_ONLY_VALIDATION",
            "profiles":results, "query_argv_template":policy["profile_query"]["argv_template"],
            "profile_loaded":False, "files_written":False}


if __name__ == "__main__":
    try:
        print(json.dumps(validate(), sort_keys=True, separators=(",", ":")))
    except Exception as exc:
        print(json.dumps({"status":"FAIL_EXACT_PROFILES_V2", "error":str(exc)},
                         sort_keys=True), file=sys.stderr)
        raise SystemExit(2)
