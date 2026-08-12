#!/usr/bin/env python3
"""Render four exact motion-Gauge GPU campaign profiles without writing/loading."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_policy_v1.json"
G1_POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_build_execution_policy_v1.json"
PATCH_HELPER = ROOT / "scripts/r8_liquid_motion_attached_gauge_patch_gate_v2.py"


class ProfileGeneratorError(ValueError):
    pass


def sha256_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def _safe_absolute(value: str, label: str) -> str:
    path = Path(value)
    if not value.startswith("/") or str(path) != value or any(char in value for char in "*?[]{}\n\r"):
        raise ProfileGeneratorError(f"unsafe {label}")
    return value


def _replace_exact(template: str, replacements: Mapping[str, str]) -> str:
    rendered = template
    markers = set(re.findall(r"@@([A-Z_]+)@@", template))
    if markers != set(replacements):
        raise ProfileGeneratorError("template/replacement marker set differs")
    for name, value in replacements.items():
        marker = f"@@{name}@@"
        if rendered.count(marker) < 1:
            raise ProfileGeneratorError(f"placeholder cardinality differs: {name}")
        rendered = rendered.replace(marker, value)
    if "@@" in rendered or "NOT LOADABLE AS-IS" not in rendered:
        raise ProfileGeneratorError("rendered profile marker contract differs")
    return rendered


def load_inputs() -> tuple[dict[str, Any], tuple[str, ...]]:
    policy = json.loads(POLICY_PATH.read_bytes())
    g1 = json.loads(G1_POLICY_PATH.read_bytes())
    objects = tuple(g1["object_contract"]["object_names"])
    if len(objects) != 131 or len(set(objects)) != 131:
        raise ProfileGeneratorError("G1 object inventory differs")
    return policy, objects


def render_profiles(policy: Mapping[str, Any], object_names: Sequence[str]) -> dict[str, dict[str, Any]]:
    campaign = policy["campaign"]
    source_root = _safe_absolute(campaign["source_root"], "source root")
    candidate = _safe_absolute(campaign["candidate_path"], "candidate")
    attempt_root = _safe_absolute(campaign["attempt_root"], "attempt root")
    output_root = _safe_absolute(campaign["output_root"], "output root")
    sealed = _safe_absolute(policy["source_copy"]["sealed_source_root"], "sealed source")
    patch_helper = _safe_absolute(str(PATCH_HELPER), "patch helper")
    if len(object_names) != 131 or len(set(object_names)) != 131:
        raise ProfileGeneratorError("object inventory cardinality differs")

    replacements: dict[str, dict[str, str]] = {
        "source_copy": {"PROFILE_NAME": policy["profiles"]["source_copy"]["name"],
                        "SEALED_SOURCE_ROOT": sealed, "ATTEMPT_ROOT": attempt_root,
                        "OUTPUT_ROOT": output_root},
        "patch": {"PROFILE_NAME": policy["profiles"]["patch"]["name"],
                  "PATCH_HELPER": patch_helper, "SOURCE_ROOT": source_root},
        "build": {"PROFILE_NAME": policy["profiles"]["build"]["name"],
                  "ATTEMPT_ROOT": attempt_root, "OUTPUT_ROOT": output_root},
        "static_audit": {"PROFILE_NAME": policy["profiles"]["static_audit"]["name"],
                         "CANDIDATE": candidate,
                         "OBJECT_RULES": "\n".join(f"  {source_root}/{name} r," for name in object_names),
                         "OBJECT_MOUNT_RULES": "\n".join(
                             f"  mount options=(rw, rbind) /oldroot{source_root}/{name} -> /newroot/audit/input/{name},\n"
                             f"  remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/audit/input/{name},"
                             for name in object_names)},
    }
    result: dict[str, dict[str, Any]] = {}
    for role, values in replacements.items():
        spec = policy["profiles"][role]
        template_path = ROOT / spec["template"]
        raw = template_path.read_bytes()
        if sha256_bytes(raw) != spec["template_sha256"]:
            raise ProfileGeneratorError(f"template hash differs: {role}")
        rendered = _replace_exact(raw.decode("utf-8"), values).encode("utf-8")
        text = rendered.decode("utf-8")
        if any(token in text for token in ("/dev/nvidia", "network inet ", "network inet6 ")):
            raise ProfileGeneratorError(f"GPU or external network permission present: {role}")
        if role == "static_audit":
            host_rw = [line.strip() for line in text.splitlines()
                       if line.strip().startswith("/") and line.strip().endswith(" rw,")
                       and not line.strip().startswith(("/dev/", "/proc/", "/tmp/",
                                                        "/newroot/", "/audit/tmp/"))]
            if any(token in text for token in ("/usr/bin/make rix", "g++-11 rix")) or host_rw:
                raise ProfileGeneratorError("static-audit profile can mutate or build")
            if text.count(".o r,") != 131 or f"{candidate} r," not in text:
                raise ProfileGeneratorError("static-audit exact input enumeration differs")
        if role == "patch":
            write_lines = [line.strip() for line in text.splitlines()
                           if line.strip().startswith(source_root + "/")
                           and line.strip().endswith(" rw,")]
            expected = {f"{source_root}/{item['path']} rw," for item in policy["patch_transition"]["files"]}
            if set(write_lines) != expected:
                raise ProfileGeneratorError("patch profile write set differs from exact six")
        result[role] = {
            "name": spec["name"], "instance_path": spec["instance_path"],
            "sha256": sha256_bytes(rendered), "size_bytes": len(rendered),
            "bytes": rendered,
        }
    return result


def self_check() -> dict[str, Any]:
    policy, objects = load_inputs()
    profiles = render_profiles(policy, objects)
    return {
        "status": "PASS_MOTION_GAUGE_GPU_PROFILE_RENDER_V1_NOT_WRITTEN_NOT_LOADED",
        "campaign_id": policy["campaign"]["campaign_id"],
        "profiles": {role: {key: value for key, value in item.items() if key != "bytes"}
                     for role, item in profiles.items()},
        "object_rule_count": 131,
        "files_written": False,
        "profile_queried": False,
        "profile_loaded": False,
        "gpu_exposed": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        report = self_check()
    except (OSError, UnicodeError, ValueError, KeyError, TypeError) as exc:
        print(json.dumps({"status": "FAIL_MOTION_GAUGE_GPU_PROFILE_V1", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(report, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
