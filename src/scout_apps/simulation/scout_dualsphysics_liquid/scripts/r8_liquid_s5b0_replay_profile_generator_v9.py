#!/usr/bin/env python3
"""Deterministic non-loading exact S5B0 primary replay profile generator."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
from pathlib import Path
from typing import Mapping, Sequence


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
TEMPLATE_PATH = ROOT / "config/apparmor_drafts/r8-liquid-s5b0-replay-v9.profile.template"
TOKENS = {"PROFILE_NAME": 3, "STAGE_ROOT": 13}


class ProfileV9Error(ValueError):
    pass


def render_profile(template: str, replacements: Mapping[str, str]) -> str:
    if set(replacements) != set(TOKENS):
        raise ProfileV9Error("profile replacement keys differ")
    name = replacements["PROFILE_NAME"]
    stage = replacements["STAGE_ROOT"]
    if not re.fullmatch(r"r8-liquid-s5b0-primary-bsmooth-b01-[a-z0-9-]{12,80}", name):
        raise ProfileV9Error("profile name differs")
    if not stage.startswith("/home/zrj/scout_liquid_lab/replays/") or stage != os.path.normpath(stage) or not stage.endswith(".stage.partial"):
        raise ProfileV9Error("stage root differs")
    output = template
    for token, count in TOKENS.items():
        marker = f"@@{token}@@"
        if output.count(marker) != count:
            raise ProfileV9Error(f"placeholder count differs: {token}")
        output = output.replace(marker, replacements[token])
    if "@@" in output:
        raise ProfileV9Error("unresolved placeholder")
    authority_text = "\n".join(line for line in output.splitlines() if not line.lstrip().startswith("#"))
    forbidden = ("network inet", "network inet6", "flags=(unconfined", "/dev/nvidia-modeset", "/dev/dri", "Bslosh", " C2")
    if any(token in authority_text for token in forbidden):
        raise ProfileV9Error("forbidden authority present")
    device_rules = [line.strip() for line in output.splitlines()
                    if line.strip().startswith("/dev/nvidia")]
    if device_rules != ["/dev/nvidia0 rw,", "/dev/nvidiactl rw,",
                        "/dev/nvidia-uvm rw,", "/dev/nvidia-uvm-tools rw,"]:
        raise ProfileV9Error("exact four-device host authority differs")
    writable_host = [line.strip() for line in output.splitlines()
                     if line.startswith("  /home/") and re.search(r"\b(?:w|rw|rwk|wk)\b", line)]
    expected = [f"{stage}/guest-output/ rw,", f"{stage}/guest-output/** rwk,"]
    if writable_host != expected:
        raise ProfileV9Error("host writable tree is not exactly guest-output")
    return output


def exact_replacements() -> dict[str, str]:
    return {"PROFILE_NAME": "r8-liquid-s5b0-primary-bsmooth-b01-20260812t110635z-v9",
            "STAGE_ROOT": "/home/zrj/scout_liquid_lab/replays/s5b0_primary_bsmooth_b01_20260812T110635Z_v9.stage.partial"}


def self_check() -> dict[str, object]:
    raw = TEMPLATE_PATH.read_bytes(); rendered = render_profile(raw.decode("utf-8"), exact_replacements())
    return {"status": "PASS_S5B0_REPLAY_PROFILE_GENERATOR_V9_STATIC_ONLY",
            "template_sha256": hashlib.sha256(raw).hexdigest(),
            "exact_profile_sha256": hashlib.sha256(rendered.encode()).hexdigest(),
            "device_count": 4, "host_writable_bind_count": 1,
            "profile_loaded": False, "files_written": False, "optional_bag_read": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument("command", choices=("self-check",), nargs="?", default="self-check"); parser.parse_args(argv)
    try: print(json.dumps(self_check(), sort_keys=True, separators=(",", ":"))); return 0
    except Exception as exc: print(json.dumps({"status": "FAIL_S5B0_REPLAY_PROFILE_GENERATOR_V9", "error": str(exc)}, sort_keys=True), file=sys.stderr); return 2


if __name__ == "__main__": raise SystemExit(main())
