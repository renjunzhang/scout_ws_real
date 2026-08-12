#!/usr/bin/env python3
"""Deterministic non-loading renderer for the S5B0 v4 profile template."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path
from typing import Mapping, Sequence


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
TEMPLATE_PATH = ROOT / "config/apparmor_drafts/r8-liquid-s5b0-replay-v4.profile.template"
TOKENS = (
    "PROFILE_NAME", "STAGED_CANDIDATE", "DSPH_CONFIG", "LIBCUDA",
    "LIBNVIDIA_PTXJIT", "CASE_ROOT", "RESTART_ROOT", "NVIDIA0",
    "NVIDIACTL", "NVIDIAUVM", "OUTPUT_ROOT",
)
TOKEN_COUNTS = {name: 1 for name in TOKENS}
TOKEN_COUNTS.update({"CASE_ROOT": 2, "RESTART_ROOT": 2, "OUTPUT_ROOT": 2})
EXACT_DEVICES = {
    "NVIDIA0": "/dev/nvidia0",
    "NVIDIACTL": "/dev/nvidiactl",
    "NVIDIAUVM": "/dev/nvidia-uvm",
}


class ProfileV4Error(ValueError):
    pass


def _safe_path(value: str, label: str) -> str:
    path = Path(value)
    if not value.startswith("/") or str(path) != value or any(char in value for char in "*?[]{}\n\r"):
        raise ProfileV4Error(f"unsafe {label} path")
    return value


def render_profile(template: str, replacements: Mapping[str, str]) -> str:
    if set(replacements) != set(TOKENS):
        raise ProfileV4Error("replacement token set differs")
    if not re.fullmatch(r"r8-liquid-s5b0-[a-z0-9-]{8,80}", replacements["PROFILE_NAME"]):
        raise ProfileV4Error("profile name is not exact-safe")
    for name in TOKENS[1:]:
        _safe_path(replacements[name], name)
    for name, expected in EXACT_DEVICES.items():
        if replacements[name] != expected:
            raise ProfileV4Error(f"device differs: {name}")
    if any("nvidia-uvm-tools" in value for value in replacements.values()):
        raise ProfileV4Error("uvm-tools cannot be pre-admitted")
    rendered = template
    for name in TOKENS:
        marker = f"@@{name}@@"
        if rendered.count(marker) != TOKEN_COUNTS[name]:
            raise ProfileV4Error(f"placeholder cardinality differs: {name}")
        rendered = rendered.replace(marker, replacements[name])
    if "@@" in rendered or "NOT LOADABLE AS-IS" not in rendered:
        raise ProfileV4Error("template marker contract differs")
    output = replacements["OUTPUT_ROOT"]
    writable = [line.strip() for line in rendered.splitlines() if re.search(r"\b(?:rw|rwk|rix)\b", line)]
    host_writable = [line for line in writable if line.startswith("/") and not line.startswith("/dev/") and " rix," not in line]
    if host_writable != [f"{output}/ rw,", f"{output}/** rwk,"]:
        raise ProfileV4Error("OUTPUT_ROOT is not the unique host writable tree")
    return rendered


def fixture_replacements() -> dict[str, str]:
    return {
        "PROFILE_NAME": "r8-liquid-s5b0-fixture-replay-v4",
        "STAGED_CANDIDATE": "/staging/runtime/candidate",
        "DSPH_CONFIG": "/staging/runtime/DsphConfig.xml",
        "LIBCUDA": "/runtime/lib/libcuda.so.1",
        "LIBNVIDIA_PTXJIT": "/runtime/lib/libnvidia-ptxjitcompiler.so.1",
        "CASE_ROOT": "/staging/case",
        "RESTART_ROOT": "/staging/restart",
        "NVIDIA0": "/dev/nvidia0",
        "NVIDIACTL": "/dev/nvidiactl",
        "NVIDIAUVM": "/dev/nvidia-uvm",
        "OUTPUT_ROOT": "/output",
    }


def self_check() -> dict[str, object]:
    raw = TEMPLATE_PATH.read_bytes()
    rendered = render_profile(raw.decode("utf-8"), fixture_replacements())
    return {
        "status": "PASS_S5B0_PROFILE_V4_STATIC_RENDER_ONLY",
        "template_sha256": hashlib.sha256(raw).hexdigest(),
        "rendered_sha256": hashlib.sha256(rendered.encode()).hexdigest(),
        "device_count": 3,
        "uvm_tools_admitted": False,
        "profile_loaded": False,
        "profile_queried": False,
        "files_written": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        result = self_check()
    except (OSError, UnicodeError, ProfileV4Error) as exc:
        print(json.dumps({"status": "FAIL_S5B0_PROFILE_V4", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
