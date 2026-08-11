#!/usr/bin/env python3
"""Deterministically derive the successful-campaign static-audit profile.

The G1 profile is a frozen parent input that enumerates the original A and B
roots.  This generator creates a new, narrower profile for the successful
fresh campaign: one candidate, one object root, and no B permissions.  It
never rewrites an existing file.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import stat
from pathlib import Path


WORKSPACE = Path("/home/zrj/scout_ws")
PACKAGE = WORKSPACE / "src/scout_apps/simulation/scout_dualsphysics_liquid"
BASE_PROFILE = PACKAGE / (
    "config/apparmor_drafts/"
    "r8-liquid-u3-gpu-static-audit-20260810T102641Z.profile"
)
BASE_SHA256 = "d23da58b35c69f458d7f9a7fcf03b9fc2493283cd580191d58f7ced460b45168"
OLD_STAMP = "20260810T102641Z"
NEW_STAMP = "20260810T170339Z"
OLD_A = f"u3_source_gpu_build_sm120_{OLD_STAMP}_a"
OLD_B = f"u3_source_gpu_build_sm120_{OLD_STAMP}_b"
NEW_A = f"u3_source_gpu_build_sm120_{NEW_STAMP}_a"
PROFILE_NAME = f"r8-liquid-u3-gpu-static-audit-{NEW_STAMP}"
DEFAULT_OUTPUT = PACKAGE / f"config/apparmor_drafts/{PROFILE_NAME}.profile"


class ProfileGenerationError(RuntimeError):
    """Fail closed on parent drift or non-deterministic profile output."""


def sha256_regular(path: Path) -> str:
    info = path.lstat()
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise ProfileGenerationError(f"unsafe regular input: {path}")
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


def render() -> bytes:
    if sha256_regular(BASE_PROFILE) != BASE_SHA256:
        raise ProfileGenerationError("frozen G1 static-audit profile hash drift")
    source = BASE_PROFILE.read_text(encoding="utf-8")
    if source.count(OLD_A) < 130 or source.count(OLD_B) < 130:
        raise ProfileGenerationError("parent profile no longer enumerates both exact attempts")

    # Removing every B-root line is safe because the parent places no shared
    # syntax on those lines.  All remaining A/name references are then moved
    # to the successful fresh campaign identity.
    lines = [line for line in source.splitlines() if OLD_B not in line]
    rendered = "\n".join(lines).replace(OLD_STAMP, NEW_STAMP) + "\n"

    forbidden = (
        OLD_STAMP,
        OLD_A,
        OLD_B,
        "u3_source_gpu_build_sm120_20260810T102641Z_b",
        "/usr/bin/make ",
        "/usr/bin/dash ",
        "/usr/bin/bash ",
        "/usr/local/cuda-12.8/bin/nvcc ",
        "/dev/nvidia",
    )
    if any(token in rendered for token in forbidden):
        raise ProfileGenerationError("rendered profile retained a forbidden identity/tool/device")
    if rendered.count(f"profile {PROFILE_NAME} ") != 1:
        raise ProfileGenerationError("profile name was not derived exactly once")
    if rendered.count(NEW_A) < 130:
        raise ProfileGenerationError("successful attempt is not fully enumerated")
    if rendered.count("/audit/input/DualSPHysics5.4_linux64") != 2:
        raise ProfileGenerationError("candidate read/bind/remount rules are incomplete")
    for tool in (
        "/usr/bin/file rix,",
        "/usr/bin/readelf rix,",
        "/usr/bin/sha256sum rix,",
        "/usr/local/cuda-12.8/bin/cuobjdump rix,",
    ):
        if rendered.count(tool) != 1:
            raise ProfileGenerationError(f"parser execute rule drift: {tool}")
    return rendered.encode("utf-8")


def write_new(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(
        path,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        0o600,
    )
    try:
        offset = 0
        while offset < len(payload):
            written = os.write(descriptor, payload[offset:])
            if written <= 0:
                raise ProfileGenerationError("short profile write")
            offset += written
        os.fsync(descriptor)
        os.fchmod(descriptor, 0o600)
    finally:
        os.close(descriptor)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", "write-profile"))
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    try:
        payload = render()
        if args.command == "write-profile":
            if args.output != DEFAULT_OUTPUT:
                raise ProfileGenerationError("output path differs from the frozen exact path")
            write_new(args.output, payload)
        print(
            f"status=PASS_STATIC_AUDIT_PROFILE_RENDER "
            f"path={args.output} size={len(payload)} "
            f"sha256={hashlib.sha256(payload).hexdigest()}"
        )
        return 0
    except (OSError, UnicodeError, ProfileGenerationError) as exc:
        print(f"status=FAIL_STATIC_AUDIT_PROFILE_RENDER error={exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
