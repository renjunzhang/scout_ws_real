#!/usr/bin/env python3
"""Deterministically render and verify four exact fresh-campaign v11 profiles.

``self-check`` requires all four materialized files.  ``render`` is the only
non-writing command.  ``materialize`` first proves a 4/4 absent baseline and
then creates every output with O_EXCL; any partial failure is reported and must
be preserved rather than retried under the same identity.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import os
import stat
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
G1_POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_build_execution_policy_v1.json"
G1_POLICY_SHA256 = "f3831fdfe09716fe5b8f0371bda25d087a6f10e0b0570b90eebea2caf90b7f57"
COPY_PARENT = ROOT / "config/apparmor_drafts/r8-liquid-u3-gpu-copy-20260810T102641Z-a.profile"
COPY_PARENT_SHA256 = "2461550a249284dafc72e473b745ed829463c239e93e805fff623e54a29de8e1"
BUILD_PARENT = ROOT / "config/apparmor_drafts/r8-liquid-u3-gpu-build-20260810T170339Z-a.profile"
BUILD_PARENT_SHA256 = "b11408e72ed0a316f8f8bc7ff3a5eddc9a9518dbd8657c81de595c73c752b0e9"
STATIC_PARENT = ROOT / "config/apparmor_drafts/r8-liquid-u3-gpu-static-audit-20260810T170339Z-v2.profile"
STATIC_PARENT_SHA256 = "b3a2225ed5d988932387cbc487dab637bd88a565de219d74a7cf5afd9a60647f"
PATCH_HELPER = ROOT / "scripts/r8_liquid_motion_gauge_gpu_patch_child_v11.py"
PATCH_V1 = ROOT / "scripts/r8_liquid_motion_attached_gauge_patch_gate_v1.py"
PATCH_V1_SHA256 = "601682672f39ccd19533149024e7531ec54157bd058368f7f40e63a6f894f3cf"
CAMPAIGN_GATE_V3 = ROOT / "scripts/r8_liquid_gpu_stage4_campaign_gate_v3.py"
CAMPAIGN_GATE_V3_SHA256 = "1c1bfc0df0e1959cc723ab22cdbb1163c2307ed4c19e9783f862b63ccfb7643d"

CAMPAIGN_ID = "motion_gauge_gpu_build_sm120_20260812T073037Z_v11"
BUILD_ID = CAMPAIGN_ID + "_a"
ATTEMPT_ROOT = Path("/home/zrj/scout_liquid_lab/build") / f"{BUILD_ID}.partial"
OUTPUT_ROOT = ATTEMPT_ROOT / "output"
SOURCE_ROOT = OUTPUT_ROOT / "buildtree/src/source"
CANDIDATE = OUTPUT_ROOT / "artifacts/DualSPHysics5.4_linux64"
STAMP = "20260812t073037z-v11"
PROFILE_NAMES = {
    "source_copy": f"r8-liquid-motion-gauge-gpu-source-copy-{STAMP}",
    "patch": f"r8-liquid-motion-gauge-gpu-patch-{STAMP}",
    "build": f"r8-liquid-motion-gauge-gpu-build-{STAMP}",
    "static_audit": f"r8-liquid-motion-gauge-gpu-static-audit-{STAMP}",
}
PROFILE_PATHS = {
    role: ROOT / "config/apparmor_drafts" / f"{name}.profile"
    for role, name in PROFILE_NAMES.items()
}
OLD_COPY_NAME = "r8-liquid-u3-gpu-copy-20260810T102641Z-a"
OLD_COPY_ROOT = "/home/zrj/scout_liquid_lab/build/u3_source_gpu_build_sm120_20260810T102641Z_a.partial"
OLD_BUILD_NAME = "r8-liquid-u3-gpu-build-20260810T170339Z-a"
OLD_BUILD_ROOT = "/home/zrj/scout_liquid_lab/build/u3_source_gpu_build_sm120_20260810T170339Z_a.partial"
OLD_STATIC_NAME = "r8-liquid-u3-gpu-static-audit-20260810T170339Z-v2"
OLD_STATIC_ROOT = OLD_BUILD_ROOT
TMP_DELTA = "  /newroot/work/tmp/ rw,\n"
BIN_DELTA = "  /newroot/bin wl,\n"
BUILD_BWRAP_DELTA = ("--symlink", "usr/bin", "/bin")
PATCH_NAMES = (
    "JDsGaugeItem.cpp",
    "JDsGaugeItem.h",
    "JDsGaugeSystem.cpp",
    "JDsGaugeSystem.h",
    "JSph.cpp",
    "JSph.h",
)


class ProfileError(RuntimeError):
    """Profile bytes or create-new state differ from the frozen contract."""


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def read_regular(path: Path, expected_sha256: str | None = None) -> bytes:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1:
            raise ProfileError(f"unsafe profile input: {path}")
        chunks: list[bytes] = []
        remaining = before.st_size
        while remaining:
            block = os.read(descriptor, min(1 << 20, remaining))
            if not block:
                raise ProfileError(f"short profile input read: {path}")
            chunks.append(block)
            remaining -= len(block)
        if os.read(descriptor, 1):
            raise ProfileError(f"profile input grew while read: {path}")
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    if (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) != (
        after.st_dev,
        after.st_ino,
        after.st_size,
        after.st_mtime_ns,
    ):
        raise ProfileError(f"profile input drifted while read: {path}")
    raw = b"".join(chunks)
    if expected_sha256 is not None and sha256_bytes(raw) != expected_sha256:
        raise ProfileError(f"frozen profile input SHA-256 drift: {path}")
    return raw


def write_all(descriptor: int, raw: bytes) -> None:
    offset = 0
    while offset < len(raw):
        written = os.write(descriptor, raw[offset:])
        if written <= 0:
            raise ProfileError("short create-new profile write")
        offset += written


def write_new(path: Path, raw: bytes) -> None:
    descriptor = os.open(
        path,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        0o644,
    )
    try:
        os.fchmod(descriptor, 0o644)
        write_all(descriptor, raw)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


def without_external_network(text: str) -> str:
    forbidden = {"network inet dgram,", "network inet6 dgram,"}
    return "".join(
        line
        for line in text.splitlines(keepends=True)
        if line.strip() not in forbidden
    )


def replace_cardinality(text: str, old: str, new: str, expected: int, label: str) -> str:
    if text.count(old) != expected:
        raise ProfileError(f"{label} cardinality drift: {text.count(old)} != {expected}")
    return text.replace(old, new)


def g1_objects() -> list[str]:
    current = json.loads(read_regular(G1_POLICY, G1_POLICY_SHA256))
    objects = current["object_contract"]["object_names"]
    if len(objects) != 131 or len(set(objects)) != 131 or any("/" in item for item in objects):
        raise ProfileError("frozen 131-object inventory drift")
    return list(objects)


def frozen_build_bwrap_delta() -> tuple[str, ...]:
    """Read the authenticated campaign gate without importing or executing it."""

    raw = read_regular(CAMPAIGN_GATE_V3, CAMPAIGN_GATE_V3_SHA256)
    module = ast.parse(raw, filename=str(CAMPAIGN_GATE_V3))
    assignments = [
        node
        for node in module.body
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "SANDBOX_DELTA" for target in node.targets)
    ]
    if len(assignments) != 1:
        raise ProfileError("authenticated campaign gate SANDBOX_DELTA cardinality drift")
    value = ast.literal_eval(assignments[0].value)
    if not isinstance(value, list) or any(not isinstance(item, str) for item in value):
        raise ProfileError("authenticated campaign gate SANDBOX_DELTA type drift")
    return tuple(value)


def source_copy_profile() -> bytes:
    text = read_regular(COPY_PARENT, COPY_PARENT_SHA256).decode("utf-8")
    text = replace_cardinality(text, OLD_COPY_NAME, PROFILE_NAMES["source_copy"], 3, "copy name")
    text = replace_cardinality(text, OLD_COPY_ROOT, str(ATTEMPT_ROOT), 4, "copy root")
    return without_external_network(text).encode("utf-8")


def patch_profile() -> bytes:
    name = PROFILE_NAMES["patch"]
    exact_host_writes = "\n".join(f"  {SOURCE_ROOT}/{item} rw," for item in PATCH_NAMES)
    exact_guest_writes = "\n".join(f"  /work/source/{item} rw," for item in PATCH_NAMES)
    exact_host_staging = "\n".join(
        f"  {SOURCE_ROOT}/.r8-motion-gauge-v11-{item}.partial rw,"
        for item in PATCH_NAMES
    )
    exact_guest_staging = "\n".join(
        f"  /work/source/.r8-motion-gauge-v11-{item}.partial rw,"
        for item in PATCH_NAMES
    )
    return f"""# Fresh v11 exact six-file patch profile; no workspace glob or compiler/GPU/network surface.
abi <abi/4.0>,
include <tunables/global>

profile {name} flags=(attach_disconnected,mediate_deleted) {{
  userns create,
  /usr/bin/bwrap rix,
  /usr/bin/env rix,
  /usr/bin/prlimit rix,
  /usr/bin/python3 rix,
  /usr/bin/python3.12 rix,
  {PATCH_HELPER} r,
  {PATCH_V1} r,
  /usr/ r,
  /usr/lib/** mr,
  /usr/lib64/** mr,
  /lib/** mr,
  /lib64/** mr,
  /etc/ld.so.cache r,
  /home/ r,
  /home/zrj/ r,
  /home/zrj/scout_ws/ r,
  /home/zrj/scout_ws/src/ r,
  /home/zrj/scout_ws/src/scout_apps/ r,
  /home/zrj/scout_ws/src/scout_apps/simulation/ r,
  /home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/ r,
  /home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid/scripts/ r,
  /home/zrj/scout_liquid_lab/ r,
  /home/zrj/scout_liquid_lab/build/ r,
  {ATTEMPT_ROOT}/ r,
  {OUTPUT_ROOT}/ r,
  {OUTPUT_ROOT}/buildtree/ r,
  {OUTPUT_ROOT}/buildtree/src/ r,
  {SOURCE_ROOT}/ r,
{exact_host_writes}
{exact_host_staging}
  owner /proc/** r,
  owner /proc/*/uid_map w,
  owner /proc/*/gid_map w,
  owner /proc/*/setgroups w,
  /proc/filesystems r,
  /proc/sys/kernel/overflowuid r,
  /proc/sys/kernel/overflowgid r,
  /proc/sys/user/max_user_namespaces w,
  /dev/null rw,
  /dev/zero r,
  /dev/random r,
  /dev/urandom r,
  mount options=(rw, silent, rslave) -> /,
  mount fstype=tmpfs options=(rw, nosuid, nodev) -> /tmp/,
  mount options=(rw, rbind) /tmp/newroot/ -> /tmp/newroot/,
  pivot_root oldroot=/tmp/oldroot/ /tmp/,
  mount options=(rw, silent, rprivate) -> /oldroot/,
  umount /oldroot/,
  pivot_root oldroot=/newroot/ /newroot/,
  umount /,
  mount options=(rw, rbind) /oldroot/usr/ -> /newroot/usr/,
  remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/usr/,
  mount fstype=tmpfs options=(rw, nosuid, nodev) -> /newroot/work/,
  mount options=(rw, rbind) /oldroot{SOURCE_ROOT}/ -> /newroot/work/source/,
  remount options=(rw, nosuid, nodev, bind, silent, relatime) /newroot/work/source/,
  mount options=(rw, bind) /oldroot{PATCH_HELPER} -> /newroot/work/helper.py,
  remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/work/helper.py,
  mount options=(rw, bind) /oldroot{PATCH_V1} -> /newroot/work/patch_v1.py,
  remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/work/patch_v1.py,
  /tmp/newroot/ rw,
  /tmp/newroot/** rw,
  /tmp/oldroot/ rw,
  /tmp/oldroot/** rw,
  /newroot/usr/ rw,
  /newroot/lib wl,
  /newroot/lib64 wl,
  /newroot/work/ rw,
  /newroot/work/source/ rw,
  /newroot/work/helper.py rw,
  /newroot/work/patch_v1.py rw,
  /work/helper.py r,
  /work/patch_v1.py r,
  /work/source/ r,
{exact_guest_writes}
{exact_guest_staging}
  capability sys_admin,
  capability sys_ptrace,
  capability sys_resource,
  capability setpcap,
  capability net_admin,
  ptrace (read, readby) peer={name},
  signal (send, receive) peer={name},
  network unix dgram,
  network netlink raw,
}}
""".encode("utf-8")


def build_profile() -> bytes:
    text = read_regular(BUILD_PARENT, BUILD_PARENT_SHA256).decode("utf-8")
    text = replace_cardinality(text, OLD_BUILD_NAME, PROFILE_NAMES["build"], 3, "build name")
    text = replace_cardinality(text, OLD_BUILD_ROOT, str(ATTEMPT_ROOT), 4, "build root")
    return without_external_network(text).encode("utf-8")


def static_audit_profile() -> bytes:
    text = read_regular(STATIC_PARENT, STATIC_PARENT_SHA256).decode("utf-8")
    text = replace_cardinality(text, OLD_STATIC_NAME, PROFILE_NAMES["static_audit"], 3, "static name")
    text = replace_cardinality(text, OLD_STATIC_ROOT, str(ATTEMPT_ROOT), 270, "static root")
    return without_external_network(text).encode("utf-8")


def rule_lines(text: str) -> list[str]:
    return [line.strip() for line in text.splitlines() if line.strip() and not line.lstrip().startswith("#")]


def validate_patch(text: str) -> None:
    rules = rule_lines(text)
    expected_host = {f"{SOURCE_ROOT}/{item} rw," for item in PATCH_NAMES}
    expected_guest = {f"/work/source/{item} rw," for item in PATCH_NAMES}
    expected_host_staging = {
        f"{SOURCE_ROOT}/.r8-motion-gauge-v11-{item}.partial rw,"
        for item in PATCH_NAMES
    }
    expected_guest_staging = {
        f"/work/source/.r8-motion-gauge-v11-{item}.partial rw,"
        for item in PATCH_NAMES
    }
    host_writes = {line for line in rules if line.startswith(f"{SOURCE_ROOT}/") and line.endswith(" rw,")}
    guest_writes = {line for line in rules if line.startswith("/work/source/") and line.endswith(" rw,")}
    if host_writes != expected_host | expected_host_staging or guest_writes != expected_guest | expected_guest_staging:
        raise ProfileError(
            "patch write set is not exact six targets plus six deterministic atomic staging siblings"
        )
    forbidden = (
        "/home/zrj/scout_ws/src/**",
        f"{SOURCE_ROOT}/ rw,",
        f"{SOURCE_ROOT}/**",
        "/work/source/**",
        "/dev/nvidia",
        "network inet ",
        "network inet6 ",
        "g++-13",
        "flags=(unconfined)",
    )
    if any(token in "\n".join(rules) for token in forbidden):
        raise ProfileError("patch profile contains forbidden broad read/write/tool/network rule")
    if text.count(str(PATCH_HELPER)) != 2 or text.count(str(PATCH_V1)) != 2:
        raise ProfileError("patch exact helper/module bind cardinality drift")
    if sha256_bytes(read_regular(PATCH_V1)) != PATCH_V1_SHA256:
        raise ProfileError("patch v1 byte identity drift")


def validate_build(text: str) -> None:
    semantic = "\n".join(rule_lines(text))
    if text.count(TMP_DELTA) != 1 or text.count(BIN_DELTA) != 1:
        raise ProfileError("build remediation deltas are not exact independent singletons")
    forbidden = (
        "/newroot/work/tmp/**",
        "g++-13",
        "/dev/nvidia",
        "network inet ",
        "network inet6 ",
        "flags=(unconfined)",
    )
    if any(token in semantic for token in forbidden):
        raise ProfileError("build profile contains forbidden permission/tool/network drift")
    if BUILD_BWRAP_DELTA != ("--symlink", "usr/bin", "/bin"):
        raise ProfileError("build /newroot/bin permission is not paired with exact bwrap symlink delta")
    if frozen_build_bwrap_delta() != BUILD_BWRAP_DELTA:
        raise ProfileError("build bwrap delta differs from authenticated campaign gate v3")


def validate_static(text: str, objects: Sequence[str]) -> None:
    rules = rule_lines(text)
    host_objects = {f"{SOURCE_ROOT}/{item} r," for item in objects}
    actual_host_objects = {line for line in rules if line.startswith(f"{SOURCE_ROOT}/") and line.endswith(".o r,")}
    if actual_host_objects != host_objects:
        raise ProfileError("static host object read set differs from exact 131")
    mounts = {
        f"mount options=(rw, rbind) /oldroot{SOURCE_ROOT}/{item} -> /newroot/audit/input/{item},"
        for item in objects
    }
    remounts = {
        f"remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/audit/input/{item},"
        for item in objects
    }
    if not mounts.issubset(set(rules)) or not remounts.issubset(set(rules)):
        raise ProfileError("static exact 131 mount/remount mapping drift")
    candidate_mount = f"mount options=(rw, rbind) /oldroot{CANDIDATE} -> /newroot/audit/input/DualSPHysics5.4_linux64,"
    candidate_remount = "remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/audit/input/DualSPHysics5.4_linux64,"
    magic_mount = "mount options=(rw, rbind) /oldroot/etc/magic -> /newroot/etc/magic,"
    magic_remount = "remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/etc/magic,"
    if not {candidate_mount, candidate_remount, magic_mount, magic_remount}.issubset(set(rules)):
        raise ProfileError("static candidate or /etc/magic exact bind/remount missing")
    required_exec = {
        "/usr/bin/python3 rix,",
        "/usr/bin/python3.12 rix,",
        "/usr/bin/file rix,",
        "/usr/bin/readelf rix,",
        "/usr/bin/sha256sum rix,",
        "/usr/local/cuda-12.8/bin/cuobjdump rix,",
    }
    if not required_exec.issubset(set(rules)):
        raise ProfileError("static mature 557 parser tool surface incomplete")
    forbidden = (
        "/dev/nvidia",
        "network inet ",
        "network inet6 ",
        "flags=(unconfined)",
        "/usr/bin/make ",
        "g++-11",
        "g++-13",
        "/usr/bin/dash ",
    )
    semantic = "\n".join(rules)
    if any(token in semantic for token in forbidden):
        raise ProfileError("static profile exposes compiler/shell/GPU/external network")
    host_writable = [line for line in rules if str(ATTEMPT_ROOT) in line and (line.endswith(" w,") or line.endswith(" rw,"))]
    if host_writable:
        raise ProfileError("static profile has a host writable campaign rule")


def render() -> dict[str, bytes]:
    objects = g1_objects()
    result = {
        "source_copy": source_copy_profile(),
        "patch": patch_profile(),
        "build": build_profile(),
        "static_audit": static_audit_profile(),
    }
    for role, raw in result.items():
        text = raw.decode("utf-8")
        if text.count(f"profile {PROFILE_NAMES[role]} ") != 1 or "@@" in text:
            raise ProfileError(f"profile identity marker drift: {role}")
        semantic = "\n".join(rule_lines(text))
        if "network inet dgram," in semantic or "network inet6 dgram," in semantic:
            raise ProfileError(f"external network rule remains: {role}")
    validate_patch(result["patch"].decode("utf-8"))
    validate_build(result["build"].decode("utf-8"))
    validate_static(result["static_audit"].decode("utf-8"), objects)
    return result


def file_identity(path: Path, raw: bytes) -> dict[str, Any]:
    return {
        "path": str(path.relative_to(ROOT)),
        "mode_octal": "0644",
        "size_bytes": len(raw),
        "sha256": sha256_bytes(raw),
    }


def verify_materialized(rendered: Mapping[str, bytes]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for role, expected in rendered.items():
        path = PROFILE_PATHS[role]
        if not os.path.lexists(path):
            raise ProfileError(f"materialized profile absent: {role}")
        actual = read_regular(path)
        info = os.lstat(path)
        if actual != expected or stat.S_IMODE(info.st_mode) != 0o644:
            raise ProfileError(f"materialized profile byte/mode drift: {role}")
        result[role] = {"name": PROFILE_NAMES[role], **file_identity(path, actual)}
    if len(result) != 4:
        raise ProfileError("materialized profile count is not 4/4")
    return result


def self_check() -> dict[str, Any]:
    rendered = render()
    items = verify_materialized(rendered)
    return {
        "status": "PASS_MOTION_GAUGE_GPU_PROFILES_V11_MATERIALIZED_4_OF_4",
        "campaign_id": CAMPAIGN_ID,
        "build_id": BUILD_ID,
        "attempt_root": str(ATTEMPT_ROOT),
        "profiles": items,
        "materialized_count": 4,
        "tmp_delta_count": 1,
        "bin_delta_count": 1,
        "build_bwrap_delta": list(BUILD_BWRAP_DELTA),
        "static_object_count": 131,
        "profile_loaded": False,
        "system_actions_performed": False,
    }


def materialize() -> dict[str, Any]:
    rendered = render()
    present = [str(path) for path in PROFILE_PATHS.values() if os.path.lexists(path)]
    if present:
        raise ProfileError(f"4/4 absent preflight failed: {present}")
    created: list[str] = []
    try:
        for role in ("source_copy", "patch", "build", "static_audit"):
            write_new(PROFILE_PATHS[role], rendered[role])
            created.append(role)
    except Exception as exc:
        raise ProfileError(
            "profile materialization failed; preserve partial create-new outputs "
            f"created={created}: {exc}"
        ) from exc
    return self_check()


def render_report() -> dict[str, Any]:
    rendered = render()
    return {
        "status": "PASS_MOTION_GAUGE_GPU_PROFILES_V11_RENDER_ONLY",
        "campaign_id": CAMPAIGN_ID,
        "profiles": {
            role: {"name": PROFILE_NAMES[role], **file_identity(PROFILE_PATHS[role], raw)}
            for role, raw in rendered.items()
        },
        "files_written": False,
        "profile_loaded": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("render", "self-check", "materialize"))
    args = parser.parse_args(argv)
    try:
        if args.command == "render":
            result = render_report()
        elif args.command == "self-check":
            result = self_check()
        else:
            result = materialize()
    except Exception as exc:
        print(
            json.dumps(
                {"status": "FAIL_MOTION_GAUGE_GPU_PROFILES_V11", "error": str(exc)},
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 2
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
