#!/usr/bin/env python3
"""Render four exact fresh-campaign AppArmor profiles for v10.

The default command is static-only.  ``materialize`` is an explicit,
create-new repository operation used once while freezing this revision; it
never queries or loads AppArmor and never creates the external campaign root.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import stat
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
WORKSPACE = ROOT.parents[3]
G1_POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_build_execution_policy_v1.json"
G1_POLICY_SHA256 = "f3831fdfe09716fe5b8f0371bda25d087a6f10e0b0570b90eebea2caf90b7f57"
COPY_PARENT = ROOT / "config/apparmor_drafts/r8-liquid-u3-gpu-copy-20260810T102641Z-a.profile"
COPY_PARENT_SHA256 = "2461550a249284dafc72e473b745ed829463c239e93e805fff623e54a29de8e1"
BUILD_REMEDIATION_TEMPLATE = ROOT / "config/apparmor_drafts/r8-liquid-u3-gpu-build-remediation-v1.profile.template"
BUILD_REMEDIATION_TEMPLATE_SHA256 = "dd28bd5e75c1fec0e4861390ac6d2c196412c1b9ff3de9b375bf2e8a9bf51429"
PATCH_HELPER = ROOT / "scripts/r8_liquid_motion_gauge_gpu_patch_child_v10.py"
PATCH_V1 = ROOT / "scripts/r8_liquid_motion_attached_gauge_patch_gate_v1.py"

CAMPAIGN_ID = "motion_gauge_gpu_build_sm120_20260812T062243Z_v10"
BUILD_ID = CAMPAIGN_ID + "_a"
ATTEMPT_ROOT = Path("/home/zrj/scout_liquid_lab/build") / f"{BUILD_ID}.partial"
OUTPUT_ROOT = ATTEMPT_ROOT / "output"
SOURCE_ROOT = OUTPUT_ROOT / "buildtree/src/source"
CANDIDATE = OUTPUT_ROOT / "artifacts/DualSPHysics5.4_linux64"
SEALED_SOURCE = Path("/home/zrj/scout_liquid_lab/dependency/materialized/u3_source_materialization_v1_20260806T155752Z.partial/src/source")
STAMP = "20260812t062243z"
PROFILE_NAMES = {
    "source_copy": f"r8-liquid-motion-gauge-gpu-source-copy-{STAMP}-v10",
    "patch": f"r8-liquid-motion-gauge-gpu-patch-{STAMP}-v10",
    "build": f"r8-liquid-motion-gauge-gpu-build-{STAMP}-v10",
    "static_audit": f"r8-liquid-motion-gauge-gpu-static-audit-{STAMP}-v10",
}
PROFILE_PATHS = {
    role: ROOT / "config/apparmor_drafts" / f"{name}.profile"
    for role, name in PROFILE_NAMES.items()
}
OLD_COPY_NAME = "r8-liquid-u3-gpu-copy-20260810T102641Z-a"
OLD_COPY_ROOT = "/home/zrj/scout_liquid_lab/build/u3_source_gpu_build_sm120_20260810T102641Z_a.partial"
TMP_DELTA = "  /newroot/work/tmp/ rw,\n"
BIN_DELTA = "  /newroot/bin wl,\n"
FORBIDDEN = ("/newroot/work/tmp/**", "g++-13", "/dev/nvidia", "network inet ", "network inet6 ", "flags=(unconfined)")


class ProfileError(RuntimeError):
    pass


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def read_regular(path: Path) -> bytes:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise ProfileError(f"unsafe profile input: {path}")
        data = bytearray()
        while block := os.read(descriptor, 1 << 20):
            data.extend(block)
        return bytes(data)
    finally:
        os.close(descriptor)


def write_new(path: Path, value: bytes) -> None:
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o644)
    try:
        os.fchmod(descriptor, 0o644)
        offset = 0
        while offset < len(value):
            written = os.write(descriptor, value[offset:])
            if written <= 0:
                raise ProfileError(f"short profile write: {path}")
            offset += written
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    parent = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        os.fsync(parent)
    finally:
        os.close(parent)


def g1() -> dict[str, Any]:
    raw = read_regular(G1_POLICY)
    if sha256_bytes(raw) != G1_POLICY_SHA256:
        raise ProfileError("frozen G1 policy drift")
    value = json.loads(raw)
    objects = value["object_contract"]["object_names"]
    if len(objects) != 131 or len(set(objects)) != 131:
        raise ProfileError("frozen object inventory drift")
    return value


def strip_external_network(text: str) -> str:
    lines = [line for line in text.splitlines(keepends=True) if line.strip() not in {"network inet dgram,", "network inet6 dgram,"}]
    return "".join(lines)


def source_copy_profile() -> bytes:
    raw = read_regular(COPY_PARENT)
    if sha256_bytes(raw) != COPY_PARENT_SHA256:
        raise ProfileError("mature source-copy parent drift")
    text = raw.decode("utf-8")
    if text.count(OLD_COPY_NAME) != 3 or text.count(OLD_COPY_ROOT) != 4:
        raise ProfileError("source-copy parent identity cardinality drift")
    text = text.replace(OLD_COPY_NAME, PROFILE_NAMES["source_copy"]).replace(OLD_COPY_ROOT, str(ATTEMPT_ROOT))
    return strip_external_network(text).encode("utf-8")


def patch_profile() -> bytes:
    name = PROFILE_NAMES["patch"]
    source = str(SOURCE_ROOT)
    exact_writes = "\n".join(f"  {source}/{item} rw," for item in (
        "JDsGaugeItem.cpp", "JDsGaugeItem.h", "JDsGaugeSystem.cpp",
        "JDsGaugeSystem.h", "JSph.cpp", "JSph.h"))
    guest_writes = "\n".join(f"  /work/source/{item} rw," for item in (
        "JDsGaugeItem.cpp", "JDsGaugeItem.h", "JDsGaugeSystem.cpp",
        "JDsGaugeSystem.h", "JSph.cpp", "JSph.h"))
    text = f"""# Fresh v10 exact six-file patch profile.  No compiler/GPU/network surface.\nabi <abi/4.0>,\ninclude <tunables/global>\n\nprofile {name} flags=(attach_disconnected,mediate_deleted) {{\n  userns create,\n  /usr/bin/bwrap rix,\n  /usr/bin/env rix,\n  /usr/bin/prlimit rix,\n  /usr/bin/python3 rix,\n  /usr/bin/python3.12 rix,\n  {PATCH_HELPER} r,\n  {PATCH_V1} r,\n  /usr/ r,\n  /usr/lib/** mr,\n  /usr/lib64/** mr,\n  /lib/** mr,\n  /lib64/** mr,\n  /etc/ld.so.cache r,\n  /home/ r,\n  /home/zrj/ r,\n  /home/zrj/scout_ws/ r,\n  /home/zrj/scout_ws/src/** r,\n  /home/zrj/scout_liquid_lab/ r,\n  /home/zrj/scout_liquid_lab/build/ r,\n  {ATTEMPT_ROOT}/ r,\n  {OUTPUT_ROOT}/ r,\n  {SOURCE_ROOT}/ rw,\n  {SOURCE_ROOT}/** r,\n{exact_writes}\n  owner /proc/** r,\n  owner /proc/*/uid_map w,\n  owner /proc/*/gid_map w,\n  owner /proc/*/setgroups w,\n  /proc/filesystems r,\n  /proc/sys/kernel/overflowuid r,\n  /proc/sys/kernel/overflowgid r,\n  /proc/sys/user/max_user_namespaces w,\n  /dev/null rw,\n  /dev/zero r,\n  /dev/random r,\n  /dev/urandom r,\n  mount options=(rw, silent, rslave) -> /,\n  mount fstype=tmpfs options=(rw, nosuid, nodev) -> /tmp/,\n  mount options=(rw, rbind) /tmp/newroot/ -> /tmp/newroot/,\n  pivot_root oldroot=/tmp/oldroot/ /tmp/,\n  mount options=(rw, silent, rprivate) -> /oldroot/,\n  umount /oldroot/,\n  pivot_root oldroot=/newroot/ /newroot/,\n  umount /,\n  mount options=(rw, rbind) /oldroot/usr/ -> /newroot/usr/,\n  remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/usr/,\n  mount fstype=tmpfs options=(rw, nosuid, nodev) -> /newroot/work/,\n  mount options=(rw, rbind) /oldroot{SOURCE_ROOT}/ -> /newroot/work/source/,\n  remount options=(rw, nosuid, nodev, bind, silent, relatime) /newroot/work/source/,\n  mount options=(rw, bind) /oldroot{PATCH_HELPER} -> /newroot/work/helper.py,\n  remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/work/helper.py,\n  mount options=(rw, bind) /oldroot{PATCH_V1} -> /newroot/work/patch_v1.py,\n  remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/work/patch_v1.py,\n  /tmp/newroot/ rw,\n  /tmp/newroot/** rw,\n  /tmp/oldroot/ rw,\n  /tmp/oldroot/** rw,\n  /newroot/usr/ rw,\n  /newroot/lib wl,\n  /newroot/lib64 wl,\n  /newroot/work/ rw,\n  /newroot/work/source/ rw,\n  /newroot/work/helper.py rw,\n  /newroot/work/patch_v1.py rw,\n  /work/helper.py r,\n  /work/patch_v1.py r,\n  /work/source/ r,\n  /work/source/** r,\n{guest_writes}\n  capability sys_admin,\n  capability sys_ptrace,\n  capability sys_resource,\n  capability setpcap,\n  capability net_admin,\n  ptrace (read, readby) peer={name},\n  signal (send, receive) peer={name},\n  network unix dgram,\n  network netlink raw,\n}}\n"""
    return text.encode("utf-8")


def build_profile() -> bytes:
    raw = read_regular(BUILD_REMEDIATION_TEMPLATE)
    if sha256_bytes(raw) != BUILD_REMEDIATION_TEMPLATE_SHA256:
        raise ProfileError("N2 remediation template drift")
    text = raw.decode("utf-8")
    if text.count("@@PROFILE_NAME@@") != 3 or text.count("@@ATTEMPT_ROOT@@") != 4:
        raise ProfileError("build template identity token drift")
    text = text.replace("@@PROFILE_NAME@@", PROFILE_NAMES["build"]).replace("@@ATTEMPT_ROOT@@", str(ATTEMPT_ROOT))
    anchor = "  /newroot/usr/ rw,\n"
    if text.count(anchor) != 1 or text.count(TMP_DELTA) != 1:
        raise ProfileError("build remediation anchors drift")
    text = text.replace(anchor, anchor + BIN_DELTA, 1)
    text = strip_external_network(text)
    if text.count(TMP_DELTA) != 1 or text.count(BIN_DELTA) != 1:
        raise ProfileError("build independent remediation count drift")
    return text.encode("utf-8")


def static_audit_profile() -> bytes:
    current = g1()
    objects = current["object_contract"]["object_names"]
    name = PROFILE_NAMES["static_audit"]
    source = str(SOURCE_ROOT)
    candidate = str(CANDIDATE)
    host_rules = "\n".join(f"  {source}/{obj} r," for obj in objects)
    mounts = "\n".join(f"  mount options=(rw, bind) /oldroot{source}/{obj} -> /newroot/audit/input/{obj}," for obj in objects)
    remounts = "\n".join(f"  remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/audit/input/{obj}," for obj in objects)
    text = f"""# Fresh v10 parser-only static-audit profile: exact candidate and 131 objects.\nabi <abi/4.0>,\ninclude <tunables/global>\n\nprofile {name} flags=(attach_disconnected,mediate_deleted) {{\n  userns create,\n  /usr/bin/bwrap rix,\n  /usr/bin/env rix,\n  /usr/bin/prlimit rix,\n  /usr/bin/file rix,\n  /usr/bin/readelf rix,\n  /usr/bin/x86_64-linux-gnu-readelf rix,\n  /usr/bin/sha256sum rix,\n  /usr/local/cuda-12.8/bin/cuobjdump rix,\n  / r,\n  /usr/ r,\n  /usr/lib/** mr,\n  /usr/lib64/** mr,\n  /lib/** mr,\n  /lib64/** mr,\n  /etc/magic r,\n  /home/ r,\n  /home/zrj/ r,\n  /home/zrj/scout_liquid_lab/ r,\n  /home/zrj/scout_liquid_lab/build/ r,\n  {ATTEMPT_ROOT}/ r,\n  {OUTPUT_ROOT}/ r,\n  {OUTPUT_ROOT}/artifacts/ r,\n  {candidate} r,\n  {OUTPUT_ROOT}/buildtree/ r,\n  {OUTPUT_ROOT}/buildtree/src/ r,\n  {source}/ r,\n{host_rules}\n  owner /proc/** r,\n  owner /proc/*/uid_map w,\n  owner /proc/*/gid_map w,\n  owner /proc/*/setgroups w,\n  /proc/filesystems r,\n  /proc/sys/kernel/overflowuid r,\n  /proc/sys/kernel/overflowgid r,\n  /proc/sys/user/max_user_namespaces w,\n  /dev/null rw,\n  /dev/zero r,\n  /dev/random r,\n  /dev/urandom r,\n  mount options=(rw, silent, rslave) -> /,\n  mount fstype=tmpfs options=(rw, nosuid, nodev) -> /tmp/,\n  mount options=(rw, rbind) /tmp/newroot/ -> /tmp/newroot/,\n  pivot_root oldroot=/tmp/oldroot/ /tmp/,\n  mount options=(rw, silent, rprivate) -> /oldroot/,\n  umount /oldroot/,\n  pivot_root oldroot=/newroot/ /newroot/,\n  umount /,\n  mount options=(rw, rbind) /oldroot/usr/ -> /newroot/usr/,\n  remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/usr/,\n  mount fstype=tmpfs options=(rw, nosuid, nodev) -> /newroot/audit/,\n  mount options=(rw, bind) /oldroot{candidate} -> /newroot/audit/input/DualSPHysics5.4_linux64,\n  remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/audit/input/DualSPHysics5.4_linux64,\n{mounts}\n{remounts}\n  /tmp/newroot/ rw,\n  /tmp/newroot/** rw,\n  /tmp/oldroot/ rw,\n  /tmp/oldroot/** rw,\n  /newroot/usr/ rw,\n  /newroot/lib wl,\n  /newroot/lib64 wl,\n  /newroot/etc/ rw,\n  /newroot/etc/magic rw,\n  /newroot/audit/ rw,\n  /newroot/audit/input/ rw,\n  /newroot/audit/input/** rw,\n  /newroot/audit/tmp/ rw,\n  /audit/input/** r,\n  /audit/tmp/ rw,\n  /audit/tmp/** rw,\n  capability sys_admin,\n  capability sys_ptrace,\n  capability sys_resource,\n  capability setpcap,\n  capability net_admin,\n  ptrace (read, readby) peer={name},\n  signal (send, receive) peer={name},\n  network unix dgram,\n  network netlink raw,\n}}\n"""
    return text.encode("utf-8")


def render() -> dict[str, bytes]:
    result = {
        "source_copy": source_copy_profile(),
        "patch": patch_profile(),
        "build": build_profile(),
        "static_audit": static_audit_profile(),
    }
    for role, raw in result.items():
        text = raw.decode("utf-8")
        semantic_text = "\n".join(
            line for line in text.splitlines() if not line.lstrip().startswith("#")
        )
        if any(token in semantic_text for token in FORBIDDEN):
            raise ProfileError(f"forbidden permission/tool in {role}")
        if text.count(f"profile {PROFILE_NAMES[role]} ") != 1 or "@@" in text:
            raise ProfileError(f"profile identity marker drift: {role}")
    patch_text = result["patch"].decode("utf-8")
    write_set = {line.strip() for line in patch_text.splitlines() if line.startswith(f"  {SOURCE_ROOT}/") and line.endswith(" rw,")}
    expected_write_set = {
        f"{SOURCE_ROOT}/{name} rw,"
        for name in (
            "JDsGaugeItem.cpp",
            "JDsGaugeItem.h",
            "JDsGaugeSystem.cpp",
            "JDsGaugeSystem.h",
            "JSph.cpp",
            "JSph.h",
        )
    }
    write_set.discard(f"{SOURCE_ROOT}/ rw,")
    if write_set != expected_write_set:
        raise ProfileError("patch host write set is not exact six")
    build_text = result["build"].decode("utf-8")
    if build_text.count(TMP_DELTA) != 1 or build_text.count(BIN_DELTA) != 1:
        raise ProfileError("build remediation is not two independent exact rules")
    static_text = result["static_audit"].decode("utf-8")
    if static_text.count(".o r,") != 131 or static_text.count("mount options=(rw, bind) /oldroot") != 132 or static_text.count("remount options=(ro, nosuid, nodev, bind, silent, relatime) /newroot/audit/input/") != 132:
        raise ProfileError("static exact input/mount cardinality drift")
    return result


def identity(path: Path, raw: bytes) -> dict[str, Any]:
    return {"path": str(path.relative_to(ROOT)), "name": PROFILE_NAMES[next(role for role, item in PROFILE_PATHS.items() if item == path)], "mode_octal": "0644", "size_bytes": len(raw), "sha256": sha256_bytes(raw)}


def self_check() -> dict[str, Any]:
    rendered = render()
    items = {role: identity(PROFILE_PATHS[role], raw) for role, raw in rendered.items()}
    for role, item in items.items():
        path = PROFILE_PATHS[role]
        if path.exists():
            actual = read_regular(path)
            mode = format(stat.S_IMODE(os.lstat(path).st_mode), "04o")
            if actual != rendered[role] or mode != "0644":
                raise ProfileError(f"materialized profile drift: {role}")
    return {
        "status": "PASS_MOTION_GAUGE_GPU_PROFILES_V10_STATIC",
        "campaign_id": CAMPAIGN_ID,
        "profiles": items,
        "tmp_delta_count": 1,
        "bin_delta_count": 1,
        "static_object_count": 131,
        "profile_loaded": False,
        "system_actions_performed": False,
    }


def materialize() -> dict[str, Any]:
    rendered = render()
    for role, raw in rendered.items():
        write_new(PROFILE_PATHS[role], raw)
    result = self_check()
    result["status"] = "PASS_MOTION_GAUGE_GPU_PROFILES_V10_CREATE_NEW"
    result["files_written"] = True
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", "materialize"))
    args = parser.parse_args(argv)
    try:
        result = self_check() if args.command == "self-check" else materialize()
    except (ProfileError, OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(json.dumps({"status": "FAIL_MOTION_GAUGE_GPU_PROFILES_V10", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
