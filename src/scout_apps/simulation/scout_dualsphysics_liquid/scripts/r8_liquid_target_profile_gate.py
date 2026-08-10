#!/usr/bin/env python3
"""Read-only safety gate for the MSI Ubuntu 24.04 liquid workstation.

This tool deliberately has one subcommand, ``self-check``.  It reads a fixed
profile and reports whether the current host still matches its narrow storage,
tool and namespace assumptions.  It never creates a directory, a namespace,
a sandbox, a network connection, a receipt, or an upstream process.

It is separate from the legacy ``/data/a`` gates: neither their policies nor
their receipts can make this host pass.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pwd
import shutil
import socket
import stat
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
PROFILE_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_profile_v1.json"
FINDMNT = Path("/usr/bin/findmnt")


class TargetProfileError(RuntimeError):
    """A profile ambiguity is a fail-closed NO-GO."""


def _canonical_hash(value: Any) -> str:
    return hashlib.sha256(
        json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def _read_json(path: Path) -> dict[str, Any]:
    raw = path.read_bytes()
    value = json.loads(raw.decode("utf-8"))
    if not isinstance(value, dict):
        raise TargetProfileError(f"profile is not an object: {path}")
    return value


def _is_clean_absolute(path: Path) -> bool:
    return path.is_absolute() and all(part not in ("", ".", "..", "~") for part in path.parts[1:])


def _has_symlink_component(path: Path) -> bool:
    if not _is_clean_absolute(path):
        return True
    current = Path(path.anchor)
    for part in path.parts[1:]:
        current /= part
        if current.is_symlink():
            return True
        if not current.exists():
            break
    return False


def _parse_os_release(path: Path = Path("/etc/os-release")) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key] = value.strip().strip('"')
    return values


def _memory_total_bytes(path: Path = Path("/proc/meminfo")) -> int:
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.startswith("MemTotal:"):
            parts = line.split()
            return int(parts[1]) * 1024
    raise TargetProfileError("MemTotal is absent from /proc/meminfo")


def _read_int(path: Path) -> int:
    return int(path.read_text(encoding="utf-8").strip())


def _findmnt_snapshot(target: Path) -> Mapping[str, str]:
    completed = subprocess.run(
        (str(FINDMNT), "--json", "--target", str(target), "--output", "TARGET,SOURCE,FSTYPE,OPTIONS"),
        check=False,
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        timeout=5,
    )
    if completed.returncode != 0:
        raise TargetProfileError(f"findmnt failed: {completed.stderr.strip()}")
    payload = json.loads(completed.stdout)
    filesystems = payload.get("filesystems")
    if not isinstance(filesystems, list) or len(filesystems) != 1:
        raise TargetProfileError("findmnt did not return exactly one filesystem")
    record = filesystems[0]
    if not isinstance(record, dict):
        raise TargetProfileError("findmnt filesystem record is invalid")
    required = ("target", "source", "fstype", "options")
    if any(not isinstance(record.get(key), str) for key in required):
        raise TargetProfileError("findmnt record lacks target/source/fstype/options")
    return {key: record[key] for key in required}


def _bwrap_version(path: Path) -> str:
    completed = subprocess.run(
        (str(path), "--version"),
        check=False,
        stdin=subprocess.DEVNULL,
        capture_output=True,
        text=True,
        timeout=5,
    )
    if completed.returncode != 0:
        raise TargetProfileError(f"bwrap --version failed: {completed.stderr.strip()}")
    return completed.stdout.strip()


def validate_profile(profile: Mapping[str, Any]) -> list[str]:
    """Validate only static invariants; no host access or mutation occurs."""

    errors: list[str] = []
    expected = {
        "schema_version": "smpcc-r8-liquid-target-host-profile-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_HOST_PROFILE",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "execution_authorized": False,
        "allowed_script_commands": ["self-check"],
    }
    for key, value in expected.items():
        if profile.get(key) != value:
            errors.append(f"profile field mismatch: {key}")
    storage = profile.get("storage")
    if not isinstance(storage, dict):
        return errors + ["storage profile is missing or invalid"]
    root = Path(str(storage.get("approved_root", "")))
    if not _is_clean_absolute(root):
        errors.append("approved_root is not a clean absolute path")
    if root in (Path("/"), Path("/home"), Path("/home/zrj")):
        errors.append("approved_root is too broad")
    if storage.get("directory_mode_octal") != "0750":
        errors.append("directory mode must be 0750")
    directories = storage.get("required_directories")
    if not isinstance(directories, list) or not directories:
        errors.append("required_directories is missing")
    else:
        for relative in directories:
            candidate = Path(str(relative))
            if candidate.is_absolute() or any(part in ("", ".", "..") for part in candidate.parts):
                errors.append(f"unsafe layout relative path: {relative}")
    protected = storage.get("protected_paths")
    if not isinstance(protected, list) or not protected:
        errors.append("protected_paths is missing")
    else:
        for item in protected:
            if not _is_clean_absolute(Path(str(item))):
                errors.append(f"unsafe protected path: {item}")
    tools = profile.get("required_tools")
    if not isinstance(tools, dict) or set(tools) != {"bwrap", "git", "gpp", "make"}:
        errors.append("required_tools must define exactly bwrap/git/gpp/make")
    return errors


def build_report(profile: Mapping[str, Any]) -> dict[str, Any]:
    """Collect a read-only, fail-closed current-host profile report."""

    errors = validate_profile(profile)
    observations: dict[str, Any] = {}
    storage = profile.get("storage", {})
    host = profile.get("host", {})
    resources = profile.get("resources", {})
    root = Path(str(storage.get("approved_root", "")))
    try:
        os_release = _parse_os_release()
        observations["hostname"] = socket.gethostname()
        observations["os_id"] = os_release.get("ID")
        observations["os_version_id"] = os_release.get("VERSION_ID")
        observations["kernel_release"] = os.uname().release
        if observations["hostname"] != host.get("hostname"):
            errors.append("hostname mismatch")
        if observations["os_id"] != host.get("os_id"):
            errors.append("OS ID mismatch")
        if observations["os_version_id"] != host.get("os_version_id"):
            errors.append("OS version mismatch")
        if not str(observations["kernel_release"]).startswith(str(host.get("kernel_release_prefix", ""))):
            errors.append("kernel release prefix mismatch")

        if _has_symlink_component(root) or root.is_symlink() or not root.is_dir():
            errors.append("approved root is missing, not a directory, or traverses a symlink")
        else:
            root_stat = root.stat()
            owner = pwd.getpwnam(str(storage.get("owner")))
            observations["root_identity"] = {
                "path": str(root),
                "device": root_stat.st_dev,
                "inode": root_stat.st_ino,
                "mode_octal": f"{stat.S_IMODE(root_stat.st_mode):04o}",
                "uid": root_stat.st_uid,
                "gid": root_stat.st_gid,
            }
            if root_stat.st_uid != owner.pw_uid or root_stat.st_gid != owner.pw_gid:
                errors.append("approved root ownership mismatch")
            if stat.S_IMODE(root_stat.st_mode) != int(str(storage.get("directory_mode_octal")), 8):
                errors.append("approved root mode mismatch")
            for relative in storage.get("required_directories", []):
                candidate = root / str(relative)
                if _has_symlink_component(candidate) or candidate.is_symlink() or not candidate.is_dir():
                    errors.append(f"required directory is unsafe or missing: {relative}")
                elif stat.S_IMODE(candidate.stat().st_mode) != int(str(storage.get("directory_mode_octal")), 8):
                    errors.append(f"required directory mode mismatch: {relative}")
            for protected in storage.get("protected_paths", []):
                protected_path = Path(str(protected))
                try:
                    root.relative_to(protected_path)
                except ValueError:
                    continue
                errors.append(f"approved root is inside protected path: {protected_path}")

            mount = _findmnt_snapshot(root)
            observations["mount"] = dict(mount)
            for key in ("target", "source", "fstype"):
                expected = storage.get(f"mount_{key}" if key != "fstype" else "filesystem_type")
                if mount[key] != expected:
                    errors.append(f"mount {key} mismatch")
            mount_options = set(mount["options"].split(","))
            for option in storage.get("required_mount_options", []):
                if option not in mount_options:
                    errors.append(f"required mount option absent: {option}")
            usage = shutil.disk_usage(root)
            observations["free_bytes"] = usage.free
            if usage.free < int(resources_value(storage, "minimum_free_bytes")):
                errors.append("approved root free space below minimum")

        memory_total = _memory_total_bytes()
        observations["memory_total_bytes"] = memory_total
        if memory_total < int(resources_value(resources, "minimum_memory_bytes")):
            errors.append("memory total below minimum")
        userns = _read_int(Path("/proc/sys/user/max_user_namespaces"))
        observations["user_max_user_namespaces"] = userns
        if userns < int(resources_value(resources, "minimum_user_namespaces")):
            errors.append("user namespace capacity below minimum")

        tool_observations: dict[str, Any] = {}
        for name, item in profile.get("required_tools", {}).items():
            path = Path(str(item.get("path", "")))
            if not path.is_file() or path.is_symlink():
                errors.append(f"required tool missing or unsafe: {name}")
                continue
            tool_observations[name] = {"path": str(path), "mode_octal": f"{stat.S_IMODE(path.stat().st_mode):04o}"}
            if name == "bwrap":
                version = _bwrap_version(path)
                tool_observations[name]["version"] = version
                if not version.startswith(str(item.get("version_prefix", ""))):
                    errors.append("bwrap version mismatch")
        observations["tools"] = tool_observations
    except (OSError, KeyError, ValueError, subprocess.SubprocessError, TargetProfileError) as exc:
        errors.append(f"host inspection failed: {exc}")

    report = {
        "schema_version": "smpcc-r8-liquid-target-host-profile-report-v1",
        "document_type": "SMPCC_R8_LIQUID_TARGET_HOST_PROFILE_REPORT",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "profile_id": profile.get("profile_id"),
        "profile_sha256": _canonical_hash(profile),
        "host_id": profile.get("host_id"),
        "status": "PASS_PROFILE_READ_ONLY_ONLY" if not errors else "NO_GO_PROFILE_MISMATCH",
        "errors": errors,
        "observations": observations,
        "execution_authorized": False,
        "self_check_creates_no_namespace": True,
        "self_check_executes_no_upstream_code": True,
        "next_allowed_stage": "TARGET_HOST_SANDBOX_POLICY_AND_MOCK_ONLY_TESTS" if not errors else "REVIEW_PROFILE_MISMATCH",
    }
    report["report_sha256"] = _canonical_hash(report)
    return report


def resources_value(mapping: Mapping[str, Any], key: str) -> Any:
    value = mapping.get(key)
    if value is None:
        raise TargetProfileError(f"profile field missing: {key}")
    return value


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    args = parser.parse_args(argv)
    if args.command != "self-check":
        return 2
    try:
        report = build_report(_read_json(PROFILE_PATH))
    except (OSError, ValueError, TargetProfileError) as exc:
        report = {
            "schema_version": "smpcc-r8-liquid-target-host-profile-report-v1",
            "document_type": "SMPCC_R8_LIQUID_TARGET_HOST_PROFILE_REPORT",
            "captured_at_utc": datetime.now(timezone.utc).isoformat(),
            "status": "NO_GO_PROFILE_MISMATCH",
            "errors": [f"profile load failed: {exc}"],
            "execution_authorized": False,
        }
        report["report_sha256"] = _canonical_hash(report)
    print(json.dumps(report, ensure_ascii=False, sort_keys=True, separators=(",", ":")))
    return 0 if report["status"] == "PASS_PROFILE_READ_ONLY_ONLY" else 2


if __name__ == "__main__":
    raise SystemExit(main())
