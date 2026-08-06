#!/usr/bin/env python3
"""Fail-closed safety boundary for the SIM-R8 offline liquid workspace.

The default ``self-check`` is read-only.  ``prepare-root`` is the only command
in this module that creates directories, and it can create only the fixed
layout below :data:`APPROVED_ROOT`.  The module never starts ROS, Gazebo or a
fluid solver and never deletes data.
"""

from __future__ import annotations

import argparse
import ctypes
import errno
import hashlib
import json
import math
import os
import re
import secrets
import socket
import stat
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple


APPROVED_ROOT = Path("/data/a/scout_sim_replacement/r8_liquid")
STORAGE_ANCHOR = Path("/data/a")
EXPECTED_STORAGE_MOUNT = Path("/data")
EXPECTED_STORAGE_IDENTITY = {
    "target": "/data",
    "source": "/dev/nvme1n1",
    "fstype": "xfs",
    "uuid": "78023025-5a6d-4bf1-9a9c-6bea603df462",
    "major_minor": "259:7",
    "fsroot": "/",
}
SCHEMA_VERSION = "smpcc-r8-liquid-safety-preflight-v1"
DOCUMENT_TYPE = "SMPCC_R8_LIQUID_SAFETY_PREFLIGHT"
GIB = 1024**3
RESERVATION_MULTIPLIER = 1.5
MINIMUM_FREE_BYTES = 100 * GIB
MINIMUM_FREE_FRACTION = 0.15
EMERGENCY_FREE_BYTES = 50 * GIB
EMERGENCY_FREE_FRACTION = 0.05
SIMULATION_PORTS = frozenset(
    (11311, 11328, 11330, 11331, 11345, 11362, 11364, 11365, 11890, 11924)
)
SIMULATION_PROCESS_NAMES = frozenset(
    (
        "roscore",
        "rosmaster",
        "roslaunch",
        "rosbag",
        "gzserver",
        "gzclient",
        "gazebo",
        "rviz",
    )
)
LAYOUT_DIRS = (
    "dependency",
    "dependency/source",
    "dependency/build",
    "dependency/install",
    "dependency/manifests",
    "dependency/vm_images",
    "dependency/toolchains",
    "audits",
    "audits/sandbox",
    "audits/tools",
    "audits/tools/gencase",
    "releases",
    "runs",
    "runs/development",
    "scratch",
    "scratch/vm",
    "scratch/build",
    "locks",
    "quarantine",
)
PREPARE_RECEIPT_RE = re.compile(
    r"^safety_preflight_prepare_root_[0-9]{8}T[0-9]{6}Z\.json$"
)
FINDMNT = Path("/usr/bin/findmnt")
NVIDIA_SMI = Path("/usr/bin/nvidia-smi")


class LiquidSafetyError(RuntimeError):
    """A safety ambiguity is a hard NO-GO, never a warning."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise LiquidSafetyError(message)


def _lexical_parts_are_safe(path: Path) -> bool:
    return path.is_absolute() and all(part not in ("", ".", "..", "~") for part in path.parts[1:])


def has_symlink_component(path: Path) -> bool:
    """Return true if any existing path component is a symlink."""

    require(path.is_absolute(), f"path must be absolute: {path}")
    current = Path(path.anchor)
    for part in path.parts[1:]:
        current = current / part
        if current.is_symlink():
            return True
        if not current.exists():
            break
    return False


def open_directory_nofollow(path: Path) -> int:
    """Open an absolute directory component-by-component without symlinks."""

    path = Path(path)
    require(_lexical_parts_are_safe(path), f"directory path is unsafe: {path}")
    flags = os.O_RDONLY
    if hasattr(os, "O_DIRECTORY"):
        flags |= os.O_DIRECTORY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    current_fd = os.open(path.anchor, flags)
    try:
        for part in path.parts[1:]:
            next_fd = os.open(part, flags, dir_fd=current_fd)
            os.close(current_fd)
            current_fd = next_fd
        return current_fd
    except Exception:
        os.close(current_fd)
        raise


def validate_approved_root(
    candidate: Path,
    *,
    require_exists: bool = False,
) -> Path:
    """Accept only the exact narrow root and reject symlink traversal."""

    candidate = Path(candidate)
    require(_lexical_parts_are_safe(candidate), f"approved root is not a clean absolute path: {candidate}")
    require(_lexical_parts_are_safe(APPROVED_ROOT), f"configured approved root is unsafe: {APPROVED_ROOT}")
    require(candidate == APPROVED_ROOT, f"output root must equal the approved root: {APPROVED_ROOT}")
    require(candidate not in (Path("/"), Path("/data"), Path("/data/a")), "broad output root is forbidden")
    require(not has_symlink_component(candidate), f"approved root may not traverse a symlink: {candidate}")
    require(candidate.resolve(strict=False) == candidate, f"approved root is not canonical: {candidate}")
    if candidate.exists():
        require(candidate.is_dir() and not candidate.is_symlink(), f"approved root is not a real directory: {candidate}")
    if require_exists:
        require(candidate.is_dir(), f"approved root is missing: {candidate}")
    else:
        parent = candidate.parent
        require(parent.is_dir(), f"approved root parent is missing: {parent}")
        require(os.access(parent, os.W_OK | os.X_OK), f"approved root parent is not writable/searchable: {parent}")
    return candidate


def ensure_within_approved_root(
    path: Path,
    *,
    root: Optional[Path] = None,
    allow_root: bool = False,
    require_exists: bool = False,
) -> Path:
    """Validate a path lexically and canonically below an approved root."""

    root = validate_approved_root(APPROVED_ROOT if root is None else root, require_exists=False)
    path = Path(path)
    require(_lexical_parts_are_safe(path), f"path is not a clean absolute path: {path}")
    require(not has_symlink_component(path), f"path may not traverse a symlink: {path}")
    try:
        relative = path.relative_to(root)
    except ValueError as exc:
        raise LiquidSafetyError(f"path escapes approved root {root}: {path}") from exc
    require(allow_root or relative != Path("."), "operation requires a true child of the approved root")
    if require_exists:
        require(path.exists(), f"required path is missing: {path}")
    return path


def mkdir_exact(path: Path, *, root: Optional[Path] = None) -> bool:
    """Create one validated directory without changing existing permissions."""

    path = ensure_within_approved_root(path, root=root, allow_root=(path == root))
    if path.exists():
        require(path.is_dir() and not path.is_symlink(), f"layout path is not a real directory: {path}")
        directory_fd = open_directory_nofollow(path)
        os.close(directory_fd)
        return False
    parent_fd = open_directory_nofollow(path.parent)
    try:
        os.mkdir(path.name, mode=0o750, dir_fd=parent_fd)
        os.fsync(parent_fd)
    finally:
        os.close(parent_fd)
    return True


def prepare_layout(*, root: Optional[Path] = None) -> Dict[str, Any]:
    """Create only the frozen liquid directory layout; never delete or chmod."""

    root = validate_approved_root(APPROVED_ROOT if root is None else root, require_exists=False)
    created = []
    if not root.exists():
        if mkdir_exact(root, root=root):
            created.append(str(root))
    else:
        require(root.is_dir() and not root.is_symlink(), f"approved root is not a real directory: {root}")
    for relative in LAYOUT_DIRS:
        directory = root / relative
        # Parent entries precede children in LAYOUT_DIRS.
        if mkdir_exact(directory, root=root):
            created.append(str(directory))
    return {
        "approved_root": str(root),
        "created_directories": created,
        "layout_directories": [str(root / relative) for relative in LAYOUT_DIRS],
    }


def atomic_write_json_new(
    path: Path,
    value: Mapping[str, Any],
    *,
    root: Optional[Path] = None,
) -> None:
    """Create a JSON file once using O_EXCL and fsync; never replace."""

    path = ensure_within_approved_root(path, root=root)
    require(path.parent.is_dir() and not path.parent.is_symlink(), f"output parent is unsafe: {path.parent}")
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    directory_fd = open_directory_nofollow(path.parent)
    fd: Optional[int] = None
    partial_name = f".{path.name}.partial.{os.getpid()}.{secrets.token_hex(8)}"
    published_and_synced = False
    try:
        fd = os.open(partial_name, flags, 0o640, dir_fd=directory_fd)
        payload = json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
        with os.fdopen(fd, "w", encoding="utf-8", closefd=False) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(
            partial_name,
            path.name,
            src_dir_fd=directory_fd,
            dst_dir_fd=directory_fd,
            follow_symlinks=False,
        )
        os.fsync(directory_fd)
        published_and_synced = True
    finally:
        if fd is not None:
            os.close(fd)
        if published_and_synced:
            os.unlink(partial_name, dir_fd=directory_fd)
            os.fsync(directory_fd)
        os.close(directory_fd)


def rename_directory_noreplace(
    source: Path,
    destination: Path,
    *,
    root: Optional[Path] = None,
) -> None:
    """Atomically publish a directory with Linux RENAME_NOREPLACE."""

    root = validate_approved_root(APPROVED_ROOT if root is None else root, require_exists=True)
    source = ensure_within_approved_root(source, root=root, require_exists=True)
    destination = ensure_within_approved_root(destination, root=root)
    require(source.is_dir() and not source.is_symlink(), f"source directory is unsafe: {source}")
    require(not destination.exists() and not destination.is_symlink(), f"destination already exists: {destination}")
    source_parent_fd = open_directory_nofollow(source.parent)
    destination_parent_fd = open_directory_nofollow(destination.parent)
    try:
        libc = ctypes.CDLL(None, use_errno=True)
        renameat2 = getattr(libc, "renameat2", None)
        require(renameat2 is not None, "renameat2 is unavailable; no safe publication fallback is allowed")
        renameat2.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
        renameat2.restype = ctypes.c_int
        result = renameat2(
            source_parent_fd,
            os.fsencode(source.name),
            destination_parent_fd,
            os.fsencode(destination.name),
            1,  # RENAME_NOREPLACE
        )
        if result != 0:
            error_number = ctypes.get_errno()
            if error_number == errno.EEXIST:
                raise LiquidSafetyError(f"destination appeared during publication: {destination}")
            raise OSError(error_number, os.strerror(error_number), str(destination))
        os.fsync(source_parent_fd)
        if destination_parent_fd != source_parent_fd:
            os.fsync(destination_parent_fd)
    finally:
        os.close(source_parent_fd)
        os.close(destination_parent_fd)


def validate_prepare_receipt_path(path: Path, *, root: Optional[Path] = None) -> Path:
    """Restrict prepare receipts to one timestamped file class in manifests."""

    root = validate_approved_root(APPROVED_ROOT if root is None else root, require_exists=False)
    path = ensure_within_approved_root(path, root=root)
    require(path.parent == root / "dependency/manifests", "prepare receipt must be directly under dependency/manifests")
    require(PREPARE_RECEIPT_RE.fullmatch(path.name) is not None, "prepare receipt filename is invalid")
    return path


def _listening_ports_from_proc() -> Tuple[int, ...]:
    ports = set()
    for proc_file in (Path("/proc/net/tcp"), Path("/proc/net/tcp6")):
        if not proc_file.is_file():
            continue
        lines = proc_file.read_text(encoding="ascii", errors="replace").splitlines()[1:]
        for line in lines:
            fields = line.split()
            if len(fields) < 4 or fields[3] != "0A":
                continue
            try:
                ports.add(int(fields[1].rsplit(":", 1)[1], 16))
            except (IndexError, ValueError):
                continue
    return tuple(sorted(ports))


def _simulation_processes() -> Tuple[Dict[str, Any], ...]:
    found = []
    proc_root = Path("/proc")
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            comm = (entry / "comm").read_text(encoding="utf-8", errors="replace").strip()
            raw_cmdline = (entry / "cmdline").read_bytes()
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        argv = [
            token.decode("utf-8", errors="replace")
            for token in raw_cmdline.split(b"\0")
            if token
        ]
        argv_basenames = {Path(token).name for token in argv}
        matched_names = sorted(({comm} | argv_basenames).intersection(SIMULATION_PROCESS_NAMES))
        if matched_names:
            found.append(
                {
                    "pid": int(entry.name),
                    "comm": comm,
                    "matched_names": matched_names,
                    "cmdline": " ".join(argv),
                }
            )
    return tuple(sorted(found, key=lambda item: item["pid"]))


def _findmnt_snapshot(path: Path) -> Dict[str, Any]:
    """Return one unambiguous mount identity, or fail closed."""

    try:
        completed = subprocess.run(
            [
                str(FINDMNT),
                "-T",
                str(path),
                "-J",
                "-o",
                "TARGET,SOURCE,FSTYPE,UUID,MAJ:MIN,FSROOT,OPTIONS",
            ],
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
            env={"PATH": "/usr/bin:/bin", "LC_ALL": "C"},
        )
    except (FileNotFoundError, subprocess.SubprocessError) as exc:
        raise LiquidSafetyError(f"findmnt failed for {path}: {exc}") from exc
    try:
        decoded = json.loads(completed.stdout)
        filesystems = decoded["filesystems"]
        require(isinstance(filesystems, list) and len(filesystems) == 1, f"findmnt result is ambiguous for {path}")
        raw = filesystems[0]
        require(isinstance(raw, dict), f"findmnt result is malformed for {path}")
        snapshot = {
            "target": raw.get("target"),
            "source": raw.get("source"),
            "fstype": raw.get("fstype"),
            "uuid": raw.get("uuid"),
            "major_minor": raw.get("maj:min"),
            "fsroot": raw.get("fsroot"),
            "options": raw.get("options"),
        }
        require(
            all(isinstance(snapshot[key], str) and snapshot[key] for key in EXPECTED_STORAGE_IDENTITY),
            f"findmnt identity is incomplete for {path}",
        )
        return snapshot
    except (KeyError, TypeError, json.JSONDecodeError) as exc:
        raise LiquidSafetyError(f"findmnt returned malformed JSON for {path}: {exc}") from exc


def _require_expected_storage_identity(
    identity: Mapping[str, Any],
    source: str,
    *,
    keys: Optional[Sequence[str]] = None,
) -> None:
    checked_keys = tuple(EXPECTED_STORAGE_IDENTITY) if keys is None else tuple(keys)
    mismatches = {
        key: {"expected": EXPECTED_STORAGE_IDENTITY[key], "actual": identity.get(key)}
        for key in checked_keys
        if identity.get(key) != EXPECTED_STORAGE_IDENTITY[key]
    }
    require(not mismatches, f"{source} storage identity differs from the fixed /data/a identity: {mismatches}")


def _path_identity(path: Path) -> Dict[str, Any]:
    stat_result = os.lstat(path)
    return {
        "path": str(path),
        "resolved_path": str(path.resolve(strict=True)),
        "device_id": stat_result.st_dev,
        "inode": stat_result.st_ino,
        "uid": stat_result.st_uid,
        "gid": stat_result.st_gid,
        "mode": f"{stat_result.st_mode & 0o7777:04o}",
        "is_symlink": stat.S_ISLNK(stat_result.st_mode),
    }


def _is_relative_to(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
    except ValueError:
        return False
    return True


def _decode_mountinfo_field(value: str) -> str:
    return (
        value.replace("\\040", " ")
        .replace("\\011", "\t")
        .replace("\\012", "\n")
        .replace("\\134", "\\")
    )


def _read_mountinfo() -> Tuple[Dict[str, str], ...]:
    """Parse the kernel mount table strictly enough for identity checks."""

    try:
        lines = Path("/proc/self/mountinfo").read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as exc:
        raise LiquidSafetyError(f"kernel mount table is unreadable: {exc}") from exc
    require(lines, "kernel mount table is empty")
    entries = []
    for line_number, line in enumerate(lines, 1):
        fields = line.split()
        require("-" in fields, f"kernel mount table line {line_number} has no separator")
        separator = fields.index("-")
        require(
            separator >= 6 and len(fields) >= separator + 4,
            f"kernel mount table line {line_number} is malformed",
        )
        mount_point_text = _decode_mountinfo_field(fields[4])
        mount_point = Path(mount_point_text)
        require(mount_point.is_absolute(), f"kernel mount point is not absolute: {mount_point_text}")
        entries.append(
            {
                "target": mount_point_text,
                "source": _decode_mountinfo_field(fields[separator + 2]),
                "fstype": fields[separator + 1],
                "major_minor": fields[2],
                "fsroot": _decode_mountinfo_field(fields[3]),
            }
        )
    return tuple(entries)


def _mountinfo_for_root(root: Path) -> Tuple[Dict[str, str], Tuple[str, ...]]:
    """Fix the backing mount identity and reject nested mounts to the root."""

    entries = _read_mountinfo()
    storage_entries = [entry for entry in entries if entry["target"] == str(EXPECTED_STORAGE_MOUNT)]
    require(len(storage_entries) == 1, f"fixed storage mount is missing or ambiguous: {EXPECTED_STORAGE_MOUNT}")
    storage_identity = storage_entries[0]
    _require_expected_storage_identity(
        storage_identity,
        "mountinfo",
        keys=("target", "source", "fstype", "major_minor", "fsroot"),
    )

    forbidden = []
    for entry in entries:
        mount_point = Path(entry["target"])
        nested_ancestor = (
            mount_point != EXPECTED_STORAGE_MOUNT
            and _is_relative_to(mount_point, EXPECTED_STORAGE_MOUNT)
            and _is_relative_to(root, mount_point)
        )
        root_or_descendant = _is_relative_to(mount_point, root)
        if nested_ancestor or root_or_descendant:
            forbidden.append(str(mount_point))
    return storage_identity, tuple(sorted(set(forbidden)))


def _mount_points_affecting_root(root: Path) -> Tuple[str, ...]:
    """Compatibility helper for the strict single-read mountinfo audit."""

    return _mountinfo_for_root(root)[1]


def _gpu_snapshot() -> Dict[str, Any]:
    query = "index,name,uuid,driver_version,memory.total,memory.used,memory.free,temperature.gpu,pstate"
    result: Dict[str, Any] = {"available": False, "devices": [], "compute_processes": []}
    try:
        devices = subprocess.run(
            [str(NVIDIA_SMI), f"--query-gpu={query}", "--format=csv,noheader,nounits"],
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
            env={"PATH": "/usr/bin:/bin", "LC_ALL": "C"},
        )
        result["available"] = True
        result["devices"] = [line.strip() for line in devices.stdout.splitlines() if line.strip()]
        processes = subprocess.run(
            [str(NVIDIA_SMI), "--query-compute-apps=pid,process_name,used_gpu_memory", "--format=csv,noheader,nounits"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
            env={"PATH": "/usr/bin:/bin", "LC_ALL": "C"},
        )
        result["compute_processes"] = [line.strip() for line in processes.stdout.splitlines() if line.strip()]
    except (FileNotFoundError, subprocess.SubprocessError) as exc:
        result["error"] = str(exc)
    return result


def resource_policy(capacity_bytes: int, available_bytes: int, estimated_case_bytes: int) -> Dict[str, Any]:
    require(capacity_bytes > 0, "filesystem capacity must be positive")
    require(available_bytes >= 0, "filesystem available bytes must be non-negative")
    require(estimated_case_bytes >= 0, "estimated case bytes must be non-negative")
    reservation = int(math.ceil(estimated_case_bytes * RESERVATION_MULTIPLIER))
    minimum = max(MINIMUM_FREE_BYTES, int(math.ceil(capacity_bytes * MINIMUM_FREE_FRACTION)))
    emergency = max(EMERGENCY_FREE_BYTES, int(math.ceil(capacity_bytes * EMERGENCY_FREE_FRACTION)))
    free_after = available_bytes - reservation
    return {
        "estimated_case_bytes": estimated_case_bytes,
        "reservation_multiplier": RESERVATION_MULTIPLIER,
        "reservation_bytes": reservation,
        "capacity_bytes": capacity_bytes,
        "available_bytes": available_bytes,
        "free_after_reservation_bytes": free_after,
        "minimum_free_after_reservation_bytes": minimum,
        "emergency_free_watermark_bytes": emergency,
        "status": "PASS" if free_after >= minimum else "NO_GO",
    }


def _current_umask() -> str:
    try:
        for line in Path("/proc/self/status").read_text(encoding="utf-8").splitlines():
            if line.startswith("Umask:"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return "unknown"


def build_preflight(
    *,
    estimated_case_bytes: int = 0,
    root: Optional[Path] = None,
    require_simulation_stopped: bool = True,
    require_gpu_idle: bool = False,
) -> Dict[str, Any]:
    """Build a read-only safety receipt.  It never creates the root."""

    errors = []
    try:
        root = validate_approved_root(APPROVED_ROOT if root is None else root, require_exists=False)
    except LiquidSafetyError as exc:
        errors.append(str(exc))
        root = Path(APPROVED_ROOT if root is None else root)

    stat_target = root if root.exists() else root.parent
    try:
        stats = os.statvfs(stat_target)
        capacity = stats.f_blocks * stats.f_frsize
        available = stats.f_bavail * stats.f_frsize
        policy = resource_policy(capacity, available, estimated_case_bytes)
        if policy["status"] != "PASS":
            errors.append("insufficient free space after reservation")
        storage_anchor_identity = _path_identity(STORAGE_ANCHOR)
        require(not storage_anchor_identity["is_symlink"], f"storage anchor is a symlink: {STORAGE_ANCHOR}")
        storage_mount = _findmnt_snapshot(STORAGE_ANCHOR)
        _require_expected_storage_identity(storage_mount, "findmnt /data/a")
        mount_snapshot = _findmnt_snapshot(stat_target)
        _require_expected_storage_identity(mount_snapshot, f"findmnt {stat_target}")
        mountinfo_storage, affecting_mounts = _mountinfo_for_root(root)
        stat_target_identity = _path_identity(stat_target)
        require(
            stat_target_identity["device_id"] == storage_anchor_identity["device_id"],
            f"approved root is not on the fixed {STORAGE_ANCHOR} filesystem",
        )
        filesystem = dict(
            mount_snapshot,
            storage_anchor=str(STORAGE_ANCHOR),
            storage_anchor_identity=storage_anchor_identity,
            fixed_storage_identity=dict(EXPECTED_STORAGE_IDENTITY),
            mountinfo_storage_identity=mountinfo_storage,
            stat_target=str(stat_target),
            stat_target_identity=stat_target_identity,
            capacity_bytes=capacity,
            available_bytes=available,
            inode_total=stats.f_files,
            inode_available=stats.f_favail,
        )
        filesystem["forbidden_nested_mount_points"] = list(affecting_mounts)
        if affecting_mounts:
            errors.append(f"nested mount affects approved root: {list(affecting_mounts)}")
    except (OSError, LiquidSafetyError) as exc:
        errors.append(f"filesystem preflight failed: {exc}")
        policy = {}
        filesystem = {"stat_target": str(stat_target), "error": str(exc)}

    listening = sorted(set(_listening_ports_from_proc()).intersection(SIMULATION_PORTS))
    processes = list(_simulation_processes())
    if require_simulation_stopped and listening:
        errors.append(f"simulation-related listening ports are active: {listening}")
    if require_simulation_stopped and processes:
        errors.append("ROS/Gazebo process set is not empty")

    gpu = _gpu_snapshot()
    if require_gpu_idle and gpu.get("compute_processes"):
        errors.append("GPU compute process set is not empty")

    core = {
        "schema_version": SCHEMA_VERSION,
        "document_type": DOCUMENT_TYPE,
        "created_utc": utc_now(),
        "development_only": True,
        "formal": False,
        "fidelity_validation_status": "UNVALIDATED",
        "physical_primary_eligible": False,
        "approved_root": str(root),
        "approved_root_exists": root.is_dir() and not root.is_symlink(),
        "identity": {
            "uid": os.getuid(),
            "gid": os.getgid(),
            "umask": _current_umask(),
            "hostname": socket.gethostname(),
        },
        "filesystem": filesystem,
        "resource_policy": policy,
        "simulation_process_check": {
            "required_stopped": require_simulation_stopped,
            "active_ports": listening,
            "active_processes": processes,
        },
        "gpu": gpu,
        "network_policy": "dependency_acquisition_only; solver_and_postprocess_offline",
        "destructive_cleanup": "forbidden",
        "status": "PASS" if not errors else "NO_GO",
        "errors": errors,
    }
    return dict(core, receipt_hash=canonical_hash(core))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    check = sub.add_parser("self-check", help="read-only environment and path preflight")
    check.add_argument("--estimated-case-bytes", type=int, default=0)
    check.add_argument("--require-gpu-idle", action="store_true")

    prepare = sub.add_parser("prepare-root", help="create only the fixed approved layout")
    prepare.add_argument("--estimated-case-bytes", type=int, default=0)
    prepare.add_argument("--receipt", type=Path, required=True)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        report = build_preflight(
            estimated_case_bytes=args.estimated_case_bytes,
            require_simulation_stopped=True,
            require_gpu_idle=bool(getattr(args, "require_gpu_idle", False)),
        )
        if report["status"] != "PASS":
            print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
            return 2
        if args.command == "self-check":
            print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
            return 0
        if args.command == "prepare-root":
            receipt_path = validate_prepare_receipt_path(args.receipt)
            require(
                not receipt_path.exists() and not receipt_path.is_symlink(),
                f"prepare receipt already exists: {receipt_path}",
            )
            preflight_before = report
            layout = prepare_layout()
            post_creation = build_preflight(
                estimated_case_bytes=args.estimated_case_bytes,
                require_simulation_stopped=True,
                require_gpu_idle=False,
            )
            receipt = dict(
                post_creation,
                action="prepare-root",
                layout=layout,
                preflight_before={
                    "approved_root_exists": preflight_before["approved_root_exists"],
                    "receipt_hash": preflight_before["receipt_hash"],
                    "status": preflight_before["status"],
                },
            )
            receipt_core = dict(receipt)
            receipt_core.pop("receipt_hash", None)
            receipt["receipt_hash"] = canonical_hash(receipt_core)
            atomic_write_json_new(receipt_path, receipt)
            print(json.dumps(receipt, ensure_ascii=False, indent=2, sort_keys=True))
            return 0 if receipt["status"] == "PASS" else 2
        raise LiquidSafetyError(f"unsupported command: {args.command}")
    except (LiquidSafetyError, OSError, ValueError) as exc:
        print(f"R8_LIQUID_SAFETY_NO_GO: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
