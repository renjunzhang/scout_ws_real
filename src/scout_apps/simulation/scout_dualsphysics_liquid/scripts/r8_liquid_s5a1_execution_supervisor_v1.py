#!/usr/bin/env python3
"""One-shot S5A1 host supervisor and sandbox worker.

No execution is implicit: ``self-check`` is static.  ``run-one-shot`` requires
an exact final S5A0 receipt path and SHA-256, creates the frozen partial root
once, launches one network-unshared worker, validates the 20-file package on
the host, and publishes with Linux RENAME_NOREPLACE.  Any failure preserves
the partial root and publishes a create-new failure receipt when safe.
"""

from __future__ import annotations

import argparse
import ctypes
import errno
import hashlib
import json
import os
import stat
import subprocess
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
SCRIPT = Path(__file__).resolve()
POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a1_primary_handoff_v3_v1.json"
PACKAGE_SCHEMA = ROOT / "schema/target_host_s5a1_handoff_package_v1.json"
RECEIPT_SCHEMA = ROOT / "schema/target_host_s5a1_execution_receipt_v1.json"
S5A0_SCHEMA = ROOT / "schema/target_host_s5a0_selected_bag_receipt_v1.json"
S5A0_FINAL_SCHEMA = ROOT / "schema/target_host_s5a0_primary_supervisor_execution_receipt_v1.json"
S5A0_SUPERVISOR_POLICY = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a0_primary_supervisor_policy_v1.json"
GATE = ROOT / "scripts/r8_liquid_s5a1_handoff_gate_v1.py"
EXTRACTOR = ROOT / "scripts/r8_liquid_s5a1_ros1_signal_extractor_v1.py"
BRIDGE = ROOT / "scripts/r8_liquid_s5a1_motion_bridge_v1.py"
READER = ROOT / "scripts/r8_liquid_ros1_bag_v2_reader_v1.py"
COMPAT_GATE = ROOT / "scripts/r8_liquid_s5_c1_compatibility_gate_v1.py"
TRANSFER_ID = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v1"
PARTIAL_ROOT = Path(f"/home/zrj/scout_liquid_lab/incoming/{TRANSFER_ID}.partial")
FINAL_ROOT = Path(f"/home/zrj/scout_liquid_lab/incoming/{TRANSFER_ID}")
EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{TRANSFER_ID}.execution_v1.json")
EXECUTION_RECEIPT_PARTIAL = EXECUTION_RECEIPT.with_name(EXECUTION_RECEIPT.name + ".partial")
S5A0_FINAL_RECEIPT = Path("/home/zrj/scout_liquid_lab/audits/s5a0_primary_bsmooth_b01_20260811T173630Z_v1.host.final.json")
S5A0_EVIDENCE_ROOT = Path("/home/zrj/scout_liquid_lab/audits/s5a0_primary_bsmooth_b01_20260811T173630Z_v1.host.evidence")
S5A0_INNER_RECEIPT = S5A0_EVIDENCE_ROOT / "selected_bag_inner_receipt_v1.json"
EXPECTED = {
    POLICY: "1ba40330581c3736442dcd6191d19f798cef5d658d092af6026de924e4b93c2c",
    PACKAGE_SCHEMA: "4e1c10d7bd721e072def81dfd491c1d6d72e2aa45e4b2e8e773a1f69dc471496",
    GATE: "7556f7aeebd9a358299c485ad232f8f6263990a437bba9c9ec06feb0d1fc9fd6",
    EXTRACTOR: "2ae263a2efa9ebc68d1fceb22201e5297db3a96df26002597d0f0a6ba60ad4bf",
    BRIDGE: "c840f7bb467d5145f55d2e9904b7b8b62781658818ae47f3e95bd139904e2aa1",
    READER: "bc3975ba446e22097c399e10a8f6c4e60b4f8e78c5c7d613394028daa76c94fc",
    S5A0_SCHEMA: "4d748556a0a03c55dae3bdeefc0159221d267a0d71dc9ea23112710ffe73573a",
    S5A0_FINAL_SCHEMA: "e3eaa33fb0b491b337958f4e59b11c60a1ef742d2c255761b4a2e85c899a7cb9",
    S5A0_SUPERVISOR_POLICY: "74bfb10a45116083228e44ae29dd7810d57b2318fa5fbaf19817d79e8ae87199",
    COMPAT_GATE: "8c924726550a85fc7d5d0bf7af03f7f92ca60218dbe3d97c75df4ea43defba4b",
}
for _path in (GATE, EXTRACTOR, COMPAT_GATE):
    sys.path.insert(0, str(_path.parent))
import r8_liquid_s5a1_handoff_gate_v1 as gate  # noqa: E402
import r8_liquid_s5a1_ros1_signal_extractor_v1 as extractor  # noqa: E402
import r8_liquid_s5_c1_compatibility_gate_v1 as compatibility  # noqa: E402


class ExecutionError(ValueError):
    """One-shot admission, sandbox, worker, validation, or publication failure."""


def sha256_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def read_file_identity(
    path: Path, expected_sha256: str | None = None, maximum_bytes: int = 64 * 1024 * 1024
) -> tuple[bytes, dict[str, Any]]:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    digest = hashlib.sha256()
    try:
        metadata = os.fstat(descriptor)
        if (not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1
                or not 0 < metadata.st_size <= maximum_bytes):
            raise ExecutionError(f"unsafe input identity: {path}")
        raw = bytearray()
        remaining = metadata.st_size
        while remaining:
            block = os.read(descriptor, min(remaining, 1024 * 1024))
            if not block:
                raise ExecutionError(f"short input read: {path}")
            digest.update(block)
            raw.extend(block)
            remaining -= len(block)
    finally:
        os.close(descriptor)
    observed = digest.hexdigest()
    if expected_sha256 is not None and observed != expected_sha256:
        raise ExecutionError(f"input hash drifted: {path}")
    return bytes(raw), {"path": str(path), "sha256": observed, "size_bytes": metadata.st_size,
                        "mode": f"{stat.S_IMODE(metadata.st_mode):04o}", "inode": metadata.st_ino,
                        "nlink": metadata.st_nlink}


def file_identity(path: Path, expected_sha256: str | None = None) -> dict[str, Any]:
    return read_file_identity(path, expected_sha256)[1]


def opened_fd_identity(
    descriptor: int, reported_path: Path, expected_sha256: str,
    maximum_bytes: int = 64 * 1024 * 1024,
) -> dict[str, Any]:
    """Hash an already-open fd without changing its offset or trusting its path."""
    metadata = os.fstat(descriptor)
    if (not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1
            or not 0 < metadata.st_size <= maximum_bytes):
        raise ExecutionError(f"unsafe opened input identity: {reported_path}")
    digest = hashlib.sha256()
    offset = 0
    while offset < metadata.st_size:
        block = os.pread(descriptor, min(1024 * 1024, metadata.st_size - offset), offset)
        if not block:
            raise ExecutionError(f"short opened input read: {reported_path}")
        digest.update(block)
        offset += len(block)
    observed = digest.hexdigest()
    if observed != expected_sha256:
        raise ExecutionError(f"opened input hash drifted: {reported_path}")
    return {"path": str(reported_path), "sha256": observed, "size_bytes": metadata.st_size,
            "mode": f"{stat.S_IMODE(metadata.st_mode):04o}", "inode": metadata.st_ino,
            "nlink": metadata.st_nlink}


def assert_same_file_identity(observed: Mapping[str, Any], expected: Mapping[str, Any], label: str) -> None:
    if _receipt_file_identity(observed) != _receipt_file_identity(expected):
        raise ExecutionError(f"{label} opened fd identity differs from frozen identity")


def verify_parents() -> dict[str, dict[str, Any]]:
    return {str(path): file_identity(path, digest) for path, digest in EXPECTED.items()}


def root_identity(path: Path) -> dict[str, Any]:
    if not path.exists() and not path.is_symlink():
        return {"path": str(path), "exists": False, "mode": "ABSENT", "entry_count": 0,
                "inventory_sha256": None}
    metadata = os.lstat(path)
    if not stat.S_ISDIR(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) != 0o700:
        raise ExecutionError(f"unsafe transfer root: {path}")
    entries = sorted(path.iterdir(), key=lambda item: item.name)
    digest = hashlib.sha256()
    for entry in entries:
        identity = file_identity(entry)
        digest.update(f"{entry.name}\0{identity['size_bytes']}\0{identity['sha256']}\n".encode())
    return {"path": str(path), "exists": True, "mode": "0700", "entry_count": len(entries),
            "inventory_sha256": digest.hexdigest()}


def load_package(root: Path) -> dict[str, bytes]:
    if sorted(path.name for path in root.iterdir()) != sorted(gate.PACKAGE_INVENTORY):
        raise ExecutionError("worker package inventory differs")
    package: dict[str, bytes] = {}
    for name in gate.PACKAGE_INVENTORY:
        path = root / name
        raw, identity = read_file_identity(path, maximum_bytes=16 * 1024 * 1024)
        if identity["mode"] != "0600":
            raise ExecutionError(f"package mode differs: {name}")
        package[name] = raw
    gate.validate_package_bytes(package)
    if not json.loads(package["transfer_manifest.json"])["safety"]["real_bag_read"]:
        raise ExecutionError("actual package reports real_bag_read=false")
    return package


def write_existing_partial(root: Path, package: Mapping[str, bytes]) -> None:
    metadata = os.lstat(root)
    if not stat.S_ISDIR(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) != 0o700 or any(root.iterdir()):
        raise ExecutionError("worker output root is not fresh empty 0700")
    for name in gate.PACKAGE_INVENTORY:
        descriptor = os.open(root / name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
        try:
            os.fchmod(descriptor, 0o600)
            view = memoryview(package[name])
            while view:
                written = os.write(descriptor, view)
                if written <= 0:
                    raise ExecutionError("short worker output write")
                view = view[written:]
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def rename_noreplace(source: Path, target: Path) -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    function = libc.renameat2
    function.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    function.restype = ctypes.c_int
    if function(-100, os.fsencode(source), -100, os.fsencode(target), 1) != 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error), str(target))


def _sandbox_dirs(paths: Sequence[Path]) -> list[str]:
    directories = {Path("/")}
    for path in paths:
        parent = path.parent
        while parent != Path("/"):
            directories.add(parent)
            parent = parent.parent
    argv: list[str] = []
    for directory in sorted(directories - {Path("/")}, key=lambda item: (len(item.parts), str(item))):
        argv.extend(("--dir", str(directory)))
    return argv


def build_bwrap_argv(
    receipt: Path, source: Path, partial: Path, *, receipt_fd: int | None = None,
    source_fd: int | None = None,
) -> list[str]:
    readonly = [SCRIPT, POLICY, PACKAGE_SCHEMA, RECEIPT_SCHEMA, S5A0_SCHEMA, GATE,
                EXTRACTOR, BRIDGE, READER, COMPAT_GATE]
    if receipt_fd is None:
        readonly.append(receipt)
    if source_fd is None:
        readonly.append(source)
    argv = ["/usr/bin/timeout", "--signal=TERM", "--kill-after=5s", "180s",
            "/usr/bin/prlimit", "--as=1073741824", "--rss=805306368", "--nofile=256",
            "/usr/bin/bwrap", "--die-with-parent", "--new-session", "--unshare-all",
            "--clearenv", "--ro-bind", "/usr", "/usr", "--ro-bind", "/lib", "/lib"]
    if Path("/lib64").exists():
        argv += ["--ro-bind", "/lib64", "/lib64"]
    argv += _sandbox_dirs(readonly + [receipt, source, partial])
    for path in readonly:
        argv += ["--ro-bind", str(path), str(path)]
    if receipt_fd is not None:
        argv += ["--ro-bind", f"/proc/self/fd/{receipt_fd}", str(receipt)]
    if source_fd is not None:
        argv += ["--ro-bind", f"/proc/self/fd/{source_fd}", str(source)]
    argv += ["--bind", str(partial), str(partial), "--setenv", "PATH", "/usr/bin",
             "--setenv", "HOME", "/nonexistent", "--setenv", "PYTHONDONTWRITEBYTECODE", "1",
             "--chdir", "/", "--", "/usr/bin/python3", str(SCRIPT), "_worker",
             "--s5a0-receipt", str(receipt), "--source", str(source), "--partial-root", str(partial)]
    return argv


def worker(receipt_path: Path, source: Path, partial: Path) -> dict[str, Any]:
    if os.getuid() != 1000 or os.getgid() != 1000:
        raise ExecutionError("worker uid/gid differs from 1000:1000")
    policy, policy_identity = gate.read_json(POLICY)
    receipt_raw = receipt_path.read_bytes()
    gate.validate_s5a0_receipt(receipt_raw, str(receipt_path), policy)
    extracted, before, after = extractor.read_exact_primary(
        source, expected_size=policy["source"]["bag_size_bytes"], expected_mode=int(policy["source"]["bag_mode"], 8),
        expected_sha256=policy["source"]["bag_sha256"])
    if before != after:
        raise ExecutionError("source changed while extracting")
    clock = extractor.nearest_clock_alignment(extracted["odom"], extracted["clock"])
    tf_qc = extractor.tf_cross_check(
        extracted["odom"], extracted["transforms"], maximum_time_delta_s=0.5,
        maximum_position_delta_m=0.05, maximum_angular_delta_rad=0.1)
    gate_tf = {key: tf_qc[key] for key in ("status", "odom_frame", "base_frame",
               "container_mount_source", "tf_used_as_motion")}
    package = gate.build_package(
        policy=policy, policy_identity=policy_identity, s5a0_receipt_raw=receipt_raw,
        s5a0_receipt_path=str(receipt_path), odom_records=extracted["odom"],
        clock_alignment=clock, tf_alignment=gate_tf,
        qc_only_witness_counts=extracted["qc_only_topic_counts"], real_bag_read=True)
    write_existing_partial(partial, package)
    return {"status": "WORKER_PASS", "tf_qc": tf_qc, "source_before": before,
            "source_after": after, "package_digest": sha256_bytes(b"".join(package.values())),
            "real_bag_read": True,
            "source_bag_executed": False, "optional_bag_read": False, "ros_started": False,
            "gpu_exposed": False, "solver_executed": False, "network_used": False, "sudo_used": False}


def reserve_receipt(path: Path = EXECUTION_RECEIPT_PARTIAL) -> int:
    descriptor = os.open(
        path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o440
    )
    os.fchmod(descriptor, 0o440)
    return descriptor


def publish_reserved_receipt(
    descriptor: int, partial_path: Path, final_path: Path, value: Mapping[str, Any]
) -> None:
    try:
        schema = json.loads(RECEIPT_SCHEMA.read_bytes())
        Draft202012Validator(schema).validate(value)
        raw = gate.canonical_json(value)
        view = memoryview(raw)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise ExecutionError("short execution receipt write")
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    rename_noreplace(partial_path, final_path)


def _identity_ref(path: Path, expected: str | None = None) -> dict[str, str]:
    observed = file_identity(path, expected)
    return {"path": observed["path"], "sha256": observed["sha256"]}


def _receipt_file_identity(value: Mapping[str, Any]) -> dict[str, Any]:
    return {key: value[key] for key in ("path", "sha256", "size_bytes", "mode", "inode", "nlink")}


def _package_summary(package: Mapping[str, bytes], root: Path) -> dict[str, Any]:
    root_observed = root_identity(root)
    return {"manifest_sha256": sha256_bytes(package["transfer_manifest.json"]),
            "checksums_sha256": sha256_bytes(package["checksums.sha256"]),
            "inventory_sha256": root_observed["inventory_sha256"], "entry_count": 20,
            "all_file_modes_0600": True, "validated": True}


def resolve_s5a0_final_receipt(
    final_path: Path, final_sha256: str, policy: Mapping[str, Any]
) -> tuple[dict[str, Any], bytes, dict[str, Any], dict[str, Any]]:
    if final_path != S5A0_FINAL_RECEIPT:
        raise ExecutionError("S5A0 final receipt path differs from the frozen exact path")
    outer_raw, outer_identity = read_file_identity(
        final_path, final_sha256, maximum_bytes=1024 * 1024
    )
    outer_schema = json.loads(S5A0_FINAL_SCHEMA.read_bytes())
    outer = json.loads(outer_raw)
    Draft202012Validator(outer_schema).validate(outer)
    if (outer["status"] != "PASS_S5A0_PRIMARY_ONE_SHOT_DEVELOPMENT_ONLY"
            or outer["failure"] is not None
            or outer["policy"]["path"] != str(S5A0_SUPERVISOR_POLICY)
            or outer["policy"]["sha256"] != EXPECTED[S5A0_SUPERVISOR_POLICY]
            or outer["policy"]["schema_path"] != str(S5A0_FINAL_SCHEMA)
            or outer["policy"]["schema_sha256"] != EXPECTED[S5A0_FINAL_SCHEMA]
            or outer["evidence"]["final_root"] != str(S5A0_EVIDENCE_ROOT)
            or not outer["evidence"]["atomic_rename_noreplace"]
            or outer["worker"]["inner_receipt_schema_valid"] is not True
            or outer["worker"]["inner_receipt_status"] != policy["source"]["required_s5a0_status"]
            or outer["host_source"]["path_unchanged"] is not True
            or outer["host_source"]["mount_unchanged"] is not True
            or outer["host_source"]["file_unchanged"] is not True
            or outer["host_source"]["root_unchanged"] is not True
            or outer["host_source"]["root_scope"] != "SELECTED_ATTEMPT_ROOT_ONLY"
            or outer["host_source"]["corpus_root_enumerated"] is not False
            or outer["host_source"]["source_root_writable_bind_exposed"] is not False
            or outer["host_source"]["files_written_under_selected_attempt_root"] != 0
            or outer["claims"]["source_bag_executed"] is not False
            or outer["selection"]["optional_pair_exposed"] is not False
            or outer["sandbox"]["optional_pair_exposed"] is not False
            or outer["output"]["published_path"] != str(S5A0_FINAL_RECEIPT)):
        raise ExecutionError("S5A0 final receipt lacks exact PASS/final-evidence semantics")
    witness = outer["worker"]["inner_receipt"]
    if witness is None or witness["path"] != str(S5A0_INNER_RECEIPT):
        raise ExecutionError("S5A0 final receipt does not reference the exact inner receipt")
    inner_raw, inner_identity = read_file_identity(
        S5A0_INNER_RECEIPT, witness["sha256"], maximum_bytes=2 * 1024 * 1024
    )
    for key in ("path", "sha256", "size_bytes", "mode", "nlink"):
        if inner_identity[key] != witness[key]:
            raise ExecutionError(f"S5A0 inner receipt witness drifted: {key}")
    gate.validate_s5a0_receipt(inner_raw, str(S5A0_INNER_RECEIPT), policy)
    return outer_identity, inner_raw, inner_identity, outer


def execute_one_shot(receipt_path: Path, receipt_sha256: str) -> dict[str, Any]:
    if os.getuid() != 1000 or os.getgid() != 1000:
        raise ExecutionError("host uid/gid differs from 1000:1000")
    if PARTIAL_ROOT.exists() or PARTIAL_ROOT.is_symlink() or FINAL_ROOT.exists() or FINAL_ROOT.is_symlink():
        raise ExecutionError("partial or final root already exists")
    if (EXECUTION_RECEIPT.exists() or EXECUTION_RECEIPT.is_symlink()
            or EXECUTION_RECEIPT_PARTIAL.exists() or EXECUTION_RECEIPT_PARTIAL.is_symlink()):
        raise ExecutionError("execution receipt or reservation already exists")
    parent_identities = verify_parents()
    supervisor_identity = file_identity(SCRIPT)
    receipt_schema_identity = file_identity(RECEIPT_SCHEMA)
    policy, _ = gate.read_json(POLICY)
    receipt_identity, inner_raw, inner_identity, _outer = resolve_s5a0_final_receipt(
        receipt_path, receipt_sha256, policy
    )
    s5a0, _ = gate.validate_s5a0_receipt(inner_raw, str(S5A0_INNER_RECEIPT), policy)
    source_path = Path(s5a0["selection"]["absolute_path"])
    if source_path != Path(policy["source"]["bag_absolute_path"]):
        raise ExecutionError("receipt source is not the exact primary bag")
    compatibility_report = compatibility.self_check()
    compatibility_evidence = {
        "gate_sha256": EXPECTED[COMPAT_GATE], "status": compatibility_report["status"],
        "files_written": compatibility_report["files_modified"],
        "solver_executed": compatibility_report["solver_executed"],
        "gpu_exposed": compatibility_report["gpu_exposed"],
    }
    partial_before, final_before = root_identity(PARTIAL_ROOT), root_identity(FINAL_ROOT)
    receipt_descriptor = reserve_receipt()
    command_hash = sha256_bytes(b"")
    worker_report: dict[str, Any] | None = None
    real_bag_read_attempted = False
    real_bag_read_confirmed = False
    return_code = -1
    package: dict[str, bytes] | None = None
    atomic_rename = False
    failure: str | None = None
    source_before = _receipt_file_identity(s5a0["source"]["selected_after"])
    source_after = dict(source_before)
    source_unchanged = True
    inner_fd: int | None = None
    source_fd: int | None = None
    inner_fd_before: dict[str, Any] | None = None
    source_fd_before: dict[str, Any] | None = None
    package_summary = {"manifest_sha256": None, "checksums_sha256": None,
                       "inventory_sha256": None, "entry_count": 0,
                       "all_file_modes_0600": False, "validated": False}
    try:
        os.mkdir(PARTIAL_ROOT, 0o700)
        os.chmod(PARTIAL_ROOT, 0o700)
        inner_fd = os.open(S5A0_INNER_RECEIPT, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
        inner_fd_before = opened_fd_identity(
            inner_fd, S5A0_INNER_RECEIPT, inner_identity["sha256"], 2 * 1024 * 1024
        )
        assert_same_file_identity(inner_fd_before, inner_identity, "S5A0 inner receipt")
        source_fd = os.open(source_path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
        source_fd_before = opened_fd_identity(
            source_fd, source_path, policy["source"]["bag_sha256"]
        )
        assert_same_file_identity(source_fd_before, s5a0["source"]["selected_after"], "primary source")
        source_before = _receipt_file_identity(source_fd_before)
        source_after = dict(source_before)
        command = build_bwrap_argv(
            S5A0_INNER_RECEIPT, source_path, PARTIAL_ROOT,
            receipt_fd=inner_fd, source_fd=source_fd,
        )
        command_hash = sha256_bytes("\0".join(command).encode())
        real_bag_read_attempted = True
        completed = subprocess.run(command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, check=False, timeout=190,
                                   pass_fds=(inner_fd, source_fd))
        return_code = completed.returncode
        if len(completed.stdout) > 65536 or len(completed.stderr) > 65536:
            raise ExecutionError("worker output exceeds bound")
        if return_code != 0:
            raise ExecutionError(f"worker rc={return_code}: {completed.stderr.decode('utf-8', 'replace')}")
        worker_report = json.loads(completed.stdout)
        if worker_report.get("status") != "WORKER_PASS" or worker_report.get("real_bag_read") is not True:
            raise ExecutionError("worker did not return exact PASS/real-read evidence")
        inner_fd_after = opened_fd_identity(
            inner_fd, S5A0_INNER_RECEIPT, inner_identity["sha256"], 2 * 1024 * 1024
        )
        source_fd_after = opened_fd_identity(
            source_fd, source_path, policy["source"]["bag_sha256"]
        )
        assert_same_file_identity(inner_fd_after, inner_fd_before, "S5A0 inner receipt post-worker")
        assert_same_file_identity(source_fd_after, source_fd_before, "primary source post-worker")
        source_after = _receipt_file_identity(source_fd_after)
        if (_receipt_file_identity(worker_report["source_before"]) != source_before
                or _receipt_file_identity(worker_report["source_after"]) != source_after):
            raise ExecutionError("worker source identity differs from opened fd identity")
        inner_after = file_identity(S5A0_INNER_RECEIPT, inner_identity["sha256"])
        host_source_after = _receipt_file_identity(
            file_identity(source_path, policy["source"]["bag_sha256"])
        )
        if _receipt_file_identity(inner_after) != _receipt_file_identity(inner_identity):
            raise ExecutionError("S5A0 inner receipt path identity changed across worker")
        if host_source_after != _receipt_file_identity(s5a0["source"]["selected_after"]):
            raise ExecutionError("primary source path identity changed across worker")
        real_bag_read_confirmed = True
        package = load_package(PARTIAL_ROOT)
        package_summary = _package_summary(package, PARTIAL_ROOT)
        rename_noreplace(PARTIAL_ROOT, FINAL_ROOT)
        atomic_rename = True
        package = load_package(FINAL_ROOT)
    except Exception as error:
        failure = str(error)
    finally:
        for descriptor, before, path, digest, maximum, label in (
            (inner_fd, inner_fd_before, S5A0_INNER_RECEIPT, inner_identity["sha256"], 2 * 1024 * 1024, "inner fd"),
            (source_fd, source_fd_before, source_path, policy["source"]["bag_sha256"], 64 * 1024 * 1024, "source fd"),
        ):
            if descriptor is not None:
                try:
                    after = opened_fd_identity(descriptor, path, digest, maximum)
                    if before is not None:
                        assert_same_file_identity(after, before, label)
                    if label == "source fd":
                        source_after = _receipt_file_identity(after)
                except Exception as identity_error:
                    source_unchanged = False
                    failure = f"{failure}; {identity_error}" if failure else str(identity_error)
                finally:
                    os.close(descriptor)
        try:
            inner_path_after = file_identity(S5A0_INNER_RECEIPT, inner_identity["sha256"])
            source_path_after = file_identity(source_path, policy["source"]["bag_sha256"])
            if _receipt_file_identity(inner_path_after) != _receipt_file_identity(inner_identity):
                raise ExecutionError("S5A0 inner receipt path identity changed")
            if _receipt_file_identity(source_path_after) != _receipt_file_identity(s5a0["source"]["selected_after"]):
                source_unchanged = False
                raise ExecutionError("primary source path identity changed")
        except Exception as identity_error:
            failure = f"{failure}; {identity_error}" if failure else str(identity_error)

    source_unchanged = source_unchanged and source_before == source_after
    status = ("S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED" if failure is None
              else "S5A1_EXECUTION_FAILED_PRESERVED")
    next_stage = ("S5B0_REQUIRES_EXACT_TRANSFER_HASH_AND_NEW_GPU_AUTHORIZATION" if failure is None
                  else "STOP_AND_PRESERVE_EVIDENCE")
    partial_after, final_after = root_identity(PARTIAL_ROOT), root_identity(FINAL_ROOT)
    def captured_ref(path: Path) -> dict[str, str]:
        identity = parent_identities[str(path)]
        return {"path": identity["path"], "sha256": identity["sha256"]}
    receipt_value = {
        "schema_version": "smpcc-r8-liquid-s5a1-execution-receipt-v1",
        "document_type": "SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V1",
        "receipt_id": f"{TRANSFER_ID}_execution", "status": status,
        "parents": {
            "policy": captured_ref(POLICY), "package_schema": captured_ref(PACKAGE_SCHEMA),
            "handoff_gate": captured_ref(GATE), "signal_extractor": captured_ref(EXTRACTOR),
            "s5a0_receipt_schema": captured_ref(S5A0_SCHEMA),
            "s5a0_final_receipt_schema": captured_ref(S5A0_FINAL_SCHEMA),
            "s5a0_supervisor_policy": captured_ref(S5A0_SUPERVISOR_POLICY),
            "compatibility_gate": captured_ref(COMPAT_GATE),
            "execution_supervisor": {"path": supervisor_identity["path"], "sha256": supervisor_identity["sha256"]},
            "execution_receipt_schema": {"path": receipt_schema_identity["path"], "sha256": receipt_schema_identity["sha256"]},
        },
        "execution": {"uid": os.getuid(), "gid": os.getgid(),
                      "supplementary_groups": sorted(os.getgroups()), "bwrap_path": "/usr/bin/bwrap",
                      "timeout_path": "/usr/bin/timeout", "prlimit_path": "/usr/bin/prlimit",
                      "python_path": "/usr/bin/python3", "argv_sha256": command_hash,
                      "timeout_seconds": 180, "return_code": return_code, "one_shot": True},
        "inputs": {"s5a0_receipt": _receipt_file_identity(receipt_identity),
                   "source_before": source_before, "source_after": source_after,
                   "source_unchanged": source_unchanged, "source_provenance": "PARTIAL_BAG_ONLY"},
        "roots": {"partial_before": partial_before, "partial_after": partial_after,
                  "final_before": final_before, "final_after": final_after,
                  "atomic_rename": atomic_rename},
        "package": package_summary, "compatibility": compatibility_evidence,
        "safety": {"real_bag_read_attempted": real_bag_read_attempted,
                   "real_bag_read_confirmed": real_bag_read_confirmed,
                   "source_bag_executed": False,
                   "optional_bag_read": False, "ros_started": False, "gpu_exposed": False,
                   "solver_executed": False, "network_used": False, "sudo_used": False,
                   "failure_preserved": failure is not None},
        "failure": failure, "next": next_stage,
    }
    publish_reserved_receipt(
        receipt_descriptor, EXECUTION_RECEIPT_PARTIAL, EXECUTION_RECEIPT, receipt_value
    )
    if failure is not None:
        raise ExecutionError(failure)
    return receipt_value


def mock_failure_receipt() -> dict[str, Any]:
    digest = "0" * 64
    ref = lambda path: {"path": str(path), "sha256": digest}
    file_value = {"path": str(S5A0_FINAL_RECEIPT), "sha256": digest, "size_bytes": 1,
                  "mode": "0600", "inode": 1, "nlink": 1}
    absent_partial = {"path": str(PARTIAL_ROOT), "exists": False, "mode": "ABSENT",
                      "entry_count": 0, "inventory_sha256": None}
    absent_final = {"path": str(FINAL_ROOT), "exists": False, "mode": "ABSENT",
                    "entry_count": 0, "inventory_sha256": None}
    return {
        "schema_version": "smpcc-r8-liquid-s5a1-execution-receipt-v1",
        "document_type": "SMPCC_R8_LIQUID_S5A1_EXECUTION_RECEIPT_V1",
        "receipt_id": f"{TRANSFER_ID}_execution", "status": "S5A1_EXECUTION_FAILED_PRESERVED",
        "parents": {"policy": ref(POLICY), "package_schema": ref(PACKAGE_SCHEMA),
                    "handoff_gate": ref(GATE), "signal_extractor": ref(EXTRACTOR),
                    "s5a0_receipt_schema": ref(S5A0_SCHEMA),
                    "s5a0_final_receipt_schema": ref(S5A0_FINAL_SCHEMA),
                    "s5a0_supervisor_policy": ref(S5A0_SUPERVISOR_POLICY),
                    "compatibility_gate": ref(COMPAT_GATE), "execution_supervisor": ref(SCRIPT),
                    "execution_receipt_schema": ref(RECEIPT_SCHEMA)},
        "execution": {"uid": 1000, "gid": 1000, "supplementary_groups": [1000],
                      "bwrap_path": "/usr/bin/bwrap", "timeout_path": "/usr/bin/timeout",
                      "prlimit_path": "/usr/bin/prlimit", "python_path": "/usr/bin/python3",
                      "argv_sha256": digest, "timeout_seconds": 180, "return_code": 2, "one_shot": True},
        "inputs": {"s5a0_receipt": file_value, "source_before": file_value,
                   "source_after": file_value, "source_unchanged": True,
                   "source_provenance": "PARTIAL_BAG_ONLY"},
        "roots": {"partial_before": absent_partial, "partial_after": absent_partial,
                  "final_before": absent_final, "final_after": absent_final, "atomic_rename": False},
        "package": {"manifest_sha256": None, "checksums_sha256": None,
                    "inventory_sha256": None, "entry_count": 0,
                    "all_file_modes_0600": False, "validated": False},
        "compatibility": {"gate_sha256": EXPECTED[COMPAT_GATE],
                          "status": "PASS_S5_C1_TO_C1M_DEVELOPMENT_REPLAY_COMPATIBILITY_V1",
                          "files_written": False, "solver_executed": False, "gpu_exposed": False},
        "safety": {"real_bag_read_attempted": False, "real_bag_read_confirmed": False,
                   "source_bag_executed": False,
                   "optional_bag_read": False, "ros_started": False, "gpu_exposed": False,
                   "solver_executed": False, "network_used": False, "sudo_used": False,
                   "failure_preserved": True},
        "failure": "STATIC_MOCK_FAILURE", "next": "STOP_AND_PRESERVE_EVIDENCE",
    }


def static_self_check() -> dict[str, Any]:
    parents = verify_parents()
    schema = json.loads(RECEIPT_SCHEMA.read_bytes())
    Draft202012Validator.check_schema(schema)
    gate.assert_schema_deep_closed(schema)
    Draft202012Validator(schema).validate(mock_failure_receipt())
    policy = json.loads(POLICY.read_bytes())
    if Path(policy["package"]["planned_partial_root"]) != PARTIAL_ROOT or Path(policy["package"]["planned_final_root"]) != FINAL_ROOT:
        raise ExecutionError("execution roots differ from policy")
    argv = build_bwrap_argv(Path("/home/zrj/scout_liquid_lab/audits/exact-s5a0.json"),
                            Path(policy["source"]["bag_absolute_path"]), PARTIAL_ROOT)
    joined = "\0".join(argv).lower()
    for forbidden in ("sudo", "/dev/nvidia", "--share-net", "roscore", "dualsphysics5.4_linux64"):
        if forbidden in joined:
            raise ExecutionError(f"forbidden execution token: {forbidden}")
    for required in ("--unshare-all", "--clearenv", "--die-with-parent", "--bind", "--ro-bind"):
        if required not in argv:
            raise ExecutionError(f"missing sandbox token: {required}")
    return {"status": "PASS_S5A1_EXECUTION_SUPERVISOR_V1_SELF_CHECK",
            "verified_parent_count": len(parents), "argv_sha256": sha256_bytes("\0".join(argv).encode()),
            "real_bag_read_attempted": False, "real_bag_read_confirmed": False,
            "worker_run": False, "bwrap_run": False,
            "files_written": False, "network_used": False, "sudo_used": False,
            "gpu_exposed": False, "solver_executed": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("self-check")
    run_parser = sub.add_parser("run-one-shot")
    run_parser.add_argument("--s5a0-receipt", type=Path, required=True)
    run_parser.add_argument("--s5a0-receipt-sha256", required=True)
    run_parser.add_argument("--execute-token", required=True)
    worker_parser = sub.add_parser("_worker")
    worker_parser.add_argument("--s5a0-receipt", type=Path, required=True)
    worker_parser.add_argument("--source", type=Path, required=True)
    worker_parser.add_argument("--partial-root", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "self-check":
            result = static_self_check()
        elif args.command == "run-one-shot":
            if args.execute_token != "S5A1_ONE_SHOT_EXACT_PRIMARY":
                raise ExecutionError("exact one-shot execution token differs")
            result = execute_one_shot(args.s5a0_receipt, args.s5a0_receipt_sha256)
        else:
            result = worker(args.s5a0_receipt, args.source, args.partial_root)
    except Exception as error:
        print(json.dumps({"status": "FAIL", "error": str(error)}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, allow_nan=False, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
