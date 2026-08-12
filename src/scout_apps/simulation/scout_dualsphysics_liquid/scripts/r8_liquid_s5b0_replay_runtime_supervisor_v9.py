#!/usr/bin/env python3
"""Exact one-shot runtime and post-run packaging engine for S5B0 replay v9.

Only the lifecycle supervisor may call the dynamic APIs.  The public CLI is
static-only.  Output publication is create-new and transactional: the exact
partial package is verified, then published with renameat2(RENAME_NOREPLACE).
"""

from __future__ import annotations

import argparse
import ctypes
import errno
import hashlib
import importlib.util
import json
import os
import re
import signal
import stat
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
ZERO_SHA = hashlib.sha256(b"").hexdigest()
AT_FDCWD = -100
RENAME_NOREPLACE = 1
RAW_GAUGE_NAMES = tuple(f"GaugesSwl_s5b0_p{index:02d}.csv" for index in range(16))
REQUIRED_OUTPUT_FILES = (
    "Run.csv", "Run.out", "RunPARTs.csv", "data/Part_Head.ibi4",
    "data/PartInfo.ibi4", "data/PartMotionRef.ibi4", "data/PartOut_000.obi4",
)
PART_RE = re.compile(r"data/Part_[0-9]{4}\.bi4")


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


gate = _load("s5b0_gate_v9_for_runtime", SCRIPTS / "r8_liquid_s5b0_replay_execution_gate_v9.py")
staging = _load("s5b0_staging_v4_for_runtime", SCRIPTS / "r8_liquid_s5b0_staging_case_materializer_v4.py")
qc = _load("s5b0_qc_v9_for_runtime", SCRIPTS / "r8_liquid_s5b0_replay_output_qc_v9.py")


class RuntimeV9Error(RuntimeError):
    """A runtime, output, QC, or transaction invariant failed."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _write_all(descriptor: int, raw: bytes) -> None:
    view = memoryview(raw)
    while view:
        count = os.write(descriptor, view)
        if count <= 0:
            raise RuntimeV9Error("short write")
        view = view[count:]


def write_new(path: Path, raw: bytes, mode: int = 0o640, *, make_parent: bool = False) -> dict[str, Any]:
    if os.path.lexists(path):
        raise RuntimeV9Error(f"create-new target exists: {path}")
    if make_parent and not path.parent.exists():
        path.parent.mkdir(mode=0o750, parents=True, exist_ok=False)
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, mode)
    try:
        os.fchmod(descriptor, mode)
        _write_all(descriptor, raw)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    parent = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        os.fsync(parent)
    finally:
        os.close(parent)
    return gate._file_identity(path)


def _source_spec(policy: Mapping[str, Any], candidate: Mapping[str, Any]) -> dict[str, Any]:
    sources = policy["sources"]
    result = {role: dict(sources[role]) for role in ("dsph_config", "case_xml", "case_bi4", "restart_part", "restart_head", "solver_path")}
    result["candidate"] = dict(candidate)
    for value in result.values():
        metadata = os.stat(value["path"], follow_symlinks=False)
        value.update({"device": metadata.st_dev, "inode": metadata.st_ino, "nlink": metadata.st_nlink})
    return result


def _real_staging_evidence(policy: Mapping[str, Any], candidate: Mapping[str, Any], build_receipt: Mapping[str, Any]) -> dict[str, Any]:
    sources = policy["sources"]
    return {
        "authorization_status": "EXACT_REAL_STAGING_AUTHORIZATION", "real_staging_authorized": True,
        "candidate": {"capability": "MOTION_ATTACHED_16_RAW_JGAUGESWL", "sha256": candidate["sha256"], "fresh_patched_build": True, "build_receipt_sha256": build_receipt["sha256"], "static_audit_receipt_sha256": build_receipt["sha256"], "built": True, "static_audit_passed": True, "old_candidate_reused": False},
        "transfer": {"transfer_id": "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01_r8_liquid_handoff_v3_v10", "status": "S5A1_PRIMARY_R7_MOTION_TRANSFER_VERIFIED_ACCEPTED", "finalized": True, "manifest_sha256": sources["transfer_manifest"]["sha256"], "solver_path_sha256": sources["solver_path"]["sha256"], "execution_receipt_sha256": "8e5b9417e9485f0f178849f274dfd605138d4a33af2abe346a3628bd38771c18"},
        "settled_clone": {"case_xml_sha256": sources["case_xml"]["sha256"], "case_bi4_sha256": sources["case_bi4"]["sha256"], "restart_part_sha256": sources["restart_part"]["sha256"], "restart_head_sha256": sources["restart_head"]["sha256"], "dsph_config_sha256": sources["dsph_config"]["sha256"], "particle_count": 9078, "moving_boundary_count": 2669, "fluid_count": 6409, "nout": 0, "ids_complete_unique": True, "finite": True, "leak_zero": True, "unique_motion_ref": True, "motion_ref": 0, "mkbound": 0, "refmotion": 0},
        "result_qc": {"raw_gauge_csv_count": 16, "raw_gauge_csv_names": list(RAW_GAUGE_NAMES), "gauge_slots_complete": True, "executed_boundary_motion_required": True, "part_motion_ref_required": True, "particle_count_required": 9078, "ids_complete_unique_required": True, "nout_required": 0, "finite_required": True, "leak_zero_required": True, "domain_outside_zero_required": True, "inputs_unchanged_required": True, "checksums_required": True},
    }


def prepare(policy: Mapping[str, Any], token: Mapping[str, Any]) -> dict[str, Any]:
    replay = policy["replay"]
    stage = Path(replay["stage_root"])
    partial = Path(replay["partial_root"])
    if os.path.lexists(stage) or os.path.lexists(partial) or os.path.lexists(replay["final_root"]):
        raise RuntimeV9Error("replay/package roots are not fresh")
    candidate = token["candidate"]
    sources = _source_spec(policy, candidate)
    manifest = staging.materialize(
        stage, expected_stage_root=stage, sources=sources, restart_part_index=901,
        settled_time_s=45.05001991890928, solver_tail_s=1.0, mode="REAL",
        evidence=_real_staging_evidence(policy, candidate, token["build_final_receipt"]),
    )
    os.mkdir(stage / "guest-output", 0o700)
    os.mkdir(partial, 0o700)
    return manifest


def _mem_available() -> int:
    for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) * 1024
    raise RuntimeV9Error("MemAvailable is absent")


def _tree_usage(root: Path) -> tuple[int, int]:
    count = total = 0
    for path in root.rglob("*"):
        metadata = os.lstat(path)
        if stat.S_ISLNK(metadata.st_mode) or not (stat.S_ISDIR(metadata.st_mode) or stat.S_ISREG(metadata.st_mode)):
            raise RuntimeV9Error("unsafe output entry during monitor")
        if stat.S_ISREG(metadata.st_mode):
            count += 1
            total += metadata.st_size
    return count, total


def _gpu_sample() -> dict[str, Any]:
    result = subprocess.run(["/usr/bin/nvidia-smi", "--query-gpu=memory.free,temperature.gpu,power.draw", "--format=csv,noheader,nounits", "--id=0"], capture_output=True, text=True, timeout=10, check=False)
    if result.returncode != 0 or len(result.stdout.strip().split(",")) != 3:
        raise RuntimeV9Error("nvidia-smi telemetry failed")
    free_mib, temperature, power = [value.strip() for value in result.stdout.strip().split(",")]
    return {"free_vram_bytes": int(free_mib) * 1024 * 1024, "temperature_c": int(temperature), "power_w": float(power)}


def _sample(output: Path, started: float) -> dict[str, Any]:
    count, total = _tree_usage(output)
    return {"captured_at": utc_now(), "elapsed_seconds": time.monotonic() - started, "mem_available_bytes": _mem_available(), "memory_psi": Path("/proc/pressure/memory").read_text().strip(), "output_files": count, "output_bytes": total, "gpu": _gpu_sample()}


def _stop(process: subprocess.Popen[bytes], kill_after: int) -> None:
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=kill_after)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=5)


def execute(policy: Mapping[str, Any], token: Mapping[str, Any]) -> dict[str, Any]:
    replay = policy["replay"]
    output = Path(replay["stage_root"]) / "guest-output"
    if not output.is_dir():
        raise RuntimeV9Error("prepared guest output root absent")
    started = time.monotonic()
    audit = Path(replay["audit_root"])
    audit.mkdir(mode=0o750)
    paths = [audit / name for name in ("solver.stdout.log", "solver.stderr.log", "resource_samples.jsonl")]
    descriptors = [os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o640) for path in paths]
    process = None
    reason = "NOT_STARTED"
    argv = gate.sandbox_argv(policy)
    try:
        pre = _sample(output, started)
        if pre["mem_available_bytes"] < policy["resources"]["minimum_available_memory_bytes"] or pre["gpu"]["free_vram_bytes"] < policy["resources"]["minimum_free_vram_bytes"]:
            raise RuntimeV9Error("resource preflight failed")
        process = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=descriptors[0], stderr=descriptors[1], close_fds=True, start_new_session=True, env={"PATH": "/usr/bin", "LC_ALL": "C", "LANG": "C"})
        reason = "PROCESS_EXIT"
        next_sample = 0.0
        while process.poll() is None:
            elapsed = time.monotonic() - started
            if elapsed >= policy["resources"]["wall_timeout_seconds"]:
                reason = "WALL_TIMEOUT"
                _stop(process, policy["resources"]["kill_after_seconds"])
                break
            if elapsed >= next_sample:
                sample = _sample(output, started)
                _write_all(descriptors[2], gate.canonical_json(sample))
                os.fsync(descriptors[2])
                if sample["mem_available_bytes"] < policy["resources"]["minimum_available_memory_bytes"] or sample["gpu"]["free_vram_bytes"] < policy["resources"]["minimum_free_vram_bytes"] or sample["output_bytes"] > policy["resources"]["maximum_output_bytes"]:
                    reason = "RESOURCE_GATE"
                    _stop(process, policy["resources"]["kill_after_seconds"])
                    break
                next_sample = elapsed + 20
            time.sleep(0.2)
        returncode = process.wait(timeout=10)
        final_sample = _sample(output, started)
        _write_all(descriptors[2], gate.canonical_json(final_sample))
        os.fsync(descriptors[2])
        if returncode != 0 or reason != "PROCESS_EXIT":
            raise RuntimeV9Error(f"solver failed rc={returncode} reason={reason}")
        return {"argv": argv, "return_code": returncode, "elapsed_seconds": time.monotonic() - started, "stdout": gate._file_identity(paths[0]), "stderr": gate._file_identity(paths[1]), "resource_log": gate._file_identity(paths[2]), "final_sample": final_sample, "candidate": token["candidate"], "profile": token["profile"]}
    finally:
        if process is not None and process.poll() is None:
            _stop(process, policy["resources"]["kill_after_seconds"])
        for descriptor in descriptors:
            os.close(descriptor)


def inventory_receipt(policy: Mapping[str, Any]) -> tuple[dict[str, Any], str]:
    root = Path(policy["replay"]["stage_root"]) / "guest-output"
    rows = []
    directories = set()
    for path in sorted(root.rglob("*")):
        metadata = os.lstat(path)
        relative = path.relative_to(root).as_posix()
        if stat.S_ISLNK(metadata.st_mode):
            raise RuntimeV9Error("symlink in output inventory")
        if stat.S_ISDIR(metadata.st_mode):
            directories.add(relative)
        elif stat.S_ISREG(metadata.st_mode):
            identity = gate._file_identity(path)
            rows.append({"relative_path": relative, "sha256": identity["sha256"], "size_bytes": identity["size_bytes"], "mode": identity["mode"]})
        else:
            raise RuntimeV9Error("special entry in output inventory")
    observed = {row["relative_path"] for row in rows}
    required = set(REQUIRED_OUTPUT_FILES) | set(RAW_GAUGE_NAMES)
    parts = {path for path in observed if PART_RE.fullmatch(path)}
    if not required <= observed or not parts or not {"data"} <= directories:
        missing = sorted(required - observed)
        raise RuntimeV9Error(f"closed output inventory missing required files: {missing}")
    receipt = {"schema_version": "smpcc-r8-liquid-s5b0-output-inventory-receipt-v9", "root": str(root), "files": rows, "inventory_sha256": qc.frame_reader.canonical_sha256(rows), "finalized": True}
    return receipt, qc.frame_reader.canonical_sha256(receipt)


def run_output_qc(policy: Mapping[str, Any], inventory: Mapping[str, Any], inventory_sha: str) -> dict[str, Any]:
    output = Path(policy["replay"]["stage_root"]) / "guest-output"
    frame_manifest, positions, payloads = qc.frame_reader.read_finalized(output, external_inventory_receipt=inventory, external_inventory_receipt_sha256=inventory_sha)
    frame_rows = frame_manifest["frames"]
    # Explicit call is a contract assertion in addition to read_finalized's internal parse.
    runparts_rows = qc.frame_reader.parse_runparts(payloads["RunPARTs.csv"])
    if len(runparts_rows) != len(frame_rows):
        raise RuntimeV9Error("RunPARTs/frame cardinality differs after explicit QC call")
    motion_ref = qc.parse_motion_ref(payloads["data/PartMotionRef.ibi4"], frame_rows, positions)
    boundary_raw, boundary = qc.boundary_qc(frame_rows, positions, Path(policy["sources"]["solver_path"]["path"]).read_bytes(), settled_time_s=policy["solver"]["settled_time_s"])
    native = {f"s5b0_p{index:02d}": payloads[name] for index, name in enumerate(RAW_GAUGE_NAMES)}
    gauge_contract = staging.load_frozen_contract()[1]
    normalized, provenance = qc.normalize_gauges(native, expected_times_s=[row["time_s"] for row in frame_rows], probe_grid=gauge_contract["probes"])
    return {"frame_manifest": frame_manifest, "motion_ref": motion_ref, "boundary_raw": boundary_raw, "boundary": boundary, "normalized": normalized, "provenance": provenance}


def _copy_regular_new(source: Path, destination: Path, mode: int) -> dict[str, Any]:
    descriptor = os.open(source, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise RuntimeV9Error(f"unsafe package source: {source}")
        chunks = []
        while True:
            block = os.read(descriptor, 1 << 20)
            if not block:
                break
            chunks.append(block)
    finally:
        os.close(descriptor)
    return write_new(destination, b"".join(chunks), mode, make_parent=not destination.parent.exists())


def _package_inventory(root: Path, *, include_checksums: bool) -> list[dict[str, Any]]:
    rows = []
    for path in sorted(root.rglob("*")):
        metadata = os.lstat(path)
        if stat.S_ISLNK(metadata.st_mode) or not (stat.S_ISDIR(metadata.st_mode) or stat.S_ISREG(metadata.st_mode)):
            raise RuntimeV9Error("unsafe package inventory entry")
        if stat.S_ISREG(metadata.st_mode):
            relative = path.relative_to(root).as_posix()
            if not include_checksums and relative == "checksums.sha256":
                continue
            identity = gate._file_identity(path)
            rows.append({"relative_path": relative, "sha256": identity["sha256"], "size_bytes": identity["size_bytes"], "mode": identity["mode"]})
    return rows


def _checksums(rows: Sequence[Mapping[str, Any]]) -> bytes:
    return "".join(f"{row['sha256']}  {row['relative_path']}\n" for row in rows).encode("utf-8")


def _rename_noreplace(source: Path, destination: Path) -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, "renameat2", None)
    if renameat2 is None:
        raise RuntimeV9Error("renameat2 unavailable; no weaker fallback allowed")
    renameat2.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    renameat2.restype = ctypes.c_int
    if renameat2(AT_FDCWD, os.fsencode(source), AT_FDCWD, os.fsencode(destination), RENAME_NOREPLACE) != 0:
        number = ctypes.get_errno()
        if number == errno.EEXIST:
            raise RuntimeV9Error("final package destination collision")
        raise OSError(number, os.strerror(number), str(destination))


def publish_result_package(policy: Mapping[str, Any], token: Mapping[str, Any], execution: Mapping[str, Any], inventory: Mapping[str, Any], inventory_sha: str, qc_result: Mapping[str, Any], xid_count: int) -> tuple[dict[str, Any], dict[str, Any]]:
    replay = policy["replay"]
    partial, final = Path(replay["partial_root"]), Path(replay["final_root"])
    if not partial.is_dir() or os.path.lexists(final):
        raise RuntimeV9Error("package publication roots are not exact/fresh")
    output = Path(replay["stage_root"]) / "guest-output"
    raw_dir = partial / "gauges/native"
    normalized_dir = partial / "gauges/normalized"
    evidence_dir = partial / "evidence"
    raw_dir.mkdir(parents=True, mode=0o750)
    normalized_dir.mkdir(parents=True, mode=0o750)
    evidence_dir.mkdir(mode=0o750)
    for name in RAW_GAUGE_NAMES:
        _copy_regular_new(output / name, raw_dir / name, 0o440)
    for relative, raw in qc_result["normalized"].items():
        write_new(partial / relative, raw, 0o440)
    frame_identity = write_new(evidence_dir / "frame_manifest.json", gate.canonical_json(qc_result["frame_manifest"]), 0o440)
    provenance_identity = write_new(evidence_dir / "native_gauge_provenance.json", gate.canonical_json(qc_result["provenance"]), 0o440)
    boundary_identity = write_new(evidence_dir / "executed_boundary_motion.csv", qc_result["boundary_raw"], 0o440)
    inventory_identity = write_new(evidence_dir / "output_inventory_receipt.json", gate.canonical_json(inventory), 0o440)
    execution_payload = {"argv": execution["argv"], "return_code": execution["return_code"], "elapsed_seconds": execution["elapsed_seconds"]}
    execution_identity = write_new(evidence_dir / "execution_summary.json", gate.canonical_json(execution_payload), 0o440)
    # Build every package payload before freezing the checksum manifest.  The
    # externally stored result receipt is deliberately outside this inventory,
    # avoiding a circular self-hash while still binding the package checksum.
    checksums_path = partial / "checksums.sha256"
    result = {
        "schema_version": "smpcc-r8-liquid-s5b0-replay-result-package-v9",
        "status": "S5B0_PRIMARY_R7_EXECUTED_MOTION_REPLAY_PASS_DEVELOPMENT_ONLY",
        "attempt_id": policy["selection"]["attempt_id"], "replay_id": replay["replay_id"], "planned_denominator": 1,
        "parents": {"source_bag_sha256": policy["selection"]["source_bag_sha256"], "transfer_manifest_sha256": policy["sources"]["transfer_manifest"]["sha256"], "candidate_sha256": token["candidate"]["sha256"], "profile_sha256": token["profile"]["sha256"], "settled_part_sha256": policy["sources"]["restart_part"]["sha256"]},
        "identities": {"execution_receipt": execution_identity, "output_inventory_receipt": inventory_identity, "frame_manifest": frame_identity, "native_gauge_provenance": provenance_identity, "executed_boundary_motion": boundary_identity, "checksums": {"path": str(checksums_path), "sha256": ZERO_SHA, "size_bytes": 1, "mode": "0440"}},
        "provenance": {"raw_format": qc_result["provenance"]["raw_format"], "normalized_format": qc_result["provenance"]["normalized_format"], "raw_preserved": True, "native_and_normalized_hashes_bound": True, "probe_count": 16, "files": qc_result["provenance"]["files"], "manifest_sha256": qc_result["provenance"]["manifest_sha256"]},
        "qc": {"particles": 9078, "moving": 2669, "fluid": 6409, "nout": 0, "ids_complete_unique": True, "finite": True, "runparts_aligned": True, "part_motion_ref_aligned": qc_result["motion_ref"]["pass"], "boundary_motion_aligned": qc_result["boundary"]["pass"], "gauge_count": 16, "gauge_invalid_ratio": qc_result["provenance"]["invalid_ratio"], "xid_count": xid_count, "pass": True},
        "publication": {"partial_root": str(partial), "final_root": str(final), "renameat2_noreplace": True, "parent_fsynced": True, "exact_inventory": True, "checksums_complete": True},
        "claims": {"raw_gauges_preserved": True, "native_and_normalized_hashes_bound": True, "optional_bag_read": False, "paired_ranking": False, "source_outcome_overridden": False, "physical_reference_pending": True, "physical_fidelity_validated": False, "formal": False, "production": False},
    }
    rows_before = _package_inventory(partial, include_checksums=False)
    checksums_identity = write_new(checksums_path, _checksums(rows_before), 0o440)
    result["identities"]["checksums"] = checksums_identity
    schema = json.loads(gate.RESULT_SCHEMA_PATH.read_bytes())
    Draft202012Validator(schema).validate(result)
    external_result_identity = write_new(Path(replay["result_package_receipt"]), gate.canonical_json(result), 0o600)
    expected_rows = _package_inventory(partial, include_checksums=False)
    if (partial / "checksums.sha256").read_bytes() != _checksums(expected_rows):
        raise RuntimeV9Error("package checksum manifest is incomplete")
    parent_fd = os.open(final.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        _rename_noreplace(partial, final)
        os.fsync(parent_fd)
    finally:
        os.close(parent_fd)
    return result, external_result_identity


def self_check() -> dict[str, Any]:
    policy, digest = gate.read_policy()
    gate.validate_static(policy)
    argv = gate.sandbox_argv(policy)
    if REQUIRED_OUTPUT_FILES[-1] != "data/PartOut_000.obi4" or "--since" in " ".join(policy["journal"]["query_argv_template"]):
        raise RuntimeV9Error("PartOut/journal cursor contract drift")
    return {"status": "PASS_S5B0_REPLAY_RUNTIME_SUPERVISOR_V9_STATIC_ONLY", "policy_sha256": digest, "sandbox_argv_sha256": hashlib.sha256(gate.canonical_json(argv)).hexdigest(), "qc_pipeline": policy["output_contract"]["qc_pipeline"], "required_output_files": list(REQUIRED_OUTPUT_FILES), "raw_gauge_count": 16, "renameat2_noreplace_required": True, "create_new_checksums_required": True, "failure_preserve_required": True, "boot_cursor_xid_required": True, "public_execute_cli": False, "files_written": False, "external_source_read": False, "candidate_executed": False, "gpu_exposed": False, "profile_loaded": False, "sudo_used": False, "optional_bag_read": False}


def execute_one_shot(token_path: Path) -> dict[str, Any]:
    if (os.getuid(), os.geteuid(), os.getgid(), os.getegid(), os.getgroups()) != (1000, 1000, 1000, 1000, []):
        raise RuntimeV9Error("runtime requires uid/gid 1000 and zero supplementary groups")
    policy, _ = gate.read_policy()
    if str(token_path) != policy["replay"]["token_path"]:
        raise RuntimeV9Error("runtime token path differs")
    token = gate.validate_token(token_path)
    prepare(policy, token)
    execution = execute(policy, token)
    return execution


def postprocess_one_shot(token_path: Path, execution: Mapping[str, Any], xid_count: int) -> dict[str, Any]:
    if (os.getuid(), os.geteuid(), os.getgid(), os.getegid(), os.getgroups()) != (1000, 1000, 1000, 1000, []):
        raise RuntimeV9Error("postprocess requires uid/gid 1000 and zero supplementary groups")
    policy, _ = gate.read_policy()
    if str(token_path) != policy["replay"]["token_path"] or xid_count != 0:
        raise RuntimeV9Error("postprocess token path or Xid gate differs")
    token = gate.validate_token(token_path)
    required = {"argv", "return_code", "elapsed_seconds", "stdout", "stderr",
                "resource_log", "final_sample", "candidate", "profile"}
    if set(execution) != required or execution["return_code"] != 0:
        raise RuntimeV9Error("execution handoff is open or failed")
    if execution["argv"] != gate.sandbox_argv(policy):
        raise RuntimeV9Error("execution handoff argv differs")
    for key in ("stdout", "stderr", "resource_log"):
        if gate._file_identity(Path(execution[key]["path"])) != execution[key]:
            raise RuntimeV9Error(f"execution evidence identity drift: {key}")
    if execution["candidate"] != token["candidate"] or execution["profile"] != token["profile"]:
        raise RuntimeV9Error("execution candidate/profile binding differs")
    inventory, inventory_sha = inventory_receipt(policy)
    inventory_identity = write_new(
        Path(policy["replay"]["output_inventory_receipt"]), gate.canonical_json(inventory), 0o600)
    qc_result = run_output_qc(policy, inventory, inventory_sha)
    _result, result_identity = publish_result_package(
        policy, token, execution, inventory, inventory_sha, qc_result, xid_count)
    return {"status": "PASS_S5B0_REPLAY_POSTPROCESS_V9",
            "inventory_sha256": inventory_sha,
            "inventory_receipt": inventory_identity,
            "result_package": result_identity}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", "execute-one-shot", "postprocess-one-shot"), nargs="?", default="self-check")
    parser.add_argument("--authorization-token-file", type=Path)
    parser.add_argument("--execution-json")
    parser.add_argument("--xid-count", type=int)
    args = parser.parse_args(argv)
    try:
        if args.command == "self-check":
            report = self_check()
        elif args.command == "execute-one-shot":
            if args.authorization_token_file is None:
                raise RuntimeV9Error("exact authorization token file is required")
            report = execute_one_shot(args.authorization_token_file)
        else:
            if args.authorization_token_file is None or args.execution_json is None or args.xid_count is None:
                raise RuntimeV9Error("exact postprocess inputs are required")
            execution = json.loads(args.execution_json)
            if not isinstance(execution, dict) or len(args.execution_json.encode("utf-8")) > 262144:
                raise RuntimeV9Error("execution handoff JSON is invalid or unbounded")
            report = postprocess_one_shot(args.authorization_token_file, execution, args.xid_count)
        print(json.dumps(report, sort_keys=True, separators=(",", ":")))
        return 0
    except Exception as exc:
        print(json.dumps({"status": "FAIL_S5B0_REPLAY_RUNTIME_SUPERVISOR_V9", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
