#!/usr/bin/env python3
"""Create-new postvalidation for the completed V6 RTX 5080 GPU smoke.

The V6 solver returned zero and exported its exact 30-file output, then the
legacy log reader rejected a legitimate zero-byte stderr.  This validator
does not execute the candidate.  It admits empty stderr, rechecks all frozen
evidence and output bytes, runs the existing offline QC once, and publishes a
separate adjudication receipt without changing any V6 evidence.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import re
import stat
import sys
from pathlib import Path
from typing import Any


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
V6_RUNNER_PATH = SCRIPT_PATH.with_name("r8_liquid_u3_gpu_runtime_smoke_v6.py")
OUTPUT_ROOT = Path("/home/zrj/scout_liquid_lab/gpu_runs/u3_c1m_gpu_smoke_v6_20260810T190500Z.partial/output")
QC_REPORT = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_smoke_v6_20260810T190500Z.postvalidated_qc_v1.json")
FINAL_RECEIPT = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_smoke_v6_20260810T190500Z.postvalidation_v1.json")
CANDIDATE = {
    "path": "/home/zrj/scout_liquid_lab/build/u3_source_gpu_build_sm120_20260810T170339Z_a.partial/output/artifacts/DualSPHysics5.4_linux64",
    "sha256": "cace408f99c3ca75b53bfb542565e92ec134631a41f1d233aace346e6455b39f",
    "size_bytes": 105654136,
    "mode": "0400",
    "inode": 15733148,
    "nlink": 1,
}
PARENTS = {
    "v6_policy": (PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_runtime_smoke_v6.json", "de521172c88aa9c8dca47794bd79394627bac6b08bb8cb569b0ffd0751a81fb0"),
    "v6_schema": (PACKAGE_ROOT / "schema/target_host_u3_gpu_runtime_smoke_revision_v6.json", "b58378b78daa55ff2031f832d562ecff5b8117ae55d596938a6fa2a885bd0ec0"),
    "v6_runner": (V6_RUNNER_PATH, "42267675af6a63fd2337191b751e374e6c0bd2033acb9a10edae6bdd2395b6a1"),
    "v6_tests": (PACKAGE_ROOT / "tests/test_u3_gpu_runtime_smoke_v6.py", "e2ab84b66c7f731ccef2b8ee30563e4c8c436d0ba5410ea843321006998a265b"),
    "v6_probe": (Path("/home/zrj/scout_liquid_lab/audits/u3_gpu_smoke_v6_stage_probe_20260810T190500Z.json"), "b34a225f7ea5668b95501ce75eda6228ceca5f6194b63e334eca02a41ce901c2"),
    "v6_start": (Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_smoke_v6_20260810T190500Z.start.json"), "83f54562fec21c934783460f75641a5f9afba90e7004b001fbe7506759622c6e"),
    "v6_failure": (Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_smoke_v6_20260810T190500Z.failure.json"), "7ff2c8ccffad9ebff685e6c36076518a6cc3ab1c83f2e9075e1beb0b94351ae1"),
    "v6_resources": (Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_smoke_v6_20260810T190500Z.resources.jsonl"), "fc7ff1a0484dfa52e30ab6eafe11273374e7bf8b630f54152223c85b0a795463"),
    "v6_stdout": (Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_smoke_v6_20260810T190500Z.stdout.log"), "f567e38cc088c4c11013efcc7bedb596465f3977f80a60af40a04da9841d4dd1"),
    "v6_stderr": (Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_smoke_v6_20260810T190500Z.stderr.log"), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"),
}


def load_v6():
    spec = importlib.util.spec_from_file_location("r8_gpu_smoke_v6_for_postvalidation", V6_RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load v6 GPU smoke runner")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


v6 = load_v6()
base = v6.base


def read_regular_allow_empty(path: Path, maximum: int) -> tuple[bytes, os.stat_result]:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise base.SmokeError(f"unsafe postvalidation input: {path}")
        if metadata.st_size < 0 or metadata.st_size > maximum:
            raise base.SmokeError(f"postvalidation input exceeds {maximum}: {path}")
        data = bytearray()
        while len(data) < metadata.st_size:
            block = os.read(descriptor, min(1 << 20, metadata.st_size - len(data)))
            if not block:
                raise base.SmokeError(f"short postvalidation read: {path}")
            data.extend(block)
        if os.read(descriptor, 1):
            raise base.SmokeError(f"postvalidation input grew: {path}")
        return bytes(data), metadata
    finally:
        os.close(descriptor)


def identity_allow_empty(path: Path, maximum: int = 32 * 1024 * 1024) -> dict[str, Any]:
    raw, metadata = read_regular_allow_empty(path, maximum)
    return {
        "path": str(path),
        "sha256": hashlib.sha256(raw).hexdigest(),
        "size_bytes": metadata.st_size,
        "mode": f"0{stat.S_IMODE(metadata.st_mode):03o}",
        "uid": metadata.st_uid,
        "gid": metadata.st_gid,
        "inode": metadata.st_ino,
        "nlink": metadata.st_nlink,
    }


def verify_parents() -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for name, (path, digest) in PARENTS.items():
        observed = identity_allow_empty(path, maximum=16 * 1024 * 1024)
        if observed["sha256"] != digest:
            raise base.SmokeError(f"postvalidation parent drift: {name}: {path}")
        result[name] = observed
    return result


def log_safety() -> dict[str, Any]:
    stdout_path = PARENTS["v6_stdout"][0]
    stderr_path = PARENTS["v6_stderr"][0]
    stdout_raw, _ = read_regular_allow_empty(stdout_path, 32 * 1024 * 1024)
    stderr_raw, _ = read_regular_allow_empty(stderr_path, 32 * 1024 * 1024)
    text = (stdout_raw + b"\n" + stderr_raw).decode("utf-8", "replace")
    patterns = {
        "cuda_error": r"(?i)CUDA\s+(?:error|failure)|cudaError",
        "nvidia_xid": r"(?i)NVRM:\s*Xid|\bXid\b",
        "nan_inf": r"(?i)(?:^|[^A-Za-z])(nan|inf)(?:[^A-Za-z]|$)",
        "exception": r"\*\*\*\s*Exception|Simulation INTERRUPTED",
    }
    matches = {name: bool(re.search(pattern, text)) for name, pattern in patterns.items()}
    if any(matches.values()):
        raise base.SmokeError(f"postvalidated solver log safety failed: {matches}")
    markers = {
        "rtx5080": 'Device 0: "NVIDIA GeForce RTX 5080 Laptop GPU"' in text,
        "compute_12": "Compute capability: 12.0" in text,
        "single_gpu": 'RunMode="Stable - Pos-Cell - Single-GPU"' in text,
        "simulation_finished": "[Simulation finished" in text,
        "returncode_zero": "Finished execution (code=0)." in text,
        "particles_9078": "Total particles: 9,078" in text,
        "parts_21": "PART files.......................: 21" in text,
        "excluded_zero": "Excluded particles...............: 0" in text,
    }
    if not all(markers.values()) or stderr_raw:
        raise base.SmokeError(f"postvalidated completion markers differ: markers={markers} stderr={len(stderr_raw)}")
    return {
        "forbidden_matches": matches,
        "completion_markers": markers,
        "stdout_bytes": len(stdout_raw),
        "stderr_bytes": len(stderr_raw),
        "empty_stderr_admitted": len(stderr_raw) == 0,
    }


def resource_evidence() -> dict[str, Any]:
    raw, _ = read_regular_allow_empty(PARENTS["v6_resources"][0], 16 * 1024 * 1024)
    samples = [json.loads(line) for line in raw.decode("utf-8").splitlines() if line]
    if len(samples) < 2:
        raise base.SmokeError("too few V6 resource samples")
    for sample in samples:
        if sample["gpu"]["uuid"] != "GPU-d29cd22d-8b25-8f4e-9dcf-f2fc4bb113fb":
            raise base.SmokeError("GPU UUID drift in V6 resource log")
        if sample["memory"]["MemAvailableBytes"] < 4294967296:
            raise base.SmokeError("V6 resource sample fell below 4 GiB")
    memory_values = [sample["gpu"]["memory_used_mib"] for sample in samples]
    power_values = [sample["gpu"]["power_draw_w"] for sample in samples]
    if max(memory_values) <= min(memory_values) or max(power_values) < 60:
        raise base.SmokeError("V6 resource log does not prove active GPU use")
    return {
        "sample_count": len(samples),
        "elapsed_last_seconds": samples[-1]["elapsed_seconds"],
        "gpu_memory_used_mib_min": min(memory_values),
        "gpu_memory_used_mib_max": max(memory_values),
        "gpu_power_draw_w_max": max(power_values),
        "gpu_temperature_c_max": max(sample["gpu"]["temperature_c"] for sample in samples),
        "minimum_mem_available_bytes": min(sample["memory"]["MemAvailableBytes"] for sample in samples),
    }


def self_check() -> dict[str, Any]:
    if os.path.lexists(QC_REPORT) or os.path.lexists(FINAL_RECEIPT):
        raise base.SmokeError("postvalidation one-shot output already exists")
    parents = verify_parents()
    failure = json.loads(read_regular_allow_empty(PARENTS["v6_failure"][0], 1 << 20)[0])
    if (
        failure.get("candidate_executed") is not True
        or failure.get("candidate_mode_restored") is not True
        or "file size outside 1..33554432" not in failure.get("error", "")
    ):
        raise base.SmokeError("V6 failure is not the frozen empty-stderr validator defect")
    candidate = base.verify_file(CANDIDATE)
    return {
        "status": "PASS_GPU_SMOKE_V6_POSTVALIDATION_SELF_CHECK",
        "postvalidator": base.identity(SCRIPT_PATH, maximum=4 * 1024 * 1024),
        "postvalidator_tests": base.identity(
            PACKAGE_ROOT / "tests/test_u3_gpu_smoke_v6_postvalidate_v1.py",
            maximum=4 * 1024 * 1024,
        ),
        "parents": parents,
        "candidate": candidate,
        "candidate_executed": False,
        "system_action_used": False,
    }


def validate() -> dict[str, Any]:
    check = self_check()
    policy, _ = v6.load_contract()
    candidate_before = base.verify_file(CANDIDATE)
    inventory_before = base.scan_exact_output(
        OUTPUT_ROOT, maximum_total=policy["limits"]["maximum_output_bytes"]
    )
    if inventory_before["file_count"] != 30 or inventory_before["part_file_count"] != 21:
        raise base.SmokeError("V6 output inventory differs")
    logs = log_safety()
    resources = resource_evidence()
    if base.compute_processes(policy):
        raise base.SmokeError("GPU compute process exists during postvalidation")
    qc_policy = json.loads(json.dumps(policy))
    qc_policy["run"]["qc_report"] = str(QC_REPORT)
    qc_report, qc_identity, qc_argv = base.run_qc(qc_policy)
    qc_status = qc_report.get("verdict", {}).get("status", qc_report.get("status"))
    if qc_status != "C1M_ZERO_MOTION_SMOKE_PASS":
        raise base.SmokeError(f"V6 output QC differs: {qc_status}")
    candidate_after = base.verify_file(CANDIDATE)
    inventory_after = base.scan_exact_output(
        OUTPUT_ROOT, maximum_total=policy["limits"]["maximum_output_bytes"]
    )
    if (
        candidate_before != candidate_after
        or inventory_before["canonical_sha256"] != inventory_after["canonical_sha256"]
        or inventory_before["total_bytes"] != inventory_after["total_bytes"]
    ):
        raise base.SmokeError("candidate or V6 output changed during postvalidation")
    receipt = {
        "schema_version": "r8-liquid-u3-gpu-smoke-v6-postvalidation-v1",
        "document_type": "SMPCC_R8_LIQUID_U3_GPU_SMOKE_V6_POSTVALIDATION_V1",
        "status": "PASS_GPU_FUNCTIONAL_SMOKE_DEVELOPMENT_ONLY",
        "adjudication": "V6_SOLVER_PASS_LEGACY_EMPTY_STDERR_READER_FALSE_NEGATIVE",
        "captured_at_utc": base.utc_now(),
        "run_id": "u3_c1m_gpu_smoke_v6_20260810T190500Z",
        "solver_returncode": 0,
        "solver_returncode_evidence": "EXPORT_ONLY_ON_ZERO_RETURN_PLUS_FINISHED_EXECUTION_CODE_0",
        "solver_elapsed_seconds": 38.540202,
        "physical_time_seconds": 1.0,
        "steps": 22054,
        "particles": 9078,
        "moving_particles": 2669,
        "fluid_particles": 6409,
        "excluded_particles": 0,
        "part_file_count": 21,
        "candidate_executed_in_parent_run": True,
        "candidate_executed_by_postvalidator": False,
        "candidate_before": candidate_before,
        "candidate_after": candidate_after,
        "output_inventory": inventory_after,
        "log_safety": logs,
        "resource_evidence": resources,
        "qc_argv": qc_argv,
        "qc": qc_identity,
        "qc_status": qc_status,
        "qc_development_only": True,
        "compute_processes_after": [],
        "network_used": False,
        "project_workspace_writable_in_guest": False,
        "gpu_runtime_status": "PASS_GPU_FUNCTIONAL_SMOKE_DEVELOPMENT_ONLY",
        "development_only": True,
        "formal": False,
        "physical_primary_eligible": False,
        "stage3_complete": True,
        "next": "STAGE4_LIQUID_STANDALONE_VALIDATION",
        "self_check": check,
    }
    publication = base.write_json_exclusive(FINAL_RECEIPT, receipt)
    receipt["final_receipt"] = publication
    return receipt


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("command", choices=("self-check", "validate"))
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        result = self_check() if args.command == "self-check" else validate()
    except (base.SmokeError, OSError, ValueError, json.JSONDecodeError) as exc:
        print(
            base.canonical_bytes(
                {"status": "FAIL_GPU_SMOKE_V6_POSTVALIDATION", "error_type": type(exc).__name__, "error": str(exc)}
            ).decode("utf-8"),
            end="", file=sys.stderr,
        )
        return 1
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
