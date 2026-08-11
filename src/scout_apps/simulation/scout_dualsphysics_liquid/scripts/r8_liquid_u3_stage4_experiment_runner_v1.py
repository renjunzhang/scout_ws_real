#!/usr/bin/env python3
"""One-shot rootless runner for frozen Stage-4 CPU/GPU experiments.

The runner accepts one closed policy, stages immutable inputs into a fresh lab
root, and executes exactly one DualSPHysics process inside a capability-free,
network-isolated bubblewrap guest.  The workspace is never bound into the
guest and the only host-writable bind is a fresh guest-output directory.
Inputs, logs, inventories and O_EXCL receipts remain available for audit.
"""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import math
import os
import re
import signal
import stat
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator

import r8_liquid_u3_gpu_stage4_runner_v4 as legacy


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_stage4_experiment_policy_v1.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_stage4_experiment_runner_v1.py"
CONFIG_ROOT = PACKAGE_ROOT / "config/target_hosts"
LAB_ROOT = Path("/home/zrj/scout_liquid_lab")
RUNS_ROOT = LAB_ROOT / "stage4_experiments"
AUDIT_ROOT = LAB_ROOT / "audits"
LOCK_ROOT = LAB_ROOT / "locks"
PLAN_PATH = PACKAGE_ROOT.parents[3] / "docs/实物实验注意事项/对比试验/仿真接入液体/20260810_RTX5080_DualSPHysics_GPU_8小时快速构建方案_Agent执行版.md"
PLAN_SHA256 = "9a17c2296417b2ea3bc0b65a710e88e287a99abc4cbf6e264857efe06d1bd27d"
NUMERICAL_CONTRACT_PATH = CONFIG_ROOT / "liquid_zrj_msi_u2404_u3_stage4_numerical_contract_v1.json"
NUMERICAL_CONTRACT_SHA256 = "7b3d7447485d901cd60402f2a7ddd422c587ffa57ba3e5664130d8cd3abd9cd7"
SETTLED_QC_PATH = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_viscoart0p3_repeatability_restart_20260811T124800Z_v15.qc_v1.json")
SETTLED_QC_SHA256 = "4703c08e09b33fb16ad68b368dd5e99c2b2c930a3091008fa399241fc58a7fd8"
CPU_CANDIDATE_PATH = Path("/home/zrj/scout_liquid_lab/build/u3_source_cpu_build_20260807T011031Z.partial/output/artifacts/DualSPHysics5.4CPU_linux64")
CPU_CANDIDATE_SHA256 = "5aa464a8f37b0185bac863987f0d1079a0f1a3d6daead6581562c832278ea202"
GPU_CANDIDATE_PATH = Path("/home/zrj/scout_liquid_lab/build/u3_source_gpu_build_sm120_20260810T170339Z_a.partial/output/artifacts/DualSPHysics5.4_linux64")
GPU_CANDIDATE_SHA256 = "cace408f99c3ca75b53bfb542565e92ec134631a41f1d233aace346e6455b39f"
BASE_CASE_BI4_SHA256 = "b463ddfe548b3db78b02f23b075dadbdd3c71ea766eb092701b56978ddb3a8e7"
BASE_CASE_XML_SHA256 = "28205c2234dda565da600947d03481c492c6bb493b2e058bd04cd360b7862acb"
SETTLED_PART_PATH = Path("/home/zrj/scout_liquid_lab/gpu_stage4_runs/u3_c1m_gpu_viscoart0p3_cold_b_20260811T113902Z_v14.partial/output/data/Part_0901.bi4")
SETTLED_PART_SHA256 = "023c2ae47281351f7c601a60709799cf8155745193d7e2aa5ee862256ad9b1f4"
SETTLED_HEAD_PATH = SETTLED_PART_PATH.parent / "Part_Head.ibi4"
GPU_DEVICE_PATHS = ("/dev/nvidia0", "/dev/nvidiactl", "/dev/nvidia-uvm", "/dev/nvidia-uvm-tools")
RUN_ID_RE = re.compile(r"u3_c1m_(?:cpu|gpu)_[A-Za-z0-9_]+_v[0-9]+")
MAX_JSON_BYTES = 4 * 1024 * 1024
MAX_INPUT_BYTES = 128 * 1024 * 1024
COMMON_FLAGS = (
    "-ompthreads:1",
    "-stable:1",
    "-vres:0",
    "-cellmode:full",
    "-cfl:0.1",
    "-shifting:none",
    "-viscoart:0.3",
    "-sv:binx,info",
    "-svres:1",
    "-svtimers:0",
    "-svdomainvtk:0",
    "-saveposdouble:1",
    "-nortimes:1",
    "-createdirs:1",
    "-csvsep:0",
)


class Stage4ExperimentError(RuntimeError):
    """Unsafe, drifted, colliding or unsuccessful experiment."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _expected_environment(backend: str) -> dict[str, str]:
    return {
        "HOME": "/nonexistent",
        "PATH": "/usr/bin",
        "LC_ALL": "C",
        "LANG": "C",
        "TZ": "UTC0",
        "TMPDIR": "/tmp",
        "OMP_NUM_THREADS": "1",
        "OMP_DYNAMIC": "FALSE",
        "CUDA_VISIBLE_DEVICES": "0" if backend == "GPU" else "",
    }


def _identity(path: Path, *, maximum: int = MAX_INPUT_BYTES) -> dict[str, Any]:
    value = legacy.identity(path, maximum=maximum)
    return {key: value[key] for key in ("path", "sha256", "size_bytes", "mode", "inode", "nlink", "uid", "gid")}


def _verify(spec: dict[str, Any], *, full: bool = True, maximum: int = MAX_INPUT_BYTES) -> dict[str, Any]:
    return legacy.verify_spec(spec, full=full, maximum=maximum)


def _float_arg(argv: list[str], prefix: str) -> float:
    hits = [item for item in argv if item.startswith(prefix)]
    if len(hits) != 1:
        raise Stage4ExperimentError(f"solver argv cardinality differs for {prefix}")
    try:
        value = float(hits[0].split(":", 1)[1])
    except ValueError as exc:
        raise Stage4ExperimentError(f"invalid solver numeric flag: {hits[0]}") from exc
    if not math.isfinite(value):
        raise Stage4ExperimentError(f"non-finite solver numeric flag: {hits[0]}")
    return value


def expected_run_paths(run_id: str) -> dict[str, str]:
    attempt = RUNS_ROOT / f"{run_id}.partial"
    return {
        "attempt_root": str(attempt),
        "output_root": str(attempt / "output"),
        "stage_root": str(attempt / "staging"),
        "guest_output_root": str(attempt / "staging/guest-output"),
        "start_receipt": str(AUDIT_ROOT / f"{run_id}.start.json"),
        "final_receipt": str(AUDIT_ROOT / f"{run_id}.final.json"),
        "failure_receipt": str(AUDIT_ROOT / f"{run_id}.failure.json"),
        "stdout_log": str(AUDIT_ROOT / f"{run_id}.stdout.log"),
        "stderr_log": str(AUDIT_ROOT / f"{run_id}.stderr.log"),
        "resource_log": str(AUDIT_ROOT / f"{run_id}.resources.jsonl"),
    }


def semantic_validate(policy: dict[str, Any], policy_path: Path) -> None:
    if policy_path.parent != CONFIG_ROOT or policy_path.resolve(strict=True) != policy_path:
        raise Stage4ExperimentError("policy path leaves the exact target-host config root")
    run_id = policy["run_id"]
    if RUN_ID_RE.fullmatch(run_id) is None:
        raise Stage4ExperimentError("run_id differs from the frozen namespace")
    if policy["run"] != expected_run_paths(run_id):
        raise Stage4ExperimentError("run paths differ from the deterministic identity")
    if policy["parents"]["plan"] != {"path": str(PLAN_PATH), "sha256": PLAN_SHA256}:
        raise Stage4ExperimentError("plan parent differs")
    if policy["parents"]["numerical_contract"] != {
        "path": str(NUMERICAL_CONTRACT_PATH),
        "sha256": NUMERICAL_CONTRACT_SHA256,
    }:
        raise Stage4ExperimentError("numerical contract parent differs")
    if policy["parents"]["settled_qc"] != {"path": str(SETTLED_QC_PATH), "sha256": SETTLED_QC_SHA256}:
        raise Stage4ExperimentError("settled QC parent differs")
    expected_parent_paths = {"runner": SCRIPT_PATH, "schema": SCHEMA_PATH, "tests": TEST_PATH}
    for name, path in expected_parent_paths.items():
        if policy["parents"][name]["path"] != str(path):
            raise Stage4ExperimentError(f"execution parent path differs: {name}")

    backend = policy["backend"]["kind"]
    if (backend == "GPU") is not isinstance(policy["gpu"], dict):
        raise Stage4ExperimentError("GPU identity presence differs from backend")
    if backend == "GPU" and policy["backend"]["index"] != 0:
        raise Stage4ExperimentError("GPU index differs")
    if backend == "CPU" and policy["backend"]["index"] is not None:
        raise Stage4ExperimentError("CPU backend must not carry a GPU index")
    candidate = policy["inputs"]["candidate"]
    expected_candidate = (
        (CPU_CANDIDATE_PATH, CPU_CANDIDATE_SHA256, "0755")
        if backend == "CPU"
        else (GPU_CANDIDATE_PATH, GPU_CANDIDATE_SHA256, "0400")
    )
    if (candidate["path"], candidate["sha256"], candidate["mode"]) != tuple(map(str, expected_candidate)):
        raise Stage4ExperimentError("candidate path/hash/mode differs for backend")
    if policy["inputs"]["case_bi4"]["sha256"] != BASE_CASE_BI4_SHA256:
        raise Stage4ExperimentError("case BI4 identity differs")

    experiment = policy["experiment"]
    restart = policy["restart"]
    if experiment["family"] == "BACKEND_PARITY_SHORT":
        if experiment["motion_kind"] != "ZERO" or restart["enabled"]:
            raise Stage4ExperimentError("backend parity must be a fresh zero-motion run")
        if policy["inputs"]["case_xml"]["sha256"] != BASE_CASE_XML_SHA256:
            raise Stage4ExperimentError("backend parity XML identity differs")
    else:
        if not restart["enabled"] or experiment["motion_kind"] == "FRESH":
            raise Stage4ExperimentError("dynamic replay must restart from the settled checkpoint")
        part = policy["inputs"]["restart_part"]
        head = policy["inputs"]["restart_head"]
        if (
            part is None
            or head is None
            or part["path"] != str(SETTLED_PART_PATH)
            or part["sha256"] != SETTLED_PART_SHA256
            or head["path"] != str(SETTLED_HEAD_PATH)
        ):
            raise Stage4ExperimentError("settled restart identity differs")

    if policy["solver"]["environment"] != _expected_environment(backend):
        raise Stage4ExperimentError("sanitized environment differs")
    argv = policy["solver"]["argv"]
    if argv[:3] != ["/runtime/candidate", "/case/C1M_case", "/output"]:
        raise Stage4ExperimentError("solver argv prefix differs")
    wanted_backend = "-cpu" if backend == "CPU" else "-gpu:0"
    other_backend = "-gpu:0" if backend == "CPU" else "-cpu"
    if argv.count(wanted_backend) != 1 or other_backend in argv:
        raise Stage4ExperimentError("backend flag differs")
    if any(argv.count(flag) != 1 for flag in COMMON_FLAGS):
        raise Stage4ExperimentError("common numerical/output flag cardinality differs")
    viscosity_flags = [item for item in argv if item.startswith(("-viscoart:", "-viscolam:", "-viscolamsps:"))]
    if viscosity_flags != ["-viscoart:0.3"]:
        raise Stage4ExperimentError("viscosity formulation differs")
    forbidden = ("http://", "https://", "/home/", "..", "-gpu:1", "-shifting:", "-cfl:")
    for token in argv:
        low = token.lower()
        if any(fragment in low for fragment in forbidden) and token not in ("-shifting:none", "-cfl:0.1"):
            raise Stage4ExperimentError(f"forbidden solver fragment: {token}")
    tmax = _float_arg(argv, "-tmax:")
    tout = _float_arg(argv, "-tout:")
    expected = policy["expected_output"]
    if not math.isclose(tmax, expected["end_time_s"], rel_tol=0.0, abs_tol=1e-12):
        raise Stage4ExperimentError("tmax differs from expected output")
    if not math.isclose(tout, expected["output_period_s"], rel_tol=0.0, abs_tol=1e-12):
        raise Stage4ExperimentError("tout differs from expected output")
    if expected["part_count"] != expected["part_last"] - expected["part_first"] + 1:
        raise Stage4ExperimentError("output cardinality differs")
    part_flags = [item for item in argv if item.startswith("-partbegin:")]
    if restart["enabled"]:
        wanted = f"-partbegin:{restart['part_index']}:{restart['part_first']}"
        if part_flags != [wanted] or argv.index(wanted) + 1 >= len(argv) or argv[argv.index(wanted) + 1] != "/restart":
            raise Stage4ExperimentError("restart argv differs")
        if restart != {"enabled": True, "part_index": 901, "part_first": 901, "guest_dir": "/restart"}:
            raise Stage4ExperimentError("restart checkpoint contract differs")
    elif part_flags or policy["inputs"]["restart_part"] is not None or policy["inputs"]["restart_head"] is not None:
        raise Stage4ExperimentError("fresh run carries restart state")
    limits = policy["limits"]
    if (
        not 10 <= limits["monitor_interval_seconds"] <= 30
        or limits["minimum_mem_available_bytes"] < 4294967296
        or limits["wall_timeout_seconds"] > 1800
        or limits["maximum_output_bytes"] != expected["maximum_output_bytes"]
    ):
        raise Stage4ExperimentError("resource limits differ")
    sandbox = policy["sandbox"]
    if (
        sandbox["network"]
        or sandbox["project_workspace_write_bind_count"] != 0
        or sandbox["host_write_bind_count"] != 1
        or sandbox["sudo_used"]
        or sandbox["apparmor_profile_used"]
    ):
        raise Stage4ExperimentError("sandbox boundary differs")


def load_policy(path: Path) -> dict[str, Any]:
    schema = legacy.read_json(SCHEMA_PATH)
    policy = legacy.read_json(path)
    Draft202012Validator.check_schema(schema)
    errors = sorted(Draft202012Validator(schema).iter_errors(policy), key=lambda item: list(item.absolute_path))
    if errors:
        first = errors[0]
        raise Stage4ExperimentError(f"policy schema failure at {list(first.absolute_path)}: {first.message}")
    semantic_validate(policy, path)
    return policy


def fresh_paths(policy: dict[str, Any]) -> list[Path]:
    run = policy["run"]
    return [Path(run[key]) for key in ("attempt_root", "start_receipt", "final_receipt", "failure_receipt", "stdout_log", "stderr_log", "resource_log")]


def solver_processes() -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    own_pid = os.getpid()
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == own_pid:
            continue
        try:
            raw = (entry / "cmdline").read_bytes()
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        text = raw.replace(b"\x00", b" ").decode("utf-8", "replace")
        if "DualSPHysics5.4" in text:
            rows.append({"pid": int(entry.name), "cmdline_sha256": hashlib.sha256(raw).hexdigest()})
    return sorted(rows, key=lambda item: item["pid"])


def preflight(policy_path: Path, *, require_fresh: bool) -> dict[str, Any]:
    policy = load_policy(policy_path)
    verified_parents = {
        name: _verify(spec, full=False, maximum=MAX_JSON_BYTES) for name, spec in policy["parents"].items()
    }
    verified_inputs = {
        name: None if spec is None else _verify(spec, full=True)
        for name, spec in policy["inputs"].items()
    }
    verified_tools = {
        name: _verify(spec, full=False, maximum=16 * 1024 * 1024) for name, spec in policy["tools"].items()
    }
    available = legacy.mem_available_bytes()
    if available < policy["limits"]["minimum_mem_available_bytes"]:
        raise Stage4ExperimentError(f"MemAvailable below frozen minimum: {available}")
    competing_solvers = solver_processes()
    if competing_solvers:
        raise Stage4ExperimentError(f"competing solver processes exist: {competing_solvers}")
    gpu: dict[str, Any] | None = None
    devices: list[dict[str, Any]] = []
    compute: list[dict[str, Any]] = []
    if policy["backend"]["kind"] == "GPU":
        compute = legacy.compute_processes(policy["tools"]["nvidia_smi"]["path"])
        if compute:
            raise Stage4ExperimentError(f"competing GPU compute processes exist: {compute}")
        gpu = legacy.gpu_sample(policy)
        for path_text in GPU_DEVICE_PATHS:
            metadata = os.stat(path_text)
            if not stat.S_ISCHR(metadata.st_mode):
                raise Stage4ExperimentError(f"GPU device is not a character device: {path_text}")
            devices.append({"path": path_text, "major": os.major(metadata.st_rdev), "minor": os.minor(metadata.st_rdev)})
    collisions = [str(path) for path in fresh_paths(policy) if os.path.lexists(path)] if require_fresh else []
    if collisions:
        raise Stage4ExperimentError(f"one-shot identity collision: {collisions}")
    return {
        "captured_at_utc": utc_now(),
        "uid": os.getuid(),
        "gid": os.getgid(),
        "policy": _identity(policy_path, maximum=MAX_JSON_BYTES),
        "verified_parents": verified_parents,
        "verified_inputs": verified_inputs,
        "verified_tools": verified_tools,
        "mem_available_bytes": available,
        "memory_psi": Path("/proc/pressure/memory").read_text(encoding="ascii").strip(),
        "competing_solvers": competing_solvers,
        "gpu": gpu,
        "gpu_devices": devices,
        "gpu_compute_processes": compute,
        "fresh": require_fresh,
    }


def _mkdir(path: Path, mode: int) -> None:
    os.mkdir(path, mode)
    os.chmod(path, mode, follow_symlinks=False)


def _copy(spec: dict[str, Any], destination: Path, mode: int) -> dict[str, Any]:
    return legacy.copy_exclusive(spec, destination, mode, os.getuid(), os.getgid())


def prepare_stage(policy: dict[str, Any]) -> dict[str, Any]:
    run = policy["run"]
    attempt = Path(run["attempt_root"])
    stage = Path(run["stage_root"])
    _mkdir(attempt, 0o700)
    _mkdir(Path(run["output_root"]), 0o700)
    _mkdir(stage, 0o700)
    for name in ("runtime", "case", "guest-output"):
        _mkdir(stage / name, 0o700)
    staged = {
        "candidate": _copy(policy["inputs"]["candidate"], stage / "runtime/candidate", 0o550),
        "dsph_config": _copy(policy["inputs"]["dsph_config"], stage / "runtime/DsphConfig.xml", 0o440),
        "case_bi4": _copy(policy["inputs"]["case_bi4"], stage / "case/C1M_case.bi4", 0o440),
        "case_xml": _copy(policy["inputs"]["case_xml"], stage / "case/C1M_case.xml", 0o440),
    }
    if policy["restart"]["enabled"]:
        _mkdir(stage / "restart", 0o700)
        staged["restart_part"] = _copy(
            policy["inputs"]["restart_part"],
            stage / f"restart/Part_{policy['restart']['part_index']:04d}.bi4",
            0o440,
        )
        staged["restart_head"] = _copy(policy["inputs"]["restart_head"], stage / "restart/Part_Head.ibi4", 0o440)
    return staged


def bwrap_argv(policy: dict[str, Any]) -> list[str]:
    stage = Path(policy["run"]["stage_root"])
    argv = [
        policy["tools"]["bwrap"]["path"],
        "--die-with-parent",
        "--new-session",
        "--unshare-user",
        "--unshare-pid",
        "--unshare-net",
        "--unshare-ipc",
        "--unshare-uts",
        "--unshare-cgroup-try",
        "--uid",
        "1000",
        "--gid",
        "1000",
        "--cap-drop",
        "ALL",
        "--hostname",
        "r8-liquid-stage4",
        "--clearenv",
    ]
    for key, value in policy["solver"]["environment"].items():
        argv.extend(("--setenv", key, value))
    argv.extend(
        (
            "--ro-bind",
            "/usr",
            "/usr",
            "--symlink",
            "usr/lib",
            "/lib",
            "--symlink",
            "usr/lib64",
            "/lib64",
            "--dir",
            "/etc",
            "--ro-bind",
            "/etc/ld.so.cache",
            "/etc/ld.so.cache",
            "--proc",
            "/proc",
            "--dev",
            "/dev",
        )
    )
    if policy["backend"]["kind"] == "GPU":
        for device in GPU_DEVICE_PATHS:
            argv.extend(("--dev-bind", device, device))
        argv.extend(("--ro-bind", "/sys", "/sys"))
    argv.extend(
        (
            "--tmpfs",
            "/tmp",
            "--ro-bind",
            str(stage / "runtime"),
            "/runtime",
            "--ro-bind",
            str(stage / "case"),
            "/case",
        )
    )
    if policy["restart"]["enabled"]:
        argv.extend(("--ro-bind", str(stage / "restart"), "/restart"))
    argv.extend(("--bind", str(stage / "guest-output"), "/output", "--chdir", "/runtime", "--", *policy["solver"]["argv"]))
    writable = [argv[index + 1 : index + 3] for index, item in enumerate(argv) if item == "--bind"]
    if writable != [[str(stage / "guest-output"), "/output"]]:
        raise Stage4ExperimentError("writable bind proof differs")
    if "--unshare-net" not in argv or "--share-net" in argv:
        raise Stage4ExperimentError("network namespace proof differs")
    return argv


def _swap_free_bytes() -> int:
    for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
        if line.startswith("SwapFree:"):
            return int(line.split()[1]) * 1024
    raise Stage4ExperimentError("SwapFree is absent")


def resource_sample(policy: dict[str, Any], pgid: int, started: float) -> dict[str, Any]:
    count, total = legacy.output_usage(Path(policy["run"]["guest_output_root"]))
    value: dict[str, Any] = {
        "captured_at_utc": utc_now(),
        "elapsed_seconds": round(time.monotonic() - started, 6),
        "mem_available_bytes": legacy.mem_available_bytes(),
        "swap_free_bytes": _swap_free_bytes(),
        "memory_psi": Path("/proc/pressure/memory").read_text(encoding="ascii").strip(),
        "process_group_rss_bytes": legacy.process_group_rss_bytes(pgid),
        "output_file_count": count,
        "output_bytes": total,
        "gpu": None,
    }
    if policy["backend"]["kind"] == "GPU":
        value["gpu"] = legacy.gpu_sample(policy)
    return value


def terminate_group(process: subprocess.Popen[bytes], kill_after: float) -> None:
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
            return
        process.wait(timeout=5)


def _open_exclusive(path: Path, mode: int = 0o640) -> int:
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, mode)
    os.fchmod(descriptor, mode)
    return descriptor


def _write_line(descriptor: int, value: Any) -> None:
    data = legacy.canonical_bytes(value)
    view = memoryview(data)
    while view:
        written = os.write(descriptor, view)
        if written <= 0:
            raise Stage4ExperimentError("short resource-log write")
        view = view[written:]


def _lock_file(policy: dict[str, Any]) -> tuple[int, Path]:
    name = "gpu0.lock" if policy["backend"]["kind"] == "GPU" else "cpu-stage4.lock"
    path = LOCK_ROOT / name
    descriptor = os.open(path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
    metadata = os.fstat(descriptor)
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or metadata.st_uid != os.getuid():
        os.close(descriptor)
        raise Stage4ExperimentError(f"unsafe lock identity: {path}")
    os.fchmod(descriptor, 0o600)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        os.close(descriptor)
        raise Stage4ExperimentError(f"backend lock is already held: {path}") from exc
    return descriptor, path


def execute(policy_path: Path) -> dict[str, Any]:
    if os.getuid() != 1000 or os.geteuid() != 1000 or os.getgid() != 1000:
        raise Stage4ExperimentError("runner requires direct uid/gid 1000 without sudo")
    policy = load_policy(policy_path)
    pre = preflight(policy_path, require_fresh=True)
    lock_fd, lock_path = _lock_file(policy)
    run = policy["run"]
    process: subprocess.Popen[bytes] | None = None
    started_wall = time.time()
    started_mono = time.monotonic()
    returncode: int | None = None
    termination_reason = "NOT_STARTED"
    argv: list[str] = []
    staged: dict[str, Any] | None = None
    try:
        start = {
            "schema_version": "r8-liquid-u3-stage4-experiment-receipt-v1",
            "document_type": "SMPCC_R8_LIQUID_U3_STAGE4_EXPERIMENT_START_V1",
            "status": "STAGE4_EXPERIMENT_ONE_SHOT_STARTED",
            "run_id": policy["run_id"],
            "backend": policy["backend"]["kind"],
            "started_at_utc": utc_now(),
            "preflight": pre,
            "lock_path": str(lock_path),
            "candidate_source_executed": False,
            "network_used": False,
            "project_workspace_writable_in_guest": False,
            "sudo_used": False,
        }
        start_identity = legacy.write_json_exclusive(Path(run["start_receipt"]), start)
        staged = prepare_stage(policy)
        stdout_fd = _open_exclusive(Path(run["stdout_log"]))
        stderr_fd = _open_exclusive(Path(run["stderr_log"]))
        resource_fd = _open_exclusive(Path(run["resource_log"]))
        try:
            argv = bwrap_argv(policy)
            process = subprocess.Popen(
                argv,
                stdin=subprocess.DEVNULL,
                stdout=stdout_fd,
                stderr=stderr_fd,
                close_fds=True,
                start_new_session=True,
                env={"PATH": "/usr/bin", "LC_ALL": "C", "LANG": "C"},
            )
            termination_reason = "PROCESS_EXIT"
            next_sample = 0.0
            while process.poll() is None:
                elapsed = time.monotonic() - started_mono
                if elapsed >= policy["limits"]["wall_timeout_seconds"]:
                    termination_reason = "WALL_TIMEOUT"
                    terminate_group(process, policy["limits"]["kill_after_seconds"])
                    break
                if elapsed >= next_sample:
                    sample = resource_sample(policy, process.pid, started_mono)
                    _write_line(resource_fd, sample)
                    if sample["mem_available_bytes"] < policy["limits"]["minimum_mem_available_bytes"]:
                        termination_reason = "MEMORY_FLOOR"
                        terminate_group(process, policy["limits"]["kill_after_seconds"])
                        break
                    if sample["output_bytes"] > policy["limits"]["maximum_output_bytes"]:
                        termination_reason = "OUTPUT_LIMIT"
                        terminate_group(process, policy["limits"]["kill_after_seconds"])
                        break
                    next_sample = elapsed + policy["limits"]["monitor_interval_seconds"]
                time.sleep(0.2)
            returncode = process.wait(timeout=10)
            _write_line(resource_fd, resource_sample(policy, process.pid, started_mono))
            os.fsync(resource_fd)
        finally:
            if process is not None and process.poll() is None:
                terminate_group(process, policy["limits"]["kill_after_seconds"])
            for descriptor in (stdout_fd, stderr_fd, resource_fd):
                os.close(descriptor)
        if returncode != 0 or termination_reason != "PROCESS_EXIT":
            raise Stage4ExperimentError(f"solver failed rc={returncode} termination={termination_reason}")
        output_inventory = legacy.export_output(
            Path(run["guest_output_root"]),
            Path(run["output_root"]),
            policy["expected_output"],
        )
        candidate_after = _verify(policy["inputs"]["candidate"], full=True)
        compute_after: list[dict[str, Any]] = []
        gpu_after: dict[str, Any] | None = None
        if policy["backend"]["kind"] == "GPU":
            compute_after = legacy.compute_processes(policy["tools"]["nvidia_smi"]["path"])
            if compute_after:
                raise Stage4ExperimentError(f"GPU compute process remains: {compute_after}")
            gpu_after = legacy.gpu_sample(policy)
        final = {
            "schema_version": "r8-liquid-u3-stage4-experiment-receipt-v1",
            "document_type": "SMPCC_R8_LIQUID_U3_STAGE4_EXPERIMENT_FINAL_V1",
            "status": "PASS_U3_STAGE4_EXPERIMENT_RAW_OUTPUT",
            "run_id": policy["run_id"],
            "purpose": policy["purpose"],
            "backend": policy["backend"]["kind"],
            "started_at_utc": datetime.fromtimestamp(started_wall, tz=timezone.utc).isoformat(),
            "completed_at_utc": utc_now(),
            "elapsed_seconds": round(time.monotonic() - started_mono, 6),
            "returncode": returncode,
            "termination_reason": termination_reason,
            "policy": _identity(policy_path, maximum=MAX_JSON_BYTES),
            "start_receipt": start_identity,
            "sandbox_argv": argv,
            "solver_argv": policy["solver"]["argv"],
            "staged_inputs": staged,
            "candidate_source_after": candidate_after,
            "candidate_source_executed": False,
            "staged_candidate_executed": True,
            "network_used": False,
            "project_workspace_writable_in_guest": False,
            "apparmor_profile_used": False,
            "sudo_used": False,
            "output_inventory": output_inventory,
            "stdout": _identity(Path(run["stdout_log"]), maximum=MAX_JSON_BYTES),
            "stderr": _identity(Path(run["stderr_log"]), maximum=MAX_JSON_BYTES),
            "resource_log": _identity(Path(run["resource_log"]), maximum=MAX_JSON_BYTES),
            "gpu_after": gpu_after,
            "compute_processes_after": compute_after,
            "development_only": True,
            "formal": False,
            "physical_primary_eligible": False,
            "qc_pending": True,
        }
        receipt = legacy.write_json_exclusive(Path(run["final_receipt"]), final)
        final["final_receipt"] = receipt
        return final
    except Exception as exc:
        failure = {
            "schema_version": "r8-liquid-u3-stage4-experiment-receipt-v1",
            "document_type": "SMPCC_R8_LIQUID_U3_STAGE4_EXPERIMENT_FAILURE_V1",
            "status": "FAIL_U3_STAGE4_EXPERIMENT",
            "run_id": policy.get("run_id"),
            "backend": policy.get("backend", {}).get("kind"),
            "failed_at_utc": utc_now(),
            "elapsed_seconds": round(time.monotonic() - started_mono, 6),
            "error_type": type(exc).__name__,
            "error": str(exc),
            "returncode": returncode,
            "termination_reason": termination_reason,
            "attempt_root_preserved": os.path.lexists(run["attempt_root"]),
            "candidate_source_executed": False,
            "network_used": False,
            "project_workspace_writable_in_guest": False,
            "sudo_used": False,
        }
        if not os.path.lexists(run["failure_receipt"]):
            legacy.write_json_exclusive(Path(run["failure_receipt"]), failure)
        raise
    finally:
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_UN)
        finally:
            os.close(lock_fd)


def self_check(policy_path: Path) -> dict[str, Any]:
    policy = load_policy(policy_path)
    verified = preflight(policy_path, require_fresh=False)
    argv = bwrap_argv(policy)
    return {
        "status": "PASS_U3_STAGE4_EXPERIMENT_RUNNER_V1_SELF_CHECK",
        "policy": _identity(policy_path, maximum=MAX_JSON_BYTES),
        "verified": verified,
        "sandbox_argv": argv,
        "network_used": False,
        "candidate_executed": False,
        "project_workspace_writable_in_guest": False,
        "sudo_used": False,
    }


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("policy", type=Path)
    result.add_argument("command", choices=("self-check", "preflight", "run"))
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        if args.command == "self-check":
            result = self_check(args.policy)
        elif args.command == "preflight":
            result = {"status": "PASS_U3_STAGE4_EXPERIMENT_PREFLIGHT", "evidence": preflight(args.policy, require_fresh=True)}
        else:
            result = execute(args.policy)
    except (Stage4ExperimentError, legacy.Stage4RunError, OSError, ValueError, subprocess.SubprocessError) as exc:
        print(
            json.dumps(
                {"status": "FAIL_U3_STAGE4_EXPERIMENT_RUNNER_V1", "error_type": type(exc).__name__, "error": str(exc)},
                ensure_ascii=False,
                sort_keys=True,
            ),
            file=sys.stderr,
        )
        return 1
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
