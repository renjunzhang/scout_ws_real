#!/usr/bin/env python3
"""One-shot, root-supervised, network-isolated RTX 5080 Stage-4 runner.

The runner consumes one closed exact policy, copies immutable inputs into a
root-only /var/tmp transport, executes one DualSPHysics command as an
unprivileged/capability-free bwrap guest, and exports only regular nlink=1
outputs into a fresh scout_liquid_lab run root.  It never writes the project,
the source candidate, or an input tree.
"""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import math
import os
import re
import shutil
import signal
import stat
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_gpu_stage4_run_policy_v1.json"
EXPECTED_ENVIRONMENT = {
    "HOME": "/nonexistent",
    "PATH": "/usr/bin",
    "LC_ALL": "C",
    "LANG": "C",
    "TZ": "UTC0",
    "TMPDIR": "/tmp",
    "OMP_NUM_THREADS": "1",
    "OMP_DYNAMIC": "FALSE",
    "CUDA_VISIBLE_DEVICES": "0",
}
EXPECTED_DEVICES = ["/dev/nvidia0", "/dev/nvidiactl", "/dev/nvidia-uvm", "/dev/nvidia-uvm-tools"]
ROOT_OUTPUT_NAMES = {"CfgInit_Domain.vtk", "CfgInit_MapCells.vtk", "Run.csv", "Run.out", "RunPARTs.csv", "data"}
STATIC_DATA_NAMES = {"Part_Head.ibi4", "PartInfo.ibi4", "PartMotionRef.ibi4", "PartOut_000.obi4"}
PART_RE = re.compile(r"Part_([0-9]{4})\.bi4")
MAX_POLICY_BYTES = 2 * 1024 * 1024
MAX_SINGLE_FILE_BYTES = 64 * 1024 * 1024


class Stage4RunError(RuntimeError):
    """Fail-closed execution or identity error."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def canonical_bytes(value: Any) -> bytes:
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode("utf-8")


def sha256_file(path: Path, *, maximum: int = MAX_SINGLE_FILE_BYTES) -> str:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    digest = hashlib.sha256()
    total = 0
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise Stage4RunError(f"unsafe non-regular or linked file: {path}")
        if metadata.st_size < 0 or metadata.st_size > maximum:
            raise Stage4RunError(f"file exceeds bound: {path}")
        while True:
            block = os.read(descriptor, 1 << 20)
            if not block:
                break
            digest.update(block)
            total += len(block)
    finally:
        os.close(descriptor)
    if total != metadata.st_size:
        raise Stage4RunError(f"short read: {path}")
    return digest.hexdigest()


def identity(path: Path, *, maximum: int = MAX_SINGLE_FILE_BYTES) -> dict[str, Any]:
    metadata = os.lstat(path)
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
        raise Stage4RunError(f"unsafe file identity: {path}")
    return {
        "path": str(path),
        "inode": metadata.st_ino,
        "mode": f"{stat.S_IMODE(metadata.st_mode):04o}",
        "size_bytes": metadata.st_size,
        "nlink": metadata.st_nlink,
        "uid": metadata.st_uid,
        "gid": metadata.st_gid,
        "sha256": sha256_file(path, maximum=maximum),
    }


def verify_spec(spec: dict[str, Any], *, full: bool, maximum: int = MAX_SINGLE_FILE_BYTES) -> dict[str, Any]:
    observed = identity(Path(spec["path"]), maximum=maximum)
    required = ("sha256", "size_bytes", "mode", "inode", "nlink") if full else ("sha256",)
    drift = {name: (spec.get(name), observed.get(name)) for name in required if spec.get(name) != observed.get(name)}
    if drift:
        raise Stage4RunError(f"identity drift for {spec['path']}: {drift}")
    return observed


def read_json(path: Path) -> dict[str, Any]:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or not 0 < metadata.st_size <= MAX_POLICY_BYTES:
            raise Stage4RunError(f"unsafe JSON file: {path}")
        raw = b""
        while len(raw) < metadata.st_size:
            block = os.read(descriptor, metadata.st_size - len(raw))
            if not block:
                break
            raw += block
    finally:
        os.close(descriptor)
    try:
        value = json.loads(raw.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise Stage4RunError(f"invalid JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise Stage4RunError(f"JSON root is not object: {path}")
    return value


def mem_available_bytes() -> int:
    for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) * 1024
    raise Stage4RunError("MemAvailable is absent")


def compute_processes(nvidia_smi: str) -> list[dict[str, Any]]:
    completed = subprocess.run(
        [nvidia_smi, "--query-compute-apps=pid,process_name,used_gpu_memory", "--format=csv,noheader,nounits"],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=20,
        check=False,
        env={"PATH": "/usr/bin", "LC_ALL": "C", "LANG": "C"},
    )
    if completed.returncode != 0:
        raise Stage4RunError(f"nvidia-smi compute query failed rc={completed.returncode}")
    rows: list[dict[str, Any]] = []
    for line in completed.stdout.decode("utf-8", "replace").splitlines():
        if not line.strip():
            continue
        fields = [item.strip() for item in line.split(",", 2)]
        if len(fields) != 3 or not fields[0].isdigit():
            raise Stage4RunError(f"malformed compute process row: {line!r}")
        rows.append({"pid": int(fields[0]), "process_name": fields[1], "used_gpu_memory_mib": fields[2]})
    return rows


def gpu_sample(policy: dict[str, Any]) -> dict[str, Any]:
    nvidia_smi = policy["tools"]["nvidia_smi"]["path"]
    query = "uuid,pci.bus_id,name,driver_version,memory.used,utilization.gpu,power.draw,temperature.gpu"
    completed = subprocess.run(
        [nvidia_smi, f"--query-gpu={query}", "--format=csv,noheader,nounits", "-i", "0"],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=20,
        check=False,
        env={"PATH": "/usr/bin", "LC_ALL": "C", "LANG": "C"},
    )
    if completed.returncode != 0:
        raise Stage4RunError(f"nvidia-smi GPU query failed rc={completed.returncode}")
    fields = [item.strip() for item in completed.stdout.decode("utf-8", "replace").strip().split(",")]
    if len(fields) != 8:
        raise Stage4RunError(f"malformed GPU query: {fields!r}")
    expected = policy["gpu"]
    if fields[0] != expected["uuid"] or fields[1] != expected["pci_bus_id"] or fields[2] != expected["name"] or fields[3] != expected["driver_version"]:
        raise Stage4RunError(f"GPU identity drift: {fields[:4]!r}")
    return {
        "uuid": fields[0],
        "pci_bus_id": fields[1],
        "name": fields[2],
        "driver_version": fields[3],
        "memory_used_mib": float(fields[4]),
        "utilization_percent": float(fields[5]),
        "power_draw_w": float(fields[6]),
        "temperature_c": float(fields[7]),
    }


def output_usage(root: Path) -> tuple[int, int]:
    if not root.exists():
        return 0, 0
    count = total = 0
    for base, dirs, files in os.walk(root, followlinks=False):
        for name in dirs:
            path = Path(base) / name
            if stat.S_ISLNK(os.lstat(path).st_mode):
                raise Stage4RunError(f"symlink directory appeared: {path}")
        for name in files:
            path = Path(base) / name
            metadata = os.lstat(path)
            if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
                raise Stage4RunError(f"unsafe output appeared: {path}")
            count += 1
            total += metadata.st_size
    return count, total


def process_group_rss_bytes(pgid: int) -> int:
    pages = 0
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / "stat").read_text(encoding="ascii").split()
            if int(fields[4]) == pgid:
                pages += int(fields[23])
        except (FileNotFoundError, PermissionError, ValueError, IndexError):
            continue
    return pages * os.sysconf("SC_PAGE_SIZE")


def resource_sample(policy: dict[str, Any], pgid: int, started: float, stage_output: Path) -> dict[str, Any]:
    count, total = output_usage(stage_output)
    return {
        "captured_at_utc": utc_now(),
        "elapsed_seconds": round(time.monotonic() - started, 6),
        "mem_available_bytes": mem_available_bytes(),
        "swap_free_bytes": next(
            int(line.split()[1]) * 1024 for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines() if line.startswith("SwapFree:")
        ),
        "memory_psi": Path("/proc/pressure/memory").read_text(encoding="ascii").strip(),
        "process_group_rss_bytes": process_group_rss_bytes(pgid),
        "output_file_count": count,
        "output_bytes": total,
        "gpu": gpu_sample(policy),
    }


def _float_arg(argv: list[str], prefix: str) -> float:
    hits = [item for item in argv if item.startswith(prefix)]
    if len(hits) != 1:
        raise Stage4RunError(f"solver argv cardinality differs for {prefix}")
    try:
        value = float(hits[0].split(":", 1)[1])
    except ValueError as exc:
        raise Stage4RunError(f"invalid solver numeric flag: {hits[0]}") from exc
    if not math.isfinite(value):
        raise Stage4RunError(f"non-finite solver numeric flag: {hits[0]}")
    return value


def semantic_validate(policy: dict[str, Any], policy_path: Path) -> None:
    run_id = policy["run_id"]
    run = policy["run"]
    attempt = Path(run["attempt_root"])
    if attempt != Path("/home/zrj/scout_liquid_lab/gpu_stage4_runs") / f"{run_id}.partial":
        raise Stage4RunError("attempt root is not the exact dedicated child")
    if Path(run["output_root"]) != attempt / "output":
        raise Stage4RunError("output root is not exact attempt child")
    audit_root = Path("/home/zrj/scout_liquid_lab/audits")
    for key in ("start_receipt", "final_receipt", "failure_receipt", "stdout_log", "stderr_log", "resource_log"):
        path = Path(run[key])
        if path.parent != audit_root or run_id not in path.name:
            raise Stage4RunError(f"audit path differs: {key}")
    expected_stage = Path("/var/tmp") / f"r8-liquid-u3-stage4-{run_id.lower().replace('_', '-')}"
    if Path(policy["sandbox"]["stage_root"]) != expected_stage:
        raise Stage4RunError("stage root is not deterministic")
    if policy["solver"]["environment"] != EXPECTED_ENVIRONMENT:
        raise Stage4RunError("solver environment differs")
    if policy["gpu"]["device_paths"] != EXPECTED_DEVICES or policy["gpu"]["index"] != 0:
        raise Stage4RunError("GPU device contract differs")
    argv = policy["solver"]["argv"]
    expected_prefix = ["/runtime/DualSPHysics5.4_linux64", "/case/C1M_zero", "/output"]
    if argv[:3] != expected_prefix:
        raise Stage4RunError("solver argv prefix differs")
    required = ("-gpu:0", "-ompthreads:1", "-stable:1", "-vres:0", "-cellmode:full", "-sv:binx,info", "-saveposdouble:1", "-createdirs:1")
    if any(argv.count(item) != 1 for item in required):
        raise Stage4RunError("required solver flag cardinality differs")
    forbidden_fragments = ("-cpu", "-gpu:1", "http://", "https://", "/home/", "..")
    if any(fragment in item.lower() for item in argv for fragment in forbidden_fragments):
        raise Stage4RunError("solver argv contains forbidden path/backend/network fragment")
    delta = policy["single_delta"]
    if delta["parameter"] == "Shifting":
        expected_shift = {"None": "-shifting:none", "NoBound": "-shifting:nobound"}.get(delta["candidate"])
        if expected_shift is None or [item for item in argv if item.startswith("-shifting:")] != [expected_shift]:
            raise Stage4RunError("Shifting delta and solver argv differ")
    tmax = _float_arg(argv, "-tmax:")
    tout = _float_arg(argv, "-tout:")
    if not math.isclose(tmax, policy["expected_output"]["end_time_s"], rel_tol=0, abs_tol=1.0e-12):
        raise Stage4RunError("tmax differs from expected output")
    if not math.isclose(tout, policy["expected_output"]["output_period_s"], rel_tol=0, abs_tol=1.0e-12):
        raise Stage4RunError("tout differs from expected output")
    restart = policy["restart"]
    part_flags = [item for item in argv if item.startswith("-partbegin:")]
    if restart["enabled"]:
        expected_flag = f"-partbegin:{restart['part_index']}:{restart['part_first']}"
        if part_flags != [expected_flag] or argv[argv.index(expected_flag) + 1] != "/restart":
            raise Stage4RunError("restart argv differs")
        if policy["inputs"]["restart_part"] is None or policy["inputs"]["restart_head"] is None:
            raise Stage4RunError("restart inputs are absent")
    elif part_flags or policy["inputs"]["restart_part"] is not None or policy["inputs"]["restart_head"] is not None:
        raise Stage4RunError("fresh run carries restart state")
    limits = policy["limits"]
    if not 10 <= limits["monitor_interval_seconds"] <= 30 or limits["minimum_mem_available_bytes"] < 4294967296:
        raise Stage4RunError("resource contract is weaker than Stage 4 minimum")
    if limits["maximum_output_bytes"] > 536870912 or limits["wall_timeout_seconds"] > 3600:
        raise Stage4RunError("resource contract exceeds Stage 4 cap")
    if policy["sandbox"]["network"] or policy["sandbox"]["project_workspace_write_bind_count"] != 0:
        raise Stage4RunError("sandbox network/project-write boundary differs")
    if policy["parents"]["runner"]["path"] != str(SCRIPT_PATH) or policy["parents"]["schema"]["path"] != str(SCHEMA_PATH):
        raise Stage4RunError("runner/schema parent path differs")
    if policy_path.parent != PACKAGE_ROOT / "config/target_hosts" and policy_path.parent != Path("/home/zrj/scout_liquid_lab/audits"):
        raise Stage4RunError("policy path leaves reviewed config/audit roots")


def load_policy(path: Path) -> dict[str, Any]:
    schema = read_json(SCHEMA_PATH)
    policy = read_json(path)
    Draft202012Validator.check_schema(schema)
    errors = sorted(Draft202012Validator(schema).iter_errors(policy), key=lambda item: list(item.absolute_path))
    if errors:
        first = errors[0]
        raise Stage4RunError(f"policy schema failure at {list(first.absolute_path)}: {first.message}")
    semantic_validate(policy, path)
    return policy


def verify_parents_and_inputs(policy: dict[str, Any]) -> dict[str, Any]:
    parents = {name: verify_spec(spec, full=False, maximum=MAX_POLICY_BYTES) for name, spec in policy["parents"].items()}
    inputs: dict[str, Any] = {}
    for name, spec in policy["inputs"].items():
        inputs[name] = None if spec is None else verify_spec(spec, full=True, maximum=128 * 1024 * 1024)
    tools = {name: verify_spec(spec, full=False, maximum=16 * 1024 * 1024) for name, spec in policy["tools"].items()}
    if inputs["candidate"]["mode"] != "0400":
        raise Stage4RunError("source candidate is not disarmed 0400")
    return {"parents": parents, "inputs": inputs, "tools": tools}


def fresh_paths(policy: dict[str, Any]) -> list[Path]:
    run = policy["run"]
    return [Path(run[key]) for key in ("attempt_root", "start_receipt", "final_receipt", "failure_receipt", "stdout_log", "stderr_log", "resource_log")] + [Path(policy["sandbox"]["stage_root"])]


def preflight(policy_path: Path, *, require_fresh: bool) -> dict[str, Any]:
    policy = load_policy(policy_path)
    verified = verify_parents_and_inputs(policy)
    available = mem_available_bytes()
    if available < policy["limits"]["minimum_mem_available_bytes"]:
        raise Stage4RunError(f"MemAvailable below frozen minimum: {available}")
    competitors = compute_processes(policy["tools"]["nvidia_smi"]["path"])
    if competitors:
        raise Stage4RunError(f"competing GPU compute processes exist: {competitors}")
    devices = []
    for path_text in policy["gpu"]["device_paths"]:
        metadata = os.stat(path_text)
        if not stat.S_ISCHR(metadata.st_mode):
            raise Stage4RunError(f"GPU device is not character device: {path_text}")
        devices.append({"path": path_text, "major": os.major(metadata.st_rdev), "minor": os.minor(metadata.st_rdev)})
    collisions = [str(path) for path in fresh_paths(policy) if os.path.lexists(path)] if require_fresh else []
    if collisions:
        raise Stage4RunError(f"one-shot identity collision: {collisions}")
    return {
        "captured_at_utc": utc_now(),
        "policy": identity(policy_path, maximum=MAX_POLICY_BYTES),
        "schema": identity(SCHEMA_PATH, maximum=MAX_POLICY_BYTES),
        "runner": identity(SCRIPT_PATH, maximum=MAX_POLICY_BYTES),
        "verified": verified,
        "memory": {"mem_available_bytes": available, "psi": Path("/proc/pressure/memory").read_text(encoding="ascii").strip()},
        "gpu": gpu_sample(policy),
        "devices": devices,
        "compute_processes": competitors,
        "fresh": require_fresh,
    }


def mkdir_exclusive(path: Path, mode: int, uid: int, gid: int) -> None:
    os.mkdir(path, mode)
    os.chown(path, uid, gid, follow_symlinks=False)
    os.chmod(path, mode, follow_symlinks=False)


def copy_exclusive(spec: dict[str, Any], destination: Path, mode: int, uid: int, gid: int) -> dict[str, Any]:
    source_fd = os.open(spec["path"], os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    destination_fd = os.open(destination, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, mode)
    digest = hashlib.sha256()
    total = 0
    try:
        source_meta = os.fstat(source_fd)
        if not stat.S_ISREG(source_meta.st_mode) or source_meta.st_nlink != 1 or source_meta.st_ino != spec["inode"] or source_meta.st_size != spec["size_bytes"]:
            raise Stage4RunError(f"source changed while staging: {spec['path']}")
        os.fchown(destination_fd, uid, gid)
        os.fchmod(destination_fd, mode)
        while True:
            block = os.read(source_fd, 1 << 20)
            if not block:
                break
            digest.update(block)
            view = memoryview(block)
            while view:
                written = os.write(destination_fd, view)
                if written <= 0:
                    raise Stage4RunError("short stage write")
                view = view[written:]
            total += len(block)
        os.fsync(destination_fd)
    finally:
        os.close(source_fd)
        os.close(destination_fd)
    if total != spec["size_bytes"] or digest.hexdigest() != spec["sha256"]:
        raise Stage4RunError(f"staged content differs: {destination}")
    return identity(destination, maximum=128 * 1024 * 1024)


def prepare_stage(policy: dict[str, Any]) -> dict[str, Any]:
    stage = Path(policy["sandbox"]["stage_root"])
    mkdir_exclusive(stage, 0o700, 0, 0)
    for name in ("runtime", "case", "output"):
        mkdir_exclusive(stage / name, 0o700 if name != "output" else 0o777, 0, 0)
    if policy["restart"]["enabled"]:
        mkdir_exclusive(stage / "restart", 0o700, 0, 0)
    staged = {
        "candidate": copy_exclusive(policy["inputs"]["candidate"], stage / "runtime/DualSPHysics5.4_linux64", 0o555, 0, 0),
        "dsph_config": copy_exclusive(policy["inputs"]["dsph_config"], stage / "runtime/DsphConfig.xml", 0o444, 0, 0),
        "case_bi4": copy_exclusive(policy["inputs"]["case_bi4"], stage / "case/C1M_zero.bi4", 0o444, 0, 0),
        "case_xml": copy_exclusive(policy["inputs"]["case_xml"], stage / "case/C1M_zero.xml", 0o444, 0, 0),
    }
    if policy["restart"]["enabled"]:
        index = policy["restart"]["part_index"]
        staged["restart_part"] = copy_exclusive(policy["inputs"]["restart_part"], stage / "restart" / f"Part_{index:04d}.bi4", 0o444, 0, 0)
        staged["restart_head"] = copy_exclusive(policy["inputs"]["restart_head"], stage / "restart/Part_Head.ibi4", 0o444, 0, 0)
    return staged


def bwrap_argv(policy: dict[str, Any]) -> list[str]:
    stage = Path(policy["sandbox"]["stage_root"])
    argv = [
        policy["tools"]["bwrap"]["path"], "--die-with-parent", "--new-session",
        "--unshare-user", "--unshare-pid", "--unshare-net", "--unshare-ipc", "--unshare-uts", "--unshare-cgroup-try",
        "--uid", "1000", "--gid", "1000", "--cap-drop", "ALL", "--hostname", "r8-liquid-stage4", "--clearenv",
    ]
    for key, value in EXPECTED_ENVIRONMENT.items():
        argv.extend(("--setenv", key, value))
    argv.extend((
        "--ro-bind", "/usr", "/usr", "--symlink", "usr/lib", "/lib", "--symlink", "usr/lib64", "/lib64",
        "--dir", "/etc", "--ro-bind", "/etc/ld.so.cache", "/etc/ld.so.cache", "--proc", "/proc", "--dev", "/dev",
    ))
    for device in policy["gpu"]["device_paths"]:
        argv.extend(("--dev-bind", device, device))
    argv.extend(("--ro-bind", "/sys", "/sys", "--tmpfs", "/tmp"))
    argv.extend(("--ro-bind", str(stage / "runtime"), "/runtime", "--ro-bind", str(stage / "case"), "/case"))
    if policy["restart"]["enabled"]:
        argv.extend(("--ro-bind", str(stage / "restart"), "/restart"))
    argv.extend(("--bind", str(stage / "output"), "/output", "--chdir", "/runtime", "--", *policy["solver"]["argv"]))
    if "--unshare-net" not in argv or "--share-net" in argv:
        raise Stage4RunError("network namespace contract differs")
    write_binds = [argv[index + 1:index + 3] for index, item in enumerate(argv) if item == "--bind"]
    if write_binds != [[str(stage / "output"), "/output"]]:
        raise Stage4RunError("writable bind set differs")
    return argv


def write_bytes_exclusive(path: Path, data: bytes, *, mode: int = 0o640, uid: int = 1000, gid: int = 1000) -> dict[str, Any]:
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, mode)
    try:
        if os.geteuid() == 0:
            os.fchown(descriptor, uid, gid)
        os.fchmod(descriptor, mode)
        view = memoryview(data)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise Stage4RunError("short receipt write")
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return identity(path, maximum=max(MAX_POLICY_BYTES, len(data)))


def write_json_exclusive(path: Path, value: Any) -> dict[str, Any]:
    return write_bytes_exclusive(path, canonical_bytes(value))


def safe_output_inventory(root: Path, expected: dict[str, Any]) -> dict[str, Any]:
    entries = {item.name for item in os.scandir(root)}
    if entries != ROOT_OUTPUT_NAMES:
        raise Stage4RunError(f"output root names differ: {sorted(entries)}")
    data = root / "data"
    data_entries = {item.name for item in os.scandir(data)}
    part_names = sorted(name for name in data_entries if PART_RE.fullmatch(name))
    if data_entries != STATIC_DATA_NAMES | set(part_names):
        raise Stage4RunError(f"output data names differ: {sorted(data_entries)}")
    part_indices = [int(PART_RE.fullmatch(name).group(1)) for name in part_names]
    wanted = list(range(expected["part_first"], expected["part_last"] + 1))
    if part_indices != wanted or len(part_indices) != expected["part_count"]:
        raise Stage4RunError(f"Part range differs: observed={part_indices[:2]}..{part_indices[-2:] if part_indices else []}")
    records: dict[str, dict[str, Any]] = {}
    total = 0
    for base, dirs, files in os.walk(root, followlinks=False):
        for name in dirs:
            path = Path(base) / name
            if not stat.S_ISDIR(os.lstat(path).st_mode):
                raise Stage4RunError(f"unsafe output directory: {path}")
        for name in files:
            path = Path(base) / name
            item = identity(path)
            relative = str(path.relative_to(root))
            records[relative] = {key: item[key] for key in ("size_bytes", "sha256", "mode", "nlink")}
            total += item["size_bytes"]
    if total > expected["maximum_output_bytes"]:
        raise Stage4RunError(f"output exceeds bound: {total}")
    return {
        "file_count": len(records),
        "part_file_count": len(part_indices),
        "part_first": part_indices[0],
        "part_last": part_indices[-1],
        "total_bytes": total,
        "canonical_sha256": hashlib.sha256(json.dumps(records, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest(),
        "files": dict(sorted(records.items())),
    }


def export_output(stage_output: Path, lab_output: Path, expected: dict[str, Any]) -> dict[str, Any]:
    stage_inventory = safe_output_inventory(stage_output, expected)
    if list(os.scandir(lab_output)):
        raise Stage4RunError("lab output is not empty before export")
    mkdir_exclusive(lab_output / "data", 0o700, 1000, 1000)
    for relative in sorted(stage_inventory["files"]):
        source = stage_output / relative
        destination = lab_output / relative
        spec = identity(source)
        copy_exclusive(spec, destination, 0o440, 1000, 1000)
    lab_inventory = safe_output_inventory(lab_output, expected)
    if stage_inventory["canonical_sha256"] != lab_inventory["canonical_sha256"]:
        # Modes intentionally differ; compare only content identities.
        stage_content = {name: item["sha256"] for name, item in stage_inventory["files"].items()}
        lab_content = {name: item["sha256"] for name, item in lab_inventory["files"].items()}
        if stage_content != lab_content:
            raise Stage4RunError("exported output content differs")
    return lab_inventory


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


def execute(policy_path: Path) -> dict[str, Any]:
    if not (
        os.geteuid() == 0
        and os.environ.get("R8_STAGE4_ROOT_SUPERVISOR") == "1"
        and os.environ.get("SUDO_UID") == "1000"
        and os.environ.get("SUDO_GID") == "1000"
    ):
        raise Stage4RunError("run requires exact sudo root-supervisor identity")
    pre = preflight(policy_path, require_fresh=True)
    policy = load_policy(policy_path)
    run = policy["run"]
    lock_path = Path("/home/zrj/scout_liquid_lab/locks/gpu0.lock")
    lock_fd = os.open(lock_path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
    os.fchown(lock_fd, 1000, 1000)
    process: subprocess.Popen[bytes] | None = None
    started_wall = time.time()
    started_mono = time.monotonic()
    staged: dict[str, Any] | None = None
    output_inventory: dict[str, Any] | None = None
    returncode: int | None = None
    termination_reason = "NOT_STARTED"
    argv: list[str] = []
    try:
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise Stage4RunError("GPU lock is already held") from exc
        start = {
            "schema_version": "r8-liquid-u3-gpu-stage4-run-receipt-v1",
            "document_type": "SMPCC_R8_LIQUID_U3_GPU_STAGE4_RUN_START_V1",
            "status": "GPU_STAGE4_ONE_SHOT_STARTED",
            "run_id": policy["run_id"],
            "started_at_utc": utc_now(),
            "preflight": pre,
            "candidate_source_executed": False,
            "staged_candidate_execution_pending": True,
            "network_used": False,
            "project_workspace_writable_in_guest": False,
        }
        start_identity = write_json_exclusive(Path(run["start_receipt"]), start)
        mkdir_exclusive(Path(run["attempt_root"]), 0o700, 1000, 1000)
        mkdir_exclusive(Path(run["output_root"]), 0o700, 1000, 1000)
        staged = prepare_stage(policy)
        stdout_fd = os.open(run["stdout_log"], os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
        stderr_fd = os.open(run["stderr_log"], os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
        resource_fd = os.open(run["resource_log"], os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
        for descriptor in (stdout_fd, stderr_fd, resource_fd):
            os.fchown(descriptor, 1000, 1000)
            os.fchmod(descriptor, 0o640)
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
                    sample = resource_sample(policy, process.pid, started_mono, Path(policy["sandbox"]["stage_root"]) / "output")
                    os.write(resource_fd, canonical_bytes(sample))
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
            final_sample = resource_sample(policy, process.pid, started_mono, Path(policy["sandbox"]["stage_root"]) / "output")
            os.write(resource_fd, canonical_bytes(final_sample))
            os.fsync(resource_fd)
        finally:
            if process is not None and process.poll() is None:
                terminate_group(process, policy["limits"]["kill_after_seconds"])
            for descriptor in (stdout_fd, stderr_fd, resource_fd):
                os.close(descriptor)
        if returncode != 0 or termination_reason != "PROCESS_EXIT":
            raise Stage4RunError(f"solver failed rc={returncode} termination={termination_reason}")
        output_inventory = export_output(Path(policy["sandbox"]["stage_root"]) / "output", Path(run["output_root"]), policy["expected_output"])
        candidate_after = verify_spec(policy["inputs"]["candidate"], full=True, maximum=128 * 1024 * 1024)
        competitors_after = compute_processes(policy["tools"]["nvidia_smi"]["path"])
        if competitors_after:
            raise Stage4RunError(f"GPU compute process remains: {competitors_after}")
        final = {
            "schema_version": "r8-liquid-u3-gpu-stage4-run-receipt-v1",
            "document_type": "SMPCC_R8_LIQUID_U3_GPU_STAGE4_RUN_FINAL_V1",
            "status": "PASS_GPU_STAGE4_RUN_RAW_OUTPUT",
            "run_id": policy["run_id"],
            "purpose": policy["purpose"],
            "started_at_utc": datetime.fromtimestamp(started_wall, tz=timezone.utc).isoformat(),
            "completed_at_utc": utc_now(),
            "elapsed_seconds": round(time.monotonic() - started_mono, 6),
            "returncode": returncode,
            "termination_reason": termination_reason,
            "policy": identity(policy_path, maximum=MAX_POLICY_BYTES),
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
            "output_inventory": output_inventory,
            "stdout": identity(Path(run["stdout_log"])),
            "stderr": identity(Path(run["stderr_log"])),
            "resource_log": identity(Path(run["resource_log"]), maximum=16 * 1024 * 1024),
            "gpu_after": gpu_sample(policy),
            "compute_processes_after": competitors_after,
            "development_only": True,
            "formal": False,
            "physical_primary_eligible": False,
            "qc_pending": True,
        }
        receipt = write_json_exclusive(Path(run["final_receipt"]), final)
        final["final_receipt"] = receipt
        return final
    except Exception as exc:
        failure = {
            "schema_version": "r8-liquid-u3-gpu-stage4-run-receipt-v1",
            "document_type": "SMPCC_R8_LIQUID_U3_GPU_STAGE4_RUN_FAILURE_V1",
            "status": "FAIL_GPU_STAGE4_RUN",
            "run_id": policy.get("run_id") if isinstance(locals().get("policy"), dict) else None,
            "failed_at_utc": utc_now(),
            "elapsed_seconds": round(time.monotonic() - started_mono, 6),
            "error_type": type(exc).__name__,
            "error": str(exc),
            "returncode": returncode,
            "termination_reason": termination_reason,
            "stage_preserved": os.path.lexists(policy["sandbox"]["stage_root"]) if isinstance(locals().get("policy"), dict) else False,
            "attempt_root_preserved": os.path.lexists(run["attempt_root"]) if isinstance(locals().get("run"), dict) else False,
            "candidate_source_executed": False,
            "network_used": False,
            "project_workspace_writable_in_guest": False,
        }
        if isinstance(locals().get("run"), dict) and not os.path.lexists(run["failure_receipt"]):
            write_json_exclusive(Path(run["failure_receipt"]), failure)
        raise
    finally:
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_UN)
        finally:
            os.close(lock_fd)


def self_check(policy_path: Path) -> dict[str, Any]:
    policy = load_policy(policy_path)
    verified = verify_parents_and_inputs(policy)
    argv = bwrap_argv(policy)
    return {
        "status": "PASS_U3_GPU_STAGE4_RUNNER_V1_SELF_CHECK",
        "policy": identity(policy_path, maximum=MAX_POLICY_BYTES),
        "verified": verified,
        "sandbox_argv": argv,
        "network_used": False,
        "candidate_executed": False,
        "project_workspace_writable_in_guest": False,
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
            result = {"status": "PASS_U3_GPU_STAGE4_RUNNER_V1_PREFLIGHT", "evidence": preflight(args.policy, require_fresh=True)}
        else:
            result = execute(args.policy)
    except (Stage4RunError, OSError, ValueError, subprocess.SubprocessError) as exc:
        print(json.dumps({"status": "FAIL_U3_GPU_STAGE4_RUNNER_V1", "error_type": type(exc).__name__, "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
