#!/usr/bin/env python3
"""One-shot RTX 5080 C1M GPU smoke runner with an ephemeral root bwrap sandbox.

The only privileged operation is ``sudo -n bwrap``.  Bubblewrap creates fresh
mount/PID/network/IPC/UTS namespaces, then drops to uid/gid 1000 with every
capability removed.  The guest sees read-only system/input trees, four exact
NVIDIA character devices, and one writable output directory outside the Git
workspace.  The candidate is opened with O_NOFOLLOW, temporarily changed from
0400 to 0500, and restored to 0400 in a parent-side finally block.
"""

from __future__ import annotations

import argparse
import datetime as dt
import fcntl
import hashlib
import json
import os
import re
import signal
import stat
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, BinaryIO

import jsonschema


sys.dont_write_bytecode = True

SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
POLICY_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_gpu_runtime_smoke_v1.json"
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_gpu_runtime_smoke_policy_v1.json"

MAX_JSON_BYTES = 16 * 1024 * 1024
MAX_IDENTITY_BYTES = 160 * 1024 * 1024
EXPECTED_ROOT_FILES = {
    "CfgInit_Domain.vtk",
    "CfgInit_MapCells.vtk",
    "Run.csv",
    "Run.out",
    "RunPARTs.csv",
}
EXPECTED_DATA_FILES = {
    *(f"Part_{index:04d}.bi4" for index in range(21)),
    "Part_Head.ibi4",
    "PartInfo.ibi4",
    "PartMotionRef.ibi4",
    "PartOut_000.obi4",
}


class SmokeError(RuntimeError):
    """A fail-closed policy, identity, sandbox, execution, or QC error."""


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="microseconds")


def canonical_bytes(value: Any) -> bytes:
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")


def secure_read(path: Path, *, maximum: int = MAX_IDENTITY_BYTES) -> tuple[bytes, os.stat_result]:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise SmokeError(f"unsafe regular file identity: {path}")
        if not 0 < metadata.st_size <= maximum:
            raise SmokeError(f"file size outside 1..{maximum}: {path}")
        data = bytearray()
        while len(data) < metadata.st_size:
            block = os.read(descriptor, min(1 << 20, metadata.st_size - len(data)))
            if not block:
                raise SmokeError(f"short read: {path}")
            data.extend(block)
        if os.read(descriptor, 1):
            raise SmokeError(f"file grew during read: {path}")
        return bytes(data), metadata
    finally:
        os.close(descriptor)


def read_json(path: Path) -> dict[str, Any]:
    raw, _ = secure_read(path, maximum=MAX_JSON_BYTES)
    try:
        value = json.loads(raw.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise SmokeError(f"invalid JSON: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise SmokeError(f"JSON root is not an object: {path}")
    return value


def sha256_file(path: Path, *, maximum: int = MAX_IDENTITY_BYTES) -> str:
    raw, _ = secure_read(path, maximum=maximum)
    return hashlib.sha256(raw).hexdigest()


def identity(path: Path, *, maximum: int = MAX_IDENTITY_BYTES) -> dict[str, Any]:
    raw, metadata = secure_read(path, maximum=maximum)
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


def verify_file(expected: dict[str, Any], *, maximum: int = MAX_IDENTITY_BYTES) -> dict[str, Any]:
    observed = identity(Path(expected["path"]), maximum=maximum)
    for key in ("sha256", "size_bytes", "mode", "inode", "nlink"):
        if key in expected and observed[key] != expected[key]:
            raise SmokeError(
                f"identity mismatch for {expected['path']} field {key}: "
                f"{observed[key]!r} != {expected[key]!r}"
            )
    return observed


def write_bytes_exclusive(path: Path, raw: bytes, *, mode: int = 0o640) -> dict[str, Any]:
    descriptor = os.open(
        path,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        mode,
    )
    try:
        if os.geteuid() == 0:
            os.fchown(descriptor, 1000, 1000)
        offset = 0
        while offset < len(raw):
            offset += os.write(descriptor, raw[offset:])
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return identity(path, maximum=max(MAX_JSON_BYTES, len(raw) + 1))


def write_json_exclusive(path: Path, value: Any, *, mode: int = 0o640) -> dict[str, Any]:
    return write_bytes_exclusive(path, canonical_bytes(value), mode=mode)


def require_real_directory(path: Path, *, owner_uid: int | None = None) -> os.stat_result:
    metadata = os.lstat(path)
    if not stat.S_ISDIR(metadata.st_mode):
        raise SmokeError(f"not a real directory: {path}")
    if owner_uid is not None and metadata.st_uid != owner_uid:
        raise SmokeError(f"directory owner differs: {path}")
    return metadata


def ensure_directory(path: Path, *, mode: int = 0o700) -> None:
    try:
        os.mkdir(path, mode)
        if os.geteuid() == 0:
            os.chown(path, 1000, 1000)
    except FileExistsError:
        require_real_directory(path, owner_uid=1000)


def load_contract() -> tuple[dict[str, Any], dict[str, Any]]:
    policy = read_json(POLICY_PATH)
    schema = read_json(SCHEMA_PATH)
    try:
        jsonschema.Draft202012Validator.check_schema(schema)
        jsonschema.Draft202012Validator(schema).validate(policy)
    except jsonschema.exceptions.SchemaError as exc:
        raise SmokeError(f"invalid closed schema: {exc.message}") from exc
    except jsonschema.exceptions.ValidationError as exc:
        raise SmokeError(f"policy fails closed schema at {list(exc.absolute_path)}: {exc.message}") from exc
    return policy, schema


def verify_parent_receipt(spec: dict[str, Any]) -> dict[str, Any]:
    observed = verify_file(spec, maximum=8 * 1024 * 1024)
    payload = read_json(Path(spec["path"]))
    if "status" in spec and payload.get("status") != spec["status"]:
        raise SmokeError(f"parent receipt status mismatch: {spec['path']}")
    if payload.get("candidate_executed") is True and "build_final" in spec["path"]:
        raise SmokeError("build receipt unexpectedly says candidate was executed")
    observed["status"] = payload.get("status")
    return observed


def mem_available_bytes() -> int:
    for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) * 1024
    raise SmokeError("MemAvailable missing")


def memory_snapshot() -> dict[str, int]:
    wanted = {"MemAvailable", "SwapTotal", "SwapFree"}
    result: dict[str, int] = {}
    for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
        name = line.split(":", 1)[0]
        if name in wanted:
            result[name + "Bytes"] = int(line.split()[1]) * 1024
    if set(result) != {item + "Bytes" for item in wanted}:
        raise SmokeError("memory snapshot incomplete")
    return result


def pressure_snapshot() -> dict[str, str]:
    result: dict[str, str] = {}
    for line in Path("/proc/pressure/memory").read_text(encoding="ascii").splitlines():
        key = line.split()[0]
        result[key] = line
    return result


def run_text(argv: list[str], *, timeout: float = 20.0) -> tuple[int, str, str]:
    completed = subprocess.run(
        argv,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        check=False,
        text=True,
        encoding="utf-8",
        errors="replace",
        env={"PATH": "/usr/bin", "LC_ALL": "C", "LANG": "C"},
    )
    return completed.returncode, completed.stdout, completed.stderr


def gpu_snapshot(policy: dict[str, Any]) -> dict[str, Any]:
    argv = [
        policy["tools"]["nvidia_smi"]["path"],
        "--query-gpu=index,uuid,pci.bus_id,name,driver_version,memory.total,memory.used,temperature.gpu,power.draw",
        "--format=csv,noheader,nounits",
    ]
    rc, stdout, stderr = run_text(argv)
    if rc != 0:
        raise SmokeError(f"nvidia-smi query failed rc={rc}: {stderr.strip()}")
    rows = [line for line in stdout.splitlines() if line.strip()]
    if len(rows) != 1:
        raise SmokeError(f"expected one GPU row, observed {len(rows)}")
    fields = [value.strip() for value in rows[0].split(",")]
    if len(fields) != 9:
        raise SmokeError(f"GPU query field count differs: {rows[0]!r}")
    expected = policy["gpu"]
    if fields[:5] != [
        str(expected["index"]), expected["uuid"], expected["pci_bus_id"],
        expected["name"], expected["driver_version"],
    ]:
        raise SmokeError(f"GPU fixed identity drift: {fields[:5]!r}")
    return {
        "index": int(fields[0]),
        "uuid": fields[1],
        "pci_bus_id": fields[2],
        "name": fields[3],
        "driver_version": fields[4],
        "memory_total_mib": int(fields[5]),
        "memory_used_mib": int(fields[6]),
        "temperature_c": int(fields[7]),
        "power_draw_w": float(fields[8]),
    }


def compute_processes(policy: dict[str, Any]) -> list[dict[str, str]]:
    argv = [
        policy["tools"]["nvidia_smi"]["path"],
        "--query-compute-apps=pid,process_name,used_memory",
        "--format=csv,noheader,nounits",
    ]
    rc, stdout, stderr = run_text(argv)
    if rc != 0 and "No running processes found" not in stderr:
        raise SmokeError(f"compute-process query failed rc={rc}: {stderr.strip()}")
    rows: list[dict[str, str]] = []
    for line in stdout.splitlines():
        if not line.strip():
            continue
        values = [item.strip() for item in line.split(",", 2)]
        if len(values) == 3:
            rows.append({"pid": values[0], "name": values[1], "used_memory_mib": values[2]})
    return rows


def verify_device(path: Path) -> dict[str, Any]:
    metadata = os.lstat(path)
    if not stat.S_ISCHR(metadata.st_mode):
        raise SmokeError(f"GPU path is not a character device: {path}")
    return {
        "path": str(path),
        "mode": f"0{stat.S_IMODE(metadata.st_mode):03o}",
        "uid": metadata.st_uid,
        "gid": metadata.st_gid,
        "major": os.major(metadata.st_rdev),
        "minor": os.minor(metadata.st_rdev),
        "inode": metadata.st_ino,
    }


def bwrap_argv(policy: dict[str, Any], *, command: list[str], include_run: bool) -> list[str]:
    sandbox = policy["sandbox"]
    environment = policy["solver"]["environment"]
    prefix = (
        [policy["tools"]["bwrap"]["path"]]
        if os.geteuid() == 0
        else [policy["tools"]["sudo"]["path"], "-n", "--", policy["tools"]["bwrap"]["path"]]
    )
    argv = [
        *prefix,
        "--die-with-parent", "--new-session", "--unshare-user",
        "--unshare-pid", "--unshare-net", "--unshare-ipc", "--unshare-uts", "--unshare-cgroup-try",
        "--uid", str(sandbox["child_uid"]), "--gid", str(sandbox["child_gid"]),
        "--cap-drop", "ALL", "--hostname", "r8-liquid-gpu-smoke-v1", "--clearenv",
    ]
    for key, value in environment.items():
        argv.extend(("--setenv", key, value))
    argv.extend((
        "--ro-bind", "/usr", "/usr",
        "--symlink", "usr/lib", "/lib",
        "--symlink", "usr/lib64", "/lib64",
        "--dir", "/etc",
        "--ro-bind", "/etc/ld.so.cache", "/etc/ld.so.cache",
        "--proc", "/proc",
        "--dev", "/dev",
    ))
    for device in policy["gpu"]["device_paths"]:
        argv.extend(("--dev-bind", device, device))
    argv.extend(("--ro-bind", "/sys", "/sys", "--tmpfs", "/tmp"))
    if include_run:
        inputs = policy["inputs"]
        argv.extend((
            "--dir", "/runtime",
            "--ro-bind", inputs["candidate"]["path"], "/runtime/DualSPHysics5.4_linux64",
            "--ro-bind", inputs["dsph_config"]["path"], "/runtime/DsphConfig.xml",
            "--dir", "/case",
            "--ro-bind", inputs["case_bi4"]["path"], "/case/C1M_zero.bi4",
            "--ro-bind", inputs["case_xml"]["path"], "/case/C1M_zero.xml",
            "--dir", "/output",
            "--bind", policy["run"]["output_root"], "/output",
            "--chdir", "/runtime",
        ))
    else:
        argv.extend(("--chdir", "/tmp"))
    argv.extend(("--", *command))
    verify_bwrap_argv(policy, argv, command=command, include_run=include_run)
    return argv


def pairs(argv: list[str], flag: str) -> list[list[str]]:
    width = 2
    return [argv[index + 1:index + 1 + width] for index, value in enumerate(argv) if value == flag]


def verify_bwrap_argv(policy: dict[str, Any], argv: list[str], *, command: list[str], include_run: bool) -> None:
    expected_prefix = (
        [policy["tools"]["bwrap"]["path"]]
        if os.geteuid() == 0
        else [policy["tools"]["sudo"]["path"], "-n", "--", policy["tools"]["bwrap"]["path"]]
    )
    if argv[:len(expected_prefix)] != expected_prefix:
        raise SmokeError("root/sudo bwrap prefix differs")
    for required in ("--unshare-user", "--unshare-pid", "--unshare-net", "--unshare-ipc", "--unshare-uts", "--cap-drop"):
        if argv.count(required) != 1:
            raise SmokeError(f"sandbox option count differs: {required}")
    if "--share-net" in argv or argv[argv.index("--cap-drop") + 1] != "ALL":
        raise SmokeError("network/capability isolation differs")
    if argv[argv.index("--uid") + 1] != "1000" or argv[argv.index("--gid") + 1] != "1000":
        raise SmokeError("guest uid/gid differs")
    if [pair for pair in pairs(argv, "--dev-bind")] != [[p, p] for p in policy["gpu"]["device_paths"]]:
        raise SmokeError("GPU device bind set differs")
    bind_pairs = pairs(argv, "--bind")
    if bind_pairs != ([[policy["run"]["output_root"], "/output"]] if include_run else []):
        raise SmokeError("writable bind set differs")
    if any("/home/zrj/scout_ws" in item for pair in bind_pairs for item in pair):
        raise SmokeError("Git workspace is writable in guest")
    if argv[-len(command):] != command:
        raise SmokeError("guest command suffix differs")


def static_semantic_check(policy: dict[str, Any]) -> None:
    run = policy["run"]
    root = Path(run["attempt_root"])
    if Path(run["output_root"]) != root / "output":
        raise SmokeError("output root is not the exact child of attempt root")
    if root.parent != Path("/home/zrj/scout_liquid_lab/gpu_runs"):
        raise SmokeError("attempt root leaves dedicated lab gpu_runs")
    if any(Path(run[key]).parent != Path("/home/zrj/scout_liquid_lab/audits") for key in (
        "start_receipt", "final_receipt", "failure_receipt", "stdout_log", "stderr_log", "resource_log", "qc_report"
    )):
        raise SmokeError("audit output leaves dedicated lab audits")
    expected_argv = [
        "/runtime/DualSPHysics5.4_linux64", "/case/C1M_zero", "/output",
        "-gpu:0", "-ompthreads:1", "-stable:1", "-vres:0", "-cellmode:full",
        "-tmax:1.0", "-tout:0.05", "-sv:binx,info", "-svres:1",
        "-svtimers:0", "-svdomainvtk:0", "-saveposdouble:1",
        "-nortimes:1", "-createdirs:1", "-csvsep:0",
    ]
    if policy["solver"]["argv"] != expected_argv:
        raise SmokeError("solver argv differs from frozen C1M smoke")
    if policy["sandbox"]["project_write_bind_count"] != 0 or policy["sandbox"]["host_root_write_bind_count"] != 1:
        raise SmokeError("sandbox writable boundary differs")
    bwrap_argv(policy, command=["/usr/bin/nvidia-smi", "-L"], include_run=False)
    bwrap_argv(policy, command=expected_argv, include_run=True)


def fresh_paths(policy: dict[str, Any]) -> list[Path]:
    run = policy["run"]
    return [Path(run[key]) for key in (
        "attempt_root", "start_receipt", "final_receipt", "failure_receipt",
        "stdout_log", "stderr_log", "resource_log", "qc_report",
    )]


def preflight(*, require_fresh: bool = True) -> dict[str, Any]:
    root_supervisor = (
        os.geteuid() == 0
        and os.environ.get("R8_GPU_SMOKE_ROOT_SUPERVISOR") == "1"
        and os.environ.get("SUDO_UID") == "1000"
        and os.environ.get("SUDO_GID") == "1000"
    )
    if not root_supervisor and (os.getuid() != 1000 or os.getgid() != 1000):
        raise SmokeError(f"host identity differs: uid={os.getuid()} gid={os.getgid()} groups={os.getgroups()}")
    policy, schema = load_contract()
    static_semantic_check(policy)
    tools = {name: verify_file(spec) for name, spec in policy["tools"].items()}
    qc_tools = {
        "script": verify_file(policy["qc"]["script"], maximum=4 * 1024 * 1024),
        "reader": verify_file(policy["qc"]["reader"], maximum=4 * 1024 * 1024),
    }
    parents = {name: verify_parent_receipt(spec) for name, spec in policy["parents"].items() if name != "plan"}
    parents["plan"] = verify_file(policy["parents"]["plan"], maximum=4 * 1024 * 1024)
    inputs = {name: verify_file(spec) for name, spec in policy["inputs"].items()}
    available = mem_available_bytes()
    if available < policy["limits"]["minimum_mem_available_bytes"]:
        raise SmokeError(f"MemAvailable below 4 GiB: {available}")
    devices = [verify_device(Path(path)) for path in policy["gpu"]["device_paths"]]
    gpu = gpu_snapshot(policy)
    competitors = compute_processes(policy)
    if competitors:
        raise SmokeError(f"competing compute processes exist: {competitors}")
    if require_fresh:
        collisions = [str(path) for path in fresh_paths(policy) if os.path.lexists(path)]
        if collisions:
            raise SmokeError(f"one-shot paths already exist: {collisions}")
    return {
        "captured_at_utc": utc_now(),
        "policy": identity(POLICY_PATH, maximum=4 * 1024 * 1024),
        "schema": identity(SCHEMA_PATH, maximum=4 * 1024 * 1024),
        "runner": identity(SCRIPT_PATH, maximum=4 * 1024 * 1024),
        "tools": tools,
        "qc_tools": qc_tools,
        "parents": parents,
        "inputs": inputs,
        "gpu": gpu,
        "devices": devices,
        "compute_processes": competitors,
        "memory": memory_snapshot(),
        "memory_psi": pressure_snapshot(),
        "fresh_paths": require_fresh,
        "root_supervisor": root_supervisor,
    }


def normalize_output_ownership(root: Path) -> None:
    """Make root-created user-namespace output user-owned without following links."""

    if os.geteuid() != 0:
        return
    require_real_directory(root)
    for base, dirs, files in os.walk(root, topdown=False, followlinks=False):
        for name in files:
            path = Path(base) / name
            metadata = os.lstat(path)
            if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
                raise SmokeError(f"unsafe output before ownership normalization: {path}")
            os.chown(path, 1000, 1000, follow_symlinks=False)
        for name in dirs:
            path = Path(base) / name
            metadata = os.lstat(path)
            if not stat.S_ISDIR(metadata.st_mode):
                raise SmokeError(f"unsafe output directory before ownership normalization: {path}")
            os.chown(path, 1000, 1000, follow_symlinks=False)
    os.chown(root, 1000, 1000, follow_symlinks=False)


def output_usage(root: Path) -> tuple[int, int]:
    count = 0
    total = 0
    if not root.exists():
        return count, total
    for base, dirs, files in os.walk(root, followlinks=False):
        for name in dirs:
            if (Path(base) / name).is_symlink():
                raise SmokeError("symlink directory appeared in output")
        for name in files:
            path = Path(base) / name
            metadata = os.lstat(path)
            if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
                raise SmokeError(f"unsafe output appeared: {path}")
            count += 1
            total += metadata.st_size
    return count, total


def process_group_rss_bytes(pgid: int) -> int:
    total_pages = 0
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / "stat").read_text(encoding="ascii").split()
            if int(fields[4]) == pgid:
                total_pages += int(fields[23])
        except (FileNotFoundError, PermissionError, ValueError, IndexError):
            continue
    return total_pages * os.sysconf("SC_PAGE_SIZE")


def resource_sample(policy: dict[str, Any], *, pgid: int, started: float, output: Path) -> dict[str, Any]:
    count, total = output_usage(output)
    return {
        "captured_at_utc": utc_now(),
        "elapsed_seconds": round(time.monotonic() - started, 6),
        "memory": memory_snapshot(),
        "memory_psi": pressure_snapshot(),
        "process_group_rss_bytes": process_group_rss_bytes(pgid),
        "gpu": gpu_snapshot(policy),
        "output_file_count": count,
        "output_bytes": total,
    }


def terminate_group(process: subprocess.Popen[bytes], *, kill_after: int) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + kill_after
    while process.poll() is None and time.monotonic() < deadline:
        time.sleep(0.1)
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=5)


def scan_exact_output(root: Path, *, maximum_total: int) -> dict[str, Any]:
    require_real_directory(root, owner_uid=1000)
    root_entries = {entry.name: entry for entry in os.scandir(root)}
    if set(root_entries) != EXPECTED_ROOT_FILES | {"data"}:
        raise SmokeError(
            f"output root inventory differs: missing={sorted((EXPECTED_ROOT_FILES | {'data'}) - set(root_entries))} "
            f"extra={sorted(set(root_entries) - (EXPECTED_ROOT_FILES | {'data'}))}"
        )
    data = root / "data"
    require_real_directory(data, owner_uid=1000)
    data_entries = {entry.name: entry for entry in os.scandir(data)}
    if set(data_entries) != EXPECTED_DATA_FILES:
        raise SmokeError(
            f"output data inventory differs: missing={sorted(EXPECTED_DATA_FILES - set(data_entries))} "
            f"extra={sorted(set(data_entries) - EXPECTED_DATA_FILES)}"
        )
    records: list[dict[str, Any]] = []
    total = 0
    for relative in sorted(EXPECTED_ROOT_FILES) + [f"data/{name}" for name in sorted(EXPECTED_DATA_FILES)]:
        item = identity(root / relative, maximum=32 * 1024 * 1024)
        if item["uid"] != 1000 or item["gid"] != 1000:
            raise SmokeError(f"output ownership differs: {relative}")
        total += item["size_bytes"]
        item["relative_path"] = relative
        item.pop("path")
        records.append(item)
    if len(records) != 30 or total > maximum_total:
        raise SmokeError(f"output aggregate differs: files={len(records)} bytes={total}")
    canonical = [{key: value for key, value in item.items() if key != "inode"} for item in records]
    return {
        "file_count": len(records),
        "part_file_count": sum(1 for item in records if re.fullmatch(r"data/Part_[0-9]{4}\.bi4", item["relative_path"])),
        "total_bytes": total,
        "canonical_sha256": hashlib.sha256(canonical_bytes(canonical)).hexdigest(),
        "files": records,
    }


def run_qc(policy: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any], list[str]]:
    command = [
        policy["tools"]["python"]["path"], "-B", policy["qc"]["script"]["path"],
        "--run-dir", policy["run"]["output_root"],
        "--case-bi4", policy["inputs"]["case_bi4"]["path"],
        "--case-xml", policy["inputs"]["case_xml"]["path"],
        "--expected-case-bi4-sha256", policy["inputs"]["case_bi4"]["sha256"],
        "--expected-case-xml-sha256", policy["inputs"]["case_xml"]["sha256"],
    ]
    completed = subprocess.run(
        command,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=180,
        check=False,
        env={"PATH": "/usr/bin", "LC_ALL": "C", "LANG": "C", "PYTHONDONTWRITEBYTECODE": "1"},
    )
    if completed.returncode != 0:
        raise SmokeError(f"output QC failed rc={completed.returncode}: {completed.stderr[-4096:].decode('utf-8', 'replace')}")
    if len(completed.stdout) > MAX_JSON_BYTES:
        raise SmokeError("output QC report exceeds ceiling")
    try:
        report = json.loads(completed.stdout.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise SmokeError(f"output QC did not return JSON: {exc}") from exc
    status_value = report.get("verdict", {}).get("status", report.get("status"))
    if status_value != policy["qc"]["expected_status"]:
        raise SmokeError(f"output QC status differs: {status_value!r}")
    publication = write_bytes_exclusive(Path(policy["run"]["qc_report"]), canonical_bytes(report))
    return report, publication, command


def log_safety(stdout_path: Path, stderr_path: Path) -> dict[str, Any]:
    stdout_raw, _ = secure_read(stdout_path, maximum=32 * 1024 * 1024)
    stderr_raw, _ = secure_read(stderr_path, maximum=32 * 1024 * 1024)
    text = (stdout_raw + b"\n" + stderr_raw).decode("utf-8", "replace")
    forbidden_patterns = {
        "cuda_error": r"(?i)CUDA\s+(?:error|failure)|cudaError",
        "nvidia_xid": r"(?i)NVRM:\s*Xid|\bXid\b",
        "nan_inf": r"(?i)(?:^|[^A-Za-z])(nan|inf)(?:[^A-Za-z]|$)",
        "exception": r"\*\*\*\s*Exception|Simulation INTERRUPTED",
    }
    matches = {name: bool(re.search(pattern, text)) for name, pattern in forbidden_patterns.items()}
    if any(matches.values()):
        raise SmokeError(f"solver log safety scan failed: {matches}")
    return {"forbidden_matches": matches, "stdout_bytes": len(stdout_raw), "stderr_bytes": len(stderr_raw)}


def sandbox_probe() -> dict[str, Any]:
    evidence = preflight(require_fresh=True)
    policy, _ = load_contract()
    identity_command = [
        "/usr/bin/python3.12", "-I", "-B", "-S", "-c",
        "import json,os; s=dict(line.split(':',1) for line in open('/proc/self/status') if ':' in line); print(json.dumps({'uid':os.getuid(),'gid':os.getgid(),'groups':os.getgroups(),'cap_eff':s['CapEff'].strip()}))",
    ]
    identity_argv = bwrap_argv(policy, command=identity_command, include_run=False)
    identity_completed = subprocess.run(
        identity_argv,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=30,
        check=False,
    )
    try:
        child_identity = json.loads(identity_completed.stdout.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise SmokeError(
            f"sandbox identity probe returned invalid JSON rc={identity_completed.returncode}: "
            f"{identity_completed.stderr.decode('utf-8', 'replace')!r}"
        ) from exc
    if (
        identity_completed.returncode != 0
        or child_identity.get("uid") != 1000
        or child_identity.get("gid") != 1000
        or child_identity.get("cap_eff") != "0000000000000000"
    ):
        raise SmokeError(
            f"sandbox child identity/capabilities differ rc={identity_completed.returncode}: {child_identity!r}"
        )
    command = ["/usr/bin/nvidia-smi", "-L"]
    argv = bwrap_argv(policy, command=command, include_run=False)
    completed = subprocess.run(
        argv,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=30,
        check=False,
    )
    stdout = completed.stdout.decode("utf-8", "replace")
    stderr = completed.stderr.decode("utf-8", "replace")
    expected_marker = policy["gpu"]["uuid"]
    if completed.returncode != 0 or expected_marker not in stdout:
        raise SmokeError(f"GPU sandbox probe failed rc={completed.returncode} stdout={stdout!r} stderr={stderr!r}")
    return {
        "status": "PASS_EPHEMERAL_GPU_SANDBOX_PROBE",
        "identity_argv": identity_argv,
        "child_identity": child_identity,
        "argv": argv,
        "returncode": completed.returncode,
        "stdout": stdout,
        "stderr": stderr,
        "preflight": evidence,
        "candidate_executed": False,
    }


def execute_one_shot() -> dict[str, Any]:
    pre = preflight(require_fresh=True)
    policy, _ = load_contract()
    run = policy["run"]
    audits = Path("/home/zrj/scout_liquid_lab/audits")
    lab = Path("/home/zrj/scout_liquid_lab")
    require_real_directory(lab, owner_uid=1000)
    require_real_directory(audits, owner_uid=1000)
    ensure_directory(Path(run["gpu_lock"]).parent)
    lock_fd = os.open(run["gpu_lock"], os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
    if os.geteuid() == 0:
        os.fchown(lock_fd, 1000, 1000)
    candidate_fd: int | None = None
    process: subprocess.Popen[bytes] | None = None
    candidate_executed = False
    candidate_mode_restored = False
    start_identity: dict[str, Any] | None = None
    started_wall = time.time()
    started_mono = time.monotonic()
    try:
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise SmokeError("GPU lock is held by another process") from exc
        if compute_processes(policy):
            raise SmokeError("compute process appeared after preflight")
        start_receipt = {
            "schema_version": "r8-liquid-u3-gpu-runtime-smoke-receipt-v1",
            "document_type": "SMPCC_R8_LIQUID_U3_GPU_RUNTIME_SMOKE_START_V1",
            "status": "ONE_SHOT_STARTED",
            "run_id": run["run_id"],
            "started_at_utc": utc_now(),
            "preflight": pre,
            "candidate_executed": False,
            "network_used": False,
            "project_workspace_writable_in_guest": False,
        }
        start_identity = write_json_exclusive(Path(run["start_receipt"]), start_receipt)
        ensure_directory(Path(run["attempt_root"]).parent)
        ensure_directory(Path(run["attempt_root"]))
        ensure_directory(Path(run["output_root"]))

        stdout_fd = os.open(run["stdout_log"], os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
        stderr_fd = os.open(run["stderr_log"], os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
        resource_stream = os.fdopen(
            os.open(run["resource_log"], os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640),
            "wb", buffering=0,
        )
        try:
            candidate_path = Path(policy["inputs"]["candidate"]["path"])
            candidate_fd = os.open(candidate_path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
            candidate_before = identity(candidate_path)
            if candidate_before["mode"] != "0400":
                raise SmokeError(f"candidate is not disarmed before run: {candidate_before['mode']}")
            os.fchmod(candidate_fd, 0o500)
            candidate_armed = identity(candidate_path)
            if candidate_armed["mode"] != "0500" or candidate_armed["inode"] != candidate_before["inode"]:
                raise SmokeError("candidate temporary execute-mode transition failed")

            argv = bwrap_argv(policy, command=policy["solver"]["argv"], include_run=True)
            process = subprocess.Popen(
                argv,
                stdin=subprocess.DEVNULL,
                stdout=stdout_fd,
                stderr=stderr_fd,
                close_fds=True,
                start_new_session=True,
                env={"PATH": "/usr/bin", "LC_ALL": "C", "LANG": "C"},
            )
            candidate_executed = True
            termination_reason = "PROCESS_EXIT"
            next_sample = 0.0
            while process.poll() is None:
                elapsed = time.monotonic() - started_mono
                if elapsed >= policy["limits"]["wall_timeout_seconds"]:
                    termination_reason = "WALL_TIMEOUT"
                    terminate_group(process, kill_after=policy["limits"]["kill_after_seconds"])
                    break
                if elapsed >= next_sample:
                    sample = resource_sample(policy, pgid=process.pid, started=started_mono, output=Path(run["output_root"]))
                    if sample["output_bytes"] > policy["limits"]["maximum_output_bytes"]:
                        termination_reason = "OUTPUT_LIMIT"
                        resource_stream.write(canonical_bytes(sample))
                        terminate_group(process, kill_after=policy["limits"]["kill_after_seconds"])
                        break
                    resource_stream.write(canonical_bytes(sample))
                    next_sample = elapsed + policy["limits"]["monitor_interval_seconds"]
                time.sleep(0.2)
            returncode = process.wait(timeout=5)
        finally:
            if process is not None and process.poll() is None:
                terminate_group(process, kill_after=policy["limits"]["kill_after_seconds"])
            if candidate_fd is not None:
                os.fchmod(candidate_fd, 0o400)
                os.close(candidate_fd)
                candidate_fd = None
                candidate_mode_restored = True
            os.close(stdout_fd)
            os.close(stderr_fd)
            resource_stream.close()

        candidate_after = identity(Path(policy["inputs"]["candidate"]["path"]))
        if candidate_after["mode"] != "0400" or candidate_after["sha256"] != policy["inputs"]["candidate"]["sha256"]:
            raise SmokeError("candidate identity/mode not restored after execution")
        if returncode != 0 or termination_reason != "PROCESS_EXIT":
            raise SmokeError(f"solver failed rc={returncode} termination={termination_reason}")
        normalize_output_ownership(Path(run["output_root"]))
        output_inventory = scan_exact_output(
            Path(run["output_root"]), maximum_total=policy["limits"]["maximum_output_bytes"]
        )
        if output_inventory["file_count"] != policy["qc"]["expected_file_count"] or output_inventory["part_file_count"] != policy["qc"]["expected_part_count"]:
            raise SmokeError("output file/Part count differs")
        log_check = log_safety(Path(run["stdout_log"]), Path(run["stderr_log"]))
        qc_report, qc_identity, qc_argv = run_qc(policy)
        final = {
            "schema_version": "r8-liquid-u3-gpu-runtime-smoke-receipt-v1",
            "document_type": "SMPCC_R8_LIQUID_U3_GPU_RUNTIME_SMOKE_FINAL_V1",
            "status": "PASS_GPU_FUNCTIONAL_SMOKE_DEVELOPMENT_ONLY",
            "run_id": run["run_id"],
            "started_at_utc": dt.datetime.fromtimestamp(started_wall, tz=dt.timezone.utc).isoformat(),
            "completed_at_utc": utc_now(),
            "elapsed_seconds": round(time.monotonic() - started_mono, 6),
            "returncode": returncode,
            "termination_reason": termination_reason,
            "sandbox_argv": argv,
            "solver_argv": policy["solver"]["argv"],
            "start_receipt": start_identity,
            "candidate_before": candidate_before,
            "candidate_armed": candidate_armed,
            "candidate_after": candidate_after,
            "candidate_mode_restored": candidate_mode_restored,
            "candidate_executed": candidate_executed,
            "gpu_runtime_status": "PASS_GPU_FUNCTIONAL_SMOKE_DEVELOPMENT_ONLY",
            "network_used": False,
            "ros_gazebo_used": False,
            "project_workspace_writable_in_guest": False,
            "gpu": gpu_snapshot(policy),
            "compute_processes_after": compute_processes(policy),
            "output_inventory": output_inventory,
            "stdout": identity(Path(run["stdout_log"]), maximum=32 * 1024 * 1024),
            "stderr": identity(Path(run["stderr_log"]), maximum=32 * 1024 * 1024),
            "resource_log": identity(Path(run["resource_log"]), maximum=16 * 1024 * 1024),
            "log_safety": log_check,
            "qc_argv": qc_argv,
            "qc": qc_identity,
            "qc_status": qc_report.get("verdict", {}).get("status", qc_report.get("status")),
            "development_only": True,
            "formal": False,
            "physical_primary_eligible": False,
            "next": "STAGE4_LIQUID_STANDALONE_VALIDATION",
        }
        final_identity = write_json_exclusive(Path(run["final_receipt"]), final)
        final["final_receipt"] = final_identity
        return final
    except Exception as exc:
        if candidate_fd is not None:
            try:
                os.fchmod(candidate_fd, 0o400)
                candidate_mode_restored = True
            finally:
                os.close(candidate_fd)
        failure = {
            "schema_version": "r8-liquid-u3-gpu-runtime-smoke-receipt-v1",
            "document_type": "SMPCC_R8_LIQUID_U3_GPU_RUNTIME_SMOKE_FAILURE_V1",
            "status": "FAIL_GPU_FUNCTIONAL_SMOKE",
            "run_id": run["run_id"],
            "failed_at_utc": utc_now(),
            "elapsed_seconds": round(time.monotonic() - started_mono, 6),
            "error_type": type(exc).__name__,
            "error": str(exc),
            "candidate_executed": candidate_executed,
            "candidate_mode_restored": candidate_mode_restored,
            "network_used": False,
            "project_workspace_writable_in_guest": False,
            "attempt_root_preserved": os.path.lexists(run["attempt_root"]),
        }
        failure_path = Path(run["failure_receipt"])
        if not os.path.lexists(failure_path):
            write_json_exclusive(failure_path, failure)
        raise
    finally:
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_UN)
        finally:
            os.close(lock_fd)


def self_check() -> dict[str, Any]:
    policy, _ = load_contract()
    static_semantic_check(policy)
    return {
        "status": "PASS_U3_GPU_RUNTIME_SMOKE_V1_SELF_CHECK",
        "policy": identity(POLICY_PATH, maximum=4 * 1024 * 1024),
        "schema": identity(SCHEMA_PATH, maximum=4 * 1024 * 1024),
        "runner": identity(SCRIPT_PATH, maximum=4 * 1024 * 1024),
        "candidate_executed": False,
        "system_action_used": False,
    }


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("command", choices=("self-check", "preflight", "sandbox-probe", "run"))
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        if args.command == "self-check":
            result = self_check()
        elif args.command == "preflight":
            result = {"status": "PASS_U3_GPU_RUNTIME_SMOKE_PREFLIGHT", "evidence": preflight(require_fresh=True)}
        elif args.command == "sandbox-probe":
            result = sandbox_probe()
        else:
            result = execute_one_shot()
    except (SmokeError, OSError, ValueError, subprocess.SubprocessError) as exc:
        print(canonical_bytes({"status": "FAIL_U3_GPU_RUNTIME_SMOKE_GATE", "error_type": type(exc).__name__, "error": str(exc)}).decode("utf-8"), end="", file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
