#!/usr/bin/env python3
"""Fresh RTX 5080 build campaign gate used on the path to liquid stage 4.

This gate intentionally leaves every historical campaign untouched.  It copies
the sealed 352-file source set into one create-new attempt root, adds the frozen
84-byte wrapper, and runs one network-less, GPU-less, resource-bounded Make
inside the exact AppArmor/bwrap profile from the campaign policy.

The only persistent host system action is a temporary AppArmor load/remove
lifecycle.  The build child always runs as uid/gid 1000 with no supplementary
groups.  Receipts and logs are create-new and failures are preserved.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import signal
import stat
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from types import ModuleType
from typing import Any, Iterable, Mapping, Sequence


WORKSPACE = Path("/home/zrj/scout_ws")
PACKAGE = WORKSPACE / "src/scout_apps/simulation/scout_dualsphysics_liquid"
POLICY_PATH = PACKAGE / "config/target_hosts/liquid_zrj_msi_u2404_gpu_stage4_campaign_20260810T162710Z_v1.json"
G1_GATE_PATH = PACKAGE / "scripts/r8_liquid_target_u3_gpu_build_gate_v1.py"


class CampaignError(RuntimeError):
    pass


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def canonical_json(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_fd(descriptor: int) -> str:
    digest = hashlib.sha256()
    os.lseek(descriptor, 0, os.SEEK_SET)
    while True:
        block = os.read(descriptor, 1024 * 1024)
        if not block:
            break
        digest.update(block)
    return digest.hexdigest()


def sha256_file(path: Path, *, limit: int | None = None) -> str:
    info = path.lstat()
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise CampaignError(f"not a single-link regular file: {path}")
    if limit is not None and info.st_size > limit:
        raise CampaignError(f"file exceeds hash limit: {path}")
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        return sha256_fd(descriptor)
    finally:
        os.close(descriptor)


def read_json(path: Path, *, limit: int = 64 * 1024 * 1024) -> dict[str, Any]:
    info = path.lstat()
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_size > limit:
        raise CampaignError(f"unsafe JSON input: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise CampaignError(f"JSON root is not an object: {path}")
    return value


def write_all(descriptor: int, data: bytes) -> None:
    offset = 0
    while offset < len(data):
        written = os.write(descriptor, data[offset:])
        if written <= 0:
            raise CampaignError("short write")
        offset += written


def write_new(path: Path, data: bytes, *, mode: int) -> None:
    descriptor = os.open(
        path,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        mode,
    )
    try:
        write_all(descriptor, data)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    os.chmod(path, mode, follow_symlinks=False)


def write_json_new(path: Path, value: Mapping[str, Any], *, mode: int = 0o640) -> None:
    write_new(path, json.dumps(value, sort_keys=True, indent=2, ensure_ascii=False).encode("utf-8") + b"\n", mode=mode)


def load_g1() -> ModuleType:
    spec = importlib.util.spec_from_file_location("r8_gpu_g1_frozen_for_fresh_campaign", G1_GATE_PATH)
    if spec is None or spec.loader is None:
        raise CampaignError("cannot load frozen G1 module")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def policy() -> dict[str, Any]:
    return read_json(POLICY_PATH)


def workspace_path(relative: str) -> Path:
    path = (WORKSPACE / relative).resolve()
    if not path.is_relative_to(WORKSPACE.resolve()):
        raise CampaignError(f"workspace-relative path escapes root: {relative}")
    return path


def assert_no_symlink_components(path: Path, *, leaf_may_be_absent: bool) -> None:
    absolute = path if path.is_absolute() else path.resolve()
    parts = absolute.parts
    current = Path(parts[0])
    for index, part in enumerate(parts[1:], start=1):
        current = current / part
        if leaf_may_be_absent and index == len(parts) - 1 and not current.exists():
            return
        info = current.lstat()
        if stat.S_ISLNK(info.st_mode):
            raise CampaignError(f"symlink path component: {current}")


def source_entries(current: Mapping[str, Any]) -> list[dict[str, Any]]:
    receipt_path = Path(current["source"]["receipt"])
    if sha256_file(receipt_path) != current["source"]["receipt_sha256"]:
        raise CampaignError("source receipt hash drift")
    receipt = read_json(receipt_path)
    if receipt.get("status") != "PASS_STATIC_SOURCE_MATERIALIZATION":
        raise CampaignError("source receipt status drift")
    sealed = receipt["results"]["materialization"]["sealed_output"]
    if sealed["manifest_sha256"] != current["source"]["manifest_sha256"]:
        raise CampaignError("sealed manifest hash drift")
    entries = sealed["entries"]
    if not isinstance(entries, list) or len(entries) != current["source"]["file_count"]:
        raise CampaignError("sealed source count drift")
    if sum(int(item["size_bytes"]) for item in entries) != current["source"]["total_bytes"]:
        raise CampaignError("sealed source byte count drift")
    return entries


def validate_sealed_source(current: Mapping[str, Any]) -> dict[str, Any]:
    root = Path(current["source"]["root"])
    assert_no_symlink_components(root, leaf_may_be_absent=False)
    expected_paths: set[str] = set()
    for entry in source_entries(current):
        raw = str(entry["path"])
        prefix = "src/source/"
        if not raw.startswith(prefix):
            raise CampaignError(f"source entry outside prefix: {raw}")
        relative = raw[len(prefix):]
        if relative in expected_paths or relative.startswith("/") or ".." in Path(relative).parts:
            raise CampaignError(f"unsafe or duplicate source path: {relative}")
        expected_paths.add(relative)
        path = root / relative
        info = path.lstat()
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise CampaignError(f"source is not a single-link regular file: {path}")
        if info.st_size != entry["size_bytes"] or sha256_file(path) != entry["sha256"]:
            raise CampaignError(f"source content drift: {path}")
        if info.st_mode & 0o111:
            raise CampaignError(f"sealed source gained execute bit: {path}")
        descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
        try:
            if os.read(descriptor, 4) == b"\x7fELF":
                raise CampaignError(f"ELF found in sealed source: {path}")
        finally:
            os.close(descriptor)
    actual_paths = {
        str(path.relative_to(root))
        for path in root.rglob("*")
        if path.is_file()
    }
    if actual_paths != expected_paths:
        raise CampaignError(
            f"sealed source inventory drift missing={sorted(expected_paths-actual_paths)} extra={sorted(actual_paths-expected_paths)}"
        )
    makefile = root / "Makefile"
    if sha256_file(makefile) != current["source"]["makefile_sha256"]:
        raise CampaignError("Makefile hash drift")
    return {"file_count": len(expected_paths), "total_bytes": current["source"]["total_bytes"]}


def expected_source_map(current: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for entry in source_entries(current):
        relative = str(entry["path"])[len("src/source/"):]
        result[relative] = {"size_bytes": int(entry["size_bytes"]), "sha256": str(entry["sha256"])}
    return result


def validate_build_input(current: Mapping[str, Any]) -> dict[str, Any]:
    root = Path(current["paths"]["build_source_root"])
    expected = expected_source_map(current)
    wrapper = current["wrapper"]
    expected[str(wrapper["relative_path"])] = {
        "size_bytes": int(wrapper["size_bytes"]),
        "sha256": str(wrapper["sha256"]),
    }
    actual_files: dict[str, Path] = {}
    for path in root.rglob("*"):
        info = path.lstat()
        if stat.S_ISLNK(info.st_mode):
            raise CampaignError(f"symlink in build input: {path}")
        if stat.S_ISREG(info.st_mode):
            relative = str(path.relative_to(root))
            actual_files[relative] = path
            if info.st_nlink != 1 or info.st_mode & 0o111:
                raise CampaignError(f"unsafe build input file: {path}")
        elif not stat.S_ISDIR(info.st_mode):
            raise CampaignError(f"special build input entry: {path}")
    if set(actual_files) != set(expected):
        raise CampaignError(
            f"build input inventory drift missing={sorted(set(expected)-set(actual_files))} extra={sorted(set(actual_files)-set(expected))}"
        )
    total = 0
    for relative, contract in expected.items():
        path = actual_files[relative]
        info = path.lstat()
        if info.st_size != contract["size_bytes"] or sha256_file(path) != contract["sha256"]:
            raise CampaignError(f"build input content drift: {path}")
        total += info.st_size
    wrapper_path = root / str(wrapper["relative_path"])
    if stat.S_IMODE(wrapper_path.lstat().st_mode) != int(wrapper["mode_octal"], 8):
        raise CampaignError("wrapper mode drift")
    return {"file_count": len(actual_files), "total_bytes": total}


def mem_available_bytes() -> int:
    for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) * 1024
    raise CampaignError("MemAvailable missing")


def swap_free_bytes() -> int:
    for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
        if line.startswith("SwapFree:"):
            return int(line.split()[1]) * 1024
    raise CampaignError("SwapFree missing")


def verify_parents_and_tools(current: Mapping[str, Any]) -> dict[str, Any]:
    parent = current["parent"]
    for key in ("plan", "g1_policy", "remediation_template"):
        path = workspace_path(parent[f"{key}_path"])
        if sha256_file(path) != parent[f"{key}_sha256"]:
            raise CampaignError(f"parent hash drift: {key}")
    profile_path = workspace_path(current["profile"]["path"])
    if sha256_file(profile_path) != current["profile"]["sha256"]:
        raise CampaignError("build profile hash drift")
    g1 = load_g1()
    g1_policy = read_json(workspace_path(parent["g1_policy_path"]))
    tools = g1.verify_tool_identities(g1_policy)
    obj = g1.static_make_object_contract(Path(current["source"]["root"]) / "Makefile")
    expected_obj = current["object_contract"]
    if (
        obj["total_object_count"] != expected_obj["total"]
        or obj["cpp_object_count"] != expected_obj["cpp"]
        or obj["cuda_object_count"] != expected_obj["cuda"]
        or obj["object_names_canonical_sha256"] != expected_obj["canonical_sha256"]
    ):
        raise CampaignError("object contract drift")
    if current["build"]["parallel_jobs"] != 1 or "-j1" not in current["build"]["make_argv"]:
        raise CampaignError("parallelism is not literal -j1")
    make_joined = " ".join(current["build"]["make_argv"])
    required = ("compute_120", "sm_120", "g++-11", "CUDA=12", "DIRTOOLKIT=/usr/local/cuda-12.8")
    forbidden = ("g++-13", "sm_89", "compute_89", "-j2", "-j4", "fast_math=YES")
    if any(item not in make_joined for item in required) or any(item in make_joined for item in forbidden):
        raise CampaignError("Make argv architecture/toolchain drift")
    return {"tool_count": len(tools), "object_contract": obj}


def common_self_check(*, require_fresh_root: bool) -> dict[str, Any]:
    current = policy()
    if os.getuid() != 1000 or os.getgid() != 1000:
        raise CampaignError("gate must be invoked by uid/gid 1000")
    parent = verify_parents_and_tools(current)
    sealed = validate_sealed_source(current)
    root = Path(current["paths"]["attempt_root"])
    if require_fresh_root and root.exists():
        raise CampaignError("fresh attempt root already exists")
    if not require_fresh_root:
        assert_no_symlink_components(root, leaf_may_be_absent=False)
    available = mem_available_bytes()
    if available < current["build"]["minimum_available_memory_bytes"]:
        raise CampaignError(f"MemAvailable below 4 GiB: {available}")
    return {
        "status": "PASS_FRESH_GPU_CAMPAIGN_SELF_CHECK",
        "captured_at": utc_now(),
        "campaign_id": current["campaign_id"],
        "build_id": current["build_id"],
        "attempt_root": str(root),
        "sealed_source": sealed,
        "object_contract": parent["object_contract"],
        "tool_count": parent["tool_count"],
        "mem_available_bytes": available,
        "swap_free_bytes": swap_free_bytes(),
        "parallel_jobs": 1,
        "system_action_used": False,
        "build_started": False,
    }


def mkdir_new(path: Path, mode: int) -> None:
    os.mkdir(path, mode)
    os.chmod(path, mode, follow_symlinks=False)


def copy_regular_new(source: Path, target: Path, *, mode: int = 0o600) -> None:
    source_fd = os.open(source, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        source_info = os.fstat(source_fd)
        if not stat.S_ISREG(source_info.st_mode) or source_info.st_nlink != 1:
            raise CampaignError(f"unsafe source during copy: {source}")
        target_fd = os.open(
            target,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
            mode,
        )
        try:
            while True:
                block = os.read(source_fd, 1024 * 1024)
                if not block:
                    break
                write_all(target_fd, block)
            os.fsync(target_fd)
        finally:
            os.close(target_fd)
    finally:
        os.close(source_fd)
    os.chmod(target, mode, follow_symlinks=False)


def prepare() -> dict[str, Any]:
    result = common_self_check(require_fresh_root=True)
    current = policy()
    root = Path(current["paths"]["attempt_root"])
    output = Path(current["paths"]["output_root"])
    source_out = Path(current["paths"]["build_source_root"])
    artifacts = Path(current["paths"]["artifact_root"])
    assert_no_symlink_components(root.parent, leaf_may_be_absent=False)
    mkdir_new(root, 0o750)
    mkdir_new(output, 0o700)
    mkdir_new(output / "buildtree", 0o700)
    mkdir_new(output / "buildtree/src", 0o700)
    mkdir_new(source_out, 0o700)
    mkdir_new(artifacts, 0o700)

    source_root = Path(current["source"]["root"])
    for relative, contract in sorted(expected_source_map(current).items()):
        target = source_out / relative
        components = Path(relative).parts[:-1]
        directory = source_out
        for component in components:
            directory = directory / component
            if not directory.exists():
                mkdir_new(directory, 0o700)
        copy_regular_new(source_root / relative, target)
        if target.lstat().st_size != contract["size_bytes"] or sha256_file(target) != contract["sha256"]:
            raise CampaignError(f"post-copy mismatch: {relative}")

    wrapper = current["wrapper"]
    write_new(
        source_out / wrapper["relative_path"],
        wrapper["content_utf8"].encode("utf-8"),
        mode=int(wrapper["mode_octal"], 8),
    )
    inventory = validate_build_input(current)
    receipt = {
        **result,
        "status": "PASS_FRESH_GPU_SOURCE_COPY_AND_WRAPPER",
        "completed_at": utc_now(),
        "inventory": inventory,
        "wrapper_sha256": wrapper["sha256"],
        "source_copied": True,
        "build_started": False,
        "system_action_used": False,
    }
    write_json_new(Path(current["receipts"]["prepare"]), receipt)
    return receipt


def conflicting_build_processes() -> list[dict[str, Any]]:
    names = {"make", "nvcc", "ptxas", "cicc", "cc1plus", "collect2", "as", "ld", "ld.bfd"}
    result: list[dict[str, Any]] = []
    ancestors: set[int] = {os.getpid(), os.getppid()}
    for child in Path("/proc").iterdir():
        if not child.name.isdigit():
            continue
        pid = int(child.name)
        if pid in ancestors:
            continue
        try:
            comm = (child / "comm").read_text(encoding="utf-8").strip()
            cmdline = (child / "cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8", "replace")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if comm in names or any(f"/{name} " in cmdline for name in names):
            result.append({"pid": pid, "comm": comm, "cmdline": cmdline[:512]})
    return result


def build_child_argv(current: Mapping[str, Any]) -> list[str]:
    profile = current["profile"]["name"]
    output = current["paths"]["output_root"]
    build = current["build"]
    env_items = [f"{key}={value}" for key, value in build["environment"].items()]
    return [
        "/usr/bin/timeout", "--foreground", f"--kill-after={build['kill_after_seconds']}s", f"{build['wall_timeout_seconds']}s",
        "/usr/bin/aa-exec", "-p", profile, "--",
        "/usr/bin/bwrap",
        "--die-with-parent", "--new-session", "--clearenv",
        "--unshare-user", "--unshare-pid", "--unshare-net", "--unshare-ipc", "--unshare-uts",
        "--disable-userns", "--assert-userns-disabled",
        "--ro-bind", "/usr", "/usr",
        "--symlink", "usr/lib", "/lib",
        "--symlink", "usr/lib64", "/lib64",
        "--tmpfs", "/work",
        "--dir", "/work/tmp",
        "--dir", "/work/output",
        "--bind", output, "/work/output",
        "--chdir", "/work/output/buildtree/src/source",
        "/usr/bin/env", "-i", *env_items,
        "/usr/bin/prlimit",
        f"--cpu={build['cpu_limit_seconds']}",
        f"--as={build['address_space_limit_bytes']}",
        f"--fsize={build['file_size_limit_bytes']}",
        "--nproc=4096", "--nofile=1024", "--core=0", "--",
        *build["make_argv"],
    ]


def run_command(argv: Sequence[str], *, timeout_seconds: int = 30) -> dict[str, Any]:
    started = time.monotonic()
    completed = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout_seconds, check=False)
    return {
        "argv": list(argv),
        "returncode": completed.returncode,
        "elapsed_seconds": round(time.monotonic() - started, 6),
        "stdout": completed.stdout.decode("utf-8", "replace")[-16384:],
        "stderr": completed.stderr.decode("utf-8", "replace")[-16384:],
    }


def profile_is_loaded(name: str) -> bool:
    result = run_command(["/usr/sbin/aa-status"], timeout_seconds=30)
    if result["returncode"] != 0:
        raise CampaignError(f"aa-status failed: {result['stderr']}")
    return any(line.strip() == name for line in result["stdout"].splitlines())


def resource_sample(started: float) -> dict[str, Any]:
    sample: dict[str, Any] = {
        "captured_at": utc_now(),
        "elapsed_seconds": round(time.monotonic() - started, 3),
        "mem_available_bytes": mem_available_bytes(),
        "swap_free_bytes": swap_free_bytes(),
        "memory_psi": Path("/proc/pressure/memory").read_text(encoding="ascii").strip(),
    }
    gpu = subprocess.run(
        [
            "/usr/bin/nvidia-smi",
            "--query-gpu=memory.used,memory.free,temperature.gpu,power.draw",
            "--format=csv,noheader,nounits",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=10,
        check=False,
    )
    sample["nvidia_smi_returncode"] = gpu.returncode
    sample["nvidia_smi"] = gpu.stdout.decode("utf-8", "replace").strip()
    rss: list[dict[str, Any]] = []
    for child in Path("/proc").iterdir():
        if not child.name.isdigit():
            continue
        try:
            comm = (child / "comm").read_text(encoding="utf-8").strip()
            if comm not in {"make", "nvcc", "ptxas", "cicc", "cc1plus", "collect2", "as", "ld", "ld.bfd"}:
                continue
            status_text = (child / "status").read_text(encoding="utf-8")
            rss_line = next((line for line in status_text.splitlines() if line.startswith("VmRSS:")), "VmRSS: 0 kB")
            rss.append({"pid": int(child.name), "comm": comm, "rss_bytes": int(rss_line.split()[1]) * 1024})
        except (FileNotFoundError, PermissionError, ProcessLookupError, StopIteration, ValueError):
            continue
    sample["build_processes"] = sorted(rss, key=lambda item: item["pid"])
    return sample


def monitor_resources(path: Path, stop: threading.Event, started: float, interval: int) -> None:
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
    try:
        while True:
            payload = canonical_json(resource_sample(started)) + b"\n"
            write_all(descriptor, payload)
            os.fsync(descriptor)
            if stop.wait(interval):
                break
    finally:
        os.close(descriptor)


def candidate_and_objects(current: Mapping[str, Any]) -> dict[str, Any] | None:
    candidate = Path(current["paths"]["candidate"])
    if not candidate.exists():
        return None
    descriptor = os.open(candidate, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_size <= 0:
            raise CampaignError("candidate is not a non-empty single-link regular file")
        candidate_hash = sha256_fd(descriptor)
        os.fchmod(descriptor, 0o400)
        final_info = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    g1 = load_g1()
    object_contract = g1.static_make_object_contract(Path(current["source"]["root"]) / "Makefile")
    source_root = Path(current["paths"]["build_source_root"])
    expected_names = set(object_contract["object_names"])
    actual_names = {path.name for path in source_root.glob("*.o")}
    if actual_names != expected_names:
        raise CampaignError(
            f"object inventory mismatch missing={sorted(expected_names-actual_names)} extra={sorted(actual_names-expected_names)}"
        )
    objects: list[dict[str, Any]] = []
    for name in sorted(expected_names):
        path = source_root / name
        descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
        try:
            info = os.fstat(descriptor)
            header = os.read(descriptor, 20)
            if (
                not stat.S_ISREG(info.st_mode)
                or info.st_nlink != 1
                or info.st_mode & 0o111
                or len(header) < 20
                or header[:4] != b"\x7fELF"
                or header[4] != 2
                or header[5] != 1
                or int.from_bytes(header[16:18], "little") != 1
                or int.from_bytes(header[18:20], "little") != 62
            ):
                raise CampaignError(f"invalid ET_REL object: {path}")
            digest = sha256_fd(descriptor)
        finally:
            os.close(descriptor)
        objects.append({"name": name, "size_bytes": info.st_size, "sha256": digest})
    return {
        "candidate": {
            "path": str(candidate),
            "inode": final_info.st_ino,
            "mode_octal": f"{stat.S_IMODE(final_info.st_mode):04o}",
            "size_bytes": final_info.st_size,
            "sha256": candidate_hash,
        },
        "objects": {
            "count": len(objects),
            "inventory_sha256": sha256_bytes(canonical_json(objects)),
            "entries": objects,
        },
    }


def build() -> dict[str, Any]:
    current = policy()
    if os.getgroups():
        raise CampaignError(f"build gate requires zero supplementary groups, got {os.getgroups()}")
    common_self_check(require_fresh_root=False)
    validate_build_input(current)
    prepare_receipt = read_json(Path(current["receipts"]["prepare"]))
    if prepare_receipt.get("status") != "PASS_FRESH_GPU_SOURCE_COPY_AND_WRAPPER":
        raise CampaignError("prepare receipt does not admit build")
    candidate = Path(current["paths"]["candidate"])
    if candidate.exists():
        raise CampaignError("candidate exists before build")
    conflicts = conflicting_build_processes()
    if conflicts:
        raise CampaignError(f"conflicting build processes: {conflicts}")
    for key in ("build_start", "build_stdout", "build_stderr", "build_resource", "build_final"):
        if Path(current["receipts"][key]).exists():
            raise CampaignError(f"create-new build output already exists: {key}")

    child_argv = build_child_argv(current)
    start_receipt = {
        "document_type": "r8-liquid-fresh-gpu-build-start-v1",
        "status": "STARTED_FRESH_GPU_BUILD",
        "campaign_id": current["campaign_id"],
        "build_id": current["build_id"],
        "created_at": utc_now(),
        "attempt_root": current["paths"]["attempt_root"],
        "profile_name": current["profile"]["name"],
        "profile_sha256": current["profile"]["sha256"],
        "parallel_jobs": 1,
        "mem_available_bytes": mem_available_bytes(),
        "swap_free_bytes": swap_free_bytes(),
        "child_argv": child_argv,
        "make_argv": current["build"]["make_argv"],
        "network_disabled": True,
        "gpu_device_exposed": False,
    }
    write_json_new(Path(current["receipts"]["build_start"]), start_receipt)

    if os.environ.get("R8_EXACT_PROFILE_PRELOADED_SHA256") != current["profile"]["sha256"]:
        raise CampaignError("exact preloaded-profile supervisor token missing or drifted")
    lifecycle: list[dict[str, Any]] = [
        {"action": "verify_preloaded", "returncode": 0, "loaded": True}
    ]

    stop = threading.Event()
    started = time.monotonic()
    monitor_thread: threading.Thread | None = None
    returncode: int | None = None
    stdout_path = Path(current["receipts"]["build_stdout"])
    stderr_path = Path(current["receipts"]["build_stderr"])
    try:
        monitor_thread = threading.Thread(
            target=monitor_resources,
            args=(Path(current["receipts"]["build_resource"]), stop, started, current["build"]["monitor_interval_seconds"]),
            daemon=True,
        )
        monitor_thread.start()
        stdout_file = open(stdout_path, "xb", buffering=0)
        stderr_file = open(stderr_path, "xb", buffering=0)
        try:
            process = subprocess.Popen(
                child_argv,
                stdin=subprocess.DEVNULL,
                stdout=stdout_file,
                stderr=stderr_file,
                start_new_session=True,
            )
            returncode = process.wait()
        finally:
            stdout_file.close()
            stderr_file.close()
    finally:
        stop.set()
        if monitor_thread is not None:
            monitor_thread.join(timeout=30)

    elapsed = time.monotonic() - started
    artifact = candidate_and_objects(current)
    final = {
        "document_type": "r8-liquid-fresh-gpu-build-final-v1",
        "status": "PASS_GPU_BUILD_CANDIDATE_DISARMED" if returncode == 0 and artifact else "FAIL_GPU_BUILD",
        "campaign_id": current["campaign_id"],
        "build_id": current["build_id"],
        "completed_at": utc_now(),
        "elapsed_seconds": round(elapsed, 6),
        "returncode": returncode,
        "parallel_jobs": 1,
        "profile_lifecycle": lifecycle,
        "stdout": {
            "path": str(stdout_path),
            "size_bytes": stdout_path.lstat().st_size,
            "sha256": sha256_file(stdout_path),
        },
        "stderr": {
            "path": str(stderr_path),
            "size_bytes": stderr_path.lstat().st_size,
            "sha256": sha256_file(stderr_path),
        },
        "resource_log": {
            "path": current["receipts"]["build_resource"],
            "size_bytes": Path(current["receipts"]["build_resource"]).lstat().st_size,
            "sha256": sha256_file(Path(current["receipts"]["build_resource"])),
        },
        "artifact": artifact,
        "make_count": 1,
        "network_used": False,
        "gpu_device_exposed": False,
        "candidate_executed": False,
        "profile_loaded_after_child": True,
        "project_workspace_written_by_build": False,
    }
    write_json_new(Path(current["receipts"]["build_final"]), final)
    if final["status"] != "PASS_GPU_BUILD_CANDIDATE_DISARMED":
        raise CampaignError(f"GPU build failed rc={returncode}; evidence={current['receipts']['build_final']}")
    return final


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("self-check", "prepare", "verify-input", "build"))
    args = parser.parse_args(argv)
    try:
        if args.phase == "self-check":
            result = common_self_check(require_fresh_root=True)
        elif args.phase == "prepare":
            result = prepare()
        elif args.phase == "verify-input":
            current = policy()
            common_self_check(require_fresh_root=False)
            result = {
                "status": "PASS_FRESH_GPU_BUILD_INPUT",
                "captured_at": utc_now(),
                "inventory": validate_build_input(current),
                "build_argv": build_child_argv(current),
            }
        elif args.phase == "build":
            result = build()
        else:
            raise AssertionError("unreachable")
    except (CampaignError, OSError, KeyError, ValueError, json.JSONDecodeError, subprocess.TimeoutExpired) as exc:
        print(json.dumps({"status": "FAIL", "phase": args.phase, "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 1
    print(json.dumps(result, sort_keys=True, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
