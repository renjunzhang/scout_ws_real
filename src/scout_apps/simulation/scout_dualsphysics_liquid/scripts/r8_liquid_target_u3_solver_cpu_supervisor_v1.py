#!/usr/bin/env python3
"""Root lifecycle supervisor for one fresh U3 C1 CPU-solver smoke v1.

Workspace commands are read-only reviewers and snapshot-byte producers.  The
hidden production ``run`` entry works only from a root-owned immutable snapshot.
It loads two fresh transient AppArmor labels, hands a fixed stdin frame to the
UID-1000/NNP gate, closes a zero-denial journal window, removes all labelled
tasks and both profiles, and only then permits a separate UID-1000/NNP process
to export the exact 29-file solver tree with dirfd/O_NOFOLLOW/O_EXCL.
"""

from __future__ import annotations

import argparse
import errno
import hashlib
import importlib.util
import json
import os
import signal
import stat
import struct
import subprocess
import sys
import time
from pathlib import Path, PurePosixPath
from typing import Any, Mapping


SCRIPT_PATH = Path(__file__).resolve()
WORKSPACE_PACKAGE_ROOT = Path("/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid")
WORKSPACE_SUPERVISOR_PATH = WORKSPACE_PACKAGE_ROOT / "scripts/r8_liquid_target_u3_solver_cpu_supervisor_v1.py"
CASE_ID = "u3_c1_solver_cpu_smoke_v1_20260808T124816Z"
HOST_UID = 1000
HOST_GID = 1000
BOOTSTRAP_PROFILE = "r8-liquid-u3-solver-cpu-bootstrap-v1-20260808t124816z"
RUNTIME_PROFILE = "r8-liquid-u3-solver-cpu-runtime-v1-20260808t124816z"
LABELS = (BOOTSTRAP_PROFILE, RUNTIME_PROFILE)
SNAPSHOT_ROOT = Path(f"/run/r8-liquid-{CASE_ID}.snapshot")
SNAPSHOT_ENV = "R8_LIQUID_U3_SOLVER_CPU_V1_SNAPSHOT"
ADMISSION_FD_ENV = "R8_LIQUID_U3_SOLVER_CPU_V1_ADMISSION_FD"
ADMISSION_SHA256_ENV = "R8_LIQUID_U3_SOLVER_CPU_V1_ADMISSION_SHA256"
EXPORT_FD_ENV = "R8_LIQUID_U3_SOLVER_CPU_V1_EXPORT_FD"
EXPORT_SHA256_ENV = "R8_LIQUID_U3_SOLVER_CPU_V1_EXPORT_SHA256"
ADMISSION_TOKEN = f"{CASE_ID}:single-cpu-solver-smoke-attempt"

POLICY_NAME = "liquid_zrj_msi_u2404_u3_solver_cpu_execution_policy_v1.json"
SCHEMA_NAME = "target_host_u3_solver_cpu_execution_policy_v1.json"
PROFILE_NAME = "r8-liquid-u3-solver-cpu-v1.profile"
GATE_NAME = "r8_liquid_target_u3_solver_cpu_gate_v1.py"
HELPER_NAME = "r8_liquid_u3_solver_cpu_bootstrap_helper_v1.py"
SUPERVISOR_NAME = "r8_liquid_target_u3_solver_cpu_supervisor_v1.py"
HARNESS_NAME = "r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v11.py"

IN_SNAPSHOT = SCRIPT_PATH.parent == SNAPSHOT_ROOT
BASE = SNAPSHOT_ROOT if IN_SNAPSHOT else WORKSPACE_PACKAGE_ROOT
POLICY_PATH = BASE / POLICY_NAME if IN_SNAPSHOT else BASE / "config/target_hosts" / POLICY_NAME
SCHEMA_PATH = BASE / SCHEMA_NAME if IN_SNAPSHOT else BASE / "schema" / SCHEMA_NAME
PROFILE_PATH = BASE / PROFILE_NAME if IN_SNAPSHOT else BASE / "config/apparmor_drafts" / PROFILE_NAME
GATE_PATH = BASE / GATE_NAME if IN_SNAPSHOT else BASE / "scripts" / GATE_NAME
HELPER_PATH = BASE / HELPER_NAME if IN_SNAPSHOT else BASE / "scripts" / HELPER_NAME
HARNESS_PATH = BASE / HARNESS_NAME if IN_SNAPSHOT else BASE / "scripts" / HARNESS_NAME

ATTEMPT_ROOT = Path(f"/home/zrj/scout_liquid_lab/cases/{CASE_ID}.partial")
OUTPUT_ROOT = ATTEMPT_ROOT / "output"
START_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.start.json")
EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.execution.json")
LIFECYCLE_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.lifecycle.json")
LIFECYCLE_FAILURE_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.lifecycle_incomplete.json")
CONSOLE_LOG = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.console.log")

BUILD_RECEIPT = Path("/home/zrj/scout_liquid_lab/audits/u3_source_cpu_build_20260807T023724Z_cpu_build.json")
BUILD_RECEIPT_SHA256 = "d407233107d4bc9eeea81b0c5a95cbfc98ad6529f1cb7494b68ae5ecc4b1d604"
GENCASE_EXECUTION_RECEIPT = Path("/home/zrj/scout_liquid_lab/audits/u3_c1_gencase_v6_20260808T072315Z.execution.json")
GENCASE_EXECUTION_SHA256 = "22b643acde8a0202f00679ed37dc2d2db10faab4697c2ad5d02923653f6d3a1f"
GENCASE_LIFECYCLE_RECEIPT = Path("/home/zrj/scout_liquid_lab/audits/u3_c1_gencase_v6_20260808T072315Z.lifecycle.json")
GENCASE_LIFECYCLE_SHA256 = "f3491918db8bdc6385f3c3dc9980712a39aa1362ed0473daf87f3ec6d62d760f"

INPUT_SOURCES: tuple[tuple[str, str, str, int], ...] = (
    (
        "DualSPHysics5.4CPU_linux64",
        "/home/zrj/scout_liquid_lab/build/u3_source_cpu_build_20260807T023724Z.partial/output/artifacts/DualSPHysics5.4CPU_linux64",
        "5aa464a8f37b0185bac863987f0d1079a0f1a3d6daead6581562c832278ea202",
        32_649_520,
    ),
    (
        "DsphConfig.xml",
        "/home/zrj/scout_liquid_lab/dependency/runtime/u3_c1_gencase_seed_v2_20260807T135341Z.partial/input/DsphConfig.xml",
        "0644c9a6a6687678950fc8966e352b4bbd3de9d3cb787db9e507c2eb7ccaddcd",
        293,
    ),
    (
        "C1_static.xml",
        "/home/zrj/scout_liquid_lab/cases/u3_c1_gencase_v6_20260808T072315Z.partial/output/C1_static.xml",
        "4b644589a323132d65969d919108680808ac88b48f2166dd5ec590e575f6fd14",
        7_284,
    ),
    (
        "C1_static.bi4",
        "/home/zrj/scout_liquid_lab/cases/u3_c1_gencase_v6_20260808T072315Z.partial/output/C1_static.bi4",
        "f87526120631055b8548566c7d5614b22c96fa1da0e9beba150788740f20d6ea",
        400_843,
    ),
    (
        "ld-linux-x86-64.so.2",
        "/usr/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2",
        "cd4df4f3c7b83673d61189bf2eaebd33ca4f2853ab9772b8a25e025ef99b1e81",
        236_616,
    ),
    (
        "libgomp.so.1",
        "/usr/lib/x86_64-linux-gnu/libgomp.so.1.0.0",
        "135f3c8f006d2fe5e68e51281c7974cb991a03de3bfb3593d68d174dfcf854d1",
        352_304,
    ),
    (
        "libstdc++.so.6",
        "/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.33",
        "1fd75fe70354a416d75aef22bcae68c47bd25d20e2d0568c30b1a9838cf62f11",
        2_592_224,
    ),
    (
        "libm.so.6",
        "/usr/lib/x86_64-linux-gnu/libm.so.6",
        "e9c4b28d340e415b8137480ec442662f981e1399386c5931dae0e886e3639e91",
        952_616,
    ),
    (
        "libgcc_s.so.1",
        "/usr/lib/x86_64-linux-gnu/libgcc_s.so.1",
        "d93224d2b0dab4247598be683adca02f5cf00586f99c187579cd7e92058fb7cb",
        183_024,
    ),
    (
        "libc.so.6",
        "/usr/lib/x86_64-linux-gnu/libc.so.6",
        "8db37cf3f2169f59a0f07ef1fea308c35656668c64c8ff294e1860f4121eb161",
        2_125_328,
    ),
)

EXPECTED_ROOT_FILES = (
    "CfgInit_Domain.vtk",
    "CfgInit_MapCells.vtk",
    "Run.csv",
    "Run.out",
    "RunPARTs.csv",
)
EXPECTED_DATA_FILES = (
    "PartInfo.ibi4",
    "PartOut_000.obi4",
    "Part_Head.ibi4",
    *(f"Part_{index:04d}.bi4" for index in range(21)),
)
EXPECTED_PATHS = tuple(sorted(EXPECTED_ROOT_FILES + tuple(f"data/{name}" for name in EXPECTED_DATA_FILES)))

OUTPUT_MAGIC = b"R8SOLVEROUTV1\0\0\0"
OUTPUT_VERSION = 1
OUTPUT_HEADER = struct.Struct(">16sIIQQQ")
OUTPUT_METADATA_LIMIT = 262_144
OUTPUT_TOTAL_LIMIT = 67_108_864
OUTPUT_CONSOLE_LIMIT = 4_194_304
OUTPUT_FRAME_LIMIT = OUTPUT_HEADER.size + OUTPUT_METADATA_LIMIT + OUTPUT_TOTAL_LIMIT + OUTPUT_CONSOLE_LIMIT

V11_HARNESS_SHA256 = "72b7f1bf287b4fe750b46dccdb7a1eae9379230c7bdc30dbc6a6cb33a403ef5b"
V11_HARNESS_SIZE = 159_126
SYSCTL_PATHS = (
    Path("/proc/sys/user/max_user_namespaces"),
    Path("/proc/sys/kernel/unprivileged_userns_clone"),
    Path("/proc/sys/kernel/apparmor_restrict_unprivileged_userns"),
)

SNAPSHOT_SOURCE_PAIRS = (
    (str(WORKSPACE_PACKAGE_ROOT / "scripts" / GATE_NAME), GATE_NAME),
    (str(WORKSPACE_PACKAGE_ROOT / "scripts" / HELPER_NAME), HELPER_NAME),
    (str(WORKSPACE_PACKAGE_ROOT / "scripts" / SUPERVISOR_NAME), SUPERVISOR_NAME),
    (str(WORKSPACE_PACKAGE_ROOT / "config/apparmor_drafts" / PROFILE_NAME), PROFILE_NAME),
    (str(WORKSPACE_PACKAGE_ROOT / "schema" / SCHEMA_NAME), SCHEMA_NAME),
    (str(WORKSPACE_PACKAGE_ROOT / "config/target_hosts" / POLICY_NAME), POLICY_NAME),
    (str(WORKSPACE_PACKAGE_ROOT / "scripts" / HARNESS_NAME), HARNESS_NAME),
    *((source, name) for name, source, _digest, _size in INPUT_SOURCES),
)

SNAPSHOT_BOOTSTRAP_SOURCE = f'''import hashlib,json,os,stat,sys
ROOT="/run"
NAME="r8-liquid-{CASE_ID}.snapshot"
SOURCES={SNAPSHOT_SOURCE_PAIRS!r}
def open_abs(path):
 parts=[p for p in path.split("/") if p]
 d=os.open("/",os.O_RDONLY|os.O_DIRECTORY|os.O_CLOEXEC)
 try:
  for part in parts[:-1]:
   n=os.open(part,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW|os.O_CLOEXEC,dir_fd=d)
   os.close(d); d=n
  return os.open(parts[-1],os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC,dir_fd=d)
 finally: os.close(d)
def read_source(path,size,digest):
 f=open_abs(path)
 try:
  m=os.fstat(f)
  if not stat.S_ISREG(m.st_mode) or m.st_nlink!=1 or m.st_size!=size: raise SystemExit(71)
  b=bytearray()
  while len(b)<size:
   q=os.read(f,min(1048576,size-len(b)))
   if not q: raise SystemExit(72)
   b.extend(q)
  if os.read(f,1)!=b"" or hashlib.sha256(b).hexdigest()!=digest: raise SystemExit(73)
  return bytes(b)
 finally: os.close(f)
if os.geteuid()!=0 or len(sys.argv)!=1+2*len(SOURCES): raise SystemExit(70)
payloads=[]
for i,(source,name) in enumerate(SOURCES):
 digest=sys.argv[1+2*i]; size=int(sys.argv[2+2*i])
 if len(digest)!=64 or size<1 or size>83886080: raise SystemExit(74)
 payloads.append((name,read_source(source,size,digest),digest,size))
runfd=os.open(ROOT,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW|os.O_CLOEXEC)
try:
 os.mkdir(NAME,0o700,dir_fd=runfd)
 snapfd=os.open(NAME,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW|os.O_CLOEXEC,dir_fd=runfd)
 try:
  manifest=[]
  for name,raw,digest,size in payloads:
   f=os.open(name,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW|os.O_CLOEXEC,0o400,dir_fd=snapfd)
   try:
    view=memoryview(raw)
    while view:
     count=os.write(f,view)
     if count<=0: raise SystemExit(75)
     view=view[count:]
    os.fsync(f); os.fchmod(f,0o444); os.fsync(f); m=os.fstat(f)
    if m.st_uid!=0 or m.st_gid!=0 or m.st_nlink!=1 or stat.S_IMODE(m.st_mode)!=0o444: raise SystemExit(76)
   finally: os.close(f)
   manifest.append({{"name":name,"sha256":digest,"size_bytes":size}})
  os.fchmod(snapfd,0o555); os.fsync(snapfd)
 finally: os.close(snapfd)
 os.fsync(runfd)
finally: os.close(runfd)
os.write(1,(json.dumps({{"status":"ROOT_SOLVER_CPU_V1_SNAPSHOT_CREATED_NOT_EXECUTED","files":manifest}},sort_keys=True,separators=(",",":"))+"\\n").encode("ascii"))
'''
SNAPSHOT_BOOTSTRAP_BYTES = SNAPSHOT_BOOTSTRAP_SOURCE.encode("ascii")
SNAPSHOT_BOOTSTRAP_SHA256 = hashlib.sha256(SNAPSHOT_BOOTSTRAP_BYTES).hexdigest()
SNAPSHOT_BOOTSTRAP_LOADER_SOURCE = f'''import hashlib,sys
B=sys.stdin.buffer
n={len(SNAPSHOT_BOOTSTRAP_BYTES)}
p=bytearray()
while len(p)<n:
 q=B.read(n-len(p))
 if not q: raise SystemExit(81)
 p.extend(q)
p=bytes(p)
if B.read(1)!=b"" or hashlib.sha256(p).hexdigest()!="{SNAPSHOT_BOOTSTRAP_SHA256}": raise SystemExit(82)
g={{"__name__":"__main__"}}
exec(compile(p,"<r8-liquid-solver-cpu-v1-root-snapshot-bootstrap>","exec",dont_inherit=True,optimize=2),g,g)
'''
SNAPSHOT_BOOTSTRAP_LOADER_BYTES = SNAPSHOT_BOOTSTRAP_LOADER_SOURCE.encode("ascii")
SNAPSHOT_BOOTSTRAP_LOADER_SHA256 = hashlib.sha256(SNAPSHOT_BOOTSTRAP_LOADER_BYTES).hexdigest()

sys.dont_write_bytecode = True


class SupervisorError(RuntimeError):
    """A fail-closed supervisor, frame, export or lifecycle error."""


def read_regular_bytes(path: Path, *, limit: int = 80 * 1024 * 1024) -> bytes:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or not 0 < metadata.st_size <= limit:
            raise SupervisorError(f"unsafe regular file: {path}")
        result = bytearray()
        while len(result) < metadata.st_size:
            block = os.read(descriptor, min(1 << 20, metadata.st_size - len(result)))
            if not block:
                raise SupervisorError(f"short read: {path}")
            result.extend(block)
        if os.read(descriptor, 1) != b"":
            raise SupervisorError(f"file grew during read: {path}")
        return bytes(result)
    finally:
        os.close(descriptor)


def read_regular_bytes_allow_empty(path: Path, *, limit: int) -> bytes:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or not 0 <= metadata.st_size <= limit:
            raise SupervisorError(f"unsafe bounded regular file: {path}")
        result = bytearray()
        while len(result) < metadata.st_size:
            block = os.read(descriptor, min(1 << 20, metadata.st_size - len(result)))
            if not block:
                raise SupervisorError(f"short bounded read: {path}")
            result.extend(block)
        if os.read(descriptor, 1) != b"":
            raise SupervisorError(f"bounded file grew during read: {path}")
        return bytes(result)
    finally:
        os.close(descriptor)


def sha256_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def read_json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(read_regular_bytes(path, limit=2 * 1024 * 1024).decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError(f"invalid JSON: {path}") from exc
    if not isinstance(value, dict):
        raise SupervisorError(f"JSON root is not an object: {path}")
    return value


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise SupervisorError(f"cannot load reviewed module: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def artifact_paths() -> dict[str, Path]:
    return {
        "gate": GATE_PATH,
        "helper": HELPER_PATH,
        "supervisor": SCRIPT_PATH,
        "profile": PROFILE_PATH,
        "schema": SCHEMA_PATH,
    }


def verify_receipt(path: Path, digest: str, status: str) -> dict[str, Any]:
    raw = read_regular_bytes(path, limit=256 * 1024)
    if sha256_bytes(raw) != digest:
        raise SupervisorError(f"frozen receipt digest differs: {path.name}")
    try:
        value = json.loads(raw.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError(f"frozen receipt is invalid: {path.name}") from exc
    if not isinstance(value, dict) or value.get("status") != status:
        raise SupervisorError(f"frozen receipt status differs: {path.name}")
    return {"path": str(path), "sha256": digest, "size_bytes": len(raw), "status": status}


def verify_provenance_and_inputs() -> tuple[dict[str, Any], dict[str, bytes]]:
    receipts = {
        "cpu_build": verify_receipt(BUILD_RECEIPT, BUILD_RECEIPT_SHA256, "PASS_U3_CPU_BUILD_STATIC_ELF_AUDIT"),
        "gencase_execution": verify_receipt(
            GENCASE_EXECUTION_RECEIPT,
            GENCASE_EXECUTION_SHA256,
            "PASS_U3_C1_GENCASE_V6_CASE_EXPORTED_CLEANUP_PENDING",
        ),
        "gencase_lifecycle": verify_receipt(
            GENCASE_LIFECYCLE_RECEIPT,
            GENCASE_LIFECYCLE_SHA256,
            "PASS_U3_C1_GENCASE_V6_LIFECYCLE_CLEANUP_AND_CASE_EXPORT",
        ),
    }
    build = read_json_object(BUILD_RECEIPT)
    artifact = build.get("artifact", {})
    if (
        artifact.get("sha256") != INPUT_SOURCES[0][2]
        or artifact.get("size_bytes") != INPUT_SOURCES[0][3]
        or artifact.get("needed") != ["libgomp.so.1", "libstdc++.so.6", "libm.so.6", "libgcc_s.so.1", "libc.so.6"]
        or artifact.get("rpath") is not False
        or artifact.get("runpath") is not False
        or artifact.get("mode_after_audit") != "0o400"
        or build.get("compiled_artifact_executed") is not False
    ):
        raise SupervisorError("CPU build receipt artifact contract differs")
    inputs: dict[str, bytes] = {}
    observed: dict[str, Any] = {}
    for name, source, digest, size in INPUT_SOURCES:
        raw = read_regular_bytes(Path(source), limit=80 * 1024 * 1024)
        if len(raw) != size or sha256_bytes(raw) != digest:
            raise SupervisorError(f"solver source input differs: {name}")
        inputs[name] = raw
        metadata = os.stat(source, follow_symlinks=False)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise SupervisorError(f"solver source input inode differs: {name}")
        observed[name] = {"source": source, "sha256": digest, "size_bytes": size}
    solver = inputs["DualSPHysics5.4CPU_linux64"]
    if solver[:4] != b"\x7fELF" or solver[4:6] != b"\x02\x01":
        raise SupervisorError("CPU solver is not ELF64 little-endian")
    if b"libcuda.so" in solver or b"libnvidia" in solver:
        raise SupervisorError("CPU solver contains a forbidden dynamic GPU dependency")
    if not inputs["C1_static.bi4"].startswith(b"#FileJBD JPartDataBi4"):
        raise SupervisorError("C1 input BI4 code differs")
    return {"receipts": receipts, "inputs": observed}, inputs


def verify_policy_static() -> dict[str, Any]:
    policy = read_json_object(POLICY_PATH)
    schema = read_json_object(SCHEMA_PATH)
    if schema.get("additionalProperties") is not False or set(policy) != set(schema.get("required", [])):
        raise SupervisorError("solver policy/schema contract differs")
    if policy.get("status") != "REVIEWED_FRESH_SINGLE_CPU_SOLVER_SMOKE_V1_PENDING_STATIC_VERIFICATION":
        raise SupervisorError("solver policy status differs")
    trusted = policy.get("trusted_artifacts", {})
    if not isinstance(trusted, dict) or set(trusted) != set(artifact_paths()):
        raise SupervisorError("solver trusted artifact set differs")
    observed_artifacts: dict[str, Any] = {}
    for name, path in artifact_paths().items():
        raw = read_regular_bytes(path, limit=4 * 1024 * 1024)
        entry = trusted[name]
        if sha256_bytes(raw) != entry.get("sha256") or len(raw) != entry.get("size_bytes"):
            raise SupervisorError(f"solver trusted artifact differs: {name}")
        observed_artifacts[name] = {"path": str(path), "sha256": sha256_bytes(raw), "size_bytes": len(raw)}
    harness_raw = read_regular_bytes(HARNESS_PATH, limit=512 * 1024)
    if sha256_bytes(harness_raw) != V11_HARNESS_SHA256 or len(harness_raw) != V11_HARNESS_SIZE:
        raise SupervisorError("frozen v11 lifecycle harness bytes differ")
    gate = load_module(GATE_PATH, "_r8_solver_cpu_v1_static_gate")
    gate_review = gate.verify_review_artifacts(verify_tools=True)
    provenance, _inputs = verify_provenance_and_inputs()
    policy_raw = read_regular_bytes(POLICY_PATH)
    snapshot_sources: list[dict[str, Any]] = []
    for source, name in SNAPSHOT_SOURCE_PAIRS:
        raw = read_regular_bytes(Path(source), limit=80 * 1024 * 1024)
        snapshot_sources.append({"source": source, "name": name, "sha256": sha256_bytes(raw), "size_bytes": len(raw)})
    return {
        "policy": {"path": str(POLICY_PATH), "sha256": sha256_bytes(policy_raw), "size_bytes": len(policy_raw)},
        "artifacts": observed_artifacts,
        "gate_review": gate_review,
        "provenance": provenance,
        "lifecycle_harness": {"path": str(HARNESS_PATH), "sha256": V11_HARNESS_SHA256, "size_bytes": V11_HARNESS_SIZE},
        "snapshot_sources": snapshot_sources,
        "production_run_allowed": False,
        "execution_performed": False,
    }


def snapshot_manifest_arguments(review: Mapping[str, Any]) -> list[str]:
    sources = review.get("snapshot_sources")
    if not isinstance(sources, list) or len(sources) != len(SNAPSHOT_SOURCE_PAIRS):
        raise SupervisorError("snapshot source manifest differs")
    arguments: list[str] = []
    for expected, observed in zip(SNAPSHOT_SOURCE_PAIRS, sources, strict=True):
        if observed.get("source") != expected[0] or observed.get("name") != expected[1]:
            raise SupervisorError("snapshot source order differs")
        arguments.extend((observed["sha256"], str(observed["size_bytes"])))
    return arguments


def verify_snapshot_producer_identity() -> dict[str, Any]:
    if SCRIPT_PATH != WORKSPACE_SUPERVISOR_PATH or os.geteuid() != HOST_UID or os.getuid() != HOST_UID:
        raise SupervisorError("snapshot producer must be the exact unprivileged supervisor")
    status = Path("/proc/self/status").read_text(encoding="ascii")
    fields = {line.split(":", 1)[0]: line.split(":", 1)[1].strip() for line in status.splitlines() if ":" in line}
    uid = [int(value) for value in fields["Uid"].split()]
    gid = [int(value) for value in fields["Gid"].split()]
    caps = [int(fields[key], 16) for key in ("CapInh", "CapPrm", "CapEff", "CapAmb")]
    if uid != [HOST_UID] * 4 or gid != [HOST_GID] * 4 or any(caps):
        raise SupervisorError("snapshot producer identity differs")
    return {"uid": uid, "gid": gid, "active_capabilities_zero": True}


def parse_success_frame(raw: bytes) -> dict[str, Any]:
    if not OUTPUT_HEADER.size <= len(raw) <= OUTPUT_FRAME_LIMIT:
        raise SupervisorError("solver success frame total length is out of bounds")
    magic, version, metadata_size, file_count, payload_size, console_size = OUTPUT_HEADER.unpack_from(raw)
    if magic != OUTPUT_MAGIC or version != OUTPUT_VERSION:
        raise SupervisorError("solver success frame magic/version differs")
    if not 1 <= metadata_size <= OUTPUT_METADATA_LIMIT or file_count != 29:
        raise SupervisorError("solver success frame metadata/count differs")
    if not 1 <= payload_size <= OUTPUT_TOTAL_LIMIT or not 0 <= console_size <= OUTPUT_CONSOLE_LIMIT:
        raise SupervisorError("solver success frame payload/console length differs")
    expected_total = OUTPUT_HEADER.size + metadata_size + payload_size + console_size
    if len(raw) != expected_total:
        raise SupervisorError("solver success frame is truncated or has trailing bytes")
    metadata_raw = raw[OUTPUT_HEADER.size : OUTPUT_HEADER.size + metadata_size]
    payload_raw = raw[OUTPUT_HEADER.size + metadata_size : OUTPUT_HEADER.size + metadata_size + payload_size]
    console = raw[-console_size:] if console_size else b""
    try:
        metadata = json.loads(metadata_raw.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError("solver frame metadata is invalid") from exc
    canonical = json.dumps(metadata, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    if not isinstance(metadata, dict) or canonical != metadata_raw:
        raise SupervisorError("solver frame metadata is not a canonical object")
    required_keys = {
        "document_type",
        "status",
        "solver_argv",
        "environment",
        "guest_inputs",
        "guest_identity",
        "guest_label",
        "candidate_label",
        "guest_work_tmpfs",
        "output_audit",
        "output_manifest",
        "console",
        "stdin_consumed_to_eof_then_replaced_by_guest_eof_pipe",
        "host_writable_bind_count",
    }
    if set(metadata) != required_keys:
        raise SupervisorError("solver frame metadata key set differs")
    if (
        metadata["document_type"] != "SMPCC_R8_LIQUID_U3_C1_CPU_SOLVER_GUEST_FRAME_V1"
        or metadata["status"] != "GUEST_SOLVER_SMOKE_COMPLETE_PENDING_HOST_REAUDIT_AND_O_EXCL_EXPORT"
        or metadata["stdin_consumed_to_eof_then_replaced_by_guest_eof_pipe"] is not True
        or metadata["host_writable_bind_count"] != 0
    ):
        raise SupervisorError("solver frame status or transport evidence differs")
    gate = load_module(GATE_PATH, "_r8_solver_cpu_v1_frame_gate")
    if metadata["solver_argv"] != gate.SOLVER_ARGV or metadata["environment"] != gate.ENVIRONMENT:
        raise SupervisorError("solver frame argv/environment differs")
    identity = metadata["guest_identity"]
    if (
        identity.get("uid") != [0, 0, 0, 0]
        or identity.get("gid") != [0, 0, 0, 0]
        or identity.get("groups") != []
        or identity.get("no_new_privs") != 1
        or set(identity.get("capabilities", {})) != {"CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb"}
        or any(identity["capabilities"].values())
    ):
        raise SupervisorError("solver frame guest identity differs")
    expected_label = BOOTSTRAP_PROFILE + " (enforce)"
    if metadata["guest_label"] != expected_label or metadata["candidate_label"] != expected_label:
        raise SupervisorError("solver frame AppArmor inheritance differs")
    tmpfs = metadata["guest_work_tmpfs"]
    if tmpfs.get("mountpoint") != "/work" or tmpfs.get("filesystem") != "tmpfs" or tmpfs.get("total_bytes") != 268_435_456:
        raise SupervisorError("solver frame tmpfs evidence differs")
    expected_inputs = {}
    helper = load_module(HELPER_PATH, "_r8_solver_cpu_v1_frame_helper")
    for name, guest_path, digest, size, mode in helper.INPUTS:
        expected_inputs[name] = {"guest_path": guest_path, "sha256": digest, "size_bytes": size, "mode": f"{mode:04o}"}
    if metadata["guest_inputs"] != expected_inputs:
        raise SupervisorError("solver frame input evidence differs")
    manifest = metadata["output_manifest"]
    if not isinstance(manifest, list) or len(manifest) != 29:
        raise SupervisorError("solver frame output manifest count differs")
    paths = [entry.get("path") for entry in manifest if isinstance(entry, dict)]
    if len(paths) != 29 or tuple(paths) != EXPECTED_PATHS:
        raise SupervisorError("solver frame output path set/order differs")
    payloads: dict[str, bytes] = {}
    offset = 0
    for entry in manifest:
        path = entry["path"]
        pure = PurePosixPath(path)
        size = entry.get("size_bytes")
        digest = entry.get("sha256")
        if pure.is_absolute() or ".." in pure.parts or len(pure.parts) not in (1, 2):
            raise SupervisorError("solver frame contains an unsafe relative path")
        if not isinstance(size, int) or not 1 <= size <= 16_777_216 or not isinstance(digest, str) or len(digest) != 64:
            raise SupervisorError("solver frame manifest entry differs")
        end = offset + size
        if end > len(payload_raw):
            raise SupervisorError("solver frame payload is truncated")
        payload = payload_raw[offset:end]
        if sha256_bytes(payload) != digest:
            raise SupervisorError(f"solver frame payload digest differs: {path}")
        payloads[path] = payload
        offset = end
    if offset != len(payload_raw):
        raise SupervisorError("solver frame payload has trailing bytes")
    if metadata["console"] != {"sha256": sha256_bytes(console), "size_bytes": len(console)}:
        raise SupervisorError("solver frame console evidence differs")
    output_audit = metadata["output_audit"]
    if output_audit.get("file_count") != 29 or output_audit.get("total_bytes") != len(payload_raw):
        raise SupervisorError("solver frame output audit differs")
    run_out = payloads["Run.out"].decode("utf-8", "strict")
    if "Finished execution (code=0)." not in run_out or "[Simulation finished" not in run_out:
        raise SupervisorError("host re-audit rejected Run.out completion")
    return {
        "metadata": metadata,
        "payloads": payloads,
        "console": console,
        "frame_sha256": sha256_bytes(raw),
        "frame_size_bytes": len(raw),
    }


def _assert_absent(path: Path) -> None:
    try:
        os.lstat(path)
    except FileNotFoundError:
        return
    raise SupervisorError(f"one-shot path already exists: {path}")


def assert_one_shot_paths_absent() -> None:
    for path in (ATTEMPT_ROOT, START_RECEIPT, EXECUTION_RECEIPT, LIFECYCLE_RECEIPT, LIFECYCLE_FAILURE_RECEIPT, CONSOLE_LOG):
        _assert_absent(path)


def open_directory_chain_nofollow(path: Path) -> int:
    if not path.is_absolute():
        raise SupervisorError("directory chain must be absolute")
    descriptor = os.open("/", os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        for component in path.parts[1:]:
            child = os.open(component, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=descriptor)
            os.close(descriptor)
            descriptor = child
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


def mkdir_new_at(parent_fd: int, name: str, mode: int) -> int:
    if not name or "/" in name or name in (".", ".."):
        raise SupervisorError("unsafe output directory basename")
    try:
        os.mkdir(name, mode, dir_fd=parent_fd)
    except FileExistsError as exc:
        raise SupervisorError(f"one-shot output directory exists: {name}") from exc
    descriptor = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=parent_fd)
    metadata = os.fstat(descriptor)
    if not stat.S_ISDIR(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) != mode:
        os.close(descriptor)
        raise SupervisorError(f"new output directory mode differs: {name}")
    return descriptor


def verify_uid1000_nnp() -> dict[str, Any]:
    fields: dict[str, str] = {}
    for line in Path("/proc/self/status").read_text(encoding="ascii").splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            fields[key] = value.strip()
    uid = [int(value) for value in fields["Uid"].split()]
    gid = [int(value) for value in fields["Gid"].split()]
    groups = [int(value) for value in fields.get("Groups", "").split()]
    caps = {key: int(fields[key], 16) for key in ("CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb")}
    nnp = int(fields["NoNewPrivs"])
    if uid != [HOST_UID] * 4 or gid != [HOST_GID] * 4 or groups or any(caps.values()) or nnp != 1:
        raise SupervisorError("UID1000 exporter identity differs")
    return {"uid": uid, "gid": gid, "groups": groups, "capabilities": caps, "no_new_privs": nnp}


def consume_capability(fd_env: str, digest_env: str) -> dict[str, Any]:
    try:
        descriptor = int(os.environ[fd_env])
        expected = os.environ[digest_env]
    except (KeyError, ValueError) as exc:
        raise SupervisorError("root one-use capability is absent") from exc
    payload = bytearray()
    try:
        metadata = os.fstat(descriptor)
        if (
            descriptor < 3
            or len(expected) != 64
            or not stat.S_ISFIFO(metadata.st_mode)
            or metadata.st_uid != 0
            or metadata.st_gid != 0
            or stat.S_IMODE(metadata.st_mode) != 0o600
            or not os.get_inheritable(descriptor)
        ):
            raise SupervisorError("root one-use capability metadata differs")
        while len(payload) < 32:
            block = os.read(descriptor, 32 - len(payload))
            if not block:
                raise SupervisorError("root one-use capability is truncated")
            payload.extend(block)
        if os.read(descriptor, 1) != b"":
            raise SupervisorError("root one-use capability has trailing bytes")
    finally:
        os.close(descriptor)
    if sha256_bytes(payload) != expected:
        raise SupervisorError("root one-use capability digest differs")
    os.environ.pop(fd_env, None)
    os.environ.pop(digest_env, None)
    return {"transport": "root_owned_anonymous_pipe_strict_eof", "sha256": expected, "size_bytes": 32}


def write_payload_new(parent_fd: int, name: str, payload: bytes, mode: int) -> dict[str, Any]:
    descriptor = os.open(
        name,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        0o400,
        dir_fd=parent_fd,
    )
    try:
        view = memoryview(payload)
        while view:
            count = os.write(descriptor, view)
            if count <= 0:
                raise SupervisorError(f"short O_EXCL write: {name}")
            view = view[count:]
        os.fsync(descriptor)
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise SupervisorError(f"O_EXCL output inode differs: {name}")
        os.fchmod(descriptor, mode)
        os.fsync(descriptor)
        return {
            "sha256": sha256_bytes(payload),
            "size_bytes": len(payload),
            "mode": f"{mode:04o}",
            "device": metadata.st_dev,
            "inode": metadata.st_ino,
            "nlink": metadata.st_nlink,
        }
    finally:
        os.close(descriptor)


def export_frame_o_excl(frame: bytes) -> dict[str, Any]:
    identity = verify_uid1000_nnp()
    capability = consume_capability(EXPORT_FD_ENV, EXPORT_SHA256_ENV)
    parsed = parse_success_frame(frame)
    cases_fd = open_directory_chain_nofollow(ATTEMPT_ROOT.parent)
    audits_fd = open_directory_chain_nofollow(CONSOLE_LOG.parent)
    attempt_fd = output_fd = data_fd = -1
    exported: dict[str, Any] = {}
    try:
        attempt_fd = mkdir_new_at(cases_fd, ATTEMPT_ROOT.name, 0o700)
        output_fd = mkdir_new_at(attempt_fd, OUTPUT_ROOT.name, 0o700)
        data_fd = mkdir_new_at(output_fd, "data", 0o700)
        for relative in EXPECTED_PATHS:
            payload = parsed["payloads"][relative]
            if relative.startswith("data/"):
                parent_fd, name = data_fd, relative.split("/", 1)[1]
            else:
                parent_fd, name = output_fd, relative
            record = write_payload_new(parent_fd, name, payload, 0o440)
            exported[relative] = {"path": str(OUTPUT_ROOT / relative), **record}
        if sorted(os.listdir(output_fd)) != sorted((*EXPECTED_ROOT_FILES, "data")):
            raise SupervisorError("host solver output root set differs")
        if sorted(os.listdir(data_fd)) != sorted(EXPECTED_DATA_FILES):
            raise SupervisorError("host solver data file set differs")
        console = write_payload_new(audits_fd, CONSOLE_LOG.name, parsed["console"], 0o440)
        for descriptor in (data_fd, output_fd, attempt_fd):
            os.fsync(descriptor)
            os.fchmod(descriptor, 0o550)
            os.fsync(descriptor)
        os.fsync(audits_fd)
        os.fsync(cases_fd)
    finally:
        for descriptor in (data_fd, output_fd, attempt_fd, audits_fd, cases_fd):
            if descriptor >= 0:
                os.close(descriptor)
    return {
        "files": exported,
        "console_log": {"path": str(CONSOLE_LOG), **console},
        "frame_sha256": parsed["frame_sha256"],
        "export_identity": identity,
        "root_cleanup_capability": capability,
        "classification": "SIM_ONLY_UNVALIDATED",
    }


def write_json_new(path: Path, value: Mapping[str, Any], *, mode: int = 0o440) -> dict[str, Any]:
    if path.parent != Path("/home/zrj/scout_liquid_lab/audits") or os.geteuid() != 0:
        raise SupervisorError("receipt creation requires root and exact audit directory")
    raw = (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + "\n").encode("utf-8")
    directory = open_directory_chain_nofollow(path.parent)
    try:
        descriptor = os.open(
            path.name,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
            0o400,
            dir_fd=directory,
        )
        try:
            view = memoryview(raw)
            while view:
                count = os.write(descriptor, view)
                if count <= 0:
                    raise SupervisorError("short append-only receipt write")
                view = view[count:]
            os.fsync(descriptor)
            os.fchown(descriptor, HOST_UID, HOST_GID)
            os.fchmod(descriptor, mode)
            os.fsync(descriptor)
            metadata = os.fstat(descriptor)
            if metadata.st_uid != HOST_UID or metadata.st_gid != HOST_GID or metadata.st_nlink != 1 or stat.S_IMODE(metadata.st_mode) != mode:
                raise SupervisorError("append-only receipt inode differs")
        finally:
            os.close(descriptor)
        os.fsync(directory)
    finally:
        os.close(directory)
    return {"path": str(path), "sha256": sha256_bytes(raw), "size_bytes": len(raw), "mode": f"{mode:04o}", "creation": "O_EXCL_NOFOLLOW_DIRFD_ANCHORED"}


def load_lifecycle_harness() -> Any:
    raw = read_regular_bytes(HARNESS_PATH, limit=512 * 1024)
    if sha256_bytes(raw) != V11_HARNESS_SHA256 or len(raw) != V11_HARNESS_SIZE:
        raise SupervisorError("frozen v11 lifecycle harness differs")
    module = load_module(HARNESS_PATH, "_r8_solver_cpu_v1_lifecycle_harness")
    module.LABELS = LABELS
    module.BOOTSTRAP_PROFILE = BOOTSTRAP_PROFILE
    module.RUNTIME_PROFILE = RUNTIME_PROFILE
    module.HOST_UID = HOST_UID
    module.HOST_GID = HOST_GID
    module.SYSCTL_PATHS = SYSCTL_PATHS
    return module


def verify_root_snapshot(policy_sha256: str) -> tuple[dict[str, Any], dict[str, Any]]:
    if os.geteuid() != 0 or not IN_SNAPSHOT or SCRIPT_PATH != SNAPSHOT_ROOT / SUPERVISOR_NAME:
        raise SupervisorError("run requires the exact root-owned solver snapshot")
    root = os.lstat(SNAPSHOT_ROOT)
    if not stat.S_ISDIR(root.st_mode) or root.st_uid != 0 or root.st_gid != 0 or stat.S_IMODE(root.st_mode) != 0o555:
        raise SupervisorError("solver snapshot root metadata differs")
    names = sorted(os.listdir(SNAPSHOT_ROOT))
    expected_names = sorted(name for _source, name in SNAPSHOT_SOURCE_PAIRS)
    if names != expected_names:
        raise SupervisorError("solver snapshot file set differs")
    policy_raw = read_regular_bytes(POLICY_PATH)
    if len(policy_sha256) != 64 or sha256_bytes(policy_raw) != policy_sha256:
        raise SupervisorError("externally frozen solver policy digest differs")
    policy = json.loads(policy_raw.decode("utf-8"))
    expected_by_name: dict[str, tuple[str, int]] = {}
    trusted = policy["trusted_artifacts"]
    for kind, path in artifact_paths().items():
        entry = trusted[kind]
        expected_by_name[path.name] = (entry["sha256"], entry["size_bytes"])
    expected_by_name[POLICY_NAME] = (policy_sha256, len(policy_raw))
    expected_by_name[HARNESS_NAME] = (V11_HARNESS_SHA256, V11_HARNESS_SIZE)
    for name, _source, digest, size in INPUT_SOURCES:
        expected_by_name[name] = (digest, size)
    observed: dict[str, Any] = {}
    for name in expected_names:
        path = SNAPSHOT_ROOT / name
        metadata = os.lstat(path)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != 0 or metadata.st_gid != 0 or metadata.st_nlink != 1 or stat.S_IMODE(metadata.st_mode) != 0o444:
            raise SupervisorError(f"solver snapshot file metadata differs: {name}")
        digest, size = expected_by_name[name]
        raw = read_regular_bytes(path, limit=80 * 1024 * 1024)
        if len(raw) != size or sha256_bytes(raw) != digest:
            raise SupervisorError(f"solver snapshot file identity differs: {name}")
        observed[name] = {"path": str(path), "sha256": digest, "size_bytes": size, "mode": "0444"}
    return policy, {"root": str(SNAPSHOT_ROOT), "mode": "0555", "files": observed}


def read_snapshot_inputs() -> dict[str, bytes]:
    if not IN_SNAPSHOT:
        raise SupervisorError("solver inputs require exact root snapshot")
    result: dict[str, bytes] = {}
    for name, _source, digest, size in INPUT_SOURCES:
        raw = read_regular_bytes(SNAPSHOT_ROOT / name, limit=80 * 1024 * 1024)
        if len(raw) != size or sha256_bytes(raw) != digest:
            raise SupervisorError(f"snapshot solver input differs: {name}")
        result[name] = raw
    return result


def create_root_capability() -> tuple[int, dict[str, Any]]:
    if os.geteuid() != 0:
        raise SupervisorError("root capability creation requires euid 0")
    read_fd, write_fd = os.pipe2(os.O_CLOEXEC)
    try:
        nonce = os.getrandom(32)
        view = memoryview(nonce)
        while view:
            count = os.write(write_fd, view)
            if count <= 0:
                raise SupervisorError("short root capability write")
            view = view[count:]
    except BaseException:
        os.close(read_fd)
        raise
    finally:
        os.close(write_fd)
    return read_fd, {"transport": "ROOT_OWNED_ANONYMOUS_PIPE_SINGLE_STRICT_EOF_READ", "sha256": sha256_bytes(nonce), "size_bytes": 32}


def gate_handoff_argv() -> list[str]:
    return [
        "/usr/bin/setpriv",
        "--reuid=1000",
        "--regid=1000",
        "--clear-groups",
        "--bounding-set=-all",
        "--inh-caps=-all",
        "--ambient-caps=-all",
        "--no-new-privs",
        "--",
        "/usr/bin/python3.12",
        "-I",
        "-B",
        "-S",
        str(GATE_PATH),
        "internal-run",
    ]


def export_handoff_argv() -> list[str]:
    return [
        "/usr/bin/setpriv",
        "--reuid=1000",
        "--regid=1000",
        "--clear-groups",
        "--bounding-set=-all",
        "--inh-caps=-all",
        "--ambient-caps=-all",
        "--no-new-privs",
        "--",
        "/usr/bin/python3.12",
        "-I",
        "-B",
        "-S",
        str(SCRIPT_PATH),
        "export-frame",
    ]


def command_evidence(result: Mapping[str, Any], *, include_stderr_prefix: bool = False) -> dict[str, Any]:
    stdout = result.get("stdout", b"")
    stderr = result.get("stderr", b"")
    evidence = {
        "argv": result.get("argv"),
        "returncode": result.get("returncode"),
        "stdin_size_bytes": result.get("stdin_size_bytes"),
        "stdin_fully_written": result.get("stdin_fully_written"),
        "stdout_size_bytes": len(stdout),
        "stdout_sha256": sha256_bytes(stdout),
        "stderr_size_bytes": len(stderr),
        "stderr_sha256": sha256_bytes(stderr),
        "failure": result.get("failure"),
        "start_new_session": result.get("start_new_session"),
    }
    if include_stderr_prefix:
        evidence["stderr_utf8_prefix"] = stderr[:4096].decode("utf-8", "replace")
    return evidence


def require_closed_zero_denial_audit(audit: Mapping[str, Any]) -> None:
    if (
        audit.get("capture_valid") is not True
        or audit.get("capture_errors") != []
        or audit.get("matching_total") != 0
        or audit.get("stored_count") != 0
        or audit.get("dropped_count") != 0
        or audit.get("storage_overflow") is not False
        or audit.get("unexpected_total") != 0
        or audit.get("sanitized_denials") != []
        or not audit.get("start_cursor")
        or not audit.get("end_cursor")
        or audit.get("boot_id_before") != audit.get("boot_id_after")
    ):
        raise SupervisorError("AppArmor journal window is not a closed zero-denial window")


def verify_exported_outputs(parsed: Mapping[str, Any]) -> dict[str, Any]:
    attempt = os.lstat(ATTEMPT_ROOT)
    output = os.lstat(OUTPUT_ROOT)
    data = os.lstat(OUTPUT_ROOT / "data")
    for metadata, label in ((attempt, "attempt"), (output, "output"), (data, "data")):
        if not stat.S_ISDIR(metadata.st_mode) or metadata.st_uid != HOST_UID or metadata.st_gid != HOST_GID or stat.S_IMODE(metadata.st_mode) != 0o550:
            raise SupervisorError(f"exported {label} directory metadata differs")
    if sorted(os.listdir(OUTPUT_ROOT)) != sorted((*EXPECTED_ROOT_FILES, "data")) or sorted(os.listdir(OUTPUT_ROOT / "data")) != sorted(EXPECTED_DATA_FILES):
        raise SupervisorError("exported solver output tree differs")
    observed: dict[str, Any] = {}
    for relative in EXPECTED_PATHS:
        path = OUTPUT_ROOT / relative
        raw = read_regular_bytes(path, limit=16_777_216)
        metadata = os.lstat(path)
        expected = parsed["payloads"][relative]
        if raw != expected or metadata.st_uid != HOST_UID or metadata.st_gid != HOST_GID or stat.S_IMODE(metadata.st_mode) != 0o440:
            raise SupervisorError(f"exported solver file identity differs: {relative}")
        observed[relative] = {"path": str(path), "sha256": sha256_bytes(raw), "size_bytes": len(raw), "mode": "0440"}
    console_raw = read_regular_bytes_allow_empty(CONSOLE_LOG, limit=OUTPUT_CONSOLE_LIMIT)
    console_metadata = os.lstat(CONSOLE_LOG)
    if (
        console_raw != parsed["console"]
        or console_metadata.st_uid != HOST_UID
        or console_metadata.st_gid != HOST_GID
        or console_metadata.st_nlink != 1
        or stat.S_IMODE(console_metadata.st_mode) != 0o440
    ):
        raise SupervisorError("exported solver console identity differs")
    return {
        "files": observed,
        "file_count": len(observed),
        "console_log": {
            "path": str(CONSOLE_LOG),
            "sha256": sha256_bytes(console_raw),
            "size_bytes": len(console_raw),
            "mode": "0440",
            "nlink": console_metadata.st_nlink,
        },
    }


def run_once(*, policy_sha256: str, admission_token: str) -> dict[str, Any]:
    if admission_token != ADMISSION_TOKEN:
        raise SupervisorError("explicit one-shot solver admission token differs")
    policy, snapshot = verify_root_snapshot(policy_sha256)
    assert_one_shot_paths_absent()
    harness = load_lifecycle_harness()
    gate = load_module(GATE_PATH, "_r8_solver_cpu_v1_runtime_gate")
    with harness.TerminationGuard() as termination_guard:
        termination_guard.checkpoint()
        sudo_admission = harness.clear_invoking_user_sudo_timestamp()
        harness.validate_sudo_cleanup_evidence(sudo_admission)
        sysctls_before = harness.read_sysctls()
        if policy.get("profile_lifecycle", {}).get("expected_sysctls") != sysctls_before:
            raise SupervisorError("current sysctls differ from frozen solver baseline")
        profiles_before = harness.profile_state()
        harness.require_profile_counts(profiles_before, 0)
        initial_zero = harness.require_stable_zero_labels()
        audit_anchor = harness.capture_journal_anchor()
        start_document = {
            "document_type": "SMPCC_R8_LIQUID_U3_C1_CPU_SOLVER_V1_START_RECEIPT",
            "case_id": CASE_ID,
            "status": "ONE_SHOT_STARTED_NO_RETRY_SAME_IDENTITY",
            "policy_sha256": policy_sha256,
            "snapshot": snapshot,
            "sysctls_before": sysctls_before,
            "profiles_before": profiles_before,
            "initial_stable_zero_labels": initial_zero,
            "journal_anchor": audit_anchor,
            "sudo_timestamp_admission": sudo_admission,
        }
        start_record = write_json_new(START_RECEIPT, start_document)
        load_attempted = False
        gate_result: dict[str, Any] | None = None
        parsed: dict[str, Any] | None = None
        audit: dict[str, Any] | None = None
        export_result: dict[str, Any] | None = None
        export_verification: dict[str, Any] | None = None
        primary_error: str | None = None
        cleanup_errors: list[str] = []
        cleanup: dict[str, Any] = {}
        profiles_after: dict[str, Any] = {}
        sysctls_after: dict[str, Any] = {}
        sudo_clear: dict[str, Any] = {}
        execution_status = "ATTEMPT_ABORTED_CLEANUP_PENDING"
        try:
            harness.run_checked(["/usr/sbin/apparmor_parser", "-Q", "-K", "-T", str(PROFILE_PATH)], timeout_seconds=15, require_silent=True)
            termination_guard.checkpoint()
            load_attempted = True
            harness.run_checked(["/usr/sbin/apparmor_parser", "-a", "-K", "-T", str(PROFILE_PATH)], timeout_seconds=15, require_silent=True)
            harness.require_profile_counts(harness.profile_state(), 1)
            helper_bytes = read_regular_bytes(HELPER_PATH, limit=4 * 1024 * 1024)
            frame = gate.build_input_frame(helper_bytes, read_snapshot_inputs())
            admission_fd, admission_capability = create_root_capability()
            gate_env = {
                "HOME": "/nonexistent",
                "PATH": "/usr/bin:/usr/sbin",
                "LC_ALL": "C",
                "LANG": "C",
                "TZ": "UTC0",
                SNAPSHOT_ENV: str(SNAPSHOT_ROOT),
                ADMISSION_FD_ENV: str(admission_fd),
                ADMISSION_SHA256_ENV: admission_capability["sha256"],
            }
            try:
                gate_result = harness.run_bounded_command(
                    gate_handoff_argv(),
                    stdin_bytes=frame,
                    stdout_limit=OUTPUT_FRAME_LIMIT,
                    stderr_limit=4_718_592,
                    timeout_seconds=1_830,
                    env=gate_env,
                    pass_fds=(admission_fd,),
                )
            finally:
                os.close(admission_fd)
            termination_guard.checkpoint()
            prequery_cleanup = harness.terminate_labeled_processes()
            if prequery_cleanup.get("initial") != [] or prequery_cleanup.get("after_term") != [] or prequery_cleanup.get("stable_zero_scans") != [[], [], []]:
                raise SupervisorError("solver gate left AppArmor-labelled task residue")
            harness.require_profile_counts(harness.profile_state(), 1)
            harness.run_checked(["/usr/bin/journalctl", "--sync"], timeout_seconds=15, require_silent=True)
            audit = harness.capture_apparmor_denials(audit_anchor)
            audit["prequery_label_cleanup"] = prequery_cleanup
            audit["postquery_stable_zero_labels"] = harness.require_stable_zero_labels()
            require_closed_zero_denial_audit(audit)
            if gate_result.get("returncode") != 0 or gate_result.get("failure") is not None or gate_result.get("stdin_fully_written") is not True or gate_result.get("stderr") != b"":
                raise SupervisorError("UID1000 solver gate did not return one clean successful frame")
            parsed = parse_success_frame(gate_result["stdout"])
            execution_status = "PASS_U3_C1_CPU_SOLVER_V1_FRAME_VALID_CLEANUP_PENDING"
        except BaseException as exc:
            primary_error = f"{type(exc).__name__}: {exc}"[:4096]
        finally:
            termination_guard.begin_cleanup()
            try:
                cleanup = harness.terminate_labeled_processes()
            except BaseException as exc:
                cleanup_errors.append(f"label_cleanup: {type(exc).__name__}: {exc}"[:4096])
            pre_unload_zero = cleanup.get("stable_zero_scans") == [[], [], []]
            if not pre_unload_zero:
                try:
                    cleanup = {**cleanup, "stable_zero_scans": harness.require_stable_zero_labels(), "fallback_zero_scan_used": True}
                    pre_unload_zero = True
                except BaseException as exc:
                    cleanup_errors.append(f"pre_unload_zero_scan: {type(exc).__name__}: {exc}"[:4096])
            try:
                if not pre_unload_zero:
                    raise SupervisorError("profile unload forbidden before three zero-label scans")
                counts = harness.read_kernel_profile_counts()
                if load_attempted or any(counts.values()):
                    unload = harness.run_bounded_command(
                        ["/usr/sbin/apparmor_parser", "-R", "-K", str(PROFILE_PATH)],
                        stdout_limit=65_536,
                        stderr_limit=65_536,
                        timeout_seconds=15,
                    )
                    if unload["returncode"] != 0 or unload["failure"] is not None or unload["stdout"] or unload["stderr"]:
                        raise SupervisorError("transient solver profiles failed to unload")
                profiles_after = harness.profile_state()
                harness.require_profile_counts(profiles_after, 0)
                cleanup["post_unload_stable_zero_scans"] = harness.require_stable_zero_labels()
            except BaseException as exc:
                cleanup_errors.append(f"profile_cleanup: {type(exc).__name__}: {exc}"[:4096])
            try:
                sysctls_after = harness.read_sysctls()
                if sysctls_after != sysctls_before:
                    raise SupervisorError("host sysctls changed across solver lifecycle")
            except BaseException as exc:
                cleanup_errors.append(f"sysctl_postcondition: {type(exc).__name__}: {exc}"[:4096])

        if parsed is not None and primary_error is None and not cleanup_errors:
            try:
                export_fd, export_capability = create_root_capability()
                export_env = {
                    "HOME": "/nonexistent",
                    "PATH": "/usr/bin:/usr/sbin",
                    "LC_ALL": "C",
                    "LANG": "C",
                    "TZ": "UTC0",
                    EXPORT_FD_ENV: str(export_fd),
                    EXPORT_SHA256_ENV: export_capability["sha256"],
                }
                try:
                    export_command = harness.run_bounded_command(
                        export_handoff_argv(),
                        stdin_bytes=gate_result["stdout"],
                        stdout_limit=262_144,
                        stderr_limit=65_536,
                        timeout_seconds=120,
                        env=export_env,
                        pass_fds=(export_fd,),
                    )
                finally:
                    os.close(export_fd)
                if export_command["returncode"] != 0 or export_command["failure"] is not None or export_command["stderr"] or not export_command["stdin_fully_written"]:
                    raise SupervisorError("UID1000 solver O_EXCL exporter failed")
                message = json.loads(export_command["stdout"].decode("utf-8"))
                if message.get("status") != "PASS_UID1000_SOLVER_TREE_O_EXCL_EXPORT_V1" or not isinstance(message.get("export"), dict):
                    raise SupervisorError("solver exporter receipt differs")
                export_result = {"command": command_evidence(export_command), "capability": export_capability, "guest_report": message["export"]}
                export_verification = verify_exported_outputs(parsed)
                execution_status = "PASS_U3_C1_CPU_SOLVER_V1_SMOKE_EXPORTED_CLEANUP_PENDING"
            except BaseException as exc:
                primary_error = f"{type(exc).__name__}: {exc}"[:4096]

        try:
            sudo_clear = harness.clear_invoking_user_sudo_timestamp()
            harness.validate_sudo_cleanup_evidence(sudo_clear)
        except BaseException as exc:
            cleanup_errors.append(f"sudo_timestamp: {type(exc).__name__}: {exc}"[:4096])

        execution_document = {
            "document_type": "SMPCC_R8_LIQUID_U3_C1_CPU_SOLVER_V1_EXECUTION_RECEIPT",
            "case_id": CASE_ID,
            "status": execution_status if primary_error is None and not cleanup_errors else "ATTEMPT_ABORTED_NO_RETRY_SAME_IDENTITY",
            "policy_sha256": policy_sha256,
            "start_receipt": start_record,
            "gate": None if gate_result is None else command_evidence(gate_result, include_stderr_prefix=True),
            "frame": None if parsed is None else {
                "sha256": parsed["frame_sha256"],
                "size_bytes": parsed["frame_size_bytes"],
                "metadata": parsed["metadata"],
                "payloads": {path: {"sha256": sha256_bytes(raw), "size_bytes": len(raw)} for path, raw in parsed["payloads"].items()},
            },
            "audit": audit,
            "export": export_result,
            "export_verification": export_verification,
            "primary_error": primary_error,
            "cleanup_errors": cleanup_errors,
            "production_authorized": False,
            "classification": "SIM_ONLY_UNVALIDATED",
            "settled_state_authorized": False,
        }
        execution_record = write_json_new(EXECUTION_RECEIPT, execution_document)
        success = execution_document["status"] == "PASS_U3_C1_CPU_SOLVER_V1_SMOKE_EXPORTED_CLEANUP_PENDING" and parsed is not None and export_verification is not None
        if success:
            lifecycle_document = {
                "document_type": "SMPCC_R8_LIQUID_U3_C1_CPU_SOLVER_V1_LIFECYCLE_RECEIPT",
                "case_id": CASE_ID,
                "status": "PASS_U3_C1_CPU_SOLVER_V1_LIFECYCLE_CLEANUP_AND_SMOKE_EXPORT",
                "execution_receipt": execution_record,
                "cleanup": cleanup,
                "profiles_after": profiles_after,
                "sysctls": {"before": sysctls_before, "after": sysctls_after, "unchanged": True},
                "sudo_timestamp": sudo_clear,
                "next_allowed_stage": "SEPARATE_OUTPUT_QC_THEN_FRESH_LONGER_SETTLE_IDENTITIES",
                "settled_state_authorized": False,
                "production_authorized": False,
            }
            lifecycle_record = write_json_new(LIFECYCLE_RECEIPT, lifecycle_document)
            return {"execution_status": execution_document["status"], "execution_receipt": execution_record, "lifecycle_receipt": lifecycle_record}
        failure_document = {
            "document_type": "SMPCC_R8_LIQUID_U3_C1_CPU_SOLVER_V1_LIFECYCLE_INCOMPLETE_RECEIPT",
            "case_id": CASE_ID,
            "status": "LIFECYCLE_OR_EXECUTION_INCOMPLETE_NO_RETRY_SAME_IDENTITY",
            "execution_receipt": execution_record,
            "primary_error": primary_error,
            "cleanup_errors": cleanup_errors,
            "cleanup_observation": cleanup,
            "profiles_after_observation": profiles_after,
            "sysctls_after_observation": sysctls_after,
            "sudo_timestamp_observation": sudo_clear,
            "production_authorized": False,
        }
        failure_record = write_json_new(LIFECYCLE_FAILURE_RECEIPT, failure_document)
        raise SupervisorError(f"solver v1 failed closed; preserved {failure_record['path']}")


def export_frame_entry() -> dict[str, Any]:
    if os.geteuid() != HOST_UID or not IN_SNAPSHOT:
        raise SupervisorError("export-frame requires UID1000 and exact root snapshot")
    frame = sys.stdin.buffer.read(OUTPUT_FRAME_LIMIT + 1)
    if len(frame) > OUTPUT_FRAME_LIMIT or sys.stdin.buffer.read(1) != b"":
        raise SupervisorError("solver export frame exceeds fixed ceiling")
    return export_frame_o_excl(frame)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-check")
    subparsers.add_parser("emit-bootstrap")
    run_parser = subparsers.add_parser("run")
    run_parser.add_argument("--policy-sha256", required=True)
    run_parser.add_argument("--admission-token", required=True)
    subparsers.add_parser("export-frame", help=argparse.SUPPRESS)
    arguments = parser.parse_args(argv)
    try:
        if arguments.command == "self-check":
            review = verify_policy_static()
            print(json.dumps({
                "status": "PASS_SOLVER_CPU_V1_SUPERVISOR_STATIC_EXECUTION_NOT_PERFORMED",
                "review": review,
                "snapshot_bootstrap": {
                    "source_sha256": SNAPSHOT_BOOTSTRAP_SHA256,
                    "source_size_bytes": len(SNAPSHOT_BOOTSTRAP_BYTES),
                    "loader_sha256": SNAPSHOT_BOOTSTRAP_LOADER_SHA256,
                    "loader_size_bytes": len(SNAPSHOT_BOOTSTRAP_LOADER_BYTES),
                    "loader_source": SNAPSHOT_BOOTSTRAP_LOADER_SOURCE,
                    "manifest_arguments": snapshot_manifest_arguments(review),
                },
                "admission_token": ADMISSION_TOKEN,
            }, ensure_ascii=False, sort_keys=True))
            return 0
        if arguments.command == "emit-bootstrap":
            verify_snapshot_producer_identity()
            verify_policy_static()
            sys.stdout.buffer.write(SNAPSHOT_BOOTSTRAP_BYTES)
            sys.stdout.buffer.flush()
            return 0
        if arguments.command == "export-frame":
            result = export_frame_entry()
            print(json.dumps({"status": "PASS_UID1000_SOLVER_TREE_O_EXCL_EXPORT_V1", "export": result}, ensure_ascii=False, sort_keys=True))
            return 0
        result = run_once(policy_sha256=arguments.policy_sha256, admission_token=arguments.admission_token)
        print(json.dumps({"status": "SOLVER_CPU_V1_ONE_SHOT_LIFECYCLE_RECORDED", "result": result}, ensure_ascii=False, sort_keys=True))
        return 0
    except (SupervisorError, OSError, ValueError, UnicodeError, subprocess.SubprocessError) as exc:
        target = sys.stderr if arguments.command in ("emit-bootstrap", "export-frame") else sys.stdout
        print(json.dumps({"status": "NO_GO", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=target)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
