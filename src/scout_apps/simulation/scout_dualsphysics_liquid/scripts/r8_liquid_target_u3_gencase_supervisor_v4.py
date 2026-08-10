#!/usr/bin/env python3
"""Root lifecycle supervisor and host frame auditor for U3 C1 GenCase v4.

The workspace entry points are read-only producers and static reviewers.  The
one-shot ``run`` entry is accepted only from a root-owned 0555 snapshot.  It
loads the fresh transient AppArmor pair, hands one root-owned anonymous-FD
capability plus one fixed stdin frame to the UID-1000 gate, closes a journal
cursor audit window, removes every labeled task and both profiles, then permits
a separate UID-1000/NNP exporter to create the two expected outputs with
O_EXCL.  Existing paths are never overwritten or deleted.
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
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Iterable, Mapping


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_DIR = SCRIPT_PATH.parent.parent
WORKSPACE_PACKAGE_ROOT = Path("/home/zrj/scout_ws/src/scout_apps/simulation/scout_dualsphysics_liquid")
WORKSPACE_SUPERVISOR_PATH = WORKSPACE_PACKAGE_ROOT / "scripts/r8_liquid_target_u3_gencase_supervisor_v4.py"
CASE_ID = "u3_c1_gencase_v4_20260808T065518Z"
HOST_UID = 1000
HOST_GID = 1000
BOOTSTRAP_PROFILE = "r8-liquid-u3-gencase-bootstrap-v4-20260808t065518z"
RUNTIME_PROFILE = "r8-liquid-u3-gencase-runtime-v4-20260808t065518z"
LABELS = (BOOTSTRAP_PROFILE, RUNTIME_PROFILE)
SNAPSHOT_ENV = "R8_LIQUID_U3_GENCASE_V4_SNAPSHOT"
ADMISSION_FD_ENV = "R8_LIQUID_U3_GENCASE_V4_ADMISSION_FD"
ADMISSION_SHA256_ENV = "R8_LIQUID_U3_GENCASE_V4_ADMISSION_SHA256"
EXPORT_FD_ENV = "R8_LIQUID_U3_GENCASE_V4_EXPORT_FD"
EXPORT_SHA256_ENV = "R8_LIQUID_U3_GENCASE_V4_EXPORT_SHA256"
ADMISSION_TOKEN = f"{CASE_ID}:single-gencase-attempt"
SNAPSHOT_ROOT = Path(f"/run/r8-liquid-{CASE_ID}.snapshot")

POLICY_NAME = "liquid_zrj_msi_u2404_u3_gencase_execution_policy_v4.json"
SCHEMA_NAME = "target_host_u3_gencase_execution_policy_v4.json"
PROFILE_NAME = "r8-liquid-u3-gencase-v4.profile"
GATE_NAME = "r8_liquid_target_u3_gencase_gate_v4.py"
HELPER_NAME = "r8_liquid_u3_gencase_bootstrap_helper_v4.py"
SUPERVISOR_NAME = "r8_liquid_target_u3_gencase_supervisor_v4.py"
HARNESS_NAME = "r8_liquid_target_u3_stdio_apparmor_transport_probe_supervisor_v11.py"

IN_SNAPSHOT = SCRIPT_PATH.parent == SNAPSHOT_ROOT
BASE = SNAPSHOT_ROOT if IN_SNAPSHOT else WORKSPACE_PACKAGE_ROOT
POLICY_PATH = BASE / POLICY_NAME if IN_SNAPSHOT else BASE / "config/target_hosts" / POLICY_NAME
SCHEMA_PATH = BASE / SCHEMA_NAME if IN_SNAPSHOT else BASE / "schema" / SCHEMA_NAME
PROFILE_PATH = BASE / PROFILE_NAME if IN_SNAPSHOT else BASE / "config/apparmor_drafts" / PROFILE_NAME
GATE_PATH = BASE / GATE_NAME if IN_SNAPSHOT else BASE / "scripts" / GATE_NAME
HELPER_PATH = BASE / HELPER_NAME if IN_SNAPSHOT else BASE / "scripts" / HELPER_NAME
HARNESS_PATH = BASE / HARNESS_NAME if IN_SNAPSHOT else BASE / "scripts" / HARNESS_NAME

SEED_ID = "u3_c1_gencase_seed_v2_20260807T135341Z"
SEED_ROOT = Path(f"/home/zrj/scout_liquid_lab/dependency/runtime/{SEED_ID}.partial")
SEED_INPUT_ROOT = SEED_ROOT / "input"
SEED_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{SEED_ID}.json")
SEED_RECEIPT_SHA256 = "1bbf958dfe2f7ce026ce05d77e7ee2c2516c5d0ddc4345b021904e355003009d"
ATTEMPT_ROOT = Path(f"/home/zrj/scout_liquid_lab/cases/{CASE_ID}.partial")
OUTPUT_ROOT = ATTEMPT_ROOT / "output"
START_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.start.json")
EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.execution.json")
LIFECYCLE_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.lifecycle.json")
LIFECYCLE_FAILURE_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.lifecycle_incomplete.json")
CONSOLE_LOG = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.console.log")

INPUT_CONTRACT: dict[str, tuple[str, int]] = {
    "GenCase_linux64": ("a1b6414e0f716669363d1a80a05e133085c04995e15716e3c1f0406e0d023226", 5_809_384),
    "DsphConfig.xml": ("0644c9a6a6687678950fc8966e352b4bbd3de9d3cb787db9e507c2eb7ccaddcd", 293),
    "C1_static_Def.xml": ("d738d303300b3f339ac37ca3a604fbe8dff9e034d8c8ad7aacb2654434525819", 5_714),
}
GENCASE_ARGV = [
    "/work/runtime/GenCase_linux64",
    "/work/runtime/C1_static_Def",
    "/work/output/C1_static",
    "-dp:0.002",
    "-threads:1",
    "-save:-all,+bi",
    "-createdirs:1",
]
HELPER_MAGIC = b"R8C1HELPERV4\0\0\0\0\0"
INPUT_MAGIC = b"R8C1INPUTV4\0\0\0\0\0"
OUTPUT_MAGIC = b"R8C1GENCASEV4\0\0\0"
OUTPUT_HEADER = struct.Struct(">16sIIQQQ")
OUTPUT_VERSION = 1
OUTPUT_METADATA_LIMIT = 65_536
OUTPUT_BI4_LIMIT = 16_777_216
OUTPUT_XML_LIMIT = 524_288
OUTPUT_CONSOLE_LIMIT = 1_048_576
OUTPUT_FRAME_LIMIT = OUTPUT_HEADER.size + OUTPUT_METADATA_LIMIT + OUTPUT_BI4_LIMIT + OUTPUT_XML_LIMIT + OUTPUT_CONSOLE_LIMIT

SYSCTL_PATHS = (
    Path("/proc/sys/user/max_user_namespaces"),
    Path("/proc/sys/kernel/unprivileged_userns_clone"),
    Path("/proc/sys/kernel/apparmor_restrict_unprivileged_userns"),
)

V11_HARNESS_SHA256 = "72b7f1bf287b4fe750b46dccdb7a1eae9379230c7bdc30dbc6a6cb33a403ef5b"
V11_HARNESS_SIZE = 159_126

SNAPSHOT_SOURCE_PAIRS = (
    (str(WORKSPACE_PACKAGE_ROOT / "scripts" / GATE_NAME), GATE_NAME),
    (str(WORKSPACE_PACKAGE_ROOT / "scripts" / HELPER_NAME), HELPER_NAME),
    (str(WORKSPACE_PACKAGE_ROOT / "scripts" / SUPERVISOR_NAME), SUPERVISOR_NAME),
    (str(WORKSPACE_PACKAGE_ROOT / "config/apparmor_drafts" / PROFILE_NAME), PROFILE_NAME),
    (str(WORKSPACE_PACKAGE_ROOT / "schema" / SCHEMA_NAME), SCHEMA_NAME),
    (str(WORKSPACE_PACKAGE_ROOT / "config/target_hosts" / POLICY_NAME), POLICY_NAME),
    (str(WORKSPACE_PACKAGE_ROOT / "scripts" / HARNESS_NAME), HARNESS_NAME),
    (str(SEED_INPUT_ROOT / "GenCase_linux64"), "GenCase_linux64"),
    (str(SEED_INPUT_ROOT / "DsphConfig.xml"), "DsphConfig.xml"),
    (str(SEED_INPUT_ROOT / "C1_static_Def.xml"), "C1_static_Def.xml"),
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
 if len(digest)!=64 or size<1 or size>33554432: raise SystemExit(74)
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
os.write(1,(json.dumps({{"status":"ROOT_GENCASE_V4_SNAPSHOT_CREATED_NOT_EXECUTED","files":manifest}},sort_keys=True,separators=(",",":"))+"\\n").encode("ascii"))
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
exec(compile(p,"<r8-liquid-gencase-v4-root-snapshot-bootstrap>","exec",dont_inherit=True,optimize=2),g,g)
'''
SNAPSHOT_BOOTSTRAP_LOADER_BYTES = SNAPSHOT_BOOTSTRAP_LOADER_SOURCE.encode("ascii")
SNAPSHOT_BOOTSTRAP_LOADER_SHA256 = hashlib.sha256(SNAPSHOT_BOOTSTRAP_LOADER_BYTES).hexdigest()

sys.dont_write_bytecode = True


class SupervisorError(RuntimeError):
    """A fail-closed supervisor, frame, output, or lifecycle error."""


def read_regular_bytes(path: Path, *, limit: int = 20 * 1024 * 1024) -> bytes:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or metadata.st_size > limit:
            raise SupervisorError(f"unsafe regular file: {path}")
        result = bytearray()
        while True:
            block = os.read(descriptor, 1 << 20)
            if not block:
                return bytes(result)
            result.extend(block)
            if len(result) > limit:
                raise SupervisorError(f"regular file exceeds its frozen ceiling: {path}")
    finally:
        os.close(descriptor)


def sha256_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def read_json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(read_regular_bytes(path, limit=2 * 1024 * 1024).decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError(f"invalid JSON object: {path}") from exc
    if not isinstance(value, dict):
        raise SupervisorError(f"JSON root is not an object: {path}")
    return value


def policy_artifact_paths() -> dict[str, Path]:
    return {
        "gate": GATE_PATH,
        "helper": HELPER_PATH,
        "supervisor": SCRIPT_PATH,
        "profile": PROFILE_PATH,
        "schema": SCHEMA_PATH,
    }


def verify_static_artifact_hashes(policy: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    expected = policy.get("trusted_artifacts")
    if not isinstance(expected, dict) or set(expected) != set(policy_artifact_paths()):
        raise SupervisorError("trusted artifact set differs")
    observed: dict[str, dict[str, Any]] = {}
    for name, path in policy_artifact_paths().items():
        raw = read_regular_bytes(path)
        digest = sha256_bytes(raw)
        entry = expected[name]
        if digest != entry.get("sha256") or len(raw) != entry.get("size_bytes"):
            raise SupervisorError(f"trusted artifact bytes differ: {name}")
        observed[name] = {"path": str(path), "sha256": digest, "size_bytes": len(raw)}
    return observed


def verify_policy_static() -> dict[str, Any]:
    policy = read_json_object(POLICY_PATH)
    schema = read_json_object(SCHEMA_PATH)
    if schema.get("additionalProperties") is not False or set(policy) != set(schema.get("required", [])):
        raise SupervisorError("policy/schema top-level contract differs")
    if policy.get("status") != "ADMITTED_SINGLE_GENCASE_RUNTIME_V4_AFTER_FROZEN_V11_ZERO_DENIAL_PROBE":
        raise SupervisorError("v4 one-shot admission status differs")
    if policy.get("allowed_gate_commands") != ["self-check", "internal-run"]:
        raise SupervisorError("v4 gate command surface differs")
    artifacts = verify_static_artifact_hashes(policy)
    harness_raw = read_regular_bytes(HARNESS_PATH)
    harness = policy.get("lifecycle_harness", {})
    if (sha256_bytes(harness_raw) != V11_HARNESS_SHA256 or len(harness_raw) != V11_HARNESS_SIZE
            or harness.get("sha256") != V11_HARNESS_SHA256
            or harness.get("size_bytes") != V11_HARNESS_SIZE):
        raise SupervisorError("frozen v11 lifecycle harness identity differs")
    seed_inputs = verify_seed_receipt_and_read_inputs()
    probe = policy.get("required_harmless_probes", {})
    v11_execution = Path("/home/zrj/scout_liquid_lab/audits/u3_stdio_apparmor_transport_probe_v11_20260808T052940Z.execution.json")
    v11_lifecycle = Path("/home/zrj/scout_liquid_lab/audits/u3_stdio_apparmor_transport_probe_v11_20260808T052940Z.lifecycle.json")
    frozen_receipts = {
        "execution": (v11_execution, "02a9421481850986e8e4d7ef488e1ed8a6d177b8f40a683625a916d1fc563f71"),
        "lifecycle": (v11_lifecycle, "8f56909f33a4192becb9f194a5e168e5152eb5d5416e4313fec8ec010467e9f8"),
    }
    receipt_evidence: dict[str, Any] = {}
    for name, (path, digest) in frozen_receipts.items():
        raw = read_regular_bytes(path, limit=256 * 1024)
        if sha256_bytes(raw) != digest:
            raise SupervisorError(f"frozen v11 {name} receipt digest differs")
        receipt_evidence[name] = {"path": str(path), "sha256": digest, "size_bytes": len(raw)}
    if (probe.get("required_receipt_sha256") != frozen_receipts["execution"][1]
            or probe.get("required_lifecycle_receipt_sha256") != frozen_receipts["lifecycle"][1]):
        raise SupervisorError("v4 policy does not pin the frozen v11 PASS lifecycle")
    predecessor = policy.get("rejected_predecessor", {})
    v3_paths = {
        "execution": (
            Path("/home/zrj/scout_liquid_lab/audits/u3_c1_gencase_v3_20260808T062327Z.execution.json"),
            "31facb61f218b9da03f7c5280b89f4a6d83ca37088427e864b3b360b43b40015",
        ),
        "lifecycle": (
            Path("/home/zrj/scout_liquid_lab/audits/u3_c1_gencase_v3_20260808T062327Z.lifecycle_incomplete.json"),
            "2003864975382a457129da959e67bd25c97ed3bd09c28d9a7939c4cea90a194e",
        ),
    }
    v3_documents: dict[str, Any] = {}
    for name, (path, digest) in v3_paths.items():
        raw = read_regular_bytes(path, limit=256 * 1024)
        if sha256_bytes(raw) != digest:
            raise SupervisorError(f"frozen v3 {name} receipt digest differs")
        v3_documents[name] = json.loads(raw.decode("utf-8"))
    denial = v3_documents["execution"].get("audit", {}).get("unexpected_denials", [])
    cleanup_v3 = v3_documents["lifecycle"]
    if (predecessor.get("execution_receipt_sha256") != v3_paths["execution"][1]
            or predecessor.get("lifecycle_receipt_sha256") != v3_paths["lifecycle"][1]
            or v3_documents["execution"].get("status") != "ATTEMPT_ABORTED_NO_RETRY_SAME_IDENTITY"
            or len(denial) != 1
            or {field["key"]: field["value"] for field in denial[0].get("fields", [])}.get("name") != "/proc/sys/kernel/cap_last_cap"
            or cleanup_v3.get("cleanup_errors") != []
            or cleanup_v3.get("cleanup_observation", {}).get("post_unload_stable_zero_scans") != [[], [], []]
            or any(cleanup_v3.get("profiles_after_observation", {}).get("kernel_exact_counts", {}).values())
            or cleanup_v3.get("sysctls_after_observation") != policy.get("profile_lifecycle", {}).get("expected_sysctls")):
        raise SupervisorError("frozen v3 fail-closed provenance differs")
    policy_raw = read_regular_bytes(POLICY_PATH)
    snapshot_sources: list[dict[str, Any]] = []
    for source_name, destination_name in SNAPSHOT_SOURCE_PAIRS:
        path = Path(source_name)
        raw = read_regular_bytes(path, limit=32 * 1024 * 1024)
        snapshot_sources.append({
            "source": source_name,
            "name": destination_name,
            "sha256": sha256_bytes(raw),
            "size_bytes": len(raw),
        })
    return {
        "policy": {"path": str(POLICY_PATH), "sha256": sha256_bytes(policy_raw), "size_bytes": len(policy_raw)},
        "schema": {"path": str(SCHEMA_PATH), "sha256": artifacts["schema"]["sha256"]},
        "artifacts": artifacts,
        "lifecycle_harness": {"path": str(HARNESS_PATH), "sha256": V11_HARNESS_SHA256, "size_bytes": V11_HARNESS_SIZE},
        "seed_inputs": {name: {"sha256": sha256_bytes(raw), "size_bytes": len(raw)} for name, raw in seed_inputs.items()},
        "frozen_v11_receipts": receipt_evidence,
        "frozen_v3_failure": {name: {"path": str(path), "sha256": digest} for name, (path, digest) in v3_paths.items()},
        "snapshot_sources": snapshot_sources,
        "production_run_allowed": True,
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
        raise SupervisorError("snapshot byte producer must be the exact unprivileged workspace supervisor")
    status = Path("/proc/self/status").read_text(encoding="ascii")
    fields = {line.split(":", 1)[0]: line.split(":", 1)[1].strip() for line in status.splitlines() if ":" in line}
    uid = [int(value) for value in fields["Uid"].split()]
    gid = [int(value) for value in fields["Gid"].split()]
    caps = [int(fields[key], 16) for key in ("CapInh", "CapPrm", "CapEff", "CapAmb")]
    if uid != [HOST_UID] * 4 or gid != [HOST_GID] * 4 or any(caps):
        raise SupervisorError("snapshot byte producer identity differs")
    return {"uid": uid, "gid": gid, "active_capabilities_zero": True}


def _assert_absent(path: Path) -> None:
    try:
        os.lstat(path)
    except FileNotFoundError:
        return
    raise SupervisorError(f"one-shot path already exists: {path}")


def verify_seed_receipt_and_read_inputs() -> dict[str, bytes]:
    receipt_raw = read_regular_bytes(SEED_RECEIPT, limit=64 * 1024)
    if sha256_bytes(receipt_raw) != SEED_RECEIPT_SHA256:
        raise SupervisorError("seed v2 receipt digest differs")
    try:
        receipt = json.loads(receipt_raw.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError("seed v2 receipt is invalid") from exc
    if receipt.get("status") != "PASS_NONEXECUTABLE_GENCASE_SEED_V2_MATERIALIZATION":
        raise SupervisorError("seed v2 receipt is not the required non-executing PASS")
    for field in (
        "upstream_code_executed",
        "precompiled_binary_executed",
        "compiled_artifact_executed",
        "network_used",
        "gpu_device_exposed",
        "source_checkout_created",
        "system_packages_changed",
        "sudo_used",
    ):
        if receipt.get(field) is not False:
            raise SupervisorError(f"seed v2 receipt violates non-execution: {field}")
    result: dict[str, bytes] = {}
    names = sorted(entry.name for entry in os.scandir(SEED_INPUT_ROOT))
    if names != sorted(INPUT_CONTRACT):
        raise SupervisorError("seed v2 input file set differs")
    for name, (digest, size) in INPUT_CONTRACT.items():
        path = SEED_INPUT_ROOT / name
        metadata = os.stat(path, follow_symlinks=False)
        if (
            not stat.S_ISREG(metadata.st_mode)
            or stat.S_ISLNK(metadata.st_mode)
            or metadata.st_nlink != 1
            or stat.S_IMODE(metadata.st_mode) != 0o400
            or metadata.st_size != size
        ):
            raise SupervisorError(f"seed v2 file contract differs: {name}")
        raw = read_regular_bytes(path, limit=16 * 1024 * 1024)
        if sha256_bytes(raw) != digest:
            raise SupervisorError(f"seed v2 file digest differs: {name}")
        result[name] = raw
    return result


def build_fixed_input_frame(helper_bytes: bytes, seed_inputs: Mapping[str, bytes], policy: Mapping[str, Any]) -> bytes:
    helper = policy["trusted_artifacts"]["helper"]
    if len(helper_bytes) != helper["size_bytes"] or sha256_bytes(helper_bytes) != helper["sha256"]:
        raise SupervisorError("helper bytes differ before stdin framing")
    if set(seed_inputs) != set(INPUT_CONTRACT):
        raise SupervisorError("seed input frame set differs")
    parts = [HELPER_MAGIC, helper_bytes, INPUT_MAGIC]
    for name, (digest, size) in INPUT_CONTRACT.items():
        raw = seed_inputs[name]
        if len(raw) != size or sha256_bytes(raw) != digest:
            raise SupervisorError(f"seed input frame identity differs: {name}")
        parts.append(raw)
    return b"".join(parts)


def _validate_generated_xml(raw: bytes) -> None:
    try:
        root = ET.fromstring(raw)
    except ET.ParseError as exc:
        raise SupervisorError("framed generated XML is malformed") from exc
    if root.tag != "case" or root.find("./casedef/geometry/definition[@dp='0.002']") is None:
        raise SupervisorError("framed XML case/dp contract differs")
    if len(root.findall(".//drawcylinder[@mask='2']")) != 1:
        raise SupervisorError("framed XML boundary mask=2 contract differs")
    keyed = {element.get("key"): element.get("value") for element in root.findall(".//parameter")}
    if keyed.get("MinFluidStop") != "1" or "PartsOutMax" in keyed:
        raise SupervisorError("framed XML MinFluidStop/PartsOutMax contract differs")
    if {"motion", "floating", "inout", "wavegen"}.intersection(element.tag for element in root.iter()):
        raise SupervisorError("framed XML enables a forbidden dynamic feature")


def _validate_generated_bi4(raw: bytes) -> None:
    if len(raw) < 94 or not raw.startswith(b"#FileJBD JPartDataBi4"):
        raise SupervisorError("framed BI4 file header/code differs")
    if raw[58] != 0x0A or raw[59] != 0 or raw[60] != 0 or raw[61] != 0 or raw[62:64] != b"\0\0":
        raise SupervisorError("framed BI4 byte-order/si64 header differs")
    size_item_definition = struct.unpack_from("<I", raw, 64)[0]
    if size_item_definition < 30 or size_item_definition > len(raw) - 68:
        raise SupervisorError("framed BI4 item-definition size is out of bounds")
    if struct.unpack_from("<I", raw, 68)[0] != 6 or raw[72:78] != b"\nITEM\n":
        raise SupervisorError("framed BI4 item-definition marker differs")
    if struct.unpack_from("<I", raw, 78)[0] != 12 or raw[82:94] != b"JPartDataBi4":
        raise SupervisorError("framed BI4 root item name differs")


def parse_success_frame(raw: bytes) -> dict[str, Any]:
    if not OUTPUT_HEADER.size <= len(raw) <= OUTPUT_FRAME_LIMIT:
        raise SupervisorError("guest success frame total length is out of bounds")
    magic, version, metadata_size, bi4_size, xml_size, console_size = OUTPUT_HEADER.unpack_from(raw)
    if magic != OUTPUT_MAGIC or version != OUTPUT_VERSION:
        raise SupervisorError("guest success frame magic/version differs")
    if not 1 <= metadata_size <= OUTPUT_METADATA_LIMIT:
        raise SupervisorError("guest success frame metadata length differs")
    if not 1 <= bi4_size <= OUTPUT_BI4_LIMIT or not 1 <= xml_size <= OUTPUT_XML_LIMIT:
        raise SupervisorError("guest success frame payload length differs")
    if not 0 <= console_size <= OUTPUT_CONSOLE_LIMIT:
        raise SupervisorError("guest success frame console length differs")
    expected_total = OUTPUT_HEADER.size + metadata_size + bi4_size + xml_size + console_size
    if len(raw) != expected_total:
        raise SupervisorError("guest success frame has truncation or trailing bytes")
    metadata_raw = raw[OUTPUT_HEADER.size:OUTPUT_HEADER.size + metadata_size]
    bi4_start = OUTPUT_HEADER.size + metadata_size
    xml_start = bi4_start + bi4_size
    bi4 = raw[bi4_start:xml_start]
    console_start = xml_start + xml_size
    xml = raw[xml_start:console_start]
    console_payload = raw[console_start:]
    try:
        metadata = json.loads(metadata_raw.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError("guest frame metadata is invalid") from exc
    if not isinstance(metadata, dict):
        raise SupervisorError("guest frame metadata is not an object")
    canonical = json.dumps(metadata, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    if canonical != metadata_raw:
        raise SupervisorError("guest frame metadata is not canonical JSON")
    required_keys = {
        "document_type",
        "status",
        "gencase_argv",
        "guest_inputs",
        "guest_identity",
        "guest_label",
        "candidate_label",
        "guest_work_tmpfs",
        "guest_outputs",
        "candidate_console",
        "stdin_frame_consumed_to_eof_then_replaced_by_guest_eof_pipe",
        "candidate_stdout_stderr_were_internal_pipe_only",
    }
    if set(metadata) != required_keys:
        raise SupervisorError("guest frame metadata key set differs")
    if metadata["document_type"] != "SMPCC_R8_LIQUID_U3_C1_GENCASE_GUEST_FRAME_V4" or metadata["status"] != "GUEST_FRAME_COMPLETE_PENDING_HOST_REAUDIT_AND_O_EXCL_EXPORT":
        raise SupervisorError("guest frame status/type differs")
    if metadata["gencase_argv"] != GENCASE_ARGV:
        raise SupervisorError("guest frame GenCase argv differs")
    if metadata["stdin_frame_consumed_to_eof_then_replaced_by_guest_eof_pipe"] is not True or metadata["candidate_stdout_stderr_were_internal_pipe_only"] is not True:
        raise SupervisorError("guest frame fd isolation evidence differs")
    expected_inputs = {
        name: {"sha256": digest, "size_bytes": size}
        for name, (digest, size) in INPUT_CONTRACT.items()
    }
    if metadata["guest_inputs"] != expected_inputs:
        raise SupervisorError("guest frame input identity differs")
    identity = metadata["guest_identity"]
    if (
        identity.get("uid") != [0, 0, 0, 0]
        or identity.get("gid") != [0, 0, 0, 0]
        or identity.get("groups") != []
        or identity.get("no_new_privs") != 1
        or set(identity.get("capabilities", {})) != {"CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb"}
        or any(identity["capabilities"].values())
    ):
        raise SupervisorError("guest frame final identity differs")
    expected_label = BOOTSTRAP_PROFILE + " (enforce)"
    if metadata["guest_label"] != expected_label or metadata["candidate_label"] != expected_label:
        raise SupervisorError("guest helper or GenCase AppArmor inheritance evidence differs")
    work_tmpfs = metadata["guest_work_tmpfs"]
    if (
        work_tmpfs.get("mountpoint") != "/work"
        or work_tmpfs.get("filesystem") != "tmpfs"
        or work_tmpfs.get("total_bytes") != 67_108_864
        or work_tmpfs.get("inode_ceiling_claimed") is not False
    ):
        raise SupervisorError("guest frame /work tmpfs evidence differs")
    payloads = {"C1_static.bi4": bi4, "C1_static.xml": xml}
    expected_outputs = {
        name: {"sha256": sha256_bytes(payload), "size_bytes": len(payload)}
        for name, payload in payloads.items()
    }
    if metadata["guest_outputs"] != expected_outputs:
        raise SupervisorError("guest frame output metadata/payload differs")
    console = metadata["candidate_console"]
    if set(console) != {"sha256", "size_bytes", "framed_for_separate_0440_console_log"} or console["framed_for_separate_0440_console_log"] is not True:
        raise SupervisorError("candidate console frame contract differs")
    if console.get("size_bytes") != len(console_payload) or console.get("sha256") != sha256_bytes(console_payload):
        raise SupervisorError("candidate console size evidence differs")
    _validate_generated_bi4(bi4)
    _validate_generated_xml(xml)
    return {"metadata": metadata, "payloads": payloads, "console": console_payload, "frame_sha256": sha256_bytes(raw), "frame_size_bytes": len(raw)}


def _label_from_attr(raw: str) -> str:
    stripped = raw.strip()
    return stripped.split(" (", 1)[0]


def labeled_processes(proc_root: Path = Path("/proc")) -> list[dict[str, Any]]:
    """Use AppArmor attr/current as authority; command names are never selectors."""

    observed: list[dict[str, Any]] = []
    for entry in proc_root.iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            raw = (entry / "attr/current").read_text(encoding="utf-8")
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError as exc:
            raise SupervisorError(f"cannot read AppArmor label for live-looking pid {entry.name}") from exc
        except OSError as exc:
            if exc.errno in (errno.ENOENT, errno.ESRCH):
                continue
            raise SupervisorError(f"unexpected attr/current scan error for pid {entry.name}") from exc
        label = _label_from_attr(raw)
        if label in LABELS:
            observed.append({"pid": int(entry.name), "label": label, "attr_current": raw.strip()})
    return sorted(observed, key=lambda item: item["pid"])


def require_stable_zero_labels(*, scans: int = 3, interval: float = 0.1) -> list[list[dict[str, Any]]]:
    history: list[list[dict[str, Any]]] = []
    for index in range(scans):
        current = labeled_processes()
        history.append(current)
        if current:
            raise SupervisorError(f"AppArmor-labeled process residue remains: {current}")
        if index + 1 < scans:
            time.sleep(interval)
    return history


def terminate_labeled_processes() -> dict[str, Any]:
    initial = labeled_processes()
    term_pidfds: list[int] = []
    for item in initial:
        try:
            pidfd = os.pidfd_open(item["pid"], 0)
            current = Path(f"/proc/{item['pid']}/attr/current").read_text(encoding="utf-8")
            if _label_from_attr(current) != item["label"]:
                os.close(pidfd)
                continue
            signal.pidfd_send_signal(pidfd, signal.SIGTERM)
            term_pidfds.append(pidfd)
        except (FileNotFoundError, ProcessLookupError):
            continue
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and labeled_processes():
        time.sleep(0.1)
    for descriptor in term_pidfds:
        os.close(descriptor)
    after_term = labeled_processes()
    kill_pidfds: list[int] = []
    for item in after_term:
        try:
            pidfd = os.pidfd_open(item["pid"], 0)
            current = Path(f"/proc/{item['pid']}/attr/current").read_text(encoding="utf-8")
            if _label_from_attr(current) != item["label"]:
                os.close(pidfd)
                continue
            signal.pidfd_send_signal(pidfd, signal.SIGKILL)
            kill_pidfds.append(pidfd)
        except (FileNotFoundError, ProcessLookupError):
            continue
    try:
        stable = require_stable_zero_labels()
    finally:
        for descriptor in kill_pidfds:
            os.close(descriptor)
    return {"initial": initial, "after_term": after_term, "stable_zero_scans": stable}


def snapshot_sysctls() -> dict[str, str]:
    values: dict[str, str] = {}
    for path in SYSCTL_PATHS:
        if path.exists():
            raw = path.read_text(encoding="ascii")
            if len(raw) > 128:
                raise SupervisorError(f"sysctl value is unexpectedly large: {path}")
            values[str(path)] = raw
    if not values:
        raise SupervisorError("no reviewed host sysctl is available")
    return values


def write_json_new(path: Path, value: Mapping[str, Any], *, mode: int = 0o440) -> dict[str, Any]:
    audit_root = Path("/home/zrj/scout_liquid_lab/audits")
    if path.parent != audit_root or os.geteuid() != 0:
        raise SupervisorError("receipt creation requires root and the exact audit directory")
    raw = (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + "\n").encode("utf-8")
    directory = os.open("/", os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        for component in audit_root.parts[1:]:
            child = os.open(component, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC, dir_fd=directory)
            os.close(directory)
            directory = child
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
            if (not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1
                    or metadata.st_uid != HOST_UID or metadata.st_gid != HOST_GID
                    or stat.S_IMODE(metadata.st_mode) != mode):
                raise SupervisorError("append-only receipt inode differs")
        finally:
            os.close(descriptor)
        os.fsync(directory)
    finally:
        os.close(directory)
    return {
        "path": str(path), "sha256": sha256_bytes(raw), "size_bytes": len(raw),
        "uid": HOST_UID, "gid": HOST_GID, "mode": f"{mode:04o}",
        "creation": "O_EXCL_NOFOLLOW_DIRFD_ANCHORED",
    }


def _mkdir_new(path: Path, mode: int) -> None:
    try:
        os.mkdir(path, mode)
    except FileExistsError as exc:
        raise SupervisorError(f"one-shot output directory already exists: {path}") from exc
    metadata = os.lstat(path)
    if not stat.S_ISDIR(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) != mode:
        raise SupervisorError(f"new output directory contract differs: {path}")


def verify_host_export_identity(status_path: Path = Path("/proc/self/status")) -> dict[str, Any]:
    fields: dict[str, str] = {}
    for line in status_path.read_text(encoding="ascii").splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            fields[key] = value.strip()
    try:
        uid = [int(value) for value in fields["Uid"].split()]
        gid = [int(value) for value in fields["Gid"].split()]
        groups = [int(value) for value in fields.get("Groups", "").split()]
        caps = {key: int(fields[key], 16) for key in ("CapInh", "CapPrm", "CapEff", "CapBnd", "CapAmb")}
        no_new_privs = int(fields["NoNewPrivs"])
    except (KeyError, ValueError) as exc:
        raise SupervisorError("cannot parse host export identity") from exc
    if uid != [HOST_UID] * 4 or gid != [HOST_GID] * 4 or groups:
        raise SupervisorError("host export UID/GID/group identity differs")
    if any(caps.values()) or no_new_privs != 1:
        raise SupervisorError("host export caps or NoNewPrivs identity differs")
    return {"uid": uid, "gid": gid, "groups": groups, "capabilities": caps, "no_new_privs": no_new_privs}


def loaded_profile_counts(profiles_path: Path = Path("/sys/kernel/security/apparmor/profiles")) -> dict[str, int]:
    counts = {name: 0 for name in LABELS}
    try:
        lines = profiles_path.read_text(encoding="utf-8").splitlines()
    except (PermissionError, OSError) as exc:
        raise SupervisorError("cannot prove AppArmor profile unload state") from exc
    for line in lines:
        name = line.split(" ", 1)[0]
        if name in counts:
            counts[name] += 1
    return counts


def open_directory_chain_nofollow(path: Path) -> int:
    if not path.is_absolute():
        raise SupervisorError("directory chain must be absolute")
    descriptor = os.open("/", os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        for component in path.parts[1:]:
            next_descriptor = os.open(
                component,
                os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
                dir_fd=descriptor,
            )
            os.close(descriptor)
            descriptor = next_descriptor
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


def mkdir_new_at(parent_fd: int, name: str, mode: int) -> int:
    if not name or "/" in name or name in (".", ".."):
        raise SupervisorError("unsafe one-shot directory basename")
    try:
        os.mkdir(name, mode, dir_fd=parent_fd)
    except FileExistsError as exc:
        raise SupervisorError(f"one-shot directory already exists: {name}") from exc
    descriptor = os.open(
        name,
        os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC,
        dir_fd=parent_fd,
    )
    metadata = os.fstat(descriptor)
    if not stat.S_ISDIR(metadata.st_mode) or stat.S_IMODE(metadata.st_mode) != mode:
        os.close(descriptor)
        raise SupervisorError(f"new one-shot directory mode differs: {name}")
    return descriptor


def consume_export_capability() -> dict[str, Any]:
    try:
        descriptor = int(os.environ[EXPORT_FD_ENV])
        expected_sha256 = os.environ[EXPORT_SHA256_ENV]
    except (KeyError, ValueError) as exc:
        raise SupervisorError("root export FD capability is absent") from exc
    if descriptor < 3 or len(expected_sha256) != 64:
        raise SupervisorError("root export FD capability metadata differs")
    payload = bytearray()
    try:
        metadata = os.fstat(descriptor)
        if (not stat.S_ISFIFO(metadata.st_mode) or metadata.st_uid != 0 or metadata.st_gid != 0
                or stat.S_IMODE(metadata.st_mode) != 0o600 or not os.get_inheritable(descriptor)):
            raise SupervisorError("root export pipe inode differs")
        while len(payload) < 32:
            block = os.read(descriptor, 32 - len(payload))
            if not block:
                raise SupervisorError("root export capability is truncated")
            payload.extend(block)
        if os.read(descriptor, 1) != b"":
            raise SupervisorError("root export capability has trailing bytes")
    finally:
        os.close(descriptor)
    if sha256_bytes(payload) != expected_sha256:
        raise SupervisorError("root export capability digest differs")
    os.environ.pop(EXPORT_FD_ENV, None)
    os.environ.pop(EXPORT_SHA256_ENV, None)
    return {"transport": "root_owned_anonymous_pipe_strict_eof", "sha256": expected_sha256, "size_bytes": 32}


def export_frame_o_excl(frame: bytes) -> dict[str, Any]:
    """UID-1000-only finalizer admitted by a root-owned one-use pipe."""

    verify_host_export_identity()
    capability = consume_export_capability()
    parsed = parse_success_frame(frame)
    cases_fd = open_directory_chain_nofollow(ATTEMPT_ROOT.parent)
    audits_fd = open_directory_chain_nofollow(CONSOLE_LOG.parent)
    attempt_fd = -1
    output_fd = -1
    exported: dict[str, dict[str, Any]] = {}
    try:
        attempt_fd = mkdir_new_at(cases_fd, ATTEMPT_ROOT.name, 0o700)
        output_fd = mkdir_new_at(attempt_fd, OUTPUT_ROOT.name, 0o700)
        for name in ("C1_static.bi4", "C1_static.xml"):
            payload = parsed["payloads"][name]
            descriptor = os.open(
                name,
                os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
                0o400,
                dir_fd=output_fd,
            )
            try:
                view = memoryview(payload)
                while view:
                    count = os.write(descriptor, view)
                    if count <= 0:
                        raise SupervisorError(f"short O_EXCL output write: {name}")
                    view = view[count:]
                os.fsync(descriptor)
                metadata = os.fstat(descriptor)
                if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
                    raise SupervisorError(f"O_EXCL output inode contract differs: {name}")
                os.fchmod(descriptor, 0o440)
                os.fsync(descriptor)
                exported[name] = {
                    "path": str(OUTPUT_ROOT / name),
                    "sha256": sha256_bytes(payload),
                    "size_bytes": len(payload),
                    "device": metadata.st_dev,
                    "inode": metadata.st_ino,
                    "nlink": metadata.st_nlink,
                    "mode_after_export": "0440",
                }
            finally:
                os.close(descriptor)
        if sorted(os.listdir(output_fd)) != ["C1_static.bi4", "C1_static.xml"]:
            raise SupervisorError("host output set differs after O_EXCL export")
        os.fsync(output_fd)
        os.fchmod(output_fd, 0o550)
        os.fchmod(attempt_fd, 0o550)
        os.fsync(output_fd)
        os.fsync(attempt_fd)
        console_descriptor = os.open(
            CONSOLE_LOG.name,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
            0o400,
            dir_fd=audits_fd,
        )
        try:
            view = memoryview(parsed["console"])
            while view:
                count = os.write(console_descriptor, view)
                if count <= 0:
                    raise SupervisorError("short O_EXCL console-log write")
                view = view[count:]
            os.fsync(console_descriptor)
            os.fchmod(console_descriptor, 0o440)
            os.fsync(console_descriptor)
        finally:
            os.close(console_descriptor)
        os.fsync(audits_fd)
        os.fsync(cases_fd)
    finally:
        if output_fd >= 0:
            os.close(output_fd)
        if attempt_fd >= 0:
            os.close(attempt_fd)
        os.close(audits_fd)
        os.close(cases_fd)
    return {
        "files": exported,
        "console_log": {"path": str(CONSOLE_LOG), "sha256": sha256_bytes(parsed["console"]), "size_bytes": len(parsed["console"]), "mode": "0440"},
        "frame_sha256": parsed["frame_sha256"],
        "root_cleanup_capability": capability,
        "classification": "SIM_ONLY_UNVALIDATED",
    }


def load_lifecycle_harness() -> Any:
    raw = read_regular_bytes(HARNESS_PATH)
    if sha256_bytes(raw) != V11_HARNESS_SHA256 or len(raw) != V11_HARNESS_SIZE:
        raise SupervisorError("frozen v11 lifecycle harness bytes differ")
    if IN_SNAPSHOT:
        metadata = os.lstat(HARNESS_PATH)
        if (metadata.st_uid != 0 or metadata.st_gid != 0 or metadata.st_nlink != 1
                or stat.S_IMODE(metadata.st_mode) != 0o444):
            raise SupervisorError("snapshot lifecycle harness metadata differs")
    spec = importlib.util.spec_from_file_location("_r8_gencase_v4_lifecycle_harness", HARNESS_PATH)
    if spec is None or spec.loader is None:
        raise SupervisorError("cannot load the frozen lifecycle harness")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.LABELS = LABELS
    module.BOOTSTRAP_PROFILE = BOOTSTRAP_PROFILE
    module.RUNTIME_PROFILE = RUNTIME_PROFILE
    module.HOST_UID = HOST_UID
    module.HOST_GID = HOST_GID
    module.SYSCTL_PATHS = SYSCTL_PATHS
    return module


def verify_root_snapshot(policy_sha256: str) -> tuple[dict[str, Any], dict[str, Any]]:
    if os.geteuid() != 0 or not IN_SNAPSHOT or SCRIPT_PATH != SNAPSHOT_ROOT / SUPERVISOR_NAME:
        raise SupervisorError("run requires the exact root-owned snapshot supervisor")
    root = os.lstat(SNAPSHOT_ROOT)
    if (not stat.S_ISDIR(root.st_mode) or root.st_uid != 0 or root.st_gid != 0
            or stat.S_IMODE(root.st_mode) != 0o555):
        raise SupervisorError("snapshot root metadata differs")
    names = sorted(os.listdir(SNAPSHOT_ROOT))
    expected_names = sorted(destination for _source, destination in SNAPSHOT_SOURCE_PAIRS)
    if names != expected_names:
        raise SupervisorError("snapshot file set differs")
    policy_raw = read_regular_bytes(POLICY_PATH)
    if len(policy_sha256) != 64 or sha256_bytes(policy_raw) != policy_sha256:
        raise SupervisorError("externally frozen policy digest differs")
    policy = json.loads(policy_raw.decode("utf-8"))
    schema = read_json_object(SCHEMA_PATH)
    if not isinstance(policy, dict) or schema.get("additionalProperties") is not False or set(policy) != set(schema.get("required", [])):
        raise SupervisorError("snapshot policy/schema contract differs")
    if policy.get("status") != "ADMITTED_SINGLE_GENCASE_RUNTIME_V4_AFTER_FROZEN_V11_ZERO_DENIAL_PROBE":
        raise SupervisorError("snapshot policy is not admitted")
    observed: dict[str, Any] = {}
    trusted = policy.get("trusted_artifacts", {})
    destination_to_kind = {
        GATE_NAME: "gate", HELPER_NAME: "helper", SUPERVISOR_NAME: "supervisor",
        PROFILE_NAME: "profile", SCHEMA_NAME: "schema",
    }
    for _source, name in SNAPSHOT_SOURCE_PAIRS:
        path = SNAPSHOT_ROOT / name
        metadata = os.lstat(path)
        if (not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != 0 or metadata.st_gid != 0
                or metadata.st_nlink != 1 or stat.S_IMODE(metadata.st_mode) != 0o444):
            raise SupervisorError(f"snapshot file metadata differs: {name}")
        raw = read_regular_bytes(path, limit=32 * 1024 * 1024)
        digest = sha256_bytes(raw)
        if name == POLICY_NAME:
            expected_digest, expected_size = policy_sha256, len(policy_raw)
        elif name == HARNESS_NAME:
            expected_digest, expected_size = V11_HARNESS_SHA256, V11_HARNESS_SIZE
        elif name in INPUT_CONTRACT:
            expected_digest, expected_size = INPUT_CONTRACT[name]
        else:
            entry = trusted.get(destination_to_kind.get(name, ""), {})
            expected_digest, expected_size = entry.get("sha256"), entry.get("size_bytes")
        if digest != expected_digest or len(raw) != expected_size:
            raise SupervisorError(f"snapshot file identity differs: {name}")
        observed[name] = {"path": str(path), "sha256": digest, "size_bytes": len(raw), "uid": 0, "gid": 0, "mode": "0444"}
    return policy, {"root": str(SNAPSHOT_ROOT), "mode": "0555", "files": observed}


def read_snapshot_inputs() -> dict[str, bytes]:
    if not IN_SNAPSHOT:
        raise SupervisorError("runtime inputs must come from the root snapshot")
    inputs: dict[str, bytes] = {}
    for name, (digest, size) in INPUT_CONTRACT.items():
        raw = read_regular_bytes(SNAPSHOT_ROOT / name, limit=16 * 1024 * 1024)
        if len(raw) != size or sha256_bytes(raw) != digest:
            raise SupervisorError(f"snapshot input differs: {name}")
        inputs[name] = raw
    return inputs


def create_root_capability() -> tuple[int, dict[str, Any]]:
    if os.geteuid() != 0:
        raise SupervisorError("root capability creation requires euid 0")
    read_fd, write_fd = os.pipe2(os.O_CLOEXEC)
    try:
        for descriptor in (read_fd, write_fd):
            metadata = os.fstat(descriptor)
            if (not stat.S_ISFIFO(metadata.st_mode) or metadata.st_uid != 0 or metadata.st_gid != 0
                    or stat.S_IMODE(metadata.st_mode) != 0o600):
                raise SupervisorError("root anonymous pipe metadata differs")
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
    return read_fd, {
        "transport": "ROOT_OWNED_ANONYMOUS_PIPE_SINGLE_STRICT_EOF_READ",
        "sha256": sha256_bytes(nonce), "size_bytes": 32,
        "pipe_uid": 0, "pipe_gid": 0, "pipe_mode": "0600",
    }


def gate_handoff_argv() -> list[str]:
    return [
        "/usr/bin/setpriv", "--reuid=1000", "--regid=1000", "--clear-groups",
        "--bounding-set=-all", "--inh-caps=-all", "--ambient-caps=-all", "--no-new-privs", "--",
        "/usr/bin/python3.12", "-I", "-B", "-S", str(GATE_PATH), "internal-run",
    ]


def export_handoff_argv() -> list[str]:
    return [
        "/usr/bin/setpriv", "--reuid=1000", "--regid=1000", "--clear-groups",
        "--bounding-set=-all", "--inh-caps=-all", "--ambient-caps=-all", "--no-new-privs", "--",
        "/usr/bin/python3.12", "-I", "-B", "-S", str(SCRIPT_PATH), "export-frame",
    ]


def command_evidence(result: Mapping[str, Any], *, stderr_prefix: bool = False) -> dict[str, Any]:
    stdout = result.get("stdout", b"")
    stderr = result.get("stderr", b"")
    evidence = {
        "argv": result.get("argv"), "returncode": result.get("returncode"),
        "stdin_size_bytes": result.get("stdin_size_bytes"),
        "stdin_fully_written": result.get("stdin_fully_written"),
        "stdout_size_bytes": len(stdout), "stdout_sha256": sha256_bytes(stdout),
        "stderr_size_bytes": len(stderr), "stderr_sha256": sha256_bytes(stderr),
        "failure": result.get("failure"), "start_new_session": result.get("start_new_session"),
    }
    if stderr_prefix:
        evidence["stderr_utf8_prefix"] = stderr[:4096].decode("utf-8", "replace")
    return evidence


def require_closed_zero_denial_audit(audit: Mapping[str, Any]) -> None:
    if (audit.get("capture_valid") is not True or audit.get("capture_errors") != []
            or audit.get("matching_total") != 0 or audit.get("stored_count") != 0
            or audit.get("dropped_count") != 0 or audit.get("storage_overflow") is not False
            or audit.get("expected_mount_total") != 0 or audit.get("mkdir_total") != 0
            or audit.get("unexpected_total") != 0 or audit.get("sanitized_denials") != []
            or audit.get("expected_mount_denials") != [] or audit.get("unexpected_denials") != []
            or not audit.get("start_cursor") or not audit.get("end_cursor")
            or audit.get("boot_id_before") != audit.get("boot_id_after")):
        raise SupervisorError("AppArmor journal window is not a closed zero-denial success window")


def verify_exported_outputs(parsed: Mapping[str, Any]) -> dict[str, Any]:
    attempt = os.lstat(ATTEMPT_ROOT)
    output = os.lstat(OUTPUT_ROOT)
    if (not stat.S_ISDIR(attempt.st_mode) or attempt.st_uid != HOST_UID or attempt.st_gid != HOST_GID
            or stat.S_IMODE(attempt.st_mode) != 0o550
            or not stat.S_ISDIR(output.st_mode) or output.st_uid != HOST_UID or output.st_gid != HOST_GID
            or stat.S_IMODE(output.st_mode) != 0o550):
        raise SupervisorError("exported directory metadata differs")
    if sorted(os.listdir(OUTPUT_ROOT)) != ["C1_static.bi4", "C1_static.xml"]:
        raise SupervisorError("exported output file set differs")
    observed: dict[str, Any] = {}
    for name, expected in parsed["payloads"].items():
        path = OUTPUT_ROOT / name
        metadata = os.lstat(path)
        raw = read_regular_bytes(path, limit=OUTPUT_BI4_LIMIT if name.endswith(".bi4") else OUTPUT_XML_LIMIT)
        if (not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != HOST_UID or metadata.st_gid != HOST_GID
                or metadata.st_nlink != 1 or stat.S_IMODE(metadata.st_mode) != 0o440 or raw != expected):
            raise SupervisorError(f"exported output identity differs: {name}")
        observed[name] = {"path": str(path), "sha256": sha256_bytes(raw), "size_bytes": len(raw), "mode": "0440"}
    console_metadata = os.lstat(CONSOLE_LOG)
    console = read_regular_bytes(CONSOLE_LOG, limit=OUTPUT_CONSOLE_LIMIT)
    if (console_metadata.st_uid != HOST_UID or console_metadata.st_gid != HOST_GID
            or console_metadata.st_nlink != 1 or stat.S_IMODE(console_metadata.st_mode) != 0o440
            or console != parsed["console"]):
        raise SupervisorError("exported console log identity differs")
    return {"files": observed, "console_log": {"path": str(CONSOLE_LOG), "sha256": sha256_bytes(console), "size_bytes": len(console), "mode": "0440"}}


def assert_one_shot_paths_absent() -> None:
    for path in (ATTEMPT_ROOT, START_RECEIPT, EXECUTION_RECEIPT, LIFECYCLE_RECEIPT, LIFECYCLE_FAILURE_RECEIPT, CONSOLE_LOG):
        _assert_absent(path)


def run_once(*, policy_sha256: str, admission_token: str) -> dict[str, Any]:
    if admission_token != ADMISSION_TOKEN:
        raise SupervisorError("explicit one-shot admission token differs")
    policy, snapshot = verify_root_snapshot(policy_sha256)
    assert_one_shot_paths_absent()
    harness = load_lifecycle_harness()
    with harness.TerminationGuard() as termination_guard:
        termination_guard.checkpoint()
        sudo_admission = harness.clear_invoking_user_sudo_timestamp()
        harness.validate_sudo_cleanup_evidence(sudo_admission)
        termination_guard.checkpoint()
        sysctls_before = harness.read_sysctls()
        if policy.get("profile_lifecycle", {}).get("expected_sysctls") != sysctls_before:
            raise SupervisorError("current sysctls differ from the frozen v4 baseline")
        profiles_before = harness.profile_state()
        harness.require_profile_counts(profiles_before, 0)
        initial_zero = harness.require_stable_zero_labels()
        audit_anchor = harness.capture_journal_anchor()
        start_document = {
            "document_type": "SMPCC_R8_LIQUID_U3_C1_GENCASE_V4_START_RECEIPT",
            "case_id": CASE_ID, "status": "ONE_SHOT_STARTED_NO_RETRY_SAME_IDENTITY",
            "policy_sha256": policy_sha256, "snapshot": snapshot,
            "sysctls_before": sysctls_before, "profiles_before": profiles_before,
            "initial_stable_zero_labels": initial_zero, "journal_anchor": audit_anchor,
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
            profile_path = str(PROFILE_PATH)
            harness.run_checked(["/usr/sbin/apparmor_parser", "-Q", "-K", "-T", profile_path], timeout_seconds=15, require_silent=True)
            termination_guard.checkpoint()
            load_attempted = True
            harness.run_checked(["/usr/sbin/apparmor_parser", "-a", "-K", "-T", profile_path], timeout_seconds=15, require_silent=True)
            loaded = harness.profile_state()
            harness.require_profile_counts(loaded, 1)
            helper_bytes = read_regular_bytes(HELPER_PATH)
            frame = build_fixed_input_frame(helper_bytes, read_snapshot_inputs(), policy)
            admission_fd, admission_capability = create_root_capability()
            gate_env = {
                "HOME": "/nonexistent", "PATH": "/usr/bin:/usr/sbin", "LC_ALL": "C.UTF-8",
                "LANG": "C.UTF-8", "TZ": "UTC0", SNAPSHOT_ENV: str(SNAPSHOT_ROOT),
                ADMISSION_FD_ENV: str(admission_fd), ADMISSION_SHA256_ENV: admission_capability["sha256"],
            }
            try:
                gate_result = harness.run_bounded_command(
                    gate_handoff_argv(), stdin_bytes=frame, stdout_limit=OUTPUT_FRAME_LIMIT,
                    stderr_limit=1_114_112, timeout_seconds=150, env=gate_env, pass_fds=(admission_fd,),
                )
            finally:
                os.close(admission_fd)
            termination_guard.checkpoint()
            prequery_cleanup = harness.terminate_labeled_processes()
            if (prequery_cleanup.get("initial") != [] or prequery_cleanup.get("term_sent") != []
                    or prequery_cleanup.get("after_term") != [] or prequery_cleanup.get("kill_sent") != []
                    or prequery_cleanup.get("stable_zero_scans") != [[], [], []]):
                raise SupervisorError("gate left AppArmor-labeled task residue")
            harness.require_profile_counts(harness.profile_state(), 1)
            harness.run_checked(["/usr/bin/journalctl", "--sync"], timeout_seconds=15, require_silent=True)
            audit = harness.capture_apparmor_denials(audit_anchor)
            audit["prequery_label_cleanup"] = prequery_cleanup
            audit["postquery_stable_zero_labels"] = harness.require_stable_zero_labels()
            harness.require_profile_counts(harness.profile_state(), 1)
            require_closed_zero_denial_audit(audit)
            if (gate_result.get("returncode") != 0 or gate_result.get("failure") is not None
                    or gate_result.get("stdin_fully_written") is not True or gate_result.get("stderr") != b""):
                raise SupervisorError("UID-1000 gate did not return one clean successful frame")
            parsed = parse_success_frame(gate_result["stdout"])
            execution_status = "PASS_U3_C1_GENCASE_V4_FRAME_VALID_CLEANUP_PENDING"
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
                    fallback_zero = harness.require_stable_zero_labels()
                    cleanup = {**cleanup, "stable_zero_scans": fallback_zero, "fallback_zero_scan_used": True}
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
                        stdout_limit=65_536, stderr_limit=65_536, timeout_seconds=15,
                    )
                    if unload["returncode"] != 0 or unload["failure"] is not None or unload["stdout"] or unload["stderr"]:
                        raise SupervisorError("transient AppArmor profile unload failed")
                profiles_after = harness.profile_state()
                harness.require_profile_counts(profiles_after, 0)
                cleanup["post_unload_stable_zero_scans"] = harness.require_stable_zero_labels()
            except BaseException as exc:
                cleanup_errors.append(f"profile_cleanup: {type(exc).__name__}: {exc}"[:4096])
            try:
                sysctls_after = harness.read_sysctls()
                if sysctls_after != sysctls_before:
                    raise SupervisorError("host sysctls changed across GenCase lifecycle")
            except BaseException as exc:
                cleanup_errors.append(f"sysctl_postcondition: {type(exc).__name__}: {exc}"[:4096])

        if parsed is not None and primary_error is None and not cleanup_errors:
            try:
                export_fd, export_capability = create_root_capability()
                export_env = {
                    "HOME": "/nonexistent", "PATH": "/usr/bin:/usr/sbin", "LC_ALL": "C.UTF-8",
                    "LANG": "C.UTF-8", "TZ": "UTC0", EXPORT_FD_ENV: str(export_fd),
                    EXPORT_SHA256_ENV: export_capability["sha256"],
                }
                try:
                    export_command = harness.run_bounded_command(
                        export_handoff_argv(), stdin_bytes=gate_result["stdout"], stdout_limit=131_072,
                        stderr_limit=65_536, timeout_seconds=30, env=export_env, pass_fds=(export_fd,),
                    )
                finally:
                    os.close(export_fd)
                if (export_command["returncode"] != 0 or export_command["failure"] is not None
                        or export_command["stderr"] or not export_command["stdin_fully_written"]):
                    raise SupervisorError("UID-1000 O_EXCL exporter failed")
                export_message = json.loads(export_command["stdout"].decode("utf-8"))
                if (not isinstance(export_message, dict) or export_message.get("status") != "PASS_UID1000_O_EXCL_EXPORT_V4"
                        or not isinstance(export_message.get("export"), dict)):
                    raise SupervisorError("UID-1000 exporter receipt frame differs")
                export_result = {
                    "command": command_evidence(export_command),
                    "capability": export_capability,
                    "guest_report": export_message["export"],
                }
                export_verification = verify_exported_outputs(parsed)
                execution_status = "PASS_U3_C1_GENCASE_V4_CASE_EXPORTED_CLEANUP_PENDING"
            except BaseException as exc:
                primary_error = f"{type(exc).__name__}: {exc}"[:4096]

        try:
            sudo_clear = harness.clear_invoking_user_sudo_timestamp()
            harness.validate_sudo_cleanup_evidence(sudo_clear)
        except BaseException as exc:
            cleanup_errors.append(f"sudo_timestamp: {type(exc).__name__}: {exc}"[:4096])

        execution_document = {
            "document_type": "SMPCC_R8_LIQUID_U3_C1_GENCASE_V4_EXECUTION_RECEIPT",
            "case_id": CASE_ID, "status": execution_status if primary_error is None and not cleanup_errors else "ATTEMPT_ABORTED_NO_RETRY_SAME_IDENTITY",
            "policy_sha256": policy_sha256, "start_receipt": start_record,
            "gate": None if gate_result is None else command_evidence(gate_result, stderr_prefix=True),
            "frame": None if parsed is None else {
                "sha256": parsed["frame_sha256"], "size_bytes": parsed["frame_size_bytes"],
                "metadata": parsed["metadata"],
                "payloads": {name: {"sha256": sha256_bytes(raw), "size_bytes": len(raw)} for name, raw in parsed["payloads"].items()},
            },
            "audit": audit, "export": export_result, "export_verification": export_verification,
            "primary_error": primary_error, "cleanup_errors": cleanup_errors,
            "production_authorized": False, "classification": "SIM_ONLY_UNVALIDATED",
        }
        execution_record = write_json_new(EXECUTION_RECEIPT, execution_document)
        success = (
            execution_document["status"] == "PASS_U3_C1_GENCASE_V4_CASE_EXPORTED_CLEANUP_PENDING"
            and not cleanup_errors and primary_error is None and parsed is not None
            and export_verification is not None
        )
        if success:
            lifecycle_document = {
                "document_type": "SMPCC_R8_LIQUID_U3_C1_GENCASE_V4_LIFECYCLE_RECEIPT",
                "case_id": CASE_ID, "status": "PASS_U3_C1_GENCASE_V4_LIFECYCLE_CLEANUP_AND_CASE_EXPORT",
                "execution_receipt": execution_record, "cleanup": cleanup,
                "profiles_after": profiles_after,
                "sysctls": {"before": sysctls_before, "after": sysctls_after, "unchanged": True},
                "sudo_timestamp": sudo_clear,
                "next_allowed_stage": "VALIDATE_AND_VISUALIZE_STATIC_CASE_THEN_SEPARATE_SOLVER_ADMISSION",
                "production_authorized": False,
            }
            lifecycle_record = write_json_new(LIFECYCLE_RECEIPT, lifecycle_document)
            return {"execution_status": execution_document["status"], "execution_receipt": execution_record, "lifecycle_receipt": lifecycle_record}
        failure_document = {
            "document_type": "SMPCC_R8_LIQUID_U3_C1_GENCASE_V4_LIFECYCLE_INCOMPLETE_RECEIPT",
            "case_id": CASE_ID, "status": "LIFECYCLE_OR_EXECUTION_INCOMPLETE_NO_RETRY_SAME_IDENTITY",
            "execution_receipt": execution_record, "primary_error": primary_error,
            "cleanup_errors": cleanup_errors, "cleanup_observation": cleanup,
            "profiles_after_observation": profiles_after, "sysctls_after_observation": sysctls_after,
            "sudo_timestamp_observation": sudo_clear, "production_authorized": False,
        }
        failure_record = write_json_new(LIFECYCLE_FAILURE_RECEIPT, failure_document)
        raise SupervisorError(f"v4 attempt failed closed; preserved {failure_record['path']}")


def export_frame_entry() -> dict[str, Any]:
    if os.geteuid() != HOST_UID or not IN_SNAPSHOT:
        raise SupervisorError("export-frame requires UID1000 and the exact root snapshot")
    frame = sys.stdin.buffer.read(OUTPUT_FRAME_LIMIT + 1)
    if len(frame) > OUTPUT_FRAME_LIMIT or sys.stdin.buffer.read(1) != b"":
        raise SupervisorError("export frame exceeds its fixed ceiling")
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
                "status": "PASS_V4_SUPERVISOR_STATIC_ADMITTED_EXECUTION_NOT_PERFORMED",
                "review": review,
                "snapshot_bootstrap": {
                    "source_sha256": SNAPSHOT_BOOTSTRAP_SHA256,
                    "source_size_bytes": len(SNAPSHOT_BOOTSTRAP_BYTES),
                    "loader_sha256": SNAPSHOT_BOOTSTRAP_LOADER_SHA256,
                    "loader_size_bytes": len(SNAPSHOT_BOOTSTRAP_LOADER_BYTES),
                    "loader_source": SNAPSHOT_BOOTSTRAP_LOADER_SOURCE,
                    "manifest_arguments": snapshot_manifest_arguments(review),
                },
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
            print(json.dumps({"status": "PASS_UID1000_O_EXCL_EXPORT_V4", "export": result}, ensure_ascii=False, sort_keys=True))
            return 0
        result = run_once(policy_sha256=arguments.policy_sha256, admission_token=arguments.admission_token)
        print(json.dumps({"status": "V4_ONE_SHOT_LIFECYCLE_RECORDED", "result": result}, ensure_ascii=False, sort_keys=True))
        return 0
    except (SupervisorError, OSError, ValueError, UnicodeError, subprocess.SubprocessError) as exc:
        target = sys.stderr if arguments.command in ("emit-bootstrap", "export-frame") else sys.stdout
        print(json.dumps({"status": "NO_GO", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=target)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
