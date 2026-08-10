#!/usr/bin/env python3
"""Root lifecycle supervisor and host frame auditor for U3 C1 GenCase v2.

The current v2 policy is static-only and ``run`` refuses before creating a
snapshot, directory, receipt, profile, namespace, or process.  The implemented
helpers freeze the intended successor contract: reviewed bytes are copied from
one O_NOFOLLOW descriptor into a root-owned snapshot, the seed/helper payload is
a fixed-length stdin frame, guest success is one exact stdout binary frame, and
UID 1000 performs O_EXCL host export only after both AppArmor labels are absent.
Execution and lifecycle receipts are separate and append-only.
"""

from __future__ import annotations

import argparse
import errno
import hashlib
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
SNAPSHOT_ENV = "R8_LIQUID_U3_GENCASE_V2_SNAPSHOT"
SNAPSHOT_DIR = Path(os.environ[SNAPSHOT_ENV]) if SNAPSHOT_ENV in os.environ else None
CASE_ID = "u3_c1_gencase_v2_20260807T140057Z"
HOST_UID = 1000
HOST_GID = 1000
BOOTSTRAP_PROFILE = "r8-liquid-u3-gencase-bootstrap-v2"
RUNTIME_PROFILE = "r8-liquid-u3-gencase-runtime-v2"
LABELS = (BOOTSTRAP_PROFILE, RUNTIME_PROFILE)

if SNAPSHOT_DIR is None:
    POLICY_PATH = PACKAGE_DIR / "config/target_hosts/liquid_zrj_msi_u2404_u3_gencase_execution_policy_v2.json"
    SCHEMA_PATH = PACKAGE_DIR / "schema/target_host_u3_gencase_execution_policy_v2.json"
    PROFILE_PATH = PACKAGE_DIR / "config/apparmor_drafts/r8-liquid-u3-gencase-v2.profile"
    GATE_PATH = SCRIPT_PATH.with_name("r8_liquid_target_u3_gencase_gate_v2.py")
    HELPER_PATH = SCRIPT_PATH.with_name("r8_liquid_u3_gencase_bootstrap_helper_v2.py")
else:
    POLICY_PATH = SNAPSHOT_DIR / "liquid_zrj_msi_u2404_u3_gencase_execution_policy_v2.json"
    SCHEMA_PATH = SNAPSHOT_DIR / "target_host_u3_gencase_execution_policy_v2.json"
    PROFILE_PATH = SNAPSHOT_DIR / "r8-liquid-u3-gencase-v2.profile"
    GATE_PATH = SNAPSHOT_DIR / "r8_liquid_target_u3_gencase_gate_v2.py"
    HELPER_PATH = SNAPSHOT_DIR / "r8_liquid_u3_gencase_bootstrap_helper_v2.py"

SEED_ID = "u3_c1_gencase_seed_v2_20260807T135341Z"
SEED_ROOT = Path(f"/home/zrj/scout_liquid_lab/dependency/runtime/{SEED_ID}.partial")
SEED_INPUT_ROOT = SEED_ROOT / "input"
SEED_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{SEED_ID}.json")
SEED_RECEIPT_SHA256 = "1bbf958dfe2f7ce026ce05d77e7ee2c2516c5d0ddc4345b021904e355003009d"
ATTEMPT_ROOT = Path(f"/home/zrj/scout_liquid_lab/cases/{CASE_ID}.partial")
OUTPUT_ROOT = ATTEMPT_ROOT / "output"
EXECUTION_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.execution.json")
LIFECYCLE_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.lifecycle.json")
LIFECYCLE_FAILURE_RECEIPT = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.lifecycle_incomplete.json")
CONSOLE_LOG = Path(f"/home/zrj/scout_liquid_lab/audits/{CASE_ID}.console.log")
SNAPSHOT_ROOT = Path(f"/run/r8-liquid-{CASE_ID}.snapshot")

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
HELPER_MAGIC = b"R8C1HELPERV2\0\0\0\0\0"
INPUT_MAGIC = b"R8C1INPUTV2\0\0\0\0\0"
OUTPUT_MAGIC = b"R8C1GENCASEV2\0\0\0"
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
    if policy.get("status") != "NO_GO_V2_PENDING_HARMLESS_RPX_SIGNAL_TIMEOUT_PROBE_AND_INDEPENDENT_STATIC_GO":
        raise SupervisorError("this supervisor is only the static NO-GO v2 revision")
    if policy.get("allowed_gate_commands") != ["self-check"]:
        raise SupervisorError("v2 command surface differs")
    artifacts = verify_static_artifact_hashes(policy)
    return {
        "policy": {"path": str(POLICY_PATH), "sha256": sha256_bytes(read_regular_bytes(POLICY_PATH))},
        "schema": {"path": str(SCHEMA_PATH), "sha256": artifacts["schema"]["sha256"]},
        "artifacts": artifacts,
        "production_run_allowed": False,
    }


def refuse_current_v2_run_before_mutation() -> None:
    policy = read_json_object(POLICY_PATH)
    if policy.get("status") != "ADMITTED_SINGLE_GENCASE_RUNTIME_V2_AFTER_PROBES":
        raise SupervisorError(
            "v2 is static-only: no snapshot, profile load, namespace, GenCase, output, or receipt may be created"
        )


def _assert_absent(path: Path) -> None:
    try:
        os.lstat(path)
    except FileNotFoundError:
        return
    raise SupervisorError(f"one-shot path already exists: {path}")


def create_root_owned_snapshot(policy: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    """Copy each already-hash-checked descriptor's bytes; never hash then reopen."""

    if os.geteuid() != 0:
        raise SupervisorError("root-owned snapshot creation requires euid 0")
    _assert_absent(SNAPSHOT_ROOT)
    os.mkdir(SNAPSHOT_ROOT, 0o700)
    sources = {**policy_artifact_paths(), "policy": POLICY_PATH}
    snapshot: dict[str, dict[str, Any]] = {}
    for name, source in sources.items():
        # One descriptor supplies the bytes used for hash verification and copy.
        raw = read_regular_bytes(source)
        if name != "policy":
            expected = policy["trusted_artifacts"][name]
            if sha256_bytes(raw) != expected["sha256"] or len(raw) != expected["size_bytes"]:
                raise SupervisorError(f"snapshot source identity differs: {name}")
        destination = SNAPSHOT_ROOT / source.name
        descriptor = os.open(
            destination,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
            0o444,
        )
        try:
            view = memoryview(raw)
            while view:
                count = os.write(descriptor, view)
                if count <= 0:
                    raise SupervisorError("short root-owned snapshot write")
                view = view[count:]
            os.fsync(descriptor)
            metadata = os.fstat(descriptor)
            if metadata.st_uid != 0 or metadata.st_gid != 0 or stat.S_IMODE(metadata.st_mode) != 0o444:
                raise SupervisorError("root-owned snapshot ownership/mode differs")
        finally:
            os.close(descriptor)
        copied = read_regular_bytes(destination)
        if copied != raw:
            raise SupervisorError("root-owned snapshot is not byte-identical")
        snapshot[name] = {"path": str(destination), "sha256": sha256_bytes(copied), "size_bytes": len(copied)}
    os.chmod(SNAPSHOT_ROOT, 0o555)
    return snapshot


def verify_seed_receipt_and_read_inputs() -> dict[str, bytes]:
    receipt_raw = read_regular_bytes(SEED_RECEIPT, limit=64 * 1024)
    if sha256_bytes(receipt_raw) != SEED_RECEIPT_SHA256:
        raise SupervisorError("v2 seed receipt digest differs")
    try:
        receipt = json.loads(receipt_raw.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise SupervisorError("v2 seed receipt is invalid") from exc
    if receipt.get("status") != "PASS_NONEXECUTABLE_GENCASE_SEED_V2_MATERIALIZATION":
        raise SupervisorError("v2 seed receipt is not the required non-executing PASS")
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
            raise SupervisorError(f"v2 seed receipt violates non-execution: {field}")
    result: dict[str, bytes] = {}
    names = sorted(entry.name for entry in os.scandir(SEED_INPUT_ROOT))
    if names != sorted(INPUT_CONTRACT):
        raise SupervisorError("v2 seed input file set differs")
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
            raise SupervisorError(f"v2 seed file contract differs: {name}")
        raw = read_regular_bytes(path, limit=16 * 1024 * 1024)
        if sha256_bytes(raw) != digest:
            raise SupervisorError(f"v2 seed file digest differs: {name}")
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
        "guest_work_tmpfs",
        "guest_outputs",
        "candidate_console",
        "stdin_frame_consumed_to_eof_then_replaced_by_guest_eof_pipe",
        "candidate_stdout_stderr_were_internal_pipe_only",
    }
    if set(metadata) != required_keys:
        raise SupervisorError("guest frame metadata key set differs")
    if metadata["document_type"] != "SMPCC_R8_LIQUID_U3_C1_GENCASE_GUEST_FRAME_V1" or metadata["status"] != "GUEST_FRAME_COMPLETE_PENDING_HOST_REAUDIT_AND_O_EXCL_EXPORT":
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


def write_json_new(path: Path, value: Mapping[str, Any], *, mode: int = 0o440) -> None:
    raw = (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2) + "\n").encode("utf-8")
    descriptor = os.open(
        path,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC,
        0o600,
    )
    try:
        view = memoryview(raw)
        while view:
            count = os.write(descriptor, view)
            if count <= 0:
                raise SupervisorError("short append-only receipt write")
            view = view[count:]
        os.fsync(descriptor)
        os.fchmod(descriptor, mode)
    finally:
        os.close(descriptor)


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


def export_frame_o_excl(frame: bytes) -> dict[str, Any]:
    """UID-1000-only finalizer; caller must first prove both labels stably zero."""

    verify_host_export_identity()
    require_stable_zero_labels()
    profile_counts = loaded_profile_counts()
    if any(profile_counts.values()):
        raise SupervisorError(f"v2 AppArmor profile remains loaded before host export: {profile_counts}")
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
        os.fchmod(output_fd, 0o550)
        os.fchmod(attempt_fd, 0o550)
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
        finally:
            os.close(console_descriptor)
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
        "profiles_unloaded_before_export": profile_counts,
        "classification": "SIM_ONLY_UNVALIDATED",
    }


def lifecycle_receipt_contract(
    *,
    execution_receipt_sha256: str,
    sysctls_before: Mapping[str, str],
    sysctls_after: Mapping[str, str],
    cleanup: Mapping[str, Any],
    profiles_after_aa_status: Mapping[str, int],
    sudo_k_returncode: int,
) -> dict[str, Any]:
    if sysctls_before != sysctls_after:
        raise SupervisorError("host sysctls changed across the lifecycle")
    if any(profiles_after_aa_status.get(name) != 0 for name in LABELS):
        raise SupervisorError("aa-status still reports a v2 profile")
    if cleanup.get("stable_zero_scans") is None or sudo_k_returncode != 0:
        raise SupervisorError("lifecycle cleanup or sudo timestamp clear is incomplete")
    return {
        "document_type": "SMPCC_R8_LIQUID_U3_C1_GENCASE_LIFECYCLE_RECEIPT",
        "case_id": CASE_ID,
        "status": "PASS_U3_C1_GENCASE_V2_LIFECYCLE_CLEANUP",
        "execution_receipt": str(EXECUTION_RECEIPT),
        "execution_receipt_sha256": execution_receipt_sha256,
        "labeled_processes_after": [],
        "profiles_after_aa_status": dict(profiles_after_aa_status),
        "host_sysctls": {"before": dict(sysctls_before), "after": dict(sysctls_after), "unchanged": True},
        "sudo_timestamp_clear": {"command": ["/usr/bin/sudo", "-k"], "returncode": sudo_k_returncode},
        "next_allowed_stage": "SEPARATE_COMPILED_SOLVER_RUNTIME_ADMISSION_REQUIRED",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", "run"))
    arguments = parser.parse_args()
    try:
        if arguments.command == "self-check":
            review = verify_policy_static()
            print(json.dumps({"status": "PASS_SUPERVISOR_STATIC_DRAFT_NO_GO", "review": review}, ensure_ascii=False, sort_keys=True))
            return 0
        # This is intentionally the first run action.  It fails before euid
        # checks or any state-changing operation under the current v2 policy.
        refuse_current_v2_run_before_mutation()
        raise SupervisorError("unreachable: production orchestration belongs to a fresh post-probe revision")
    except (SupervisorError, OSError, ValueError, UnicodeError, subprocess.SubprocessError) as exc:
        print(json.dumps({"status": "NO_GO", "error": str(exc)}, ensure_ascii=False, sort_keys=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
