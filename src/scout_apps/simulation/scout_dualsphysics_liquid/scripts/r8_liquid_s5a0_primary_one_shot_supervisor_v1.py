#!/usr/bin/env python3
"""One-shot host supervisor for the exact S5A0 primary bag.

``self-check`` is static-only.  ``run`` requires the frozen execution id and
is the only path that may read the exact primary or launch bubblewrap.  The
supervisor exposes one read-only bag and one dedicated evidence directory,
mounts neither procfs nor devfs, freezes host before/after evidence, validates
the worker receipt twice, and atomically publishes a mode-0600 receipt without
replacement.
"""

from __future__ import annotations

import argparse
import copy
import ctypes
import errno
import hashlib
import json
import os
import stat
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence, TextIO

from jsonschema import Draft202012Validator

# ``python -I`` intentionally omits the script directory.  Add back only the
# resolved directory containing this frozen supervisor so its three sibling
# validators remain importable without accepting cwd, PYTHONPATH or user-site
# injection.
MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_ros1_bag_v2_reader_v1 as bag_reader
import r8_liquid_s5a0_selected_bag_intake_gate_v1 as gate_v1
import r8_liquid_s5a0_selected_bag_intake_gate_v2 as gate_v2


sys.dont_write_bytecode = True

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
POLICY_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5a0_primary_supervisor_policy_v1.json"
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_s5a0_primary_supervisor_execution_receipt_v1.json"
EXECUTION_ID = "s5a0_primary_bsmooth_b01_20260811T173630Z_v1"
PASS_STATUS = "PASS_S5A0_PRIMARY_ONE_SHOT_DEVELOPMENT_ONLY"
FAIL_STATUS = "STOP_AND_PRESERVE_EVIDENCE"
MAX_STATIC_BYTES = 2 * 1024 * 1024
AT_FDCWD = -100
RENAME_NOREPLACE = 1


class SupervisorError(ValueError):
    def __init__(self, code: str, phase: str, message: str) -> None:
        super().__init__(f"{code}: {message}")
        self.code = code
        self.phase = phase
        self.message = message


def _fail(code: str, phase: str, message: str) -> None:
    raise SupervisorError(code, phase, message)


def _read_json(path: Path, maximum: int, label: str) -> tuple[dict[str, Any], bytes]:
    try:
        raw = path.read_bytes()
    except OSError as exc:
        _fail("JSON_READ_FAILED", "static", f"cannot read {label}: {exc}")
    if not raw or len(raw) > maximum:
        _fail("JSON_SIZE_INVALID", "static", f"{label} is empty or oversized")
    try:
        value = json.loads(raw)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        _fail("JSON_INVALID", "static", f"{label} is invalid: {exc}")
    if not isinstance(value, dict):
        _fail("JSON_ROOT_INVALID", "static", f"{label} root is not an object")
    return value, raw


def _sha256_file(path: Path, maximum: int = 64 * 1024 * 1024) -> str:
    try:
        metadata = path.stat()
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_size > maximum:
            _fail("HASH_INPUT_INVALID", "preflight", f"hash input is not bounded regular: {path}")
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            while True:
                block = stream.read(1024 * 1024)
                if not block:
                    break
                digest.update(block)
        return digest.hexdigest()
    except OSError as exc:
        _fail("HASH_INPUT_FAILED", "preflight", f"cannot hash {path}: {exc}")


def load_policy(path: Path = POLICY_PATH) -> tuple[dict[str, Any], str]:
    policy, raw = _read_json(path, MAX_STATIC_BYTES, "supervisor policy")
    validate_policy(policy)
    return policy, hashlib.sha256(raw).hexdigest()


def load_schema(path: Path = SCHEMA_PATH) -> tuple[dict[str, Any], str]:
    schema, raw = _read_json(path, MAX_STATIC_BYTES, "execution receipt schema")
    gate_v1.assert_deep_closed(schema)
    Draft202012Validator.check_schema(schema)
    return schema, hashlib.sha256(raw).hexdigest()


def validate_policy(policy: Mapping[str, Any]) -> None:
    required = {"schema_version", "document_type", "execution_id", "status", "selection", "inputs", "tools", "paths", "limits", "contract", "expected_evidence_inventory"}
    if set(policy) != required:
        _fail("POLICY_KEYS_INVALID", "static", "supervisor policy root is not closed")
    if policy["schema_version"] != "smpcc-r8-liquid-s5a0-primary-supervisor-policy-v1" or policy["execution_id"] != EXECUTION_ID:
        _fail("POLICY_IDENTITY_INVALID", "static", "policy version/execution id differs")
    selection = policy["selection"]
    expected_path = "/home/zrj/slosh_bags/matrix_bags/SIM-S1_CORE_H1_C1_Bsmooth_b01_r01/capture.bag"
    if selection.get("absolute_path") != expected_path or selection.get("selected_role") != "PRIMARY" or selection.get("selected_count") != 1:
        _fail("SELECTION_INVALID", "static", "exact primary selection differs")
    if selection.get("optional_pair_authorized") is not False:
        _fail("OPTIONAL_AUTHORIZED", "static", "optional pair is not allowed")
    try:
        gate_v1._absolute_normalized(selection["absolute_path"], "primary path")
        gate_v1._relative_normalized(selection["relative_path"], "primary relative path")
    except gate_v1.S5A0GateError as exc:
        _fail("SELECTION_PATH_INVALID", "static", str(exc))
    if os.path.join(selection["source_root"], selection["relative_path"]) != selection["absolute_path"]:
        _fail("PATH_BINDING_INVALID", "static", "absolute/relative primary binding differs")
    if not gate_v1._is_lower_hex(selection.get("expected_sha256"), 64):
        _fail("SOURCE_HASH_INVALID", "static", "expected primary hash is invalid")
    contract = policy["contract"]
    true_flags = {
        "one_shot", "exact_path_only", "glob_forbidden", "dedicated_evidence_root_only",
        "audit_root_bind_forbidden", "source_read_only_bind", "network_unshared",
        "proc_mount_forbidden", "dev_mount_forbidden", "inner_receipt_required",
        "host_closed_schema_validation", "final_receipt_o_excl", "atomic_noreplace_publish",
        "failure_preserves_partial",
    }
    if any(contract.get(name) is not True for name in true_flags):
        _fail("CONTRACT_CONTROL_DISABLED", "static", "required fail-closed control is disabled")
    for name in ("source_bag_executed", "ros_started", "gpu_exposed", "solver_started", "sudo_used"):
        if contract.get(name) is not False:
            _fail("FORBIDDEN_ACTION_ENABLED", "static", f"{name} is enabled")
    if contract.get("final_receipt_mode") != "0600":
        _fail("RECEIPT_MODE_INVALID", "static", "final receipt mode is not 0600")
    paths = policy["paths"]
    audit = Path(gate_v1._absolute_normalized(paths["audit_root"], "audit root"))
    for name in ("partial_evidence_root", "final_evidence_root", "final_receipt_path", "failure_receipt_path"):
        candidate = Path(gate_v1._absolute_normalized(paths[name], name))
        if candidate.parent != audit:
            _fail("OUTPUT_SCOPE_INVALID", "static", f"{name} is outside/directly below audit root")
    if len(set(paths.values())) != len(paths):
        _fail("OUTPUT_ALIAS", "static", "supervisor output paths/names alias")
    if sorted(policy["expected_evidence_inventory"]) != sorted([
        paths["reservation_name"], paths["inner_receipt_name"], paths["stdout_name"], paths["stderr_name"], paths["rc_name"]
    ]):
        _fail("EVIDENCE_INVENTORY_INVALID", "static", "expected evidence inventory differs")
    for group in (policy["inputs"].values(), policy["tools"]):
        for item in group:
            gate_v1._absolute_normalized(item["path"], "static input/tool path")
            if not gate_v1._is_lower_hex(item["sha256"], 64):
                _fail("STATIC_HASH_INVALID", "static", "static input/tool hash is invalid")


def verify_static_inputs(policy: Mapping[str, Any]) -> list[dict[str, Any]]:
    for name, item in policy["inputs"].items():
        observed = _sha256_file(Path(item["path"]), MAX_STATIC_BYTES)
        if observed != item["sha256"]:
            _fail("STATIC_INPUT_HASH_DRIFT", "preflight", f"{name} hash differs")
    witnesses = []
    for item in policy["tools"]:
        observed = _sha256_file(Path(item["path"]))
        if observed != item["sha256"]:
            _fail("TOOL_HASH_DRIFT", "preflight", f"{item['name']} hash differs")
        witnesses.append({"name": item["name"], "path": item["path"], "expected_sha256": item["sha256"], "observed_sha256": observed, "matched": True})
    return witnesses


def _decode_mount(value: str) -> str:
    for encoded, decoded in (("\\040", " "), ("\\011", "\t"), ("\\012", "\n"), ("\\134", "\\")):
        value = value.replace(encoded, decoded)
    return value


def host_mount_identity(path: Path, mountinfo_text: str | None = None) -> dict[str, Any]:
    selected = gate_v1._absolute_normalized(str(path), "host mount target")
    if mountinfo_text is None:
        mountinfo_text = Path("/proc/self/mountinfo").read_text(encoding="utf-8")
    candidates = []
    for line in mountinfo_text.splitlines():
        fields = line.split()
        if "-" not in fields:
            continue
        separator = fields.index("-")
        if separator < 6 or len(fields) <= separator + 3:
            continue
        mount_point = _decode_mount(fields[4])
        if selected != mount_point and not selected.startswith(mount_point.rstrip("/") + "/"):
            continue
        try:
            mount_id = int(fields[0])
        except ValueError:
            continue
        candidates.append({
            "mount_id": mount_id, "device": fields[2], "root": _decode_mount(fields[3]),
            "mount_point": mount_point, "filesystem_type": fields[separator + 1],
            "mount_source": _decode_mount(fields[separator + 2]),
            "mount_options": sorted(fields[5].split(",")), "super_options": sorted(fields[separator + 3].split(",")),
        })
    if not candidates:
        _fail("HOST_MOUNT_MISSING", "source", "no host mount covers the primary")
    return max(candidates, key=lambda item: len(item["mount_point"]))


def _snapshot_summary(snapshot: Mapping[str, Any]) -> dict[str, Any]:
    return {"root": snapshot["root"], "entry_count": snapshot["entry_count"], "sha256": snapshot["sha256"]}


def capture_source_state(policy: Mapping[str, Any], mountinfo_text: str | None = None) -> dict[str, Any]:
    selection = policy["selection"]
    source = Path(selection["absolute_path"])
    chain = gate_v1.audit_path_chain(source, policy["limits"]["maximum_parent_components"])
    mount = host_mount_identity(source, mountinfo_text)
    try:
        _data, before, after = bag_reader.read_regular_file(
            source,
            limits=gate_v1.reader_limits_from_policy(gate_v1.load_policy()[0]),
            expected_sha256=selection["expected_sha256"],
            expected_size_bytes=selection["expected_size_bytes"],
            expected_mode=int(selection["expected_mode"], 8),
        )
    except bag_reader.BagV2Error as exc:
        _fail("HOST_PRIMARY_IDENTITY_INVALID", "source", str(exc))
    if before != after:
        _fail("HOST_PRIMARY_CHANGED_DURING_HASH", "source", "primary changed during host hash")
    # Inventory only the exact selected attempt directory.  Walking the corpus
    # root would enumerate the separately unauthorized optional bag and the
    # other 86 out-of-scope attempts.  The sandbox argv independently proves
    # that no source-root path is writable.
    snapshot = gate_v1.snapshot_source_root(source.parent, policy["limits"]["maximum_source_entries"])
    return {"path_chain": chain, "mount": mount, "file": before, "root": _snapshot_summary(snapshot)}


def _normalised_argv_sha256(argv: Sequence[str]) -> str:
    normalized = list(argv)
    marker = normalized.index("--expected-argv-sha256")
    normalized[marker + 1] = "0" * 64
    return gate_v1.canonical_sha256(normalized)


def build_bwrap_argv(policy: Mapping[str, Any]) -> tuple[list[str], str]:
    validate_policy(policy)
    selection, paths, limits = policy["selection"], policy["paths"], policy["limits"]
    inputs = policy["inputs"]
    argv = [
        "/usr/bin/bwrap", "--die-with-parent", "--new-session", "--unshare-all", "--unshare-net", "--clearenv", "--cap-drop", "ALL",
        "--ro-bind", "/usr", "/usr", "--ro-bind", "/lib", "/lib", "--ro-bind", "/lib64", "/lib64",
        "--tmpfs", "/tmp", "--dir", "/app",
        "--ro-bind", inputs["reader"]["path"], "/app/r8_liquid_ros1_bag_v2_reader_v1.py",
        "--ro-bind", inputs["gate_v1"]["path"], "/app/r8_liquid_s5a0_selected_bag_intake_gate_v1.py",
        "--ro-bind", inputs["gate_v2"]["path"], "/app/r8_liquid_s5a0_selected_bag_intake_gate_v2.py",
        "--ro-bind", inputs["worker_v3"]["path"], "/app/worker.py",
        "--dir", "/policy", "--ro-bind", inputs["base_policy"]["path"], "/policy/base.json",
        "--dir", "/schema", "--ro-bind", inputs["inner_schema"]["path"], "/schema/inner.json",
        "--dir", "/selected", "--ro-bind", selection["absolute_path"], "/selected/capture.bag",
        "--dir", "/evidence", "--bind", paths["partial_evidence_root"], "/evidence",
        "--setenv", "PATH", "/usr/bin", "--setenv", "HOME", "/nonexistent",
        "--setenv", "PYTHONDONTWRITEBYTECODE", "1", "--setenv", "PYTHONHASHSEED", "0", "--chdir", "/app", "--",
        "/usr/bin/timeout", "--foreground", "--signal=KILL", f"--kill-after={limits['kill_after_seconds']}s", f"{limits['timeout_seconds']}s",
        "/usr/bin/prlimit", f"--rss={limits['maximum_rss_bytes']}:{limits['maximum_rss_bytes']}",
        f"--as={limits['maximum_address_space_bytes']}:{limits['maximum_address_space_bytes']}",
        f"--fsize={max(limits['maximum_stdout_bytes'], limits['maximum_stderr_bytes'])}:{max(limits['maximum_stdout_bytes'], limits['maximum_stderr_bytes'])}",
        f"--nofile={limits['nofile_limit']}:{limits['nofile_limit']}", f"--nproc={limits['nproc_limit']}:{limits['nproc_limit']}",
        "/usr/bin/python3", "-I", "-B", "/app/worker.py", "sandbox-inspect",
        "--policy", "/policy/base.json", "--schema", "/schema/inner.json", "--source", "/selected/capture.bag",
        "--receipt", "/evidence/selected_bag_inner_receipt_v1.json", "--final-evidence-root", paths["final_evidence_root"],
        "--expected-argv-sha256", "0" * 64,
    ]
    digest = _normalised_argv_sha256(argv)
    argv[argv.index("--expected-argv-sha256") + 1] = digest
    forbidden = [selection["forbidden_optional_path"], "--proc", "--dev", "sudo", "roscore", "rosbag", "nvidia-smi"]
    if any(value in argv for value in forbidden) or paths["audit_root"] in argv:
        _fail("BWRAP_EXPOSURE_INVALID", "static", "argv exposes a forbidden path/action")
    if argv.count(selection["absolute_path"]) != 1 or argv.count(paths["partial_evidence_root"]) != 1:
        _fail("BWRAP_BIND_COUNT_INVALID", "static", "source/evidence bind count differs")
    return argv, digest


def _write_all(descriptor: int, raw: bytes) -> None:
    offset = 0
    while offset < len(raw):
        written = os.write(descriptor, raw[offset:])
        if written <= 0:
            _fail("WRITE_STALLED", "evidence", "bounded write made no progress")
        offset += written


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(
        path,
        os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_CLOEXEC", 0),
    )
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _write_json_excl(path: Path, value: object, maximum: int, mode: int = 0o600) -> None:
    raw = gate_v1._canonical_bytes(value) + b"\n"
    if len(raw) > maximum:
        _fail("OUTPUT_OVERSIZED", "evidence", f"JSON output exceeds bound: {path}")
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0)
    try:
        descriptor = os.open(path, flags, mode)
    except OSError as exc:
        _fail("OUTPUT_CREATE_NEW_FAILED", "evidence", f"cannot create {path}: {exc}")
    try:
        os.fchmod(descriptor, mode)
        _write_all(descriptor, raw)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    _fsync_directory(path.parent)


def atomic_publish_json(path: Path, value: object, maximum: int) -> None:
    temporary = path.with_name(f".{path.name}.publishing")
    _write_json_excl(temporary, value, maximum, 0o600)
    try:
        os.link(temporary, path, follow_symlinks=False)
    except OSError as exc:
        _fail("ATOMIC_PUBLISH_FAILED", "publish", f"cannot publish {path}: {exc}")
    os.unlink(temporary)
    _fsync_directory(path.parent)
    metadata = os.lstat(path)
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or stat.S_IMODE(metadata.st_mode) != 0o600:
        _fail("PUBLISHED_RECEIPT_INVALID", "publish", "published receipt identity/mode differs")


def _rename_noreplace(source: Path, destination: Path) -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, "renameat2", None)
    if renameat2 is None:
        _fail("RENAME_NOREPLACE_UNAVAILABLE", "finalize", "libc renameat2 is unavailable")
    result = renameat2(AT_FDCWD, os.fsencode(source), AT_FDCWD, os.fsencode(destination), RENAME_NOREPLACE)
    if result != 0:
        error = ctypes.get_errno()
        _fail("EVIDENCE_RENAME_FAILED", "finalize", os.strerror(error))
    _fsync_directory(destination.parent)


def create_evidence_root(policy: Mapping[str, Any]) -> Path:
    paths = policy["paths"]
    audit, partial = Path(paths["audit_root"]), Path(paths["partial_evidence_root"])
    metadata = os.lstat(audit)
    if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISDIR(metadata.st_mode):
        _fail("AUDIT_ROOT_INVALID", "preflight", "audit root is not a non-symlink directory")
    targets = [partial, Path(paths["final_evidence_root"]), Path(paths["final_receipt_path"]), Path(paths["failure_receipt_path"])]
    for target in targets:
        try:
            os.lstat(target)
        except FileNotFoundError:
            continue
        _fail("AUDIT_IDENTITY_EXISTS", "preflight", f"create-new target already exists: {target}")
    try:
        os.mkdir(partial, 0o700)
    except OSError as exc:
        _fail("PARTIAL_ROOT_CREATE_FAILED", "preflight", str(exc))
    _fsync_directory(audit)
    _write_json_excl(partial / paths["reservation_name"], {"execution_id": EXECUTION_ID}, 4096, 0o600)
    return partial


def _file_witness(
    path: Path,
    reported_path: Path | None = None,
    maximum_bytes: int | None = None,
) -> dict[str, Any]:
    metadata = os.lstat(path)
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
        _fail("EVIDENCE_FILE_INVALID", "validate", f"not a single-link regular file: {path}")
    if maximum_bytes is not None and metadata.st_size > maximum_bytes:
        _fail("EVIDENCE_FILE_OVERSIZED", "validate", f"evidence file exceeds bound: {path}")
    return {"path": str(reported_path or path), "size_bytes": metadata.st_size, "mode": f"{stat.S_IMODE(metadata.st_mode):04o}", "nlink": 1, "sha256": _sha256_file(path)}


def _collect_existing_evidence(policy: Mapping[str, Any], context: dict[str, Any]) -> None:
    partial = Path(policy["paths"]["partial_evidence_root"])
    if not partial.is_dir():
        return
    limits = policy["limits"]
    bounds = {
        policy["paths"]["stdout_name"]: limits["maximum_stdout_bytes"],
        policy["paths"]["stderr_name"]: limits["maximum_stderr_bytes"],
        policy["paths"]["inner_receipt_name"]: limits["maximum_inner_receipt_bytes"],
        policy["paths"]["rc_name"]: 4096,
        policy["paths"]["reservation_name"]: 4096,
    }
    witnesses: dict[str, dict[str, Any]] = {}
    for name in sorted(policy["expected_evidence_inventory"]):
        candidate = partial / name
        if candidate.is_file():
            witnesses[name] = _file_witness(candidate, maximum_bytes=bounds[name])
    context["inventory"] = [witnesses[name] for name in sorted(witnesses)]
    mapping = {
        "inner_receipt": policy["paths"]["inner_receipt_name"],
        "stdout": policy["paths"]["stdout_name"],
        "stderr": policy["paths"]["stderr_name"],
        "rc_file": policy["paths"]["rc_name"],
    }
    for field, name in mapping.items():
        if name in witnesses:
            context["worker"][field] = witnesses[name]


def _empty_context(policy: Mapping[str, Any], policy_sha: str, schema_sha: str) -> dict[str, Any]:
    selection, paths, limits = policy["selection"], policy["paths"], policy["limits"]
    return {
        "policy_sha": policy_sha, "schema_sha": schema_sha, "static_verified": None,
        "tools": [{"name": item["name"], "path": item["path"], "expected_sha256": item["sha256"], "observed_sha256": None, "matched": None} for item in policy["tools"]],
        "before": None, "after": None, "argv": [], "argv_sha": gate_v1.canonical_sha256([]),
        "worker": {"started": False, "return_code": None, "timed_out": False, "inner_receipt_schema_valid": None, "inner_receipt_status": None, "inner_receipt": None, "stdout": None, "stderr": None, "rc_file": None},
        "partial_created": False, "reservation": False, "partial_preserved": False, "renamed": False, "inventory": [],
    }


def build_execution_receipt(policy: Mapping[str, Any], schema_sha: str, context: Mapping[str, Any], *, failure: SupervisorError | None, published_path: str | None) -> dict[str, Any]:
    selection, paths, limits = policy["selection"], policy["paths"], policy["limits"]
    before, after = context["before"], context["after"]
    unchanged = before is not None and after is not None
    host = {
        "path_chain_before": before["path_chain"] if before else [], "path_chain_after": after["path_chain"] if after else [],
        "mount_before": before["mount"] if before else None, "mount_after": after["mount"] if after else None,
        "file_before": before["file"] if before else None, "file_after": after["file"] if after else None,
        "root_before": before["root"] if before else None, "root_after": after["root"] if after else None,
        "path_unchanged": before["path_chain"] == after["path_chain"] if unchanged else None,
        "mount_unchanged": before["mount"] == after["mount"] if unchanged else None,
        "file_unchanged": before["file"] == after["file"] if unchanged else None,
        "root_unchanged": before["root"] == after["root"] if unchanged else None,
        "root_scope": "SELECTED_ATTEMPT_ROOT_ONLY",
        "corpus_root_enumerated": False,
        "source_root_writable_bind_exposed": False,
        "files_written_under_selected_attempt_root": 0 if unchanged and before["root"] == after["root"] else None,
    }
    return {
        "schema_version": "smpcc-r8-liquid-s5a0-primary-supervisor-execution-receipt-v1",
        "document_type": "SMPCC_R8_LIQUID_S5A0_PRIMARY_SUPERVISOR_EXECUTION_RECEIPT_V1",
        "execution_id": EXECUTION_ID, "captured_at_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "status": FAIL_STATUS if failure else PASS_STATUS,
        "failure": {"code": failure.code, "phase": failure.phase, "message": failure.message} if failure else None,
        "policy": {"path": str(POLICY_PATH), "sha256": context["policy_sha"], "schema_path": str(SCHEMA_PATH), "schema_sha256": schema_sha, "static_inputs_verified": context["static_verified"]},
        "selection": {"attempt_id": selection["attempt_id"], "selected_role": "PRIMARY", "selected_count": 1, "source_root": selection["source_root"], "absolute_path": selection["absolute_path"], "relative_path": selection["relative_path"], "expected_size_bytes": selection["expected_size_bytes"], "expected_mode": selection["expected_mode"], "expected_sha256": selection["expected_sha256"], "optional_pair_exposed": False},
        "host_source": host,
        "sandbox": {"argv": context["argv"], "argv_sha256": context["argv_sha"], "source_read_only_bind": True, "network_unshared": True, "proc_mounted": False, "dev_mounted": False, "audit_root_bound": False, "dedicated_evidence_root_bound": True, "optional_pair_exposed": False, "tools": context["tools"], "limits": {"timeout_seconds": limits["timeout_seconds"], "kill_after_seconds": limits["kill_after_seconds"], "maximum_rss_bytes": limits["maximum_rss_bytes"], "maximum_address_space_bytes": limits["maximum_address_space_bytes"], "nofile_limit": limits["nofile_limit"], "nproc_limit": limits["nproc_limit"]}},
        "worker": context["worker"],
        "evidence": {"partial_root": paths["partial_evidence_root"], "final_root": paths["final_evidence_root"], "partial_create_new": context["partial_created"], "reservation_o_excl": context["reservation"], "partial_preserved": context["partial_preserved"], "atomic_rename_noreplace": context["renamed"], "inventory": context["inventory"]},
        "claims": {"development_only": True, "formal": False, "physical_primary_eligible": False, "source_outcome": "UNKNOWN", "source_bag_executed": False, "ros_started": False, "gpu_exposed": False, "solver_started": False, "sudo_used": False},
        "output": {"final_receipt_path": paths["final_receipt_path"], "failure_receipt_path": paths["failure_receipt_path"], "published_path": published_path, "create_new": True, "temporary_o_excl": True, "final_link_noreplace": True, "mode": "0600", "atomic_noreplace": True, "overwrote_existing_audit": False},
    }


def validate_execution_receipt(receipt: Mapping[str, Any], schema: Mapping[str, Any]) -> None:
    errors = list(Draft202012Validator(schema).iter_errors(receipt))
    if errors:
        _fail("EXECUTION_RECEIPT_SCHEMA_INVALID", "validate", errors[0].message)
    if receipt["status"] == PASS_STATUS:
        if receipt["failure"] is not None or not all(receipt["host_source"][name] is True for name in ("path_unchanged", "mount_unchanged", "file_unchanged", "root_unchanged")):
            _fail("PASS_SEMANTICS_INVALID", "validate", "PASS lacks complete unchanged source evidence")
        if receipt["worker"]["return_code"] != 0 or receipt["worker"]["inner_receipt_schema_valid"] is not True:
            _fail("PASS_WORKER_INVALID", "validate", "PASS worker evidence differs")
        evidence = receipt["evidence"]
        if (evidence["partial_create_new"] is not True
                or evidence["reservation_o_excl"] is not True
                or evidence["atomic_rename_noreplace"] is not True
                or len(evidence["inventory"]) != 5):
            _fail("PASS_EVIDENCE_INVALID", "validate", "PASS lacks complete create-new evidence")


Runner = Callable[[Sequence[str], TextIO, TextIO, int], int]


def _default_runner(argv: Sequence[str], stdout: TextIO, stderr: TextIO, timeout: int) -> int:
    completed = subprocess.run(argv, stdout=stdout, stderr=stderr, check=False, timeout=timeout)
    return completed.returncode


def run_one_shot(policy_path: Path = POLICY_PATH, *, runner: Runner = _default_runner) -> dict[str, Any]:
    policy, policy_sha = load_policy(policy_path)
    schema, schema_sha = load_schema()
    context = _empty_context(policy, policy_sha, schema_sha)
    failure: SupervisorError | None = None
    try:
        context["tools"] = verify_static_inputs(policy)
        context["static_verified"] = True
        context["before"] = capture_source_state(policy)
        partial = create_evidence_root(policy)
        context["partial_created"] = True
        context["reservation"] = True
        argv, argv_sha = build_bwrap_argv(policy)
        context["argv"], context["argv_sha"] = argv, argv_sha
        stdout_path, stderr_path = partial / policy["paths"]["stdout_name"], partial / policy["paths"]["stderr_name"]
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
        stdout_fd = os.open(stdout_path, flags, 0o600)
        stderr_fd = os.open(stderr_path, flags, 0o600)
        context["worker"]["started"] = True
        try:
            with os.fdopen(stdout_fd, "w", encoding="utf-8") as stdout, os.fdopen(stderr_fd, "w", encoding="utf-8") as stderr:
                return_code = runner(argv, stdout, stderr, policy["limits"]["timeout_seconds"] + policy["limits"]["kill_after_seconds"] + 5)
        except subprocess.TimeoutExpired:
            context["worker"]["timed_out"] = True
            _fail("HOST_TIMEOUT", "worker", "host timeout expired")
        context["worker"]["return_code"] = return_code
        _write_json_excl(partial / policy["paths"]["rc_name"], {"return_code": return_code}, 4096)
        if return_code != 0:
            _fail("WORKER_NONZERO", "worker", f"worker returned {return_code}")
        inner_path = partial / policy["paths"]["inner_receipt_name"]
        inner, _ = _read_json(inner_path, policy["limits"]["maximum_inner_receipt_bytes"], "inner receipt")
        inner_schema = gate_v1.load_schema()
        gate_v2.validate_runtime_receipt(inner, inner_schema)
        if inner["status"] != gate_v1.PASS_STATUS or inner["selection"]["absolute_path"] != policy["selection"]["absolute_path"] or inner["sandbox"]["bwrap_argv_sha256"] != argv_sha:
            _fail("INNER_RECEIPT_SEMANTICS_INVALID", "validate", "inner receipt identity/argv differs")
        context["worker"]["inner_receipt_schema_valid"] = True
        context["worker"]["inner_receipt_status"] = inner["status"]
        context["after"] = capture_source_state(policy)
        if context["before"] != context["after"]:
            _fail("HOST_SOURCE_CHANGED", "validate", "host source path/mount/hash/root changed")
        expected_names = sorted(policy["expected_evidence_inventory"])
        observed_names = sorted(entry.name for entry in os.scandir(partial))
        if observed_names != expected_names:
            _fail("EVIDENCE_INVENTORY_INVALID", "validate", f"inventory differs: {observed_names!r}")
        final_root = Path(policy["paths"]["final_evidence_root"])
        bounds = {
            policy["paths"]["stdout_name"]: policy["limits"]["maximum_stdout_bytes"],
            policy["paths"]["stderr_name"]: policy["limits"]["maximum_stderr_bytes"],
            policy["paths"]["inner_receipt_name"]: policy["limits"]["maximum_inner_receipt_bytes"],
            policy["paths"]["rc_name"]: 4096,
            policy["paths"]["reservation_name"]: 4096,
        }
        witnesses = {
            name: _file_witness(partial / name, final_root / name, bounds[name])
            for name in expected_names
        }
        context["inventory"] = [witnesses[name] for name in expected_names]
        context["worker"].update({"inner_receipt": witnesses[policy["paths"]["inner_receipt_name"]], "stdout": witnesses[policy["paths"]["stdout_name"]], "stderr": witnesses[policy["paths"]["stderr_name"]], "rc_file": witnesses[policy["paths"]["rc_name"]]})
        _rename_noreplace(partial, final_root)
        context["renamed"] = True
        receipt = build_execution_receipt(policy, schema_sha, context, failure=None, published_path=policy["paths"]["final_receipt_path"])
        validate_execution_receipt(receipt, schema)
        atomic_publish_json(Path(policy["paths"]["final_receipt_path"]), receipt, policy["limits"]["maximum_execution_receipt_bytes"])
        return receipt
    except Exception as exc:
        failure = exc if isinstance(exc, SupervisorError) else SupervisorError(
            "UNEXPECTED_EXECUTION_ERROR", "execution", f"{type(exc).__name__}: {exc}"
        )
        partial_path = Path(policy["paths"]["partial_evidence_root"])
        try:
            partial_metadata = os.lstat(partial_path)
            partial_exists = stat.S_ISDIR(partial_metadata.st_mode)
        except FileNotFoundError:
            partial_exists = False
        context["partial_created"] = context["partial_created"] or partial_exists
        context["reservation"] = context["reservation"] or (
            partial_exists and (partial_path / policy["paths"]["reservation_name"]).is_file()
        )
        context["partial_preserved"] = partial_exists and not context["renamed"]
        _collect_existing_evidence(policy, context)
        try:
            if context["before"] is not None:
                context["after"] = capture_source_state(policy)
        except SupervisorError:
            pass
        receipt = build_execution_receipt(policy, schema_sha, context, failure=failure, published_path=policy["paths"]["failure_receipt_path"])
        validate_execution_receipt(receipt, schema)
        atomic_publish_json(Path(policy["paths"]["failure_receipt_path"]), receipt, policy["limits"]["maximum_execution_receipt_bytes"])
        return receipt


def self_check() -> dict[str, Any]:
    policy, policy_sha = load_policy()
    schema, schema_sha = load_schema()
    tools = verify_static_inputs(policy)
    argv, argv_sha = build_bwrap_argv(policy)
    delimiter = argv.index("--")
    if any(token in argv[:delimiter] for token in ("--proc", "--dev")):
        _fail("PROC_DEV_EXPOSED", "static", "proc/dev appears in bwrap setup")
    return {"status": "PASS_S5A0_PRIMARY_ONE_SHOT_SUPERVISOR_STATIC_CONTRACT", "execution_id": EXECUTION_ID, "policy_sha256": policy_sha, "schema_sha256": schema_sha, "argv_sha256": argv_sha, "tool_count": len(tools), "real_bag_read": False, "bwrap_executed": False, "proc_mounted": False, "dev_mounted": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("self-check")
    run = subparsers.add_parser("run")
    run.add_argument("--authorize-execution-id", required=True)
    arguments = parser.parse_args(argv)
    try:
        if arguments.command == "self-check":
            result = self_check()
        else:
            if arguments.authorize_execution_id != EXECUTION_ID:
                _fail("EXECUTION_NOT_AUTHORIZED", "authorization", "exact execution id was not supplied")
            result = run_one_shot()
    except SupervisorError as exc:
        print(json.dumps({"status": FAIL_STATUS, "code": exc.code, "phase": exc.phase, "error": exc.message}, sort_keys=True), file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    if arguments.command == "self-check":
        return 0
    return 0 if result["status"] == PASS_STATUS else 2


if __name__ == "__main__":
    raise SystemExit(main())
