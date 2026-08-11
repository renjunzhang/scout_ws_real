#!/usr/bin/env python3
"""Two-phase GPU Stage-4 runner for damped initialization and undamped validation.

The runner is intentionally phase-addressed.  ``run-init`` may create exactly
one damped initialization root.  ``run-tail`` is admitted only after the first
phase has a valid final receipt and stages its exact Part_0201 checkpoint with
the original, damping-free XML.  Both solver invocations run as uid/gid 1000,
with zero capabilities, one GPU, no network namespace, and one writable bind.

Existing Stage-4 files and receipts are read-only parents.  All roots, logs and
receipts produced here are create-new and are preserved on either success or
failure.
"""

from __future__ import annotations

import argparse
import copy
import fcntl
import json
import math
import os
import signal
import stat
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator

import r8_liquid_u3_ddt_ramp_qc_v1 as metric_contract
import r8_liquid_u3_gpu_stage4_runner_v4 as legacy


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_stage4_damped_restart_policy_v1.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_gpu_damped_restart_runner_v1.py"
QC_PATH = SCRIPT_PATH.parent / "r8_liquid_u3_damped_restart_qc_v1.py"
QC_SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_damped_restart_qc_v1.json"
QC_TEST_PATH = PACKAGE_ROOT / "tests/test_u3_damped_restart_qc_v1.py"
LAB_ROOT = Path("/home/zrj/scout_liquid_lab")
AUDIT_ROOT = LAB_ROOT / "audits"
RUNS_PARENT = LAB_ROOT / "gpu_stage4_runs"
LOCK_PATH = LAB_ROOT / "locks/gpu0.lock"
MAX_JSON_BYTES = 4 * 1024 * 1024
MAX_INPUT_BYTES = 128 * 1024 * 1024
EXPECTED_ENVIRONMENT = dict(legacy.EXPECTED_ENVIRONMENT)
EXPECTED_DEVICES = list(legacy.EXPECTED_DEVICES)
PHASE_KEYS = ("damped_init", "undamped_tail")


class DampedRestartRunError(RuntimeError):
    """Closed-policy, identity, isolation or one-shot execution failure."""


def canonical_bytes(value: Any) -> bytes:
    return legacy.canonical_bytes(value)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path) -> dict[str, Any]:
    try:
        return legacy.read_json(path)
    except legacy.Stage4RunError as exc:
        raise DampedRestartRunError(str(exc)) from exc


def _node_signature(node: ET.Element, *, omit_special: bool) -> tuple[Any, ...]:
    children = []
    for child in list(node):
        if omit_special and node.tag == "execution" and child.tag == "special":
            continue
        children.append(_node_signature(child, omit_special=omit_special))
    text = (node.text or "").strip()
    return (node.tag, tuple(sorted(node.attrib.items())), text, tuple(children))


def _parse_xml(path: Path) -> ET.Element:
    try:
        return ET.parse(path).getroot()
    except (ET.ParseError, OSError) as exc:
        raise DampedRestartRunError(f"invalid case XML {path}: {exc}") from exc


def validate_xml_delta(policy: dict[str, Any]) -> dict[str, Any]:
    undamped_path = Path(policy["inputs"]["undamped_case_xml"]["path"])
    damped_path = Path(policy["inputs"]["damped_case_xml"]["path"])
    undamped = _parse_xml(undamped_path)
    damped = _parse_xml(damped_path)
    if undamped.find("./execution/special") is not None:
        raise DampedRestartRunError("undamped validation XML unexpectedly contains <special>")
    special = damped.find("./execution/special")
    if special is None or [child.tag for child in special] != ["damping"]:
        raise DampedRestartRunError("damped XML special node differs")
    damping = special.find("./damping")
    if damping is None or [child.tag for child in damping] != ["dampingzone"]:
        raise DampedRestartRunError("damped XML damping node differs")
    zone = damping.find("./dampingzone")
    if zone is None or [child.tag for child in zone] != [
        "overlimit", "redumax", "factorxyz", "limitmin", "limitmax"
    ]:
        raise DampedRestartRunError("dampingzone child set/order differs")
    wanted = policy["damping_contract"]
    expected_attributes = {
        "overlimit": {"value": format(wanted["overlimit_m"], ".3f")},
        "redumax": {"value": format(wanted["redumax_per_s"], "g")},
        "factorxyz": {axis: format(wanted["factorxyz"][axis], "g") for axis in ("x", "y", "z")},
        "limitmin": {axis: format(wanted["limitmin_m"][axis], "g") for axis in ("x", "y", "z")},
        "limitmax": {axis: format(wanted["limitmax_m"][axis], "g") for axis in ("x", "y", "z")},
    }
    observed_attributes = {child.tag: dict(child.attrib) for child in zone}
    if observed_attributes != expected_attributes:
        raise DampedRestartRunError(
            f"damping XML values differ: observed={observed_attributes!r} expected={expected_attributes!r}"
        )
    if _node_signature(undamped, omit_special=True) != _node_signature(damped, omit_special=True):
        raise DampedRestartRunError("case XML semantic delta is not limited to execution/special")
    z0 = wanted["limitmin_m"]["z"]
    z1 = wanted["limitmax_m"]["z"]
    over = wanted["overlimit_m"]
    domain_z = (0.0, 0.058)
    if not (z1 > z0 and domain_z[0] > z1 and domain_z[1] <= z1 + over):
        raise DampedRestartRunError("damping plane does not cover the full frozen fluid column")
    return {
        "status": "PASS_EXACT_INITIALIZATION_ONLY_DAMPING_XML_DELTA",
        "undamped_has_special": False,
        "damped_zone_count": 1,
        "fluid_z_domain_m": list(domain_z),
        "uniform_max_reduction_coverage": True,
        "attributes": observed_attributes,
    }


def _verify_source_semantics(policy: dict[str, Any]) -> dict[str, Any]:
    damping = Path(policy["parents"]["damping_source"]["path"]).read_text(encoding="utf-8")
    gpu = Path(policy["parents"]["gpu_call_source"]["path"]).read_text(encoding="utf-8")
    restart = Path(policy["parents"]["restart_source"]["path"]).read_text(encoding="utf-8")
    required_damping = (
        'cmd=="dampingzone"',
        "ComputeDampingGpu(double timestep",
        "List[c]->ComputeDampingGpu(dt",
        "velrhop[p1].x=float(redudtx*velrhop[p1].x)",
    )
    if any(token not in damping for token in required_damping):
        raise DampedRestartRunError("sealed damping source semantics differ")
    if "Damping->ComputeDampingGpu(TimeStep,dt" not in gpu:
        raise DampedRestartRunError("sealed GPU damping call is absent")
    if 'printf("    -partbegin:begin[:first] dir' not in restart:
        raise DampedRestartRunError("sealed restart CLI source is absent")
    return {
        "damping_plane_parser": True,
        "gpu_damping_call": True,
        "restart_cli": True,
        "built_in_damping_time_cutoff": False,
    }


def _exact_solver_argv(phase_name: str) -> list[str]:
    prefix = [
        "/runtime/DualSPHysics5.4_linux64",
        "/case/C1M_zero",
        "/output",
        "-gpu:0",
        "-ompthreads:1",
        "-stable:1",
        "-vres:0",
        "-cellmode:full",
        "-cfl:0.1",
    ]
    restart = [] if phase_name == "damped_init" else ["-partbegin:201:201", "/restart"]
    return [
        *prefix,
        *restart,
        "-shifting:none",
        f"-tmax:{'10.05' if phase_name == 'damped_init' else '20.05'}",
        "-tout:0.05",
        "-sv:binx,info",
        "-svres:1",
        "-svtimers:0",
        "-svdomainvtk:0",
        "-saveposdouble:1",
        "-nortimes:1",
        "-createdirs:1",
        "-csvsep:0",
    ]


def _phase_paths(run_id: str) -> dict[str, str]:
    attempt = RUNS_PARENT / f"{run_id}.partial"
    return {
        "attempt_root": str(attempt),
        "output_root": str(attempt / "output"),
        "start_receipt": str(AUDIT_ROOT / f"{run_id}.start.json"),
        "final_receipt": str(AUDIT_ROOT / f"{run_id}.final.json"),
        "failure_receipt": str(AUDIT_ROOT / f"{run_id}.failure.json"),
        "stdout_log": str(AUDIT_ROOT / f"{run_id}.stdout.log"),
        "stderr_log": str(AUDIT_ROOT / f"{run_id}.stderr.log"),
        "resource_log": str(AUDIT_ROOT / f"{run_id}.resources.jsonl"),
    }


def semantic_validate(policy: dict[str, Any], policy_path: Path) -> dict[str, Any]:
    if policy_path.parent not in (PACKAGE_ROOT / "config/target_hosts", AUDIT_ROOT):
        raise DampedRestartRunError("policy path leaves reviewed config/audit roots")
    if tuple(policy["phases"]) != PHASE_KEYS:
        raise DampedRestartRunError("phase set/order differs")
    if policy["solver_invariants"]["environment"] != EXPECTED_ENVIRONMENT:
        raise DampedRestartRunError("sanitized solver environment differs")
    if policy["gpu"]["device_paths"] != EXPECTED_DEVICES:
        raise DampedRestartRunError("GPU device set differs")
    exact_damping = {
        "single_intervention": "INITIALIZATION_ONLY_STANDARD_DAMPING_PLANE",
        "xml_xpath": "case.execution.special.damping.dampingzone",
        "type": "plane",
        "limitmin_m": {"x": 0, "y": 0, "z": -0.004},
        "limitmax_m": {"x": 0, "y": 0, "z": -0.002},
        "overlimit_m": 0.08,
        "redumax_per_s": 10,
        "factorxyz": {"x": 1, "y": 1, "z": 1},
        "normal_fluid_only": True,
        "initialization_only": True,
        "built_in_time_cutoff": False,
        "undamped_tail_required": True,
        "semantic_delta_check_required": True,
    }
    if policy["damping_contract"] != exact_damping:
        raise DampedRestartRunError("damping contract differs from the one frozen intervention")
    if policy["acceptance"]["metric_limits"] != dict(sorted(metric_contract.METRIC_LIMITS.items())):
        raise DampedRestartRunError("17-metric thresholds drifted")
    expected_phase = {
        "damped_init": {
            "purpose": "damped_initialization",
            "case_xml_input": "damped_case_xml",
            "restart": {"enabled": False, "part_index": None, "part_first": None, "guest_dir": None, "source_phase": None},
            "output": (0.0, 10.05, 0, 201, 202),
        },
        "undamped_tail": {
            "purpose": "undamped_validation_tail",
            "case_xml_input": "undamped_case_xml",
            "restart": {"enabled": True, "part_index": 201, "part_first": 201, "guest_dir": "/restart", "source_phase": "damped_init"},
            "output": (10.05, 20.05, 201, 401, 201),
        },
    }
    for phase_name in PHASE_KEYS:
        phase = policy["phases"][phase_name]
        wanted = expected_phase[phase_name]
        if phase["purpose"] != wanted["purpose"] or phase["case_xml_input"] != wanted["case_xml_input"]:
            raise DampedRestartRunError(f"{phase_name} purpose/XML differs")
        if phase["restart"] != wanted["restart"]:
            raise DampedRestartRunError(f"{phase_name} restart contract differs")
        if phase["solver_argv"] != _exact_solver_argv(phase_name):
            raise DampedRestartRunError(f"{phase_name} exact solver argv differs")
        if phase["run"] != _phase_paths(phase["run_id"]):
            raise DampedRestartRunError(f"{phase_name} output/audit paths differ")
        wanted_stage = Path("/var/tmp") / f"r8-liquid-u3-stage4-{phase['run_id'].lower().replace('_', '-')}"
        if Path(phase["sandbox"]["stage_root"]) != wanted_stage:
            raise DampedRestartRunError(f"{phase_name} stage root differs")
        if phase["sandbox"]["network"] or phase["sandbox"]["project_workspace_write_bind_count"] != 0:
            raise DampedRestartRunError(f"{phase_name} sandbox boundary differs")
        start, end, first, last, count = wanted["output"]
        output = phase["expected_output"]
        if (output["start_time_s"], output["end_time_s"], output["part_first"], output["part_last"], output["part_count"]) != (
            start, end, first, last, count
        ) or not math.isclose(output["output_period_s"], 0.05, rel_tol=0, abs_tol=1e-12):
            raise DampedRestartRunError(f"{phase_name} output schedule differs")
        limits = phase["limits"]
        if not 10 <= limits["monitor_interval_seconds"] <= 30:
            raise DampedRestartRunError(f"{phase_name} monitor interval differs")
        if limits["minimum_mem_available_bytes"] < 4294967296 or limits["wall_timeout_seconds"] > 3600:
            raise DampedRestartRunError(f"{phase_name} resource boundary is weaker")
        forbidden = ("-cpu", "-gpu:1", "http://", "https://", "/home/", "..")
        if any(fragment in arg.lower() for arg in phase["solver_argv"] for fragment in forbidden):
            raise DampedRestartRunError(f"{phase_name} argv contains forbidden fragment")
    parent_paths = {
        "runner": SCRIPT_PATH,
        "schema": SCHEMA_PATH,
        "tests": TEST_PATH,
        "qc": QC_PATH,
        "qc_schema": QC_SCHEMA_PATH,
        "qc_tests": QC_TEST_PATH,
    }
    for key, path in parent_paths.items():
        if policy["parents"][key]["path"] != str(path):
            raise DampedRestartRunError(f"{key} parent path differs")
    xml_delta = validate_xml_delta(policy)
    source_semantics = _verify_source_semantics(policy)
    return {"xml_delta": xml_delta, "source_semantics": source_semantics}


def load_policy(path: Path) -> dict[str, Any]:
    schema = read_json(SCHEMA_PATH)
    policy = read_json(path)
    Draft202012Validator.check_schema(schema)
    errors = sorted(Draft202012Validator(schema).iter_errors(policy), key=lambda item: list(item.absolute_path))
    if errors:
        first = errors[0]
        raise DampedRestartRunError(f"policy schema failure at {list(first.absolute_path)}: {first.message}")
    semantic_validate(policy, path)
    return policy


def verify_frozen_files(policy: dict[str, Any]) -> dict[str, Any]:
    try:
        parents = {
            name: legacy.verify_spec(spec, full=False, maximum=MAX_JSON_BYTES)
            for name, spec in policy["parents"].items()
        }
        inputs = {
            name: legacy.verify_spec(spec, full=True, maximum=MAX_INPUT_BYTES)
            for name, spec in policy["inputs"].items()
        }
        tools = {
            name: legacy.verify_spec(spec, full=False, maximum=16 * 1024 * 1024)
            for name, spec in policy["tools"].items()
        }
    except legacy.Stage4RunError as exc:
        raise DampedRestartRunError(str(exc)) from exc
    if inputs["candidate"]["mode"] != "0400":
        raise DampedRestartRunError("source candidate is not disarmed 0400")
    return {"parents": parents, "inputs": inputs, "tools": tools}


def _dynamic_restart(policy: dict[str, Any]) -> dict[str, Any]:
    init = policy["phases"]["damped_init"]
    receipt_path = Path(init["run"]["final_receipt"])
    receipt = read_json(receipt_path)
    if (
        receipt.get("document_type") != "SMPCC_R8_LIQUID_U3_DAMPED_RESTART_RUN_FINAL_V1"
        or receipt.get("status") != "PASS_GPU_STAGE4_DAMPED_RESTART_RAW_OUTPUT"
        or receipt.get("phase") != "damped_init"
        or receipt.get("run_id") != init["run_id"]
        or receipt.get("returncode") != 0
        or receipt.get("termination_reason") != "PROCESS_EXIT"
    ):
        raise DampedRestartRunError("damped-init final receipt does not admit tail")
    files = receipt.get("output_inventory", {}).get("files", {})
    expected = {
        "restart_part": Path(init["run"]["output_root"]) / "data/Part_0201.bi4",
        "restart_head": Path(init["run"]["output_root"]) / "data/Part_Head.ibi4",
    }
    result: dict[str, Any] = {"init_receipt": legacy.identity(receipt_path, maximum=MAX_JSON_BYTES)}
    for key, path in expected.items():
        item = legacy.identity(path, maximum=MAX_INPUT_BYTES)
        relative = str(path.relative_to(Path(init["run"]["output_root"])))
        frozen = files.get(relative)
        if not isinstance(frozen, dict) or any(item[name] != frozen.get(name) for name in ("sha256", "size_bytes", "mode", "nlink")):
            raise DampedRestartRunError(f"dynamic restart file differs from init receipt: {path}")
        result[key] = item
    return result


def _device_identities(policy: dict[str, Any]) -> list[dict[str, Any]]:
    devices = []
    for name in policy["gpu"]["device_paths"]:
        metadata = os.stat(name, follow_symlinks=False)
        if not stat.S_ISCHR(metadata.st_mode):
            raise DampedRestartRunError(f"GPU device is not character special: {name}")
        devices.append({"path": name, "mode": f"{stat.S_IMODE(metadata.st_mode):04o}", "major": os.major(metadata.st_rdev), "minor": os.minor(metadata.st_rdev)})
    return devices


def _phase_fresh_paths(phase: dict[str, Any]) -> list[Path]:
    return [Path(phase["run"][key]) for key in (
        "attempt_root", "start_receipt", "final_receipt", "failure_receipt",
        "stdout_log", "stderr_log", "resource_log",
    )] + [Path(phase["sandbox"]["stage_root"])]


def preflight(policy_path: Path, phase_name: str, *, require_fresh: bool) -> dict[str, Any]:
    policy = load_policy(policy_path)
    phase = policy["phases"][phase_name]
    verified = verify_frozen_files(policy)
    available = legacy.mem_available_bytes()
    if available < phase["limits"]["minimum_mem_available_bytes"]:
        raise DampedRestartRunError(f"MemAvailable below frozen minimum: {available}")
    competitors = legacy.compute_processes(policy["tools"]["nvidia_smi"]["path"])
    if competitors:
        raise DampedRestartRunError(f"GPU compute competitor exists: {competitors}")
    collisions = [str(path) for path in _phase_fresh_paths(phase) if os.path.lexists(path)] if require_fresh else []
    if collisions:
        raise DampedRestartRunError(f"one-shot identity collision: {collisions}")
    dynamic = None if phase_name == "damped_init" else _dynamic_restart(policy)
    return {
        "captured_at_utc": utc_now(),
        "experiment_id": policy["experiment_id"],
        "phase": phase_name,
        "policy": legacy.identity(policy_path, maximum=MAX_JSON_BYTES),
        "schema": legacy.identity(SCHEMA_PATH, maximum=MAX_JSON_BYTES),
        "runner": legacy.identity(SCRIPT_PATH, maximum=MAX_JSON_BYTES),
        "verified": verified,
        "dynamic_restart": dynamic,
        "memory": {"mem_available_bytes": available, "psi": Path("/proc/pressure/memory").read_text(encoding="ascii").strip()},
        "gpu": legacy.gpu_sample(policy),
        "devices": _device_identities(policy),
        "compute_processes": competitors,
        "fresh": require_fresh,
    }


def _prepare_stage(policy: dict[str, Any], phase_name: str, dynamic: dict[str, Any] | None) -> dict[str, Any]:
    phase = policy["phases"][phase_name]
    stage = Path(phase["sandbox"]["stage_root"])
    legacy.mkdir_exclusive(stage, 0o700, 0, 0)
    for name in ("runtime", "case", "output"):
        legacy.mkdir_exclusive(stage / name, 0o777 if name == "output" else 0o700, 0, 0)
    staged = {
        "candidate": legacy.copy_exclusive(policy["inputs"]["candidate"], stage / "runtime/DualSPHysics5.4_linux64", 0o555, 0, 0),
        "dsph_config": legacy.copy_exclusive(policy["inputs"]["dsph_config"], stage / "runtime/DsphConfig.xml", 0o444, 0, 0),
        "case_bi4": legacy.copy_exclusive(policy["inputs"]["case_bi4"], stage / "case/C1M_zero.bi4", 0o444, 0, 0),
        "case_xml": legacy.copy_exclusive(policy["inputs"][phase["case_xml_input"]], stage / "case/C1M_zero.xml", 0o444, 0, 0),
    }
    if phase["restart"]["enabled"]:
        if dynamic is None:
            raise DampedRestartRunError("tail dynamic restart evidence is absent")
        legacy.mkdir_exclusive(stage / "restart", 0o700, 0, 0)
        staged["restart_part"] = legacy.copy_exclusive(dynamic["restart_part"], stage / "restart/Part_0201.bi4", 0o444, 0, 0)
        staged["restart_head"] = legacy.copy_exclusive(dynamic["restart_head"], stage / "restart/Part_Head.ibi4", 0o444, 0, 0)
    return staged


def _bwrap_argv(policy: dict[str, Any], phase_name: str) -> list[str]:
    phase = policy["phases"][phase_name]
    stage = Path(phase["sandbox"]["stage_root"])
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
    if phase["restart"]["enabled"]:
        argv.extend(("--ro-bind", str(stage / "restart"), "/restart"))
    argv.extend(("--bind", str(stage / "output"), "/output", "--chdir", "/runtime", "--", *phase["solver_argv"]))
    if "--unshare-net" not in argv or "--share-net" in argv:
        raise DampedRestartRunError("network namespace contract differs")
    write_binds = [argv[index + 1:index + 3] for index, item in enumerate(argv) if item == "--bind"]
    if write_binds != [[str(stage / "output"), "/output"]]:
        raise DampedRestartRunError("writable bind set differs")
    return argv


def _terminate_group(process: subprocess.Popen[bytes], kill_after: float) -> None:
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


def execute(policy_path: Path, phase_name: str) -> dict[str, Any]:
    if not (
        os.geteuid() == 0
        and os.environ.get("R8_STAGE4_ROOT_SUPERVISOR") == "1"
        and os.environ.get("SUDO_UID") == "1000"
        and os.environ.get("SUDO_GID") == "1000"
    ):
        raise DampedRestartRunError("run requires exact sudo root-supervisor identity")
    pre = preflight(policy_path, phase_name, require_fresh=True)
    policy = load_policy(policy_path)
    phase = policy["phases"][phase_name]
    dynamic = pre["dynamic_restart"]
    lock_fd = os.open(LOCK_PATH, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
    os.fchown(lock_fd, 1000, 1000)
    process: subprocess.Popen[bytes] | None = None
    started_wall = time.time()
    started_mono = time.monotonic()
    returncode: int | None = None
    termination_reason = "NOT_STARTED"
    staged: dict[str, Any] | None = None
    argv: list[str] = []
    run = phase["run"]
    try:
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise DampedRestartRunError("GPU lock is already held") from exc
        legacy.ensure_runs_parent()
        start = {
            "schema_version": "r8-liquid-u3-damped-restart-run-receipt-v1",
            "document_type": "SMPCC_R8_LIQUID_U3_DAMPED_RESTART_RUN_START_V1",
            "status": "GPU_STAGE4_DAMPED_RESTART_PHASE_STARTED",
            "experiment_id": policy["experiment_id"],
            "phase": phase_name,
            "run_id": phase["run_id"],
            "started_at_utc": utc_now(),
            "preflight": pre,
            "candidate_source_executed": False,
            "staged_candidate_execution_pending": True,
            "network_used": False,
            "project_workspace_writable_in_guest": False,
        }
        start_identity = legacy.write_json_exclusive(Path(run["start_receipt"]), start)
        legacy.mkdir_exclusive(Path(run["attempt_root"]), 0o700, 1000, 1000)
        legacy.mkdir_exclusive(Path(run["output_root"]), 0o700, 1000, 1000)
        staged = _prepare_stage(policy, phase_name, dynamic)
        stdout_fd = os.open(run["stdout_log"], os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
        stderr_fd = os.open(run["stderr_log"], os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
        resource_fd = os.open(run["resource_log"], os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
        for descriptor in (stdout_fd, stderr_fd, resource_fd):
            os.fchown(descriptor, 1000, 1000)
            os.fchmod(descriptor, 0o640)
        try:
            argv = _bwrap_argv(policy, phase_name)
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
                if elapsed >= phase["limits"]["wall_timeout_seconds"]:
                    termination_reason = "WALL_TIMEOUT"
                    _terminate_group(process, phase["limits"]["kill_after_seconds"])
                    break
                if elapsed >= next_sample:
                    sample = legacy.resource_sample(policy, process.pid, started_mono, Path(phase["sandbox"]["stage_root"]) / "output")
                    os.write(resource_fd, canonical_bytes(sample))
                    if sample["mem_available_bytes"] < phase["limits"]["minimum_mem_available_bytes"]:
                        termination_reason = "MEMORY_FLOOR"
                        _terminate_group(process, phase["limits"]["kill_after_seconds"])
                        break
                    if sample["output_bytes"] > phase["limits"]["maximum_output_bytes"]:
                        termination_reason = "OUTPUT_LIMIT"
                        _terminate_group(process, phase["limits"]["kill_after_seconds"])
                        break
                    next_sample = elapsed + phase["limits"]["monitor_interval_seconds"]
                time.sleep(0.2)
            returncode = process.wait(timeout=10)
            final_sample = legacy.resource_sample(policy, process.pid, started_mono, Path(phase["sandbox"]["stage_root"]) / "output")
            os.write(resource_fd, canonical_bytes(final_sample))
            os.fsync(resource_fd)
        finally:
            if process is not None and process.poll() is None:
                _terminate_group(process, phase["limits"]["kill_after_seconds"])
            for descriptor in (stdout_fd, stderr_fd, resource_fd):
                os.close(descriptor)
        if returncode != 0 or termination_reason != "PROCESS_EXIT":
            raise DampedRestartRunError(f"solver failed rc={returncode} termination={termination_reason}")
        output_inventory = legacy.export_output(
            Path(phase["sandbox"]["stage_root"]) / "output",
            Path(run["output_root"]),
            phase["expected_output"],
        )
        candidate_after = legacy.verify_spec(policy["inputs"]["candidate"], full=True, maximum=MAX_INPUT_BYTES)
        competitors_after = legacy.compute_processes(policy["tools"]["nvidia_smi"]["path"])
        if competitors_after:
            raise DampedRestartRunError(f"GPU compute process remains: {competitors_after}")
        final = {
            "schema_version": "r8-liquid-u3-damped-restart-run-receipt-v1",
            "document_type": "SMPCC_R8_LIQUID_U3_DAMPED_RESTART_RUN_FINAL_V1",
            "status": "PASS_GPU_STAGE4_DAMPED_RESTART_RAW_OUTPUT",
            "experiment_id": policy["experiment_id"],
            "phase": phase_name,
            "run_id": phase["run_id"],
            "purpose": phase["purpose"],
            "started_at_utc": datetime.fromtimestamp(started_wall, tz=timezone.utc).isoformat(),
            "completed_at_utc": utc_now(),
            "elapsed_seconds": round(time.monotonic() - started_mono, 6),
            "returncode": returncode,
            "termination_reason": termination_reason,
            "policy": legacy.identity(policy_path, maximum=MAX_JSON_BYTES),
            "start_receipt": start_identity,
            "dynamic_restart": dynamic,
            "sandbox_argv": argv,
            "solver_argv": phase["solver_argv"],
            "staged_inputs": staged,
            "candidate_source_after": candidate_after,
            "candidate_source_executed": False,
            "staged_candidate_executed": True,
            "network_used": False,
            "project_workspace_writable_in_guest": False,
            "apparmor_profile_used": False,
            "output_inventory": output_inventory,
            "stdout": legacy.identity(Path(run["stdout_log"])),
            "stderr": legacy.identity(Path(run["stderr_log"])),
            "resource_log": legacy.identity(Path(run["resource_log"]), maximum=16 * 1024 * 1024),
            "gpu_after": legacy.gpu_sample(policy),
            "compute_processes_after": competitors_after,
            "development_only": True,
            "formal": False,
            "physical_primary_eligible": False,
            "qc_pending": True,
        }
        receipt_identity = legacy.write_json_exclusive(Path(run["final_receipt"]), final)
        final["final_receipt"] = receipt_identity
        return final
    except Exception as exc:
        failure = {
            "schema_version": "r8-liquid-u3-damped-restart-run-receipt-v1",
            "document_type": "SMPCC_R8_LIQUID_U3_DAMPED_RESTART_RUN_FAILURE_V1",
            "status": "FAIL_GPU_STAGE4_DAMPED_RESTART_PHASE",
            "experiment_id": policy.get("experiment_id") if isinstance(locals().get("policy"), dict) else None,
            "phase": phase_name,
            "run_id": phase.get("run_id") if isinstance(locals().get("phase"), dict) else None,
            "failed_at_utc": utc_now(),
            "elapsed_seconds": round(time.monotonic() - started_mono, 6),
            "error_type": type(exc).__name__,
            "error": str(exc),
            "returncode": returncode,
            "termination_reason": termination_reason,
            "stage_preserved": os.path.lexists(phase["sandbox"]["stage_root"]) if isinstance(locals().get("phase"), dict) else False,
            "attempt_root_preserved": os.path.lexists(run["attempt_root"]) if isinstance(locals().get("run"), dict) else False,
            "candidate_source_executed": False,
            "network_used": False,
            "project_workspace_writable_in_guest": False,
        }
        if isinstance(locals().get("run"), dict) and not os.path.lexists(run["failure_receipt"]):
            legacy.write_json_exclusive(Path(run["failure_receipt"]), failure)
        raise
    finally:
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_UN)
        finally:
            os.close(lock_fd)


def self_check(policy_path: Path) -> dict[str, Any]:
    policy = load_policy(policy_path)
    verified = verify_frozen_files(policy)
    semantics = semantic_validate(policy, policy_path)
    return {
        "status": "PASS_U3_GPU_DAMPED_RESTART_RUNNER_V1_SELF_CHECK",
        "policy": legacy.identity(policy_path, maximum=MAX_JSON_BYTES),
        "verified": verified,
        "semantics": semantics,
        "sandbox_argv": {name: _bwrap_argv(policy, name) for name in PHASE_KEYS},
        "network_used": False,
        "candidate_executed": False,
        "project_workspace_writable_in_guest": False,
    }


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("policy", type=Path)
    result.add_argument("command", choices=("self-check", "preflight-init", "run-init", "preflight-tail", "run-tail"))
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        if args.command == "self-check":
            result = self_check(args.policy)
        elif args.command == "preflight-init":
            result = {"status": "PASS_DAMPED_INIT_PREFLIGHT", "evidence": preflight(args.policy, "damped_init", require_fresh=True)}
        elif args.command == "preflight-tail":
            result = {"status": "PASS_UNDAMPED_TAIL_PREFLIGHT", "evidence": preflight(args.policy, "undamped_tail", require_fresh=True)}
        elif args.command == "run-init":
            result = execute(args.policy, "damped_init")
        else:
            result = execute(args.policy, "undamped_tail")
    except (DampedRestartRunError, legacy.Stage4RunError, OSError, ValueError, subprocess.SubprocessError) as exc:
        print(json.dumps({
            "status": "FAIL_U3_GPU_DAMPED_RESTART_RUNNER_V1",
            "error_type": type(exc).__name__,
            "error": str(exc),
        }, ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
