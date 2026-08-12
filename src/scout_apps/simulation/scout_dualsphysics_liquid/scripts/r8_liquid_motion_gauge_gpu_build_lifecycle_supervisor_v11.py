#!/usr/bin/env python3
"""Root-only one-shot lifecycle supervisor for Motion-Gauge GPU build v11.

The public default is a read-only ``self-check``.  The dynamic ``execute``
entry is an indivisible five-stage state machine; it exposes neither arbitrary
commands nor independently selectable phases.  Root is used only for exact
AppArmor lifecycle operations and the gate always runs through setpriv as
uid/gid 1000 with no supplementary groups.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import stat
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
WORKSPACE = ROOT.parents[3]
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_motion_gauge_gpu_build_execution_policy_v11.json"
POLICY_SCHEMA_PATH = ROOT / "schema/target_host_motion_gauge_gpu_build_execution_policy_v11.json"
GATE_PATH = ROOT / "scripts/r8_liquid_motion_gauge_gpu_build_execution_gate_v11.py"
TOKEN_SCHEMA_PATH = ROOT / "schema/target_host_motion_gauge_gpu_build_authorization_token_v11.json"
SUPERVISOR_RECEIPT_SCHEMA_PATH = ROOT / "schema/target_host_motion_gauge_gpu_build_supervisor_receipt_v11.json"
TOKEN_PRODUCER_PATH = ROOT / "scripts/r8_liquid_motion_gauge_gpu_build_authorization_token_v11.py"
CAMPAIGN_ID = "motion_gauge_gpu_build_sm120_20260812T073037Z_v11"
BUILD_ID = CAMPAIGN_ID + "_a"
TOKEN_PATH = Path("/home/zrj/scout_liquid_lab/audits") / f"{BUILD_ID}.authorization.json"
SUPERVISOR_FINAL_PATH = Path("/home/zrj/scout_liquid_lab/audits") / f"{BUILD_ID}.supervisor.final.json"
SUPERVISOR_FAILURE_PATH = Path("/home/zrj/scout_liquid_lab/audits") / f"{BUILD_ID}.supervisor.failure.json"
APPARMOR_PARSER = "/usr/sbin/apparmor_parser"
AA_STATUS = "/usr/sbin/aa-status"
SETPRIV = "/usr/bin/setpriv"
PYTHON = "/usr/bin/python3"
SUDO = "/usr/bin/sudo"
IDENTITY_KEYS = ("path", "mode_octal", "size_bytes", "sha256")
PROFILE_PHASES = (
    ("source-copy", "SOURCE_COPY"),
    ("patch", "PATCH"),
    ("build", "BUILD"),
    ("static-audit", "STATIC_AUDIT"),
)
FULL_PHASES = ("source-copy", "patch", "wrapper", "build", "static-audit")


class SupervisorError(RuntimeError):
    """A root boundary, frozen identity, phase, or cleanup invariant failed."""


def sha256_bytes(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def resolve_pinned(path_text: str) -> Path:
    path = Path(path_text)
    if path.is_absolute():
        return path
    if not path.parts or ".." in path.parts:
        raise SupervisorError(f"unsafe relative pinned path: {path_text}")
    return WORKSPACE / path


def file_identity(path: Path, *, expected: Mapping[str, Any] | None = None) -> dict[str, Any]:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1:
            raise SupervisorError(f"unsafe identity input: {path}")
        digest = hashlib.sha256()
        remaining = before.st_size
        while remaining:
            block = os.read(descriptor, min(1 << 20, remaining))
            if not block:
                raise SupervisorError(f"short identity read: {path}")
            digest.update(block)
            remaining -= len(block)
        if os.read(descriptor, 1):
            raise SupervisorError(f"identity input grew while read: {path}")
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    if (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) != (
        after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns
    ):
        raise SupervisorError(f"identity input drifted while read: {path}")
    result = {
        "path": str(path.resolve()),
        "mode_octal": format(stat.S_IMODE(after.st_mode), "04o"),
        "size_bytes": after.st_size,
        "sha256": digest.hexdigest(),
    }
    if expected is not None and result != {key: expected[key] for key in IDENTITY_KEYS}:
        raise SupervisorError(f"pinned identity drift: {path}")
    return result


def read_json(path: Path, *, limit: int = 4 * 1024 * 1024) -> dict[str, Any]:
    identity = file_identity(path)
    if identity["size_bytes"] > limit:
        raise SupervisorError(f"JSON input exceeds bound: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise SupervisorError(f"JSON input is not an object: {path}")
    return value


def policy_and_profiles() -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    current = read_json(POLICY_PATH)
    policy_schema = read_json(POLICY_SCHEMA_PATH)
    Draft202012Validator.check_schema(policy_schema)
    Draft202012Validator(policy_schema).validate(current)
    if current.get("schema_version") != "smpcc-r8-liquid-motion-gauge-gpu-build-execution-policy-v11":
        raise SupervisorError("v11 policy marker drift")
    campaign = current.get("campaign", {})
    if campaign.get("campaign_id") != CAMPAIGN_ID or campaign.get("build_id") != BUILD_ID:
        raise SupervisorError("v11 campaign/build identity drift")
    authorization = current.get("authorization", {})
    if (authorization.get("default_authorized"), authorization.get("uid"),
        authorization.get("gid"), authorization.get("supplementary_groups")) != (False, 1000, 1000, 0):
        raise SupervisorError("authorization privilege contract drift")
    expected_paths = {
        "token_schema_path": TOKEN_SCHEMA_PATH,
        "token_producer_path": TOKEN_PRODUCER_PATH,
        "supervisor_path": Path(__file__).resolve(),
        "supervisor_receipt_schema_path": SUPERVISOR_RECEIPT_SCHEMA_PATH,
    }
    for key, expected in expected_paths.items():
        if resolve_pinned(authorization.get(key, "")) != expected:
            raise SupervisorError(f"policy {key} drift")
    token_schema_identity = file_identity(TOKEN_SCHEMA_PATH)
    if token_schema_identity["sha256"] != authorization["token_schema_sha256"]:
        raise SupervisorError("policy token schema SHA-256 drift")
    receipt_schema_identity = file_identity(SUPERVISOR_RECEIPT_SCHEMA_PATH)
    if receipt_schema_identity["sha256"] != authorization["supervisor_receipt_schema_sha256"]:
        raise SupervisorError("policy supervisor receipt schema SHA-256 drift")
    profile_list = current.get("profiles")
    if not isinstance(profile_list, list):
        raise SupervisorError("policy profiles are not an array")
    profiles = {item.get("role"): item for item in profile_list}
    if list(profiles) != ["SOURCE_COPY", "PATCH", "BUILD", "STATIC_AUDIT"]:
        raise SupervisorError("exact profile set/order drift")
    for item in profile_list:
        file_identity(resolve_pinned(item["path"]), expected={
            "path": str(resolve_pinned(item["path"]).resolve()),
            **{key: item[key] for key in ("mode_octal", "size_bytes", "sha256")},
        })
    return current, profiles


def static_gate_report(*, token_may_exist: bool = False) -> dict[str, Any]:
    spec = importlib.util.spec_from_file_location("r8_motion_v11_gate_for_supervisor", GATE_PATH)
    if spec is None or spec.loader is None:
        raise SupervisorError("cannot load exact v11 gate for static admission")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    report = module.validate_static_contract(
        require_fresh=True, require_token_fresh=not token_may_exist
    )
    if report.get("status") != "PASS_MOTION_GAUGE_GPU_BUILD_V11_STATIC_DEFAULT_DENY":
        raise SupervisorError("v11 gate static admission is not PASS")
    return report


def canonical_json(value: Any) -> bytes:
    return (json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True,
                       separators=(",", ":")) + "\n").encode("utf-8")


def write_all(descriptor: int, raw: bytes) -> None:
    offset = 0
    while offset < len(raw):
        count = os.write(descriptor, raw[offset:])
        if count <= 0:
            raise SupervisorError("short supervisor receipt write")
        offset += count


def write_new(path: Path, value: Mapping[str, Any]) -> dict[str, Any]:
    schema = read_json(SUPERVISOR_RECEIPT_SCHEMA_PATH)
    Draft202012Validator.check_schema(schema)
    Draft202012Validator(schema).validate(value)
    descriptor = os.open(
        path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600,
    )
    try:
        os.fchmod(descriptor, 0o600)
        write_all(descriptor, canonical_json(value))
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)
    return file_identity(path)


def validate_token(current: Mapping[str, Any], profiles: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    if str(TOKEN_PATH) != current["campaign"]["authorization_token_path"]:
        raise SupervisorError("policy token path drift")
    identity = file_identity(TOKEN_PATH)
    if identity["mode_octal"] != "0600" or identity["size_bytes"] > 16384:
        raise SupervisorError("authorization token mode/size drift")
    value = read_json(TOKEN_PATH, limit=16384)
    token_schema = read_json(TOKEN_SCHEMA_PATH)
    Draft202012Validator.check_schema(token_schema)
    Draft202012Validator(token_schema).validate(value)
    if value.get("campaign_id") != CAMPAIGN_ID or value.get("user_authorized") is not True:
        raise SupervisorError("authorization token decision drift")
    expected_identities = {
        "policy": file_identity(POLICY_PATH),
        "gate": file_identity(GATE_PATH),
        "supervisor": file_identity(Path(__file__).resolve()),
        "supervisor_receipt_schema": file_identity(SUPERVISOR_RECEIPT_SCHEMA_PATH),
        "token_producer": file_identity(TOKEN_PRODUCER_PATH),
    }
    for key, expected in expected_identities.items():
        if value.get(key) != expected:
            raise SupervisorError(f"authorization token {key} identity drift")
    expected_profiles = [
        {key: item[key] for key in ("role", "name", "path", "mode_octal", "size_bytes", "sha256")}
        for item in current["profiles"]
    ]
    if value.get("profiles") != expected_profiles:
        raise SupervisorError("authorization token profile identity drift")
    if not isinstance(value.get("authorization_reference"), str) or sha256_bytes(
        value["authorization_reference"].encode("utf-8")
    ) != value.get("authorization_reference_sha256"):
        raise SupervisorError("authorization reference hash drift")
    # Let the gate's schema and byte-identity validator make the final token decision.
    if set(profiles) != {item["role"] for item in value["profiles"]}:
        raise SupervisorError("authorization token profile role drift")
    return value


def run_capture(argv: Sequence[str], *, timeout: int,
                runner: Callable[[Sequence[str], int], tuple[int, bytes, bytes]] | None = None) -> dict[str, Any]:
    started = time.monotonic()
    if runner is None:
        completed = subprocess.run(list(argv), stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, timeout=timeout, check=False,
                                   close_fds=True)
        rc, stdout, stderr = completed.returncode, completed.stdout, completed.stderr
    else:
        rc, stdout, stderr = runner(argv, timeout)
    if len(stdout) + len(stderr) > 16 * 1024 * 1024:
        raise SupervisorError("lifecycle command output exceeded 16 MiB")
    return {
        "argv": list(argv), "return_code": int(rc),
        "elapsed_seconds": round(time.monotonic() - started, 6),
        "stdout_sha256": sha256_bytes(stdout), "stderr_sha256": sha256_bytes(stderr),
        "stdout_tail": stdout.decode("utf-8", "replace")[-4096:],
        "stderr_tail": stderr.decode("utf-8", "replace")[-4096:],
        "loaded": None,
    }


def status_event(name: str, *, action: str,
                 runner: Callable[[Sequence[str], int], tuple[int, bytes, bytes]] | None = None) -> dict[str, Any]:
    event = run_capture([AA_STATUS], timeout=30, runner=runner)
    event["action"] = action
    if event["return_code"] == 0:
        matches = [line.strip() for line in event["stdout_tail"].splitlines() if line.strip() == name]
        event["loaded"] = len(matches) == 1
    return event


def child_argv(phase: str, *, lifecycle_events: Sequence[Mapping[str, Any]] | None = None,
               zero_residue: bool = False) -> list[str]:
    if phase not in FULL_PHASES:
        raise SupervisorError(f"unknown frozen phase: {phase}")
    argv = [
        SETPRIV, "--reuid=1000", "--regid=1000", "--clear-groups", "--",
        PYTHON, str(GATE_PATH), phase, "--authorization-token-file", str(TOKEN_PATH),
    ]
    if lifecycle_events is not None:
        argv += ["--lifecycle-events-json", json.dumps(list(lifecycle_events), ensure_ascii=False,
                                                       sort_keys=True, separators=(",", ":"))]
    if zero_residue:
        argv.append("--zero-residue")
    return argv


def cleanup_profile(profile: Mapping[str, Any], *, must_unload: bool,
                    events: list[dict[str, Any]], failures: list[str],
                    runner: Callable[[Sequence[str], int], tuple[int, bytes, bytes]] | None = None) -> bool:
    path = resolve_pinned(profile["path"])
    name = str(profile["name"])
    if must_unload:
        try:
            unload = run_capture([APPARMOR_PARSER, "-K", "-T", "-R", "--", str(path)],
                                 timeout=60, runner=runner)
            unload["action"] = "unload"
            events.append(unload)
            if unload["return_code"] != 0:
                failures.append(f"{name}: unload rc={unload['return_code']}")
        except Exception as exc:
            failures.append(f"{name}: unload exception={exc}")
    try:
        residue = status_event(name, action="verify_zero_residue", runner=runner)
        events.append(residue)
        if residue["return_code"] != 0:
            failures.append(f"{name}: zero-residue aa-status rc={residue['return_code']}")
            return False
        if residue["loaded"]:
            failures.append(f"{name}: profile residue remains")
            return False
        return True
    except Exception as exc:
        failures.append(f"{name}: zero-residue status exception={exc}")
        return False


def run_profile_phase(phase: str, profile: Mapping[str, Any], *,
                      runner: Callable[[Sequence[str], int], tuple[int, bytes, bytes]] | None = None) -> tuple[int, list[dict[str, Any]], bool, list[str]]:
    events: list[dict[str, Any]] = []
    failures: list[str] = []
    must_unload = False
    body_rc = 2
    path = resolve_pinned(profile["path"])
    name = str(profile["name"])
    try:
        initial = status_event(name, action="verify_initial_absence", runner=runner)
        events.append(initial)
        if initial["return_code"] != 0:
            raise SupervisorError(f"initial aa-status rc={initial['return_code']}: {name}")
        if initial["loaded"]:
            raise SupervisorError(f"profile already loaded before phase: {name}")
        load = run_capture([APPARMOR_PARSER, "-K", "-T", "-a", "--", str(path)],
                           timeout=60, runner=runner)
        load["action"] = "load"
        events.append(load)
        if load["return_code"] != 0:
            raise SupervisorError(f"profile load rc={load['return_code']}: {name}")
        # Set immediately after parser rc=0; status failure must still unload.
        must_unload = True
        loaded = status_event(name, action="verify_loaded", runner=runner)
        events.append(loaded)
        if loaded["return_code"] != 0:
            raise SupervisorError(f"loaded aa-status rc={loaded['return_code']}: {name}")
        if not loaded["loaded"]:
            raise SupervisorError(f"profile absent after successful load: {name}")
        body = run_capture(child_argv(phase), timeout=5500, runner=runner)
        body["action"] = "uid1000_gid1000_zero_groups_gate_body"
        events.append(body)
        body_rc = body["return_code"]
        if body_rc != 0:
            raise SupervisorError(f"gate body rc={body_rc}: {phase}")
    except Exception as exc:
        failures.append(f"{phase}: body/lifecycle exception={exc}")
    zero = cleanup_profile(profile, must_unload=must_unload, events=events,
                           failures=failures, runner=runner)
    # Invalidate the caller's transient authorization after every individual
    # profile lifecycle.  This is attempted independently of body, unload, and
    # status outcomes, so one cleanup exception cannot suppress another.
    sudo_k(events=events, failures=failures, runner=runner)
    if not failures and zero and body_rc == 0:
        try:
            finalize = run_capture(
                child_argv(phase, lifecycle_events=events, zero_residue=True),
                timeout=300, runner=runner,
            )
            finalize["action"] = "uid1000_gid1000_zero_groups_gate_finalize"
            events.append(finalize)
            if finalize["return_code"] != 0:
                failures.append(f"{phase}: lifecycle finalizer rc={finalize['return_code']}")
        except Exception as exc:
            failures.append(f"{phase}: lifecycle finalizer exception={exc}")
    return body_rc, events, zero, failures


def run_wrapper(*, runner: Callable[[Sequence[str], int], tuple[int, bytes, bytes]] | None = None) -> tuple[int, list[dict[str, Any]], list[str]]:
    events: list[dict[str, Any]] = []
    failures: list[str] = []
    try:
        body = run_capture(child_argv("wrapper"), timeout=300, runner=runner)
        body["action"] = "uid1000_gid1000_zero_groups_wrapper_no_profile"
        events.append(body)
        if body["return_code"] != 0:
            failures.append(f"wrapper: gate rc={body['return_code']}")
        return body["return_code"], events, failures
    except Exception as exc:
        failures.append(f"wrapper: exception={exc}")
        return 2, events, failures


def sudo_k(*, events: list[dict[str, Any]], failures: list[str],
           runner: Callable[[Sequence[str], int], tuple[int, bytes, bytes]] | None = None) -> None:
    try:
        event = run_capture([SUDO, "-k"], timeout=30, runner=runner)
        event["action"] = "sudo_timestamp_invalidate"
        events.append(event)
        if event["return_code"] != 0:
            failures.append(f"sudo -k rc={event['return_code']}")
    except Exception as exc:
        failures.append(f"sudo -k exception={exc}")


def execute_one_shot(*,
                     runner: Callable[[Sequence[str], int], tuple[int, bytes, bytes]] | None = None) -> dict[str, Any]:
    if os.geteuid() != 0 or os.getuid() != 0:
        raise SupervisorError("dynamic entry requires real/effective root")
    if os.environ.get("SUDO_UID") != "1000" or os.environ.get("SUDO_GID") != "1000":
        raise SupervisorError("dynamic entry requires exact SUDO_UID=1000 and SUDO_GID=1000")
    current, profiles = policy_and_profiles()
    static_gate_report(token_may_exist=True)
    validate_token(current, profiles)
    all_events: list[dict[str, Any]] = []
    failures: list[str] = []
    completed: list[str] = []
    try:
        for phase, role in PROFILE_PHASES[:2]:
            rc, events, zero, phase_failures = run_profile_phase(phase, profiles[role], runner=runner)
            all_events.extend(events); failures.extend(phase_failures)
            if rc != 0 or not zero or phase_failures:
                break
            completed.append(phase)
        if not failures and completed == ["source-copy", "patch"]:
            rc, events, phase_failures = run_wrapper(runner=runner)
            all_events.extend(events); failures.extend(phase_failures)
            if rc == 0 and not phase_failures:
                completed.append("wrapper")
        for phase, role in PROFILE_PHASES[2:]:
            if failures or completed != list(FULL_PHASES[:len(completed)]):
                break
            expected_before = ["source-copy", "patch", "wrapper"] if phase == "build" else [
                "source-copy", "patch", "wrapper", "build"
            ]
            if completed != expected_before:
                break
            rc, events, zero, phase_failures = run_profile_phase(phase, profiles[role], runner=runner)
            all_events.extend(events); failures.extend(phase_failures)
            if rc != 0 or not zero or phase_failures:
                break
            completed.append(phase)
    finally:
        # Independent final read-only sweep.  Never remove a profile this
        # process did not prove it loaded; each owned profile's removal was
        # already attempted inside run_profile_phase.
        for role in ("SOURCE_COPY", "PATCH", "BUILD", "STATIC_AUDIT"):
            profile = profiles[role]
            try:
                observed = status_event(profile["name"], action="final_sweep_status", runner=runner)
                all_events.append(observed)
                if observed["return_code"] != 0:
                    failures.append(f"{profile['name']}: final sweep aa-status rc={observed['return_code']}")
                    continue
                if observed["loaded"]:
                    failures.append(f"{profile['name']}: final sweep found profile residue")
            except Exception as exc:
                failures.append(f"{profile['name']}: final sweep exception={exc}")
    token_value = read_json(TOKEN_PATH, limit=16384)
    zero_residue = not any(
        failure for failure in failures
        if "residue" in failure or "unload" in failure or "aa-status" in failure
        or "profile absent" in failure or "profile already loaded" in failure
    )
    result = {
        "schema_version": "smpcc-r8-liquid-motion-gauge-gpu-build-supervisor-receipt-v11",
        "build_id": BUILD_ID,
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "status": (
            "PASS_MOTION_GAUGE_GPU_BUILD_V11_ONE_SHOT_STATIC_AUDIT_DEVELOPMENT_CANDIDATE"
            if completed == list(FULL_PHASES) and not failures
            else "FAIL_MOTION_GAUGE_GPU_BUILD_V11_STOP_PRESERVE_EVIDENCE"
        ),
        "campaign_id": CAMPAIGN_ID,
        "completed_phases": completed,
        "events": all_events,
        "failures": failures,
        "profile_zero_residue": zero_residue,
        "candidate_executed": False,
        "gpu_exposed": False,
        "network_used": False,
        "root_make": False,
        "policy_sha256": file_identity(POLICY_PATH)["sha256"],
        "gate_sha256": file_identity(GATE_PATH)["sha256"],
        "supervisor_sha256": file_identity(Path(__file__).resolve())["sha256"],
        "token_sha256": file_identity(TOKEN_PATH)["sha256"],
        "authorization_reference_sha256": token_value["authorization_reference_sha256"],
        "static_gate_status": "PASS_MOTION_GAUGE_GPU_BUILD_V11_STATIC_DEFAULT_DENY",
    }
    receipt_path = SUPERVISOR_FINAL_PATH if result["status"].startswith("PASS_") else SUPERVISOR_FAILURE_PATH
    write_new(receipt_path, result)
    return result


def self_check() -> dict[str, Any]:
    current, profiles = policy_and_profiles()
    receipt_schema = read_json(SUPERVISOR_RECEIPT_SCHEMA_PATH)
    Draft202012Validator.check_schema(receipt_schema)
    gate_report = static_gate_report()
    argv = {phase: child_argv(phase) for phase in FULL_PHASES}
    for phase, current_argv in argv.items():
        prefix = [SETPRIV, "--reuid=1000", "--regid=1000", "--clear-groups", "--", PYTHON, str(GATE_PATH), phase]
        if current_argv[:len(prefix)] != prefix:
            raise SupervisorError(f"setpriv/gate prefix drift: {phase}")
    if current["safety"] != {
        **current["safety"], "network_used": False, "gpu_exposed": False,
        "candidate_executed": False, "root_make_allowed": False,
    }:
        raise SupervisorError("policy safety contract drift")
    return {
        "status": "PASS_MOTION_GAUGE_GPU_BUILD_LIFECYCLE_SUPERVISOR_V11_STATIC_ONLY",
        "campaign_id": CAMPAIGN_ID,
        "policy_sha256": file_identity(POLICY_PATH)["sha256"],
        "gate_sha256": file_identity(GATE_PATH)["sha256"],
        "supervisor_sha256": file_identity(Path(__file__).resolve())["sha256"],
        "token_producer_sha256": file_identity(TOKEN_PRODUCER_PATH)["sha256"],
        "profile_sha256": [profiles[role]["sha256"] for _, role in PROFILE_PHASES],
        "full_state_machine": list(FULL_PHASES),
        "setpriv_uid": 1000, "setpriv_gid": 1000, "supplementary_groups": 0,
        "wrapper_profile": "NONE",
        "token_validated": False,
        "supervisor_receipt_schema_sha256": file_identity(SUPERVISOR_RECEIPT_SCHEMA_PATH)["sha256"],
        "gate_static_status": gate_report["status"],
        "system_actions_performed": False,
        "profile_loaded": False,
        "make_run": False,
        "compiler_run": False,
        "candidate_executed": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", "execute"), nargs="?", default="self-check")
    args = parser.parse_args(argv)
    try:
        if args.command == "self-check":
            report = self_check()
        else:
            report = execute_one_shot()
        print(json.dumps(report, ensure_ascii=False, sort_keys=True, separators=(",", ":")))
        return 0 if report["status"].startswith("PASS_") else 1
    except Exception as exc:
        print(json.dumps({"status": "FAIL_MOTION_GAUGE_GPU_BUILD_SUPERVISOR_V11", "error": str(exc)},
                         ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
