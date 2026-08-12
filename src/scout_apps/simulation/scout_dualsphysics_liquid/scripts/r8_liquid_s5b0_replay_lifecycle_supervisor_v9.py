#!/usr/bin/env python3
"""Root-only one-shot lifecycle for the exact S5B0 primary replay v9.

The default CLI is static-only.  The dynamic entry is intentionally a single
state machine: validate the exact token, create the start witness, anchor the
current kernel journal cursor, load/verify one profile, execute exactly one
staged candidate, unload in ``finally``, prove zero residue, run the closed QC
and transaction, and publish either a final or failure receipt create-new.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import re
import stat
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"
POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5b0_replay_execution_policy_v9.json"
RECEIPT_SCHEMA_PATH = ROOT / "schema/target_host_s5b0_replay_execution_receipt_v9.json"
GATE_PATH = SCRIPTS / "r8_liquid_s5b0_replay_execution_gate_v9.py"
RUNTIME_PATH = SCRIPTS / "r8_liquid_s5b0_replay_runtime_supervisor_v9.py"
TOKEN_PRODUCER_PATH = SCRIPTS / "r8_liquid_s5b0_replay_authorization_token_v9.py"
APPARMOR_PARSER = "/usr/sbin/apparmor_parser"
AA_STATUS = "/usr/sbin/aa-status"
SUDO = "/usr/bin/sudo"
SETPRIV = "/usr/bin/setpriv"
PYTHON = "/usr/bin/python3"
ZERO_SHA = hashlib.sha256(b"").hexdigest()
CURSOR_RE = re.compile(r"^-- cursor: (\S+)$", re.MULTILINE)
XID_RE = re.compile(r"NVRM:\s*Xid", re.IGNORECASE)


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise LifecycleV9Error(f"cannot load frozen component: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


gate = _load("s5b0_gate_v9_for_lifecycle", GATE_PATH)
runtime = _load("s5b0_runtime_v9_for_lifecycle", RUNTIME_PATH)


class LifecycleV9Error(RuntimeError):
    """A frozen identity, journal, lifecycle, residue, or receipt gate failed."""


def utc_now() -> str:
    return runtime.utc_now()


def canonical_json(value: Any) -> bytes:
    return gate.canonical_json(value)


def _run(argv: Sequence[str], timeout: int, *,
         runner: Callable[[Sequence[str], int], tuple[int, bytes, bytes]] | None = None
         ) -> dict[str, Any]:
    started = time.monotonic()
    if runner is None:
        result = subprocess.run(list(argv), stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, check=False, timeout=timeout,
                                close_fds=True, env={"PATH": "/usr/bin:/usr/sbin", "LC_ALL": "C", "LANG": "C"})
        rc, stdout, stderr = result.returncode, result.stdout, result.stderr
    else:
        rc, stdout, stderr = runner(list(argv), timeout)
    if len(stdout) + len(stderr) > 16 * 1024 * 1024:
        raise LifecycleV9Error("lifecycle command output exceeds bound")
    return {"argv": list(argv), "return_code": int(rc),
            "elapsed_seconds": round(time.monotonic() - started, 6),
            "stdout_sha256": hashlib.sha256(stdout).hexdigest(),
            "stderr_sha256": hashlib.sha256(stderr).hexdigest(),
            "stdout": stdout, "stderr": stderr}


def _status(name: str, *, action: str,
            runner: Callable[[Sequence[str], int], tuple[int, bytes, bytes]] | None = None
            ) -> dict[str, Any]:
    event = _run([AA_STATUS], 30, runner=runner)
    event["action"] = action
    event["loaded"] = (event["return_code"] == 0 and
                       sum(line.strip() == name for line in event["stdout"].decode("utf-8", "replace").splitlines()) == 1)
    return event


def _event_public(event: Mapping[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in event.items() if key not in {"stdout", "stderr"}}


def child_argv(policy: Mapping[str, Any], phase: str = "execute-one-shot", *,
               execution: Mapping[str, Any] | None = None, xid_count: int | None = None) -> list[str]:
    """Exact privilege drop used for every non-lifecycle replay operation."""
    if phase not in {"execute-one-shot", "postprocess-one-shot"}:
        raise LifecycleV9Error("unknown runtime child phase")
    argv = [SETPRIV, "--reuid=1000", "--regid=1000", "--clear-groups", "--",
            PYTHON, str(RUNTIME_PATH), phase, "--authorization-token-file",
            policy["replay"]["token_path"]]
    if phase == "postprocess-one-shot":
        if execution is None or xid_count is None:
            raise LifecycleV9Error("postprocess child inputs are absent")
        encoded = json.dumps(dict(execution), allow_nan=False, sort_keys=True, separators=(",", ":"))
        if len(encoded.encode("utf-8")) > 262144:
            raise LifecycleV9Error("postprocess execution handoff exceeds bound")
        argv.extend(("--execution-json", encoded, "--xid-count", str(xid_count)))
    return argv


def _cursor(raw: bytes) -> str:
    matches = CURSOR_RE.findall(raw.decode("utf-8", "strict"))
    if len(matches) != 1 or len(matches[0]) > 4096:
        raise LifecycleV9Error("journal cursor anchor is absent or ambiguous")
    return matches[0]


def _boot_id(path: str) -> str:
    value = Path(path).read_text(encoding="ascii").strip()
    if re.fullmatch(r"[0-9a-f-]{36}", value) is None:
        raise LifecycleV9Error("kernel boot identity is malformed")
    return value


def journal_anchor(policy: Mapping[str, Any], *,
                   runner: Callable[[Sequence[str], int], tuple[int, bytes, bytes]] | None = None
                   ) -> dict[str, Any]:
    boot = _boot_id(policy["journal"]["boot_id_path"])
    event = _run(policy["journal"]["anchor_argv"], 30, runner=runner)
    if event["return_code"] != 0:
        raise LifecycleV9Error("journal cursor anchor command failed")
    return {"boot_id": boot, "cursor": _cursor(event["stdout"]), "event": _event_public(event)}


def journal_finish(policy: Mapping[str, Any], anchor: Mapping[str, Any], *,
                   runner: Callable[[Sequence[str], int], tuple[int, bytes, bytes]] | None = None
                   ) -> dict[str, Any]:
    sync = _run(policy["journal"]["sync_argv"], 30, runner=runner)
    if sync["return_code"] != 0:
        raise LifecycleV9Error("journal sync failed")
    boot_after = _boot_id(policy["journal"]["boot_id_path"])
    if boot_after != anchor["boot_id"]:
        raise LifecycleV9Error("host rebooted during exact replay")
    argv = [anchor["boot_id"] if item == "{boot_id}" else anchor["cursor"] if item == "{cursor}" else item
            for item in policy["journal"]["query_argv_template"]]
    query = _run(argv, 60, runner=runner)
    if query["return_code"] != 0:
        raise LifecycleV9Error("journal cursor query failed")
    end_cursor = _cursor(query["stdout"])
    xid_count = len(XID_RE.findall(query["stdout"].decode("utf-8", "replace")))
    return {"boot_id_before": anchor["boot_id"], "boot_id_after": boot_after,
            "start_cursor": anchor["cursor"], "end_cursor": end_cursor,
            "xid_count": xid_count, "same_boot": True, "cursor_anchored": True,
            "events": [anchor["event"], _event_public(sync), _event_public(query)]}


def _process_residue(policy: Mapping[str, Any]) -> list[int]:
    markers = (policy["replay"]["replay_id"], policy["replay"]["stage_root"],
               policy["replay"]["profile_name"])
    found: list[int] = []
    for child in Path("/proc").iterdir():
        if not child.name.isdigit() or int(child.name) == os.getpid():
            continue
        try:
            raw = (child / "cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8", "replace")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if any(marker in raw for marker in markers):
            found.append(int(child.name))
    return found


def _mount_residue(policy: Mapping[str, Any]) -> list[str]:
    stage = policy["replay"]["stage_root"]
    result = []
    for line in Path("/proc/self/mountinfo").read_text(encoding="utf-8").splitlines():
        if stage in line or policy["replay"]["replay_id"] in line:
            result.append(line)
    return result


def _empty_evidence(token: Mapping[str, Any] | None, error: str | None = None, *,
                    runtime_child_attempted: bool = False) -> dict[str, Any]:
    return {"runtime_child_attempted": runtime_child_attempted,
            "argv": [], "return_code": None, "elapsed_seconds": 0.0,
            "stdout_sha256": ZERO_SHA, "stderr_sha256": ZERO_SHA,
            "resource_log_sha256": ZERO_SHA, "output_inventory_sha256": ZERO_SHA,
            "result_package_sha256": ZERO_SHA,
            "candidate": None if token is None else token["candidate"],
            "profile": None if token is None else token["profile"],
            "runtime_attempt_receipt": None, "lifecycle_receipt": None,
            "error": error}


def _claims(*, executed: bool, gpu: bool) -> dict[str, Any]:
    return {"planned_denominator": 1, "candidate_execution_count": int(executed),
            "candidate_source_executed": False, "staged_candidate_executed": executed,
            "gpu_exposed": gpu, "network_used": False, "optional_bag_read": False,
            "c2_executed": False, "second_execution": False,
            "source_outcome_overridden": False, "development_only": True,
            "physical_reference_pending": True, "physical_fidelity_validated": False,
            "formal": False, "production": False}


def make_receipt(policy: Mapping[str, Any], policy_sha: str, *, status: str, phase: str,
                 evidence: Mapping[str, Any], journal: Mapping[str, Any],
                 lifecycle: Mapping[str, Any], claims: Mapping[str, Any]) -> dict[str, Any]:
    value = {"schema_version": "smpcc-r8-liquid-s5b0-replay-execution-receipt-v9",
             "document_type": "SMPCC_R8_LIQUID_S5B0_REPLAY_EXECUTION_RECEIPT_V9",
             "status": status, "phase": phase,
             "attempt_id": policy["selection"]["attempt_id"],
             "replay_id": policy["replay"]["replay_id"], "policy_sha256": policy_sha,
             "created_at": utc_now(), "evidence": dict(evidence),
             "journal": {key: journal.get(key) for key in ("boot_id_before", "boot_id_after",
                         "start_cursor", "end_cursor", "xid_count", "same_boot", "cursor_anchored")},
             "lifecycle": dict(lifecycle), "claims": dict(claims)}
    schema = json.loads(RECEIPT_SCHEMA_PATH.read_bytes())
    Draft202012Validator(schema).validate(value)
    return value


def _write_receipt(path: str, value: Mapping[str, Any]) -> dict[str, Any]:
    return runtime.write_new(Path(path), canonical_json(value), 0o600)


def _assert_runtime_attempt_fresh(policy: Mapping[str, Any]) -> None:
    """Stop before journal/profile work when the unique attempt is consumed.

    The O_EXCL write immediately before child creation remains the atomic race
    gate.  This earlier check makes crash recovery fail closed without loading
    AppArmor merely to rediscover an already durable attempt witness.
    """

    if os.path.lexists(policy["replay"]["attempt_receipt"]):
        raise LifecycleV9Error("runtime attempt already consumed")


def execute_one_shot(*,
                     runner: Callable[[Sequence[str], int], tuple[int, bytes, bytes]] | None = None
                     ) -> dict[str, Any]:
    if os.getuid() != 0 or os.geteuid() != 0 or os.environ.get("SUDO_UID") != "1000" or os.environ.get("SUDO_GID") != "1000":
        raise LifecycleV9Error("dynamic entry requires exact sudo root boundary from uid/gid 1000")
    policy, policy_sha = gate.read_policy()
    token = gate.validate_token(Path(policy["replay"]["token_path"]))
    profile_path = Path(policy["replay"]["profile_path"])
    if token["profile"] != gate._file_identity(profile_path):
        raise LifecycleV9Error("exact profile identity drift")
    _assert_runtime_attempt_fresh(policy)
    anchor = journal_anchor(policy, runner=runner)
    journal = {"boot_id_before": anchor["boot_id"], "boot_id_after": None,
               "start_cursor": anchor["cursor"], "end_cursor": None, "xid_count": 0,
               "same_boot": True, "cursor_anchored": True}
    lifecycle = {"profile_loaded": False, "profile_unloaded": False,
                 "zero_profile_residue": False, "zero_process_residue": False,
                 "zero_mount_residue": False, "sudo_timestamp_cleared": False,
                 "failure_preserved": True}
    start = make_receipt(policy, policy_sha, status="STARTED", phase="START",
                         evidence=_empty_evidence(token), journal=journal,
                         lifecycle=lifecycle, claims=_claims(executed=False, gpu=False))
    _write_receipt(policy["replay"]["start_receipt"], start)
    execution = None; inventory = None; inventory_sha = ZERO_SHA; result_identity = None
    runtime_child_attempted = False; runtime_attempt_identity = None
    lifecycle_identity = None
    loaded_by_us = False; events: list[dict[str, Any]] = []
    failure: Exception | None = None
    try:
        # Source staging, candidate launch, QC and publication are all owned by
        # the uid/gid 1000 child.  Root remains only for AppArmor/journal/sudo-k.
        initial = _status(policy["replay"]["profile_name"], action="verify_initial_absence", runner=runner)
        events.append(_event_public(initial))
        if initial["return_code"] != 0 or initial["loaded"]:
            raise LifecycleV9Error("profile is loaded or aa-status failed before replay")
        load = _run([APPARMOR_PARSER, "-K", "-T", "-a", "--", str(profile_path)], 60, runner=runner)
        load["action"] = "load"; events.append(_event_public(load))
        if load["return_code"] != 0:
            raise LifecycleV9Error("profile load failed")
        loaded_by_us = True; lifecycle["profile_loaded"] = True
        verified = _status(policy["replay"]["profile_name"], action="verify_loaded", runner=runner)
        events.append(_event_public(verified))
        if verified["return_code"] != 0 or not verified["loaded"]:
            raise LifecycleV9Error("profile verification after load failed")
        # Commit a durable conservative witness before process creation.  Any
        # timeout, launch error, parent crash, non-zero exit, or malformed child
        # JSON therefore consumes the unique execution attempt.
        attempt_evidence = _empty_evidence(token, runtime_child_attempted=True)
        attempt = make_receipt(policy, policy_sha,
            status="RUNTIME_CHILD_ATTEMPT_COMMITTED", phase="ATTEMPT",
            evidence=attempt_evidence, journal=journal, lifecycle=lifecycle,
            claims=_claims(executed=True, gpu=True))
        runtime_attempt_identity = _write_receipt(
            policy["replay"]["attempt_receipt"], attempt)
        runtime_child_attempted = True
        body = _run(child_argv(policy), policy["resources"]["wall_timeout_seconds"] + 300,
                    runner=runner)
        body["action"] = "uid1000_gid1000_zero_groups_runtime_one_shot"
        events.append(_event_public(body))
        if body["return_code"] != 0:
            raise LifecycleV9Error("uid1000 runtime one-shot failed")
        execution = json.loads(body["stdout"])
    except Exception as exc:
        failure = exc
    finally:
        if loaded_by_us:
            unload = _run([APPARMOR_PARSER, "-K", "-T", "-R", "--", str(profile_path)], 60, runner=runner)
            unload["action"] = "unload"; events.append(_event_public(unload))
            lifecycle["profile_unloaded"] = unload["return_code"] == 0
            if unload["return_code"] != 0 and failure is None:
                failure = LifecycleV9Error("profile unload failed")
        residue = _status(policy["replay"]["profile_name"], action="verify_zero_profile_residue", runner=runner)
        events.append(_event_public(residue))
        lifecycle["zero_profile_residue"] = residue["return_code"] == 0 and not residue["loaded"]
        process_rows = _process_residue(policy); mount_rows = _mount_residue(policy)
        lifecycle["zero_process_residue"] = not process_rows
        lifecycle["zero_mount_residue"] = not mount_rows
        clear = _run([SUDO, "-k"], 30, runner=runner); clear["action"] = "sudo_timestamp_invalidate"
        events.append(_event_public(clear)); lifecycle["sudo_timestamp_cleared"] = clear["return_code"] == 0
        if not all(lifecycle[key] for key in ("zero_profile_residue", "zero_process_residue",
                                              "zero_mount_residue", "sudo_timestamp_cleared")) and failure is None:
            failure = LifecycleV9Error("one or more lifecycle residue gates failed")
    try:
        journal = journal_finish(policy, anchor, runner=runner)
        if journal["xid_count"] != 0 and failure is None:
            failure = LifecycleV9Error("kernel Xid detected in exact cursor interval")
    except Exception as exc:
        if failure is None: failure = exc
    try:
        if failure is None and execution is not None:
            post = _run(child_argv(policy, "postprocess-one-shot", execution=execution,
                                  xid_count=journal["xid_count"]), 900, runner=runner)
            post["action"] = "uid1000_gid1000_zero_groups_postprocess_one_shot"
            events.append(_event_public(post))
            if post["return_code"] != 0:
                raise LifecycleV9Error("uid1000 postprocess one-shot failed")
            post_report = json.loads(post["stdout"])
            if post_report.get("status") != "PASS_S5B0_REPLAY_POSTPROCESS_V9":
                raise LifecycleV9Error("postprocess result status differs")
            inventory_sha = post_report["inventory_sha256"]
            result_identity = post_report["result_package"]
            evidence = {"runtime_child_attempted": True,
                "argv": execution["argv"], "return_code": execution["return_code"],
                "elapsed_seconds": execution["elapsed_seconds"],
                "stdout_sha256": execution["stdout"]["sha256"],
                "stderr_sha256": execution["stderr"]["sha256"],
                "resource_log_sha256": execution["resource_log"]["sha256"],
                "output_inventory_sha256": inventory_sha,
                "result_package_sha256": result_identity["sha256"],
                "candidate": token["candidate"], "profile": token["profile"],
                "runtime_attempt_receipt": runtime_attempt_identity,
                "lifecycle_receipt": None, "error": None}
            lifecycle_receipt = make_receipt(policy, policy_sha,
                status="LIFECYCLE_CLEANUP_PASS_AWAITING_FINAL",
                phase="LIFECYCLE", evidence=evidence, journal=journal, lifecycle=lifecycle,
                claims=_claims(executed=True, gpu=True))
            lifecycle_identity = _write_receipt(policy["replay"]["lifecycle_receipt"], lifecycle_receipt)
            # FINAL is the success commit marker and is deliberately last.  It
            # binds the already durable lifecycle witness by exact identity.
            final_evidence = dict(evidence)
            final_evidence["lifecycle_receipt"] = lifecycle_identity
            final = make_receipt(policy, policy_sha,
                status="S5B0_PRIMARY_R7_EXECUTED_MOTION_REPLAY_PASS_DEVELOPMENT_ONLY",
                phase="FINAL", evidence=final_evidence, journal=journal, lifecycle=lifecycle,
                claims=_claims(executed=True, gpu=True))
            final_identity = _write_receipt(policy["replay"]["final_receipt"], final)
            return {"status": final["status"], "final_receipt": final_identity,
                    "lifecycle_receipt": lifecycle_identity, "result_package": result_identity,
                    "events": events, "journal": journal}
    except Exception as exc:
        failure = exc
    executed = runtime_child_attempted
    evidence = _empty_evidence(token, str(failure),
                               runtime_child_attempted=runtime_child_attempted)
    evidence["runtime_attempt_receipt"] = runtime_attempt_identity
    evidence["lifecycle_receipt"] = lifecycle_identity
    if execution is not None:
        evidence.update({"argv": execution["argv"], "return_code": execution["return_code"],
            "elapsed_seconds": execution["elapsed_seconds"],
            "stdout_sha256": execution["stdout"]["sha256"],
            "stderr_sha256": execution["stderr"]["sha256"],
            "resource_log_sha256": execution["resource_log"]["sha256"],
            "output_inventory_sha256": inventory_sha,
            "result_package_sha256": (ZERO_SHA if result_identity is None
                                      else result_identity["sha256"])})
    failure_receipt = make_receipt(policy, policy_sha, status="STOP_AND_PRESERVE_EVIDENCE",
        phase="FAILURE", evidence=evidence, journal=journal, lifecycle=lifecycle,
        claims=_claims(executed=executed, gpu=executed))
    failure_identity = _write_receipt(policy["replay"]["failure_receipt"], failure_receipt)
    return {"status": "STOP_AND_PRESERVE_EVIDENCE", "failure_receipt": failure_identity,
            "events": events, "journal": journal, "error": str(failure)}


def self_check() -> dict[str, Any]:
    policy, policy_sha = gate.read_policy(); gate.validate_static(policy)
    schema = json.loads(RECEIPT_SCHEMA_PATH.read_bytes())
    Draft202012Validator.check_schema(schema); gate.assert_deep_closed(schema)
    if policy["execution_components"]["lifecycle_supervisor"] != str(Path(__file__).resolve()):
        raise LifecycleV9Error("policy lifecycle path drift")
    expected_prefix = [SETPRIV, "--reuid=1000", "--regid=1000", "--clear-groups", "--",
                       PYTHON, str(RUNTIME_PATH), "execute-one-shot"]
    if child_argv(policy)[:len(expected_prefix)] != expected_prefix:
        raise LifecycleV9Error("setpriv runtime child contract drift")
    return {"status": "PASS_S5B0_REPLAY_LIFECYCLE_SUPERVISOR_V9_STATIC_ONLY",
            "policy_sha256": policy_sha, "profile_name": policy["replay"]["profile_name"],
            "state_machine": ["TOKEN", "START", "JOURNAL_ANCHOR", "PREPARE", "LOAD", "VERIFY",
                              "RUN_ONCE", "FINALLY_UNLOAD", "ZERO_RESIDUE", "JOURNAL_QUERY", "QC",
                              "RENAME_NOREPLACE", "FINAL_OR_FAILURE"],
            "journal_boot_cursor_anchor": True, "failure_preserved": True,
            "setpriv_uid": 1000, "setpriv_gid": 1000, "supplementary_groups": 0,
            "execute_public_default": False, "files_written": False, "external_source_read": False,
            "candidate_executed": False, "gpu_exposed": False, "profile_loaded": False,
            "sudo_used": False, "optional_bag_read": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check", "execute"), nargs="?", default="self-check")
    args = parser.parse_args(argv)
    try:
        report = self_check() if args.command == "self-check" else execute_one_shot()
        print(json.dumps(report, ensure_ascii=False, sort_keys=True, separators=(",", ":")))
        return 0 if report["status"].startswith(("PASS_", "S5B0_")) else 1
    except Exception as exc:
        print(json.dumps({"status": "FAIL_S5B0_REPLAY_LIFECYCLE_SUPERVISOR_V9",
                          "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
