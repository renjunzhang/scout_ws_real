#!/usr/bin/env python3
"""Execute the reviewed Phase-Rejoining formal matrix exactly once.

The readiness auditor intentionally never starts trials.  This separate entry
point requires a matching READY_NOT_EXECUTED report and a human-authored
approval file, then follows the frozen order sequentially without retries.
"""

import argparse
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any, Dict, List, Sequence

import yaml


FORMAL_SEEDS = list(range(3101, 3117))
REQUIRED_CONDITIONS = ("C0", "C1", "C2", "C3", "C4", "IS")
APPROVAL_SCHEMA = "spmpc_formal_simulation_human_approval_v1"
READINESS_SCHEMA = "spmpc_formal_simulation_readiness_v2"
MANIFEST_SCHEMA = "spmpc_phase_rejoin_formal_campaign_v1"
SUMMARY_SCHEMA = "spmpc_closed_loop_trial_summary_v2"
ALLOWED_OUTPUT_ROOT = Path("/data/a/spmpc_exec_identification")


def _load_auditor():
    adjacent = Path(__file__).resolve().parent / \
        "run_independent_plant_campaign.py"
    if not adjacent.is_file():
        raise RuntimeError("frozen readiness auditor is absent")
    spec = importlib.util.spec_from_file_location(
        "phase_rejoin_formal_auditor", adjacent)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise RuntimeError("JSON root is not a mapping: {}".format(path))
    return value


def _load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise RuntimeError("YAML root is not a mapping: {}".format(path))
    return value


def _sha256_text(value: Any) -> bool:
    return (isinstance(value, str) and len(value) == 64 and
            all(character in "0123456789abcdef" for character in value))


def validate_readiness(
        readiness: Dict[str, Any], session_path: Path,
        session_sha256: str) -> None:
    if (readiness.get("schema") != READINESS_SCHEMA or
            readiness.get("status") != "READY_NOT_EXECUTED" or
            readiness.get("formal_trials_started") is not False or
            readiness.get("reasons") != [] or
            Path(str(readiness.get("session", ""))).resolve() !=
                session_path.resolve() or
            readiness.get("session_sha256") != session_sha256):
        raise RuntimeError("readiness report does not authorize review")


def validate_approval(approval: Dict[str, Any], session_sha256: str) -> None:
    reviewer = approval.get("reviewer")
    approved_at = approval.get("approved_at")
    if (approval.get("schema") != APPROVAL_SCHEMA or
            approval.get("approved") is not True or
            approval.get("session_sha256") != session_sha256 or
            approval.get("formal_seeds_authorized") != FORMAL_SEEDS or
            not isinstance(reviewer, str) or not reviewer.strip() or
            not isinstance(approved_at, str) or not approved_at.strip()):
        raise RuntimeError("human approval is absent or mismatched")


def _resolve_reference(
        owner: Dict[str, Any], key: str, session_path: Path) -> Path:
    reference = owner.get(key)
    if not isinstance(reference, dict):
        raise RuntimeError("session reference is absent: {}".format(key))
    raw_path = reference.get("path")
    if not isinstance(raw_path, str) or not raw_path:
        raise RuntimeError("session path is absent: {}".format(key))
    path = Path(raw_path)
    if not path.is_absolute():
        path = session_path.parent / path
    path = path.resolve()
    if (not path.is_file() or
            reference.get("sha256") != _sha256_file(path)):
        raise RuntimeError("session reference changed: {}".format(key))
    return path


def _formal_order(path: Path) -> List[Dict[str, Any]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        if tuple(reader.fieldnames or ()) != (
                "schema", "block", "seed", "position", "condition"):
            raise RuntimeError("formal order columns are invalid")
    if len(rows) != len(FORMAL_SEEDS) * len(REQUIRED_CONDITIONS):
        raise RuntimeError("formal order does not contain 96 trials")
    parsed: List[Dict[str, Any]] = []
    for row_index, row in enumerate(rows):
        expected_block = row_index // len(REQUIRED_CONDITIONS) + 1
        expected_seed = FORMAL_SEEDS[expected_block - 1]
        expected_position = row_index % len(REQUIRED_CONDITIONS) + 1
        entry = {
            "block": int(row["block"]),
            "seed": int(row["seed"]),
            "position": int(row["position"]),
            "condition": row["condition"],
        }
        if (row.get("schema") != "spmpc_phase_rejoin_formal_order_v1" or
                entry["block"] != expected_block or
                entry["seed"] != expected_seed or
                entry["position"] != expected_position or
                entry["condition"] not in REQUIRED_CONDITIONS):
            raise RuntimeError("formal order row sequence is invalid")
        parsed.append(entry)
    for block in range(1, len(FORMAL_SEEDS) + 1):
        entries = parsed[
            (block - 1) * len(REQUIRED_CONDITIONS):
            block * len(REQUIRED_CONDITIONS)]
        if {entry["condition"] for entry in entries} != \
                set(REQUIRED_CONDITIONS):
            raise RuntimeError("formal order block is incomplete")
    return parsed


def _write_manifest(path: Path, manifest: Dict[str, Any]) -> None:
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write("\n")
    os.replace(str(temporary), str(path))


class TrialInfrastructureFailure(RuntimeError):
    """A trial process did not leave a complete, readable evidence pair."""


class TrialSummaryContractFailure(RuntimeError):
    """A zero-exit trial wrote a summary outside the frozen contract."""


def _trial_record(
        entry: Dict[str, Any], cycle_path: Path, summary_path: Path,
        session_sha256: str, process_returncode: int) -> Dict[str, Any]:
    try:
        summary = _load_json(summary_path)
    except (OSError, RuntimeError, json.JSONDecodeError) as error:
        raise TrialInfrastructureFailure(
            "trial process left no readable summary: {}".format(error)) \
            from error

    trial = summary.get("trial")
    status = summary.get("status")
    task_success = trial.get("task_success") if isinstance(trial, dict) \
        else None
    sequence_completed = trial.get("sequence_completed") \
        if isinstance(trial, dict) else None
    runtime_error = trial.get("runtime_error") \
        if isinstance(trial, dict) else None
    status_consistent = (
        status == "TRIAL_COMPLETE" and sequence_completed is True and
        task_success is True and runtime_error == "" and
        process_returncode == 0) or (
        status == "TRIAL_COMPLETE_WITH_FAILURE" and
        task_success is False and runtime_error == "" and
        process_returncode == 0) or (
        status == "RUNTIME_ERROR" and task_success is False and
        isinstance(runtime_error, str) and bool(runtime_error) and
        process_returncode != 0)
    if (summary.get("schema") != SUMMARY_SCHEMA or
            summary.get("seed") != entry["seed"] or
            summary.get("condition_id") != entry["condition"] or
            summary.get("formal_trials_started") is not True or
            summary.get("development_pilot_only") is not False or
            summary.get("frozen_session_sha256") != session_sha256 or
            not isinstance(sequence_completed, bool) or
            not status_consistent):
        error = "formal trial summary contract failed"
        if process_returncode != 0:
            raise TrialInfrastructureFailure(
                "trial process failed without a valid summary")
        raise TrialSummaryContractFailure(error)

    try:
        cycle_sha256 = _sha256_file(cycle_path)
        summary_sha256 = _sha256_file(summary_path)
    except OSError as error:
        raise TrialInfrastructureFailure(
            "formal trial evidence is incomplete: {}".format(error)) from error

    record = dict(entry)
    record.update({
        "cycle_csv": str(cycle_path),
        "cycle_csv_sha256": cycle_sha256,
        "summary_json": str(summary_path),
        "summary_json_sha256": summary_sha256,
        "process_returncode": process_returncode,
        "task_success": task_success,
        "status": status,
    })
    return record


def _clean_commit_matches(session: Dict[str, Any]) -> None:
    runtime = session.get("runtime", {})
    expected = runtime.get("git_commit")
    workspace = Path("/home/a/scout_ws")
    actual = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=str(workspace),
        text=True).strip()
    status = subprocess.check_output(
        ["git", "status", "--porcelain"], cwd=str(workspace),
        text=True).strip()
    if actual != expected or status:
        raise RuntimeError("working tree no longer matches frozen commit")


def execute(
        session_path: Path, readiness_path: Path, approval_path: Path,
        output_dir: Path) -> int:
    session_path = session_path.resolve()
    readiness_path = readiness_path.resolve()
    approval_path = approval_path.resolve()
    output_dir = output_dir.resolve()
    if os.path.commonpath((str(output_dir), str(ALLOWED_OUTPUT_ROOT))) != \
            str(ALLOWED_OUTPUT_ROOT.resolve()):
        raise RuntimeError("formal output is outside the evidence root")
    if output_dir.exists():
        raise RuntimeError("formal output already exists")

    session = _load_yaml(session_path)
    session_sha256 = _sha256_file(session_path)
    validate_readiness(
        _load_json(readiness_path), session_path, session_sha256)
    validate_approval(_load_json(approval_path), session_sha256)

    runtime = session.get("runtime")
    if not isinstance(runtime, dict):
        raise RuntimeError("frozen runtime contract is absent")
    frozen_runner = _resolve_reference(runtime, "runner", session_path)
    frozen_auditor = _resolve_reference(
        runtime, "readiness_auditor", session_path)
    if frozen_runner != Path(__file__).resolve():
        raise RuntimeError("invoke the formal runner frozen by the session")
    adjacent_auditor = (
        Path(__file__).resolve().parent /
        "run_independent_plant_campaign.py").resolve()
    if frozen_auditor != adjacent_auditor:
        raise RuntimeError("frozen readiness auditor is not adjacent to runner")

    auditor = _load_auditor()
    auditor.require_formal_session_schema(session)
    reasons, _ = auditor.audit_formal_session(session, session_path)
    if reasons:
        raise RuntimeError("formal session is NO-GO: {}".format(
            "; ".join(reasons)))
    _clean_commit_matches(session)

    assets = session["assets"]
    executable = _resolve_reference(runtime, "executable", session_path)
    plant = _resolve_reference(assets, "plant_config", session_path)
    path = _resolve_reference(assets, "path", session_path)
    artifact = _resolve_reference(
        assets, "phase_rejoin_artifact", session_path)
    order_path = _resolve_reference(
        assets, "formal_order", session_path)
    condition_paths = {
        name: _resolve_reference(
            session["conditions"][name], "config", session_path)
        for name in REQUIRED_CONDITIONS
    }
    order = _formal_order(order_path)

    output_dir.mkdir(parents=True, exist_ok=False)
    cycles_dir = output_dir / "cycles"
    summaries_dir = output_dir / "summaries"
    cycles_dir.mkdir()
    summaries_dir.mkdir()
    manifest_path = output_dir / "campaign_manifest.json"
    manifest: Dict[str, Any] = {
        "schema": MANIFEST_SCHEMA,
        "status": "RUNNING",
        "formal_trials_started": True,
        "frozen_session": str(session_path),
        "frozen_session_sha256": session_sha256,
        "readiness_report": str(readiness_path),
        "readiness_report_sha256": _sha256_file(readiness_path),
        "human_approval": str(approval_path),
        "human_approval_sha256": _sha256_file(approval_path),
        "retry_policy": "none",
        "replacement_policy":
            "infrastructure_failure_only_same_seed_condition",
        "planned_trial_count": len(order),
        "completed_trial_count": 0,
        "trials": [],
    }
    _write_manifest(manifest_path, manifest)

    for entry in order:
        stem = "block{block:02d}_seed{seed}_pos{position}_{condition}".format(
            **entry)
        cycle_path = cycles_dir / (stem + "_cycles.csv")
        summary_path = summaries_dir / (stem + "_summary.json")
        command = [
            str(executable),
            "--plant", str(plant),
            "--path", str(path),
            "--artifact", str(artifact),
            "--condition", str(condition_paths[entry["condition"]]),
            "--seed", str(entry["seed"]),
            "--cycle-csv", str(cycle_path),
            "--summary-json", str(summary_path),
            "--frozen-session-sha256", session_sha256,
        ]
        try:
            completed = subprocess.run(
                command, check=False, text=True,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except OSError as error:
            manifest["status"] = "STOPPED_INFRASTRUCTURE_FAILURE"
            manifest["failed_command"] = command
            manifest["infrastructure_failure"] = str(error)
            _write_manifest(manifest_path, manifest)
            raise RuntimeError(
                "formal trial process could not start; no retry allowed") \
                from error
        try:
            record = _trial_record(
                entry, cycle_path, summary_path, session_sha256,
                completed.returncode)
        except TrialInfrastructureFailure as error:
            manifest["status"] = "STOPPED_INFRASTRUCTURE_FAILURE"
            manifest["failed_command"] = command
            manifest["failed_returncode"] = completed.returncode
            manifest["failed_stdout"] = completed.stdout
            manifest["failed_stderr"] = completed.stderr
            manifest["infrastructure_failure"] = str(error)
            _write_manifest(manifest_path, manifest)
            raise RuntimeError(
                "formal trial left no valid evidence; no retry allowed") \
                from error
        except TrialSummaryContractFailure as error:
            manifest["status"] = "STOPPED_SUMMARY_CONTRACT_FAILURE"
            manifest["summary_contract_failure"] = str(error)
            _write_manifest(manifest_path, manifest)
            raise RuntimeError(str(error)) from error
        manifest["trials"].append(record)
        manifest["completed_trial_count"] = len(manifest["trials"])
        _write_manifest(manifest_path, manifest)
        if _sha256_file(session_path) != session_sha256:
            manifest["status"] = "STOPPED_SESSION_CHANGED"
            _write_manifest(manifest_path, manifest)
            raise RuntimeError("frozen session changed during campaign")

    final_reasons, _ = auditor.audit_formal_session(session, session_path)
    if final_reasons:
        manifest["status"] = "STOPPED_FROZEN_INPUT_CHANGED"
        manifest["final_audit_reasons"] = final_reasons
        _write_manifest(manifest_path, manifest)
        raise RuntimeError("frozen inputs changed during campaign")
    manifest["status"] = "COMPLETE_FORMAL_CAMPAIGN_UNANALYZED"
    _write_manifest(manifest_path, manifest)
    print("formal campaign complete; analyze the 96 frozen summaries")
    return 0


def main(argv: Sequence[str]) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--readiness-report", type=Path, required=True)
    parser.add_argument("--human-approval", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        return execute(
            args.session, args.readiness_report, args.human_approval,
            args.out_dir)
    except (OSError, RuntimeError, ValueError, yaml.YAMLError,
            json.JSONDecodeError) as error:
        print("formal campaign rejected: {}".format(error), file=sys.stderr)
        return 4


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
