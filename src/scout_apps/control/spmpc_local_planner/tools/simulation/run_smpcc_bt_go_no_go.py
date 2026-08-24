#!/usr/bin/env python3
"""Run the frozen, one-shot S-MPCC/BT Go/No-Go campaign.

The full campaign is deliberately serial and fail-fast.  It freezes the BT
evidence first, uses one D1 monitor canary, and permits the remaining direct
trials only after the first D1 direct trial passes the candidate contract.
No trial is retried and no existing evidence directory is reused.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import pathlib
import stat
import subprocess
import sys
from datetime import datetime, timezone


FROZEN_BASELINE_HEAD = "b48039cb75ac0ec2f97a7d1fde4caf57c87520f3"
PRIVATE_RUNNER = pathlib.Path(
    "devel/.private/spmpc_local_planner/lib/spmpc_local_planner/"
    "spmpc_phase_rejoin_closed_loop_trial")
SEEDS = (9911, 9912, 9913, 9914, 9915)
EFFECTIVE_CORRECTION_FRACTION_MIN = 0.10
FROZEN_INPUT_SHA256 = {
    "plant": "13bd56b2aa33919fd9d44fa4cab849896fc80e7743146f0123df3d2de05cf10d",
    "path": "8790c5ba5e7d167d2fcbfe2c366116ddffa5101ea3d9684821703461a4ee54fd",
    "artifact": "ff2b4d4b8858a1df2545cc32705dbe5c3542918ca51a761c7e3810166c2721cc",
}
CONDITION_FILES = {
    "bt_nominal": "G_BT_nominal_freeze.yaml",
    "bt_d1": "G_BT_D1_initial_pose.yaml",
    "bt_d2": "G_BT_D2_short_speed_cap.yaml",
    "monitor_d1": "G_SMPCC_BT_monitor_D1_initial_pose.yaml",
    "direct_d1": "G_SMPCC_BT_direct_D1_initial_pose.yaml",
    "direct_d2": "G_SMPCC_BT_direct_D2_short_speed_cap.yaml",
}
TRIAL_CONTRACTS = {
    "bt_nominal": ("bounded_tracking", "NONE"),
    "bt_d1": ("bounded_tracking", "D1_INITIAL_POSE"),
    "bt_d2": ("bounded_tracking", "D2_SHORT_LINEAR_SPEED_CAP"),
    "monitor_d1": ("smpcc_bt_monitor", "D1_INITIAL_POSE"),
    "direct_d1": ("smpcc_bt_direct", "D1_INITIAL_POSE"),
    "direct_d2": ("smpcc_bt_direct", "D2_SHORT_LINEAR_SPEED_CAP"),
}


class CampaignFailure(RuntimeError):
    """An explicit failure that permanently stops the one-shot campaign."""

    def __init__(self, reasons: list[str], record: dict | None = None):
        super().__init__("; ".join(reasons))
        self.reasons = reasons
        self.record = record


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_summary(path: pathlib.Path) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("summary root must be an object")
    return payload


def default_runner(workspace: pathlib.Path) -> pathlib.Path:
    """Return the only runner allowed by the frozen campaign.

    In particular, do not fall back to ``devel/lib`` or choose by mtime.
    """

    return workspace / PRIVATE_RUNNER


def runner_environment(workspace: pathlib.Path) -> dict[str, str]:
    """Construct the frozen loader environment without caller dependence."""

    environment = os.environ.copy()
    environment["LD_LIBRARY_PATH"] = ":".join((
        str(workspace / "devel/.private/spmpc_local_planner/lib"),
        "/home/a/acados/lib",
        "/opt/ros/noetic/lib",
        "/opt/ros/noetic/lib/x86_64-linux-gnu",
    ))
    return environment


def _git_bytes(workspace: pathlib.Path, *arguments: str) -> bytes:
    completed = subprocess.run(
        ["git", *arguments], cwd=workspace, capture_output=True, check=False)
    if completed.returncode != 0:
        stderr = completed.stderr.decode("utf-8", errors="replace").strip()
        raise RuntimeError(
            f"git {' '.join(arguments)} failed: {stderr or completed.returncode}")
    return completed.stdout


def working_diff_digest(
        tracked_diff: bytes,
        untracked_entries: list[tuple[str, str, bytes]]) -> str:
    """Hash a structured tracked diff plus untracked path/mode/content."""

    digest = hashlib.sha256()
    digest.update(b"spmpc-working-diff-v1\0tracked\0")
    digest.update(len(tracked_diff).to_bytes(8, "big"))
    digest.update(tracked_diff)
    for relative_path, git_mode, content in sorted(untracked_entries):
        encoded_path = relative_path.encode("utf-8", errors="surrogateescape")
        encoded_mode = git_mode.encode("ascii")
        digest.update(b"\0untracked\0")
        digest.update(len(encoded_path).to_bytes(8, "big"))
        digest.update(encoded_path)
        digest.update(len(encoded_mode).to_bytes(8, "big"))
        digest.update(encoded_mode)
        digest.update(len(content).to_bytes(8, "big"))
        digest.update(content)
    return digest.hexdigest()


def working_diff_evidence(
        workspace: pathlib.Path, package: pathlib.Path) -> dict:
    """Freeze all package changes, excluding generated ``Testing/`` trees."""

    scope = package.relative_to(workspace).as_posix()
    testing_exclusion = f":(exclude,glob){scope}/**/Testing/**"
    tracked_diff = _git_bytes(
        workspace, "diff", "--binary", "--full-index", "--no-ext-diff",
        "--no-renames", FROZEN_BASELINE_HEAD, "--", scope,
        testing_exclusion)
    untracked_output = _git_bytes(
        workspace, "ls-files", "--others", "--exclude-standard", "-z",
        "--", scope)
    untracked_paths = sorted(
        item.decode("utf-8", errors="surrogateescape")
        for item in untracked_output.split(b"\0") if item)
    entries: list[tuple[str, str, bytes]] = []
    included_paths: list[str] = []
    for relative_path in untracked_paths:
        if "Testing" in pathlib.PurePosixPath(relative_path).parts:
            continue
        path = workspace / relative_path
        file_stat = path.lstat()
        if stat.S_ISLNK(file_stat.st_mode):
            git_mode = "120000"
            content = os.readlink(path).encode(
                "utf-8", errors="surrogateescape")
        elif stat.S_ISREG(file_stat.st_mode):
            git_mode = "100755" if file_stat.st_mode & 0o111 else "100644"
            content = path.read_bytes()
        else:
            raise RuntimeError(
                f"unsupported untracked evidence file type: {relative_path}")
        entries.append((relative_path, git_mode, content))
        included_paths.append(relative_path)
    return {
        "sha256": working_diff_digest(tracked_diff, entries),
        "algorithm": "spmpc-working-diff-v1",
        "scope": scope,
        "testing_directories_excluded": True,
        "untracked_paths": included_paths,
    }


def source_freeze_evidence(
        workspace: pathlib.Path, package: pathlib.Path) -> dict:
    observed_head = _git_bytes(
        workspace, "rev-parse", "HEAD").decode("ascii").strip()
    if observed_head != FROZEN_BASELINE_HEAD:
        raise SystemExit(
            "baseline HEAD mismatch: expected "
            f"{FROZEN_BASELINE_HEAD}, observed {observed_head}")
    return {
        "baseline_head": FROZEN_BASELINE_HEAD,
        "observed_head": observed_head,
        "working_diff": working_diff_evidence(workspace, package),
    }


def campaign_steps(phase: str) -> list[tuple[str, int]]:
    bt_freeze = [
        (key, seed)
        for key in ("bt_nominal", "bt_d1", "bt_d2")
        for seed in SEEDS
    ]
    monitor_canary = [("monitor_d1", SEEDS[0])]
    direct = (
        [("direct_d1", SEEDS[0])] +
        [("direct_d1", seed) for seed in SEEDS[1:]] +
        [("direct_d2", seed) for seed in SEEDS]
    )
    if phase == "bt-freeze":
        return bt_freeze
    if phase == "monitor":
        return monitor_canary
    if phase in ("direct", "all"):
        # ``direct`` remains accepted, but cannot bypass either prerequisite.
        return bt_freeze + monitor_canary + direct
    raise ValueError(f"unknown campaign phase: {phase}")


def _finite_number(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    numeric = float(value)
    return numeric if math.isfinite(numeric) else None


def trial_contract_reasons(summary: dict, key: str, seed: int) -> list[str]:
    """Validate trial identity and unambiguous completion evidence."""

    label = f"{key}/seed{seed}"
    expected_mode, expected_disturbance = TRIAL_CONTRACTS[key]
    reasons: list[str] = []
    if summary.get("mode") != expected_mode:
        reasons.append(f"{label}:MODE_CONTRACT")
    if summary.get("seed") != seed:
        reasons.append(f"{label}:SEED_CONTRACT")
    disturbance = summary.get("go_no_go_disturbance")
    if (not isinstance(disturbance, dict) or
            disturbance.get("id") != expected_disturbance):
        reasons.append(f"{label}:DISTURBANCE_CONTRACT")
    trial = summary.get("trial")
    if (not isinstance(trial, dict) or
            trial.get("sequence_completed") is not True or
            trial.get("task_success") is not True or
            trial.get("runtime_error") != ""):
        reasons.append(f"{label}:TRIAL_DID_NOT_COMPLETE")
    return reasons


def candidate_contract_reasons(
        summary: dict, key: str, seed: int) -> list[str]:
    """Validate monitor/direct candidate, reference, padding and correction."""

    reasons = trial_contract_reasons(summary, key, seed)
    label = f"{key}/seed{seed}"
    audit = summary.get("smpcc_bt_go_no_go_audit")
    if not isinstance(audit, dict):
        reasons.append(f"{label}:CANDIDATE_AUDIT_CONTRACT")
        return reasons

    controller_audit = summary.get("controller_audit")
    publication_contract = (
        summary.get("final_command_transaction") is True and
        summary.get("command_history_source") == "final_published_command" and
        summary.get("dual_channel_execution_model") is True and
        isinstance(controller_audit, dict) and
        _finite_number(controller_audit.get("publications")) is not None and
        _finite_number(controller_audit.get("publications")) > 0 and
        controller_audit.get("publication_failures") == 0 and
        controller_audit.get("receipt_inconsistent_cycles") == 0 and
        controller_audit.get("history_not_committed_cycles") == 0 and
        controller_audit.get("controlled_stops") == 0)
    if not publication_contract:
        reasons.append(f"{label}:PUBLICATION_SEMANTICS_CONTRACT")

    attempts = _finite_number(audit.get("candidate_attempts"))
    failures = _finite_number(audit.get("candidate_failures"))
    reference_cycles = _finite_number(
        audit.get("bt_timed_reference_cycles"))
    padded_stages = _finite_number(
        audit.get("bt_reference_padded_stage_count"))
    eligible_cycles = _finite_number(audit.get("correction_eligible_cycles"))
    effective_cycles = _finite_number(audit.get("effective_correction_cycles"))
    correction_fraction = _finite_number(
        audit.get("effective_correction_fraction"))
    contract_fraction = _finite_number(
        audit.get("minimum_effective_correction_fraction"))

    if attempts is None or attempts <= 0 or not attempts.is_integer():
        reasons.append(f"{label}:NO_CANDIDATE_ATTEMPTS")
    if failures is None or failures != 0:
        reasons.append(f"{label}:CANDIDATE_FAILURE")
    if (attempts is None or reference_cycles is None or
            not reference_cycles.is_integer() or reference_cycles != attempts):
        reasons.append(f"{label}:REFERENCE_CYCLE_MISMATCH")
    if (padded_stages is None or padded_stages <= 0 or
            not padded_stages.is_integer()):
        reasons.append(f"{label}:TERMINAL_PADDING_CONTRACT")
    if (contract_fraction is None or
            contract_fraction != EFFECTIVE_CORRECTION_FRACTION_MIN):
        reasons.append(f"{label}:CORRECTION_THRESHOLD_CONTRACT")
    if (eligible_cycles is None or eligible_cycles <= 0 or
            not eligible_cycles.is_integer() or
            effective_cycles is None or effective_cycles <= 0 or
            not effective_cycles.is_integer() or
            correction_fraction is None or
            correction_fraction < EFFECTIVE_CORRECTION_FRACTION_MIN):
        reasons.append(f"{label}:NO_EFFECTIVE_CORRECTION")
    return reasons


def main(argv: list[str] | None = None) -> int:
    workspace = pathlib.Path(__file__).resolve().parents[6]
    package = workspace / "src/scout_apps/control/spmpc_local_planner"
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--phase", choices=("bt-freeze", "monitor", "direct", "all"),
        default="all")
    parser.add_argument("--out-dir", required=True, type=pathlib.Path)
    parser.add_argument(
        "--runner", type=pathlib.Path,
        default=default_runner(workspace))
    parser.add_argument(
        "--plant", type=pathlib.Path,
        default=pathlib.Path(
            "/data/a/spmpc_exec_identification/"
            "phase_rejoin_c2_bt_c3_c4_same_asset_dev_20260824.i1bUYq/"
            "inputs/plant_config.yaml"))
    parser.add_argument(
        "--path", type=pathlib.Path,
        default=pathlib.Path(
            "/data/a/spmpc_exec_identification/"
            "phase_rejoin_c2_bt_c3_c4_same_asset_dev_20260824.i1bUYq/"
            "inputs/P2_s_curve.json"))
    parser.add_argument(
        "--artifact", type=pathlib.Path,
        default=pathlib.Path(
            "/data/a/spmpc_exec_identification/"
            "tail_commit_15d_recovery_full_fTRij6/v3/"
            "phase_rejoin_p2_smooth43_tail_commit_15d_v3.csv"))
    args = parser.parse_args(argv)

    if args.out_dir.exists():
        raise SystemExit(f"refusing to reuse output directory: {args.out_dir}")
    required_runner = default_runner(workspace).resolve()
    if args.runner.resolve() != required_runner:
        raise SystemExit(
            "runner must be the frozen private package executable: "
            f"{required_runner}")

    campaign_script = pathlib.Path(__file__).resolve()
    analyzer_script = (
        package / "tools/simulation/analyze_smpcc_bt_go_no_go.py").resolve()
    assets = {
        "runner_executable": required_runner,
        "plant": args.plant.resolve(),
        "path": args.path.resolve(),
        "artifact": args.artifact.resolve(),
        "campaign_script": campaign_script,
        "analyzer_script": analyzer_script,
    }
    conditions_dir = package / "config/simulation/conditions"
    for key, filename in CONDITION_FILES.items():
        assets[f"condition_{key}"] = (conditions_dir / filename).resolve()
    missing = [str(path) for path in assets.values() if not path.is_file()]
    if missing:
        raise SystemExit("missing campaign assets: " + ", ".join(missing))
    input_hashes = {
        name: sha256(assets[name]) for name in FROZEN_INPUT_SHA256
    }
    mismatched_inputs = [
        f"{name}: expected {FROZEN_INPUT_SHA256[name]}, "
        f"observed {input_hashes[name]}"
        for name in FROZEN_INPUT_SHA256
        if input_hashes[name] != FROZEN_INPUT_SHA256[name]
    ]
    if mismatched_inputs:
        raise SystemExit(
            "frozen campaign input hash mismatch: " +
            "; ".join(mismatched_inputs))

    source_freeze = source_freeze_evidence(workspace, package)
    trial_environment = runner_environment(workspace)
    args.out_dir.mkdir(parents=True)
    trials_dir = args.out_dir / "trials"
    trials_dir.mkdir()

    manifest = {
        "schema": "spmpc_smpcc_bt_go_no_go_campaign_v2",
        "status": "RUNNING",
        "development_only": True,
        "one_shot_no_retry": True,
        "failure_stops_immediately": True,
        "phase": args.phase,
        "seeds": list(SEEDS),
        "started_at": datetime.now(timezone.utc).isoformat(),
        "source_freeze": source_freeze,
        "frozen_input_sha256": dict(FROZEN_INPUT_SHA256),
        "runner_environment": {
            "LD_LIBRARY_PATH": trial_environment["LD_LIBRARY_PATH"],
        },
        "assets": {
            name: {"path": str(path), "sha256": sha256(path)}
            for name, path in assets.items()
        },
        "planned_trials": [
            {"key": key, "seed": seed}
            for key, seed in campaign_steps(args.phase)
        ],
        "trials": [],
    }
    manifest_path = args.out_dir / "campaign_manifest.json"
    decision_path = args.out_dir / "route_decision.json"

    def write_manifest() -> None:
        temporary = manifest_path.with_suffix(".json.partial")
        temporary.write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8")
        temporary.replace(manifest_path)

    def write_no_go(failure: CampaignFailure) -> None:
        decision = {
            "schema": "spmpc_smpcc_bt_go_no_go_early_decision_v1",
            "development_only": True,
            "one_shot_failure_stops_route_a": True,
            "decision": "NO_GO_ROUTE_B",
            "reasons": failure.reasons,
            "failed_trial": failure.record,
            "created_at": datetime.now(timezone.utc).isoformat(),
        }
        # Exclusive creation guarantees one decision and forbids replacement.
        with decision_path.open("x", encoding="utf-8") as stream:
            json.dump(decision, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        manifest["status"] = "FAILED_STOPPED"
        manifest["failure_reasons"] = failure.reasons
        manifest["route_decision_artifact"] = str(decision_path)
        manifest["completed_at"] = datetime.now(timezone.utc).isoformat()
        write_manifest()

    write_manifest()

    def run_trial(key: str, seed: int) -> dict:
        condition = assets[f"condition_{key}"]
        trial_dir = trials_dir / f"{key}_seed{seed}"
        trial_dir.mkdir()
        cycle_csv = trial_dir / "cycles.csv"
        summary_json = trial_dir / "summary.json"
        command = [
            str(assets["runner_executable"]),
            "--plant", str(assets["plant"]),
            "--path", str(assets["path"]),
            "--artifact", str(assets["artifact"]),
            "--condition", str(condition),
            "--seed", str(seed),
            "--cycle-csv", str(cycle_csv),
            "--summary-json", str(summary_json),
        ]
        try:
            completed = subprocess.run(
                command, text=True, capture_output=True, check=False,
                env=trial_environment)
        except OSError as exception:
            record = {
                "key": key,
                "seed": seed,
                "returncode": None,
                "stdout": "",
                "stderr": str(exception),
                "cycle_csv": str(cycle_csv),
                "summary_json": str(summary_json),
            }
            manifest["trials"].append(record)
            write_manifest()
            raise CampaignFailure(
                [f"{key}/seed{seed}:RUNNER_EXECUTION_ERROR"],
                record) from exception
        record = {
            "key": key,
            "seed": seed,
            "returncode": completed.returncode,
            "stdout": completed.stdout,
            "stderr": completed.stderr,
            "cycle_csv": str(cycle_csv),
            "summary_json": str(summary_json),
        }
        manifest["trials"].append(record)
        write_manifest()
        execution_reasons: list[str] = []
        label = f"{key}/seed{seed}"
        if completed.returncode != 0:
            execution_reasons.append(
                f"{label}:RUNNER_EXIT_{completed.returncode}")
        if not summary_json.is_file():
            execution_reasons.append(f"{label}:SUMMARY_MISSING")
        if execution_reasons:
            raise CampaignFailure(execution_reasons, record)
        try:
            summary = read_summary(summary_json)
        except (OSError, json.JSONDecodeError, ValueError) as exception:
            raise CampaignFailure(
                [f"{label}:SUMMARY_INVALID:{type(exception).__name__}"],
                record) from exception
        reasons = (
            candidate_contract_reasons(summary, key, seed)
            if key.startswith(("monitor_", "direct_"))
            else trial_contract_reasons(summary, key, seed)
        )
        record["contract_gate"] = {
            "passed": not reasons,
            "reasons": reasons,
        }
        write_manifest()
        if reasons:
            raise CampaignFailure(reasons, record)
        return summary

    try:
        for key, seed in campaign_steps(args.phase):
            run_trial(key, seed)
            if (key, seed) == ("monitor_d1", SEEDS[0]):
                manifest["monitor_canary_gate"] = {
                    "passed": True, "key": key, "seed": seed}
                write_manifest()
            if (key, seed) == ("direct_d1", SEEDS[0]):
                manifest["direct_canary_gate"] = {
                    "passed": True, "key": key, "seed": seed}
                write_manifest()
    except CampaignFailure as failure:
        write_no_go(failure)
        print(str(failure), file=sys.stderr)
        print("NO_GO_ROUTE_B")
        return 4

    manifest["status"] = "COMPLETE_AWAITING_ANALYZER"
    manifest["completed_at"] = datetime.now(timezone.utc).isoformat()
    write_manifest()
    print(manifest_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
