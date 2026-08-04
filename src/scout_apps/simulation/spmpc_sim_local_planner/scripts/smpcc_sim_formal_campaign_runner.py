#!/usr/bin/env python3
"""Serial, fail-closed dispatcher for the frozen SMPCC-SIM formal campaign.

This module deliberately does *not* know how to make a formal matrix, a
randomization table, an asset, a Bslosh release, or a ROS/Gazebo backend.  It
only consumes a validated formal freeze/master pair already accepted by the
formal runtime adapter and chooses the single next attempt in the frozen
40 -> 64 -> conditional-88 order.

The generic one-row lifecycle remains owned by
``smpcc_sim_formal_runtime_adapter.py`` and ``smpcc_sim_toolchain.py``.  This
outer dispatcher adds campaign-level invariants that a one-row API cannot
know on its own:

* one frozen row at a time, in the exact frozen randomization-table order;
* no advance over an unresolved acquisition attempt, no replacement after a
  method/protocol terminal result, and no historical out-of-order ledger;
* S2A only after immutable, hash-bound S1 closure plus extension evidence;
* S2B only after immutable, hash-bound S1/S2A/selectivity/trigger evidence;
* one non-blocking campaign lock per frozen ledger, in addition to the
  generic runner's ROS/Gazebo-master lock; and
* an explicit ``--execute-formal-row`` opt-in.  A normal invocation prepares
  exactly one row and exits non-zero without starting ROS/Gazebo.

There is intentionally no batch loop.  Re-invocation after a completed
attempt is the resume mechanism.  Besides making an operator-visible boundary
between cases, this means a timeout or a terminal method result cannot be
silently hidden by a later row in the same process.
"""

from __future__ import annotations

import argparse
import contextlib
import errno
import fcntl
import importlib.util
import json
import os
import stat
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, Mapping, Optional, Sequence, Tuple


def _load_adapter():
    """Load the sibling formal adapter without requiring a ROS install."""
    existing = sys.modules.get("smpcc_sim_formal_runtime_adapter")
    if existing is not None:
        return existing
    module_path = Path(__file__).with_name("smpcc_sim_formal_runtime_adapter.py")
    spec = importlib.util.spec_from_file_location("smpcc_sim_formal_runtime_adapter", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load SMPCC-SIM formal adapter: {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules["smpcc_sim_formal_runtime_adapter"] = module
    spec.loader.exec_module(module)
    return module


adapter = _load_adapter()
toolchain = adapter.toolchain


CAMPAIGN_ID = "SMPCC-SIM-FORMAL-CAMPAIGN-RUNNER-v1"
CAMPAIGN_SCHEMA_VERSION = "smpcc-sim-formal-campaign-runner-v1"
LOCK_FILENAME = ".smpcc_formal_campaign.lock"
STAGE_ORDER: Tuple[str, ...] = (
    "SIM-S1_CORE",
    "SIM-S2A_SELECTIVITY",
    "SIM-S2B_TRANSFER",
)
STAGE_EVIDENCE_KEYS: Dict[str, Tuple[str, ...]] = {
    "SIM-S2A_SELECTIVITY": ("stage1_closure", "stage1_extension_gate"),
    "SIM-S2B_TRANSFER": (
        "stage1_closure",
        "stage1_extension_gate",
        "stage2a_completion",
        "s2a_selectivity",
        "stage2b_trigger",
    ),
}
WRITE_BITS = stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH


class CampaignError(RuntimeError):
    """A campaign-level NO-GO condition.

    This deliberately has a different type from the adapter error so callers
    can distinguish a frozen campaign scheduling violation from a row ABI
    preflight failure, while both are rendered as a non-zero ``NO_GO`` CLI
    result.
    """


def require(condition: bool, message: str) -> None:
    if not condition:
        raise CampaignError(message)


def _toolchain_call(callback, *args, **kwargs):
    try:
        return callback(*args, **kwargs)
    except toolchain.ContractError as exc:
        raise CampaignError(str(exc)) from exc


def _adapter_call(callback, *args, **kwargs):
    try:
        return callback(*args, **kwargs)
    except adapter.AdapterError as exc:
        raise CampaignError(str(exc)) from exc


def _read_json(path: Path, label: str) -> Mapping[str, Any]:
    value = _toolchain_call(toolchain.read_json, path)
    require(isinstance(value, Mapping), f"{label} must be a JSON object")
    return value


def _absolute_regular_file(value: Path | str, label: str) -> Path:
    path = Path(value)
    require(path.is_absolute(), f"{label} path must be absolute")
    require(path.is_file(), f"{label} file is missing: {path}")
    return path.resolve()


def _require_read_only(path: Path, label: str) -> None:
    """Reject a mutable freeze/evidence artifact instead of trusting intent."""
    try:
        mode = stat.S_IMODE(path.stat().st_mode)
    except OSError as exc:
        raise CampaignError(f"cannot stat {label}: {path}") from exc
    require(mode & WRITE_BITS == 0, f"{label} must be read-only: {path}")


def _sha256_file(path: Path) -> str:
    return str(_toolchain_call(toolchain.sha256_file, path))


def _canonical_hash(value: Any) -> str:
    return str(_toolchain_call(toolchain.canonical_hash, value))


def _is_sha256(value: Any) -> bool:
    return bool(_toolchain_call(toolchain.is_sha256, value))


def _ensure_below(path: Path, root: Path, label: str) -> Path:
    return Path(_toolchain_call(toolchain.ensure_within, path, root, label))


@dataclass(frozen=True)
class CampaignContext:
    """All trusted, immutable inputs for one formal campaign identity."""

    sim_root: Path
    formal_freeze_path: Path
    formal_freeze_file_hash: str
    formal_freeze_hash: str
    formal_freeze: Mapping[str, Any]
    formal_master_path: Path
    formal_master_file_hash: str
    formal_master_hash: str
    formal_master: Mapping[str, Any]
    ledger: Mapping[str, Any]
    ledger_root: Path
    index_path: Path
    schedule: Tuple[Mapping[str, Any], ...]
    records: Tuple[Mapping[str, Any], ...]
    summary: Mapping[str, Any]


@dataclass(frozen=True)
class SerialHistory:
    """Validated prefix state of the immutable append-only dataset index."""

    next_schedule_index: int
    next_attempt_number: int
    terminal_row_ids: frozenset[str]
    stage_terminal_head_hashes: Mapping[str, str]


@dataclass(frozen=True)
class CampaignDispatch:
    """The only row/attempt a campaign invocation may prepare or execute."""

    row: Mapping[str, Any]
    attempt_id: str
    frozen_order_index: int
    history: SerialHistory
    stage_entry_evidence: Optional[Mapping[str, Any]]
    stage_entry_evidence_path: Optional[Path]
    retry_authorization_path: Optional[Path]


def frozen_schedule(master: Mapping[str, Any]) -> Tuple[Mapping[str, Any], ...]:
    """Return the execution order encoded in frozen randomization table rows.

    We do not sort by a caller-provided criterion and do not regenerate a
    Latin square.  The list order inside each frozen table is itself the
    execution schedule.  The protocol only fixes the outer S1 -> S2A -> S2B
    ordering.
    """
    planned_rows = master.get("planned_rows")
    tables = master.get("randomization_tables")
    require(isinstance(planned_rows, list), "formal master lacks planned_rows")
    require(isinstance(tables, Mapping), "formal master lacks randomization_tables")
    rows_by_id: Dict[str, Mapping[str, Any]] = {}
    for row in planned_rows:
        require(isinstance(row, Mapping), "formal master planned row is not an object")
        row_id = row.get("planned_row_id")
        require(isinstance(row_id, str) and row_id, "formal master planned row ID is invalid")
        require(row_id not in rows_by_id, f"formal master duplicates planned row {row_id}")
        rows_by_id[row_id] = row

    result: List[Mapping[str, Any]] = []
    seen: set[str] = set()
    for stage in STAGE_ORDER:
        table = tables.get(stage)
        require(isinstance(table, Mapping), f"formal master lacks frozen table for {stage}")
        assignments = table.get("rows")
        require(isinstance(assignments, list), f"formal frozen table {stage} lacks rows")
        for assignment in assignments:
            require(isinstance(assignment, Mapping), f"formal frozen table {stage} contains a non-object assignment")
            for key in ("path_id", "container_id", "condition_id", "block_id", "order_position"):
                require(key in assignment, f"formal frozen table {stage} assignment lacks {key}")
            row_id = "{}_{}_{}_{}_{}".format(
                stage,
                assignment["path_id"],
                assignment["container_id"],
                assignment["condition_id"],
                assignment["block_id"],
            )
            row = rows_by_id.get(row_id)
            require(row is not None, f"formal frozen schedule references missing master row {row_id}")
            for key in ("stage", "path_id", "container_id", "condition_id", "block_id", "order_position"):
                expected = stage if key == "stage" else assignment[key]
                require(row.get(key) == expected, f"formal master row {row_id} differs from frozen table at {key}")
            require(row_id not in seen, f"formal frozen schedule repeats row {row_id}")
            seen.add(row_id)
            result.append(row)
    require(set(rows_by_id) == seen, "formal master contains planned rows absent from frozen schedule")
    require(len(result) == 88, f"formal frozen schedule must contain 88 rows, got {len(result)}")
    return tuple(result)


def _terminal_record(record: Mapping[str, Any]) -> bool:
    """Match the ledger summary's terminal-state semantics."""
    return (
        record.get("method_success") is True
        or record.get("method_failure") is True
        or record.get("failure_class") in {"METHOD_FAILURE", "PROTOCOL_FAILURE"}
    )


def validate_serial_history(
    schedule: Sequence[Mapping[str, Any]],
    records: Sequence[Mapping[str, Any]],
    *,
    formal_master_hash: str,
    ledger: Mapping[str, Any],
) -> SerialHistory:
    """Require records to be an uninterrupted serial prefix of frozen order.

    ``summarize_ledger`` verifies each formal manifest and retry chain.  This
    additional pass prevents an unsafe external runner from writing valid
    *individual* row records in an out-of-order campaign sequence.
    """
    cursor = 0
    next_attempt_number = 1
    terminal_row_ids: set[str] = set()
    stage_terminal_heads: Dict[str, str] = {}
    stage_last_index = {
        stage: max(index for index, row in enumerate(schedule) if row.get("stage") == stage)
        for stage in STAGE_ORDER
    }
    seen_attempt_ids: set[str] = set()
    for record_number, record in enumerate(records, start=1):
        require(isinstance(record, Mapping), f"formal ledger record {record_number} is not an object")
        require(cursor < len(schedule), "formal dataset ledger has records beyond the frozen 88-row schedule")
        row = schedule[cursor]
        row_id = str(row["planned_row_id"])
        attempt_id = record.get("attempt_id")
        require(isinstance(attempt_id, str) and attempt_id, f"formal ledger record {record_number} has no attempt_id")
        require(attempt_id not in seen_attempt_ids, f"formal dataset ledger repeats attempt {attempt_id}")
        seen_attempt_ids.add(attempt_id)
        require(record.get("formal") is True, f"formal ledger record {attempt_id} is not formal")
        require(record.get("formal_master_hash") == formal_master_hash, f"formal ledger record {attempt_id} belongs to another master")
        require(
            record.get("dataset_ledger_id") == ledger.get("ledger_id")
            and record.get("dataset_ledger_identity_hash") == ledger.get("ledger_identity_hash"),
            f"formal ledger record {attempt_id} belongs to another frozen ledger",
        )
        require(record.get("planned_row_id") == row_id, f"formal ledger is not in frozen row order at record {record_number}: expected {row_id}")
        # The shared append-only index deliberately stores only the fields
        # needed for aggregation; ``block_id`` remains in the immutable row
        # and attempt manifest.  planned_row_id is canonical and therefore
        # already binds the block here.
        for key in ("stage", "condition_id"):
            require(record.get(key) == row.get(key), f"formal ledger record {attempt_id} {key} differs from its frozen row")
        require(_toolchain_call(toolchain.attempt_prefix, attempt_id) == row_id, f"formal ledger record {attempt_id} is not derived from its frozen row")
        number = int(_toolchain_call(toolchain.parse_attempt_number, attempt_id))
        require(number == next_attempt_number, f"formal ledger retry sequence is not contiguous for {row_id}")
        require(not (record.get("method_success") is True and record.get("method_failure") is True), f"formal ledger record {attempt_id} claims both success and failure")
        if _terminal_record(record):
            terminal_row_ids.add(row_id)
            if cursor == stage_last_index[str(row["stage"])]:
                entry_hash = record.get("entry_hash")
                require(_is_sha256(entry_hash), f"formal ledger final {row['stage']} record lacks entry_hash")
                stage_terminal_heads[str(row["stage"])] = str(entry_hash)
            cursor += 1
            next_attempt_number = 1
        else:
            require(
                record.get("failure_class") == toolchain.RETRYABLE_FAILURE_CLASS,
                f"formal ledger record {attempt_id} is neither terminal nor an authorized infrastructure acquisition loss",
            )
            require(record.get("method_success") is False and record.get("method_failure") is False, f"formal acquisition record {attempt_id} has inconsistent method result")
            next_attempt_number += 1
    return SerialHistory(
        next_schedule_index=cursor,
        next_attempt_number=next_attempt_number,
        terminal_row_ids=frozenset(terminal_row_ids),
        stage_terminal_head_hashes=dict(stage_terminal_heads),
    )


def _bound_evidence_file(entry: Mapping[str, Any], label: str) -> Tuple[Path, Mapping[str, Any]]:
    path_value = entry.get("report_path")
    report_hash = entry.get("report_hash")
    require(isinstance(path_value, str) and path_value, f"{label} lacks report_path")
    require(_is_sha256(report_hash), f"{label} lacks report_hash")
    path = _absolute_regular_file(path_value, label)
    _require_read_only(path, label)
    require(_sha256_file(path) == report_hash, f"{label} report hash mismatch")
    return path, _read_json(path, label)


def _require_report_head(
    report: Mapping[str, Any],
    expected_head: str,
    label: str,
) -> None:
    require(
        report.get("dataset_index_head_hash") == expected_head,
        f"{label} is not anchored to the immutable terminal ledger head for its completed stage",
    )


def _load_stage_entry_evidence(
    context: CampaignContext,
    row: Mapping[str, Any],
    history: SerialHistory,
    evidence_path: Optional[Path],
) -> Tuple[Optional[Mapping[str, Any]], Optional[Path]]:
    """Load one read-only evidence envelope and validate its exact stage chain."""
    stage = str(row["stage"])
    if stage == "SIM-S1_CORE":
        require(evidence_path is None, "SIM-S1_CORE cannot consume future stage-entry evidence")
        return None, None

    require(evidence_path is not None, f"{stage} requires immutable frozen stage-entry evidence")
    resolved = _absolute_regular_file(evidence_path, f"{stage} stage-entry evidence")
    _require_read_only(resolved, f"{stage} stage-entry evidence")
    evidence = _read_json(resolved, f"{stage} stage-entry evidence")
    expected_keys = set(STAGE_EVIDENCE_KEYS[stage])
    require(set(evidence) == expected_keys, f"{stage} stage-entry evidence keys must be exactly {sorted(expected_keys)}")

    reports: Dict[str, Mapping[str, Any]] = {}
    for key in STAGE_EVIDENCE_KEYS[stage]:
        entry = evidence.get(key)
        require(isinstance(entry, Mapping), f"{stage} stage-entry evidence {key} is invalid")
        _path, report = _bound_evidence_file(entry, f"{stage} stage-entry evidence {key}")
        reports[key] = report

    s1_head = history.stage_terminal_head_hashes.get("SIM-S1_CORE")
    require(s1_head is not None, "SIM-S2A/SIM-S2B cannot begin before every S1 row is terminally classified")
    _require_report_head(reports["stage1_closure"], s1_head, "S1 closure report")
    _require_report_head(reports["stage1_extension_gate"], s1_head, "S1 extension report")

    if stage == "SIM-S2B_TRANSFER":
        s2a_head = history.stage_terminal_head_hashes.get("SIM-S2A_SELECTIVITY")
        require(s2a_head is not None, "SIM-S2B cannot begin before every S2A row is terminally classified")
        for key in ("stage2a_completion", "s2a_selectivity", "stage2b_trigger"):
            _require_report_head(reports[key], s2a_head, f"{key} report")

    # This is the shared, hash-bound scientific chain validator.  It verifies
    # policy/validator IDs, report types/statuses, linkage hashes and the
    # frozen-ledger identity; the campaign-specific checks above additionally
    # pin reports to the exact stage-boundary index head.
    _toolchain_call(toolchain.validate_stage_entry, context.formal_master, row, evidence)
    return dict(evidence), resolved


def _validate_retry_authorization(
    context: CampaignContext,
    row: Mapping[str, Any],
    attempt_id: str,
    retry_path: Optional[Path],
) -> Optional[Path]:
    number = int(_toolchain_call(toolchain.parse_attempt_number, attempt_id))
    if number == 1:
        require(retry_path is None, "first formal attempt cannot consume retry authorization")
        return None
    require(retry_path is not None, f"formal retry {attempt_id} requires an immutable retry authorization")
    resolved = _absolute_regular_file(retry_path, f"formal retry {attempt_id} authorization")
    _require_read_only(resolved, f"formal retry {attempt_id} authorization")
    classifier = _toolchain_call(toolchain.validate_frozen_retry_classifier, context.formal_freeze.get("retry_classifier"))
    _toolchain_call(
        toolchain.validate_retry_authorization,
        resolved,
        row,
        attempt_id,
        expected_classifier_id=str(classifier["classifier_id"]),
        expected_classifier_rule_hash=str(classifier["classifier_rule_hash"]),
        expected_classifier_manifest_hash=str(classifier["classifier_manifest_hash"]),
        expected_verifier_id=str(classifier["verifier_id"]),
        expected_verifier_hash=str(classifier["verifier_hash"]),
        expected_reason_codes=list(classifier["reason_codes"]),
        expected_max_retries=int(classifier["max_retries_per_row"]),
    )
    return resolved


def select_next_dispatch(
    context: CampaignContext,
    *,
    stage_entry_evidence_path: Optional[Path] = None,
    retry_authorization_path: Optional[Path] = None,
) -> Optional[CampaignDispatch]:
    """Select the sole eligible next formal attempt, or ``None`` when complete."""
    history = validate_serial_history(
        context.schedule,
        context.records,
        formal_master_hash=context.formal_master_hash,
        ledger=context.ledger,
    )
    if history.next_schedule_index == len(context.schedule):
        require(stage_entry_evidence_path is None, "completed formal campaign cannot consume stage-entry evidence")
        require(retry_authorization_path is None, "completed formal campaign cannot consume retry authorization")
        return None

    row = context.schedule[history.next_schedule_index]
    stage_evidence, evidence_path = _load_stage_entry_evidence(context, row, history, stage_entry_evidence_path)
    attempt_id = f"{row['planned_row_id']}_r{history.next_attempt_number:02d}"
    retry_path = _validate_retry_authorization(context, row, attempt_id, retry_authorization_path)
    return CampaignDispatch(
        row=row,
        attempt_id=attempt_id,
        frozen_order_index=history.next_schedule_index,
        history=history,
        stage_entry_evidence=stage_evidence,
        stage_entry_evidence_path=evidence_path,
        retry_authorization_path=retry_path,
    )


def load_campaign_context(
    *,
    formal_freeze_path: Path,
    formal_master_path: Path,
    sim_root: Path = adapter.DEFAULT_SIM_ROOT,
) -> CampaignContext:
    """Read and cross-validate formal campaign inputs without writing anything."""
    try:
        root = Path(sim_root).resolve()
        require(root == Path(adapter.DEFAULT_SIM_ROOT).resolve(), "formal campaign sim_root must be the isolated formal simulation root")
        require(root.is_dir(), f"formal simulation root does not exist: {root}")
        freeze_path = _absolute_regular_file(formal_freeze_path, "formal freeze")
        master_path = _absolute_regular_file(formal_master_path, "formal master")
        _require_read_only(freeze_path, "formal freeze")
        _require_read_only(master_path, "formal master")
        freeze = _read_json(freeze_path, "formal freeze")
        master = _read_json(master_path, "formal master")
        _adapter_call(adapter.reject_development_or_rejected_lineage, freeze, "formal freeze")
        _adapter_call(adapter.reject_development_or_rejected_lineage, master, "formal master")
        freeze_report = _toolchain_call(toolchain.validate_formal_freeze, freeze)
        require(freeze_report.get("status") == "PASS", "FORMAL_SIM_NO_GO: " + "; ".join(freeze_report.get("errors", [])))
        master_report = _toolchain_call(toolchain.validate_master, master, require_formal=True)
        require(master_report.get("status") == "PASS", "formal master validation failed: " + "; ".join(master_report.get("errors", [])))
        freeze_file_hash = _sha256_file(freeze_path)
        master_file_hash = _sha256_file(master_path)
        freeze_hash = _canonical_hash(freeze)
        master_hash = master.get("master_hash")
        require(_is_sha256(master_hash), "formal master has invalid master_hash")
        require(master.get("freeze_hash") == freeze_hash, "formal master freeze hash differs from formal freeze")
        require(master.get("formal_freeze_path") == str(freeze_path), "formal master is bound to another formal freeze path")
        require(master.get("formal_freeze_file_hash") == freeze_file_hash, "formal master formal freeze file hash mismatch")
        ledger = _toolchain_call(toolchain.validate_formal_dataset_ledger, freeze.get("dataset_ledger"), freeze)
        require(
            _canonical_hash(master.get("dataset_ledger")) == _canonical_hash(freeze.get("dataset_ledger")),
            "formal master dataset ledger differs from formal freeze",
        )
        ledger_root = Path(str(ledger["ledger_root"])).resolve()
        _ensure_below(ledger_root, root, "formal campaign ledger root")
        schedule = frozen_schedule(master)
        index_path = ledger_root / "dataset_index.jsonl"
        records = tuple(_toolchain_call(toolchain.load_dataset_index, index_path))
        # This validates every existing formal attempt manifest, its frozen
        # bindings, the append-only chain, retry policy and fixed denominators.
        summary = _toolchain_call(toolchain.summarize_ledger, master, index_path)
        require(summary.get("status") == "PASS", "formal dataset ledger summary did not PASS")
        return CampaignContext(
            sim_root=root,
            formal_freeze_path=freeze_path,
            formal_freeze_file_hash=freeze_file_hash,
            formal_freeze_hash=freeze_hash,
            formal_freeze=freeze,
            formal_master_path=master_path,
            formal_master_file_hash=master_file_hash,
            formal_master_hash=str(master_hash),
            formal_master=master,
            ledger=ledger,
            ledger_root=ledger_root,
            index_path=index_path,
            schedule=schedule,
            records=records,
            summary=summary,
        )
    except CampaignError:
        raise
    except Exception as exc:  # normalize unexpected malformed artifacts to NO-GO
        raise CampaignError(f"formal campaign context validation failed: {exc!r}") from exc


@contextlib.contextmanager
def campaign_lock(context: CampaignContext) -> Iterator[Path]:
    """Take a non-blocking ledger-local lock for one serial dispatch.

    The generic runner also takes its URI-derived master lock.  This lock
    protects the larger read-select-prepare-execute window so two campaign
    processes cannot each select the same next row before either one reaches
    the generic runner's append-only index lock.  It is intentionally an
    empty advisory lock file, not a campaign ledger record.
    """
    context.ledger_root.mkdir(parents=True, exist_ok=True)
    lock_path = (context.ledger_root / LOCK_FILENAME).resolve()
    _ensure_below(lock_path, context.ledger_root, "formal campaign lock")
    try:
        stream = lock_path.open("a+", encoding="utf-8")
    except OSError as exc:
        raise CampaignError(f"cannot open formal campaign lock: {lock_path}") from exc
    try:
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            if exc.errno in (errno.EACCES, errno.EAGAIN):
                raise CampaignError("formal campaign lock is held; another campaign dispatcher may be active") from exc
            raise CampaignError(f"cannot acquire formal campaign lock: {lock_path}") from exc
        yield lock_path
    finally:
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
        finally:
            stream.close()


def _prepared_report(context: CampaignContext, dispatch: CampaignDispatch, preparation: Any) -> Dict[str, Any]:
    adapter_report = _adapter_call(adapter.formal_row_preparation_report, preparation)
    return {
        "schema_version": CAMPAIGN_SCHEMA_VERSION,
        "campaign_id": CAMPAIGN_ID,
        "status": "FORMAL_CAMPAIGN_ROW_PREPARED_NOT_EXECUTED",
        "formal": True,
        "formal_execution_authorized": False,
        "execution_model": "one serial frozen row per explicit invocation",
        "concurrent_ros_gazebo_masters_forbidden": True,
        "formal_freeze_hash": context.formal_freeze_hash,
        "formal_master_hash": context.formal_master_hash,
        "dataset_ledger_id": context.ledger["ledger_id"],
        "dataset_ledger_identity_hash": context.ledger["ledger_identity_hash"],
        "selected_frozen_order_index": dispatch.frozen_order_index,
        "planned_row_id": dispatch.row["planned_row_id"],
        "attempt_id": dispatch.attempt_id,
        "stage": dispatch.row["stage"],
        "stage_entry_evidence_path": str(dispatch.stage_entry_evidence_path) if dispatch.stage_entry_evidence_path else None,
        "retry_authorization_path": str(dispatch.retry_authorization_path) if dispatch.retry_authorization_path else None,
        "ledger_summary": dict(context.summary),
        "adapter_preparation": adapter_report,
        "reason": "explicit --execute-formal-row was not supplied; no ROS/Gazebo process and no formal attempt were started",
    }


def _complete_report(context: CampaignContext) -> Dict[str, Any]:
    return {
        "schema_version": CAMPAIGN_SCHEMA_VERSION,
        "campaign_id": CAMPAIGN_ID,
        "status": "FORMAL_CAMPAIGN_COMPLETE",
        "formal": True,
        "formal_execution_authorized": False,
        "execution_model": "one serial frozen row per explicit invocation",
        "formal_freeze_hash": context.formal_freeze_hash,
        "formal_master_hash": context.formal_master_hash,
        "dataset_ledger_id": context.ledger["ledger_id"],
        "dataset_ledger_identity_hash": context.ledger["ledger_identity_hash"],
        "ledger_summary": dict(context.summary),
        "reason": "all 88 frozen rows have terminal, immutable ledger outcomes; this runner generated no new row or report",
    }


def prepare_campaign_row(
    context: CampaignContext,
    *,
    stage_entry_evidence_path: Optional[Path] = None,
    retry_authorization_path: Optional[Path] = None,
) -> Tuple[Optional[CampaignDispatch], Optional[Any]]:
    """Select and adapter-preflight the next row without starting a process."""
    dispatch = select_next_dispatch(
        context,
        stage_entry_evidence_path=stage_entry_evidence_path,
        retry_authorization_path=retry_authorization_path,
    )
    if dispatch is None:
        return None, None
    preparation = _adapter_call(
        adapter.prepare_formal_row,
        formal_freeze_path=context.formal_freeze_path,
        formal_master_path=context.formal_master_path,
        planned_row_id=str(dispatch.row["planned_row_id"]),
        output_root=context.ledger_root,
        attempt_id=dispatch.attempt_id,
        stage_entry_evidence=dispatch.stage_entry_evidence,
        retry_authorization=None if dispatch.retry_authorization_path is None else str(dispatch.retry_authorization_path),
        sim_root=context.sim_root,
    )
    return dispatch, preparation


def execute_campaign_row(
    *,
    formal_freeze_path: Path,
    formal_master_path: Path,
    stage_entry_evidence_path: Optional[Path] = None,
    retry_authorization_path: Optional[Path] = None,
    sim_root: Path = adapter.DEFAULT_SIM_ROOT,
    authorize_execution: bool = False,
) -> Dict[str, Any]:
    """Optionally execute exactly one selected formal row under both locks."""
    context = load_campaign_context(
        formal_freeze_path=formal_freeze_path,
        formal_master_path=formal_master_path,
        sim_root=sim_root,
    )
    if not authorize_execution:
        dispatch, preparation = prepare_campaign_row(
            context,
            stage_entry_evidence_path=stage_entry_evidence_path,
            retry_authorization_path=retry_authorization_path,
        )
        return _complete_report(context) if dispatch is None else _prepared_report(context, dispatch, preparation)

    # Do not lock or create even the empty lock file until the operator has
    # made the explicit execution choice.  Once locked, reload everything so
    # a stale dry preflight cannot select a row after another dispatcher ran.
    with campaign_lock(context) as lock_path:
        locked_context = load_campaign_context(
            formal_freeze_path=formal_freeze_path,
            formal_master_path=formal_master_path,
            sim_root=sim_root,
        )
        require(
            locked_context.formal_freeze_file_hash == context.formal_freeze_file_hash
            and locked_context.formal_master_file_hash == context.formal_master_file_hash
            and locked_context.formal_master_hash == context.formal_master_hash,
            "formal freeze/master changed between campaign preflight and serial dispatch",
        )
        dispatch, preparation = prepare_campaign_row(
            locked_context,
            stage_entry_evidence_path=stage_entry_evidence_path,
            retry_authorization_path=retry_authorization_path,
        )
        if dispatch is None:
            return _complete_report(locked_context)
        result = _adapter_call(adapter.execute_prepared_formal_row, preparation, authorize_execution=True)
        # The adapter/generic runner should have already appended and validated
        # the immutable record.  Confirm the exact selected attempt appears
        # once before returning control to an operator or another invocation.
        after_records = tuple(_toolchain_call(toolchain.load_dataset_index, locked_context.index_path))
        matching = [item for item in after_records if item.get("attempt_id") == dispatch.attempt_id]
        require(len(matching) == 1, "formal row execution returned without exactly one append-only ledger record")
        return {
            "schema_version": CAMPAIGN_SCHEMA_VERSION,
            "campaign_id": CAMPAIGN_ID,
            "status": "FORMAL_CAMPAIGN_ROW_EXECUTED" if result.get("status") == "PASS" else "FORMAL_CAMPAIGN_ROW_EXECUTED_NONPASS",
            "formal": True,
            "formal_execution_authorized": True,
            "execution_model": "one serial frozen row per explicit invocation",
            "campaign_lock_path": str(lock_path),
            "concurrent_ros_gazebo_masters_forbidden": True,
            "selected_frozen_order_index": dispatch.frozen_order_index,
            "planned_row_id": dispatch.row["planned_row_id"],
            "attempt_id": dispatch.attempt_id,
            "stage": dispatch.row["stage"],
            "runtime_result": result,
            "reason": "one adapter-owned formal lifecycle completed; a subsequent row requires a separate explicit invocation",
        }


def _emit(value: Mapping[str, Any]) -> None:
    print(json.dumps(value, ensure_ascii=False, sort_keys=True))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="serial fail-closed SMPCC-SIM formal campaign dispatcher")
    parser.add_argument("--formal-freeze", type=Path, required=True)
    parser.add_argument("--formal-master", type=Path, required=True)
    parser.add_argument("--stage-entry-evidence", type=Path)
    parser.add_argument("--retry-authorization", type=Path)
    parser.add_argument("--sim-root", type=Path, default=adapter.DEFAULT_SIM_ROOT)
    parser.add_argument(
        "--execute-formal-row",
        action="store_true",
        help="required to start the one selected formal row; without it only adapter-preflight is performed and the command exits non-zero",
    )
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = _parser().parse_args(list(argv) if argv is not None else None)
    try:
        report = execute_campaign_row(
            formal_freeze_path=Path(args.formal_freeze),
            formal_master_path=Path(args.formal_master),
            stage_entry_evidence_path=None if args.stage_entry_evidence is None else Path(args.stage_entry_evidence),
            retry_authorization_path=None if args.retry_authorization is None else Path(args.retry_authorization),
            sim_root=Path(args.sim_root),
            authorize_execution=args.execute_formal_row,
        )
        _emit(report)
        if report.get("status") == "FORMAL_CAMPAIGN_COMPLETE":
            return 0
        if report.get("formal_execution_authorized") is not True:
            return 2
        return 0 if report.get("status") == "FORMAL_CAMPAIGN_ROW_EXECUTED" else 2
    except CampaignError as exc:
        _emit(
            {
                "schema_version": CAMPAIGN_SCHEMA_VERSION,
                "campaign_id": CAMPAIGN_ID,
                "status": "NO_GO",
                "formal": True,
                "formal_execution_authorized": False,
                "error": str(exc),
            }
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
