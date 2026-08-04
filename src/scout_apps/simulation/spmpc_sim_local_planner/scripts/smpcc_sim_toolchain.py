#!/usr/bin/env python3
"""Fail-closed tooling for the SMPCC-SIM 40 -> 64 -> conditional 88 matrix.

This module deliberately has no dependency on a currently running ROS/Gazebo
session.  It owns the protocol-level identities, freeze gates, seed bundles,
append-only ledgers, and a small lifecycle adapter for a *single* fresh case.
The adapter is intentionally conservative: it refuses formal rows until a
complete formal freeze is supplied, and it never uses broad process killing.

There is no formal, fidelity-validated independent liquid plant in the current
repository.  Consequently the default development H0 run is labelled
``MODEL_PROXY_MECHANISM_ONLY`` and keeps ``/slosh/height`` and
``/sim_spmpc/slosh_height`` in H_proxy/H_modal fields only.  An explicit H0-only
development surrogate may record an ``UNVALIDATED`` H_plant channel, but it is
also non-formal and never physical-primary evidence.
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import fcntl
import hashlib
import json
import math
import os
import random
import re
import shlex
import signal
import socket
import stat
import subprocess
import sys
import time
import uuid
import xmlrpc.client
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, List, Mapping, Optional, Sequence, Tuple


SCHEMA_VERSION = "smpcc-sim-toolchain-v1"
DEFAULT_SIM_ROOT = Path("/data/a/scout_sim_replacement")
FORMAL_PROTOCOL_ID = "SMPCC-SIM-40-64-88-v0.1"
# A SIM-ONLY controller release is a deliberately different protocol.  It
# may be useful for a self-contained simulation study, but it must never be
# smuggled into the real-aligned 40/64/88 freeze merely by re-labelling its
# condition as Bslosh.  Keep this marker check in the legacy formal gate as a
# defence in depth boundary in addition to the SIM-ONLY release validator.
SIM_ONLY_PROTOCOL_PREFIX = "SMPCC-SIM-ONLY-"
SIM_ONLY_VARIANT_IDS = frozenset(("SIM_Bslosh_R1",))
# The simulation protocol deliberately has its own ID, but it cannot choose
# control parameters independently of the eventual real v2.0 release.  A
# formal freeze must therefore carry a separate, hash-bound alignment receipt
# that proves the five normalized effective configurations agree with the
# real freeze on every control-relevant field.
REAL_FORMAL_PROTOCOL_ID = "SMPCC-REAL-40-64-88-v2.0"
FORMAL_REAL_PARAMETER_ALIGNMENT_DOCUMENT_TYPE = "SMPCC_SIM_REAL_PARAMETER_ALIGNMENT"
FORMAL_REAL_PARAMETER_ALIGNMENT_SCHEMA_VERSION = "smpcc-sim-real-parameter-alignment-v1"
PROXY_EVIDENCE_CLASS = "MODEL_PROXY_MECHANISM_ONLY"
FIXTURE_EVIDENCE_CLASS = "FIXTURE_NOT_FORMAL"
DEVELOPMENT_EVIDENCE_CLASS = "DEVELOPMENT_SMOKE_NOT_FORMAL"
REQUIRED_EFFECTIVE_CONFIG_FIELDS = (
    "w_control",
    "w_smooth",
    "w_alpha",
    "w_du_a",
    "w_du_vs",
    "w_slosh",
    "v_ref",
    "observer",
    "delay",
)
SEED_STREAMS = (
    "initial_robot_pose",
    "initial_liquid_state",
    "actuator_disturbance",
    "sensor_noise",
    "environment_contact_noise",
    "simulator_rng",
)
FORBIDDEN_CONTROL_PREFIXES = (
    "/sim_truth/",
    "/slosh/",
    "/benchmark/slosh_monitor/",
)
FORBIDDEN_COMMAND_TOKENS = ("killall", "pkill", "kill -9 -1", "killall5")
REQUIRED_FIREWALL_CHECKPOINTS = frozenset(("ready", "pre_motion", "postflight"))
# This is intentionally separate from the formal firewall contract.  It is a
# narrow H0 development audit that becomes mandatory only for the explicit
# UNVALIDATED direct liquid-plant opt-in.  It never confers physical-primary
# eligibility and must not relax the frozen formal firewall requirements.
DEVELOPMENT_H0_FIREWALL_CHECKPOINTS = ("ready", "pre_motion", "postflight")
DEVELOPMENT_H0_FIREWALL_ROLE_NAMES = frozenset(
    ("controller", "planner", "tracker", "cmd_gate")
)
DEVELOPMENT_H0_FIREWALL_FORBIDDEN_PREFIXES = ("/sim_truth/",)
DEVELOPMENT_H0_FIREWALL_RECORD_TYPE = "SMPCC_SIM_DEV_H0_CONTROLLER_FIREWALL_SNAPSHOT"
# A formal physical-primary plant may only enter a freeze through the
# separately reviewed evidence intake.  These are intentionally protocol
# constants rather than caller-supplied labels: copying a handful of PASS
# fields into a JSON report must not turn a development surrogate into formal
# evidence.
FORMAL_LIQUID_PLANT_INTAKE_TOOL_ID = "SMPCC-SIM-LIQUID-PLANT-FORMAL-EVIDENCE-INTAKE-v1"
FORMAL_LIQUID_PLANT_BINDING_SCHEMA_VERSION = "smpcc-sim-formal-liquid-plant-toolchain-binding-v1"
FORMAL_LIQUID_PLANT_BINDING_TYPE = "SMPCC_SIM_FORMAL_LIQUID_PLANT_TOOLCHAIN_BINDING"
FORMAL_LIQUID_PLANT_CAPABILITY_SCHEMA_VERSION = "smpcc-sim-independent-liquid-plant-capability-v1"
FORMAL_LIQUID_PLANT_FIDELITY_SCHEMA_VERSION = "smpcc-sim-formal-liquid-plant-fidelity-validation-v1"
FORMAL_LIQUID_PLANT_RELEASE_SCHEMA_VERSION = "smpcc-sim-formal-liquid-plant-release-v1"
FORMAL_LIQUID_PLANT_APPROVAL_SCHEMA_VERSION = "smpcc-sim-formal-liquid-plant-approval-v1"
FORMAL_LIQUID_PLANT_ISOLATION_SCHEMA_VERSION = "smpcc-sim-controller-plant-isolation-evidence-v1"
FORMAL_LIQUID_PLANT_REFERENCE_SCHEMA_VERSION = "smpcc-real-liquid-reference-evidence-v1"
FORMAL_LIQUID_PLANT_SIGNAL_SCHEMA_VERSION = "smpcc-sim-formal-liquid-plant-signal-evidence-v1"
FORMAL_LIQUID_PLANT_REFERENCE_KINDS = frozenset(("REAL_RGB_LIQUID_HEIGHT", "REAL_LIQUID_HEIGHT_SENSOR"))
FORMAL_LIQUID_PLANT_PROVENANCE_FIELDS = (
    "formal_intake_tool_id",
    "formal_intake_request_path",
    "formal_intake_request_hash",
    "formal_release_manifest_path",
    "formal_release_manifest_hash",
    "external_approval_path",
    "external_approval_hash",
    "external_approval_id",
    "external_approval_authority",
    "cryptographic_trust_anchor",
    "external_approval_authentication_status",
    "controller_isolation_evidence_path",
    "controller_isolation_evidence_hash",
    "fidelity_verifier_source_path",
    "fidelity_verifier_source_hash",
    "formal_reference_evidence",
    "formal_reference_evidence_set_hash",
    "formal_plant_signal_evidence",
    "formal_plant_signal_evidence_set_hash",
)
RETRYABLE_FAILURE_CLASS = "INFRASTRUCTURE_ACQUISITION"
NONRETRYABLE_FAILURE_CLASSES = frozenset(("METHOD_FAILURE", "PROTOCOL_FAILURE"))
DEFAULT_RECORDER_READY_TIMEOUT_SEC = 15.0
# W5_S10 was rejected by bcedcf7.  Match the common spelling variants rather
# than relying on one filename-style spelling, because runtime command/spec
# strings are an otherwise easy way to revive it under a new label.
FORBIDDEN_W5_PATTERN = re.compile(r"(?<![a-z0-9])w[ _-]*5(?:[ _-]*s[ _-]*10)?(?![a-z0-9])", re.IGNORECASE)
DEVELOPMENT_RUNTIME_PATH_IDS = frozenset(("H0", "H0b", "H0s"))

# A Stage-I closure only says that its ledger is closed.  It cannot by itself
# authorize Stage II: the scientific extension gate must be independently
# frozen, hash-bound and PASS.
STAGE_ENTRY_POLICY_DOCUMENT_TYPE = "SMPCC_SIM_STAGE_ENTRY_POLICY"
STAGE1_EXTENSION_REPORT_TYPE = "SIM_S1_EXTENSION_GATE"
STAGE2A_SELECTIVITY_REPORT_TYPE = "SIM_S2A_SELECTIVITY_ANALYSIS"
STAGE2B_TRIGGER_REPORT_TYPE = "SIM_S2B_TRIGGER"
REQUIRED_S1_EXTENSION_GATES = frozenset(
    (
        "plant_recording_qc",
        "bslosh_bsmooth_primary",
        "smoothmatch_completion",
        "fixedprofile_novelty",
        "success_tracking_runtime",
        "leave_one_block_out",
        "trajectory_replay_execution",
    )
)

# These are the Stage-I comparisons explicitly named by the frozen matrix.
# Stage II is intentionally registry-defined: its physical primary comparator
# is pre-registered before S1, rather than inferred by this script.
REQUIRED_S1_CONTRAST_PAIRS = frozenset(
    (
        frozenset(("Bslosh", "Bsmooth")),
        frozenset(("Bslosh", "FixedProfile")),
        frozenset(("Bslosh", "B0")),
        frozenset(("Bsmooth", "B0")),
        frozenset(("SmoothMatch", "Bsmooth")),
    )
)
FORMAL_RUNTIME_COMMAND_FIELDS = (
    "launch_command",
    "ready_command",
    "recorder_command",
    "motion_command",
    "goal_probe_command",
    "runtime_ack_command",
    "motion_release_ack_command",
    "motion_stop_command",
)
FORMAL_RUNTIME_FORBIDDEN_BACKEND_BASENAMES = frozenset(
    (
        "smpcc_sim_h0_runtime_adapter.py",
        "run_strict_fresh_fair_comparison_n3.sh",
        "run_strict_fresh_profile_baselines_n1.sh",
        "run_fixed_path_profile_baseline_suite.sh",
        "run_fixed_path_paper_matrix.sh",
        "run_proxy_spmpc_mainline_smoke.sh",
        "fixed_global_path_runner.py",
    )
)
# A source-separated formal backend may run only simulation-owned delegates.
# Do not treat a hash as authority to execute a real-stack program: an
# otherwise hash-bound absolute path or ROS package selector below the physical
# controller/experiment/liquid-model tree remains a hard NO-GO.
REAL_STACK_RUNTIME_COMPONENTS = frozenset(
    ("spmpc_local_planner", "spmpc_experiments", "slosh_models")
)
REAL_STACK_RUNTIME_REFERENCE_RE = re.compile(
    r"(?<![a-z0-9_])(?:spmpc_local_planner|spmpc_experiments|slosh_models)(?![a-z0-9_])",
    re.IGNORECASE,
)
REAL_STACK_RUNTIME_LIBRARY_BASENAMES = frozenset(
    ("libspmpc_local_planner.so", "libslosh_models.so")
)

STAGES: Dict[str, Dict[str, Any]] = {
    "SIM-S1_CORE": {
        "path_id": "H1",
        "container_id": "C1",
        "conditions": ("B0", "Bsmooth", "SmoothMatch", "FixedProfile", "Bslosh"),
        "stage_alias": "S1",
    },
    "SIM-S2A_SELECTIVITY": {
        "path_id": "L1",
        "container_id": "C1",
        "conditions": ("Bsmooth", "FixedProfile", "Bslosh"),
        "stage_alias": "S2A",
    },
    "SIM-S2B_TRANSFER": {
        "path_id": "H1",
        "container_id": "C2",
        "conditions": ("Bsmooth", "FixedProfile", "Bslosh"),
        "stage_alias": "S2B",
    },
}

CONDITION_BACKENDS = {
    "B0": "online_mpcc",
    "Bsmooth": "online_mpcc",
    "SmoothMatch": "online_mpcc",
    "FixedProfile": "fixed_profile_tracker",
    "Bslosh": "online_smpcc",
}


class ContractError(RuntimeError):
    """Raised when a protocol assertion is not satisfied."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def canonical_hash(value: Any) -> str:
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def is_sha256(value: Any) -> bool:
    return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value) is not None


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ContractError(message)


def require_source_separated_r8_execution(
    freeze: Mapping[str, Any], master: Mapping[str, Any]
) -> None:
    """Prevent the generic runner from bypassing the R8 adapter boundary."""
    import importlib.util

    module_path = Path(__file__).with_name("smpcc_sim_source_separation.py")
    spec = importlib.util.spec_from_file_location(
        "smpcc_sim_source_separation_toolchain", module_path)
    require(spec is not None and spec.loader is not None,
            "SOURCE_SEPARATED_R8_REQUIRED: source-separation gate is unavailable")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    try:
        module.require_execution_identity(freeze, master)
    except module.SourceSeparationError as exc:
        raise ContractError(str(exc)) from exc


def require_mapping(value: Any, name: str) -> Mapping[str, Any]:
    require(isinstance(value, Mapping), f"{name} must be an object")
    return value


def read_json(path: Path) -> Any:
    try:
        with path.open(encoding="utf-8") as stream:
            return json.load(stream)
    except FileNotFoundError as exc:
        raise ContractError(f"missing JSON file: {path}") from exc
    except json.JSONDecodeError as exc:
        raise ContractError(f"invalid JSON file {path}: {exc}") from exc


def write_json_new(path: Path, value: Any) -> None:
    """Create a JSON file once; silently overwriting evidence is forbidden."""
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    try:
        with path.open("x", encoding="utf-8") as stream:
            stream.write(payload)
    except FileExistsError as exc:
        raise ContractError(f"append-only evidence already exists: {path}") from exc


def write_text_new(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with path.open("x", encoding="utf-8") as stream:
            stream.write(text)
    except FileExistsError as exc:
        raise ContractError(f"append-only evidence already exists: {path}") from exc


def ensure_within(path: Path, root: Path, label: str) -> Path:
    resolved = path.resolve()
    root_resolved = root.resolve()
    try:
        resolved.relative_to(root_resolved)
    except ValueError as exc:
        raise ContractError(f"{label} must be below simulation root {root_resolved}: {resolved}") from exc
    return resolved


def derive_seed(seed_text: str, *parts: str) -> int:
    raw = "\x1f".join((seed_text,) + tuple(parts)).encode("utf-8")
    return int(hashlib.sha256(raw).hexdigest()[:16], 16)


def stable_trace(seed: int, stream: str) -> Dict[str, Any]:
    """A time-indexed trace descriptor avoids callback-count RNG consumption."""
    generator = random.Random(seed)
    samples = []
    for index in range(0, 31):
        # The values are protocol artifacts, not a claimed physical disturbance law.
        samples.append({"time_index_ms": index * 2000, "unit_uniform": round(generator.random(), 12)})
    payload = {"stream": stream, "time_indexed": True, "samples": samples}
    return dict(payload, trace_hash=canonical_hash(payload), sample_count=len(samples))


def make_seed_bundle(seed_text: str, stage: str, block: int) -> Dict[str, Any]:
    require(isinstance(stage, str) and stage, "seed bundle stage/identity is required")
    require(block > 0, "seed bundle block must be positive")
    sub_seeds = {name: derive_seed(seed_text, stage, f"b{block:02d}", name) for name in SEED_STREAMS}
    traces = {name: stable_trace(sub_seeds[name], name) for name in SEED_STREAMS}
    core = {
        "stage": stage,
        "block_id": f"b{block:02d}",
        "sub_seeds": sub_seeds,
        "traces": traces,
        "independent_sub_seeds": True,
        "time_indexed_traces": True,
    }
    return dict(core, seed_bundle_id=f"SEED_{stage}_b{block:02d}", seed_bundle_hash=canonical_hash(core))


def validate_seed_bundle(bundle: Any) -> Dict[str, Any]:
    bundle = require_mapping(bundle, "seed bundle")
    require(bundle.get("independent_sub_seeds") is True, "seed bundle must declare independent sub-seeds")
    require(bundle.get("time_indexed_traces") is True, "seed bundle must declare time-indexed traces")
    sub_seeds = require_mapping(bundle.get("sub_seeds"), "seed bundle sub_seeds")
    traces = require_mapping(bundle.get("traces"), "seed bundle traces")
    require(set(sub_seeds) == set(SEED_STREAMS), "seed bundle must contain every required sub-seed")
    require(set(traces) == set(SEED_STREAMS), "seed bundle must contain every required time-indexed trace")
    values: List[int] = []
    for stream in SEED_STREAMS:
        seed = sub_seeds.get(stream)
        require(isinstance(seed, int) and seed >= 0, f"seed bundle {stream} seed is invalid")
        values.append(seed)
        trace = require_mapping(traces.get(stream), f"seed trace {stream}")
        require(trace.get("time_indexed") is True, f"seed trace {stream} is not time-indexed")
        expected = stable_trace(seed, stream)
        require(trace.get("trace_hash") == expected["trace_hash"], f"seed trace {stream} hash mismatch")
        require(trace.get("samples") == expected["samples"], f"seed trace {stream} samples mismatch")
    require(len(set(values)) == len(values), "seed bundle sub-seeds must be distinct")
    core = {key: bundle.get(key) for key in ("stage", "block_id", "sub_seeds", "traces", "independent_sub_seeds", "time_indexed_traces")}
    require(bundle.get("seed_bundle_hash") == canonical_hash(core), "seed bundle hash mismatch")
    return {"status": "PASS", "seed_bundle_hash": bundle["seed_bundle_hash"], "trace_hashes": {name: traces[name]["trace_hash"] for name in SEED_STREAMS}}


def make_randomization(stage: str, blocks: int, seed_text: str) -> Dict[str, Any]:
    require(stage in STAGES, f"unknown stage: {stage}")
    require(blocks > 0, "blocks must be positive")
    conditions = list(STAGES[stage]["conditions"])
    stage_seed = derive_seed(seed_text, "randomization", stage)
    rng = random.Random(stage_seed)
    base = conditions[:]
    rng.shuffle(base)
    rows: List[Dict[str, Any]] = []
    for block in range(1, blocks + 1):
        # A cyclic Latin schedule guarantees position counts differ by no more than one.
        offset = (block - 1) % len(base)
        order = base[offset:] + base[:offset]
        for position, condition in enumerate(order, start=1):
            rows.append(
                {
                    "stage": stage,
                    "block_id": f"b{block:02d}",
                    "order_position": position,
                    "condition_id": condition,
                    "path_id": STAGES[stage]["path_id"],
                    "container_id": STAGES[stage]["container_id"],
                    "planned_block_segment_id": f"{stage}_b{block:02d}_seg01",
                }
            )
    core = {
        "schema_version": SCHEMA_VERSION,
        "table_id": f"{stage}_randomization",
        "stage": stage,
        "algorithm": "seeded_cyclic_latin_complete_blocks_v1",
        "seed_commitment": hashlib.sha256(seed_text.encode("utf-8")).hexdigest(),
        "rows": rows,
    }
    return dict(core, randomization_hash=canonical_hash(core))


def validate_randomization_table(table: Any, stage: str, blocks: int = 8) -> Dict[str, Any]:
    table = require_mapping(table, f"{stage} randomization table")
    require(table.get("schema_version") == SCHEMA_VERSION, f"{stage} randomization schema mismatch")
    require(table.get("stage") == stage, f"{stage} randomization stage mismatch")
    require(table.get("table_id") == f"{stage}_randomization", f"{stage} randomization table ID mismatch")
    require(is_sha256(table.get("seed_commitment")), f"{stage} randomization seed commitment is missing")
    core = {key: table.get(key) for key in ("schema_version", "table_id", "stage", "algorithm", "seed_commitment", "rows")}
    require(table.get("randomization_hash") == canonical_hash(core), f"{stage} randomization hash mismatch")
    rows = table.get("rows")
    require(isinstance(rows, list) and len(rows) == blocks * len(STAGES[stage]["conditions"]), f"{stage} randomization row count is invalid")
    groups: Dict[str, List[Mapping[str, Any]]] = {}
    positions_by_condition: Dict[str, List[int]] = {condition: [] for condition in STAGES[stage]["conditions"]}
    for item in rows:
        item = require_mapping(item, f"{stage} randomization row")
        require(item.get("stage") == stage and item.get("path_id") == STAGES[stage]["path_id"] and item.get("container_id") == STAGES[stage]["container_id"], f"{stage} randomization identity mismatch")
        block = item.get("block_id")
        require(isinstance(block, str) and re.fullmatch(r"b[0-9]{2}", block) is not None, f"{stage} randomization block ID invalid")
        position = item.get("order_position")
        condition = item.get("condition_id")
        require(isinstance(position, int) and 1 <= position <= len(STAGES[stage]["conditions"]), f"{stage} randomization position invalid")
        require(condition in STAGES[stage]["conditions"], f"{stage} randomization condition invalid")
        require(item.get("planned_block_segment_id") == f"{stage}_{block}_seg01", f"{stage} randomization planned segment mismatch")
        groups.setdefault(block, []).append(item)
        positions_by_condition[condition].append(position)
    require(set(groups) == {f"b{index:02d}" for index in range(1, blocks + 1)}, f"{stage} randomization block coverage is invalid")
    for block, group in groups.items():
        require(len(group) == len(STAGES[stage]["conditions"]), f"{stage} {block} randomization is incomplete")
        require({item["condition_id"] for item in group} == set(STAGES[stage]["conditions"]), f"{stage} {block} randomization condition set is invalid")
        require({item["order_position"] for item in group} == set(range(1, len(STAGES[stage]["conditions"]) + 1)), f"{stage} {block} randomization positions are invalid")
    for condition, positions in positions_by_condition.items():
        counts = [positions.count(position) for position in range(1, len(STAGES[stage]["conditions"]) + 1)]
        require(max(counts) - min(counts) <= 1, f"{stage} randomization position imbalance for {condition}")
    return {"status": "PASS", "randomization_hash": table["randomization_hash"], "table_id": table["table_id"]}


def has_forbidden_w5(value: Any) -> bool:
    if isinstance(value, str):
        return FORBIDDEN_W5_PATTERN.search(value) is not None
    if isinstance(value, Mapping):
        return any(has_forbidden_w5(item) for item in value.values())
    if isinstance(value, (list, tuple)):
        return any(has_forbidden_w5(item) for item in value)
    return False


def require_simulation_owned_runtime_reference(value: Any, label: str) -> None:
    """Reject a formal runtime delegate/reference from the physical stack.

    Formal command manifests are hash-bound, but a hash does not make an
    out-of-bound controller, experiment runner, or LiquidSloshModel library
    admissible.  Check both path-like values and ROS/package-selector syntax
    (for example ``pkg:=spmpc_local_planner``) without matching the distinct
    simulation package name ``spmpc_sim_local_planner``.
    """

    text = str(value)
    lexical = Path(text)
    require(
        not any(component in REAL_STACK_RUNTIME_COMPONENTS for component in lexical.parts),
        f"{label} references a forbidden real-stack package/path: {text}",
    )
    require(
        REAL_STACK_RUNTIME_REFERENCE_RE.search(text) is None,
        f"{label} references a forbidden real-stack package/path: {text}",
    )
    require(
        lexical.name.lower() not in REAL_STACK_RUNTIME_LIBRARY_BASENAMES,
        f"{label} references a forbidden real-stack library: {text}",
    )


def find_sim_only_release_markers(value: Any, path: str = "$") -> List[str]:
    """Return explicit SIM-ONLY identities embedded in an arbitrary payload.

    This is intentionally exact/prefix-based rather than a broad text
    search: an ordinary documentation string containing the word "simulation"
    must not make a formal freeze fail, while the independently named release
    and its protocol root must always be visible to the formal gate.
    """

    hits: List[str] = []
    if isinstance(value, str):
        if value in SIM_ONLY_VARIANT_IDS or value.startswith(SIM_ONLY_PROTOCOL_PREFIX):
            hits.append(f"{path}={value}")
    elif isinstance(value, Mapping):
        for key, item in value.items():
            hits.extend(find_sim_only_release_markers(item, f"{path}.{key}"))
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            hits.extend(find_sim_only_release_markers(item, f"{path}[{index}]"))
    return hits


def validate_retry_classifier(classifier: Any, formal: bool = False) -> Dict[str, Any]:
    """Validate the semantics of a condition-blind retry classifier.

    The classifier is deliberately an artifact contract, not a heuristic in
    the runner.  A launch error is non-retryable unless an external classifier
    later supplies a decision bound to this immutable contract.
    """
    classifier = require_mapping(classifier, "retry classifier")
    require(isinstance(classifier.get("classifier_id"), str) and classifier["classifier_id"], "retry classifier_id is required")
    require(is_sha256(classifier.get("classifier_rule_hash")), "retry classifier_rule_hash must be a SHA-256")
    rules = require_mapping(classifier.get("rules"), "retry classifier rules")
    require(classifier.get("classifier_rule_hash") == canonical_hash(dict(rules)), "retry classifier_rule_hash does not bind embedded rules")
    input_fields = rules.get("input_fields")
    require(isinstance(input_fields, list) and input_fields and all(isinstance(item, str) and item for item in input_fields), "retry classifier rules need explicit input_fields")
    forbidden_assignment_inputs = ("condition", "method", "backend", "config", "profile", "seed", "assignment", "planner", "controller")
    require(
        not any(any(token in item.casefold() for token in forbidden_assignment_inputs) for item in input_fields),
        "retry classifier rules read method/condition assignment fields",
    )
    require(isinstance(classifier.get("verifier_id"), str) and classifier["verifier_id"], "retry verifier_id is required")
    require(is_sha256(classifier.get("verifier_hash")), "retry verifier_hash must be a SHA-256")
    require(classifier.get("condition_blind") is True, "retry classifier must be condition_blind=true")
    require(classifier.get("pre_motion_only") is True, "retry classifier must be pre_motion_only=true")
    require(classifier.get("retryable_failure_classes") == [RETRYABLE_FAILURE_CLASS], "retry classifier may authorize only INFRASTRUCTURE_ACQUISITION")
    denied = classifier.get("nonretryable_failure_classes")
    require(isinstance(denied, list) and NONRETRYABLE_FAILURE_CLASSES.issubset(set(denied)), "retry classifier must explicitly deny METHOD_FAILURE and PROTOCOL_FAILURE")
    max_retries = classifier.get("max_retries_per_row")
    require(isinstance(max_retries, int) and 1 <= max_retries <= 99, "retry classifier max_retries_per_row must be 1..99")
    reason_codes = classifier.get("reason_codes")
    require(isinstance(reason_codes, list) and reason_codes and all(isinstance(item, str) and item for item in reason_codes), "retry classifier reason_codes must be a non-empty string list")
    require(len(set(reason_codes)) == len(reason_codes), "retry classifier reason_codes must be unique")
    for key in ("missingness_rule_hash", "stop_resume_rule_hash"):
        require(is_sha256(classifier.get(key)), f"retry classifier {key} must be a SHA-256")
    if formal:
        require(classifier.get("status") == "FROZEN", "formal retry classifier status must be FROZEN")
        require(classifier.get("protocol_id") == FORMAL_PROTOCOL_ID, "formal retry classifier protocol_id mismatch")
    return {
        "status": "PASS",
        "classifier_id": classifier["classifier_id"],
        "classifier_rule_hash": classifier["classifier_rule_hash"],
        "verifier_id": classifier["verifier_id"],
        "verifier_hash": classifier["verifier_hash"],
        "reason_codes": list(reason_codes),
        "max_retries_per_row": max_retries,
        "rules": dict(rules),
    }


def validate_frozen_retry_classifier(entry: Any) -> Dict[str, Any]:
    """Bind a formal classifier declaration to its rule, verifier and JSON."""
    entry = require_mapping(entry, "formal retry classifier")
    manifest_path = validate_bound_file(entry, "classifier_manifest_path", "classifier_manifest_hash", "formal retry classifier manifest")
    verifier_path = validate_bound_file(entry, "verifier_path", "verifier_hash", "formal retry classifier verifier")
    document = require_mapping(read_json(manifest_path), "formal retry classifier manifest")
    require(document.get("document_type") == "SMPCC_SIM_RETRY_CLASSIFIER", "formal retry classifier manifest has wrong document_type")
    report = validate_retry_classifier(document, formal=True)
    for key in (
        "classifier_id",
        "classifier_rule_hash",
        "verifier_id",
        "verifier_hash",
        "condition_blind",
        "pre_motion_only",
        "retryable_failure_classes",
        "nonretryable_failure_classes",
        "max_retries_per_row",
        "reason_codes",
        "missingness_rule_hash",
        "stop_resume_rule_hash",
        "rules",
    ):
        require(entry.get(key) == document.get(key), f"formal retry classifier {key} does not match its manifest")
    verifier_command = document.get("verifier_command")
    require(isinstance(verifier_command, list) and verifier_command == [str(verifier_path.resolve())], "formal retry classifier verifier_command must execute only the bound verifier")
    require(entry.get("verifier_command") == verifier_command, "formal retry classifier verifier_command does not match its manifest")
    require(os.access(verifier_path, os.X_OK), "formal retry classifier verifier is not executable")
    require(sha256_file(verifier_path) == document["verifier_hash"], "formal retry classifier verifier hash mismatch")
    return dict(
        report,
        classifier_manifest_path=str(manifest_path.resolve()),
        classifier_manifest_hash=sha256_file(manifest_path),
        verifier_path=str(verifier_path.resolve()),
        verifier_command=list(verifier_command),
    )


def validate_failure_classification(
    decision: Any,
    classifier: Mapping[str, Any],
    attempt_id: str,
    case_manifest_hash: str,
    failure_event_path: Optional[Path] = None,
    failure_event_hash: Optional[str] = None,
) -> Dict[str, Any]:
    """Validate an external, pre-motion acquisition decision for one case."""
    decision = require_mapping(decision, "failure classification decision")
    require(decision.get("decision_type") == "SMPCC_SIM_FAILURE_CLASSIFICATION", "failure classification has wrong decision_type")
    require(decision.get("status") == "PASS", "failure classification decision is not PASS")
    require(decision.get("attempt_id") == attempt_id, "failure classification belongs to another attempt")
    require(decision.get("case_launch_manifest_hash") == case_manifest_hash, "failure classification belongs to another case manifest")
    require(decision.get("failure_class") == RETRYABLE_FAILURE_CLASS, "only infrastructure acquisition decisions are retry-authorizable")
    require(decision.get("motion_started") is False and decision.get("assignment_consumed") is False, "retry classification must prove pre-motion, pre-assignment failure")
    require(decision.get("condition_blind") is True and decision.get("pre_motion_only") is True, "retry classification is not condition-blind pre-motion evidence")
    for key in ("classifier_id", "classifier_rule_hash", "classifier_manifest_hash", "verifier_id", "verifier_hash"):
        require(decision.get(key) == classifier.get(key), f"failure classification {key} differs from frozen classifier")
    require(decision.get("reason_code") in classifier.get("reason_codes", []), "failure classification reason_code is not frozen/allowed")
    if failure_event_path is not None or failure_event_hash is not None:
        require(failure_event_path is not None and failure_event_hash is not None, "failure classification needs both failure event path and hash")
        require(failure_event_path.is_file() and sha256_file(failure_event_path) == failure_event_hash, "failure classification failure event artifact hash mismatch")
        event = require_mapping(read_json(failure_event_path), "failure classification event")
        require(event.get("event_type") == "SMPCC_SIM_PREMOTION_FAILURE_EVENT", "failure classification event has wrong event_type")
        require(event.get("attempt_id") == attempt_id and event.get("case_launch_manifest_hash") == case_manifest_hash, "failure classification event belongs to another attempt")
        require(decision.get("failure_event_path") == str(failure_event_path.resolve()), "failure classification decision failure event path mismatch")
        require(decision.get("failure_event_hash") == failure_event_hash, "failure classification decision failure event hash mismatch")
        require(decision.get("failure_event_error_hash") == event.get("error_hash"), "failure classification decision error hash mismatch")
    core = dict(decision)
    declared_hash = core.pop("decision_hash", None)
    require(declared_hash == canonical_hash(core), "failure classification decision_hash mismatch")
    return dict(decision)


def _fixture_contrast_registry(protocol_id: str) -> Dict[str, Any]:
    """Small explicit registry used only by fixture matrices and tests."""
    items = [
        ("SIM-S1_CORE", "Bslosh", "Bsmooth", "primary_physical"),
        ("SIM-S1_CORE", "Bslosh", "FixedProfile", "novelty"),
        ("SIM-S1_CORE", "Bslosh", "B0", "key_secondary"),
        ("SIM-S1_CORE", "Bsmooth", "B0", "mechanism_diagnostic"),
        ("SIM-S1_CORE", "SmoothMatch", "Bsmooth", "completion_match_gate"),
        ("SIM-S2A_SELECTIVITY", "Bslosh", "Bsmooth", "primary_physical"),
        ("SIM-S2A_SELECTIVITY", "Bslosh", "FixedProfile", "novelty"),
        ("SIM-S2B_TRANSFER", "Bslosh", "Bsmooth", "primary_physical"),
        ("SIM-S2B_TRANSFER", "Bslosh", "FixedProfile", "novelty"),
    ]
    contrasts = []
    for stage, left, right, role in items:
        contrasts.append(
            {
                "contrast_id": f"{stage}:{left}-minus-{right}",
                "stage": stage,
                "path_id": STAGES[stage]["path_id"],
                "left_condition": left,
                "right_condition": right,
                "n_block_plan": 8,
                "minimum_n_pair": 0,
                "paired_within_block": True,
                "failure_inclusive": True,
                "continuous_outcome_requires_same_segment": True,
                "analysis_rule_hash": canonical_hash({"fixture": True, "stage": stage, "left": left, "right": right, "role": role}),
                "role": role,
            }
        )
    core = {
        "registry_id": "FIXTURE-SMPCC-SIM-CONTRASTS-v1",
        "protocol_id": protocol_id,
        "status": "FIXTURE_ONLY",
        "contrasts": contrasts,
    }
    return dict(core, registry_hash=canonical_hash(core))


def validate_contrast_registry(registry: Any, formal: bool = False) -> Dict[str, Any]:
    """Validate registered block-paired contrasts without inferring a default."""
    registry = require_mapping(registry, "contrast registry")
    require(isinstance(registry.get("registry_id"), str) and registry["registry_id"], "contrast registry_id is required")
    if formal:
        require(registry.get("protocol_id") == FORMAL_PROTOCOL_ID, "formal contrast registry protocol_id mismatch")
        require(registry.get("status") == "FROZEN", "formal contrast registry status must be FROZEN")
    else:
        require(registry.get("status") in {"FROZEN", "FIXTURE_ONLY"}, "contrast registry status is invalid")
    core = dict(registry)
    declared_hash = core.pop("registry_hash", None)
    require(declared_hash == canonical_hash(core), "contrast registry_hash mismatch")
    contrasts = registry.get("contrasts")
    require(isinstance(contrasts, list) and contrasts, "contrast registry contrasts must be a non-empty list")
    ids = set()
    pairs_by_stage: Dict[str, set[frozenset[str]]] = {stage: set() for stage in STAGES}
    primary_count: Dict[str, int] = {stage: 0 for stage in STAGES}
    primary_pairs: Dict[str, List[frozenset[str]]] = {stage: [] for stage in STAGES}
    for index, item in enumerate(contrasts):
        item = require_mapping(item, f"contrast registry contrast {index}")
        contrast_id = item.get("contrast_id")
        require(isinstance(contrast_id, str) and contrast_id and contrast_id not in ids, "contrast registry contrast_id is missing or duplicated")
        ids.add(contrast_id)
        stage = item.get("stage")
        require(stage in STAGES, f"contrast {contrast_id} has unknown stage")
        require(item.get("path_id") == STAGES[stage]["path_id"], f"contrast {contrast_id} path_id does not match stage")
        left, right = item.get("left_condition"), item.get("right_condition")
        allowed_conditions = set(STAGES[stage]["conditions"])
        require(left in allowed_conditions and right in allowed_conditions and left != right, f"contrast {contrast_id} uses invalid condition IDs")
        pair = frozenset((str(left), str(right)))
        require(pair not in pairs_by_stage[stage], f"contrast registry duplicates condition pair in {stage}")
        pairs_by_stage[stage].add(pair)
        require(item.get("n_block_plan") == 8, f"contrast {contrast_id} n_block_plan must be 8")
        minimum = item.get("minimum_n_pair")
        require(isinstance(minimum, int) and 0 <= minimum <= 8, f"contrast {contrast_id} minimum_n_pair must be 0..8")
        require(item.get("paired_within_block") is True, f"contrast {contrast_id} must be paired within block")
        require(item.get("failure_inclusive") is True, f"contrast {contrast_id} must declare failure-inclusive analysis")
        require(item.get("continuous_outcome_requires_same_segment") is True, f"contrast {contrast_id} must reject split-block continuous pairs")
        require(is_sha256(item.get("analysis_rule_hash")), f"contrast {contrast_id} analysis_rule_hash must be a SHA-256")
        require(isinstance(item.get("role"), str) and item["role"], f"contrast {contrast_id} role is required")
        if item.get("role") == "primary_physical":
            require("Bslosh" in pair, f"primary physical contrast {contrast_id} must include Bslosh")
            primary_count[stage] += 1
            primary_pairs[stage].append(pair)
    require(all(pairs_by_stage[stage] for stage in STAGES), "contrast registry must register at least one contrast for every stage")
    if formal:
        require(REQUIRED_S1_CONTRAST_PAIRS.issubset(pairs_by_stage["SIM-S1_CORE"]), "formal contrast registry omits a required SIM-S1 comparison")
        require(all(primary_count[stage] == 1 for stage in STAGES), "formal contrast registry must select exactly one primary_physical contrast per stage")
        require(
            primary_pairs["SIM-S1_CORE"] == [frozenset(("Bslosh", "Bsmooth"))],
            "formal SIM-S1 primary_physical contrast must be Bslosh-Bsmooth",
        )
    return {
        "status": "PASS",
        "registry_id": registry["registry_id"],
        "registry_hash": registry["registry_hash"],
        "contrast_count": len(contrasts),
    }


def validate_frozen_contrast_registry(entry: Any) -> Dict[str, Any]:
    entry = require_mapping(entry, "formal contrast registry")
    path = validate_bound_file(entry, "registry_path", "registry_file_hash", "formal contrast registry")
    registry = require_mapping(read_json(path), "formal contrast registry document")
    report = validate_contrast_registry(registry, formal=True)
    require(entry.get("registry_id") == registry.get("registry_id"), "formal contrast registry_id does not match registry document")
    require(entry.get("registry_hash") == registry.get("registry_hash"), "formal contrast registry canonical hash does not match registry document")
    return dict(report, registry_path=str(path.resolve()), registry_file_hash=sha256_file(path), registry=registry)


def validate_frozen_stage_entry_policy(entry: Any) -> Dict[str, Any]:
    """Bind the scientific Stage-I/II entry rules before any planned row runs.

    A closure report only establishes ledger completeness.  The extension and
    transfer decisions need their own frozen rule/validator identities so a
    later hand-written ``status=PASS`` report cannot open a new stage.
    """
    entry = require_mapping(entry, "formal stage-entry policy")
    path = validate_bound_file(entry, "policy_path", "policy_file_hash", "formal stage-entry policy")
    document = require_mapping(read_json(path), "formal stage-entry policy document")
    require(document.get("document_type") == STAGE_ENTRY_POLICY_DOCUMENT_TYPE, "formal stage-entry policy has wrong document_type")
    require(document.get("protocol_id") == FORMAL_PROTOCOL_ID and document.get("status") == "FROZEN", "formal stage-entry policy is not frozen for this protocol")
    require(isinstance(document.get("policy_id"), str) and document["policy_id"], "formal stage-entry policy_id is missing")
    core = dict(document)
    declared_hash = core.pop("policy_hash", None)
    require(declared_hash == canonical_hash(core), "formal stage-entry policy_hash mismatch")
    require(entry.get("policy_id") == document.get("policy_id") and entry.get("policy_hash") == document.get("policy_hash"), "formal stage-entry policy declaration mismatch")
    reports = require_mapping(document.get("reports"), "formal stage-entry policy reports")
    required_report_types = {
        "SIM_S1_CLOSURE",
        STAGE1_EXTENSION_REPORT_TYPE,
        "SIM_S2A_CLOSURE",
        STAGE2A_SELECTIVITY_REPORT_TYPE,
        STAGE2B_TRIGGER_REPORT_TYPE,
    }
    require(
        set(reports) == required_report_types,
        "formal stage-entry policy must freeze exactly S1 closure/extension, S2A closure/selectivity and S2B trigger reports",
    )
    normalized: Dict[str, Mapping[str, Any]] = {}
    for report_type in sorted(required_report_types):
        report_rule = require_mapping(reports.get(report_type), f"formal stage-entry policy {report_type}")
        require(report_rule.get("report_type") == report_type, f"formal stage-entry policy {report_type} type mismatch")
        for key in ("rule_hash", "validator_hash"):
            require(is_sha256(report_rule.get(key)), f"formal stage-entry policy {report_type} lacks {key}")
        normalized[report_type] = dict(report_rule)
    required_gates = normalized[STAGE1_EXTENSION_REPORT_TYPE].get("required_gate_ids")
    require(
        isinstance(required_gates, list)
        and len(required_gates) == len(set(required_gates))
        and set(required_gates) == REQUIRED_S1_EXTENSION_GATES,
        "formal stage-entry policy S1 extension gates differ from the required matrix gates",
    )
    allowed_selectivity = normalized[STAGE2A_SELECTIVITY_REPORT_TYPE].get("allowed_selectivity_statuses")
    require(
        isinstance(allowed_selectivity, list)
        and allowed_selectivity
        and set(allowed_selectivity).issubset({"SUPPORTED", "NOT_SUPPORTED", "INCONCLUSIVE"}),
        "formal stage-entry policy S2A allowed_selectivity_statuses is invalid",
    )
    return {
        "status": "PASS",
        "policy_id": document["policy_id"],
        "policy_hash": document["policy_hash"],
        "policy_path": str(path.resolve()),
        "policy_file_hash": sha256_file(path),
        "reports": normalized,
    }


def validate_frozen_runtime_launch_contract(entry: Any) -> Dict[str, Any]:
    """Bind every formal launch/probe command and ack schema before a row."""
    entry = require_mapping(entry, "formal runtime launch contract")
    path = validate_bound_file(entry, "contract_path", "contract_file_hash", "formal runtime launch contract")
    require_simulation_owned_runtime_reference(path, "formal runtime launch contract")
    document = require_mapping(read_json(path), "formal runtime launch contract document")
    require(document.get("document_type") == "SMPCC_SIM_RUNTIME_LAUNCH_CONTRACT", "formal runtime launch contract has wrong document_type")
    require(document.get("status") == "FROZEN" and document.get("protocol_id") == FORMAL_PROTOCOL_ID, "formal runtime launch contract is not frozen for this protocol")
    require(isinstance(document.get("contract_id"), str) and document["contract_id"], "formal runtime launch contract_id is missing")
    core = dict(document)
    declared_hash = core.pop("contract_hash", None)
    require(declared_hash == canonical_hash(core), "formal runtime launch contract_hash mismatch")
    require(entry.get("contract_id") == document.get("contract_id") and entry.get("contract_hash") == document.get("contract_hash"), "formal runtime launch contract declaration mismatch")
    commands = require_mapping(document.get("commands"), "formal runtime launch commands")
    require(set(commands) == set(FORMAL_RUNTIME_COMMAND_FIELDS), "formal runtime launch contract must freeze exactly every runner command")
    command_file_hashes = require_mapping(document.get("command_file_hashes"), "formal runtime launch command_file_hashes")
    require(set(command_file_hashes) == set(FORMAL_RUNTIME_COMMAND_FIELDS), "formal runtime launch contract must hash every runner command")
    normalized_commands: Dict[str, List[str]] = {}
    normalized_file_hashes: Dict[str, List[Dict[str, str]]] = {}
    for field in FORMAL_RUNTIME_COMMAND_FIELDS:
        normalized_commands[field] = command_from_spec(commands.get(field), f"formal runtime launch contract {field}")
        command = normalized_commands[field]
        require(Path(command[0]).is_absolute() and Path(command[0]).is_file(), f"formal runtime {field} executable must be an absolute file")
        for token in command:
            require_simulation_owned_runtime_reference(token, f"formal runtime {field} command")
        raw_refs = command_file_hashes.get(field)
        require(isinstance(raw_refs, list) and raw_refs, f"formal runtime {field} command_file_hashes is missing")
        refs: List[Dict[str, str]] = []
        for item in raw_refs:
            item = require_mapping(item, f"formal runtime {field} command file hash")
            path_value, expected_hash = item.get("path"), item.get("sha256")
            require(isinstance(path_value, str) and Path(path_value).is_absolute() and Path(path_value).is_file(), f"formal runtime {field} command hash path is invalid")
            require_simulation_owned_runtime_reference(path_value, f"formal runtime {field} command hash")
            require(is_sha256(expected_hash) and sha256_file(Path(path_value)) == expected_hash, f"formal runtime {field} command file hash mismatch")
            refs.append({"path": str(Path(path_value).resolve()), "sha256": str(expected_hash)})
        ref_paths = {item["path"] for item in refs}
        require(str(Path(command[0]).resolve()) in ref_paths, f"formal runtime {field} executable is not hash-bound")
        for token in command:
            if token.startswith("/") and Path(token).is_file():
                require(str(Path(token).resolve()) in ref_paths, f"formal runtime {field} absolute file argument is not hash-bound")
        normalized_file_hashes[field] = refs
    startup_timeout = document.get("startup_timeout_sec")
    require(isinstance(startup_timeout, (int, float)) and math.isfinite(float(startup_timeout)) and 0.0 < float(startup_timeout) <= 600.0, "formal runtime startup_timeout_sec is invalid")
    command_timeout = document.get("command_timeout_sec")
    require(isinstance(command_timeout, (int, float)) and math.isfinite(float(command_timeout)) and 0.0 < float(command_timeout) <= 120.0, "formal runtime command_timeout_sec is invalid")
    require(document.get("recorder_artifact_must_be_case_local") is True, "formal runtime launch contract must require a case-local recorder artifact")
    ros_master_uri = document.get("ros_master_uri")
    gazebo_master_uri = document.get("gazebo_master_uri")
    require(isinstance(ros_master_uri, str) and isinstance(gazebo_master_uri, str), "formal runtime launch contract must freeze ROS/Gazebo master URIs")
    parse_endpoint(ros_master_uri)
    parse_endpoint(gazebo_master_uri)
    for key in (
        "runtime_ack_schema_hash",
        "motion_release_ack_schema_hash",
        "motion_stop_ack_schema_hash",
        "goal_reached_rule_hash",
        "first_motion_telemetry_schema_hash",
        "recorder_artifact_rule_hash",
    ):
        require(is_sha256(document.get(key)), f"formal runtime launch contract {key} must be a SHA-256")
    return {
        "status": "PASS",
        "contract_id": document["contract_id"],
        "contract_hash": document["contract_hash"],
        "contract_path": str(path.resolve()),
        "contract_file_hash": sha256_file(path),
        "commands": normalized_commands,
        "command_file_hashes": normalized_file_hashes,
        "startup_timeout_sec": float(startup_timeout),
        "command_timeout_sec": float(command_timeout),
        "runtime_ack_schema_hash": document["runtime_ack_schema_hash"],
        "motion_release_ack_schema_hash": document["motion_release_ack_schema_hash"],
        "motion_stop_ack_schema_hash": document["motion_stop_ack_schema_hash"],
        "goal_reached_rule_hash": document["goal_reached_rule_hash"],
        "first_motion_telemetry_schema_hash": document["first_motion_telemetry_schema_hash"],
        "recorder_artifact_rule_hash": document["recorder_artifact_rule_hash"],
        "ros_master_uri": ros_master_uri,
        "gazebo_master_uri": gazebo_master_uri,
    }


def validate_formal_runtime_backend_manifest(
    entry: Any,
    freeze: Mapping[str, Any],
    launch_contract: Mapping[str, Any],
) -> Dict[str, Any]:
    """Validate the concrete backend behind the 8-command adapter ABI.

    ``runtime_launch_contract`` binds the outer runner commands.  For the
    formal adapter those commands are intentionally small wrappers, so a
    second immutable manifest must bind the actual launch/ready/recorder/
    motion/probe/ACK delegates.  This validator lives in the shared formal
    gate: a freeze cannot PASS merely because the adapter would reject a
    missing backend later at first launch.
    """
    backend = require_mapping(entry, "formal runtime backend")
    manifest_path = validate_bound_file(
        backend,
        "backend_manifest_path",
        "backend_manifest_hash",
        "formal runtime backend",
    )
    require_simulation_owned_runtime_reference(manifest_path, "formal runtime backend manifest")
    document = require_mapping(read_json(manifest_path), "formal runtime backend manifest")
    require(document.get("document_type") == "SMPCC_SIM_FORMAL_RUNTIME_BACKEND", "formal runtime backend has wrong document_type")
    require(document.get("status") == "FROZEN", "formal runtime backend is not FROZEN")
    require(document.get("protocol_id") == FORMAL_PROTOCOL_ID, "formal runtime backend protocol_id mismatch")
    require(document.get("formal") is True and document.get("development_only") is False, "formal runtime backend must be formal and not development_only")
    require(document.get("runtime_backend_implemented") is True and document.get("delegate_via_execve") is True, "formal runtime backend is not an implemented execve delegate")
    require(document.get("legacy_wrappers_forbidden") is True, "formal runtime backend must explicitly forbid legacy wrappers")
    require(isinstance(document.get("backend_id"), str) and document["backend_id"], "formal runtime backend_id is missing")
    core = dict(document)
    declared_hash = core.pop("backend_hash", None)
    require(declared_hash == canonical_hash(core), "formal runtime backend_hash mismatch")
    require(backend.get("backend_id") == document["backend_id"] and backend.get("backend_hash") == declared_hash, "formal runtime backend declaration mismatch")
    for key in ("sim_freeze_id", "git_revision", "build_id"):
        require(document.get(key) == freeze.get(key), f"formal runtime backend {key} differs from formal freeze")
    require(
        document.get("runtime_launch_contract_id") == launch_contract.get("contract_id")
        and document.get("runtime_launch_contract_hash") == launch_contract.get("contract_hash"),
        "formal runtime backend is not bound to the frozen launch contract",
    )

    lifecycle = require_mapping(document.get("lifecycle"), "formal runtime backend lifecycle")
    policy = require_mapping(freeze.get("recording_policy"), "formal recording policy")
    for key, expected in {
        "fresh_master_required": True,
        "owned_process_groups_only": True,
        "recorder_before_motion": True,
        "goal_status_exact": "GOAL_REACHED",
        "success_tail_required": True,
        "timeout_tail_required": True,
        "runtime_ack_required": True,
        "motion_release_ack_required": True,
        "motion_stop_ack_required": True,
        "controller_firewall_required": True,
        "broad_process_control_forbidden": True,
    }.items():
        require(lifecycle.get(key) == expected, f"formal runtime backend lifecycle {key} must be {expected!r}")
    require(lifecycle.get("settle_sec") == 30.0 == policy.get("settle_sec"), "formal runtime backend settle must be exactly 30 seconds")
    require(
        lifecycle.get("effective_motion_window_sec") == 60.0 == policy.get("goal_timeout_sec"),
        "formal runtime backend effective-motion window must be exactly 60 seconds",
    )
    require(lifecycle.get("tail_sec") == policy.get("tail_sec"), "formal runtime backend tail differs from frozen recording policy")
    require(
        lifecycle.get("controller_firewall_checkpoints") == ["ready", "pre_motion", "postflight"],
        "formal runtime backend must freeze ready/pre_motion/postflight firewall checkpoints",
    )

    required_environment = {
        "ROS_MASTER_URI",
        "GAZEBO_MASTER_URI",
        "SMPCC_CASE_LAUNCH_MANIFEST_PATH",
        "SMPCC_CASE_LAUNCH_MANIFEST_SHA256",
        "SMPCC_SEED_BUNDLE_PATH",
        "SMPCC_SEED_BUNDLE_SHA256",
        "SMPCC_FORMAL_FREEZE_PATH",
        "SMPCC_FORMAL_FREEZE_FILE_SHA256",
        "SMPCC_FORMAL_MASTER_PATH",
        "SMPCC_FORMAL_MASTER_FILE_SHA256",
    }
    environment = document.get("required_environment")
    require(
        isinstance(environment, list) and len(environment) == len(set(environment)) and set(environment) == required_environment,
        "formal runtime backend required_environment differs from the immutable ABI environment",
    )
    commands = require_mapping(document.get("commands"), "formal runtime backend commands")
    command_file_hashes = require_mapping(document.get("command_file_hashes"), "formal runtime backend command_file_hashes")
    require(set(commands) == set(FORMAL_RUNTIME_COMMAND_FIELDS), "formal runtime backend must provide exactly the 8 delegate commands")
    require(set(command_file_hashes) == set(FORMAL_RUNTIME_COMMAND_FIELDS), "formal runtime backend must hash exactly the 8 delegate commands")
    normalized_commands: Dict[str, List[str]] = {}
    for field in FORMAL_RUNTIME_COMMAND_FIELDS:
        command = command_from_spec(commands.get(field), f"formal runtime backend {field}")
        require(Path(command[0]).is_absolute() and Path(command[0]).is_file(), f"formal runtime backend {field} executable must be an absolute file")
        require(os.access(command[0], os.X_OK), f"formal runtime backend {field} executable is not executable")
        for token in command:
            require_simulation_owned_runtime_reference(token, f"formal runtime backend {field} command")
        require(not has_forbidden_w5(command), f"formal runtime backend {field} revives rejected W5_S10")
        joined = " ".join(command).lower()
        require("runtime_s_curve" not in joined, f"formal runtime backend {field} invokes runtime_s_curve")
        require(
            not any(Path(token).name.lower() in FORMAL_RUNTIME_FORBIDDEN_BACKEND_BASENAMES for token in command),
            f"formal runtime backend {field} invokes a forbidden legacy/H0 wrapper",
        )
        refs_raw = command_file_hashes.get(field)
        require(isinstance(refs_raw, list) and refs_raw, f"formal runtime backend {field} command_file_hashes is missing")
        refs: set[str] = set()
        for item in refs_raw:
            item = require_mapping(item, f"formal runtime backend {field} command file hash")
            path_value, expected_hash = item.get("path"), item.get("sha256")
            require(isinstance(path_value, str) and Path(path_value).is_absolute() and Path(path_value).is_file(), f"formal runtime backend {field} command hash path is invalid")
            require_simulation_owned_runtime_reference(path_value, f"formal runtime backend {field} command hash")
            require(is_sha256(expected_hash) and sha256_file(Path(path_value)) == expected_hash, f"formal runtime backend {field} command file hash mismatch")
            resolved = str(Path(path_value).resolve())
            require(resolved not in refs, f"formal runtime backend {field} repeats a command hash path")
            refs.add(resolved)
        require(str(Path(command[0]).resolve()) in refs, f"formal runtime backend {field} executable is not hash-bound")
        for token in command:
            path = Path(token)
            if path.is_absolute():
                require(path.is_file() and str(path.resolve()) in refs, f"formal runtime backend {field} absolute file argument is not hash-bound")
        normalized_commands[field] = command

    replay = require_mapping(document.get("frozen_path_replay"), "formal runtime backend frozen_path_replay")
    require(replay.get("source_mode") == "frozen_json_replay" and replay.get("runtime_generation_forbidden") is True, "formal runtime backend must require frozen JSON path replay")
    replay_path = validate_bound_file(replay, "entrypoint_path", "entrypoint_hash", "formal frozen-path replay entrypoint")
    require(replay_path.resolve() == Path(__file__).with_name("smpcc_sim_frozen_path_replay.py").resolve(), "formal runtime backend must bind the reviewed frozen-path replay entrypoint")
    config_readback = require_mapping(document.get("effective_config_readback"), "formal runtime backend effective_config_readback")
    fields = config_readback.get("consumed_fields")
    require(config_readback.get("required") is True and isinstance(fields, list) and len(fields) == len(set(fields)) and set(fields) == set(REQUIRED_EFFECTIVE_CONFIG_FIELDS), "formal runtime backend must consume every frozen effective-config field")
    require(config_readback.get("runtime_ack_schema_hash") == launch_contract.get("runtime_ack_schema_hash"), "formal runtime backend effective-config ACK schema mismatch")
    goal_policy = require_mapping(document.get("goal_probe_policy"), "formal runtime backend goal_probe_policy")
    require(goal_policy.get("exact_terminal_status") == "GOAL_REACHED" and goal_policy.get("after_motion_release_required") is True, "formal goal probe must prove post-release exact GOAL_REACHED")
    require(goal_policy.get("goal_reached_rule_hash") == launch_contract.get("goal_reached_rule_hash"), "formal runtime backend goal rule mismatch")
    stop_policy = require_mapping(document.get("motion_stop_policy"), "formal runtime backend motion_stop_policy")
    require(stop_policy.get("dedicated_cmd_gate") is True and stop_policy.get("zero_hold_required") is True, "formal runtime backend requires a dedicated zero-hold command gate")
    require(stop_policy.get("motion_stop_ack_schema_hash") == launch_contract.get("motion_stop_ack_schema_hash"), "formal runtime backend motion-stop ACK schema mismatch")
    artifacts = require_mapping(document.get("case_artifacts"), "formal runtime backend case_artifacts")
    expected_artifacts = {"recorder_artifact", "runtime_ack", "motion_release_ack", "motion_stop_ack"}
    require(set(artifacts) == expected_artifacts, "formal runtime backend must freeze exactly four case artifacts")
    normalized_artifacts: Dict[str, str] = {}
    for key in sorted(expected_artifacts):
        name = artifacts.get(key)
        suffix = ".bag.active" if key == "recorder_artifact" else ".json"
        require(isinstance(name, str) and name and not Path(name).is_absolute() and Path(name).name == name and name.endswith(suffix), f"formal runtime backend {key} must be a case-local {suffix} filename")
        normalized_artifacts[key] = name
    require(len(set(normalized_artifacts.values())) == len(normalized_artifacts), "formal runtime backend case artifacts must be distinct")
    return {
        "status": "PASS",
        "backend_id": document["backend_id"],
        "backend_hash": declared_hash,
        "backend_manifest_path": str(manifest_path.resolve()),
        "backend_manifest_hash": sha256_file(manifest_path),
        "commands": normalized_commands,
        "case_artifacts": normalized_artifacts,
        "lifecycle": dict(lifecycle),
    }


def validate_formal_dataset_ledger(entry: Any, freeze: Mapping[str, Any]) -> Dict[str, Any]:
    """Freeze one campaign root so a formal row cannot be double-consumed."""
    entry = require_mapping(entry, "formal dataset ledger")
    ledger_id = entry.get("ledger_id")
    ledger_root_value = entry.get("ledger_root")
    require(isinstance(ledger_id, str) and ledger_id, "formal dataset ledger_id is required")
    require(isinstance(ledger_root_value, str) and ledger_root_value, "formal dataset ledger_root is required")
    ledger_root = Path(ledger_root_value)
    require(ledger_root.is_absolute(), "formal dataset ledger_root must be absolute")
    ensure_within(ledger_root, DEFAULT_SIM_ROOT, "formal dataset ledger_root")
    require(entry.get("protocol_id") == FORMAL_PROTOCOL_ID and entry.get("sim_freeze_id") == freeze.get("sim_freeze_id"), "formal dataset ledger identity differs from freeze")
    require(is_sha256(entry.get("ledger_policy_hash")), "formal dataset ledger_policy_hash must be a SHA-256")
    identity = {
        "ledger_id": ledger_id,
        "ledger_root": str(ledger_root.resolve()),
        "protocol_id": entry["protocol_id"],
        "sim_freeze_id": entry["sim_freeze_id"],
        "ledger_policy_hash": entry["ledger_policy_hash"],
    }
    require(entry.get("ledger_identity_hash") == canonical_hash(identity), "formal dataset ledger_identity_hash mismatch")
    return dict(identity, ledger_identity_hash=entry["ledger_identity_hash"])


def validate_truth_capability(capability: Any) -> Dict[str, Any]:
    """Validate an independent-plant declaration without inventing one."""
    errors: List[str] = []
    if not isinstance(capability, Mapping):
        return {
            "eligible": False,
            "status": "NO_INDEPENDENT_PLANT_CAPABILITY_MANIFEST",
            "errors": ["plant capability manifest is missing"],
        }
    expected = {
        "independent_plant": True,
        "implementation_isolated_from_controller": True,
        "controller_hidden_state_access": False,
        "driven_by": "executed_simulated_base_motion",
        "truth_topic": "/sim_truth/liquid_height",
        "fidelity_validation_status": "PASS",
    }
    for key, expected_value in expected.items():
        if capability.get(key) != expected_value:
            errors.append(f"plant capability {key!r} must be {expected_value!r}")
    for key in ("plant_code_hash", "plant_parameter_hash", "plant_input_schema_hash", "plant_output_schema_hash", "fidelity_report_hash"):
        if not is_sha256(capability.get(key)):
            errors.append(f"plant capability {key} must be a SHA-256")
    if has_forbidden_w5(capability):
        errors.append("plant capability cannot revive rejected W5_S10")
    return {
        "eligible": not errors,
        "status": "PASS" if not errors else "NO_INDEPENDENT_PLANT",
        "errors": errors,
        "truth_topic": capability.get("truth_topic"),
    }


def validate_bound_file(owner: Mapping[str, Any], path_key: str, hash_key: str, label: str) -> Path:
    """A formal hash has value only when it is bound to the actual immutable file."""
    raw_path = owner.get(path_key)
    require(isinstance(raw_path, str) and raw_path, f"{label} missing {path_key}")
    path = Path(raw_path)
    require(path.is_file(), f"{label} file is missing: {path}")
    expected = owner.get(hash_key)
    require(is_sha256(expected), f"{label} missing {hash_key}")
    require(sha256_file(path) == expected, f"{label} hash mismatch for {path}")
    return path


def validate_formal_simulator_assets(entry: Any) -> Dict[str, Any]:
    """Bind the actual Gazebo world separately from clearance geometry.

    Gazebo launches an SDF/XML ``.world``/``.sdf`` artifact, while the rigid
    path/clearance validator needs a small, reviewed JSON geometry model.
    Treating that JSON as ``world_file`` would make the reported runtime world
    hash describe an artifact Gazebo never used.  Formal input therefore binds
    both files, rejects aliasing, and requires the geometry document to name
    the hash of the actual launched SDF world.
    """
    simulator = require_mapping(entry, "formal simulator assets")
    map_path = validate_bound_file(simulator, "map_file", "map_hash", "formal simulator map")
    world_path = validate_bound_file(simulator, "world_file", "world_hash", "formal launched Gazebo world")
    geometry_path = validate_bound_file(
        simulator,
        "world_geometry_file",
        "world_geometry_hash",
        "formal world clearance geometry",
    )
    robot_path = validate_bound_file(simulator, "robot_model_file", "robot_model_hash", "formal simulator robot")
    require(simulator.get("world_runtime_format") == "gazebo_sdf", "formal launched world must declare world_runtime_format=gazebo_sdf")
    require(world_path.resolve() != geometry_path.resolve(), "formal launched world and clearance geometry must be separate files")
    require(simulator.get("world_hash") != simulator.get("world_geometry_hash"), "formal launched world and clearance geometry must not alias hashes")
    require(world_path.suffix.lower() in {".world", ".sdf"}, "formal launched Gazebo world must be a .world or .sdf file")
    require(geometry_path.suffix.lower() == ".json", "formal clearance geometry must be a JSON file")
    try:
        world_text = world_path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ContractError(f"formal launched Gazebo world cannot be read: {world_path}") from exc
    require("<sdf" in world_text.lower(), "formal launched world is not an SDF/XML Gazebo artifact")
    geometry = require_mapping(read_json(geometry_path), "formal world clearance geometry")
    require(
        geometry.get("document_type") == "SMPCC_SIM_WORLD_CLEARANCE_GEOMETRY"
        and geometry.get("status") == "FROZEN",
        "formal world clearance geometry has wrong document_type/status",
    )
    require(
        geometry.get("launched_world_hash") == simulator.get("world_hash"),
        "formal clearance geometry is not bound to the actual launched world hash",
    )
    return {
        "map_path": str(map_path.resolve()),
        "world_path": str(world_path.resolve()),
        "world_geometry_path": str(geometry_path.resolve()),
        "robot_model_path": str(robot_path.resolve()),
        "world_geometry": dict(geometry),
        "map_hash": str(simulator["map_hash"]),
        "world_hash": str(simulator["world_hash"]),
        "world_geometry_hash": str(simulator["world_geometry_hash"]),
        "robot_model_hash": str(simulator["robot_model_hash"]),
    }


def formal_runtime_environment_bindings(
    formal_master_path: Path,
    formal_master_file_hash: str,
    formal_freeze_path: Path,
    formal_freeze_file_hash: str,
) -> Dict[str, str]:
    """Build the sole immutable master/freeze channel for a formal backend.

    A runtime command must not infer a formal input from its working directory
    or optional CLI selector.  The runner writes these four bindings only
    after validating the formal master and freeze, and a backend must verify
    them against the case manifest before launching anything.
    """
    for label, path, expected in (
        ("formal master", formal_master_path, formal_master_file_hash),
        ("formal freeze", formal_freeze_path, formal_freeze_file_hash),
    ):
        require(path.is_absolute(), f"{label} runtime binding path must be absolute")
        require(path.is_file(), f"{label} runtime binding file is missing: {path}")
        require(is_sha256(expected), f"{label} runtime binding hash is invalid")
        require(sha256_file(path) == expected, f"{label} runtime binding hash mismatch")
    return {
        "SMPCC_FORMAL_MASTER_PATH": str(formal_master_path),
        "SMPCC_FORMAL_MASTER_FILE_SHA256": formal_master_file_hash,
        "SMPCC_FORMAL_FREEZE_PATH": str(formal_freeze_path),
        "SMPCC_FORMAL_FREEZE_FILE_SHA256": formal_freeze_file_hash,
    }


def _formal_liquid_bound_json(
    owner: Mapping[str, Any],
    path_key: str,
    hash_key: str,
    label: str,
) -> Tuple[Path, Mapping[str, Any]]:
    """Load one provenance document only after its path/hash pair is bound."""
    path = validate_bound_file(owner, path_key, hash_key, label)
    return path, require_mapping(read_json(path), label)


def _require_formal_liquid_document(
    document: Mapping[str, Any],
    *,
    label: str,
    schema_version: str,
    report_type: str,
    status: str,
) -> None:
    """Shared non-development identity check for the formal plant evidence chain."""
    require(document.get("schema_version") == schema_version, f"{label} schema_version mismatch")
    require(document.get("report_type") == report_type, f"{label} report_type mismatch")
    require(document.get("status") == status, f"{label} status must be {status}")
    require(document.get("formal") is True, f"{label} must set formal=true")
    require(document.get("development_only") is False, f"{label} must set development_only=false")


def _validate_formal_liquid_plant_intake_provenance(
    capability: Mapping[str, Any],
    capability_report_path: Path,
    capability_report: Mapping[str, Any],
    fidelity_report: Mapping[str, Any],
) -> None:
    """Cross-bind the external plant evidence intake instead of trusting PASS text.

    The fidelity math, real-reference provenance and controller/plant code
    isolation are assessed by the separate formal intake package.  This
    protocol validator independently re-checks every artifact boundary and
    enough of the intake ABI to make a hand-written generic PASS report fail
    closed.  It intentionally records that this repository has no configured
    cryptographic trust anchor; an external approval remains required but is
    not falsely presented as locally authenticated.
    """
    require(
        capability.get("schema_version") == FORMAL_LIQUID_PLANT_BINDING_SCHEMA_VERSION,
        "formal liquid plant binding schema_version mismatch",
    )
    require(
        capability.get("binding_type") == FORMAL_LIQUID_PLANT_BINDING_TYPE,
        "formal liquid plant binding_type mismatch",
    )
    require(capability.get("status") == "PASS", "formal liquid plant binding is not PASS")
    binding_core = dict(capability)
    binding_hash = binding_core.pop("binding_payload_hash", None)
    require(binding_hash == canonical_hash(binding_core), "formal liquid plant binding_payload_hash mismatch")
    require(
        capability.get("formal_intake_report_path") == str(capability_report_path.resolve()),
        "formal liquid plant formal_intake_report_path differs from capability report",
    )
    require(
        capability.get("formal_intake_report_hash") == capability.get("plant_capability_report_hash"),
        "formal liquid plant formal_intake_report_hash differs from capability report hash",
    )
    # The capability report predates the other intake documents and names
    # this field ``report_schema_version`` (not ``schema_version``).  Keep
    # that ABI explicit rather than accepting either spelling silently.
    require(
        capability_report.get("report_schema_version") == FORMAL_LIQUID_PLANT_CAPABILITY_SCHEMA_VERSION,
        "formal liquid plant capability report schema_version mismatch",
    )
    require(
        capability_report.get("report_type") == "SMPCC_SIM_INDEPENDENT_LIQUID_PLANT_CAPABILITY"
        and capability_report.get("status") == "PASS"
        and capability_report.get("formal") is True
        and capability_report.get("development_only") is False,
        "formal liquid plant capability report identity/status mismatch",
    )
    report_core = dict(capability_report)
    report_payload_hash = report_core.pop("capability_report_payload_hash", None)
    require(
        report_payload_hash == canonical_hash(report_core),
        "formal liquid plant capability_report_payload_hash mismatch",
    )
    require(
        capability_report.get("tool_id") == FORMAL_LIQUID_PLANT_INTAKE_TOOL_ID
        and capability.get("formal_intake_tool_id") == FORMAL_LIQUID_PLANT_INTAKE_TOOL_ID,
        "formal liquid plant provenance does not come from the reviewed intake tool",
    )
    for key in FORMAL_LIQUID_PLANT_PROVENANCE_FIELDS:
        require(
            capability_report.get(key) == capability.get(key),
            f"formal liquid plant capability/binding {key} mismatch",
        )
    for key in (
        "formal",
        "development_only",
        "physical_primary_eligible",
        "independent_plant",
        "implementation_isolated_from_controller",
        "controller_hidden_state_access",
        "driven_by",
        "truth_topic",
        "fidelity_validation_status",
        "plant_code_hash",
        "plant_parameter_hash",
        "plant_input_schema_hash",
        "plant_output_schema_hash",
        "fidelity_report_hash",
    ):
        require(
            capability_report.get(key) == capability.get(key),
            f"formal liquid plant capability/binding {key} mismatch",
        )
    require(
        capability.get("cryptographic_trust_anchor") == "NOT_CONFIGURED"
        and capability.get("external_approval_authentication_status") == "NOT_INDEPENDENTLY_AUTHENTICATED",
        "formal liquid plant approval authentication status is not honestly declared",
    )

    _intake_path, intake = _formal_liquid_bound_json(
        capability,
        "formal_intake_request_path",
        "formal_intake_request_hash",
        "formal liquid plant intake request",
    )
    require(
        intake.get("schema_version") == "smpcc-sim-formal-liquid-plant-intake-request-v1"
        and intake.get("request_purpose") == "ASSEMBLE_EXTERNAL_FORMAL_CAPABILITY_ONLY"
        and intake.get("formal") is True
        and intake.get("development_only") is False,
        "formal liquid plant intake request is not a formal external-evidence request",
    )
    release_path, release = _formal_liquid_bound_json(
        capability,
        "formal_release_manifest_path",
        "formal_release_manifest_hash",
        "formal liquid plant release manifest",
    )
    _require_formal_liquid_document(
        release,
        label="formal liquid plant release manifest",
        schema_version=FORMAL_LIQUID_PLANT_RELEASE_SCHEMA_VERSION,
        report_type="SMPCC_SIM_FORMAL_LIQUID_PLANT_RELEASE",
        status="FROZEN",
    )
    require(
        release.get("release_payload_hash") == canonical_hash(
            {key: value for key, value in release.items() if key != "release_payload_hash"}
        ),
        "formal liquid plant release_payload_hash mismatch",
    )
    release_artifacts = require_mapping(release.get("artifacts"), "formal liquid plant release artifacts")
    for key in (
        "plant_code_hash",
        "plant_parameter_hash",
        "plant_input_schema_hash",
        "plant_output_schema_hash",
    ):
        require(
            require_mapping(release_artifacts.get(
                {
                    "plant_code_hash": "plant_code",
                    "plant_parameter_hash": "plant_parameters",
                    "plant_input_schema_hash": "plant_input_schema",
                    "plant_output_schema_hash": "plant_output_schema",
                }[key]), f"formal liquid plant release artifact {key}").get("sha256") == capability.get(key),
            f"formal liquid plant release {key} mismatch",
        )

    isolation_path, isolation = _formal_liquid_bound_json(
        capability,
        "controller_isolation_evidence_path",
        "controller_isolation_evidence_hash",
        "formal controller/plant isolation evidence",
    )
    _require_formal_liquid_document(
        isolation,
        label="formal controller/plant isolation evidence",
        schema_version=FORMAL_LIQUID_PLANT_ISOLATION_SCHEMA_VERSION,
        report_type="SMPCC_SIM_CONTROLLER_PLANT_ISOLATION_EVIDENCE",
        status="PASS",
    )
    require(
        isolation.get("formal_release_manifest_hash") == capability.get("formal_release_manifest_hash"),
        "formal controller/plant isolation evidence release binding mismatch",
    )
    require(
        isolation.get("truth_topic") == "/sim_truth/liquid_height"
        and isolation.get("implementation_isolated_from_controller") is True
        and isolation.get("controller_hidden_state_access") is False
        and isolation.get("controller_state_import") is False
        and isolation.get("controller_subscription_to_truth") is False
        and isolation.get("plant_reads_raw_command") is False,
        "formal controller/plant isolation evidence does not prove the information firewall",
    )
    checkpoints = isolation.get("checkpoints")
    require(
        isinstance(checkpoints, Mapping)
        and checkpoints == {"ready": "PASS", "pre_motion": "PASS", "postflight": "PASS"},
        "formal controller/plant isolation evidence lacks three PASS checkpoints",
    )
    for key in ("plant_code_hash", "plant_input_schema_hash", "plant_output_schema_hash"):
        require(isolation.get(key) == capability.get(key), f"formal isolation {key} mismatch")

    verifier_path = validate_bound_file(
        capability,
        "fidelity_verifier_source_path",
        "fidelity_verifier_source_hash",
        "formal liquid plant fidelity verifier source",
    )
    try:
        verifier_text = verifier_path.read_text(encoding="utf-8").lower()
    except OSError as exc:
        raise ContractError(f"formal liquid plant fidelity verifier cannot be read: {verifier_path}") from exc
    require(
        not any(token in verifier_text for token in ("liquidsloshmodel", "/slosh/height", "/sim_spmpc/slosh_height", "h_proxy", "h_modal")),
        "formal liquid plant fidelity verifier references a forbidden proxy/controller model",
    )
    _require_formal_liquid_document(
        fidelity_report,
        label="formal liquid plant fidelity report",
        schema_version=FORMAL_LIQUID_PLANT_FIDELITY_SCHEMA_VERSION,
        report_type="SMPCC_SIM_LIQUID_PLANT_FIDELITY_VALIDATION",
        status="PASS",
    )
    require(
        fidelity_report.get("independently_produced") is True
        and fidelity_report.get("formal_release_manifest_hash") == capability.get("formal_release_manifest_hash")
        and fidelity_report.get("fidelity_verifier_source_path") == str(verifier_path.resolve())
        and fidelity_report.get("fidelity_verifier_source_hash") == capability.get("fidelity_verifier_source_hash"),
        "formal liquid plant fidelity provenance mismatch",
    )

    reference_entries = capability.get("formal_reference_evidence")
    plant_entries = capability.get("formal_plant_signal_evidence")
    require(
        isinstance(reference_entries, list) and len(reference_entries) >= 2,
        "formal liquid plant requires at least two real-reference evidence cases",
    )
    require(
        isinstance(plant_entries, list) and len(plant_entries) == len(reference_entries),
        "formal liquid plant signal evidence must match real-reference case count",
    )
    require(
        all(isinstance(item, Mapping) for item in reference_entries)
        and all(isinstance(item, Mapping) for item in plant_entries),
        "formal liquid plant evidence entries must be objects",
    )
    require(reference_entries == sorted(reference_entries, key=lambda item: item.get("case_id", "")), "formal real-reference evidence must be sorted by case_id")
    require(plant_entries == sorted(plant_entries, key=lambda item: item.get("case_id", "")), "formal plant-signal evidence must be sorted by case_id")
    require(
        capability.get("formal_reference_evidence_set_hash") == canonical_hash(reference_entries)
        and fidelity_report.get("formal_reference_evidence") == reference_entries
        and fidelity_report.get("formal_reference_evidence_set_hash") == capability.get("formal_reference_evidence_set_hash"),
        "formal real-reference evidence set is not hash-bound to fidelity",
    )
    require(
        capability.get("formal_plant_signal_evidence_set_hash") == canonical_hash(plant_entries)
        and fidelity_report.get("formal_plant_signal_evidence") == plant_entries
        and fidelity_report.get("formal_plant_signal_evidence_set_hash") == capability.get("formal_plant_signal_evidence_set_hash"),
        "formal plant-signal evidence set is not hash-bound to fidelity",
    )
    reference_case_ids: List[str] = []
    for entry in reference_entries:
        entry = require_mapping(entry, "formal real-reference evidence entry")
        require(
            set(entry) == {"case_id", "reference_evidence_path", "reference_evidence_hash", "reference_signal_path", "reference_signal_hash", "reference_kind"},
            "formal real-reference evidence entry fields differ from intake ABI",
        )
        case_id = entry.get("case_id")
        require(isinstance(case_id, str) and case_id and case_id not in reference_case_ids, "formal real-reference case_id is missing or duplicated")
        reference_case_ids.append(case_id)
        require(entry.get("reference_kind") in FORMAL_LIQUID_PLANT_REFERENCE_KINDS, "formal real-reference kind is invalid")
        evidence_path, evidence = _formal_liquid_bound_json(entry, "reference_evidence_path", "reference_evidence_hash", f"formal real-reference evidence {case_id}")
        signal_path = validate_bound_file(entry, "reference_signal_path", "reference_signal_hash", f"formal real-reference signal {case_id}")
        _require_formal_liquid_document(
            evidence,
            label=f"formal real-reference evidence {case_id}",
            schema_version=FORMAL_LIQUID_PLANT_REFERENCE_SCHEMA_VERSION,
            report_type="SMPCC_REAL_LIQUID_REFERENCE_EVIDENCE",
            status="FROZEN",
        )
        require(
            evidence.get("case_id") == case_id
            and evidence.get("reference_kind") == entry.get("reference_kind")
            and evidence.get("real_measurement") is True
            and evidence.get("measurement_independent_of_plant") is True
            and evidence.get("formal_release_manifest_hash") == capability.get("formal_release_manifest_hash")
            and evidence.get("reference_signal_path") == str(signal_path.resolve())
            and evidence.get("reference_signal_hash") == entry.get("reference_signal_hash"),
            f"formal real-reference evidence {case_id} is not bound to a real independent signal",
        )
        source_topic = str(evidence.get("source_topic", "")).lower()
        require(
            source_topic and not any(token in source_topic for token in ("/slosh/height", "/sim_spmpc/slosh_height", "h_proxy", "h_modal", "/sim_truth/")),
            f"formal real-reference evidence {case_id} uses a forbidden proxy/truth topic",
        )
        for path_key, hash_key in (
            ("source_bag_path", "source_bag_hash"),
            ("extraction_pipeline_path", "extraction_pipeline_hash"),
            ("calibration_path", "calibration_hash"),
        ):
            validate_bound_file(evidence, path_key, hash_key, f"formal real-reference {case_id} {path_key}")
        require(evidence_path.is_file(), f"formal real-reference evidence {case_id} disappeared")

    plant_case_ids: List[str] = []
    for entry in plant_entries:
        entry = require_mapping(entry, "formal plant-signal evidence entry")
        require(
            set(entry) == {"case_id", "plant_signal_path", "plant_signal_hash", "plant_run_manifest_path", "plant_run_manifest_hash", "plant_signal_topic"},
            "formal plant-signal evidence entry fields differ from intake ABI",
        )
        case_id = entry.get("case_id")
        require(isinstance(case_id, str) and case_id and case_id not in plant_case_ids, "formal plant-signal case_id is missing or duplicated")
        plant_case_ids.append(case_id)
        require(entry.get("plant_signal_topic") == "/sim_truth/liquid_height", "formal plant signal topic mismatch")
        signal_path = validate_bound_file(entry, "plant_signal_path", "plant_signal_hash", f"formal plant signal {case_id}")
        _run_path, run = _formal_liquid_bound_json(entry, "plant_run_manifest_path", "plant_run_manifest_hash", f"formal plant run evidence {case_id}")
        _require_formal_liquid_document(
            run,
            label=f"formal plant run evidence {case_id}",
            schema_version=FORMAL_LIQUID_PLANT_SIGNAL_SCHEMA_VERSION,
            report_type="SMPCC_SIM_FORMAL_LIQUID_PLANT_SIGNAL_EVIDENCE",
            status="PASS",
        )
        require(
            run.get("case_id") == case_id
            and run.get("truth_topic") == "/sim_truth/liquid_height"
            and run.get("formal_release_manifest_hash") == capability.get("formal_release_manifest_hash")
            and run.get("plant_signal_path") == str(signal_path.resolve())
            and run.get("plant_signal_hash") == entry.get("plant_signal_hash"),
            f"formal plant run evidence {case_id} is not bound to the supplied plant signal",
        )
        for key in ("plant_code_hash", "plant_parameter_hash", "plant_input_schema_hash", "plant_output_schema_hash"):
            require(run.get(key) == capability.get(key), f"formal plant run evidence {case_id} {key} mismatch")
    require(plant_case_ids == reference_case_ids, "formal plant-signal cases differ from real-reference cases")

    approval_path, approval = _formal_liquid_bound_json(
        capability,
        "external_approval_path",
        "external_approval_hash",
        "external formal liquid-plant approval",
    )
    _require_formal_liquid_document(
        approval,
        label="external formal liquid-plant approval",
        schema_version=FORMAL_LIQUID_PLANT_APPROVAL_SCHEMA_VERSION,
        report_type="SMPCC_SIM_FORMAL_LIQUID_PLANT_APPROVAL",
        status="APPROVED",
    )
    require(
        approval.get("external_approval") is True
        and approval.get("approval_scope") == "SMPCC_SIM_FORMAL_PHYSICAL_PRIMARY"
        and approval.get("approval_id") == capability.get("external_approval_id")
        and approval.get("approval_authority") == capability.get("external_approval_authority"),
        "external formal liquid-plant approval identity/scope mismatch",
    )
    for key, expected in (
        ("formal_release_manifest_hash", capability.get("formal_release_manifest_hash")),
        ("fidelity_report_hash", capability.get("fidelity_report_hash")),
        ("controller_isolation_evidence_hash", capability.get("controller_isolation_evidence_hash")),
        ("formal_reference_evidence_set_hash", capability.get("formal_reference_evidence_set_hash")),
        ("formal_plant_signal_evidence_set_hash", capability.get("formal_plant_signal_evidence_set_hash")),
    ):
        require(approval.get(key) == expected, f"external formal liquid-plant approval {key} mismatch")
    for key in ("plant_code_hash", "plant_parameter_hash", "plant_input_schema_hash", "plant_output_schema_hash"):
        require(approval.get(key) == capability.get(key), f"external formal liquid-plant approval {key} mismatch")
    dimensions = approval.get("validation_dimensions")
    require(
        isinstance(dimensions, Mapping)
        and set(dimensions) == {"amplitude", "frequency", "damping", "phase", "ranking"}
        and all(dimensions[key] == "PASS" for key in dimensions),
        "external formal liquid-plant approval must attest all five fidelity dimensions PASS",
    )
    require(approval_path.is_file() and release_path.is_file() and isolation_path.is_file(), "formal liquid-plant provenance artifact disappeared")


def validate_formal_liquid_plant_capability(capability: Any) -> Dict[str, Any]:
    """Require structured, non-development evidence before formal primary use.

    ``validate_truth_capability`` deliberately accepts a small declaration so
    development dry-runs can prove their routing boundary.  Formal primary
    evidence needs more: a hash-bound capability report and fidelity report
    that both explicitly reject development-only status and bind the exact
    plant code, parameters and I/O schemas.  This prevents a development
    `/sim_truth` surrogate from becoming formal merely by copying hashes into
    a freeze JSON.
    """
    generic = validate_truth_capability(capability)
    if not isinstance(capability, Mapping):
        return generic
    errors = list(generic["errors"])
    report_path: Optional[Path] = None
    report: Optional[Mapping[str, Any]] = None
    try:
        report_path = validate_bound_file(
            capability,
            "plant_capability_report_path",
            "plant_capability_report_hash",
            "formal liquid plant capability report",
        )
        report = require_mapping(read_json(report_path), "formal liquid plant capability report")
        require(
            report.get("report_type") == "SMPCC_SIM_INDEPENDENT_LIQUID_PLANT_CAPABILITY",
            "formal liquid plant capability report has wrong report_type",
        )
        require(report.get("status") == "PASS", "formal liquid plant capability report is not PASS")
        require(report.get("formal") is True, "formal liquid plant capability report must set formal=true")
        require(report.get("development_only") is False, "formal liquid plant capability report must set development_only=false")
        require(
            report.get("physical_primary_eligible") is True,
            "formal liquid plant capability report must set physical_primary_eligible=true",
        )
        for key, expected in (
            ("independent_plant", True),
            ("implementation_isolated_from_controller", True),
            ("controller_hidden_state_access", False),
            ("driven_by", "executed_simulated_base_motion"),
            ("truth_topic", "/sim_truth/liquid_height"),
            ("fidelity_validation_status", "PASS"),
        ):
            require(report.get(key) == expected, f"formal liquid plant capability report {key} mismatch")
            require(capability.get(key) == report.get(key), f"formal liquid plant capability/report {key} mismatch")
        for key in (
            "plant_code_hash",
            "plant_parameter_hash",
            "plant_input_schema_hash",
            "plant_output_schema_hash",
            "fidelity_report_hash",
        ):
            require(report.get(key) == capability.get(key), f"formal liquid plant capability/report {key} mismatch")
    except ContractError as exc:
        errors.append(str(exc))

    fidelity_path: Optional[Path] = None
    fidelity_report: Optional[Mapping[str, Any]] = None
    try:
        fidelity_path = validate_bound_file(
            capability,
            "fidelity_report_path",
            "fidelity_report_hash",
            "formal liquid plant fidelity report",
        )
        fidelity_report = require_mapping(read_json(fidelity_path), "formal liquid plant fidelity report")
        require(
            fidelity_report.get("report_type") == "SMPCC_SIM_LIQUID_PLANT_FIDELITY_VALIDATION",
            "formal liquid plant fidelity report has wrong report_type",
        )
        require(fidelity_report.get("status") == "PASS", "formal liquid plant fidelity report is not PASS")
        require(fidelity_report.get("formal") is True, "formal liquid plant fidelity report must set formal=true")
        require(fidelity_report.get("development_only") is False, "formal liquid plant fidelity report must set development_only=false")
        require(
            fidelity_report.get("fidelity_validation_status") == "PASS",
            "formal liquid plant fidelity report lacks PASS fidelity_validation_status",
        )
        require(
            fidelity_report.get("truth_topic") == "/sim_truth/liquid_height",
            "formal liquid plant fidelity report truth_topic mismatch",
        )
        dimensions = fidelity_report.get("validation_dimensions")
        require(
            isinstance(dimensions, Mapping)
            and set(dimensions) == {"amplitude", "frequency", "damping", "phase", "ranking"}
            and all(dimensions[key] == "PASS" for key in dimensions),
            "formal liquid plant fidelity report must PASS amplitude/frequency/damping/phase/ranking",
        )
        for key in (
            "plant_code_hash",
            "plant_parameter_hash",
            "plant_input_schema_hash",
            "plant_output_schema_hash",
        ):
            require(fidelity_report.get(key) == capability.get(key), f"formal liquid plant fidelity/report {key} mismatch")
    except ContractError as exc:
        errors.append(str(exc))

    # A generic capability/fidelity pair with the right PASS strings is not
    # enough.  Require the formal intake chain only after both primary
    # documents were successfully hash-bound, so a missing file is reported
    # clearly without cascading attribute errors.
    if report_path is not None and report is not None and fidelity_report is not None:
        try:
            _validate_formal_liquid_plant_intake_provenance(
                capability,
                report_path,
                report,
                fidelity_report,
            )
        except ContractError as exc:
            errors.append(str(exc))

    return {
        "eligible": not errors,
        "status": "PASS" if not errors else "NO_INDEPENDENT_PLANT",
        "errors": errors,
        "truth_topic": capability.get("truth_topic"),
        "capability_report_path": None if report_path is None else str(report_path),
        "fidelity_report_path": None if fidelity_path is None else str(fidelity_path),
    }


def validate_formal_bslosh_release(
    entry: Any,
    freeze: Mapping[str, Any],
    bslosh_config: Optional[Mapping[str, Any]] = None,
) -> Dict[str, Any]:
    """Parse, not merely hash-bind, the selected source-specific release."""
    entry = require_mapping(entry, "formal Bslosh release")
    require(entry.get("status") == "FORMAL_SELECTED", "formal Bslosh is not uniquely selected")
    require(isinstance(entry.get("release_id"), str) and entry["release_id"], "formal Bslosh release_id is missing")
    for key in ("effective_config_hash", "observer_policy_hash", "delay_policy_hash"):
        require(is_sha256(entry.get(key)), f"formal Bslosh {key} must be a SHA-256")
    path = validate_bound_file(entry, "release_manifest_path", "release_manifest_hash", "formal Bslosh release")
    document = require_mapping(read_json(path), "formal Bslosh release manifest")
    sim_only_markers = find_sim_only_release_markers(document)
    require(
        not sim_only_markers,
        "SIM-ONLY Bslosh release cannot be used by the real-aligned 40/64/88 protocol: "
        + ", ".join(sim_only_markers[:4]),
    )
    require(not has_forbidden_w5(document), "formal Bslosh release manifest revives rejected W5_S10")
    require(document.get("report_type") == "SMPCC_FORMAL_BSLOSH_RELEASE", "formal Bslosh release manifest has wrong report_type")
    require(document.get("status") == "FORMAL_SELECTED", "formal Bslosh release manifest is not FORMAL_SELECTED")
    require(document.get("protocol_id") == FORMAL_PROTOCOL_ID, "formal Bslosh release manifest protocol_id mismatch")
    require(document.get("release_id") == entry.get("release_id"), "formal Bslosh release_id differs from release manifest")
    require(document.get("condition_id", document.get("selected_condition_id")) == "Bslosh", "formal Bslosh release manifest condition is not Bslosh")
    for key in ("real_freeze_id", "sim_freeze_id", "git_revision", "build_id"):
        require(document.get(key) == freeze.get(key), f"formal Bslosh release manifest {key} differs from freeze")
    for key in ("effective_config_hash", "observer_policy_hash", "delay_policy_hash"):
        require(document.get(key) == entry.get(key), f"formal Bslosh release manifest {key} differs from release declaration")
    require(document.get("source_specific_release") is True, "formal Bslosh release must be source-specific")
    require(isinstance(document.get("source_specific_release_revision"), str) and document["source_specific_release_revision"], "formal Bslosh release lacks source_specific_release_revision")
    for key in (
        "source_selection_report_hash",
        "final_candidate_report_hash",
        "efficacy_report_hash",
        "trajectory_replay_report_hash",
        "comparator_fairness_report_hash",
        "fallback_policy_hash",
        "formal_release_validator_hash",
    ):
        require(is_sha256(document.get(key)), f"formal Bslosh release manifest {key} must be a SHA-256")
    if bslosh_config is not None:
        require(document.get("effective_config_hash") == canonical_hash(dict(bslosh_config)), "formal Bslosh release config does not match frozen Bslosh config")
        require(document.get("observer_policy_hash") == canonical_hash(dict(require_mapping(bslosh_config.get("observer"), "Bslosh observer"))), "formal Bslosh observer policy differs from frozen config")
        require(document.get("delay_policy_hash") == canonical_hash(dict(require_mapping(bslosh_config.get("delay"), "Bslosh delay"))), "formal Bslosh delay policy differs from frozen config")
    return {
        "status": "PASS",
        "release_id": entry["release_id"],
        "release_manifest_path": str(path.resolve()),
        "release_manifest_hash": sha256_file(path),
    }


def validate_formal_container_entry(entry: Any, container_id: str) -> Dict[str, Any]:
    """Require C1/C2 manifests and make C2's allowed-transfer diff explicit."""
    entry = require_mapping(entry, f"formal container {container_id}")
    parameter_path = validate_bound_file(entry, "physical_parameter_file", "physical_parameter_hash", f"formal container {container_id}")
    manifest_path = validate_bound_file(entry, "container_manifest_path", "container_manifest_hash", f"formal container {container_id} manifest")
    manifest = require_mapping(read_json(manifest_path), f"formal container {container_id} manifest")
    require(manifest.get("report_type") == "SMPCC_SIM_CONTAINER_MANIFEST", f"formal container {container_id} manifest has wrong report_type")
    require(manifest.get("status") == "FROZEN", f"formal container {container_id} manifest is not FROZEN")
    require(manifest.get("container_id") == container_id, f"formal container {container_id} manifest identity mismatch")
    require(manifest.get("physical_parameter_hash") == entry.get("physical_parameter_hash"), f"formal container {container_id} manifest parameter hash mismatch")
    if container_id == "C2":
        diff_path = validate_bound_file(entry, "allowed_transfer_diff_path", "allowed_transfer_diff_hash", "formal C2 allowed-transfer diff")
        diff = require_mapping(read_json(diff_path), "formal C2 allowed-transfer diff")
        require(diff.get("report_type") == "SMPCC_SIM_C2_ALLOWED_TRANSFER_DIFF" and diff.get("status") == "PASS", "formal C2 allowed-transfer diff is not PASS")
        require(diff.get("from_container_id") == "C1" and diff.get("to_container_id") == "C2", "formal C2 allowed-transfer diff has wrong containers")
        allowed = diff.get("allowed_changed_fields")
        require(isinstance(allowed, list) and allowed and all(isinstance(item, str) and item for item in allowed), "formal C2 allowed-transfer fields are missing")
        for key in ("control_config_changes", "observer_changes", "delay_changes", "path_changes"):
            require(diff.get(key) == [], f"formal C2 allowed-transfer diff contains forbidden {key}")
        require(diff.get("c1_physical_parameter_hash") == manifest.get("c1_physical_parameter_hash"), "formal C2 manifest/diff C1 binding mismatch")
        require(diff.get("c2_physical_parameter_hash") == entry.get("physical_parameter_hash"), "formal C2 manifest/diff C2 binding mismatch")
    return {
        "status": "PASS",
        "container_id": container_id,
        "physical_parameter_hash": entry["physical_parameter_hash"],
        "container_manifest_path": str(manifest_path.resolve()),
    }


def validate_formal_freeze(freeze: Any) -> Dict[str, Any]:
    """The only entry to formal planned-row generation; missing data is fatal."""
    errors: List[str] = []
    if not isinstance(freeze, Mapping):
        return {"status": "FAIL", "errors": ["formal freeze must be a JSON object"]}
    if freeze.get("fixture") is True or freeze.get("mode") == "fixture":
        errors.append("fixture input can never be a formal freeze")
    if freeze.get("protocol_id") != FORMAL_PROTOCOL_ID:
        errors.append(f"protocol_id must equal {FORMAL_PROTOCOL_ID}")
    sim_only_markers = find_sim_only_release_markers(freeze)
    if sim_only_markers:
        errors.append(
            "SIM-ONLY release is isolated from the real-aligned 40/64/88 protocol: "
            + ", ".join(sim_only_markers[:4])
        )
    if freeze.get("sample_size") != 8:
        errors.append("formal 40/64/88 contract requires frozen sample_size=8")
    for key in ("real_freeze_id", "sim_freeze_id", "git_revision", "build_id"):
        if not isinstance(freeze.get(key), str) or not freeze.get(key).strip():
            errors.append(f"formal input missing {key}")
    if has_forbidden_w5(freeze):
        errors.append("W5_S10 is REJECT_FOR_FORMAL_STAGE and cannot appear in formal input")

    bslosh = freeze.get("formal_bslosh_release")
    if not isinstance(bslosh, Mapping):
        errors.append("formal_bslosh_release is missing")
    else:
        # The manifest content is checked below after effective configs have
        # been loaded, so its observer/delay hashes can be cross-bound too.
        for key in ("release_id", "effective_config_hash", "observer_policy_hash", "delay_policy_hash"):
            if not bslosh.get(key):
                errors.append(f"formal Bslosh {key} is missing")

    paths = freeze.get("paths")
    if not isinstance(paths, Mapping):
        errors.append("formal H1/L1 frozen JSON path registry is missing")
    else:
        for path_id in ("H1", "L1"):
            item = paths.get(path_id)
            if not isinstance(item, Mapping):
                errors.append(f"formal path {path_id} is missing")
                continue
            if item.get("source_mode") != "frozen_json_replay":
                errors.append(f"formal path {path_id} must use frozen_json_replay, never runtime s_curve")
            for file_key, hash_key in (
                ("source_path", "source_path_hash"),
                ("sim_path", "sim_path_hash"),
                ("transform_path", "transform_hash"),
                ("fit_clearance_report_path", "fit_clearance_report_hash"),
            ):
                try:
                    validate_bound_file(item, file_key, hash_key, f"formal path {path_id}")
                except ContractError as exc:
                    errors.append(str(exc))

    containers = freeze.get("containers")
    if not isinstance(containers, Mapping):
        errors.append("formal C1/C2 manifest registry is missing")
    else:
        for container_id in ("C1", "C2"):
            item = containers.get(container_id)
            if not isinstance(item, Mapping):
                errors.append(f"formal container {container_id} is missing")
                continue
            try:
                validate_formal_container_entry(item, container_id)
            except ContractError as exc:
                errors.append(str(exc))
        if isinstance(containers.get("C1"), Mapping) and isinstance(containers.get("C2"), Mapping):
            try:
                c1_hash = containers["C1"].get("physical_parameter_hash")
                c2 = containers["C2"]
                diff_path = validate_bound_file(c2, "allowed_transfer_diff_path", "allowed_transfer_diff_hash", "formal C2 allowed-transfer diff")
                diff = require_mapping(read_json(diff_path), "formal C2 allowed-transfer diff")
                manifest_path = validate_bound_file(c2, "container_manifest_path", "container_manifest_hash", "formal C2 manifest")
                manifest = require_mapping(read_json(manifest_path), "formal C2 manifest")
                require(diff.get("c1_physical_parameter_hash") == c1_hash, "formal C2 allowed-transfer diff is not bound to frozen C1 parameters")
                require(manifest.get("c1_physical_parameter_hash") == c1_hash, "formal C2 manifest is not bound to frozen C1 parameters")
            except ContractError as exc:
                errors.append(str(exc))

    configs = freeze.get("effective_configs")
    if not isinstance(configs, Mapping):
        errors.append("complete effective_configs registry is missing")
    else:
        for condition in CONDITION_BACKENDS:
            item = configs.get(condition)
            if not isinstance(item, Mapping) or not is_sha256(item.get("effective_config_hash")):
                errors.append(f"effective config/hash missing for {condition}")
                continue
            try:
                path = validate_bound_file(item, "effective_config_path", "effective_config_file_hash", f"effective config {condition}")
                from_file = read_json(path)
                require(item.get("effective_config") == from_file, f"effective config {condition} embedded/file mismatch")
                config_report = validate_effective_config(from_file, condition)
                require(config_report["effective_config_hash"] == item.get("effective_config_hash"), f"effective config {condition} canonical hash mismatch")
            except ContractError as exc:
                errors.append(str(exc))
        try:
            if isinstance(configs.get("Bsmooth"), Mapping) and isinstance(configs.get("SmoothMatch"), Mapping):
                compare_smoothmatch(
                    configs["Bsmooth"].get("effective_config"),
                    configs["SmoothMatch"].get("effective_config"),
                )
        except ContractError as exc:
            errors.append(str(exc))
    real_parameter_alignment = freeze.get("real_parameter_alignment")
    if not isinstance(real_parameter_alignment, Mapping):
        errors.append("formal real parameter alignment evidence is missing")
    elif isinstance(configs, Mapping):
        try:
            validate_formal_real_parameter_alignment(
                real_parameter_alignment,
                freeze,
                configs,
            )
        except ContractError as exc:
            errors.append(str(exc))
    if isinstance(bslosh, Mapping):
        try:
            bslosh_config: Optional[Mapping[str, Any]] = None
            if isinstance(configs, Mapping) and isinstance(configs.get("Bslosh"), Mapping):
                bslosh_config = require_mapping(configs["Bslosh"].get("effective_config"), "frozen Bslosh effective config")
            validate_formal_bslosh_release(bslosh, freeze, bslosh_config)
        except ContractError as exc:
            errors.append(str(exc))

    profiles = freeze.get("fixed_profiles")
    if not isinstance(profiles, Mapping):
        errors.append("pre-generated FixedProfile registry is missing")
    else:
        for stratum in ("H1_C1", "L1_C1", "H1_C2"):
            item = profiles.get(stratum)
            if not isinstance(item, Mapping):
                errors.append(f"pre-generated FixedProfile missing for {stratum}")
                continue
            try:
                validate_fixed_profile(item, require_file=True)
            except ContractError as exc:
                errors.append(f"FixedProfile {stratum}: {exc}")

    seed_policy = freeze.get("seed_policy")
    if not isinstance(seed_policy, Mapping) or seed_policy.get("independent_sub_seeds") is not True or seed_policy.get("time_indexed_traces") is not True:
        errors.append("formal seed policy requires independent sub-seeds and time-indexed traces")
    recording_policy = freeze.get("recording_policy")
    if not isinstance(recording_policy, Mapping):
        errors.append("formal recording/timeout/tail policy is missing")
    else:
        try:
            require(recording_policy.get("settle_sec") == 30.0, "formal settle_sec must be frozen at 30.0")
            require(recording_policy.get("goal_timeout_sec") == 60.0, "formal GOAL_REACHED timeout must be frozen at 60.0")
            require(isinstance(recording_policy.get("tail_sec"), (int, float)) and float(recording_policy["tail_sec"]) > 0.0, "formal tail_sec must be positive")
            require(recording_policy.get("post_shutdown_sec") == 30.0, "formal post_shutdown_sec must be frozen at 30.0")
            recorder_ready_timeout = recording_policy.get("recorder_ready_timeout_sec")
            require(
                isinstance(recorder_ready_timeout, (int, float))
                and math.isfinite(float(recorder_ready_timeout))
                and float(recorder_ready_timeout) > 0.0,
                "formal recorder_ready_timeout_sec must be positive and finite",
            )
            require(is_sha256(recording_policy.get("outcome_window_rule_hash")), "formal recording policy lacks outcome window rule hash")
            for key in ("first_effective_motion_rule_hash", "arrival_rule_hash", "tail_end_rule_hash", "recorder_budget_rule_hash"):
                require(is_sha256(recording_policy.get(key)), f"formal recording policy lacks {key}")
            recorder_budget = recording_policy.get("recorder_budget_sec")
            require(isinstance(recorder_budget, (int, float)) and math.isfinite(float(recorder_budget)), "formal recorder_budget_sec must be finite")
            require(float(recorder_budget) >= 30.0 + 60.0 + float(recording_policy["tail_sec"]), "formal recorder budget cannot omit settle/motion/tail")
            for key in ("recorder_before_motion_required", "goal_tail_required", "timeout_tail_required", "motion_release_ack_required", "motion_stop_ack_required"):
                require(recording_policy.get(key) is True, f"formal recording policy requires {key}=true")
        except ContractError as exc:
            errors.append(str(exc))
    retry_classifier = freeze.get("retry_classifier")
    if not isinstance(retry_classifier, Mapping):
        errors.append("formal frozen retry classifier is missing")
    else:
        try:
            validate_frozen_retry_classifier(retry_classifier)
        except ContractError as exc:
            errors.append(str(exc))
    contrast_registry = freeze.get("contrast_registry")
    if not isinstance(contrast_registry, Mapping):
        errors.append("formal frozen contrast registry is missing")
    else:
        try:
            validate_frozen_contrast_registry(contrast_registry)
        except ContractError as exc:
            errors.append(str(exc))
    stage_entry_policy = freeze.get("stage_entry_policy")
    if not isinstance(stage_entry_policy, Mapping):
        errors.append("formal frozen stage-entry policy is missing")
    else:
        try:
            validate_frozen_stage_entry_policy(stage_entry_policy)
        except ContractError as exc:
            errors.append(str(exc))
    runtime_launch_contract_report: Optional[Mapping[str, Any]] = None
    runtime_launch_contract = freeze.get("runtime_launch_contract")
    if not isinstance(runtime_launch_contract, Mapping):
        errors.append("formal frozen runtime launch contract is missing")
    else:
        try:
            runtime_launch_contract_report = validate_frozen_runtime_launch_contract(runtime_launch_contract)
        except ContractError as exc:
            errors.append(str(exc))
    runtime_backend = freeze.get("formal_runtime_backend")
    if not isinstance(runtime_backend, Mapping):
        errors.append("formal frozen runtime backend manifest is missing")
    elif runtime_launch_contract_report is not None:
        try:
            validate_formal_runtime_backend_manifest(runtime_backend, freeze, runtime_launch_contract_report)
        except ContractError as exc:
            errors.append(str(exc))
    dataset_ledger = freeze.get("dataset_ledger")
    if not isinstance(dataset_ledger, Mapping):
        errors.append("formal frozen dataset ledger is missing")
    else:
        try:
            validate_formal_dataset_ledger(dataset_ledger, freeze)
        except ContractError as exc:
            errors.append(str(exc))
    frozen_tables = freeze.get("randomization_tables")
    frozen_bundles = freeze.get("seed_bundles")
    if not isinstance(frozen_tables, Mapping):
        errors.append("formal randomization tables are missing; do not generate them at runtime")
    else:
        for stage in STAGES:
            try:
                validate_randomization_table(frozen_tables.get(stage), stage, blocks=8)
            except ContractError as exc:
                errors.append(str(exc))
    if not isinstance(frozen_bundles, Mapping):
        errors.append("formal seed bundles are missing; do not derive them at runtime")
    else:
        for stage in STAGES:
            for block in range(1, 9):
                key = f"{stage}:b{block:02d}"
                bundle = frozen_bundles.get(key)
                try:
                    report = validate_seed_bundle(bundle)
                    require(bundle.get("seed_bundle_id") == f"SEED_{stage}_b{block:02d}", f"{key} seed bundle ID mismatch")
                    require(bundle.get("stage") == stage and bundle.get("block_id") == f"b{block:02d}", f"{key} seed bundle identity mismatch")
                    if isinstance(frozen_tables, Mapping) and isinstance(frozen_tables.get(stage), Mapping):
                        # Every condition in this complete block consumes the same immutable bundle.
                        require(report["seed_bundle_hash"] == bundle.get("seed_bundle_hash"), f"{key} seed bundle hash mismatch")
                except ContractError as exc:
                    errors.append(str(exc))

    simulator = freeze.get("simulator_assets")
    world_document: Optional[Mapping[str, Any]] = None
    if not isinstance(simulator, Mapping):
        errors.append("formal simulator map/world/robot asset registry is missing")
    else:
        try:
            simulator_report = validate_formal_simulator_assets(simulator)
            world_document = require_mapping(simulator_report["world_geometry"], "formal world clearance geometry")
        except ContractError as exc:
            errors.append(str(exc))
    if isinstance(paths, Mapping) and world_document is not None:
        for path_id in ("H1", "L1"):
            item = paths.get(path_id)
            if not isinstance(item, Mapping):
                continue
            try:
                transform = require_mapping(read_json(Path(str(item.get("transform_path")))), f"formal {path_id} transform")
                report = validate_path_replay(
                    Path(str(item.get("source_path"))),
                    Path(str(item.get("sim_path"))),
                    transform,
                    world_document,
                    float(item.get("clearance_m")),
                )
                require(report["source_path_hash"] == item.get("source_path_hash") and report["sim_path_hash"] == item.get("sim_path_hash"), f"formal {path_id} replay hash mismatch")
            except (ContractError, TypeError, ValueError) as exc:
                errors.append(f"formal {path_id} rigid-transform/fit/clearance validation failed: {exc}")

    truth_capability = freeze.get("liquid_plant_capability")
    truth = validate_formal_liquid_plant_capability(truth_capability)
    if not truth["eligible"]:
        errors.extend("liquid truth: " + item for item in truth["errors"])
    if isinstance(truth_capability, Mapping):
        for file_key, hash_key in (
            ("plant_code_path", "plant_code_hash"),
            ("plant_parameter_path", "plant_parameter_hash"),
            ("plant_input_schema_path", "plant_input_schema_hash"),
            ("plant_output_schema_path", "plant_output_schema_hash"),
            ("fidelity_report_path", "fidelity_report_hash"),
        ):
            try:
                validate_bound_file(truth_capability, file_key, hash_key, "liquid plant capability")
            except ContractError as exc:
                errors.append(str(exc))
    firewall = freeze.get("controller_firewall")
    if not isinstance(firewall, Mapping):
        errors.append("formal controller subscriber firewall report is missing")
    else:
        try:
            firewall_path = validate_bound_file(firewall, "report_path", "report_hash", "formal controller firewall")
            firewall_report = require_mapping(read_json(firewall_path), "formal controller firewall report")
            frozen_nodes = firewall.get("controller_nodes")
            require(isinstance(frozen_nodes, list) and frozen_nodes and all(isinstance(node, str) and node.startswith("/") for node in frozen_nodes), "formal controller firewall must freeze controller/planner/tracker/cmd-gate nodes")
            require(firewall_report.get("controller_nodes") == sorted(frozen_nodes), "formal controller firewall report node set mismatch")
            recomputed = validate_controller_firewall(require_mapping(firewall_report.get("graph"), "formal controller firewall graph"), frozen_nodes)
            require(recomputed["status"] == "PASS" and firewall_report.get("status") == "PASS", "formal controller firewall report is not PASS")
            require(firewall_report.get("graph_hash") == recomputed["graph_hash"], "formal controller firewall report graph hash mismatch")
        except ContractError as exc:
            errors.append(str(exc))

    receipt = freeze.get("formal_freeze_receipt")
    if not isinstance(receipt, Mapping):
        errors.append("external formal freeze receipt is missing")
    else:
        try:
            receipt_path = validate_bound_file(receipt, "report_path", "report_hash", "formal freeze receipt")
            receipt_document = require_mapping(read_json(receipt_path), "formal freeze receipt")
            payload = dict(freeze)
            payload.pop("formal_freeze_receipt", None)
            require(receipt_document.get("report_type") == "SMPCC_SIM_FORMAL_FREEZE_RECEIPT", "formal freeze receipt has wrong type")
            require(receipt_document.get("status") == "PASS", "formal freeze receipt is not PASS")
            require(receipt_document.get("freeze_id") == freeze.get("sim_freeze_id"), "formal freeze receipt freeze_id mismatch")
            require(receipt_document.get("freeze_payload_hash") == canonical_hash(payload), "formal freeze receipt does not bind the immutable freeze payload")
            require(is_sha256(receipt_document.get("validator_hash")), "formal freeze receipt lacks validator hash")
        except ContractError as exc:
            errors.append(str(exc))

    return {
        "schema_version": SCHEMA_VERSION,
        "formal": True,
        "status": "PASS" if not errors else "FAIL",
        "errors": errors,
        "truth_capability": truth,
        "freeze_hash": canonical_hash(freeze),
    }


def validate_effective_config(config: Any, expected_condition: Optional[str] = None) -> Dict[str, Any]:
    require_mapping(config, "effective config")
    config = dict(config)
    if expected_condition is not None:
        require(config.get("condition_id") == expected_condition, f"effective config condition_id must be {expected_condition}")
    missing = [field for field in REQUIRED_EFFECTIVE_CONFIG_FIELDS if field not in config]
    require(not missing, "effective config missing required fields: " + ", ".join(missing))
    require(isinstance(config["observer"], Mapping), "effective config observer must be an object")
    require(isinstance(config["delay"], Mapping), "effective config delay must be an object")
    for field in REQUIRED_EFFECTIVE_CONFIG_FIELDS[:-2]:
        require(isinstance(config[field], (int, float)) and math.isfinite(float(config[field])), f"effective config {field} must be finite")
    return {"effective_config_hash": canonical_hash(config), "covered_fields": list(REQUIRED_EFFECTIVE_CONFIG_FIELDS)}


def _read_only_bound_json(
    owner: Mapping[str, Any],
    path_key: str,
    hash_key: str,
    label: str,
) -> Tuple[Path, Mapping[str, Any]]:
    """Open one immutable JSON evidence artifact after binding its file hash.

    A formal real-parameter alignment is useful only if its real-side source
    cannot be edited after the simulated freeze has accepted it.  Requiring
    absolute, read-only inputs here also makes a mutable development markdown
    or an H0 manifest fail before it can look like a real release.
    """
    path = validate_bound_file(owner, path_key, hash_key, label)
    require(path.is_absolute(), f"{label} path must be absolute")
    mode = stat.S_IMODE(path.stat().st_mode)
    require(mode & (stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH) == 0, f"{label} must be read-only")
    document = require_mapping(read_json(path), label)
    require(sha256_file(path) == owner.get(hash_key), f"{label} changed while being read")
    return path.resolve(), document


def effective_config_control_projection(config: Any, condition_id: str) -> Dict[str, Any]:
    """Return exactly the control fields that must agree across real/sim.

    File hashes and complete configs remain in the receipt for provenance, but
    this normalized projection is the actual alignment decision: all listed
    costs, ``v_ref``, and the complete observer/delay objects must be equal.
    World, plant, sensor, and recorder differences are deliberately handled
    by their own frozen artifacts; they cannot be smuggled into a control
    exception list here.
    """
    mapping = dict(require_mapping(config, f"{condition_id} effective config"))
    validate_effective_config(mapping, condition_id)
    return {field: mapping[field] for field in REQUIRED_EFFECTIVE_CONFIG_FIELDS}


def validate_formal_real_parameter_alignment(
    entry: Any,
    freeze: Mapping[str, Any],
    configs: Mapping[str, Any],
) -> Dict[str, Any]:
    """Require a real-freeze receipt that exactly aligns five control configs.

    The real startup document intentionally leaves the final values unresolved;
    this validator does not fill them from a legacy G2S/G2C/G3 template.  It
    only accepts a future read-only receipt backed by a real formal freeze and
    by five normalized real effective-config snapshots.  Any drift in
    ``w_control``, ``w_smooth``, ``w_alpha``, ``w_du_a``, ``w_du_vs``,
    ``w_slosh``, ``v_ref``, observer or delay fails closed.
    """
    alignment_entry = require_mapping(entry, "formal real parameter alignment")
    alignment_path, document = _read_only_bound_json(
        alignment_entry,
        "report_path",
        "report_hash",
        "formal real parameter alignment",
    )
    require(
        document.get("schema_version") == FORMAL_REAL_PARAMETER_ALIGNMENT_SCHEMA_VERSION,
        "formal real parameter alignment schema_version mismatch",
    )
    require(
        document.get("document_type") == FORMAL_REAL_PARAMETER_ALIGNMENT_DOCUMENT_TYPE,
        "formal real parameter alignment document_type mismatch",
    )
    require(document.get("status") == "PASS", "formal real parameter alignment is not PASS")
    require(
        document.get("formal") is True and document.get("development_only") is False,
        "formal real parameter alignment must be formal and not development_only",
    )
    require(
        document.get("real_protocol_id") == REAL_FORMAL_PROTOCOL_ID,
        "formal real parameter alignment real_protocol_id mismatch",
    )
    require(
        document.get("real_freeze_id") == freeze.get("real_freeze_id"),
        "formal real parameter alignment real_freeze_id differs from freeze",
    )
    require(
        document.get("sim_freeze_id") == freeze.get("sim_freeze_id"),
        "formal real parameter alignment sim_freeze_id differs from freeze",
    )
    require(
        isinstance(document.get("alignment_id"), str) and document["alignment_id"],
        "formal real parameter alignment alignment_id is missing",
    )
    require(
        is_sha256(document.get("alignment_rule_hash")),
        "formal real parameter alignment alignment_rule_hash must be a SHA-256",
    )
    require(not has_forbidden_w5(document), "formal real parameter alignment revives rejected W5_S10")

    real_freeze_path, real_freeze = _read_only_bound_json(
        document,
        "real_freeze_manifest_path",
        "real_freeze_manifest_hash",
        "formal real freeze manifest",
    )
    require(
        real_freeze.get("protocol_id") == REAL_FORMAL_PROTOCOL_ID,
        "formal real freeze manifest protocol_id mismatch",
    )
    require(
        real_freeze.get("freeze_id", real_freeze.get("real_freeze_id")) == freeze.get("real_freeze_id"),
        "formal real freeze manifest freeze_id differs from formal simulation freeze",
    )
    require(
        real_freeze.get("formal") is True and real_freeze.get("development_only") is False,
        "formal real freeze manifest must be formal and not development_only",
    )
    require(real_freeze.get("status") == "PASS", "formal real freeze manifest is not PASS")
    require(not has_forbidden_w5(real_freeze), "formal real freeze manifest revives rejected W5_S10")

    rows = document.get("conditions")
    require(isinstance(rows, Mapping), "formal real parameter alignment conditions must be an object")
    require(
        set(rows) == set(CONDITION_BACKENDS),
        "formal real parameter alignment must bind exactly the five condition IDs",
    )
    matched: Dict[str, Dict[str, Any]] = {}
    for condition_id in CONDITION_BACKENDS:
        row = require_mapping(rows.get(condition_id), f"formal real parameter alignment {condition_id}")
        require(row.get("status") == "PASS", f"formal real parameter alignment {condition_id} is not PASS")
        real_config_path, real_config = _read_only_bound_json(
            row,
            "real_effective_config_path",
            "real_effective_config_file_hash",
            f"formal real effective config {condition_id}",
        )
        real_config_report = validate_effective_config(real_config, condition_id)
        require(
            row.get("real_effective_config_hash") == real_config_report["effective_config_hash"],
            f"formal real effective config {condition_id} canonical hash mismatch",
        )
        sim_entry = require_mapping(configs.get(condition_id), f"formal simulated effective config {condition_id}")
        sim_config = require_mapping(sim_entry.get("effective_config"), f"formal simulated effective config {condition_id}")
        sim_report = validate_effective_config(sim_config, condition_id)
        require(
            sim_entry.get("effective_config_hash") == sim_report["effective_config_hash"],
            f"formal simulated effective config {condition_id} canonical hash mismatch during real alignment",
        )
        require(
            row.get("sim_effective_config_hash") == sim_report["effective_config_hash"],
            f"formal real parameter alignment {condition_id} sim effective-config hash differs from freeze",
        )
        real_projection = effective_config_control_projection(real_config, condition_id)
        sim_projection = effective_config_control_projection(sim_config, condition_id)
        differences = {
            field: {"real": real_projection[field], "sim": sim_projection[field]}
            for field in REQUIRED_EFFECTIVE_CONFIG_FIELDS
            if real_projection[field] != sim_projection[field]
        }
        require(
            not differences,
            "formal real parameter alignment {} differs in control fields: {}".format(
                condition_id, json.dumps(differences, ensure_ascii=False, sort_keys=True)
            ),
        )
        matched[condition_id] = {
            "real_effective_config_path": str(real_config_path),
            "real_effective_config_file_hash": sha256_file(real_config_path),
            "real_effective_config_hash": real_config_report["effective_config_hash"],
            "sim_effective_config_hash": sim_report["effective_config_hash"],
            "control_projection_hash": canonical_hash(real_projection),
            "covered_fields": list(REQUIRED_EFFECTIVE_CONFIG_FIELDS),
        }
    return {
        "status": "PASS",
        "alignment_id": document["alignment_id"],
        "alignment_rule_hash": document["alignment_rule_hash"],
        "report_path": str(alignment_path),
        "report_hash": sha256_file(alignment_path),
        "real_freeze_manifest_path": str(real_freeze_path),
        "real_freeze_manifest_hash": sha256_file(real_freeze_path),
        "conditions": matched,
    }


def flatten(value: Any, prefix: str = "") -> Dict[str, Any]:
    output: Dict[str, Any] = {}
    if isinstance(value, Mapping):
        for key in sorted(value):
            output.update(flatten(value[key], f"{prefix}.{key}" if prefix else str(key)))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            output.update(flatten(item, f"{prefix}[{index}]"))
    else:
        output[prefix] = value
    return output


def compare_smoothmatch(bsmooth: Any, smoothmatch: Any) -> Dict[str, Any]:
    validate_effective_config(bsmooth, "Bsmooth")
    validate_effective_config(smoothmatch, "SmoothMatch")
    base = dict(require_mapping(bsmooth, "Bsmooth config"))
    match = dict(require_mapping(smoothmatch, "SmoothMatch config"))
    base.pop("condition_id", None)
    match.pop("condition_id", None)
    base_flat = flatten(base)
    match_flat = flatten(match)
    keys = sorted(set(base_flat) | set(match_flat))
    diff = {
        key: {"Bsmooth": base_flat.get(key), "SmoothMatch": match_flat.get(key)}
        for key in keys
        if base_flat.get(key) != match_flat.get(key)
    }
    require(set(diff) == {"v_ref"}, "SmoothMatch may differ from Bsmooth only in v_ref; diff=" + json.dumps(diff, sort_keys=True))
    require(float(match["v_ref"]) != float(base["v_ref"]), "SmoothMatch needs its own frozen v_ref")
    return {"status": "PASS", "allowed_difference": "v_ref", "diff": diff, "coverage": list(REQUIRED_EFFECTIVE_CONFIG_FIELDS)}


def validate_fixed_profile(profile: Any, require_file: bool = True) -> Dict[str, Any]:
    profile = require_mapping(profile, "FixedProfile registry entry")
    require(profile.get("runtime_regeneration_forbidden") is True, "FixedProfile must forbid runtime regeneration")
    require(profile.get("generated_before_run") is True, "FixedProfile must be pre-generated")
    require(profile.get("read_only_replay") is True, "FixedProfile must be marked read-only replay")
    for key in ("profile_hash", "generator_hash", "tracker_config_hash", "constraint_audit_hash"):
        require(is_sha256(profile.get(key)), f"FixedProfile {key} must be a SHA-256")
    path_value = profile.get("profile_path")
    if require_file:
        require(isinstance(path_value, str) and path_value, "FixedProfile profile_path is required")
        path = Path(path_value)
        require(path.is_file(), f"FixedProfile profile does not exist: {path}")
        require(sha256_file(path) == profile["profile_hash"], "FixedProfile profile hash mismatch")
        mode = stat.S_IMODE(path.stat().st_mode)
        require(mode & (stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH) == 0, "FixedProfile CSV must be read-only")
    return {"status": "PASS", "profile_hash": profile["profile_hash"], "runtime_generation": "FORBIDDEN"}


def stage_counts(blocks: int) -> Dict[str, int]:
    return {stage: len(spec["conditions"]) * blocks for stage, spec in STAGES.items()}


def expected_row_frozen_asset_hashes(
    freeze: Mapping[str, Any],
    path_id: str,
    container_id: str,
    condition_id: str,
) -> Dict[str, str]:
    """Return the freeze-bound inputs a planned formal row must carry."""
    path_item = require_mapping(require_mapping(freeze.get("paths"), "formal paths").get(path_id), "formal path")
    container_item = require_mapping(require_mapping(freeze.get("containers"), "formal containers").get(container_id), "formal container")
    config_item = require_mapping(require_mapping(freeze.get("effective_configs"), "formal configs").get(condition_id), "formal effective config")
    simulator = require_mapping(freeze.get("simulator_assets"), "formal simulator assets")
    plant = require_mapping(freeze.get("liquid_plant_capability"), "formal plant capability")
    hashes = {
        "map_hash": str(simulator["map_hash"]),
        "world_hash": str(simulator["world_hash"]),
        "world_geometry_hash": str(simulator["world_geometry_hash"]),
        "robot_model_hash": str(simulator["robot_model_hash"]),
        "source_path_hash": str(path_item["source_path_hash"]),
        "sim_path_hash": str(path_item["sim_path_hash"]),
        "transform_hash": str(path_item["transform_hash"]),
        "physical_parameter_hash": str(container_item["physical_parameter_hash"]),
        "effective_config_hash": str(config_item["effective_config_hash"]),
        "observer_policy_hash": canonical_hash(dict(require_mapping(config_item["effective_config"].get("observer"), "formal observer"))),
        "delay_policy_hash": canonical_hash(dict(require_mapping(config_item["effective_config"].get("delay"), "formal delay"))),
        "liquid_plant_code_hash": str(plant["plant_code_hash"]),
        "plant_parameter_hash": str(plant["plant_parameter_hash"]),
        "plant_input_schema_hash": str(plant["plant_input_schema_hash"]),
        "plant_output_schema_hash": str(plant["plant_output_schema_hash"]),
    }
    if condition_id == "FixedProfile":
        profile = require_mapping(require_mapping(freeze.get("fixed_profiles"), "formal profiles").get(f"{path_id}_{container_id}"), "formal FixedProfile")
        hashes["profile_hash"] = str(profile["profile_hash"])
        hashes["tracker_config_hash"] = str(profile["tracker_config_hash"])
    return hashes


def make_master_rows(
    freeze: Mapping[str, Any],
    seed_text: Optional[str],
    fixture: bool = False,
    formal_freeze_path: Optional[Path] = None,
) -> Dict[str, Any]:
    """Generate all planned rows only after selecting fixture or validated freeze mode."""
    if fixture:
        require(isinstance(seed_text, str) and seed_text, "fixture randomization seed is required")
        require(freeze.get("fixture") is True, "fixture generation requires an explicit fixture=true input")
        blocks = int(freeze.get("sample_size", 8))
        require(blocks == 8, "fixture matrix keeps n=8 so 40/24/24 contracts are testable")
        formal = False
        freeze_report = {
            "status": "FIXTURE",
            "errors": [],
            "freeze_hash": canonical_hash(freeze),
        }
        protocol_id = str(freeze.get("protocol_id", "SIM-FIXTURE-40-64-88"))
        classification = FIXTURE_EVIDENCE_CLASS
        contrast_registry = _fixture_contrast_registry(protocol_id)
        formal_freeze_file_hash: Optional[str] = None
        formal_freeze_path_value: Optional[str] = None
        dataset_ledger: Optional[Mapping[str, Any]] = None
    else:
        freeze_report = validate_formal_freeze(freeze)
        require(freeze_report["status"] == "PASS", "FORMAL_SIM_NO_GO: " + "; ".join(freeze_report["errors"]))
        require(seed_text in (None, ""), "formal generation consumes frozen randomization/seed artifacts; --seed is forbidden")
        require(formal_freeze_path is not None and formal_freeze_path.is_file(), "formal generation requires an immutable formal_freeze_path")
        require(canonical_hash(read_json(formal_freeze_path)) == canonical_hash(freeze), "formal_freeze_path content differs from supplied freeze")
        blocks = 8
        formal = True
        protocol_id = FORMAL_PROTOCOL_ID
        classification = "FORMAL_PLANNED_ROWS_NOT_EXECUTED"
        contrast_registry = validate_frozen_contrast_registry(freeze.get("contrast_registry"))["registry"]
        formal_freeze_file_hash = sha256_file(formal_freeze_path)
        formal_freeze_path_value = str(formal_freeze_path.resolve())
        dataset_ledger = require_mapping(freeze.get("dataset_ledger"), "formal dataset ledger")

    if fixture:
        assert isinstance(seed_text, str)
        tables = {stage: make_randomization(stage, blocks, seed_text) for stage in STAGES}
        bundles = {f"{stage}:b{block:02d}": make_seed_bundle(seed_text, stage, block) for stage in STAGES for block in range(1, blocks + 1)}
    else:
        tables = dict(require_mapping(freeze.get("randomization_tables"), "formal randomization tables"))
        bundles = dict(require_mapping(freeze.get("seed_bundles"), "formal seed bundles"))
    all_counts = stage_counts(blocks)
    total = sum(all_counts.values())
    rows: List[Dict[str, Any]] = []
    for stage, table in tables.items():
        for assignment in table["rows"]:
            block = int(assignment["block_id"][1:])
            condition = assignment["condition_id"]
            bundle = bundles[f"{stage}:b{block:02d}"]
            row_id = f"{stage}_{assignment['path_id']}_{assignment['container_id']}_{condition}_{assignment['block_id']}"
            frozen_asset_hashes: Dict[str, str] = {}
            if formal:
                frozen_asset_hashes = expected_row_frozen_asset_hashes(
                    freeze,
                    str(assignment["path_id"]),
                    str(assignment["container_id"]),
                    str(condition),
                )
            rows.append(
                {
                    "schema_version": SCHEMA_VERSION,
                    "protocol_id": protocol_id,
                    "formal": formal,
                    "evidence_class": classification,
                    "stage": stage,
                    "stage_alias": STAGES[stage]["stage_alias"],
                    "planned_row_id": row_id,
                    "block_id": assignment["block_id"],
                    "order_position": assignment["order_position"],
                    "condition_id": condition,
                    "method_backend": CONDITION_BACKENDS[condition],
                    "path_id": assignment["path_id"],
                    "container_id": assignment["container_id"],
                    "planned_block_segment_id": assignment["planned_block_segment_id"],
                    "randomization_table_id": table["table_id"],
                    "randomization_hash": table["randomization_hash"],
                    "seed_bundle_id": bundle["seed_bundle_id"],
                    "seed_bundle_hash": bundle["seed_bundle_hash"],
                    "frozen_asset_hashes": frozen_asset_hashes,
                    "fixed_denominator": {
                        "n_plan_stage": all_counts[stage],
                        "n_plan_condition": blocks,
                        "n_block_plan": blocks,
                        "n_plan_total": total,
                    },
                }
            )
    master_core = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": protocol_id,
        "formal": formal,
        "evidence_class": classification,
        "freeze_hash": freeze_report["freeze_hash"],
        "freeze_validation": freeze_report,
        "randomization_tables": tables,
        "seed_bundles": bundles,
        "contrast_registry": contrast_registry,
        "dataset_ledger": dict(dataset_ledger) if dataset_ledger is not None else None,
        "planned_rows": rows,
        "counts": {"by_stage": all_counts, "total": total},
    }
    if formal:
        master_core["formal_freeze_path"] = formal_freeze_path_value
        master_core["formal_freeze_file_hash"] = formal_freeze_file_hash
    return dict(master_core, master_hash=canonical_hash(master_core))


def validate_master(master: Any, require_formal: bool = False) -> Dict[str, Any]:
    errors: List[str] = []
    if not isinstance(master, Mapping):
        return {"status": "FAIL", "errors": ["master planned-row document is not an object"]}
    formal = master.get("formal") is True
    if require_formal and not formal:
        errors.append("fixture/development master cannot be used as formal")
    if master.get("schema_version") != SCHEMA_VERSION:
        errors.append("wrong schema_version")
    master_core = dict(master)
    declared_master_hash = master_core.pop("master_hash", None)
    if declared_master_hash != canonical_hash(master_core):
        errors.append("master_hash mismatch")
    expected_counts = stage_counts(8)
    expected_total = sum(expected_counts.values())
    contrast_registry = master.get("contrast_registry")
    try:
        validate_contrast_registry(contrast_registry, formal=formal)
    except ContractError as exc:
        errors.append(f"contrast registry: {exc}")
    rows = master.get("planned_rows")
    if not isinstance(rows, list):
        return {"status": "FAIL", "errors": errors + ["planned_rows is missing"]}
    if len(rows) != expected_total:
        errors.append(f"planned row count must be {expected_total}, got {len(rows)}")
    ids = [row.get("planned_row_id") for row in rows if isinstance(row, Mapping)]
    if len(ids) != len(set(ids)):
        errors.append("planned_row_id is not unique")
    tables = master.get("randomization_tables") if isinstance(master.get("randomization_tables"), Mapping) else {}
    bundles = master.get("seed_bundles") if isinstance(master.get("seed_bundles"), Mapping) else {}
    for stage, spec in STAGES.items():
        stage_rows = [row for row in rows if isinstance(row, Mapping) and row.get("stage") == stage]
        conditions = tuple(spec["conditions"])
        if len(stage_rows) != expected_counts[stage]:
            errors.append(f"{stage} must contain {expected_counts[stage]} rows")
        table = tables.get(stage)
        if not isinstance(table, Mapping):
            errors.append(f"{stage} randomization table missing")
            continue
        try:
            validate_randomization_table(table, stage, blocks=8)
        except ContractError as exc:
            errors.append(str(exc))
            continue
        table_assignment = {(item["block_id"], item["condition_id"]): item for item in table["rows"]}
        by_block: Dict[str, List[Mapping[str, Any]]] = {}
        for row in stage_rows:
            by_block.setdefault(str(row.get("block_id")), []).append(row)
        if set(by_block) != {f"b{index:02d}" for index in range(1, 9)}:
            errors.append(f"{stage} block IDs must be b01..b08")
        position_counts: Dict[str, List[int]] = {condition: [] for condition in conditions}
        for block, group in by_block.items():
            if len(group) != len(conditions):
                errors.append(f"{stage} {block} is not complete")
                continue
            if {item.get("condition_id") for item in group} != set(conditions):
                errors.append(f"{stage} {block} condition set is wrong")
            positions = [item.get("order_position") for item in group]
            if set(positions) != set(range(1, len(conditions) + 1)):
                errors.append(f"{stage} {block} order positions are wrong")
            seed_ids = {item.get("seed_bundle_id") for item in group}
            seed_hashes = {item.get("seed_bundle_hash") for item in group}
            if len(seed_ids) != 1 or len(seed_hashes) != 1:
                errors.append(f"{stage} {block} does not share one seed bundle")
            bundle = bundles.get(f"{stage}:{block}")
            if not isinstance(bundle, Mapping) or bundle.get("seed_bundle_id") not in seed_ids:
                errors.append(f"{stage} {block} seed bundle missing")
            else:
                try:
                    validate_seed_bundle(bundle)
                except ContractError as exc:
                    errors.append(f"{stage} {block} {exc}")
            for item in group:
                condition = item.get("condition_id")
                expected_row_id = f"{stage}_{spec['path_id']}_{spec['container_id']}_{condition}_{item.get('block_id')}"
                if item.get("planned_row_id") != expected_row_id:
                    errors.append(f"{stage} {block} planned row ID is non-canonical")
                if item.get("formal") is not formal or item.get("protocol_id") != master.get("protocol_id"):
                    errors.append(f"{stage} {block} planned row formal/protocol identity mismatch")
                if condition in position_counts and isinstance(item.get("order_position"), int):
                    position_counts[condition].append(item["order_position"])
                denominator = item.get("fixed_denominator")
                if not isinstance(denominator, Mapping) or denominator.get("n_plan_stage") != expected_counts[stage] or denominator.get("n_plan_condition") != 8 or denominator.get("n_block_plan") != 8 or denominator.get("n_plan_total") != expected_total:
                    errors.append(f"{stage} {block} has invalid fixed denominator")
                if item.get("randomization_hash") != table.get("randomization_hash"):
                    errors.append(f"{stage} {block} row randomization hash mismatch")
                expected_assignment = table_assignment.get((str(item.get("block_id")), str(item.get("condition_id"))))
                if not isinstance(expected_assignment, Mapping):
                    errors.append(f"{stage} {block} planned row is absent from frozen randomization")
                elif (
                    item.get("order_position") != expected_assignment.get("order_position")
                    or item.get("planned_block_segment_id") != expected_assignment.get("planned_block_segment_id")
                    or item.get("randomization_table_id") != table.get("table_id")
                ):
                    errors.append(f"{stage} {block} planned row does not match frozen randomization assignment")
                if item.get("path_id") != spec["path_id"] or item.get("container_id") != spec["container_id"]:
                    errors.append(f"{stage} {block} path/container mismatch")
                if item.get("method_backend") != CONDITION_BACKENDS.get(condition):
                    errors.append(f"{stage} {block} backend mismatch")
                frozen_hashes = item.get("frozen_asset_hashes")
                if not isinstance(frozen_hashes, Mapping):
                    errors.append(f"{stage} {block} frozen_asset_hashes is missing")
                elif formal and not all(is_sha256(value) for value in frozen_hashes.values()):
                    errors.append(f"{stage} {block} frozen_asset_hashes contains a non-SHA-256 value")
        for condition, positions in position_counts.items():
            counts = [positions.count(position) for position in range(1, len(conditions) + 1)]
            if counts and max(counts) - min(counts) > 1:
                errors.append(f"{stage} condition {condition} is not position-balanced")
    if formal:
        report = master.get("freeze_validation")
        if not isinstance(report, Mapping) or report.get("status") != "PASS":
            errors.append("formal master lacks PASS freeze validation")
        if master.get("evidence_class") != "FORMAL_PLANNED_ROWS_NOT_EXECUTED":
            errors.append("formal master has invalid evidence class")
        try:
            freeze_owner = {
                "formal_freeze_path": master.get("formal_freeze_path"),
                "formal_freeze_file_hash": master.get("formal_freeze_file_hash"),
            }
            freeze_path = validate_bound_file(freeze_owner, "formal_freeze_path", "formal_freeze_file_hash", "formal master freeze")
            freeze = require_mapping(read_json(freeze_path), "formal master freeze")
            recomputed = validate_formal_freeze(freeze)
            require(recomputed.get("status") == "PASS", "formal master freeze no longer passes formal gate")
            require(master.get("freeze_hash") == recomputed.get("freeze_hash"), "formal master freeze_hash differs from bound freeze")
            frozen_registry = validate_frozen_contrast_registry(freeze.get("contrast_registry"))["registry"]
            require(canonical_hash(contrast_registry) == canonical_hash(frozen_registry), "formal master contrast registry differs from bound freeze")
            require(canonical_hash(master.get("dataset_ledger")) == canonical_hash(freeze.get("dataset_ledger")), "formal master dataset ledger differs from bound freeze")
            for item in rows:
                if not isinstance(item, Mapping):
                    continue
                expected_hashes = expected_row_frozen_asset_hashes(
                    freeze,
                    str(item.get("path_id")),
                    str(item.get("container_id")),
                    str(item.get("condition_id")),
                )
                require(item.get("frozen_asset_hashes") == expected_hashes, f"formal master row frozen asset hashes differ from freeze: {item.get('planned_row_id')}")
        except ContractError as exc:
            errors.append(str(exc))
    else:
        if master.get("evidence_class") not in {FIXTURE_EVIDENCE_CLASS, DEVELOPMENT_EVIDENCE_CLASS}:
            errors.append("non-formal master must be explicitly fixture/development labelled")
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "PASS" if not errors else "FAIL",
        "formal": formal,
        "errors": errors,
        "counts": {"S1": 40, "S2A": 24, "S2B": 24, "total": 88},
        "master_hash": master.get("master_hash"),
    }


def points_from_path(document: Any) -> List[Tuple[float, float, Optional[float]]]:
    source = require_mapping(document, "path JSON")
    raw = source.get("points")
    require(isinstance(raw, list) and len(raw) >= 2, "path JSON needs at least two points")
    points: List[Tuple[float, float, Optional[float]]] = []
    for index, item in enumerate(raw):
        if isinstance(item, Mapping):
            x, y = item.get("x"), item.get("y")
            yaw = item.get("yaw")
        elif isinstance(item, (list, tuple)) and len(item) >= 2:
            x, y = item[0], item[1]
            yaw = item[2] if len(item) >= 3 else None
        else:
            raise ContractError(f"path point {index} is invalid")
        require(isinstance(x, (int, float)) and isinstance(y, (int, float)) and math.isfinite(float(x)) and math.isfinite(float(y)), f"path point {index} lacks finite x/y")
        require(yaw is None or (isinstance(yaw, (int, float)) and math.isfinite(float(yaw))), f"path point {index} has invalid yaw")
        points.append((float(x), float(y), float(yaw) if yaw is not None else None))
    return points


def path_length(points: Sequence[Tuple[float, float, Optional[float]]]) -> float:
    return sum(math.hypot(points[index][0] - points[index - 1][0], points[index][1] - points[index - 1][1]) for index in range(1, len(points)))


def path_curvatures(points: Sequence[Tuple[float, float, Optional[float]]]) -> List[float]:
    headings = [math.atan2(points[index][1] - points[index - 1][1], points[index][0] - points[index - 1][0]) for index in range(1, len(points))]
    values: List[float] = []
    for index in range(1, len(headings)):
        delta = (headings[index] - headings[index - 1] + math.pi) % (2 * math.pi) - math.pi
        local_length = math.hypot(points[index + 1][0] - points[index - 1][0], points[index + 1][1] - points[index - 1][1])
        values.append(0.0 if local_length == 0 else 2.0 * delta / local_length)
    return values


def point_clearance(point: Tuple[float, float, Optional[float]], world: Mapping[str, Any]) -> float:
    x, y, _ = point
    candidates: List[float] = []
    bounds = world.get("bounds")
    if isinstance(bounds, (list, tuple)) and len(bounds) == 4:
        xmin, xmax, ymin, ymax = (float(value) for value in bounds)
        candidates.extend((x - xmin, xmax - x, y - ymin, ymax - y))
    for obstacle in world.get("obstacles", []):
        if obstacle.get("type") == "circle":
            distance = math.hypot(x - float(obstacle["x"]), y - float(obstacle["y"])) - float(obstacle["radius"])
            candidates.append(distance)
        elif obstacle.get("type") == "box":
            xmin, xmax = float(obstacle["xmin"]), float(obstacle["xmax"])
            ymin, ymax = float(obstacle["ymin"]), float(obstacle["ymax"])
            dx = max(xmin - x, 0.0, x - xmax)
            dy = max(ymin - y, 0.0, y - ymax)
            candidates.append(math.hypot(dx, dy) if dx or dy else -min(x - xmin, xmax - x, y - ymin, ymax - y))
    return min(candidates) if candidates else math.inf


def point_to_segment_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq == 0.0:
        return math.hypot(px - ax, py - ay)
    fraction = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    return math.hypot(px - (ax + fraction * dx), py - (ay + fraction * dy))


def segment_intersects_box(ax: float, ay: float, bx: float, by: float, xmin: float, xmax: float, ymin: float, ymax: float) -> bool:
    """Liang--Barsky clipping against a closed axis-aligned rectangle."""
    dx, dy = bx - ax, by - ay
    lower, upper = 0.0, 1.0
    for p, q in ((-dx, ax - xmin), (dx, xmax - ax), (-dy, ay - ymin), (dy, ymax - ay)):
        if p == 0.0:
            if q < 0.0:
                return False
            continue
        ratio = q / p
        if p < 0.0:
            if ratio > upper:
                return False
            lower = max(lower, ratio)
        else:
            if ratio < lower:
                return False
            upper = min(upper, ratio)
    return lower <= upper


def point_to_box_distance(px: float, py: float, xmin: float, xmax: float, ymin: float, ymax: float) -> float:
    dx = max(xmin - px, 0.0, px - xmax)
    dy = max(ymin - py, 0.0, py - ymax)
    return math.hypot(dx, dy)


def segment_to_box_distance(ax: float, ay: float, bx: float, by: float, xmin: float, xmax: float, ymin: float, ymax: float) -> float:
    if segment_intersects_box(ax, ay, bx, by, xmin, xmax, ymin, ymax):
        return 0.0
    corners = ((xmin, ymin), (xmin, ymax), (xmax, ymin), (xmax, ymax))
    candidates = [
        point_to_box_distance(ax, ay, xmin, xmax, ymin, ymax),
        point_to_box_distance(bx, by, xmin, xmax, ymin, ymax),
    ]
    candidates.extend(point_to_segment_distance(x, y, ax, ay, bx, by) for x, y in corners)
    return min(candidates)


def segment_clearance(
    left: Tuple[float, float, Optional[float]],
    right: Tuple[float, float, Optional[float]],
    world: Mapping[str, Any],
) -> float:
    """Exact minimum centreline clearance for supported world primitives."""
    ax, ay, _ = left
    bx, by, _ = right
    xmin, xmax, ymin, ymax = (float(value) for value in world["bounds"])
    # Every bound distance is affine along a segment, so its minimum is at an
    # endpoint.  Obstacles below use analytic segment distances.
    candidates = [
        ax - xmin,
        xmax - ax,
        ay - ymin,
        ymax - ay,
        bx - xmin,
        xmax - bx,
        by - ymin,
        ymax - by,
    ]
    for obstacle in world["obstacles"]:
        if obstacle["type"] == "circle":
            candidates.append(
                point_to_segment_distance(float(obstacle["x"]), float(obstacle["y"]), ax, ay, bx, by)
                - float(obstacle["radius"])
            )
        else:
            candidates.append(
                segment_to_box_distance(
                    ax,
                    ay,
                    bx,
                    by,
                    float(obstacle["xmin"]),
                    float(obstacle["xmax"]),
                    float(obstacle["ymin"]),
                    float(obstacle["ymax"]),
                )
            )
    return min(candidates)


def validate_world_geometry(world: Mapping[str, Any]) -> None:
    bounds = world.get("bounds")
    require(isinstance(bounds, (list, tuple)) and len(bounds) == 4, "world fit audit requires finite world bounds")
    xmin, xmax, ymin, ymax = (float(value) for value in bounds)
    require(all(math.isfinite(value) for value in (xmin, xmax, ymin, ymax)) and xmin < xmax and ymin < ymax, "world bounds are invalid")
    obstacles = world.get("obstacles", [])
    require(isinstance(obstacles, list), "world obstacles must be a list")
    for index, obstacle in enumerate(obstacles):
        obstacle = require_mapping(obstacle, f"world obstacle {index}")
        kind = obstacle.get("type")
        require(kind in {"circle", "box"}, f"world obstacle {index} has unsupported geometry {kind!r}")
        if kind == "circle":
            values = (obstacle.get("x"), obstacle.get("y"), obstacle.get("radius"))
            require(all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in values) and float(obstacle["radius"]) > 0.0, f"world circle obstacle {index} is invalid")
        else:
            values = (obstacle.get("xmin"), obstacle.get("xmax"), obstacle.get("ymin"), obstacle.get("ymax"))
            require(all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in values), f"world box obstacle {index} is invalid")
            require(float(obstacle["xmin"]) < float(obstacle["xmax"]) and float(obstacle["ymin"]) < float(obstacle["ymax"]), f"world box obstacle {index} extents are invalid")


def validate_path_replay(source_path: Path, derived_path: Path, transform: Mapping[str, Any], world: Mapping[str, Any], clearance_m: float, tolerance: float = 1e-6) -> Dict[str, Any]:
    source_document = read_json(source_path)
    derived_document = read_json(derived_path)
    source = require_mapping(source_document, "source path")
    derived = require_mapping(derived_document, "derived path")
    source_id = source.get("path_id")
    require(source_id in {"H1", "L1", "H0", "H0b", "H0s"}, "path_id must be H0/H0b/H0s/H1/L1")
    require(source.get("source_mode") == "frozen_json_replay", "runtime s_curve cannot be replayed as frozen path")
    require(derived.get("path_id") == source_id, "derived path_id must equal frozen source path_id")
    validate_world_geometry(world)
    require(isinstance(clearance_m, (int, float)) and math.isfinite(float(clearance_m)) and clearance_m > 0.0, "clearance_m must be positive and finite")
    source_points = points_from_path(source)
    derived_points = points_from_path(derived)
    require(len(source_points) == len(derived_points), "rigid transform changed point count")
    for key in ("rotation_rad", "tx", "ty", "yaw_offset_rad"):
        require(key in transform and isinstance(transform.get(key), (int, float)) and math.isfinite(float(transform[key])), f"rigid transform {key} must be finite")
    theta = float(transform["rotation_rad"])
    tx, ty = float(transform["tx"]), float(transform["ty"])
    yaw_offset = float(transform["yaw_offset_rad"])
    c, s = math.cos(theta), math.sin(theta)
    max_transform_error = 0.0
    for source_point, derived_point in zip(source_points, derived_points):
        expected_x = c * source_point[0] - s * source_point[1] + tx
        expected_y = s * source_point[0] + c * source_point[1] + ty
        max_transform_error = max(max_transform_error, math.hypot(expected_x - derived_point[0], expected_y - derived_point[1]))
        if source_point[2] is not None and derived_point[2] is not None:
            yaw_error = (source_point[2] + yaw_offset - derived_point[2] + math.pi) % (2 * math.pi) - math.pi
            max_transform_error = max(max_transform_error, abs(yaw_error))
    require(max_transform_error <= tolerance, f"derived path is not declared rigid transform (error={max_transform_error})")
    source_length, derived_length = path_length(source_points), path_length(derived_points)
    require(abs(source_length - derived_length) <= tolerance * max(1.0, source_length), "rigid transform changed arc length")
    source_kappa, derived_kappa = path_curvatures(source_points), path_curvatures(derived_points)
    max_kappa_error = max([abs(left - right) for left, right in zip(source_kappa, derived_kappa)] or [0.0])
    require(max_kappa_error <= max(tolerance * 10.0, 1e-5), "rigid transform changed curvature")
    if "zones" in source:
        require(source.get("zones") == derived.get("zones"), "derived path changed frozen Z1-Z5 zones")
    if source_id in {"H1", "L1"}:
        require(isinstance(source.get("zones"), Mapping) and set(source["zones"]).issuperset({"Z1", "Z2", "Z3", "Z4", "Z5"}), "H1/L1 frozen paths must retain Z1-Z5 zones")
    # Use analytic segment-to-circle/AABB distances.  A fixed point-sampling
    # interval can skip a narrow obstacle between samples, which is unsafe for
    # an approval gate.
    footprint_terms = (world.get("robot_footprint_radius_m", 0.0), world.get("container_footprint_radius_m", 0.0))
    require(all(isinstance(value, (int, float)) and math.isfinite(float(value)) and float(value) >= 0.0 for value in footprint_terms), "world footprint radius must be finite and non-negative")
    footprint = sum(float(value) for value in footprint_terms)
    clearances = [segment_clearance(left, right, world) - footprint for left, right in zip(derived_points, derived_points[1:])]
    minimum_clearance = min(clearances)
    require(minimum_clearance >= clearance_m, f"path/world fit fails clearance: {minimum_clearance:.6f} < {clearance_m:.6f}")
    return {
        "status": "PASS",
        "path_id": source_id,
        "source_path_hash": sha256_file(source_path),
        "sim_path_hash": sha256_file(derived_path),
        "transform_hash": canonical_hash(dict(transform)),
        "world_hash": canonical_hash(dict(world)),
        "arc_length": source_length,
        "max_transform_error": max_transform_error,
        "max_curvature_error": max_kappa_error,
        "minimum_clearance_m": minimum_clearance,
        "clearance_method": "analytic_segment_circle_aabb",
    }


def canonical_graph(graph: Mapping[str, Any]) -> Dict[str, Any]:
    subscriptions = graph.get("subscriptions", {})
    require(isinstance(subscriptions, Mapping), "graph subscriptions must be an object")
    canonical_subscriptions: Dict[str, List[str]] = {}
    for topic, nodes in sorted(subscriptions.items()):
        require(isinstance(topic, str) and topic.startswith("/"), "graph topic must be an absolute ROS topic")
        require(isinstance(nodes, list) and all(isinstance(node, str) and node.startswith("/") for node in nodes), f"graph subscribers for {topic} must be node lists")
        canonical_subscriptions[str(topic)] = sorted(nodes)
    return {"subscriptions": canonical_subscriptions}


def validate_controller_firewall(graph: Mapping[str, Any], controller_nodes: Sequence[str]) -> Dict[str, Any]:
    require(controller_nodes, "controller/planner/cmd-gate node list cannot be empty")
    normal = canonical_graph(graph)
    seen_nodes = {node for nodes in normal["subscriptions"].values() for node in nodes}
    missing = sorted(set(controller_nodes) - seen_nodes)
    violations = []
    for topic, nodes in normal["subscriptions"].items():
        if topic.startswith(FORBIDDEN_CONTROL_PREFIXES):
            for node in nodes:
                if node in controller_nodes:
                    violations.append({"node": node, "topic": topic})
    return {
        "status": "PASS" if not missing and not violations else "FAIL",
        "controller_nodes": sorted(controller_nodes),
        "missing_controller_nodes": missing,
        "violations": violations,
        "graph_hash": canonical_hash(normal),
        "graph": normal,
    }


def canonical_development_h0_subscriber_graph(graph: Mapping[str, Any]) -> Dict[str, Any]:
    """Canonicalize an H0 adapter's read-only ROS-master subscriber graph."""
    subscriptions = require_mapping(graph.get("subscriptions"), "development H0 firewall graph subscriptions")
    normal: Dict[str, List[str]] = {}
    for topic, nodes in sorted(subscriptions.items()):
        require(isinstance(topic, str) and topic.startswith("/"), "development H0 firewall graph topic must be absolute")
        require(
            isinstance(nodes, list)
            and all(isinstance(node, str) and node.startswith("/") for node in nodes),
            f"development H0 firewall graph subscribers for {topic} must be absolute node names",
        )
        normal[str(topic)] = sorted(set(nodes))
    return {"subscriptions": normal}


def validate_development_h0_firewall_contract(
    contract_value: Any, case_dir: Path
) -> Dict[str, Any]:
    """Validate the non-formal H0 live-firewall contract embedded at launch.

    This intentionally accepts no broad informal firewall graph.  The adapter
    pre-declares the four role labels and exact case-local checkpoint paths;
    later snapshot files must bind to this hash rather than to an untracked
    post-hoc graph dump.
    """
    contract = dict(require_mapping(contract_value, "development H0 firewall contract"))
    require(isinstance(contract.get("contract_id"), str) and contract["contract_id"], "development H0 firewall contract ID is required")
    require(contract.get("record_type") == DEVELOPMENT_H0_FIREWALL_RECORD_TYPE, "development H0 firewall record type mismatch")
    require(contract.get("development_only") is True, "development H0 firewall must be development-only")
    require(contract.get("formal") is False, "development H0 firewall must remain non-formal")
    require(contract.get("physical_primary_eligible") is False, "development H0 firewall must remain non-primary")
    require(contract.get("requires_development_liquid_plant") is True, "development H0 firewall must bind the liquid-plant opt-in")
    require(contract.get("record_only") is True, "development H0 firewall must remain record-only")
    require(contract.get("checkpoints") == list(DEVELOPMENT_H0_FIREWALL_CHECKPOINTS), "development H0 firewall checkpoints drift")
    require(contract.get("forbidden_topic_prefixes") == list(DEVELOPMENT_H0_FIREWALL_FORBIDDEN_PREFIXES), "development H0 firewall forbidden topic prefixes drift")
    roles = require_mapping(contract.get("node_roles"), "development H0 firewall node roles")
    require(set(roles) == DEVELOPMENT_H0_FIREWALL_ROLE_NAMES, "development H0 firewall must name controller/planner/tracker/cmd-gate roles")
    require(all(isinstance(node, str) and node.startswith("/") for node in roles.values()), "development H0 firewall role nodes must be absolute")
    protected_nodes = sorted(set(roles.values()))
    require(contract.get("controller_nodes") == protected_nodes, "development H0 firewall protected node list differs from role map")
    snapshot_paths = require_mapping(contract.get("snapshot_paths"), "development H0 firewall snapshot paths")
    require(set(snapshot_paths) == set(DEVELOPMENT_H0_FIREWALL_CHECKPOINTS), "development H0 firewall snapshot path set is incomplete")
    case = case_dir.resolve()
    for checkpoint in DEVELOPMENT_H0_FIREWALL_CHECKPOINTS:
        value = snapshot_paths[checkpoint]
        require(isinstance(value, str) and value, f"development H0 firewall {checkpoint} snapshot path is invalid")
        path = ensure_within(Path(value), case, f"development H0 firewall {checkpoint} snapshot")
        require(path.parent == case, "development H0 firewall snapshots must be directly case-local")
    require(isinstance(contract.get("adapter_source_path"), str) and contract["adapter_source_path"], "development H0 firewall adapter source path is required")
    require(is_sha256(contract.get("adapter_source_hash")), "development H0 firewall adapter source hash is invalid")
    core = {key: value for key, value in contract.items() if key != "contract_hash"}
    require(contract.get("contract_hash") == canonical_hash(core), "development H0 firewall contract hash mismatch")
    return contract


def development_h0_liquid_plant_opt_in(value: Any) -> bool:
    """Recognize only the adapter's explicitly non-primary H0 plant shape."""
    return bool(
        isinstance(value, Mapping)
        and value.get("independent_plant") is True
        and value.get("development_only") is True
        and value.get("formal") is False
        and value.get("fidelity_validation_status") == "UNVALIDATED"
        and value.get("physical_primary_eligible") is False
        and value.get("truth_topic") == "/sim_truth/liquid_height"
    )


def validate_development_h0_firewall_snapshot(
    snapshot_value: Any,
    checkpoint: str,
    case_dir: Path,
    case_manifest_hash: str,
    contract: Mapping[str, Any],
) -> Dict[str, Any]:
    """Recompute an adapter snapshot before admitting it to H0 manifests."""
    snapshot = dict(require_mapping(snapshot_value, "development H0 firewall snapshot"))
    validated_contract = validate_development_h0_firewall_contract(contract, case_dir)
    require(checkpoint in DEVELOPMENT_H0_FIREWALL_CHECKPOINTS, "invalid development H0 firewall checkpoint")
    require(snapshot.get("record_type") == DEVELOPMENT_H0_FIREWALL_RECORD_TYPE, "development H0 firewall snapshot record type mismatch")
    require(snapshot.get("checkpoint") == checkpoint, "development H0 firewall snapshot checkpoint mismatch")
    require(snapshot.get("status") == "PASS", "development H0 firewall snapshot is not PASS")
    require(snapshot.get("development_only") is True and snapshot.get("formal") is False, "development H0 firewall snapshot lost development/non-formal status")
    require(snapshot.get("physical_primary_eligible") is False, "development H0 firewall snapshot became physical-primary")
    require(snapshot.get("case_manifest_hash") == case_manifest_hash, "development H0 firewall snapshot belongs to another case manifest")
    require(snapshot.get("firewall_contract_hash") == validated_contract["contract_hash"], "development H0 firewall snapshot contract hash mismatch")
    expected_path = str(ensure_within(Path(validated_contract["snapshot_paths"][checkpoint]), case_dir, "development H0 firewall snapshot"))
    require(snapshot.get("snapshot_path") == expected_path, "development H0 firewall snapshot path mismatch")
    graph = canonical_development_h0_subscriber_graph(
        require_mapping(snapshot.get("graph"), "development H0 firewall snapshot graph")
    )
    roles = require_mapping(validated_contract["node_roles"], "development H0 firewall node roles")
    protected_nodes = set(validated_contract["controller_nodes"])
    seen_nodes = {node for nodes in graph["subscriptions"].values() for node in nodes}
    missing = sorted(protected_nodes - seen_nodes)
    all_truth_subscribers: List[Dict[str, Any]] = []
    forbidden: List[Dict[str, str]] = []
    for topic, nodes in graph["subscriptions"].items():
        if topic.startswith(DEVELOPMENT_H0_FIREWALL_FORBIDDEN_PREFIXES):
            all_truth_subscribers.append({"topic": topic, "subscribers": list(nodes)})
            for role, node in sorted(roles.items()):
                if node in nodes:
                    forbidden.append({"role": role, "node": node, "topic": topic})
    require(not missing, "development H0 firewall snapshot is missing protected controller nodes")
    require(not forbidden, "development H0 firewall snapshot has a /sim_truth controller subscriber")
    require(snapshot.get("node_roles") == dict(roles), "development H0 firewall snapshot role map mismatch")
    require(snapshot.get("controller_nodes") == sorted(protected_nodes), "development H0 firewall snapshot protected node set mismatch")
    require(snapshot.get("missing_controller_nodes") == missing, "development H0 firewall snapshot missing-node report mismatch")
    require(snapshot.get("forbidden_controller_subscribers") == forbidden, "development H0 firewall snapshot forbidden-subscriber report mismatch")
    require(snapshot.get("all_sim_truth_subscribers") == all_truth_subscribers, "development H0 firewall snapshot truth-subscriber inventory mismatch")
    require(snapshot.get("graph") == graph, "development H0 firewall snapshot graph is not canonical")
    require(snapshot.get("graph_hash") == canonical_hash(graph), "development H0 firewall snapshot graph hash mismatch")
    return snapshot


def load_development_h0_firewall_snapshot(
    path_value: Path,
    checkpoint: str,
    case_dir: Path,
    case_manifest_hash: str,
    contract: Mapping[str, Any],
) -> Tuple[Dict[str, Any], str]:
    """Load a case-local immutable H0 snapshot and return its file hash."""
    path = ensure_within(path_value, case_dir, f"development H0 firewall {checkpoint} snapshot")
    require(path.parent == case_dir.resolve(), "development H0 firewall snapshot must be directly case-local")
    require(path.is_file(), f"development H0 firewall {checkpoint} snapshot is missing")
    report = validate_development_h0_firewall_snapshot(
        read_json(path), checkpoint, case_dir, case_manifest_hash, contract
    )
    return report, sha256_file(path)


def ros_master_graph(master_uri: str) -> Dict[str, Any]:
    """Read-only ROS-master system-state probe used at lifecycle checkpoints."""
    uri = master_uri.rstrip("/")
    try:
        proxy = xmlrpc.client.ServerProxy(uri, allow_none=True)
        code, _, state = proxy.getSystemState("/smpcc_sim_toolchain")
        require(code == 1 and isinstance(state, list) and len(state) == 3, "ROS master did not return system state")
        subscriptions = {topic: nodes for topic, nodes in state[1]}
        return {"subscriptions": subscriptions}
    except (OSError, xmlrpc.client.Error, ContractError) as exc:
        raise ContractError(f"cannot read ROS master graph {master_uri}: {exc}") from exc


def socket_reachable(host: str, port: int, timeout: float = 0.25) -> bool:
    with contextlib.closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as sock:
        sock.settimeout(timeout)
        return sock.connect_ex((host, port)) == 0


def parse_endpoint(uri: str) -> Tuple[str, int]:
    match = re.fullmatch(r"(?:http://)?([^:/]+):(\d+)", uri.strip())
    require(match is not None, f"invalid master URI: {uri}")
    return match.group(1), int(match.group(2))


def http_uri(endpoint: str) -> str:
    """Keep Python 3.8 compatibility (str.removeprefix arrived later)."""
    return "http://" + (endpoint[7:] if endpoint.startswith("http://") else endpoint)


def endpoints_reachable(ros_master_uri: str, gazebo_master_uri: str) -> Dict[str, bool]:
    ros_host, ros_port = parse_endpoint(ros_master_uri)
    gazebo_host, gazebo_port = parse_endpoint(gazebo_master_uri)
    return {
        "ros": socket_reachable(ros_host, ros_port),
        "gazebo": socket_reachable(gazebo_host, gazebo_port),
    }


def wait_for_endpoints(ros_master_uri: str, gazebo_master_uri: str, timeout_sec: float) -> Dict[str, bool]:
    require(timeout_sec > 0.0, "startup timeout must be positive")
    deadline = time.monotonic() + timeout_sec
    state = endpoints_reachable(ros_master_uri, gazebo_master_uri)
    while time.monotonic() < deadline:
        if state["ros"] and state["gazebo"]:
            return state
        time.sleep(0.2)
        state = endpoints_reachable(ros_master_uri, gazebo_master_uri)
    return state


def parse_attempt_number(attempt_id: str) -> int:
    match = re.search(r"_r([0-9]{2})$", attempt_id)
    require(match is not None, f"attempt_id must end in _rNN: {attempt_id}")
    return int(match.group(1))


def attempt_prefix(attempt_id: str) -> str:
    parse_attempt_number(attempt_id)
    return attempt_id.rsplit("_r", 1)[0]


def validate_failure_class(failure_class: str, motion_started: bool, method_success: bool) -> Tuple[bool, bool]:
    """Return method_failure/retry_possible; classification is deliberately narrow."""
    allowed = {"NONE", "METHOD_FAILURE", "INFRASTRUCTURE_ACQUISITION", "PROTOCOL_FAILURE"}
    require(failure_class in allowed, f"unknown failure class: {failure_class}")
    if failure_class == "NONE":
        require(method_success, "NONE failure class requires method_success=true")
        return False, False
    require(not method_success, f"{failure_class} cannot have method_success=true")
    if failure_class == "METHOD_FAILURE":
        return True, False
    if failure_class == "INFRASTRUCTURE_ACQUISITION":
        require(not motion_started, "infrastructure acquisition failure must happen before motion")
        return False, True
    # A condition/config/profile/topic contract violation consumes the planned
    # row and is failure-inclusive; it is never an acquisition replacement.
    return True, False


def read_attempt_manifest(path: Path) -> Mapping[str, Any]:
    document = read_json(path)
    require_mapping(document, "attempt manifest")
    require(document.get("schema_version") == SCHEMA_VERSION, f"wrong attempt manifest schema: {path}")
    return document


def authorize_retry(
    previous_manifest_path: Path,
    authorization_path: Path,
    next_attempt_id: str,
    classifier_id: Optional[str] = None,
    classifier_rule_hash: Optional[str] = None,
    retry_classifier: Optional[Mapping[str, Any]] = None,
) -> Dict[str, Any]:
    """Create immutable retry authorization only for pre-motion infrastructure loss."""
    previous = read_attempt_manifest(previous_manifest_path)
    previous_id = str(previous.get("attempt_id", ""))
    require(previous_id, "previous attempt_id missing")
    require(attempt_prefix(previous_id) == attempt_prefix(next_attempt_id), "retry must retain the exact planned row identity")
    require(parse_attempt_number(next_attempt_id) == parse_attempt_number(previous_id) + 1, "retry number must directly follow previous attempt")
    failure_class = previous.get("failure_class")
    require(failure_class == "INFRASTRUCTURE_ACQUISITION", "method/protocol failures cannot be authorized for replacement retry")
    require(previous.get("method_failure") is False, "method failure cannot be overwritten by retry")
    require(previous.get("motion_started") is False, "post-motion acquisition loss cannot be replacement retry")
    classifier_report: Optional[Mapping[str, Any]] = None
    if retry_classifier is not None:
        classifier_report = validate_frozen_retry_classifier(retry_classifier)
        classifier_id = str(classifier_report["classifier_id"])
        classifier_rule_hash = str(classifier_report["classifier_rule_hash"])
        require(
            parse_attempt_number(next_attempt_id) - 1 <= int(classifier_report["max_retries_per_row"]),
            "retry exceeds frozen classifier max_retries_per_row",
        )
    require(isinstance(classifier_id, str) and classifier_id, "retry authorization classifier_id is required")
    formal_previous = previous.get("formal") is True
    if formal_previous:
        require(classifier_report is not None, "formal retry authorization requires frozen retry classifier artifact")
        require(previous.get("retry_authorization_allowed") is True, "formal previous attempt was not classifier-authorized for retry")
        decision_path_value = previous.get("failure_classification_path")
        require(isinstance(decision_path_value, str) and decision_path_value, "formal previous attempt lacks external failure classification path")
        decision_path = Path(decision_path_value)
        require(decision_path.is_file(), "formal failure classification artifact is missing")
        decision_hash = sha256_file(decision_path)
        require(decision_hash == previous.get("failure_classification_hash"), "formal failure classification artifact hash mismatch")
        failure_event_path_value = previous.get("failure_event_path")
        require(isinstance(failure_event_path_value, str) and failure_event_path_value, "formal previous attempt lacks pre-motion failure event")
        failure_event_path = Path(failure_event_path_value)
        require(failure_event_path.is_file(), "formal pre-motion failure event is missing")
        failure_event_hash = sha256_file(failure_event_path)
        require(failure_event_hash == previous.get("failure_event_hash"), "formal pre-motion failure event hash mismatch")
        decision = validate_failure_classification(
            read_json(decision_path),
            classifier_report,
            previous_id,
            str(previous.get("case_launch_manifest_hash")),
            failure_event_path=failure_event_path,
            failure_event_hash=failure_event_hash,
        )
    else:
        decision_path = None
        decision_hash = None
        failure_event_path = None
        failure_event_hash = None
        decision = None
    evidence_hash = sha256_file(previous_manifest_path)
    authorization = {
        "schema_version": SCHEMA_VERSION,
        "authorization_type": "INFRASTRUCTURE_RETRY_ONLY",
        "authorization_id": "RETRY-" + uuid.uuid4().hex,
        "classifier_id": classifier_id,
        "classifier_rule_hash": classifier_rule_hash,
        "classifier_manifest_hash": classifier_report.get("classifier_manifest_hash") if classifier_report is not None else None,
        "verifier_id": classifier_report.get("verifier_id") if classifier_report is not None else None,
        "verifier_hash": classifier_report.get("verifier_hash") if classifier_report is not None else None,
        "previous_attempt_id": previous_id,
        "previous_attempt_manifest": str(previous_manifest_path.resolve()),
        "previous_attempt_manifest_hash": evidence_hash,
        "authorized_attempt_id": next_attempt_id,
        "planned_row_id": previous.get("planned_row_id"),
        "condition_id": previous.get("condition_id"),
        "config_hash": previous.get("hashes", {}).get("effective_config_hash"),
        "profile_hash": previous.get("hashes", {}).get("profile_hash"),
        "seed_bundle_hash": previous.get("seed_bundle_hash"),
        "failure_classification_path": str(decision_path.resolve()) if decision_path is not None else None,
        "failure_classification_hash": decision_hash,
        "failure_event_path": str(failure_event_path.resolve()) if failure_event_path is not None else None,
        "failure_event_hash": failure_event_hash,
        "reason_code": decision.get("reason_code") if decision is not None else None,
        "reason": "METHOD_INDEPENDENT_INFRASTRUCTURE_ACQUISITION",
        "method_failure": False,
        "motion_started": False,
        "created_utc": utc_now(),
    }
    authorization["authorization_hash"] = canonical_hash(authorization)
    write_json_new(authorization_path, authorization)
    return authorization


def validate_retry_authorization(
    authorization_path: Path,
    row: Mapping[str, Any],
    attempt_id: str,
    expected_classifier_id: Optional[str] = None,
    expected_classifier_rule_hash: Optional[str] = None,
    expected_classifier_manifest_hash: Optional[str] = None,
    expected_verifier_id: Optional[str] = None,
    expected_verifier_hash: Optional[str] = None,
    expected_reason_codes: Optional[Sequence[str]] = None,
    expected_max_retries: Optional[int] = None,
) -> Mapping[str, Any]:
    authorization = require_mapping(read_json(authorization_path), "retry authorization")
    authorization_core = dict(authorization)
    declared_authorization_hash = authorization_core.pop("authorization_hash", None)
    require(declared_authorization_hash == canonical_hash(authorization_core), "retry authorization hash mismatch")
    require(authorization.get("authorization_type") == "INFRASTRUCTURE_RETRY_ONLY", "retry authorization type is invalid")
    require(isinstance(authorization.get("classifier_id"), str) and authorization.get("classifier_id"), "retry authorization classifier_id is missing")
    if expected_classifier_id is not None:
        require(authorization.get("classifier_id") == expected_classifier_id, "retry authorization classifier differs from frozen classifier")
    if expected_classifier_rule_hash is not None:
        require(authorization.get("classifier_rule_hash") == expected_classifier_rule_hash, "retry authorization classifier rule differs from frozen classifier")
    if expected_classifier_manifest_hash is not None:
        require(authorization.get("classifier_manifest_hash") == expected_classifier_manifest_hash, "retry authorization classifier manifest differs from frozen classifier")
    if expected_verifier_id is not None:
        require(authorization.get("verifier_id") == expected_verifier_id, "retry authorization verifier differs from frozen classifier")
    if expected_verifier_hash is not None:
        require(authorization.get("verifier_hash") == expected_verifier_hash, "retry authorization verifier hash differs from frozen classifier")
    require(authorization.get("authorized_attempt_id") == attempt_id, "retry authorization targets another attempt")
    require(authorization.get("planned_row_id") == row.get("planned_row_id"), "retry authorization row mismatch")
    require(authorization.get("condition_id") == row.get("condition_id"), "retry authorization condition mismatch")
    require(authorization.get("method_failure") is False and authorization.get("motion_started") is False, "retry authorization is not condition-blind infrastructure only")
    previous_path = Path(str(authorization.get("previous_attempt_manifest", "")))
    require(previous_path.is_file(), "retry authorization previous manifest is missing")
    require(sha256_file(previous_path) == authorization.get("previous_attempt_manifest_hash"), "retry authorization previous manifest hash mismatch")
    previous = read_attempt_manifest(previous_path)
    require(previous.get("attempt_id") == authorization.get("previous_attempt_id"), "retry authorization previous attempt identity mismatch")
    require(previous.get("planned_row_id") == row.get("planned_row_id"), "retry previous manifest planned-row mismatch")
    require(previous.get("condition_id") == row.get("condition_id"), "retry previous manifest condition mismatch")
    require(attempt_prefix(str(previous.get("attempt_id"))) == attempt_prefix(attempt_id), "retry previous manifest attempt prefix mismatch")
    require(parse_attempt_number(attempt_id) == parse_attempt_number(str(previous.get("attempt_id"))) + 1, "retry authorization must advance exactly one attempt")
    if expected_max_retries is not None:
        require(parse_attempt_number(attempt_id) - 1 <= expected_max_retries, "retry authorization exceeds frozen max_retries_per_row")
    require(previous.get("failure_class") == "INFRASTRUCTURE_ACQUISITION", "previous attempt is not authorized infrastructure failure")
    require(previous.get("method_failure") is False, "previous method failure cannot be retried")
    require(previous.get("method_success") is False, "successful attempt cannot be retried")
    require(previous.get("motion_started") is False, "previous motion had started; replacement retry denied")
    require(authorization.get("config_hash") == previous.get("hashes", {}).get("effective_config_hash"), "retry config evidence mismatch")
    require(authorization.get("profile_hash") == previous.get("hashes", {}).get("profile_hash"), "retry profile evidence mismatch")
    require(authorization.get("seed_bundle_hash") == previous.get("seed_bundle_hash"), "retry seed evidence mismatch")
    formal_expected = any(
        value is not None
        for value in (
            expected_classifier_id,
            expected_classifier_rule_hash,
            expected_classifier_manifest_hash,
            expected_verifier_id,
            expected_verifier_hash,
            expected_reason_codes,
        )
    )
    if formal_expected:
        require(previous.get("formal") is True, "frozen classifier authorization must reference a formal previous attempt")
        require(previous.get("retry_authorization_allowed") is True, "formal previous attempt was not classifier-authorized for retry")
        decision_path_value = authorization.get("failure_classification_path")
        require(isinstance(decision_path_value, str) and decision_path_value, "formal retry authorization lacks failure classification path")
        decision_path = Path(decision_path_value)
        require(decision_path.is_file(), "formal retry classification artifact is missing")
        require(sha256_file(decision_path) == authorization.get("failure_classification_hash"), "formal retry classification artifact hash mismatch")
        require(previous.get("failure_classification_path") == str(decision_path.resolve()), "formal retry classification path differs from previous attempt")
        require(previous.get("failure_classification_hash") == authorization.get("failure_classification_hash"), "formal retry classification hash differs from previous attempt")
        failure_event_path_value = authorization.get("failure_event_path")
        require(isinstance(failure_event_path_value, str) and failure_event_path_value, "formal retry authorization lacks pre-motion failure event")
        failure_event_path = Path(failure_event_path_value)
        require(failure_event_path.is_file() and sha256_file(failure_event_path) == authorization.get("failure_event_hash"), "formal retry failure event hash mismatch")
        require(previous.get("failure_event_path") == str(failure_event_path.resolve()) and previous.get("failure_event_hash") == authorization.get("failure_event_hash"), "formal retry failure event differs from previous attempt")
        expected_classifier = {
            "classifier_id": expected_classifier_id,
            "classifier_rule_hash": expected_classifier_rule_hash,
            "classifier_manifest_hash": expected_classifier_manifest_hash,
            "verifier_id": expected_verifier_id,
            "verifier_hash": expected_verifier_hash,
            "reason_codes": list(expected_reason_codes or []),
        }
        decision = validate_failure_classification(
            read_json(decision_path),
            expected_classifier,
            str(previous.get("attempt_id")),
            str(previous.get("case_launch_manifest_hash")),
            failure_event_path=failure_event_path,
            failure_event_hash=str(authorization.get("failure_event_hash")),
        )
        require(authorization.get("reason_code") == decision.get("reason_code"), "formal retry reason_code differs from classification decision")
    return authorization


def validate_formal_attempt_manifest_bindings(
    manifest: Mapping[str, Any],
    row: Mapping[str, Any],
    freeze: Mapping[str, Any],
    master_hash: str,
) -> None:
    """Re-check immutable row inputs while summarizing a formal ledger."""
    require(manifest.get("formal") is True and manifest.get("formal_master_hash") == master_hash, "formal attempt manifest is not bound to master")
    require(manifest.get("formal_freeze_hash") == canonical_hash(freeze), "formal attempt manifest freeze hash mismatch")
    require(manifest.get("planned_row_id") == row.get("planned_row_id"), "formal attempt manifest planned row mismatch")
    require(manifest.get("seed_bundle_id") == row.get("seed_bundle_id") and manifest.get("seed_bundle_hash") == row.get("seed_bundle_hash"), "formal attempt manifest seed bundle mismatch")
    hashes = require_mapping(manifest.get("hashes"), "formal attempt manifest hashes")
    expected = expected_row_frozen_asset_hashes(freeze, str(row["path_id"]), str(row["container_id"]), str(row["condition_id"]))
    for key, expected_value in expected.items():
        actual_key = "path_hash" if key == "sim_path_hash" else key
        if key in {"source_path_hash", "transform_hash"}:
            continue
        require(hashes.get(actual_key) == expected_value, f"formal attempt manifest {actual_key} differs from planned row")
    require(row.get("frozen_asset_hashes") == expected, "formal attempt row frozen asset hashes differ from bound freeze")
    for path_key, hash_key, label in (
        ("case_launch_manifest_path", "case_launch_manifest_hash", "case launch manifest"),
        ("effective_config_path", "effective_config_dump_hash", "effective config dump"),
        ("postflight_path", "postflight_hash", "postflight"),
    ):
        path_value = manifest.get(path_key)
        require(isinstance(path_value, str) and Path(path_value).is_file(), f"formal attempt {label} is missing")
        require(sha256_file(Path(path_value)) == manifest.get(hash_key), f"formal attempt {label} hash mismatch")
    case_document = require_mapping(read_json(Path(str(manifest["case_launch_manifest_path"]))), "formal case launch manifest")
    require(case_document.get("planned_row_id") == row.get("planned_row_id") and case_document.get("formal") is True, "formal case launch manifest identity mismatch")
    require(case_document.get("hashes") == hashes and case_document.get("seed_bundle_hash") == row.get("seed_bundle_hash"), "formal case launch manifest input binding mismatch")
    ledger = validate_formal_dataset_ledger(freeze.get("dataset_ledger"), freeze)
    require(case_document.get("dataset_ledger_id") == ledger["ledger_id"] and case_document.get("dataset_ledger_identity_hash") == ledger["ledger_identity_hash"], "formal case launch manifest ledger identity mismatch")
    require(manifest.get("dataset_root") == ledger["ledger_root"] and case_document.get("dataset_root") == ledger["ledger_root"], "formal attempt dataset root differs from frozen ledger")
    contract = validate_frozen_runtime_launch_contract(freeze.get("runtime_launch_contract"))
    require(case_document.get("runtime_launch_contract_id") == contract["contract_id"] and case_document.get("runtime_launch_contract_hash") == contract["contract_hash"], "formal case launch manifest launch-contract mismatch")
    require(case_document.get("ros_master_uri") == contract["ros_master_uri"] and case_document.get("gazebo_master_uri") == contract["gazebo_master_uri"], "formal case launch manifest master URI mismatch")
    if manifest.get("method_success") is True:
        for path_key, hash_key, validator, schema_key in (
            ("runtime_input_acknowledgement_path", "runtime_input_acknowledgement_hash", validate_runtime_ack, "runtime_ack_schema_hash"),
            ("motion_release_acknowledgement_path", "motion_release_acknowledgement_hash", validate_motion_release_ack, "motion_release_ack_schema_hash"),
            ("motion_stop_acknowledgement_path", "motion_stop_acknowledgement_hash", validate_motion_stop_ack, "motion_stop_ack_schema_hash"),
        ):
            ack_path_value = manifest.get(path_key)
            require(isinstance(ack_path_value, str) and Path(ack_path_value).is_file(), f"formal successful attempt lacks {path_key}")
            ack_path = Path(ack_path_value)
            require(sha256_file(ack_path) == manifest.get(hash_key), f"formal successful attempt {path_key} hash mismatch")
            if validator is validate_runtime_ack:
                validator(ack_path, str(manifest["case_launch_manifest_hash"]), hashes, require_mapping(read_json(Path(str(manifest["seed_bundle_path"]))), "formal seed bundle"), formal=True, expected_schema_hash=str(contract[schema_key]))
            else:
                validator(ack_path, str(manifest["case_launch_manifest_hash"]), expected_schema_hash=str(contract[schema_key]))


def append_dataset_index(index_path: Path, record: Mapping[str, Any]) -> Dict[str, Any]:
    """Append one chained JSONL record; duplicate attempt IDs are refused."""
    index_path.parent.mkdir(parents=True, exist_ok=True)
    attempt_id = record.get("attempt_id")
    require(isinstance(attempt_id, str) and attempt_id, "index record requires attempt_id")
    # Serialize check-and-append so two sequential runners cannot consume the
    # same attempt slot after seeing an empty index.
    with index_path.open("a+", encoding="utf-8") as stream:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
        try:
            stream.seek(0)
            existing: List[Mapping[str, Any]] = []
            previous: Optional[str] = None
            for line_number, line in enumerate(stream, start=1):
                require(line.strip(), f"dataset index has blank line {line_number}")
                item = require_mapping(json.loads(line), f"dataset index line {line_number}")
                core_existing = dict(item)
                entry_hash = core_existing.pop("entry_hash", None)
                require(core_existing.get("previous_entry_hash") == previous, f"dataset index chain break at line {line_number}")
                require(entry_hash == canonical_hash(core_existing), f"dataset index hash mismatch at line {line_number}")
                existing.append(item)
                previous = str(entry_hash)
            require(attempt_id not in {item.get("attempt_id") for item in existing}, f"append-only index already contains attempt {attempt_id}")
            core = dict(record)
            core.pop("entry_hash", None)
            core["previous_entry_hash"] = previous
            entry = dict(core, entry_hash=canonical_hash(core))
            line = json.dumps(entry, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n"
            stream.seek(0, os.SEEK_END)
            stream.write(line)
            stream.flush()
            os.fsync(stream.fileno())
        finally:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
    return entry


def load_dataset_index(index_path: Path) -> List[Mapping[str, Any]]:
    if not index_path.is_file():
        return []
    records: List[Mapping[str, Any]] = []
    previous: Optional[str] = None
    with index_path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            require(line.strip(), f"dataset index has blank line {line_number}")
            item = require_mapping(json.loads(line), f"dataset index line {line_number}")
            core = dict(item)
            entry_hash = core.pop("entry_hash", None)
            require(core.get("previous_entry_hash") == previous, f"dataset index chain break at line {line_number}")
            require(entry_hash == canonical_hash(core), f"dataset index hash mismatch at line {line_number}")
            records.append(item)
            previous = str(entry_hash)
    return records


def consecutive_method_failure_rows(index_path: Path, stage: str, condition_id: str) -> int:
    """Count terminal failures of distinct planned rows, newest first."""
    seen_rows = set()
    streak = 0
    for item in reversed(load_dataset_index(index_path)):
        if item.get("stage") != stage or item.get("condition_id") != condition_id:
            continue
        row_id = item.get("planned_row_id")
        if row_id in seen_rows:
            continue
        seen_rows.add(row_id)
        if item.get("method_failure") is True:
            streak += 1
        else:
            break
    return streak


def summarize_ledger(master: Mapping[str, Any], index_path: Path) -> Dict[str, Any]:
    """Derive fixed denominators and registered contrast pairs from the ledger.

    In particular, this function does not assume that Bslosh's comparator is
    Bsmooth.  Every N_pair is named by a frozen contrast registry entry.
    """
    validation = validate_master(master)
    require(validation["status"] == "PASS", "cannot summarize an invalid master")
    registry = require_mapping(master.get("contrast_registry"), "master contrast registry")
    validate_contrast_registry(registry, formal=master.get("formal") is True)
    records = load_dataset_index(index_path)
    rows = {str(row["planned_row_id"]): row for row in master["planned_rows"]}
    by_row: Dict[str, List[Mapping[str, Any]]] = {row_id: [] for row_id in rows}
    formal_classifier: Optional[Mapping[str, Any]] = None
    formal_freeze: Optional[Mapping[str, Any]] = None
    formal_ledger: Optional[Mapping[str, Any]] = None
    if master.get("formal") is True:
        freeze_path = Path(str(master.get("formal_freeze_path", "")))
        formal_freeze = require_mapping(read_json(freeze_path), "formal ledger freeze")
        formal_classifier = validate_frozen_retry_classifier(formal_freeze.get("retry_classifier"))
        formal_ledger = validate_formal_dataset_ledger(formal_freeze.get("dataset_ledger"), formal_freeze)
        require(canonical_hash(master.get("dataset_ledger")) == canonical_hash(formal_freeze.get("dataset_ledger")), "formal ledger master dataset ledger mismatch")
    for item in records:
        row_id = item.get("planned_row_id")
        require(row_id in by_row, f"dataset index contains unknown planned row: {row_id}")
        manifest_path_value = item.get("attempt_manifest")
        if manifest_path_value is not None:
            manifest_path = Path(str(manifest_path_value))
            require(manifest_path.is_file(), f"dataset index attempt manifest is missing: {manifest_path}")
            require(sha256_file(manifest_path) == item.get("attempt_manifest_hash"), f"dataset index manifest hash mismatch: {manifest_path}")
            manifest = read_attempt_manifest(manifest_path)
            for key in (
                "attempt_id",
                "planned_row_id",
                "stage",
                "condition_id",
                "failure_class",
                "method_failure",
                "method_success",
                "split_block",
                "actual_block_segment_id",
            ):
                require(manifest.get(key) == item.get(key), f"dataset index/manifest {key} mismatch for {row_id}")
            if master.get("formal") is True:
                assert formal_freeze is not None
                assert formal_ledger is not None
                require(item.get("dataset_ledger_id") == formal_ledger["ledger_id"] and item.get("dataset_ledger_identity_hash") == formal_ledger["ledger_identity_hash"], f"formal dataset record ledger identity mismatch: {row_id}")
                require(manifest.get("dataset_ledger_id") == formal_ledger["ledger_id"] and manifest.get("dataset_ledger_identity_hash") == formal_ledger["ledger_identity_hash"], f"formal attempt manifest ledger identity mismatch: {row_id}")
                validate_formal_attempt_manifest_bindings(manifest, rows[str(row_id)], formal_freeze, str(master.get("master_hash")))
                attempt_number = parse_attempt_number(str(manifest.get("attempt_id")))
                if attempt_number == 1:
                    require(manifest.get("retry_authorization_path") is None, f"formal r01 unexpectedly has retry authorization: {row_id}")
                    require(manifest.get("retry_of_attempt_id") is None, f"formal r01 unexpectedly has retry parent: {row_id}")
                else:
                    require(manifest.get("retry_of_attempt_id") == f"{attempt_prefix(str(manifest['attempt_id']))}_r{attempt_number - 1:02d}", f"formal retry parent is invalid: {row_id}")
                    auth_path_value = manifest.get("retry_authorization_path")
                    require(isinstance(auth_path_value, str) and auth_path_value, f"formal retry has no authorization path: {row_id}")
                    assert formal_classifier is not None
                    authorization = validate_retry_authorization(
                        Path(auth_path_value),
                        rows[str(row_id)],
                        str(manifest["attempt_id"]),
                        expected_classifier_id=str(formal_classifier["classifier_id"]),
                        expected_classifier_rule_hash=str(formal_classifier["classifier_rule_hash"]),
                        expected_classifier_manifest_hash=str(formal_classifier["classifier_manifest_hash"]),
                        expected_verifier_id=str(formal_classifier["verifier_id"]),
                        expected_verifier_hash=str(formal_classifier["verifier_hash"]),
                        expected_reason_codes=list(formal_classifier["reason_codes"]),
                        expected_max_retries=int(formal_classifier["max_retries_per_row"]),
                    )
                    require(authorization.get("authorization_hash") == manifest.get("retry_authorization_hash"), f"formal retry authorization hash differs from manifest: {row_id}")
                    require(authorization.get("authorization_id") == manifest.get("retry_authorization_id"), f"formal retry authorization ID differs from manifest: {row_id}")
        elif master.get("formal") is True:
            raise ContractError(f"formal dataset index record lacks immutable attempt manifest: {row_id}")
        by_row[str(row_id)].append(item)

    chosen: Dict[str, Dict[str, Any]] = {}
    for row_id, attempts in by_row.items():
        attempts = sorted(attempts, key=lambda item: parse_attempt_number(str(item["attempt_id"])))
        state = "NOT_ATTEMPTED" if not attempts else "UNRESOLVED_ACQUISITION"
        chosen_attempt: Optional[str] = None
        terminal_state: Optional[str] = None
        continuous_eligible = False
        actual_segment_id: Optional[str] = None
        previous_number = 0
        for item in attempts:
            number = parse_attempt_number(str(item["attempt_id"]))
            require(number == previous_number + 1, f"retry chain skips an attempt number for {row_id}")
            previous_number = number
            failure = str(item.get("failure_class"))
            method_failure = item.get("method_failure") is True
            if terminal_state is not None:
                raise ContractError(f"terminal {terminal_state} for {row_id} was followed by replacement attempt")
            if method_failure or failure == "METHOD_FAILURE":
                state = "METHOD_FAILURE"
                chosen_attempt = str(item["attempt_id"])
                terminal_state = "method failure"
                continue
            if item.get("method_success") is True:
                state = "METHOD_SUCCESS"
                chosen_attempt = str(item["attempt_id"])
                actual_segment_id = str(item.get("actual_block_segment_id", rows[row_id]["planned_block_segment_id"]))
                continuous_eligible = (
                    item.get("continuous_eligibility") is True
                    and item.get("split_block") is not True
                    and actual_segment_id == rows[row_id]["planned_block_segment_id"]
                )
                terminal_state = "method success"
                continue
            if failure == "PROTOCOL_FAILURE":
                state = "METHOD_FAILURE"
                chosen_attempt = str(item["attempt_id"])
                terminal_state = "protocol failure"
                continue
            require(failure == RETRYABLE_FAILURE_CLASS, f"unresolved non-method attempt has invalid class for {row_id}")
        chosen[row_id] = {
            "planned_row_id": row_id,
            "outcome": state,
            "chosen_attempt_id": chosen_attempt,
            "n_attempt": len(attempts),
            "continuous_eligible": continuous_eligible,
            "actual_block_segment_id": actual_segment_id,
        }

    groups: Dict[Tuple[str, str], Dict[str, int]] = {}
    stage_blocks: Dict[str, Dict[str, Dict[str, Mapping[str, Any]]]] = {}
    for row_id, row in rows.items():
        stage, condition = str(row["stage"]), str(row["condition_id"])
        bucket = groups.setdefault(
            (stage, condition),
            {
                "N_plan": 0,
                "N_attempt": 0,
                "N_method_success": 0,
                "N_method_failure": 0,
                "N_unresolved_acquisition": 0,
                "N_not_attempted": 0,
            },
        )
        bucket["N_plan"] += 1
        result = chosen[row_id]
        bucket["N_attempt"] += result["n_attempt"]
        if result["outcome"] == "METHOD_SUCCESS":
            bucket["N_method_success"] += 1
        elif result["outcome"] == "METHOD_FAILURE":
            bucket["N_method_failure"] += 1
        elif result["outcome"] == "UNRESOLVED_ACQUISITION":
            bucket["N_unresolved_acquisition"] += 1
        else:
            bucket["N_not_attempted"] += 1
        stage_blocks.setdefault(stage, {}).setdefault(str(row["block_id"]), {})[condition] = result

    contrast_results: List[Dict[str, Any]] = []
    primary_pairs_by_condition: Dict[Tuple[str, str], List[Dict[str, Any]]] = {}
    for contrast in registry["contrasts"]:
        stage = str(contrast["stage"])
        left, right = str(contrast["left_condition"]), str(contrast["right_condition"])
        pair_count = 0
        for block_id, outcomes in stage_blocks[stage].items():
            left_result, right_result = outcomes.get(left), outcomes.get(right)
            planned_segment = f"{stage}_{block_id}_seg01"
            if all(
                isinstance(result, Mapping)
                and result.get("outcome") == "METHOD_SUCCESS"
                and result.get("continuous_eligible") is True
                and result.get("actual_block_segment_id") == planned_segment
                for result in (left_result, right_result)
            ):
                pair_count += 1
        result = {
            "stage": stage,
            "contrast_id": contrast["contrast_id"],
            "left_condition": left,
            "right_condition": right,
            "role": contrast["role"],
            "N_pair": pair_count,
            "N_block_plan": contrast["n_block_plan"],
            "minimum_n_pair": contrast["minimum_n_pair"],
            "N_pair_fraction": f"{pair_count}/{contrast['n_block_plan']}",
            "split_block_excluded": True,
        }
        contrast_results.append(result)
        if contrast["role"] == "primary_physical":
            primary_pairs_by_condition.setdefault((stage, left), []).append(result)
            primary_pairs_by_condition.setdefault((stage, right), []).append(result)

    result_groups = []
    for (stage, condition), bucket in sorted(groups.items()):
        primary_pairs = primary_pairs_by_condition.get((stage, condition), [])
        # A condition-level N_pair is provided only when that condition has a
        # uniquely registered primary physical comparison.  Full statistics
        # must consume by_contrast, never this convenience field.
        primary_pair = primary_pairs[0] if len(primary_pairs) == 1 else None
        result_groups.append(
            dict(
                {
                    "stage": stage,
                    "condition_id": condition,
                    "N_pair": primary_pair["N_pair"] if primary_pair is not None else None,
                    "N_pair_contrast_id": primary_pair["contrast_id"] if primary_pair is not None else None,
                    "N_block_plan": primary_pair["N_block_plan"] if primary_pair is not None else None,
                },
                **bucket,
            )
        )
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "PASS",
        "dataset_index": str(index_path),
        "dataset_index_hash": sha256_file(index_path) if index_path.exists() else None,
        "contrast_registry_id": registry["registry_id"],
        "contrast_registry_hash": registry["registry_hash"],
        "N_plan": len(rows),
        "N_attempt": len(records),
        "N_method_success": sum(item["outcome"] == "METHOD_SUCCESS" for item in chosen.values()),
        "N_method_failure": sum(item["outcome"] == "METHOD_FAILURE" for item in chosen.values()),
        "N_unresolved_acquisition": sum(item["outcome"] == "UNRESOLVED_ACQUISITION" for item in chosen.values()),
        "N_not_attempted": sum(item["outcome"] == "NOT_ATTEMPTED" for item in chosen.values()),
        "by_stage_condition": result_groups,
        "by_contrast": contrast_results,
        "chosen_attempts": list(chosen.values()),
    }


def command_from_spec(value: Any, field: str) -> List[str]:
    if isinstance(value, str):
        command = shlex.split(value)
    elif isinstance(value, list) and all(isinstance(item, str) for item in value):
        command = list(value)
    else:
        raise ContractError(f"{field} must be a string or argv list")
    require(command, f"{field} cannot be empty")
    flattened = " ".join(command).lower()
    require(not any(token in flattened for token in FORBIDDEN_COMMAND_TOKENS), f"{field} contains forbidden broad process control")
    require(not has_forbidden_w5(command), f"{field} attempts to invoke rejected W5/W5_S10")
    return command


class TrackedChildren:
    """Own only the process groups launched by this runner; never scan/kill globally."""

    def __init__(self) -> None:
        self.children: List[Tuple[str, subprocess.Popen[Any], str]] = []

    def start(self, label: str, command: Sequence[str], env: Mapping[str, str]) -> subprocess.Popen[Any]:
        process = subprocess.Popen(list(command), env=dict(env), start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.children.append((label, process, utc_now()))
        return process

    def stop_all(self) -> List[Dict[str, Any]]:
        result: List[Dict[str, Any]] = []
        for label, process, started in reversed(self.children):
            status = {"label": label, "pid": process.pid, "started_utc": started, "returncode_before": process.poll(), "signal": None, "cleanup_error": None}
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    status["signal"] = "SIGTERM"
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                        status["signal"] = "SIGKILL"
                        process.wait(timeout=3)
                    except (OSError, subprocess.TimeoutExpired) as exc:
                        status["cleanup_error"] = repr(exc)
                except ProcessLookupError:
                    pass
                except OSError as exc:
                    status["cleanup_error"] = repr(exc)
            status["returncode_after"] = process.poll()
            result.append(status)
        return result

    def stop_label(self, target_label: str) -> Optional[Dict[str, Any]]:
        for label, process, started in reversed(self.children):
            if label != target_label:
                continue
            status = {"label": label, "pid": process.pid, "started_utc": started, "returncode_before": process.poll(), "signal": None, "cleanup_error": None}
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    status["signal"] = "SIGTERM"
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                        status["signal"] = "SIGKILL"
                        process.wait(timeout=3)
                    except (OSError, subprocess.TimeoutExpired) as exc:
                        status["cleanup_error"] = repr(exc)
                except ProcessLookupError:
                    pass
                except OSError as exc:
                    status["cleanup_error"] = repr(exc)
            status["returncode_after"] = process.poll()
            return status
        return None


def acquire_master_lock(sim_root: Path, ros_uri: str, gazebo_uri: str, attempt_id: str) -> Tuple[Path, str]:
    """Fail closed instead of allowing concurrent runners to race one master."""
    token = uuid.uuid4().hex
    name = hashlib.sha256((ros_uri + "|" + gazebo_uri).encode("utf-8")).hexdigest()[:16]
    path = sim_root / f".smpcc_sim_master_{name}.lock"
    try:
        with path.open("x", encoding="utf-8") as stream:
            stream.write(json.dumps({"attempt_id": attempt_id, "token": token, "created_utc": utc_now()}, sort_keys=True) + "\n")
    except FileExistsError as exc:
        raise ContractError(f"fresh-master lock already exists; another runner or stale lock must be audited: {path}") from exc
    return path, token


def release_master_lock(path: Optional[Path], token: Optional[str]) -> bool:
    if path is None or token is None:
        return True
    try:
        payload = read_json(path)
        if isinstance(payload, Mapping) and payload.get("token") == token:
            path.unlink()
            return not path.exists()
    except (OSError, ContractError):
        # Never delete a lock that cannot be positively attributed to this run.
        return False
    return False


def wait_for_goal(command: Sequence[str], env: Mapping[str, str], timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        probe_process: Optional[subprocess.Popen[Any]] = None
        try:
            probe_process = subprocess.Popen(
                list(command),
                env=dict(env),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            stdout, _ = probe_process.communicate(timeout=max(0.05, min(1.0, remaining)))
        except subprocess.TimeoutExpired:
            if probe_process is not None:
                try:
                    os.killpg(probe_process.pid, signal.SIGTERM)
                    probe_process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(probe_process.pid, signal.SIGKILL)
                    probe_process.wait(timeout=2)
                except ProcessLookupError:
                    pass
            continue
        if probe_process.returncode == 0 and "GOAL_REACHED" in (stdout or ""):
            return True
        time.sleep(min(0.25, max(0.0, deadline - time.monotonic())))
    return False


def run_command_with_timeout(command: Sequence[str], env: Mapping[str, str], field: str, timeout_sec: float) -> subprocess.CompletedProcess[Any]:
    """Bound auxiliary commands so a case cannot hold the master forever."""
    require(timeout_sec > 0.0 and math.isfinite(timeout_sec), f"{field} timeout must be positive and finite")
    process = subprocess.Popen(list(command), env=dict(env), start_new_session=True)
    try:
        returncode = process.wait(timeout=timeout_sec)
        return subprocess.CompletedProcess(list(command), returncode)
    except subprocess.TimeoutExpired as exc:
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)
        except ProcessLookupError:
            pass
        raise ContractError(f"{field} timed out after {timeout_sec:g} seconds") from exc


def runtime_config_asset_path(assets: Mapping[str, Any], row: Mapping[str, Any]) -> Path:
    """Select the config declaration that is admissible before a run starts.

    Formal rows always use their frozen effective-config file.  An actual H0
    development run instead starts from an explicitly named declaration and
    later proves it against a bag-derived readback after the recorder closes.
    """
    declared = assets.get("declared_config_file")
    if row.get("formal") is not True and declared is not None:
        require(isinstance(declared, str) and declared, "development declared_config_file is invalid")
        path = Path(declared)
    else:
        raw = assets.get("effective_config_file")
        require(isinstance(raw, str) and raw, "run assets missing effective_config_file")
        path = Path(raw)
    require(path.is_file(), f"runtime config asset is missing: {path}")
    return path


def is_development_h0_declared_config(assets: Mapping[str, Any], row: Mapping[str, Any]) -> bool:
    """Whether an H0-only envelope replaces the formal config schema."""
    return (
        row.get("formal") is not True
        and isinstance(row.get("stage"), str)
        and row["stage"].startswith("SIM-DEV-")
        and row.get("path_id") in DEVELOPMENT_RUNTIME_PATH_IDS
        and assets.get("declared_config_file") is not None
    )


def validate_development_effective_config_readback(
    path: Path,
    case_manifest_hash: str,
    declared_config: Mapping[str, Any],
) -> Mapping[str, Any]:
    """Accept a case-local, bag-derived H0 config readback only when exact.

    This deliberately does not substitute for a formal runtime ACK.  It closes
    the development-only loophole where a declared JSON was written into the
    manifest without proving the running controller actually used it.
    """
    readback = require_mapping(read_json(path), "development effective-config readback")
    require(readback.get("record_type") == "SMPCC_SIM_DEV_EFFECTIVE_CONFIG_READBACK", "development effective-config readback has wrong record_type")
    require(readback.get("status") == "PASS", "development effective-config readback is not PASS")
    require(readback.get("case_manifest_hash") == case_manifest_hash, "development effective-config readback belongs to another case")
    declared_hash = canonical_hash(dict(declared_config))
    require(readback.get("declared_config_hash") == declared_hash, "development effective-config readback declared hash mismatch")
    observed = require_mapping(readback.get("observed_effective_config"), "development observed effective config")
    observed_hash = canonical_hash(dict(observed))
    require(readback.get("observed_effective_config_hash") == observed_hash, "development effective-config readback observed hash mismatch")
    require(observed_hash == declared_hash, "development effective-config readback differs from the declared config")
    return dict(readback)


def asset_hashes(spec: Mapping[str, Any], row: Mapping[str, Any]) -> Dict[str, str]:
    assets = require_mapping(spec.get("assets"), "run assets")
    required = ("map_file", "world_file", "path_file")
    output: Dict[str, str] = {}
    for key in required:
        raw = assets.get(key)
        require(isinstance(raw, str) and raw, f"run assets missing {key}")
        path = Path(raw)
        require(path.is_file(), f"run asset missing: {path}")
        output[key.replace("_file", "_hash")] = sha256_file(path)
    config_path = runtime_config_asset_path(assets, row)
    config = read_json(config_path)
    require(not has_forbidden_w5(config), "runtime effective config attempts to revive rejected W5/W5_S10")
    if is_development_h0_declared_config(assets, row):
        # H0 is not a formal condition release.  Its adapter records a
        # complete, bag-derived declaration/readback envelope rather than the
        # formal top-level effective-config schema.  Keep an explicit alias
        # for legacy ledger fields, but do not pretend observer/delay fields
        # have passed the formal schema validator.
        require(isinstance(config.get("fields"), Mapping) and config["fields"], "H0 declared config envelope requires a non-empty fields object")
        envelope_hash = canonical_hash(dict(config))
        output["declared_config_hash"] = envelope_hash
        output["effective_config_hash"] = envelope_hash
        output["effective_config_hash_semantic"] = "DEVELOPMENT_DECLARED_ENVELOPE_NOT_FORMAL_EFFECTIVE_CONFIG"
    else:
        config_report = validate_effective_config(config, str(row["condition_id"]))
        output["effective_config_hash"] = config_report["effective_config_hash"]
        output["observer_policy_hash"] = canonical_hash(dict(require_mapping(config.get("observer"), "runtime observer")))
        output["delay_policy_hash"] = canonical_hash(dict(require_mapping(config.get("delay"), "runtime delay")))
    for file_key, hash_key in (("robot_model_file", "robot_model_hash"), ("physical_parameter_file", "physical_parameter_hash")):
        raw = assets.get(file_key)
        if raw is not None:
            require(isinstance(raw, str) and raw and Path(raw).is_file(), f"run asset missing {file_key}")
            output[hash_key] = sha256_file(Path(raw))
    if row.get("condition_id") == "FixedProfile":
        require("fixed_profile" in assets, "FixedProfile run requires a frozen profile registry entry")
        profile_report = validate_fixed_profile(assets["fixed_profile"])
        output["profile_hash"] = profile_report["profile_hash"]
        output["tracker_config_hash"] = str(assets["fixed_profile"]["tracker_config_hash"])
    if row.get("formal") is True:
        geometry_raw = assets.get("world_geometry_file")
        require(
            isinstance(geometry_raw, str) and geometry_raw and Path(geometry_raw).is_file(),
            "formal runtime requires the frozen world_geometry_file",
        )
        output["world_geometry_hash"] = sha256_file(Path(geometry_raw))
        truth = require_mapping(spec.get("liquid_plant_capability"), "formal runtime plant capability")
        for path_key, source_hash_key, output_hash_key in (
            ("plant_code_path", "plant_code_hash", "liquid_plant_code_hash"),
            ("plant_parameter_path", "plant_parameter_hash", "plant_parameter_hash"),
            ("plant_input_schema_path", "plant_input_schema_hash", "plant_input_schema_hash"),
            ("plant_output_schema_path", "plant_output_schema_hash", "plant_output_schema_hash"),
        ):
            raw = truth.get(path_key)
            require(isinstance(raw, str) and raw and Path(raw).is_file(), f"formal runtime plant asset missing {path_key}")
            actual_hash = sha256_file(Path(raw))
            require(actual_hash == truth.get(source_hash_key), f"formal runtime plant {source_hash_key} mismatch")
            output[output_hash_key] = actual_hash
    return output


def validate_formal_runtime_bindings(
    freeze: Mapping[str, Any],
    row: Mapping[str, Any],
    assets: Mapping[str, Any],
    hashes: Mapping[str, str],
    effective_config: Any,
    truth_capability: Any,
    seed_bundle: Mapping[str, Any],
) -> None:
    """Prove the one-row launch inputs are the exact files in the frozen release."""
    path_registry = require_mapping(freeze.get("paths"), "formal paths")
    path_item = require_mapping(path_registry.get(row["path_id"]), f"formal path {row['path_id']}")
    require(hashes.get("path_hash") == path_item.get("sim_path_hash"), "runtime path file is not the frozen transformed JSON")
    simulator = require_mapping(freeze.get("simulator_assets"), "formal simulator assets")
    require(hashes.get("map_hash") == simulator.get("map_hash"), "runtime map hash differs from frozen simulator map")
    require(hashes.get("world_hash") == simulator.get("world_hash"), "runtime world hash differs from frozen simulator world")
    require(
        hashes.get("world_geometry_hash") == simulator.get("world_geometry_hash"),
        "runtime clearance geometry hash differs from frozen simulator geometry",
    )
    require(hashes.get("robot_model_hash") == simulator.get("robot_model_hash"), "runtime robot model hash differs from frozen simulator asset")
    containers = require_mapping(freeze.get("containers"), "formal containers")
    container = require_mapping(containers.get(row["container_id"]), f"formal container {row['container_id']}")
    require(hashes.get("physical_parameter_hash") == container.get("physical_parameter_hash"), "runtime container parameters differ from frozen container")
    configs = require_mapping(freeze.get("effective_configs"), "formal configs")
    config = require_mapping(configs.get(row["condition_id"]), f"formal config {row['condition_id']}")
    require(hashes.get("effective_config_hash") == config.get("effective_config_hash"), "runtime effective config differs from frozen condition")
    require(hashes.get("observer_policy_hash") == canonical_hash(dict(require_mapping(config["effective_config"].get("observer"), "formal observer"))), "runtime observer policy differs from frozen condition")
    require(hashes.get("delay_policy_hash") == canonical_hash(dict(require_mapping(config["effective_config"].get("delay"), "formal delay"))), "runtime delay policy differs from frozen condition")
    require(config.get("effective_config") == effective_config, "runtime effective config content differs from formal registry")
    if row["condition_id"] == "FixedProfile":
        stratum = f"{row['path_id']}_{row['container_id']}"
        expected_profile = require_mapping(require_mapping(freeze.get("fixed_profiles"), "formal profiles").get(stratum), f"formal profile {stratum}")
        actual_profile = require_mapping(assets.get("fixed_profile"), "runtime FixedProfile registry")
        require(actual_profile.get("profile_hash") == expected_profile.get("profile_hash"), "runtime FixedProfile hash differs from frozen stratum")
        require(actual_profile.get("tracker_config_hash") == expected_profile.get("tracker_config_hash"), "runtime FixedProfile tracker differs from frozen stratum")
    require(canonical_hash(truth_capability) == canonical_hash(freeze.get("liquid_plant_capability")), "runtime plant capability differs from frozen plant release")
    expected_row_hashes = expected_row_frozen_asset_hashes(freeze, str(row["path_id"]), str(row["container_id"]), str(row["condition_id"]))
    for key, expected in expected_row_hashes.items():
        if key == "sim_path_hash":
            require(hashes.get("path_hash") == expected, "runtime path hash differs from formal planned row")
        elif key == "source_path_hash" or key == "transform_hash":
            continue
        else:
            require(hashes.get(key) == expected, f"runtime {key} differs from formal planned row")
    require(row.get("frozen_asset_hashes") == expected_row_hashes, "formal planned row frozen asset hashes differ from freeze")
    require(seed_bundle.get("seed_bundle_id") == row.get("seed_bundle_id") and seed_bundle.get("seed_bundle_hash") == row.get("seed_bundle_hash"), "runtime seed bundle differs from planned row")


def validate_runtime_ack(
    ack_path: Path,
    case_manifest_hash: str,
    hashes: Mapping[str, str],
    seed_bundle: Mapping[str, Any],
    formal: bool,
    expected_schema_hash: Optional[str] = None,
) -> Mapping[str, Any]:
    """Require launch-side readback; metadata alone never proves inputs were consumed."""
    ack = require_mapping(read_json(ack_path), "runtime input acknowledgement")
    require(ack.get("status") == "PASS", "runtime input acknowledgement is not PASS")
    require(ack.get("case_manifest_hash") == case_manifest_hash, "runtime acknowledgement is for another case manifest")
    require(ack.get("effective_config_hash") == hashes.get("effective_config_hash"), "runtime acknowledgement effective config mismatch")
    require(ack.get("map_hash") == hashes.get("map_hash") and ack.get("world_hash") == hashes.get("world_hash") and ack.get("path_hash") == hashes.get("path_hash"), "runtime acknowledgement map/world/path mismatch")
    require(ack.get("seed_bundle_hash") == seed_bundle.get("seed_bundle_hash"), "runtime acknowledgement seed bundle mismatch")
    require(ack.get("seed_trace_hashes") == {name: seed_bundle["traces"][name]["trace_hash"] for name in SEED_STREAMS}, "runtime acknowledgement seed trace mismatch")
    if "profile_hash" in hashes:
        require(ack.get("profile_hash") == hashes["profile_hash"], "runtime acknowledgement FixedProfile mismatch")
    if formal:
        require(ack.get("formal") is True, "formal runtime acknowledgement is not marked formal")
        if expected_schema_hash is not None:
            require(ack.get("schema_hash") == expected_schema_hash, "formal runtime acknowledgement schema differs from frozen launch contract")
        consumed = require_mapping(ack.get("consumed_hashes"), "formal runtime acknowledgement consumed_hashes")
        require(dict(consumed) == dict(hashes), "formal runtime acknowledgement does not prove every frozen input hash was consumed")
    return ack


def validate_motion_stop_ack(ack_path: Path, case_manifest_hash: str, expected_schema_hash: Optional[str] = None) -> Mapping[str, Any]:
    """Require an explicit command-zero/cmd-gate acknowledgement before tail."""
    ack = require_mapping(read_json(ack_path), "motion stop acknowledgement")
    require(ack.get("ack_type") == "SMPCC_SIM_MOTION_STOP_ACK", "motion stop acknowledgement has wrong ack_type")
    require(ack.get("status") == "PASS", "motion stop acknowledgement is not PASS")
    require(ack.get("case_manifest_hash") == case_manifest_hash, "motion stop acknowledgement is for another case")
    require(ack.get("command_zero_confirmed") is True, "motion stop acknowledgement does not prove zero command")
    require(ack.get("cmd_gate_state") in {"STOPPED", "ZERO_HELD"}, "motion stop acknowledgement cmd-gate state is not stopped")
    if expected_schema_hash is not None:
        require(ack.get("schema_hash") == expected_schema_hash, "motion stop acknowledgement schema differs from frozen launch contract")
    return ack


def validate_motion_release_ack(ack_path: Path, case_manifest_hash: str, expected_schema_hash: Optional[str] = None) -> Mapping[str, Any]:
    """Require runtime proof that the frozen assignment produced base motion."""
    ack = require_mapping(read_json(ack_path), "motion release acknowledgement")
    require(ack.get("ack_type") == "SMPCC_SIM_MOTION_RELEASE_ACK", "motion release acknowledgement has wrong ack_type")
    require(ack.get("status") == "PASS", "motion release acknowledgement is not PASS")
    require(ack.get("case_manifest_hash") == case_manifest_hash, "motion release acknowledgement is for another case")
    require(ack.get("assignment_consumed") is True, "motion release acknowledgement does not prove assignment consumption")
    require(ack.get("motion_backend_ready") is True and ack.get("base_motion_observed") is True, "motion release acknowledgement does not prove base motion")
    require(isinstance(ack.get("first_effective_motion_utc"), str) and ack["first_effective_motion_utc"], "motion release acknowledgement lacks first_effective_motion_utc")
    require(is_sha256(ack.get("base_motion_sample_hash")), "motion release acknowledgement lacks base_motion_sample_hash")
    if expected_schema_hash is not None:
        require(ack.get("schema_hash") == expected_schema_hash, "motion release acknowledgement schema differs from frozen launch contract")
    return ack


def validate_development_motion_event(path: Path, case_manifest_hash: str) -> Mapping[str, Any]:
    """Bind a non-formal first-executed-motion marker to this exact case.

    This is deliberately weaker than the frozen formal release acknowledgement:
    it only gives development H0 runs an honest wall-clock start for their
    60-second trajectory timeout.  Formal rows still require the frozen
    motion-release acknowledgement above.
    """
    event = require_mapping(read_json(path), "development motion event")
    require(event.get("record_type") == "SMPCC_SIM_DEV_MOTION_EVENT", "development motion event has wrong record_type")
    require(event.get("status") == "PASS", "development motion event is not PASS")
    require(event.get("case_manifest_hash") == case_manifest_hash, "development motion event belongs to another case")
    require(event.get("observed_topic") == "/odom", "development motion event must observe executed /odom")
    require(isinstance(event.get("first_effective_motion_utc"), str) and event["first_effective_motion_utc"], "development motion event lacks first_effective_motion_utc")
    require(isinstance(event.get("first_effective_motion_ros_time_sec"), (int, float)) and math.isfinite(float(event["first_effective_motion_ros_time_sec"])), "development motion event lacks finite ROS time")
    return event


def validate_development_goal_event(path: Path, case_manifest_hash: str) -> Mapping[str, Any]:
    """Validate an asynchronous H0 GOAL_REACHED receipt without promoting it."""
    event = require_mapping(read_json(path), "development goal event")
    require(event.get("record_type") == "SMPCC_SIM_DEV_GOAL_EVENT", "development goal event has wrong record_type")
    require(event.get("status") == "PASS" and event.get("controller_status") == "GOAL_REACHED", "development goal event is not exact GOAL_REACHED")
    require(event.get("case_manifest_hash") == case_manifest_hash, "development goal event belongs to another case")
    require(isinstance(event.get("first_arrival_utc"), str) and event["first_arrival_utc"], "development goal event lacks first_arrival_utc")
    require(isinstance(event.get("first_arrival_ros_time_sec"), (int, float)) and math.isfinite(float(event["first_arrival_ros_time_sec"])), "development goal event lacks finite ROS time")
    return event


def minimal_h0_row() -> Dict[str, Any]:
    bundle = make_seed_bundle("SIM-DEV-H0-fixture-seed-v1", "SIM-DEV-H0_SMOKE", 1)
    return {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": "SIM-DEV-H0-SMOKE-v1",
        "formal": False,
        "evidence_class": DEVELOPMENT_EVIDENCE_CLASS,
        "stage": "SIM-DEV-H0_SMOKE",
        "planned_row_id": "SIM-DEV-H0_SMOKE_H0_C1_Bsmooth_b01",
        "block_id": "b01",
        "order_position": 1,
        "condition_id": "Bsmooth",
        "method_backend": "online_mpcc",
        "path_id": "H0",
        "container_id": "C1",
        "planned_block_segment_id": "SIM-DEV-H0_SMOKE_b01_seg01",
        "seed_bundle_id": bundle["seed_bundle_id"],
        "seed_bundle_hash": bundle["seed_bundle_hash"],
        "fixed_denominator": {"n_plan_stage": 1, "n_plan_condition": 1, "n_block_plan": 1, "n_plan_total": 1},
    }


def load_stage_report(entry: Any, expected_type: str, expected_stage: str, master_hash: str, expected_plan: int) -> Tuple[Mapping[str, Any], str]:
    entry = require_mapping(entry, "stage-entry evidence")
    path = validate_bound_file(entry, "report_path", "report_hash", "stage-entry report")
    report = require_mapping(read_json(path), "stage-entry report")
    require(report.get("report_type") == expected_type, f"stage-entry report type must be {expected_type}")
    require(report.get("status") == "PASS", "stage-entry report is not PASS")
    require(report.get("stage") == expected_stage, f"stage-entry report stage must be {expected_stage}")
    require(report.get("master_hash") == master_hash, "stage-entry report belongs to another planned matrix")
    require(report.get("N_plan") == expected_plan, f"stage-entry report N_plan must be {expected_plan}")
    require(report.get("all_attempts_classified") is True and report.get("retry_chain_valid") is True, "stage-entry closure is incomplete")
    return report, str(entry["report_hash"])


def stage_entry_context(master: Mapping[str, Any]) -> Tuple[Mapping[str, Any], Mapping[str, Any], Mapping[str, Any], str]:
    """Load only the immutable formal artifacts needed to enter Stage II."""
    freeze_owner = {
        "formal_freeze_path": master.get("formal_freeze_path"),
        "formal_freeze_file_hash": master.get("formal_freeze_file_hash"),
    }
    freeze_path = validate_bound_file(freeze_owner, "formal_freeze_path", "formal_freeze_file_hash", "formal master freeze")
    freeze = require_mapping(read_json(freeze_path), "formal stage-entry freeze")
    freeze_hash = canonical_hash(freeze)
    require(master.get("freeze_hash") == freeze_hash, "formal stage-entry master/freeze hash mismatch")
    policy = validate_frozen_stage_entry_policy(freeze.get("stage_entry_policy"))
    ledger = validate_formal_dataset_ledger(freeze.get("dataset_ledger"), freeze)
    return freeze, policy, ledger, freeze_hash


def validate_stage_report_binding(
    report: Mapping[str, Any],
    report_type: str,
    master: Mapping[str, Any],
    policy: Mapping[str, Any],
    ledger: Mapping[str, Any],
    freeze_hash: str,
) -> None:
    """Make a stage report an anchored ledger/policy artifact, not a bare PASS."""
    rule = require_mapping(require_mapping(policy.get("reports"), "stage-entry policy reports").get(report_type), f"stage-entry policy {report_type}")
    require(report.get("formal_freeze_hash") == freeze_hash, f"{report_type} is not bound to the formal freeze")
    registry = require_mapping(master.get("contrast_registry"), "formal master contrast registry")
    require(report.get("contrast_registry_hash") == registry.get("registry_hash"), f"{report_type} contrast registry mismatch")
    require(report.get("stage_entry_policy_id") == policy.get("policy_id"), f"{report_type} stage-entry policy ID mismatch")
    require(report.get("stage_entry_policy_hash") == policy.get("policy_hash"), f"{report_type} stage-entry policy hash mismatch")
    require(report.get("rule_hash") == rule.get("rule_hash"), f"{report_type} rule hash differs from frozen stage-entry policy")
    require(report.get("validator_hash") == rule.get("validator_hash"), f"{report_type} validator hash differs from frozen stage-entry policy")
    head_hash = report.get("dataset_index_head_hash")
    require(is_sha256(head_hash), f"{report_type} lacks dataset_index_head_hash")
    index_path_value = report.get("dataset_index_path")
    require(isinstance(index_path_value, str) and index_path_value, f"{report_type} lacks dataset_index_path")
    index_path = Path(index_path_value)
    expected_index = Path(str(ledger["ledger_root"])) / "dataset_index.jsonl"
    require(index_path.resolve() == expected_index.resolve(), f"{report_type} dataset index differs from frozen ledger")
    records = load_dataset_index(index_path)
    require(any(item.get("entry_hash") == head_hash for item in records), f"{report_type} dataset index head is absent from append-only ledger")


def load_stage_gate_report(
    entry: Any,
    expected_type: str,
    expected_stage: str,
    master: Mapping[str, Any],
    policy: Mapping[str, Any],
    ledger: Mapping[str, Any],
    freeze_hash: str,
    expected_status: str = "PASS",
) -> Tuple[Mapping[str, Any], str]:
    entry = require_mapping(entry, f"{expected_type} evidence")
    path = validate_bound_file(entry, "report_path", "report_hash", expected_type)
    report = require_mapping(read_json(path), expected_type)
    require(report.get("report_type") == expected_type and report.get("status") == expected_status, f"{expected_type} is not {expected_status}")
    require(report.get("stage") == expected_stage, f"{expected_type} stage mismatch")
    require(report.get("master_hash") == master.get("master_hash"), f"{expected_type} belongs to another planned matrix")
    validate_stage_report_binding(report, expected_type, master, policy, ledger, freeze_hash)
    return report, str(entry["report_hash"])


def validate_stage_entry(master: Mapping[str, Any], row: Mapping[str, Any], evidence: Any) -> None:
    """Stage II cannot be entered by a shell flag: immutable reports are mandatory."""
    stage = row.get("stage")
    if stage == "SIM-S1_CORE":
        require(evidence is None or evidence == {}, "S1 cannot consume future stage-entry evidence")
        return
    evidence = require_mapping(evidence, "stage-entry evidence")
    master_hash = str(master.get("master_hash"))
    _freeze, policy, ledger, freeze_hash = stage_entry_context(master)
    stage1, stage1_hash = load_stage_report(evidence.get("stage1_closure"), "SIM_S1_CLOSURE", "SIM-S1_CORE", master_hash, 40)
    validate_stage_report_binding(stage1, "SIM_S1_CLOSURE", master, policy, ledger, freeze_hash)
    extension, extension_hash = load_stage_gate_report(
        evidence.get("stage1_extension_gate"),
        STAGE1_EXTENSION_REPORT_TYPE,
        "SIM-S1_CORE",
        master,
        policy,
        ledger,
        freeze_hash,
    )
    require(extension.get("stage1_closure_hash") == stage1_hash, "S1 extension gate is not bound to the Stage-I closure")
    gates = require_mapping(extension.get("gates"), "S1 extension gates")
    required_gates = require_mapping(policy["reports"][STAGE1_EXTENSION_REPORT_TYPE], "S1 extension policy").get("required_gate_ids")
    require(set(gates) == set(required_gates) and all(value == "PASS" for value in gates.values()), "S1 extension gate does not PASS every frozen scientific criterion")
    if stage == "SIM-S2A_SELECTIVITY":
        require(
            not evidence.get("stage2a_completion") and not evidence.get("s2a_selectivity") and not evidence.get("stage2b_trigger"),
            "S2A cannot consume future Stage-II evidence",
        )
        return
    require(stage == "SIM-S2B_TRANSFER", "unknown formal stage entry")
    stage2a, stage2a_hash = load_stage_report(evidence.get("stage2a_completion"), "SIM_S2A_CLOSURE", "SIM-S2A_SELECTIVITY", master_hash, 24)
    validate_stage_report_binding(stage2a, "SIM_S2A_CLOSURE", master, policy, ledger, freeze_hash)
    require(stage2a.get("stage1_extension_gate_hash") == extension_hash, "S2A closure is not bound to the passed Stage-I extension gate")
    selectivity, selectivity_hash = load_stage_gate_report(
        evidence.get("s2a_selectivity"),
        STAGE2A_SELECTIVITY_REPORT_TYPE,
        "SIM-S2A_SELECTIVITY",
        master,
        policy,
        ledger,
        freeze_hash,
        expected_status="ANALYZED",
    )
    require(
        selectivity.get("stage1_closure_hash") == stage1_hash
        and selectivity.get("stage1_extension_gate_hash") == extension_hash
        and selectivity.get("stage2a_closure_hash") == stage2a_hash,
        "S2A selectivity report is not bound to Stage-I/S2A closure evidence",
    )
    allowed_selectivity = require_mapping(policy["reports"][STAGE2A_SELECTIVITY_REPORT_TYPE], "S2A selectivity policy").get("allowed_selectivity_statuses")
    require(selectivity.get("selectivity_status") in allowed_selectivity, "S2A selectivity status is not pre-authorized for transfer")
    trigger, _trigger_hash = load_stage_gate_report(
        evidence.get("stage2b_trigger"),
        STAGE2B_TRIGGER_REPORT_TYPE,
        "SIM-S2B_TRANSFER",
        master,
        policy,
        ledger,
        freeze_hash,
    )
    require(
        trigger.get("stage1_extension_gate_hash") == extension_hash
        and trigger.get("stage2a_closure_hash") == stage2a_hash
        and trigger.get("s2a_selectivity_hash") == selectivity_hash,
        "S2B trigger is not bound to the frozen S1/S2A evidence chain",
    )


def run_single_row(row: Mapping[str, Any], spec: Mapping[str, Any], output_root: Path, sim_root: Path) -> Dict[str, Any]:
    """Run one case with fresh-master probes and a recorder-before-motion state machine."""
    row = require_mapping(row, "row")
    spec = require_mapping(spec, "run spec")
    require(row.get("condition_id") in CONDITION_BACKENDS, "unknown condition ID")
    require(row.get("method_backend") == CONDITION_BACKENDS[row.get("condition_id")], "row backend does not match condition")
    if row.get("condition_id") == "FixedProfile":
        require(spec.get("profile_generation_command") in (None, "", []), "FixedProfile runner forbids case-local profile generation")
        require(spec.get("runtime_profile_regeneration") in (None, False), "FixedProfile runtime regeneration is forbidden")
    formal = row.get("formal") is True
    formal_master_document: Optional[Mapping[str, Any]] = None
    formal_master_path: Optional[Path] = None
    formal_master_file_hash: Optional[str] = None
    formal_freeze_hash: Optional[str] = None
    formal_freeze_path: Optional[Path] = None
    formal_freeze_file_hash: Optional[str] = None
    formal_retry_classifier: Optional[Mapping[str, Any]] = None
    formal_launch_contract: Optional[Mapping[str, Any]] = None
    formal_runtime_backend: Optional[Mapping[str, Any]] = None
    formal_dataset_ledger: Optional[Mapping[str, Any]] = None
    dry_run = spec.get("dry_run") is True
    require(not (formal and dry_run), "formal execution cannot be dry-run")
    require(not has_forbidden_w5(row), "rejected W5/W5_S10 cannot be run as Bslosh")
    require(not has_forbidden_w5(spec), "runtime spec attempts to invoke rejected W5/W5_S10")
    if formal:
        freeze = require_mapping(spec.get("formal_freeze"), "formal freeze")
        formal_report = validate_formal_freeze(freeze)
        require(formal_report["status"] == "PASS", "FORMAL_SIM_NO_GO: " + "; ".join(formal_report["errors"]))
        master_path_value = spec.get("formal_master")
        require(isinstance(master_path_value, str) and master_path_value, "formal runner requires immutable formal_master planned-row document")
        require(Path(master_path_value).is_absolute(), "formal runner formal_master path must be absolute")
        master_path = Path(master_path_value).resolve()
        master_document = require_mapping(read_json(master_path), "formal master")
        master_report = validate_master(master_document, require_formal=True)
        require(master_report["status"] == "PASS", "formal master validation failed: " + "; ".join(master_report["errors"]))
        require_source_separated_r8_execution(freeze, master_document)
        require(master_document.get("freeze_hash") == formal_report.get("freeze_hash"), "formal row/master freeze hash mismatch")
        bound_freeze_path = Path(str(master_document.get("formal_freeze_path", "")))
        require(bound_freeze_path.is_absolute(), "formal master bound freeze path must be absolute")
        require(bound_freeze_path.is_file() and sha256_file(bound_freeze_path) == master_document.get("formal_freeze_file_hash"), "formal master bound freeze artifact is missing or changed")
        require(canonical_hash(read_json(bound_freeze_path)) == canonical_hash(freeze), "formal runner freeze differs from formal master bound freeze")
        require(canonical_hash(master_document.get("randomization_tables")) == canonical_hash(require_mapping(freeze.get("randomization_tables"), "formal frozen randomization tables")), "formal master randomization differs from freeze")
        require(canonical_hash(master_document.get("seed_bundles")) == canonical_hash(require_mapping(freeze.get("seed_bundles"), "formal frozen seed bundles")), "formal master seed bundles differ from freeze")
        require(canonical_hash(master_document.get("contrast_registry")) == canonical_hash(validate_frozen_contrast_registry(freeze.get("contrast_registry"))["registry"]), "formal master contrast registry differs from freeze")
        matching_rows = [item for item in master_document["planned_rows"] if item.get("planned_row_id") == row.get("planned_row_id")]
        require(len(matching_rows) == 1 and canonical_hash(matching_rows[0]) == canonical_hash(row), "formal row is not the immutable master planned-row assignment")
        validate_stage_entry(master_document, row, spec.get("stage_entry_evidence"))
        formal_master_document = master_document
        formal_master_path = master_path
        formal_master_file_hash = sha256_file(master_path)
        formal_freeze_hash = str(formal_report["freeze_hash"])
        formal_freeze_path = bound_freeze_path.resolve()
        formal_freeze_file_hash = sha256_file(formal_freeze_path)
        formal_retry_classifier = validate_frozen_retry_classifier(freeze.get("retry_classifier"))
        formal_launch_contract = validate_frozen_runtime_launch_contract(freeze.get("runtime_launch_contract"))
        formal_runtime_backend = validate_formal_runtime_backend_manifest(
            freeze.get("formal_runtime_backend"),
            freeze,
            formal_launch_contract,
        )
        formal_dataset_ledger = validate_formal_dataset_ledger(freeze.get("dataset_ledger"), freeze)
    else:
        require(row.get("evidence_class") in {FIXTURE_EVIDENCE_CLASS, DEVELOPMENT_EVIDENCE_CLASS}, "development/fixture row must carry non-formal label")
        if not dry_run:
            require(
                isinstance(row.get("stage"), str)
                and row["stage"].startswith("SIM-DEV-")
                and row.get("path_id") in DEVELOPMENT_RUNTIME_PATH_IDS,
                "non-formal actual execution is restricted to SIM-DEV H0/H0b/H0s rows; fixture SIM-S1/S2 rows are dry-run only",
            )
            require(
                row.get("evidence_class") == DEVELOPMENT_EVIDENCE_CLASS,
                "fixture rows are dry-run only; actual non-formal execution requires DEVELOPMENT_SMOKE_NOT_FORMAL",
            )
    attempt_id = str(spec.get("attempt_id") or f"{row['planned_row_id']}_r01")
    require(attempt_prefix(attempt_id) == str(row["planned_row_id"]), "attempt ID must derive from planned row ID")
    attempt_number = parse_attempt_number(attempt_id)
    retry_authorization: Optional[Mapping[str, Any]] = None
    if attempt_number > 1:
        auth_path = spec.get("retry_authorization")
        require(isinstance(auth_path, str) and auth_path, "r02+ requires an explicit retry authorization")
        if formal:
            assert formal_retry_classifier is not None
            retry_authorization = validate_retry_authorization(
                Path(auth_path),
                row,
                attempt_id,
                expected_classifier_id=str(formal_retry_classifier["classifier_id"]),
                expected_classifier_rule_hash=str(formal_retry_classifier["classifier_rule_hash"]),
                expected_classifier_manifest_hash=str(formal_retry_classifier["classifier_manifest_hash"]),
                expected_verifier_id=str(formal_retry_classifier["verifier_id"]),
                expected_verifier_hash=str(formal_retry_classifier["verifier_hash"]),
                expected_reason_codes=list(formal_retry_classifier["reason_codes"]),
                expected_max_retries=int(formal_retry_classifier["max_retries_per_row"]),
            )
        else:
            retry_authorization = validate_retry_authorization(Path(auth_path), row, attempt_id)
    else:
        require(not spec.get("retry_authorization"), "r01 cannot consume a retry authorization")

    output_root = ensure_within(output_root, sim_root, "result output root")
    if formal:
        assert formal_dataset_ledger is not None
        require(sim_root.resolve() == DEFAULT_SIM_ROOT.resolve(), "formal runner sim_root must be the frozen isolation root")
        require(output_root.resolve() == Path(str(formal_dataset_ledger["ledger_root"])).resolve(), "formal output_root differs from frozen dataset ledger root")
        existing_ledger = load_dataset_index(output_root / "dataset_index.jsonl")
        for item in existing_ledger:
            require(item.get("dataset_ledger_id") == formal_dataset_ledger["ledger_id"], "formal dataset index mixes another frozen ledger")
            require(item.get("dataset_ledger_identity_hash") == formal_dataset_ledger["ledger_identity_hash"], "formal dataset index ledger identity mismatch")
            require(item.get("formal_master_hash") == formal_master_document.get("master_hash"), "formal dataset index mixes another master")
        same_row_records = [item for item in existing_ledger if item.get("planned_row_id") == row.get("planned_row_id")]
        if attempt_number == 1:
            require(not same_row_records, "formal planned row already has an attempt in the frozen ledger")
    if spec.get("enforce_failure_pause", True) is True:
        failure_streak = consecutive_method_failure_rows(output_root / "dataset_index.jsonl", str(row["stage"]), str(row["condition_id"]))
        require(failure_streak < 3, f"condition/stage paused after {failure_streak} consecutive method-failure rows; audit required before another attempt")
    if retry_authorization is not None:
        prior_records = load_dataset_index(output_root / "dataset_index.jsonl")
        prior_id = str(retry_authorization["previous_attempt_id"])
        matching_prior = [item for item in prior_records if item.get("attempt_id") == prior_id]
        require(len(matching_prior) == 1, "retry previous attempt is absent from this append-only dataset index")
        require(matching_prior[0].get("attempt_manifest_hash") == retry_authorization.get("previous_attempt_manifest_hash"), "retry previous index hash differs from authorization evidence")
        actual_segment = str(spec.get("actual_block_segment_id", row["planned_block_segment_id"]))
        previous_manifest = read_attempt_manifest(Path(str(retry_authorization["previous_attempt_manifest"])))
        require(actual_segment == row["planned_block_segment_id"] == previous_manifest.get("actual_block_segment_id"), "authorized retry must remain in the original planned block segment")
    case_dir = output_root / str(row["stage"]) / str(row["block_id"]) / f"p{int(row['order_position']):02d}_{row['condition_id']}" / f"r{attempt_number:02d}"
    require(not case_dir.exists(), f"attempt output already exists: {case_dir}")
    if formal:
        assert formal_runtime_backend is not None
        backend_spec = require_mapping(spec.get("formal_runtime_backend"), "formal runtime backend spec binding")
        require(
            backend_spec.get("backend_id") == formal_runtime_backend["backend_id"]
            and backend_spec.get("backend_hash") == formal_runtime_backend["backend_hash"]
            and backend_spec.get("manifest_path") == formal_runtime_backend["backend_manifest_path"]
            and backend_spec.get("manifest_file_hash") == formal_runtime_backend["backend_manifest_hash"]
            and backend_spec.get("case_artifacts") == formal_runtime_backend["case_artifacts"],
            "formal run spec backend binding differs from frozen backend manifest",
        )
        expected_artifacts = require_mapping(formal_runtime_backend.get("case_artifacts"), "formal backend case artifacts")
        for spec_key, artifact_key in (
            ("recorder_artifact", "recorder_artifact"),
            ("runtime_ack_path", "runtime_ack"),
            ("motion_release_ack_path", "motion_release_ack"),
            ("motion_stop_ack_path", "motion_stop_ack"),
        ):
            expected_path = str((case_dir / str(expected_artifacts[artifact_key])).resolve())
            require(spec.get(spec_key) == expected_path, f"formal {spec_key} differs from frozen case-local backend artifact")
    development_firewall: Optional[Mapping[str, Any]] = None
    development_firewall_snapshot_commands: Optional[Mapping[str, Any]] = None
    development_firewall_not_live = False
    if formal:
        require(formal_master_document is not None, "formal master is unavailable")
        seed_bundle = require_mapping(formal_master_document.get("seed_bundles"), "formal seed bundles").get(f"{row['stage']}:{row['block_id']}")
    else:
        seed_bundle = spec.get("seed_bundle")
        if seed_bundle is None and row.get("stage") == "SIM-DEV-H0_SMOKE":
            seed_bundle = make_seed_bundle("SIM-DEV-H0-fixture-seed-v1", "SIM-DEV-H0_SMOKE", 1)
    seed_bundle = require_mapping(seed_bundle, "runtime seed bundle")
    seed_report = validate_seed_bundle(seed_bundle)
    require(seed_bundle.get("seed_bundle_id") == row.get("seed_bundle_id") and seed_bundle.get("seed_bundle_hash") == row.get("seed_bundle_hash"), "row does not bind the runtime seed bundle")
    assets = require_mapping(spec.get("assets"), "run assets")
    config_asset_path = runtime_config_asset_path(assets, row)
    hashes = asset_hashes(spec, row)
    effective_config = read_json(config_asset_path)
    require(not has_forbidden_w5(effective_config), "runtime effective config attempts to revive rejected W5/W5_S10")
    development_h0_runtime = (
        not formal
        and isinstance(row.get("stage"), str)
        and row["stage"].startswith("SIM-DEV-")
        and row.get("path_id") in DEVELOPMENT_RUNTIME_PATH_IDS
    )
    declared_config_path: Optional[Path] = None
    declared_config: Optional[Mapping[str, Any]] = None
    if development_h0_runtime:
        declared_value = assets.get("declared_config_file")
        if not dry_run:
            require(isinstance(declared_value, str) and declared_value, "actual H0 development run requires assets.declared_config_file")
            require(spec.get("effective_config_readback_command") is not None, "actual H0 development run requires effective_config_readback_command")
            require(isinstance(spec.get("effective_config_readback_path"), str) and spec.get("effective_config_readback_path"), "actual H0 development run requires effective_config_readback_path")
        if isinstance(declared_value, str) and declared_value:
            declared_config_path = runtime_config_asset_path(assets, row)
            declared_config = effective_config
            require(hashes.get("declared_config_hash") == canonical_hash(dict(declared_config)), "development declared config hash mismatch")
    if retry_authorization is not None:
        require(retry_authorization.get("config_hash") == hashes.get("effective_config_hash"), "retry cannot change effective config")
        require(retry_authorization.get("profile_hash") == hashes.get("profile_hash"), "retry cannot change FixedProfile")
        require(retry_authorization.get("seed_bundle_hash") == seed_bundle.get("seed_bundle_hash"), "retry cannot change seed bundle or trace")
    truth = validate_truth_capability(spec.get("liquid_plant_capability"))
    development_liquid_plant = development_h0_liquid_plant_opt_in(
        spec.get("liquid_plant_capability")
    )
    if formal:
        require(
            spec.get("development_firewall") is None
            and spec.get("development_firewall_snapshot_commands") is None,
            "formal runner refuses H0 development firewall fields",
        )
    elif development_liquid_plant:
        require(
            development_h0_runtime,
            "development liquid-plant firewall is restricted to SIM-DEV H0/H0b/H0s",
        )
        if dry_run:
            # A fixture cannot claim a live ROS-master observation.  Preserve
            # the existing dry-run matrix tests, but make the absence explicit
            # in the manifests rather than fabricating PASS snapshots.
            require(
                spec.get("development_firewall") is None
                and spec.get("development_firewall_snapshot_commands") is None,
                "dry-run development liquid-plant evidence must be NOT_LIVE and cannot inject firewall snapshots",
            )
            development_firewall_not_live = True
        else:
            development_firewall = validate_development_h0_firewall_contract(
                spec.get("development_firewall"), case_dir
            )
            commands = require_mapping(
                spec.get("development_firewall_snapshot_commands"),
                "development H0 firewall snapshot commands",
            )
            require(
                set(commands) == set(DEVELOPMENT_H0_FIREWALL_CHECKPOINTS),
                "development H0 firewall snapshot command set is incomplete",
            )
            for checkpoint in DEVELOPMENT_H0_FIREWALL_CHECKPOINTS:
                command = command_from_spec(
                    commands[checkpoint], f"development_firewall_snapshot_commands.{checkpoint}"
                )
                require(
                    "firewall-snapshot" in command,
                    f"development H0 firewall {checkpoint} command is not a snapshot command",
                )
            development_firewall_snapshot_commands = commands
    else:
        require(
            spec.get("development_firewall") is None
            and spec.get("development_firewall_snapshot_commands") is None,
            "H0 firewall fields require the explicit UNVALIDATED development liquid-plant opt-in",
        )
    if formal:
        frozen_controller_nodes = list(require_mapping(require_mapping(spec.get("formal_freeze"), "formal freeze").get("controller_firewall"), "formal controller firewall").get("controller_nodes", []))
        supplied_nodes = spec.get("controller_nodes")
        require(supplied_nodes is None or sorted(supplied_nodes) == sorted(frozen_controller_nodes), "runtime controller node set differs from frozen firewall")
        controller_nodes = frozen_controller_nodes
    else:
        controller_nodes = list(spec.get("controller_nodes", []))
    if formal:
        require(truth["eligible"], "formal physical primary requires independent plant truth")
        validate_formal_runtime_bindings(require_mapping(spec.get("formal_freeze"), "formal freeze"), row, require_mapping(spec.get("assets"), "run assets"), hashes, effective_config, spec.get("liquid_plant_capability"), seed_bundle)
        assert formal_launch_contract is not None
        for field in FORMAL_RUNTIME_COMMAND_FIELDS:
            require(command_from_spec(spec.get(field), field) == formal_launch_contract["commands"][field], f"runtime {field} differs from frozen launch contract")
        require(float(spec.get("startup_timeout_sec", 60.0)) == formal_launch_contract["startup_timeout_sec"], "runtime startup_timeout_sec differs from frozen launch contract")
    settle_sec = float(spec.get("settle_sec", 30.0))
    timeout_sec = float(spec.get("goal_timeout_sec", 60.0))
    tail_sec = float(spec.get("tail_sec", 5.0))
    post_shutdown_sec = float(spec.get("post_shutdown_sec", 30.0))
    command_timeout_sec = float(spec.get("command_timeout_sec", 30.0))
    recorder_ready_timeout_sec = float(spec.get("recorder_ready_timeout_sec", DEFAULT_RECORDER_READY_TIMEOUT_SEC))
    require(settle_sec >= 30.0, "settle_sec must be at least 30 seconds")
    require(timeout_sec == 60.0, "GOAL_REACHED timeout must be exactly 60 seconds")
    require(tail_sec > 0.0, "tail_sec must be positive")
    require(post_shutdown_sec >= 30.0, "post_shutdown_sec must be at least 30 seconds")
    require(command_timeout_sec > 0.0 and math.isfinite(command_timeout_sec), "command_timeout_sec must be positive and finite")
    require(recorder_ready_timeout_sec > 0.0 and math.isfinite(recorder_ready_timeout_sec), "recorder_ready_timeout_sec must be positive and finite")
    if formal:
        recording_policy = require_mapping(require_mapping(spec.get("formal_freeze"), "formal freeze").get("recording_policy"), "formal recording policy")
        require(settle_sec == recording_policy.get("settle_sec"), "runtime settle differs from frozen policy")
        require(timeout_sec == recording_policy.get("goal_timeout_sec"), "runtime GOAL_REACHED timeout differs from frozen policy")
        require(tail_sec == recording_policy.get("tail_sec"), "runtime tail differs from frozen outcome window")
        require(post_shutdown_sec == recording_policy.get("post_shutdown_sec"), "runtime post-shutdown wait differs from frozen policy")
        require(recorder_ready_timeout_sec == recording_policy.get("recorder_ready_timeout_sec"), "runtime recorder readiness timeout differs from frozen policy")
        assert formal_launch_contract is not None
        require(command_timeout_sec == formal_launch_contract["command_timeout_sec"], "runtime command timeout differs from frozen launch contract")
    ros_uri = str(spec.get("ros_master_uri", "127.0.0.1:11330"))
    gazebo_uri = str(spec.get("gazebo_master_uri", "127.0.0.1:11364"))
    if formal:
        assert formal_launch_contract is not None
        require(ros_uri == formal_launch_contract["ros_master_uri"] and gazebo_uri == formal_launch_contract["gazebo_master_uri"], "runtime master URIs differ from frozen launch contract")
    # The lock deliberately spans preflight through postflight.  A second
    # runner must not race the first between "master absent" and launch.
    master_lock_path, master_lock_token = acquire_master_lock(sim_root, ros_uri, gazebo_uri, attempt_id)
    case_dir_created = False
    initialization_stage = "preflight"
    try:
        pre = endpoints_reachable(ros_uri, gazebo_uri)
        require(not pre["ros"] and not pre["gazebo"], "strict-fresh preflight failed: ROS/Gazebo master already reachable")
        initialization_stage = "case_directory_create"
        case_dir.mkdir(parents=True, exist_ok=False)
        case_dir_created = True
        initialization_stage = "seed_bundle_write"
        seed_bundle_path = case_dir / "seed_bundle.json"
        write_json_new(seed_bundle_path, seed_bundle)
        os.chmod(seed_bundle_path, 0o444)
        case_launch_manifest = {
            "schema_version": SCHEMA_VERSION,
            "attempt_id": attempt_id,
            "planned_row_id": row["planned_row_id"],
            "formal": formal,
            "formal_master_hash": formal_master_document.get("master_hash") if formal_master_document is not None else None,
            "formal_master_path": str(formal_master_path) if formal_master_path is not None else None,
            "formal_master_file_hash": formal_master_file_hash,
            "formal_freeze_hash": formal_freeze_hash,
            "formal_freeze_path": str(formal_freeze_path) if formal_freeze_path is not None else None,
            "formal_freeze_file_hash": formal_freeze_file_hash,
            "contrast_registry_hash": formal_master_document.get("contrast_registry", {}).get("registry_hash") if formal_master_document is not None else None,
            "runtime_launch_contract_id": formal_launch_contract.get("contract_id") if formal_launch_contract is not None else None,
            "runtime_launch_contract_hash": formal_launch_contract.get("contract_hash") if formal_launch_contract is not None else None,
            "dataset_ledger_id": formal_dataset_ledger.get("ledger_id") if formal_dataset_ledger is not None else None,
            "dataset_ledger_identity_hash": formal_dataset_ledger.get("ledger_identity_hash") if formal_dataset_ledger is not None else None,
            "dataset_root": str(output_root.resolve()),
            "hashes": hashes,
            "declared_config_path": str(declared_config_path) if declared_config_path is not None else None,
            "declared_config_hash": canonical_hash(dict(declared_config)) if declared_config is not None else None,
            "effective_config_readback_required": development_h0_runtime and not dry_run,
            "planned_row_frozen_asset_hashes": row.get("frozen_asset_hashes", {}),
            "ros_master_uri": ros_uri,
            "gazebo_master_uri": gazebo_uri,
            "seed_bundle_path": str(seed_bundle_path),
            "seed_bundle_hash": seed_bundle["seed_bundle_hash"],
            "seed_trace_hashes": seed_report["trace_hashes"],
            "controller_nodes": sorted(controller_nodes),
            # Bind the raw capability declaration before any launch command.
            # For H0 this can be an explicit UNVALIDATED development H_plant;
            # it remains non-primary and is separately summarized below by
            # ``liquid_truth_capability`` in the final attempt manifest.
            "liquid_plant_capability": (
                dict(spec["liquid_plant_capability"])
                if isinstance(spec.get("liquid_plant_capability"), Mapping)
                else None
            ),
            "liquid_truth_capability": truth,
            # This optional H0-only declaration is written before the launch
            # child starts.  Snapshot payloads bind to the resulting immutable
            # case-manifest hash; the final manifests bind their file hashes.
            "development_firewall": (
                dict(development_firewall)
                if development_firewall is not None
                else None
            ),
            "development_firewall_contract_hash": (
                development_firewall.get("contract_hash")
                if development_firewall is not None
                else None
            ),
            "development_firewall_snapshot_paths": (
                dict(development_firewall["snapshot_paths"])
                if development_firewall is not None
                else None
            ),
            "development_firewall_required": development_firewall is not None,
            "development_firewall_status": (
                "NOT_LIVE_DRY_RUN"
                if development_firewall_not_live
                else ("REQUIRED_LIVE" if development_firewall is not None else "NOT_APPLICABLE")
            ),
            "physical_primary_eligible_at_launch": False,
        }
        case_launch_manifest_path = case_dir / "case_launch_manifest.json"
        initialization_stage = "case_launch_manifest_write"
        write_json_new(case_launch_manifest_path, case_launch_manifest)
        os.chmod(case_launch_manifest_path, 0o444)
        case_launch_manifest_hash = sha256_file(case_launch_manifest_path)
        lifecycle: List[Dict[str, Any]] = [{"event": "preflight", "utc": utc_now(), "ros_reachable": False, "gazebo_reachable": False}, {"event": "master_lock_acquired", "utc": utc_now(), "lock_path": str(master_lock_path)}]
        children = TrackedChildren()
        environment = dict(
            os.environ,
            ROS_MASTER_URI=http_uri(ros_uri),
            GAZEBO_MASTER_URI=http_uri(gazebo_uri),
            SMPCC_SEED_BUNDLE_PATH=str(seed_bundle_path),
            SMPCC_SEED_BUNDLE_SHA256=str(seed_bundle["seed_bundle_hash"]),
            SMPCC_CASE_LAUNCH_MANIFEST_PATH=str(case_launch_manifest_path),
            SMPCC_CASE_LAUNCH_MANIFEST_SHA256=case_launch_manifest_hash,
        )
        if formal:
            require(
                formal_master_path is not None
                and formal_master_file_hash is not None
                and formal_freeze_path is not None
                and formal_freeze_file_hash is not None,
                "formal runner lost immutable master/freeze bindings before runtime launch",
            )
            environment.update(
                formal_runtime_environment_bindings(
                    formal_master_path,
                    formal_master_file_hash,
                    formal_freeze_path,
                    formal_freeze_file_hash,
                )
            )
        motion_started = False
        motion_may_have_started = False
        goal_reached = False
        failure_class = "NONE"
        cleanup: List[Dict[str, Any]] = []
        firewall_reports: List[Dict[str, Any]] = []
        development_firewall_snapshot_reports: List[Dict[str, Any]] = []
        runtime_error: Optional[str] = None
        recorder_process: Optional[subprocess.Popen[Any]] = None
        recorder_artifact_path: Optional[Path] = None
        recorder_size_before_motion: Optional[int] = None
        recorder_size_before_tail: Optional[int] = None
        runtime_ack: Optional[Mapping[str, Any]] = None
        runtime_ack_path: Optional[Path] = None
        runtime_ack_hash: Optional[str] = None
        effective_config_readback: Optional[Mapping[str, Any]] = None
        effective_config_readback_path: Optional[Path] = None
        effective_config_readback_hash: Optional[str] = None
        motion_stop_ack: Optional[Mapping[str, Any]] = None
        motion_stop_ack_path: Optional[Path] = None
        motion_stop_ack_hash: Optional[str] = None
        motion_release_ack: Optional[Mapping[str, Any]] = None
        motion_release_ack_path: Optional[Path] = None
        motion_release_ack_hash: Optional[str] = None
        development_motion_event: Optional[Mapping[str, Any]] = None
        development_motion_event_path: Optional[Path] = None
        development_motion_event_hash: Optional[str] = None
        development_goal_event: Optional[Mapping[str, Any]] = None
        development_goal_event_path: Optional[Path] = None
        development_goal_event_hash: Optional[str] = None
        assignment_consumed = False
        motion_stop_issued = False
        failure_classification: Optional[Mapping[str, Any]] = None
        failure_classification_path: Optional[Path] = None
        failure_classification_hash: Optional[str] = None
        failure_event_path: Optional[Path] = None
        failure_event_hash: Optional[str] = None
        first_effective_motion_utc: Optional[str] = None
        first_arrival_utc: Optional[str] = None
        tail_end_utc: Optional[str] = None
        tail_complete = False
        motion_launch_monotonic: Optional[float] = None
        motion_deadline_monotonic: Optional[float] = None
    except BaseException as exc:
        if case_dir_created:
            recovery = {
                "schema_version": SCHEMA_VERSION,
                "record_type": "SMPCC_SIM_INITIALIZATION_RECOVERY",
                "status": "RECOVERY_REQUIRED",
                "ledger_admission": "UNCONFIRMED_DO_NOT_CONSUME",
                "attempt_id": attempt_id,
                "planned_row_id": row["planned_row_id"],
                "formal": formal,
                "initialization_stage": initialization_stage,
                "error_type": type(exc).__name__,
                "error": repr(exc),
                "created_utc": utc_now(),
            }
            try:
                write_json_new(case_dir / "initialization_recovery.json", recovery)
            except BaseException:
                pass
        release_master_lock(master_lock_path, master_lock_token)
        raise

    def capture_development_firewall_snapshot(checkpoint: str) -> None:
        """Run and revalidate one H0 adapter-owned live graph snapshot.

        The hook is absent unless the explicitly UNVALIDATED development plant
        was selected.  Formal runs retain their independent frozen firewall
        path above; no formal condition can satisfy this development hook.
        """
        if development_firewall is None:
            return
        require(
            development_firewall_snapshot_commands is not None,
            "development H0 firewall lost its snapshot command map",
        )
        require(
            checkpoint in DEVELOPMENT_H0_FIREWALL_CHECKPOINTS,
            "invalid development H0 firewall checkpoint",
        )
        require(
            not any(item.get("checkpoint") == checkpoint for item in development_firewall_snapshot_reports),
            f"development H0 firewall checkpoint was captured twice: {checkpoint}",
        )
        command = command_from_spec(
            development_firewall_snapshot_commands[checkpoint],
            f"development_firewall_snapshot_commands.{checkpoint}",
        )
        result = run_command_with_timeout(
            command,
            environment,
            f"development_firewall_snapshot.{checkpoint}",
            command_timeout_sec,
        )
        require(
            result.returncode == 0,
            f"development H0 firewall {checkpoint} snapshot command failed",
        )
        path = ensure_within(
            Path(str(development_firewall["snapshot_paths"][checkpoint])),
            case_dir,
            f"development H0 firewall {checkpoint} snapshot",
        )
        report, file_hash = load_development_h0_firewall_snapshot(
            path,
            checkpoint,
            case_dir,
            case_launch_manifest_hash,
            development_firewall,
        )
        entry = {
            "checkpoint": checkpoint,
            "path": str(path),
            "hash": file_hash,
            "status": report["status"],
            "graph_hash": report["graph_hash"],
            "controller_nodes": report["controller_nodes"],
            "physical_primary_eligible": False,
            "development_only": True,
            "formal": False,
        }
        development_firewall_snapshot_reports.append(entry)
        lifecycle.append(
            {
                "event": "development_firewall_snapshot",
                "utc": utc_now(),
                **entry,
            }
        )

    def stop_motion_before_tail(after_failure: bool = False) -> Dict[str, Any]:
        """Stop only this case's motion and, formally, prove command-zero."""
        nonlocal motion_stop_ack, motion_stop_ack_path, motion_stop_ack_hash, motion_stop_issued
        if motion_stop_issued:
            return {"already_stopped": True}
        command_result: Optional[Dict[str, Any]] = None
        if formal:
            require(spec.get("motion_stop_command") is not None, "formal runner requires motion_stop_command before frozen tail")
            stopped = run_command_with_timeout(command_from_spec(spec.get("motion_stop_command"), "motion_stop_command"), environment, "motion_stop_command", command_timeout_sec)
            command_result = {"returncode": stopped.returncode}
            require(stopped.returncode == 0, "formal motion_stop_command failed before frozen tail")
            ack_path_value = spec.get("motion_stop_ack_path")
            require(isinstance(ack_path_value, str) and ack_path_value, "formal runner requires motion_stop_ack_path")
            motion_stop_ack_path = ensure_within(Path(ack_path_value), sim_root, "motion stop acknowledgement")
            require(motion_stop_ack_path.is_file(), "formal motion stop acknowledgement is missing")
            assert formal_launch_contract is not None
            motion_stop_ack = validate_motion_stop_ack(
                motion_stop_ack_path,
                case_launch_manifest_hash,
                expected_schema_hash=str(formal_launch_contract["motion_stop_ack_schema_hash"]),
            )
            motion_stop_ack_hash = sha256_file(motion_stop_ack_path)
        elif spec.get("motion_stop_command") is not None:
            stopped = run_command_with_timeout(command_from_spec(spec.get("motion_stop_command"), "motion_stop_command"), environment, "motion_stop_command", command_timeout_sec)
            command_result = {"returncode": stopped.returncode}
            require(stopped.returncode == 0, "motion_stop_command failed before frozen tail")
        process = children.stop_label("motion_backend")
        motion_stop_issued = True
        return {
            "process": process,
            "motion_stop_command": command_result,
            "motion_stop_ack_path": str(motion_stop_ack_path) if motion_stop_ack_path is not None else None,
            "motion_stop_ack_hash": motion_stop_ack_hash,
            "after_failure": after_failure,
        }

    def classify_pre_motion_acquisition(error: str) -> bool:
        """Return true only for a frozen, external condition-blind decision."""
        nonlocal failure_classification, failure_classification_path, failure_classification_hash, failure_event_path, failure_event_hash
        if not formal or formal_retry_classifier is None:
            return False
        classifier_candidates = (
            "fresh launch did not expose both ros and gazebo masters",
            "fresh ros/gazebo launcher exited before readiness",
            "fresh launch readiness command failed before assignment consumption",
            "recorder did not create its artifact before motion",
            "recorder artifact is not growing before motion",
            "recorder exited before motion",
        )
        if not any(marker in error.casefold() for marker in classifier_candidates):
            return False
        try:
            event_core = {
                "schema_version": SCHEMA_VERSION,
                "event_type": "SMPCC_SIM_PREMOTION_FAILURE_EVENT",
                "attempt_id": attempt_id,
                "planned_row_id": row["planned_row_id"],
                "case_launch_manifest_hash": case_launch_manifest_hash,
                "failure_phase": "PRE_MOTION_PRE_ASSIGNMENT",
                "error_hash": hashlib.sha256(error.encode("utf-8")).hexdigest(),
                "lifecycle_hash": canonical_hash(lifecycle),
                "created_utc": utc_now(),
            }
            failure_event_path = case_dir / "pre_motion_failure_event.json"
            write_json_new(failure_event_path, event_core)
            os.chmod(failure_event_path, 0o444)
            failure_event_hash = sha256_file(failure_event_path)
            command_value = spec.get("failure_classifier_command")
            require(command_value is not None, "formal acquisition classifier command is required")
            classifier_command = command_from_spec(command_value, "failure_classifier_command")
            verifier_path = Path(str(formal_retry_classifier["verifier_path"]))
            require(verifier_path.is_file() and sha256_file(verifier_path) == formal_retry_classifier.get("verifier_hash"), "frozen failure classifier verifier changed")
            require(classifier_command == formal_retry_classifier.get("verifier_command"), "failure_classifier_command differs from frozen verifier command")
            classifier_environment = dict(
                environment,
                SMPCC_FAILURE_EVENT_PATH=str(failure_event_path),
                SMPCC_FAILURE_EVENT_SHA256=failure_event_hash,
            )
            classified = run_command_with_timeout(classifier_command, classifier_environment, "failure_classifier_command", command_timeout_sec)
            require(classified.returncode == 0, "failure classifier command failed")
            path_value = spec.get("failure_classification_path")
            require(isinstance(path_value, str) and path_value, "formal pre-motion failure lacks external classification path")
            path = ensure_within(Path(path_value), sim_root, "failure classification")
            require(path.is_file(), "formal pre-motion failure classification artifact is missing")
            decision = validate_failure_classification(
                read_json(path),
                formal_retry_classifier,
                attempt_id,
                case_launch_manifest_hash,
                failure_event_path=failure_event_path,
                failure_event_hash=failure_event_hash,
            )
            failure_classification = decision
            failure_classification_path = path
            failure_classification_hash = sha256_file(path)
            return True
        except ContractError as exc:
            lifecycle.append({"event": "failure_classification_rejected", "utc": utc_now(), "error": str(exc)})
            return False
        except Exception as exc:  # noqa: BLE001
            lifecycle.append({"event": "failure_classification_rejected", "utc": utc_now(), "error": repr(exc)})
            return False

    def remaining_motion_budget(phase: str) -> float:
        """Use one 60-second wall-clock budget from motion launch onward.

        The motion-release ACK and the development /odom probe can themselves
        take time.  Giving each of them a separate timeout would let a vehicle
        travel for more than 60 seconds before the GOAL_REACHED poll starts.
        """
        require(motion_deadline_monotonic is not None, f"motion deadline is unavailable before {phase}")
        remaining = motion_deadline_monotonic - time.monotonic()
        require(remaining > 0.0, f"GOAL_REACHED motion budget exhausted before {phase}")
        return remaining

    try:
        if dry_run:
            lifecycle.append({"event": "fresh_launch_simulated", "utc": utc_now()})
            lifecycle.append({"event": "settle", "utc": utc_now(), "requested_sec": settle_sec, "elapsed_sec": settle_sec, "virtual": True})
            lifecycle.append({"event": "recorder_started", "utc": utc_now(), "simulated": True})
            if truth["eligible"] and spec.get("firewall_graph"):
                for checkpoint in ("ready", "pre_motion", "postflight"):
                    report = validate_controller_firewall(require_mapping(spec["firewall_graph"], "firewall graph"), controller_nodes)
                    firewall_reports.append(dict(report, checkpoint=checkpoint, dry_run=True))
                    require(report["status"] == "PASS", "controller subscriber firewall failed before motion")
            first_effective_motion_utc = utc_now()
            lifecycle.append({"event": "motion_released", "utc": first_effective_motion_utc, "simulated": True})
            motion_started = True
            motion_may_have_started = True
            assignment_consumed = True
            goal_reached = spec.get("simulate_goal_reached", True) is True
            first_arrival_utc = utc_now() if goal_reached else None
            lifecycle.append({"event": "goal_reached" if goal_reached else "goal_timeout", "utc": first_arrival_utc or utc_now(), "timeout_sec": timeout_sec, "virtual": True})
            lifecycle.append({"event": "motion_stopped_before_tail", "utc": utc_now(), "simulated": True})
            tail_end_utc = utc_now()
            lifecycle.append({"event": "tail_recorded", "utc": tail_end_utc, "tail_sec": tail_sec, "virtual": True})
            tail_complete = True
        else:
            if formal:
                # Rehash the frozen command artifacts immediately before the
                # fresh launch; formal commands may not drift after preflight.
                formal_launch_contract = validate_frozen_runtime_launch_contract(require_mapping(spec.get("formal_freeze"), "formal freeze").get("runtime_launch_contract"))
            launch = command_from_spec(spec.get("launch_command"), "launch_command")
            launch_process = children.start("fresh_ros_gazebo", launch, environment)
            lifecycle.append({"event": "fresh_launch_started", "utc": utc_now()})
            startup_timeout_sec = float(spec.get("startup_timeout_sec", 60.0))
            launch_state = wait_for_endpoints(ros_uri, gazebo_uri, startup_timeout_sec)
            require(launch_state["ros"] and launch_state["gazebo"], "fresh launch did not expose both ROS and Gazebo masters")
            require(launch_process.poll() is None, "fresh ROS/Gazebo launcher exited before readiness")
            require(spec.get("ready_command") is not None, "actual runner requires explicit ready_command")
            ready = run_command_with_timeout(command_from_spec(spec.get("ready_command"), "ready_command"), environment, "ready_command", command_timeout_sec)
            require(ready.returncode == 0, "fresh launch readiness command failed before assignment consumption")
            capture_development_firewall_snapshot("ready")
            if formal:
                ack_path_value = spec.get("runtime_ack_path")
                require(isinstance(ack_path_value, str) and ack_path_value, "formal runner requires runtime_ack_path")
                ack_path = ensure_within(Path(ack_path_value), sim_root, "runtime acknowledgement")
                require(spec.get("runtime_ack_command") is not None, "formal runner requires runtime_ack_command")
                ack_command = run_command_with_timeout(command_from_spec(spec.get("runtime_ack_command"), "runtime_ack_command"), environment, "runtime_ack_command", command_timeout_sec)
                require(ack_command.returncode == 0 and ack_path.is_file(), "runtime input acknowledgement command failed")
                assert formal_launch_contract is not None
                runtime_ack = validate_runtime_ack(
                    ack_path,
                    case_launch_manifest_hash,
                    hashes,
                    seed_bundle,
                    formal=True,
                    expected_schema_hash=str(formal_launch_contract["runtime_ack_schema_hash"]),
                )
                runtime_ack_path = ack_path
                runtime_ack_hash = sha256_file(ack_path)
            if truth["eligible"]:
                graph = ros_master_graph(http_uri(ros_uri))
                report = validate_controller_firewall(graph, controller_nodes)
                firewall_reports.append(dict(report, checkpoint="ready"))
                require(report["status"] == "PASS", "controller subscriber firewall failed at readiness")
            time.sleep(settle_sec)
            require(launch_process.poll() is None, "fresh ROS/Gazebo launcher exited during settle")
            lifecycle.append({"event": "settle", "utc": utc_now(), "requested_sec": settle_sec, "elapsed_sec": settle_sec, "virtual": False})
            require(launch_process.poll() is None, "fresh ROS/Gazebo launcher exited before recorder")
            recorder = command_from_spec(spec.get("recorder_command"), "recorder_command")
            recorder_process = children.start("recorder", recorder, environment)
            lifecycle.append({"event": "recorder_started", "utc": utc_now(), "simulated": False})
            artifact = spec.get("recorder_artifact")
            require(isinstance(artifact, str) and artifact, "actual runner requires recorder_artifact")
            recorder_artifact_path = ensure_within(Path(artifact), sim_root, "recorder artifact")
            if formal:
                assert formal_launch_contract is not None
                require(formal_launch_contract["recorder_artifact_rule_hash"], "formal launch contract lacks recorder artifact rule")
                try:
                    recorder_artifact_path.resolve().relative_to(case_dir.resolve())
                except ValueError as exc:
                    raise ContractError("formal recorder artifact must be case-local") from exc
            # Rosbag and other recorders create their on-disk artifact
            # asynchronously.  Treating the same scheduler tick as proof of
            # absence caused a false pre-motion protocol failure even when the
            # recorder was correctly alive.  Wait only a frozen/bounded amount,
            # and still require both a file and subsequent byte growth before
            # any motion backend is launched.
            recorder_ready_start = time.monotonic()
            recorder_ready_deadline = recorder_ready_start + recorder_ready_timeout_sec
            while not recorder_artifact_path.exists() and time.monotonic() < recorder_ready_deadline:
                require(recorder_process.poll() is None, "recorder exited before creating its artifact")
                time.sleep(min(0.10, max(0.01, recorder_ready_deadline - time.monotonic())))
            require(recorder_artifact_path.exists(), f"recorder did not create its artifact before motion within {recorder_ready_timeout_sec:g} seconds")
            lifecycle.append(
                {
                    "event": "recorder_artifact_ready",
                    "utc": utc_now(),
                    "wait_sec": round(time.monotonic() - recorder_ready_start, 6),
                    "timeout_sec": recorder_ready_timeout_sec,
                }
            )
            before = recorder_artifact_path.stat().st_size
            time.sleep(1.0)
            recorder_size_before_motion = recorder_artifact_path.stat().st_size
            require(recorder_size_before_motion > before, "recorder artifact is not growing before motion")
            require(recorder_process.poll() is None, "recorder exited before motion")
            require(launch_process.poll() is None, "fresh ROS/Gazebo launcher exited before motion")
            capture_development_firewall_snapshot("pre_motion")
            if truth["eligible"]:
                graph = ros_master_graph(http_uri(ros_uri))
                report = validate_controller_firewall(graph, controller_nodes)
                firewall_reports.append(dict(report, checkpoint="pre_motion"))
                require(report["status"] == "PASS", "controller subscriber firewall failed before motion")
            if formal:
                require(asset_hashes(spec, row) == hashes, "runtime input assets changed after formal acknowledgement")
            motion = command_from_spec(spec.get("motion_command"), "motion_command")
            # Start the hard trajectory budget immediately before the motion
            # process is released.  This is conservative if first odom motion
            # appears a few ticks later, and prevents ACK/probe overhead from
            # silently extending the allowed trajectory duration.
            motion_launch_monotonic = time.monotonic()
            motion_deadline_monotonic = motion_launch_monotonic + timeout_sec
            motion_process = children.start("motion_backend", motion, environment)
            motion_may_have_started = True
            require(motion_process.poll() is None, "motion backend exited before assignment consumption")
            if formal:
                release_ack_path_value = spec.get("motion_release_ack_path")
                require(isinstance(release_ack_path_value, str) and release_ack_path_value, "formal runner requires motion_release_ack_path")
                motion_release_ack_path = ensure_within(Path(release_ack_path_value), sim_root, "motion release acknowledgement")
                require(spec.get("motion_release_ack_command") is not None, "formal runner requires motion_release_ack_command")
                release_ack_command = run_command_with_timeout(
                    command_from_spec(spec.get("motion_release_ack_command"), "motion_release_ack_command"),
                    environment,
                    "motion_release_ack_command",
                    min(command_timeout_sec, remaining_motion_budget("formal motion-release acknowledgement")),
                )
                require(release_ack_command.returncode == 0 and motion_release_ack_path.is_file(), "motion release acknowledgement command failed")
                assert formal_launch_contract is not None
                motion_release_ack = validate_motion_release_ack(
                    motion_release_ack_path,
                    case_launch_manifest_hash,
                    expected_schema_hash=str(formal_launch_contract["motion_release_ack_schema_hash"]),
                )
                motion_release_ack_hash = sha256_file(motion_release_ack_path)
                require(motion_process.poll() is None, "motion backend exited before acknowledged base motion")
                assignment_consumed = True
                first_effective_motion_utc = str(motion_release_ack["first_effective_motion_utc"])
            else:
                has_development_motion_marker = spec.get("motion_start_command") is not None or spec.get("motion_event_path") is not None
                if has_development_motion_marker:
                    require(spec.get("motion_start_command") is not None, "development motion_event_path requires motion_start_command")
                    event_path_value = spec.get("motion_event_path")
                    require(isinstance(event_path_value, str) and event_path_value, "development motion_start_command requires motion_event_path")
                    start_result = run_command_with_timeout(
                        command_from_spec(spec.get("motion_start_command"), "motion_start_command"),
                        environment,
                        "motion_start_command",
                        min(command_timeout_sec, remaining_motion_budget("development executed-motion observation")),
                    )
                    require(start_result.returncode == 0, "motion_start_command failed before executed base motion")
                    development_motion_event_path = ensure_within(Path(event_path_value), sim_root, "development motion event")
                    require(development_motion_event_path.is_file(), "development motion event is missing after motion_start_command")
                    development_motion_event = validate_development_motion_event(development_motion_event_path, case_launch_manifest_hash)
                    development_motion_event_hash = sha256_file(development_motion_event_path)
                    require(motion_process.poll() is None, "motion backend exited before executed base motion")
                    assignment_consumed = True
                    first_effective_motion_utc = str(development_motion_event["first_effective_motion_utc"])
                    lifecycle.append(
                        {
                            "event": "development_executed_motion_observed",
                            "utc": first_effective_motion_utc,
                            "event_path": str(development_motion_event_path),
                            "event_hash": development_motion_event_hash,
                            "observed_topic": development_motion_event["observed_topic"],
                        }
                    )
                else:
                    assignment_consumed = True
                    first_effective_motion_utc = utc_now()
            motion_started = True
            lifecycle.append(
                {
                    "event": "motion_released",
                    "utc": first_effective_motion_utc,
                    "simulated": False,
                    "motion_launch_monotonic": motion_launch_monotonic,
                    "motion_deadline_monotonic": motion_deadline_monotonic,
                }
            )
            goal_reached = wait_for_goal(
                command_from_spec(spec.get("goal_probe_command"), "goal_probe_command"),
                environment,
                remaining_motion_budget("GOAL_REACHED probe"),
            )
            if goal_reached and not formal and spec.get("goal_event_path") is not None:
                goal_event_path_value = spec.get("goal_event_path")
                require(isinstance(goal_event_path_value, str) and goal_event_path_value, "development goal event path is invalid")
                development_goal_event_path = ensure_within(Path(goal_event_path_value), sim_root, "development goal event")
                require(development_goal_event_path.is_file(), "development goal event is missing after GOAL_REACHED probe")
                development_goal_event = validate_development_goal_event(development_goal_event_path, case_launch_manifest_hash)
                development_goal_event_hash = sha256_file(development_goal_event_path)
                first_arrival_utc = str(development_goal_event["first_arrival_utc"])
            else:
                first_arrival_utc = utc_now() if goal_reached else None
            lifecycle.append({"event": "goal_reached" if goal_reached else "goal_timeout", "utc": first_arrival_utc or utc_now(), "timeout_sec": timeout_sec, "virtual": False})
            require(launch_process.poll() is None, "fresh ROS/Gazebo launcher exited during motion")
            stop_motion = stop_motion_before_tail()
            lifecycle.append({"event": "motion_stopped_before_tail", "utc": utc_now(), **stop_motion})
            recorder_size_before_tail = recorder_artifact_path.stat().st_size
            time.sleep(tail_sec)
            tail_end_utc = utc_now()
            lifecycle.append({"event": "tail_recorded", "utc": tail_end_utc, "tail_sec": tail_sec, "virtual": False})
            require(launch_process.poll() is None, "fresh ROS/Gazebo launcher exited during frozen tail")
            require(recorder_process is not None and recorder_process.poll() is None, "recorder exited before frozen tail completed")
            require(recorder_artifact_path is not None and recorder_size_before_motion is not None and recorder_artifact_path.stat().st_size > recorder_size_before_motion, "recorder artifact did not grow through motion/tail")
            require(recorder_artifact_path is not None and recorder_size_before_tail is not None and recorder_artifact_path.stat().st_size > recorder_size_before_tail, "recorder artifact did not grow during frozen tail")
            tail_complete = True
            capture_development_firewall_snapshot("postflight")
            if truth["eligible"]:
                graph = ros_master_graph(http_uri(ros_uri))
                report = validate_controller_firewall(graph, controller_nodes)
                firewall_reports.append(dict(report, checkpoint="postflight"))
                require(report["status"] == "PASS", "controller subscriber firewall failed after motion")
        if not goal_reached:
            failure_class = "METHOD_FAILURE"
    except ContractError as exc:
        runtime_error = str(exc)
        # A generic launch/readiness/recording contract error is never enough
        # to replace a row.  Only an independent frozen classifier can call a
        # pre-motion event acquisition infrastructure.
        failure_class = "METHOD_FAILURE" if (motion_may_have_started or assignment_consumed) else (
            RETRYABLE_FAILURE_CLASS if classify_pre_motion_acquisition(runtime_error) else "PROTOCOL_FAILURE"
        )
        lifecycle.append({"event": "runner_contract_failure", "utc": utc_now(), "error": runtime_error})
    except Exception as exc:  # noqa: BLE001
        runtime_error = repr(exc)
        failure_class = "METHOD_FAILURE" if (motion_may_have_started or assignment_consumed) else (
            RETRYABLE_FAILURE_CLASS if classify_pre_motion_acquisition(runtime_error) else "PROTOCOL_FAILURE"
        )
        lifecycle.append({"event": "runner_exception", "utc": utc_now(), "error": runtime_error})
    finally:
        if motion_may_have_started and not any(item["event"] == "tail_recorded" for item in lifecycle):
            # Even an exception/timeout cannot turn a failure into an early-stop
            # success.  Keep recorder ownership until the prescribed tail.
            try:
                stop_motion = stop_motion_before_tail(after_failure=True)
            except ContractError as exc:
                # Still terminate only the tracked motion process and preserve
                # the tail if possible; lack of zero-ack remains protocol-fail.
                stop_motion = {"process": children.stop_label("motion_backend"), "after_failure": True, "stop_error": str(exc)}
                motion_stop_issued = True
                failure_class = "PROTOCOL_FAILURE"
                runtime_error = runtime_error or str(exc)
                lifecycle.append({"event": "motion_stop_contract_failure", "utc": utc_now(), "error": str(exc)})
            lifecycle.append({"event": "motion_stopped_before_tail", "utc": utc_now(), **stop_motion})
            if dry_run:
                tail_end_utc = utc_now()
                lifecycle.append({"event": "tail_recorded", "utc": tail_end_utc, "tail_sec": tail_sec, "virtual": True, "after_failure": True})
                tail_complete = True
            else:
                if recorder_artifact_path is not None and recorder_artifact_path.exists():
                    recorder_size_before_tail = recorder_artifact_path.stat().st_size
                time.sleep(tail_sec)
                tail_end_utc = utc_now()
                lifecycle.append({"event": "tail_recorded", "utc": tail_end_utc, "tail_sec": tail_sec, "virtual": False, "after_failure": True})
                try:
                    require(recorder_process is not None and recorder_process.poll() is None, "recorder exited before post-failure frozen tail completed")
                    require(recorder_artifact_path is not None and recorder_size_before_tail is not None and recorder_artifact_path.stat().st_size > recorder_size_before_tail, "recorder artifact did not grow during post-failure frozen tail")
                    tail_complete = True
                except ContractError as exc:
                    tail_complete = False
                    failure_class = "PROTOCOL_FAILURE"
                    runtime_error = runtime_error or str(exc)
                    lifecycle.append({"event": "tail_incomplete", "utc": utc_now(), "error": str(exc)})
        try:
            cleanup = children.stop_all()
        except BaseException as exc:
            # Do not let an interruption while auditing owned children strand
            # the master lock before finalization/recovery evidence is written.
            cleanup = [{"label": "cleanup_runner", "cleanup_error": repr(exc)}]
            failure_class = "PROTOCOL_FAILURE"
            runtime_error = runtime_error or f"owned process cleanup interrupted: {exc!r}"
        lifecycle.append({"event": "owned_pid_cleanup", "utc": utc_now(), "children": cleanup})
        if any(item.get("cleanup_error") for item in cleanup):
            failure_class = "PROTOCOL_FAILURE"
            runtime_error = runtime_error or "owned process cleanup was incomplete"
    # The fresh-master lock remains owned through all final writes.  Any
    # post-shutdown failure is fail-closed: a non-ledger recovery receipt is
    # attempted, then only this runner's lock is released before re-raising.
    dataset_index_path = output_root / "dataset_index.jsonl"
    effective_path = case_dir / "effective_config.json"
    postflight_path = case_dir / "postflight.json"
    manifest_path = case_dir / "attempt_manifest.json"
    recovery_marker_path = case_dir / "finalization_recovery.json"
    finalization_stage = "post_shutdown_wait"
    lock_release_verified = False
    try:
        if development_h0_runtime and not dry_run:
            # The H0 adapter derives this receipt from the closed rosbag, so it
            # must run only after the runner has stopped its recorder/process
            # groups.  It remains a development check; formal rows use their
            # pre-motion frozen runtime ACK instead.
            try:
                finalization_stage = "effective_config_readback"
                require(declared_config is not None and declared_config_path is not None, "actual H0 development run lacks declared config for readback")
                readback_command = run_command_with_timeout(
                    command_from_spec(spec.get("effective_config_readback_command"), "effective_config_readback_command"),
                    environment,
                    "effective_config_readback_command",
                    command_timeout_sec,
                )
                require(readback_command.returncode == 0, "effective_config_readback_command failed")
                readback_path_value = spec.get("effective_config_readback_path")
                require(isinstance(readback_path_value, str) and readback_path_value, "actual H0 development run lacks effective_config_readback_path")
                effective_config_readback_path = ensure_within(Path(readback_path_value), sim_root, "development effective-config readback")
                require(effective_config_readback_path.parent.resolve() == case_dir.resolve(), "development effective-config readback must be case-local")
                require(effective_config_readback_path.is_file(), "development effective-config readback is missing after command")
                effective_config_readback = validate_development_effective_config_readback(
                    effective_config_readback_path,
                    case_launch_manifest_hash,
                    declared_config,
                )
                effective_config_readback_hash = sha256_file(effective_config_readback_path)
                effective_config = require_mapping(effective_config_readback.get("observed_effective_config"), "development observed effective config")
                require(canonical_hash(dict(effective_config)) == hashes.get("effective_config_hash"), "development observed effective config differs from launch declaration")
                lifecycle.append(
                    {
                        "event": "effective_config_readback",
                        "utc": utc_now(),
                        "path": str(effective_config_readback_path),
                        "hash": effective_config_readback_hash,
                    }
                )
            except ContractError as exc:
                # The vehicle has already executed its assignment by this
                # point.  A missing, malformed, or mismatched readback is a
                # condition/config execution failure, not a pre-motion
                # acquisition event.  Keep the attempt in the append-only
                # ledger so a later success cannot replace it.
                failure_class = "METHOD_FAILURE" if motion_may_have_started else "PROTOCOL_FAILURE"
                runtime_error = runtime_error or str(exc)
                lifecycle.append(
                    {
                        "event": "effective_config_readback_failed",
                        "utc": utc_now(),
                        "error": str(exc),
                    }
                )
        if dry_run:
            lifecycle.append({"event": "post_shutdown_wait", "utc": utc_now(), "requested_sec": post_shutdown_sec, "elapsed_sec": post_shutdown_sec, "virtual": True})
        else:
            time.sleep(post_shutdown_sec)
            lifecycle.append({"event": "post_shutdown_wait", "utc": utc_now(), "requested_sec": post_shutdown_sec, "elapsed_sec": post_shutdown_sec, "virtual": False})
        finalization_stage = "postflight_endpoint_probe"
        try:
            post = endpoints_reachable(ros_uri, gazebo_uri)
        except Exception as exc:  # noqa: BLE001
            # Never assume fresh after a failed probe.  Conservatively mark both
            # endpoints reachable, retain a failure manifest, and release only our
            # own lock after the ledger append below.
            post = {"ros": True, "gazebo": True}
            failure_class = "PROTOCOL_FAILURE"
            runtime_error = runtime_error or f"postflight endpoint probe failed: {exc!r}"
            lifecycle.append({"event": "postflight_probe_failed", "utc": utc_now(), "error": repr(exc)})
        if post["ros"] or post["gazebo"]:
            failure_class = "PROTOCOL_FAILURE"
            runtime_error = runtime_error or "postflight ROS/Gazebo master remained reachable"
        required_firewall_checkpoints_ok = (not formal) or ({report.get("checkpoint") for report in firewall_reports} == REQUIRED_FIREWALL_CHECKPOINTS and all(report.get("status") == "PASS" for report in firewall_reports))
        development_firewall_checkpoints_ok = development_firewall is None
        if development_firewall is not None:
            try:
                observed = {
                    str(item.get("checkpoint")): item
                    for item in development_firewall_snapshot_reports
                }
                require(
                    set(observed) == set(DEVELOPMENT_H0_FIREWALL_CHECKPOINTS),
                    "development H0 firewall lacks one or more required live snapshots",
                )
                for checkpoint in DEVELOPMENT_H0_FIREWALL_CHECKPOINTS:
                    item = observed[checkpoint]
                    path = ensure_within(
                        Path(str(item.get("path", ""))),
                        case_dir,
                        f"development H0 firewall {checkpoint} snapshot",
                    )
                    _report, current_hash = load_development_h0_firewall_snapshot(
                        path,
                        checkpoint,
                        case_dir,
                        case_launch_manifest_hash,
                        development_firewall,
                    )
                    require(
                        current_hash == item.get("hash"),
                        f"development H0 firewall {checkpoint} snapshot changed after capture",
                    )
                development_firewall_checkpoints_ok = True
            except ContractError as exc:
                development_firewall_checkpoints_ok = False
                runtime_error = runtime_error or str(exc)
                lifecycle.append(
                    {
                        "event": "development_firewall_snapshot_revalidation_failed",
                        "utc": utc_now(),
                        "error": str(exc),
                    }
                )
        formal_runtime_ack_still_bound = (
            not formal
            or (
                runtime_ack_path is not None
                and runtime_ack_hash is not None
                and runtime_ack_path.is_file()
                and sha256_file(runtime_ack_path) == runtime_ack_hash
            )
        )
        formal_motion_stop_ok = not formal
        if formal:
            try:
                formal_motion_stop_ok = bool(
                    motion_stop_ack is not None
                    and motion_stop_ack_path is not None
                    and motion_stop_ack_hash is not None
                    and motion_stop_ack_path.is_file()
                    and sha256_file(motion_stop_ack_path) == motion_stop_ack_hash
                    and validate_motion_stop_ack(
                        motion_stop_ack_path,
                        case_launch_manifest_hash,
                        expected_schema_hash=str(formal_launch_contract["motion_stop_ack_schema_hash"]),
                    ).get("status") == "PASS"
                )
            except ContractError as exc:
                formal_motion_stop_ok = False
                runtime_error = runtime_error or str(exc)
                lifecycle.append({"event": "motion_stop_ack_revalidation_failed", "utc": utc_now(), "error": str(exc)})
        formal_motion_release_ok = not formal
        if formal:
            try:
                formal_motion_release_ok = bool(
                    motion_release_ack is not None
                    and motion_release_ack_path is not None
                    and motion_release_ack_hash is not None
                    and motion_release_ack_path.is_file()
                    and sha256_file(motion_release_ack_path) == motion_release_ack_hash
                    and validate_motion_release_ack(
                        motion_release_ack_path,
                        case_launch_manifest_hash,
                        expected_schema_hash=str(formal_launch_contract["motion_release_ack_schema_hash"]),
                    ).get("status") == "PASS"
                )
            except ContractError as exc:
                formal_motion_release_ok = False
                runtime_error = runtime_error or str(exc)
                lifecycle.append({"event": "motion_release_ack_revalidation_failed", "utc": utc_now(), "error": str(exc)})
        if formal and not formal_runtime_ack_still_bound:
            failure_class = "PROTOCOL_FAILURE"
            runtime_error = runtime_error or "runtime acknowledgement changed after validation"
        if formal and not formal_motion_stop_ok:
            failure_class = "PROTOCOL_FAILURE"
            runtime_error = runtime_error or "formal frozen tail lacks command-zero acknowledgement"
        if formal and not formal_motion_release_ok:
            failure_class = "PROTOCOL_FAILURE"
            runtime_error = runtime_error or "formal runtime lacks immutable actual-motion acknowledgement"
        method_success = goal_reached and failure_class == "NONE" and not post["ros"] and not post["gazebo"] and tail_complete and required_firewall_checkpoints_ok and development_firewall_checkpoints_ok and formal_runtime_ack_still_bound and formal_motion_stop_ok and formal_motion_release_ok
        if not method_success and failure_class == "NONE":
            failure_class = "METHOD_FAILURE" if motion_may_have_started else "PROTOCOL_FAILURE"
        method_failure, retry_possible = validate_failure_class(failure_class, motion_may_have_started, method_success)
        retry_possible = retry_possible and (not formal or failure_classification is not None)
        # A dry-run has no physical outcome either, but retaining the proxy class
        # makes it impossible for downstream code to promote H_proxy/H_modal.
        classification = "FORMAL_EXECUTION" if formal else DEVELOPMENT_EVIDENCE_CLASS
        if not truth["eligible"]:
            classification = PROXY_EVIDENCE_CLASS
        measurement = {
            "H_proxy": {
                "topic": "/slosh/height",
                "semantic": "model_proxy_mechanism_only",
                "physical_primary_eligible": False,
            },
            "H_modal": {
                "topic": "/sim_spmpc/slosh_height",
                "semantic": "controller_internal_modal_only",
                "physical_primary_eligible": False,
            },
        }
        declared_liquid_plant = spec.get("liquid_plant_capability")
        # An H0 adapter may explicitly record an independent plant surrogate,
        # but it must be visible as UNVALIDATED record-only H_plant evidence
        # rather than silently disappear into a generic proxy classification.
        # Do not accept a looser self-declaration here: every non-primary
        # status marker is required before it can occupy the H_plant field.
        if (
            isinstance(declared_liquid_plant, Mapping)
            and declared_liquid_plant.get("independent_plant") is True
            and declared_liquid_plant.get("development_only") is True
            and declared_liquid_plant.get("formal") is False
            and declared_liquid_plant.get("fidelity_validation_status") == "UNVALIDATED"
            and declared_liquid_plant.get("physical_primary_eligible") is False
            and declared_liquid_plant.get("truth_topic") == "/sim_truth/liquid_height"
        ):
            measurement["H_plant"] = {
                "topic": "/sim_truth/liquid_height",
                "semantic": "independent_development_surrogate_UNVALIDATED_not_physical_primary",
                "fidelity_validation_status": "UNVALIDATED",
                "physical_primary_eligible": False,
            }
        elif truth["eligible"]:
            measurement["H_plant"] = {
                "topic": truth["truth_topic"],
                "semantic": "independent_plant_measurement",
                "physical_primary_eligible": False,
            }
        effective_path = case_dir / "effective_config.json"
        finalization_stage = "effective_config_write"
        write_json_new(effective_path, effective_config)
        finalization_stage = "postflight_build"
        postflight_contract_ok = (
            not post["ros"]
            and not post["gazebo"]
            and tail_complete
            and (not development_h0_runtime or dry_run or effective_config_readback is not None)
            and development_firewall_checkpoints_ok
            and (not formal or (required_firewall_checkpoints_ok and formal_runtime_ack_still_bound and formal_motion_release_ok and formal_motion_stop_ok))
        )
        postflight = {
            "schema_version": SCHEMA_VERSION,
            "attempt_id": attempt_id,
            "status": "PASS" if postflight_contract_ok else "FAIL",
            "endpoint_status": "PASS" if not post["ros"] and not post["gazebo"] else "FAIL",
            "method_outcome_status": "METHOD_SUCCESS" if method_success else failure_class,
            "pre_ros_reachable": False,
            "pre_gazebo_reachable": False,
            "ros_master_uri": ros_uri,
            "gazebo_master_uri": gazebo_uri,
            "post_ros_reachable": post["ros"],
            "post_gazebo_reachable": post["gazebo"],
            "recorder_before_motion": any(item["event"] == "recorder_started" for item in lifecycle) and motion_started,
            "recorder_ready_timeout_sec": recorder_ready_timeout_sec,
            "tail_recorded": tail_complete,
            "tail_sec": tail_sec,
            "post_shutdown_sec": post_shutdown_sec,
            "owned_pid_cleanup_only": True,
            "master_lock_held_through_dataset_index": True,
            "firewall_reports": firewall_reports,
            "development_firewall": (
                {
                    "contract_hash": development_firewall.get("contract_hash"),
                    "controller_nodes": development_firewall.get("controller_nodes"),
                    "development_only": True,
                    "formal": False,
                    "physical_primary_eligible": False,
                }
                if development_firewall is not None
                else None
            ),
            "development_firewall_snapshots": development_firewall_snapshot_reports,
            "development_firewall_checkpoints_ok": development_firewall_checkpoints_ok,
            "development_firewall_status": (
                "NOT_LIVE_DRY_RUN"
                if development_firewall_not_live
                else ("PASS" if development_firewall is not None and development_firewall_checkpoints_ok else "NOT_APPLICABLE")
            ),
            "runtime_ack_path": str(runtime_ack_path) if runtime_ack_path is not None else None,
            "runtime_ack_hash": runtime_ack_hash,
            "effective_config_readback_path": str(effective_config_readback_path) if effective_config_readback_path is not None else None,
            "effective_config_readback_hash": effective_config_readback_hash,
            "motion_stop_ack_path": str(motion_stop_ack_path) if motion_stop_ack_path is not None else None,
            "motion_stop_ack_hash": motion_stop_ack_hash,
            "motion_release_ack_path": str(motion_release_ack_path) if motion_release_ack_path is not None else None,
            "motion_release_ack_hash": motion_release_ack_hash,
            "recorder_size_before_motion": recorder_size_before_motion,
            "recorder_size_before_tail": recorder_size_before_tail,
            "created_utc": utc_now(),
        }
        postflight_path = case_dir / "postflight.json"
        finalization_stage = "postflight_write"
        write_json_new(postflight_path, postflight)
        finalization_stage = "attempt_manifest_build"
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "attempt_id": attempt_id,
            "planned_row_id": row["planned_row_id"],
            "condition_id": row["condition_id"],
            "stage": row["stage"],
            "block_id": row["block_id"],
            "order_position": row["order_position"],
            "formal": formal,
            "formal_master_hash": formal_master_document.get("master_hash") if formal_master_document is not None else None,
            "formal_freeze_hash": formal_freeze_hash,
            "dataset_ledger_id": formal_dataset_ledger.get("ledger_id") if formal_dataset_ledger is not None else None,
            "dataset_ledger_identity_hash": formal_dataset_ledger.get("ledger_identity_hash") if formal_dataset_ledger is not None else None,
            "dataset_root": str(output_root.resolve()),
            "evidence_class": classification,
            "development_class": DEVELOPMENT_EVIDENCE_CLASS if not formal else None,
            "attempt_number": attempt_number,
            "retry_of_attempt_id": None if attempt_number == 1 else f"{attempt_prefix(attempt_id)}_r{attempt_number - 1:02d}",
            "retry_authorization_path": str(spec.get("retry_authorization")) if retry_authorization is not None else None,
            "retry_authorization_hash": retry_authorization.get("authorization_hash") if retry_authorization is not None else None,
            "retry_authorization_id": retry_authorization.get("authorization_id") if retry_authorization is not None else None,
            "planned_block_segment_id": row["planned_block_segment_id"],
            "actual_block_segment_id": str(spec.get("actual_block_segment_id", row["planned_block_segment_id"])),
            "split_block": str(spec.get("actual_block_segment_id", row["planned_block_segment_id"])) != row["planned_block_segment_id"],
            "seed_bundle_id": row.get("seed_bundle_id"),
            "seed_bundle_hash": row.get("seed_bundle_hash"),
            "seed_bundle_path": str(seed_bundle_path),
            "seed_trace_hashes": seed_report["trace_hashes"],
            "case_launch_manifest_path": str(case_launch_manifest_path),
            "case_launch_manifest_hash": case_launch_manifest_hash,
            "runtime_input_acknowledgement": runtime_ack,
            "runtime_input_acknowledgement_path": str(runtime_ack_path) if runtime_ack_path is not None else None,
            "runtime_input_acknowledgement_hash": runtime_ack_hash,
            "effective_config_readback": effective_config_readback,
            "effective_config_readback_path": str(effective_config_readback_path) if effective_config_readback_path is not None else None,
            "effective_config_readback_hash": effective_config_readback_hash,
            "motion_stop_acknowledgement": motion_stop_ack,
            "motion_stop_acknowledgement_path": str(motion_stop_ack_path) if motion_stop_ack_path is not None else None,
            "motion_stop_acknowledgement_hash": motion_stop_ack_hash,
            "motion_release_acknowledgement": motion_release_ack,
            "motion_release_acknowledgement_path": str(motion_release_ack_path) if motion_release_ack_path is not None else None,
            "motion_release_acknowledgement_hash": motion_release_ack_hash,
            "development_motion_event": development_motion_event,
            "development_motion_event_path": str(development_motion_event_path) if development_motion_event_path is not None else None,
            "development_motion_event_hash": development_motion_event_hash,
            "development_goal_event": development_goal_event,
            "development_goal_event_path": str(development_goal_event_path) if development_goal_event_path is not None else None,
            "development_goal_event_hash": development_goal_event_hash,
            "failure_classification": failure_classification,
            "failure_classification_path": str(failure_classification_path.resolve()) if failure_classification_path is not None else None,
            "failure_classification_hash": failure_classification_hash,
            "failure_event_path": str(failure_event_path.resolve()) if failure_event_path is not None else None,
            "failure_event_hash": failure_event_hash,
            "hashes": hashes,
            "effective_config_path": str(effective_path),
            "effective_config_dump_hash": sha256_file(effective_path),
            "map_world_path_explicit": True,
            "ros_master_uri": ros_uri,
            "gazebo_master_uri": gazebo_uri,
            "pre_ros_reachable": False,
            "pre_gazebo_reachable": False,
            "post_ros_reachable": post["ros"],
            "post_gazebo_reachable": post["gazebo"],
            "fresh_ros_gazebo": not post["ros"] and not post["gazebo"],
            "master_lock_held_through_dataset_index": True,
            "settle_sec": settle_sec,
            "goal_timeout_sec": timeout_sec,
            "tail_sec": tail_sec,
            "post_shutdown_sec": post_shutdown_sec,
            "recorder_ready_timeout_sec": recorder_ready_timeout_sec,
            "motion_started": motion_started,
            "motion_may_have_started": motion_may_have_started,
            "assignment_consumed": assignment_consumed,
            "goal_reached": goal_reached,
            "first_effective_motion_utc": first_effective_motion_utc,
            "first_arrival_utc": first_arrival_utc,
            "tail_end_utc": tail_end_utc,
            "motion_launch_monotonic": motion_launch_monotonic,
            "motion_deadline_monotonic": motion_deadline_monotonic,
            "method_success": method_success,
            "method_failure": method_failure,
            "failure_class": failure_class,
            "retry_authorization_allowed": retry_possible,
            "primary_physical_efficacy_eligible": formal and not dry_run and method_success and truth["eligible"] and required_firewall_checkpoints_ok and formal_runtime_ack_still_bound and formal_motion_release_ok and formal_motion_stop_ok,
            "liquid_plant_capability": (
                dict(spec["liquid_plant_capability"])
                if isinstance(spec.get("liquid_plant_capability"), Mapping)
                else None
            ),
            "liquid_truth_capability": truth,
            "development_firewall": (
                {
                    "contract_hash": development_firewall.get("contract_hash"),
                    "controller_nodes": development_firewall.get("controller_nodes"),
                    "development_only": True,
                    "formal": False,
                    "physical_primary_eligible": False,
                }
                if development_firewall is not None
                else None
            ),
            "development_firewall_snapshots": development_firewall_snapshot_reports,
            "development_firewall_checkpoints_ok": development_firewall_checkpoints_ok,
            "development_firewall_status": (
                "NOT_LIVE_DRY_RUN"
                if development_firewall_not_live
                else ("PASS" if development_firewall is not None and development_firewall_checkpoints_ok else "NOT_APPLICABLE")
            ),
            "measurement_channels": measurement,
            "controller_nodes": sorted(controller_nodes),
            "lifecycle": lifecycle,
            "postflight_path": str(postflight_path),
            "postflight_hash": sha256_file(postflight_path),
            "created_utc": utc_now(),
            "dry_run": dry_run,
            "runner_error": runtime_error,
        }
        manifest_path = case_dir / "attempt_manifest.json"
        finalization_stage = "attempt_manifest_write"
        write_json_new(manifest_path, manifest)
        finalization_stage = "dataset_index_build"
        index_record = {
            "schema_version": SCHEMA_VERSION,
            "attempt_id": attempt_id,
            "planned_row_id": row["planned_row_id"],
            "attempt_manifest": str(manifest_path),
            "attempt_manifest_hash": sha256_file(manifest_path),
            "stage": row["stage"],
            "condition_id": row["condition_id"],
            "formal": formal,
            "formal_master_hash": formal_master_document.get("master_hash") if formal_master_document is not None else None,
            "dataset_ledger_id": formal_dataset_ledger.get("ledger_id") if formal_dataset_ledger is not None else None,
            "dataset_ledger_identity_hash": formal_dataset_ledger.get("ledger_identity_hash") if formal_dataset_ledger is not None else None,
            "failure_class": failure_class,
            "method_failure": method_failure,
            "method_success": method_success,
            "split_block": manifest["split_block"],
            "actual_block_segment_id": manifest["actual_block_segment_id"],
            "continuous_eligibility": method_success and postflight["tail_recorded"] and not manifest["split_block"] and (not formal or (formal_motion_release_ok and formal_motion_stop_ok)),
            "retry_authorization_path": manifest["retry_authorization_path"],
            "retry_authorization_hash": manifest["retry_authorization_hash"],
            "effective_config_readback_path": manifest["effective_config_readback_path"],
            "effective_config_readback_hash": manifest["effective_config_readback_hash"],
            "development_firewall_contract_hash": (
                development_firewall.get("contract_hash")
                if development_firewall is not None
                else None
            ),
            "development_firewall_snapshot_hashes": [
                item["hash"] for item in development_firewall_snapshot_reports
            ],
            "development_firewall_checkpoints_ok": development_firewall_checkpoints_ok,
            "development_firewall_status": manifest["development_firewall_status"],
            "created_utc": utc_now(),
        }
        finalization_stage = "dataset_index_append"
        index_entry = append_dataset_index(dataset_index_path, index_record)
        manifest["dataset_index_entry_hash"] = index_entry["entry_hash"]
        # Preserve append-only behavior: manifest was not mutated after its hash was indexed.
        finalization_stage = "master_lock_release"
        if not release_master_lock(master_lock_path, master_lock_token):
            lock_recovery = {
                "schema_version": SCHEMA_VERSION,
                "record_type": "SMPCC_SIM_FINALIZATION_RECOVERY",
                "status": "LOCK_RELEASE_AUDIT_REQUIRED",
                "ledger_admission": "CONFIRMED_INDEX_APPEND",
                "attempt_id": attempt_id,
                "planned_row_id": row["planned_row_id"],
                "formal": formal,
                "finalization_stage": finalization_stage,
                "attempt_manifest_path": str(manifest_path),
                "dataset_index_path": str(dataset_index_path),
                "dataset_index_entry_hash": index_entry["entry_hash"],
                "created_utc": utc_now(),
            }
            try:
                write_json_new(recovery_marker_path, lock_recovery)
            except BaseException:
                pass
            raise ContractError("owned master lock release could not be verified; ledger requires audit")
        lock_release_verified = True
        finalization_stage = "result_return"
        return {
            "status": "PASS" if method_success else "FAIL",
            "attempt_manifest": str(manifest_path),
            "attempt_manifest_hash": sha256_file(manifest_path),
            "postflight": str(postflight_path),
            "dataset_index": str(output_root / "dataset_index.jsonl"),
            "dataset_index_entry_hash": index_entry["entry_hash"],
            "formal": formal,
            "evidence_class": classification,
        }
    except BaseException as exc:
        # An index append may fail after a partial OS-level write.  Therefore
        # no error path is evidence of ledger admission or eligible for reuse.
        recovery = {
            "schema_version": SCHEMA_VERSION,
            "record_type": "SMPCC_SIM_FINALIZATION_RECOVERY",
            "status": "RECOVERY_REQUIRED",
            "ledger_admission": "UNCONFIRMED_DO_NOT_CONSUME",
            "attempt_id": attempt_id,
            "planned_row_id": row["planned_row_id"],
            "formal": formal,
            "finalization_stage": finalization_stage,
            "error_type": type(exc).__name__,
            "error": repr(exc),
            "effective_config_path": str(effective_path),
            "postflight_path": str(postflight_path),
            "attempt_manifest_path": str(manifest_path),
            "dataset_index_path": str(dataset_index_path),
            "created_utc": utc_now(),
        }
        try:
            write_json_new(recovery_marker_path, recovery)
        except BaseException:
            # Storage may be the original failure; do not mask it or let it
            # skip the owned-lock cleanup in the finally below.
            pass
        raise
    finally:
        if not lock_release_verified:
            release_master_lock(master_lock_path, master_lock_token)


def h0_fixture_assets(root: Path) -> Dict[str, str]:
    """Create clearly labelled development-only JSON assets below /data, never repo freeze inputs."""
    assets = root / "fixture_assets"
    assets.mkdir(parents=True, exist_ok=True)
    path = assets / "H0_fixture_path.json"
    world = assets / "H0_fixture_world.json"
    map_file = assets / "H0_fixture_map.pbstream"
    config = assets / "Bsmooth_effective_config.json"
    if not path.exists():
        write_json_new(path, {"path_id": "H0", "source_mode": "frozen_json_replay", "development_fixture": True, "points": [{"x": 0.0, "y": 0.0, "yaw": 0.0}, {"x": 1.0, "y": 0.15, "yaw": 0.1}, {"x": 2.0, "y": 0.0, "yaw": 0.0}], "zones": {"Z1": [0, 1], "Z2": [1, 2]}})
    if not world.exists():
        write_json_new(world, {"development_fixture": True, "bounds": [-2.0, 4.0, -2.0, 2.0], "obstacles": []})
    if not map_file.exists():
        write_text_new(map_file, "SIM-DEV-H0 fixture map placeholder; not a frozen formal map\n")
    if not config.exists():
        write_json_new(config, {"condition_id": "Bsmooth", "w_control": 0.3, "w_smooth": 1.0, "w_alpha": 1.0, "w_du_a": 1.0, "w_du_vs": 1.0, "w_slosh": 0.0, "v_ref": 0.20, "observer": {"nominal_source": "odom"}, "delay": {"mode": "off", "linear_sec": -1.0, "angular_sec": -1.0}})
    return {"map_file": str(map_file), "world_file": str(world), "path_file": str(path), "effective_config_file": str(config)}


def emit(value: Any) -> None:
    print(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True))


def command_generate(args: argparse.Namespace) -> int:
    freeze = require_mapping(read_json(args.freeze), "freeze input")
    master = make_master_rows(freeze, args.seed, fixture=args.fixture, formal_freeze_path=None if args.fixture else args.freeze)
    write_json_new(args.output, master)
    report = validate_master(master, require_formal=not args.fixture)
    emit({"output": str(args.output), "validation": report})
    return 0 if report["status"] == "PASS" else 2


def command_validate(args: argparse.Namespace) -> int:
    master = read_json(args.master)
    report = validate_master(master, require_formal=args.require_formal)
    emit(report)
    return 0 if report["status"] == "PASS" else 2


def command_formal_gate(args: argparse.Namespace) -> int:
    report = validate_formal_freeze(read_json(args.freeze))
    emit(report)
    return 0 if report["status"] == "PASS" else 2


def command_config_diff(args: argparse.Namespace) -> int:
    try:
        report = compare_smoothmatch(read_json(args.bsmooth), read_json(args.smoothmatch))
    except ContractError as exc:
        emit({"status": "FAIL", "errors": [str(exc)]})
        return 2
    emit(report)
    return 0


def command_path_validate(args: argparse.Namespace) -> int:
    try:
        transform = require_mapping(read_json(args.transform), "transform")
        world = require_mapping(read_json(args.world), "world")
        report = validate_path_replay(args.source, args.derived, transform, world, args.clearance_m, args.tolerance)
    except ContractError as exc:
        emit({"status": "FAIL", "errors": [str(exc)]})
        return 2
    emit(report)
    return 0


def command_truth_gate(args: argparse.Namespace) -> int:
    report = validate_truth_capability(read_json(args.capability))
    emit(report)
    return 0 if report["eligible"] else 2


def command_firewall(args: argparse.Namespace) -> int:
    try:
        graph = require_mapping(read_json(args.graph), "graph")
        report = validate_controller_firewall(graph, args.controller_node)
    except ContractError as exc:
        emit({"status": "FAIL", "errors": [str(exc)]})
        return 2
    emit(report)
    return 0 if report["status"] == "PASS" else 2


def command_authorize_retry(args: argparse.Namespace) -> int:
    try:
        classifier = read_json(args.retry_classifier) if args.retry_classifier is not None else None
        report = authorize_retry(
            args.previous_manifest,
            args.output,
            args.next_attempt_id,
            classifier_id=args.classifier_id,
            retry_classifier=classifier,
        )
    except ContractError as exc:
        emit({"status": "FAIL", "errors": [str(exc)]})
        return 2
    emit(report)
    return 0


def command_run(args: argparse.Namespace) -> int:
    try:
        row = require_mapping(read_json(args.row), "row")
        spec = require_mapping(read_json(args.spec), "run spec")
        report = run_single_row(row, spec, args.output_root, args.sim_root)
    except ContractError as exc:
        emit({"status": "FAIL", "errors": [str(exc)]})
        return 2
    emit(report)
    return 0 if report["status"] == "PASS" else 2


def command_h0_smoke(args: argparse.Namespace) -> int:
    try:
        output_root = ensure_within(args.output_root, args.sim_root, "H0 smoke output root")
        assets = h0_fixture_assets(output_root)
        row = minimal_h0_row()
        spec = {
            "dry_run": True,
            "attempt_id": row["planned_row_id"] + "_r01",
            "assets": assets,
            "seed_bundle": make_seed_bundle("SIM-DEV-H0-fixture-seed-v1", "SIM-DEV-H0_SMOKE", 1),
            "ros_master_uri": args.ros_master_uri,
            "gazebo_master_uri": args.gazebo_master_uri,
            "settle_sec": 30.0,
            "goal_timeout_sec": 60.0,
            "tail_sec": args.tail_sec,
            "simulate_goal_reached": args.simulate_goal_reached,
        }
        report = run_single_row(row, spec, output_root, args.sim_root)
    except ContractError as exc:
        emit({"status": "FAIL", "errors": [str(exc)], "formal": False})
        return 2
    emit(report)
    return 0 if report["status"] == "PASS" else 2


def command_ledger_summary(args: argparse.Namespace) -> int:
    try:
        report = summarize_ledger(require_mapping(read_json(args.master), "master"), args.index)
    except ContractError as exc:
        emit({"status": "FAIL", "errors": [str(exc)]})
        return 2
    emit(report)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Fail-closed SMPCC-SIM 40/64/88 matrix toolchain")
    sub = parser.add_subparsers(dest="command", required=True)

    item = sub.add_parser("generate", help="generate a fixture or validated formal master planned-row matrix")
    item.add_argument("--freeze", type=Path, required=True)
    item.add_argument("--seed", default=None, help="fixture-only randomization seed; formal generation consumes frozen tables")
    item.add_argument("--output", type=Path, required=True)
    item.add_argument("--fixture", action="store_true", help="require explicit fixture=true; output is never formal")
    item.set_defaults(func=command_generate)

    item = sub.add_parser("validate", help="validate planned rows, randomization, seed bundles and denominators")
    item.add_argument("--master", type=Path, required=True)
    item.add_argument("--require-formal", action="store_true")
    item.set_defaults(func=command_validate)

    item = sub.add_parser("formal-gate", help="verify all formal prerequisites without generating data")
    item.add_argument("--freeze", type=Path, required=True)
    item.set_defaults(func=command_formal_gate)

    item = sub.add_parser("config-diff", help="prove SmoothMatch differs from Bsmooth only by v_ref")
    item.add_argument("--bsmooth", type=Path, required=True)
    item.add_argument("--smoothmatch", type=Path, required=True)
    item.set_defaults(func=command_config_diff)

    item = sub.add_parser("path-validate", help="verify frozen JSON replay transform, curvature and clearance")
    item.add_argument("--source", type=Path, required=True)
    item.add_argument("--derived", type=Path, required=True)
    item.add_argument("--transform", type=Path, required=True)
    item.add_argument("--world", type=Path, required=True)
    item.add_argument("--clearance-m", type=float, required=True)
    item.add_argument("--tolerance", type=float, default=1e-6)
    item.set_defaults(func=command_path_validate)

    item = sub.add_parser("truth-gate", help="require independent plant capability before physical primary")
    item.add_argument("--capability", type=Path, required=True)
    item.set_defaults(func=command_truth_gate)

    item = sub.add_parser("firewall", help="validate runtime ROS graph against controller truth/proxy subscriptions")
    item.add_argument("--graph", type=Path, required=True)
    item.add_argument("--controller-node", action="append", required=True)
    item.set_defaults(func=command_firewall)

    item = sub.add_parser("authorize-retry", help="authorize only pre-motion method-independent acquisition retry")
    item.add_argument("--previous-manifest", type=Path, required=True)
    item.add_argument("--next-attempt-id", required=True)
    classifier_source = item.add_mutually_exclusive_group(required=True)
    classifier_source.add_argument("--classifier-id", help="development-only classifier label; never accepted for formal attempts")
    classifier_source.add_argument("--retry-classifier", type=Path, help="frozen formal retry-classifier declaration")
    item.add_argument("--output", type=Path, required=True)
    item.set_defaults(func=command_authorize_retry)

    item = sub.add_parser("run", help="execute exactly one row with fresh lifecycle or explicit dry-run")
    item.add_argument("--row", type=Path, required=True)
    item.add_argument("--spec", type=Path, required=True)
    item.add_argument("--output-root", type=Path, required=True)
    item.add_argument("--sim-root", type=Path, default=DEFAULT_SIM_ROOT)
    item.set_defaults(func=command_run)

    item = sub.add_parser("h0-smoke", help="development-only H0 dry-run smoke; never formal data")
    item.add_argument("--output-root", type=Path, required=True)
    item.add_argument("--sim-root", type=Path, default=DEFAULT_SIM_ROOT)
    item.add_argument("--ros-master-uri", default="127.0.0.1:11430")
    item.add_argument("--gazebo-master-uri", default="127.0.0.1:11464")
    item.add_argument("--tail-sec", type=float, default=5.0)
    item.add_argument("--simulate-goal-reached", action="store_true", default=True)
    item.set_defaults(func=command_h0_smoke)

    item = sub.add_parser("ledger-summary", help="derive fixed denominator N_plan/N_attempt/N_method/N_pair")
    item.add_argument("--master", type=Path, required=True)
    item.add_argument("--index", type=Path, required=True)
    item.set_defaults(func=command_ledger_summary)
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return int(args.func(args))
    except ContractError as exc:
        emit({"status": "FAIL", "errors": [str(exc)]})
        return 2


if __name__ == "__main__":
    sys.exit(main())
