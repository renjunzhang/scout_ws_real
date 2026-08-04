#!/usr/bin/env python3
"""Fail-closed, hash-bound formal runtime ABI for SMPCC-SIM.

The generic :mod:`smpcc_sim_toolchain` already owns the one-row lifecycle:
fresh ROS/Gazebo masters, one owned process group per child, 30 s settling,
recorder-before-motion, a conservative no-more-than-60 s effective-motion
window, frozen tails after either outcome, and the three runtime/motion ACKs.
This adapter is the narrow bridge between that lifecycle and a reviewed,
formal simulator backend.

It deliberately does *not* contain a fallback ROS/Gazebo launch recipe.  A
formal freeze has to name a separate, hash-bound ``SMPCC_SIM_FORMAL_RUNTIME_
BACKEND`` manifest.  That manifest freezes exactly eight concrete delegate
commands, their executable/file hashes, case-local artifact names, and the
lifecycle invariants.  The outer frozen launch contract invokes this adapter;
the adapter re-verifies master/freeze/case/seed/assets and then ``execve`` s
only the corresponding delegate.  Thus no runtime command can acquire a
condition, path, config, profile, container, Bslosh variant, or process-kill
strategy from mutable CLI input.

There are intentionally no built-in formal assets or backend delegates in
this repository.  Missing, development, fixture, H0, W5_S10, proxy-only, or
unhashed input remains a NO-GO.  The optional ``run-formal-row`` entry point
requires an explicit execution flag and delegates the actual lifecycle to the
already-tested toolchain; it is not used by unit tests or development smoke.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence


def _load_toolchain():
    """Load the sibling protocol module without requiring a ROS install."""
    existing = sys.modules.get("smpcc_sim_toolchain")
    if existing is not None:
        return existing
    module_path = Path(__file__).with_name("smpcc_sim_toolchain.py")
    spec = importlib.util.spec_from_file_location("smpcc_sim_toolchain", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load SMPCC-SIM toolchain: {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules["smpcc_sim_toolchain"] = module
    spec.loader.exec_module(module)
    return module


toolchain = _load_toolchain()


def _load_source_separation():
    """Load the execution-only R8 source-isolation gate."""
    module_path = Path(__file__).with_name("smpcc_sim_source_separation.py")
    spec = importlib.util.spec_from_file_location(
        "smpcc_sim_source_separation", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load source-separation gate: {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


source_separation = _load_source_separation()


ADAPTER_ID = "SMPCC-SIM-FORMAL-RUNTIME-ADAPTER-v2"
ADAPTER_SCHEMA_VERSION = "smpcc-sim-formal-runtime-adapter-v2"
DEFAULT_SIM_ROOT = Path("/data/a/scout_sim_replacement")

BACKEND_DOCUMENT_TYPE = "SMPCC_SIM_FORMAL_RUNTIME_BACKEND"
BACKEND_STATUS = "FROZEN"
BACKEND_ENTRY_PATH_KEY = "backend_manifest_path"
BACKEND_ENTRY_HASH_KEY = "backend_manifest_hash"
BACKEND_ARTIFACT_KEYS = (
    "recorder_artifact",
    "runtime_ack",
    "motion_release_ack",
    "motion_stop_ack",
)
BACKEND_REQUIRED_ENVIRONMENT = (
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
)

# These fields are written by this adapter only after it has resolved the
# backend from the formal freeze.  Delegates may use them for their own
# readback/diagnostics, but they must never treat a caller-provided value as a
# second asset-selection channel.
BACKEND_BOUND_ENVIRONMENT = (
    "SMPCC_FORMAL_RUNTIME_BACKEND_PATH",
    "SMPCC_FORMAL_RUNTIME_BACKEND_FILE_SHA256",
    "SMPCC_FORMAL_RUNTIME_BACKEND_ID",
    "SMPCC_FORMAL_RUNTIME_BACKEND_HASH",
    "SMPCC_FORMAL_RUNTIME_BACKEND_COMMAND_FIELD",
)

# The keys are deliberately the runner's exact ABI fields, not an ad-hoc
# collection of convenience commands.  A frozen launch contract must call the
# corresponding subcommand of this adapter without condition/path/config
# arguments; those inputs are selected only from the formal master/freeze.
ABI_SUBCOMMANDS = {
    "launch_command": "launch",
    "ready_command": "ready",
    "recorder_command": "recorder",
    "motion_command": "motion",
    "goal_probe_command": "goal-probe",
    "runtime_ack_command": "runtime-ack",
    "motion_release_ack_command": "motion-release-ack",
    "motion_stop_command": "motion-stop",
}
SUBCOMMAND_TO_FIELD = {subcommand: field for field, subcommand in ABI_SUBCOMMANDS.items()}

REQUIRED_ENVIRONMENT = (
    "SMPCC_CASE_LAUNCH_MANIFEST_PATH",
    "SMPCC_CASE_LAUNCH_MANIFEST_SHA256",
    "SMPCC_SEED_BUNDLE_PATH",
    "SMPCC_SEED_BUNDLE_SHA256",
    "SMPCC_FORMAL_FREEZE_PATH",
    "SMPCC_FORMAL_FREEZE_FILE_SHA256",
    "SMPCC_FORMAL_MASTER_PATH",
    "SMPCC_FORMAL_MASTER_FILE_SHA256",
)

FORBIDDEN_ADAPTER_ARGUMENT_PREFIXES = (
    "--condition",
    "--path",
    "--config",
    "--profile",
    "--container",
    "--variant",
    "--bslosh",
    "--h0",
)
LEGACY_DEFAULT_BSLOSH_VARIANTS = frozenset(
    (
        "b_slosh",
        "b_slosh_hard",
        "b_slosh_linear",
        "b_slosh_anti",
        "b_ours",
        "b_ours_hard",
        "b_ours_anti",
    )
)
LEGACY_VARIANT_SELECTOR_KEYS = frozenset(
    (
        "planner_variant",
        "runtime_variant",
        "variant",
        "default_variant",
        "implementation_variant",
    )
)


class AdapterError(RuntimeError):
    """A fail-closed formal-runtime adapter contract error."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AdapterError(message)


def _toolchain_call(callback, *args, **kwargs):
    try:
        return callback(*args, **kwargs)
    except toolchain.ContractError as exc:
        raise AdapterError(str(exc)) from exc


def _read_json(path: Path, label: str) -> Mapping[str, Any]:
    value = _toolchain_call(toolchain.read_json, path)
    require(isinstance(value, Mapping), f"{label} must be a JSON object")
    return value


def _sha256_file(path: Path) -> str:
    return str(_toolchain_call(toolchain.sha256_file, path))


def _canonical_hash(value: Any) -> str:
    return str(_toolchain_call(toolchain.canonical_hash, value))


def _is_sha256(value: Any) -> bool:
    return bool(_toolchain_call(toolchain.is_sha256, value))


def _absolute_existing_file(value: Any, label: str) -> Path:
    require(isinstance(value, str) and value, f"{label} path is missing")
    path = Path(value)
    require(path.is_absolute(), f"{label} path must be absolute")
    require(path.is_file(), f"{label} file is missing: {path}")
    return path.resolve()


def _bound_json_from_environment(
    environment: Mapping[str, str],
    path_key: str,
    hash_key: str,
    label: str,
) -> tuple[Path, Mapping[str, Any], str]:
    path = _absolute_existing_file(environment.get(path_key), label)
    expected = environment.get(hash_key)
    require(_is_sha256(expected), f"{label} missing {hash_key}")
    actual = _sha256_file(path)
    require(actual == expected, f"{label} file hash mismatch")
    return path, _read_json(path, label), actual


def _ensure_below(path: Path, root: Path, label: str) -> Path:
    resolved = path.resolve()
    try:
        resolved.relative_to(root.resolve())
    except ValueError as exc:
        raise AdapterError(f"{label} must be below simulation root {root.resolve()}: {resolved}") from exc
    return resolved


def _iter_strings(value: Any) -> Iterable[str]:
    if isinstance(value, str):
        yield value
    elif isinstance(value, Mapping):
        for key, item in value.items():
            yield str(key)
            yield from _iter_strings(item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            yield from _iter_strings(item)


def reject_development_or_rejected_lineage(value: Any, label: str) -> None:
    """Reject H0/runtime paths and W5 text before any runtime selection occurs."""
    require(not _toolchain_call(toolchain.has_forbidden_w5, value), f"{label} attempts to revive rejected W5/W5_S10")
    strings = tuple(item.strip().lower() for item in _iter_strings(value))
    require("runtime_s_curve" not in strings, f"{label} contains runtime_s_curve; formal H1/L1 replay is required")
    require(
        not any(item in {"h0", "h0b", "h0s"} or item.startswith("sim-dev-h0") for item in strings),
        f"{label} contains a development H0 identity",
    )


def reject_legacy_default_bslosh(value: Any, label: str, *, reject_any_string: bool = False) -> None:
    """Do not silently select the old proxy variants as formal Bslosh.

    JSON policy documents may legitimately *mention* a rejected legacy name in
    a deny-list.  Only selector fields constitute a selection.  Command argv
    has no such structure, so callers validating commands use
    ``reject_any_string=True``.
    """
    if isinstance(value, Mapping):
        for key, item in value.items():
            normalized_key = str(key).strip().lower()
            if normalized_key in LEGACY_VARIANT_SELECTOR_KEYS and isinstance(item, str):
                normalized = item.strip().lower()
                require(
                    normalized not in LEGACY_DEFAULT_BSLOSH_VARIANTS,
                    f"{label} selects legacy default {item!r}; a source-specific frozen Bslosh release is required",
                )
            reject_legacy_default_bslosh(item, label, reject_any_string=reject_any_string)
        return
    if isinstance(value, (list, tuple)):
        for item in value:
            reject_legacy_default_bslosh(item, label, reject_any_string=reject_any_string)
        return
    if reject_any_string and isinstance(value, str):
        normalized = value.strip().lower()
        require(
            normalized not in LEGACY_DEFAULT_BSLOSH_VARIANTS,
            f"{label} selects legacy default {value!r}; a source-specific frozen Bslosh release is required",
        )


def _bound_file(owner: Mapping[str, Any], path_key: str, hash_key: str, label: str) -> Path:
    return _toolchain_call(toolchain.validate_bound_file, owner, path_key, hash_key, label).resolve()


def _effective_config_contract(config: Mapping[str, Any], config_path: Path, config_file_hash: str) -> Dict[str, Any]:
    report = _toolchain_call(toolchain.validate_effective_config, config, str(config.get("condition_id")))
    flattened = _toolchain_call(toolchain.flatten, config)
    # Preserve the full canonical configuration, including observer and delay
    # subfields.  A future ACK producer must compare this whole object, not a
    # handful of launch flags.
    return {
        "effective_config_path": str(config_path),
        "effective_config_file_hash": config_file_hash,
        "effective_config_hash": report["effective_config_hash"],
        "required_top_level_fields": list(toolchain.REQUIRED_EFFECTIVE_CONFIG_FIELDS),
        "canonical_field_paths": sorted(flattened),
        "observer_policy_hash": _canonical_hash(dict(config["observer"])),
        "delay_policy_hash": _canonical_hash(dict(config["delay"])),
        "condition_id": config["condition_id"],
    }


@dataclass(frozen=True)
class FormalRuntimeContext:
    """Only immutable artifacts selected by the master/freeze are retained."""

    sim_root: Path
    case_dir: Path
    case_manifest_path: Path
    case_manifest_hash: str
    case_manifest: Mapping[str, Any]
    formal_freeze_path: Path
    formal_freeze_file_hash: str
    formal_freeze_hash: str
    formal_freeze: Mapping[str, Any]
    formal_master_path: Path
    formal_master_file_hash: str
    formal_master_hash: str
    formal_master: Mapping[str, Any]
    row: Mapping[str, Any]
    seed_bundle_path: Path
    seed_bundle: Mapping[str, Any]
    expected_hashes: Mapping[str, str]
    effective_config_contract: Mapping[str, Any]
    launch_contract: Mapping[str, Any]
    backend: FormalRuntimeBackend
    assets: Mapping[str, Mapping[str, str]]


def validate_adapter_command_abi(
    launch_contract: Mapping[str, Any],
    *,
    adapter_path: Optional[Path] = None,
) -> Dict[str, Any]:
    """Require every frozen runtime command to invoke this exact scaffold ABI.

    Asset selection must come from the case manifest and formal freeze through
    the environment.  A launch contract cannot smuggle a condition, H0 path,
    profile, container, or legacy variant through command-line arguments.
    """
    adapter = (adapter_path or Path(__file__)).resolve()
    commands = launch_contract.get("commands")
    require(isinstance(commands, Mapping), "formal launch contract has no normalized commands")
    require(set(commands) == set(ABI_SUBCOMMANDS), "formal launch contract command set differs from the required 8-command ABI")
    normalized: Dict[str, list[str]] = {}
    for field, subcommand in ABI_SUBCOMMANDS.items():
        argv = _toolchain_call(toolchain.command_from_spec, commands.get(field), field)
        adapter_indices = []
        for index, token in enumerate(argv):
            try:
                if Path(token).is_absolute() and Path(token).resolve() == adapter:
                    adapter_indices.append(index)
            except OSError:
                continue
        require(len(adapter_indices) == 1, f"{field} must hash-bind and invoke this formal adapter exactly once")
        adapter_index = adapter_indices[0]
        require(adapter_index + 1 < len(argv) and argv[adapter_index + 1] == subcommand, f"{field} must invoke adapter subcommand {subcommand!r}")
        # The scaffold accepts no command-line asset selectors.  It is okay to
        # have a Python interpreter before the adapter script; everything after
        # the subcommand would create a second, mutable selection channel.
        require(adapter_index + 2 == len(argv), f"{field} must not carry mutable command-line arguments")
        for token in argv:
            lowered = token.lower()
            require(
                not any(lowered == prefix or lowered.startswith(prefix + "=") for prefix in FORBIDDEN_ADAPTER_ARGUMENT_PREFIXES),
                f"{field} contains a forbidden runtime asset selector: {token}",
            )
        reject_development_or_rejected_lineage(argv, field)
        reject_legacy_default_bslosh(argv, field, reject_any_string=True)
        normalized[field] = argv
    return {"status": "PASS", "adapter_path": str(adapter), "commands": normalized}


def _safe_case_local_filename(value: Any, label: str, *, suffix: str) -> str:
    """Accept an immutable filename, never an artifact path or a template."""
    require(isinstance(value, str) and value, f"formal backend {label} filename is missing")
    candidate = Path(value)
    require(
        not candidate.is_absolute()
        and candidate.name == value
        and value not in {".", ".."}
        and ".." not in candidate.parts,
        f"formal backend {label} must be a case-local filename, not a path",
    )
    require(value.endswith(suffix), f"formal backend {label} must end with {suffix!r}")
    return value


def _validate_backend_delegate_command(
    commands: Mapping[str, Any],
    command_file_hashes: Mapping[str, Any],
    field: str,
) -> tuple[str, ...]:
    """Resolve one frozen delegate without allowing a mutable launch channel."""
    command = _toolchain_call(toolchain.command_from_spec, commands.get(field), f"formal backend {field}")
    require(Path(command[0]).is_absolute() and Path(command[0]).is_file(), f"formal backend {field} executable must be an absolute file")
    require(os.access(command[0], os.X_OK), f"formal backend {field} executable is not executable")
    refs_raw = command_file_hashes.get(field)
    require(isinstance(refs_raw, list) and refs_raw, f"formal backend {field} command_file_hashes is missing")
    refs: set[str] = set()
    for index, item in enumerate(refs_raw):
        require(isinstance(item, Mapping), f"formal backend {field} command file hash {index} must be an object")
        path_value, expected_hash = item.get("path"), item.get("sha256")
        require(
            isinstance(path_value, str) and Path(path_value).is_absolute() and Path(path_value).is_file(),
            f"formal backend {field} command hash path is invalid",
        )
        require(_is_sha256(expected_hash), f"formal backend {field} command hash is invalid")
        path = Path(path_value).resolve()
        require(_sha256_file(path) == expected_hash, f"formal backend {field} command file hash mismatch: {path}")
        require(str(path) not in refs, f"formal backend {field} repeats a command hash path")
        refs.add(str(path))
    require(str(Path(command[0]).resolve()) in refs, f"formal backend {field} executable is not hash-bound")
    adapter = Path(__file__).resolve()
    h0_adapter = Path(__file__).with_name("smpcc_sim_h0_runtime_adapter.py").resolve()
    frozen_path_adapter = Path(__file__).with_name("smpcc_sim_frozen_path_replay.py").resolve()
    for token in command:
        path = Path(token)
        if path.is_absolute():
            require(path.is_file(), f"formal backend {field} contains a non-file absolute argument")
            resolved = path.resolve()
            require(str(resolved) in refs, f"formal backend {field} absolute file argument is not hash-bound")
            require(
                resolved not in {adapter, h0_adapter, frozen_path_adapter},
                f"formal backend {field} cannot delegate to this adapter, H0 adapter, or frozen-path publisher",
            )
    for token in command:
        lowered = token.lower()
        require(
            not any(lowered == prefix or lowered.startswith(prefix + "=") for prefix in FORBIDDEN_ADAPTER_ARGUMENT_PREFIXES),
            f"formal backend {field} contains a mutable runtime asset selector: {token}",
        )
    reject_development_or_rejected_lineage(command, f"formal backend {field}")
    reject_legacy_default_bslosh(command, f"formal backend {field}", reject_any_string=True)
    return tuple(command)


@dataclass(frozen=True)
class FormalRuntimeBackend:
    """Reviewed delegate release selected only from the immutable freeze."""

    manifest_path: Path
    manifest_file_hash: str
    backend_id: str
    backend_hash: str
    commands: Mapping[str, tuple[str, ...]]
    artifacts: Mapping[str, str]
    lifecycle: Mapping[str, Any]


def validate_formal_runtime_backend(
    freeze: Mapping[str, Any],
    launch_contract: Mapping[str, Any],
) -> FormalRuntimeBackend:
    """Require a real, reviewable backend release before an ABI command may exec.

    The launch contract binds the *adapter* ABI.  This second manifest binds
    the concrete processes behind that ABI.  Keeping those two layers
    separate prevents the formal row runner from accepting an old shell
    wrapper merely because it has eight command-shaped strings.
    """
    # Keep formal-gate and command-time validation on the same shared schema.
    # The adapter then adds its own exact-delegate protections below.
    _toolchain_call(toolchain.validate_formal_runtime_backend_manifest, freeze.get("formal_runtime_backend"), freeze, launch_contract)
    entry = freeze.get("formal_runtime_backend")
    require(isinstance(entry, Mapping), "formal freeze lacks formal_runtime_backend declaration")
    manifest_path = _bound_file(entry, BACKEND_ENTRY_PATH_KEY, BACKEND_ENTRY_HASH_KEY, "formal runtime backend")
    document = _read_json(manifest_path, "formal runtime backend manifest")
    require(document.get("document_type") == BACKEND_DOCUMENT_TYPE, "formal runtime backend has wrong document_type")
    require(document.get("status") == BACKEND_STATUS, "formal runtime backend is not FROZEN")
    require(document.get("protocol_id") == toolchain.FORMAL_PROTOCOL_ID, "formal runtime backend protocol_id mismatch")
    require(document.get("formal") is True, "formal runtime backend must set formal=true")
    require(document.get("development_only") is False, "formal runtime backend must set development_only=false")
    require(document.get("runtime_backend_implemented") is True, "formal runtime backend is not marked implemented")
    require(document.get("delegate_via_execve") is True, "formal runtime backend must require direct execve delegation")
    require(document.get("legacy_wrappers_forbidden") is True, "formal runtime backend must explicitly forbid legacy wrappers")
    require(isinstance(document.get("backend_id"), str) and document["backend_id"], "formal runtime backend_id is missing")
    core = dict(document)
    declared_backend_hash = core.pop("backend_hash", None)
    require(declared_backend_hash == _canonical_hash(core), "formal runtime backend_hash mismatch")
    require(entry.get("backend_id") == document["backend_id"], "formal runtime backend declaration ID mismatch")
    require(entry.get("backend_hash") == declared_backend_hash, "formal runtime backend declaration hash mismatch")
    for key in ("sim_freeze_id", "git_revision", "build_id"):
        require(document.get(key) == freeze.get(key), f"formal runtime backend {key} differs from formal freeze")
    require(
        document.get("runtime_launch_contract_id") == launch_contract.get("contract_id")
        and document.get("runtime_launch_contract_hash") == launch_contract.get("contract_hash"),
        "formal runtime backend is not bound to the frozen 8-command launch contract",
    )
    replay = document.get("frozen_path_replay")
    require(isinstance(replay, Mapping), "formal runtime backend lacks frozen_path_replay binding")
    require(replay.get("source_mode") == "frozen_json_replay", "formal runtime backend must use frozen_json_replay")
    require(replay.get("runtime_generation_forbidden") is True, "formal runtime backend must forbid runtime path generation")
    replay_path = _bound_file(replay, "entrypoint_path", "entrypoint_hash", "formal frozen-path replay entrypoint")
    expected_replay_path = Path(__file__).with_name("smpcc_sim_frozen_path_replay.py").resolve()
    require(replay_path == expected_replay_path, "formal runtime backend must bind the reviewed frozen-path replay entrypoint")
    config_readback = document.get("effective_config_readback")
    require(isinstance(config_readback, Mapping), "formal runtime backend lacks effective_config_readback contract")
    require(config_readback.get("required") is True, "formal runtime backend must require effective-config readback")
    consumed_fields = config_readback.get("consumed_fields")
    require(
        isinstance(consumed_fields, list)
        and len(consumed_fields) == len(set(consumed_fields))
        and set(consumed_fields) == set(toolchain.REQUIRED_EFFECTIVE_CONFIG_FIELDS),
        "formal runtime backend must consume every frozen effective-config field",
    )
    require(
        config_readback.get("runtime_ack_schema_hash") == launch_contract.get("runtime_ack_schema_hash"),
        "formal runtime backend effective-config readback schema differs from the launch contract",
    )
    goal_policy = document.get("goal_probe_policy")
    require(isinstance(goal_policy, Mapping), "formal runtime backend lacks goal_probe_policy")
    require(goal_policy.get("exact_terminal_status") == "GOAL_REACHED", "formal runtime backend must normalize terminal state to exact GOAL_REACHED")
    require(goal_policy.get("after_motion_release_required") is True, "formal goal probe must reject pre-motion latched terminal status")
    require(
        goal_policy.get("goal_reached_rule_hash") == launch_contract.get("goal_reached_rule_hash"),
        "formal runtime backend goal rule differs from the launch contract",
    )
    stop_policy = document.get("motion_stop_policy")
    require(isinstance(stop_policy, Mapping), "formal runtime backend lacks motion_stop_policy")
    require(stop_policy.get("dedicated_cmd_gate") is True and stop_policy.get("zero_hold_required") is True, "formal runtime backend must use a dedicated zero-hold command gate")
    require(
        stop_policy.get("motion_stop_ack_schema_hash") == launch_contract.get("motion_stop_ack_schema_hash"),
        "formal runtime backend motion-stop ACK schema differs from the launch contract",
    )

    lifecycle = document.get("lifecycle")
    require(isinstance(lifecycle, Mapping), "formal runtime backend lifecycle is missing")
    recording_policy = freeze.get("recording_policy")
    require(isinstance(recording_policy, Mapping), "formal runtime backend requires frozen recording_policy")
    exact_lifecycle = {
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
    }
    for key, expected in exact_lifecycle.items():
        require(lifecycle.get(key) == expected, f"formal runtime backend lifecycle {key} must be {expected!r}")
    require(lifecycle.get("settle_sec") == 30.0 == recording_policy.get("settle_sec"), "formal runtime backend settle must be exactly 30 seconds")
    require(
        lifecycle.get("effective_motion_window_sec") == 60.0 == recording_policy.get("goal_timeout_sec"),
        "formal runtime backend effective-motion window must be exactly 60 seconds",
    )
    require(lifecycle.get("tail_sec") == recording_policy.get("tail_sec"), "formal runtime backend tail differs from frozen recording policy")
    checkpoints = lifecycle.get("controller_firewall_checkpoints")
    require(
        isinstance(checkpoints, list) and checkpoints == ["ready", "pre_motion", "postflight"],
        "formal runtime backend must require ready/pre_motion/postflight firewall checkpoints",
    )

    environment_keys = document.get("required_environment")
    require(
        isinstance(environment_keys, list)
        and len(environment_keys) == len(set(environment_keys))
        and set(environment_keys) == set(BACKEND_REQUIRED_ENVIRONMENT),
        "formal runtime backend required_environment differs from the immutable ABI environment",
    )
    commands = document.get("commands")
    command_file_hashes = document.get("command_file_hashes")
    require(isinstance(commands, Mapping) and set(commands) == set(ABI_SUBCOMMANDS), "formal runtime backend must provide exactly the 8 delegate commands")
    require(
        isinstance(command_file_hashes, Mapping) and set(command_file_hashes) == set(ABI_SUBCOMMANDS),
        "formal runtime backend must hash exactly the 8 delegate commands",
    )
    normalized_commands = {
        field: _validate_backend_delegate_command(commands, command_file_hashes, field)
        for field in ABI_SUBCOMMANDS
    }
    artifacts = document.get("case_artifacts")
    require(isinstance(artifacts, Mapping) and set(artifacts) == set(BACKEND_ARTIFACT_KEYS), "formal runtime backend must freeze exactly its four case artifacts")
    normalized_artifacts = {
        "recorder_artifact": _safe_case_local_filename(artifacts.get("recorder_artifact"), "recorder_artifact", suffix=".bag.active"),
        "runtime_ack": _safe_case_local_filename(artifacts.get("runtime_ack"), "runtime_ack", suffix=".json"),
        "motion_release_ack": _safe_case_local_filename(artifacts.get("motion_release_ack"), "motion_release_ack", suffix=".json"),
        "motion_stop_ack": _safe_case_local_filename(artifacts.get("motion_stop_ack"), "motion_stop_ack", suffix=".json"),
    }
    require(len(set(normalized_artifacts.values())) == len(normalized_artifacts), "formal runtime backend case artifacts must have distinct names")
    return FormalRuntimeBackend(
        manifest_path=manifest_path,
        manifest_file_hash=_sha256_file(manifest_path),
        backend_id=str(document["backend_id"]),
        backend_hash=str(declared_backend_hash),
        commands=normalized_commands,
        artifacts=normalized_artifacts,
        lifecycle=dict(lifecycle),
    )


def _select_row(master: Mapping[str, Any], case_manifest: Mapping[str, Any]) -> Mapping[str, Any]:
    planned_row_id = case_manifest.get("planned_row_id")
    require(isinstance(planned_row_id, str) and planned_row_id, "case manifest lacks planned_row_id")
    rows = master.get("planned_rows")
    require(isinstance(rows, list), "formal master lacks planned_rows")
    matching = [item for item in rows if isinstance(item, Mapping) and item.get("planned_row_id") == planned_row_id]
    require(len(matching) == 1, "case manifest does not bind exactly one formal planned row")
    return matching[0]


def _validate_row_identity(row: Mapping[str, Any], case_manifest: Mapping[str, Any]) -> None:
    require(row.get("formal") is True, "adapter refuses non-formal planned rows")
    require(row.get("evidence_class") == "FORMAL_PLANNED_ROWS_NOT_EXECUTED", "row lacks formal planned-row evidence class")
    stage = row.get("stage")
    require(stage in toolchain.STAGES, f"row has unknown formal stage: {stage!r}")
    expected = toolchain.STAGES[str(stage)]
    require(row.get("path_id") == expected["path_id"] and row.get("container_id") == expected["container_id"], "row path/container disagrees with its formal stage")
    require(row.get("condition_id") in expected["conditions"], "row condition is not registered for its formal stage")
    require(row.get("method_backend") == toolchain.CONDITION_BACKENDS.get(row.get("condition_id")), "row method backend is inconsistent")
    require(row.get("path_id") in {"H1", "L1"}, "formal adapter refuses H0/H0b/H0s rows")
    require(row.get("container_id") in {"C1", "C2"}, "formal adapter refuses non-C1/C2 containers")
    require(not str(row.get("planned_row_id", "")).startswith("SIM-DEV-"), "formal adapter refuses development planned rows")
    reject_development_or_rejected_lineage(row, "formal planned row")
    reject_development_or_rejected_lineage(case_manifest, "case manifest")


def _validate_case_location(
    sim_root: Path,
    case_manifest_path: Path,
    case_manifest: Mapping[str, Any],
    row: Mapping[str, Any],
) -> Path:
    case_dir = _ensure_below(case_manifest_path.parent, sim_root, "case directory")
    require(case_manifest_path.parent.resolve() == case_dir, "case manifest must be directly inside its case directory")
    attempt_id = case_manifest.get("attempt_id")
    require(isinstance(attempt_id, str) and attempt_id, "case manifest lacks attempt_id")
    attempt_number = _toolchain_call(toolchain.parse_attempt_number, attempt_id)
    require(
        _toolchain_call(toolchain.attempt_prefix, attempt_id) == row.get("planned_row_id"),
        "case manifest attempt ID is not derived from its formal row",
    )
    dataset_root = _absolute_existing_directory(case_manifest.get("dataset_root"), "case manifest dataset_root")
    expected = dataset_root / str(row["stage"]) / str(row["block_id"]) / f"p{int(row['order_position']):02d}_{row['condition_id']}" / f"r{attempt_number:02d}"
    require(case_dir == expected.resolve(), "case directory does not match immutable row/attempt layout")
    return case_dir


def _absolute_existing_directory(value: Any, label: str) -> Path:
    require(isinstance(value, str) and value, f"{label} is missing")
    path = Path(value)
    require(path.is_absolute(), f"{label} must be absolute")
    require(path.is_dir(), f"{label} does not exist: {path}")
    return path.resolve()


def _validate_seed_binding(
    environment: Mapping[str, str],
    case_manifest: Mapping[str, Any],
    row: Mapping[str, Any],
    case_dir: Path,
) -> tuple[Path, Mapping[str, Any]]:
    seed_path = _absolute_existing_file(environment.get("SMPCC_SEED_BUNDLE_PATH"), "formal seed bundle")
    require(seed_path.parent == case_dir, "formal seed bundle must be case-local")
    expected_seed_hash = environment.get("SMPCC_SEED_BUNDLE_SHA256")
    require(_is_sha256(expected_seed_hash), "formal seed bundle hash is missing")
    require(case_manifest.get("seed_bundle_path") == str(seed_path), "case manifest seed path differs from adapter environment")
    require(case_manifest.get("seed_bundle_hash") == expected_seed_hash, "case manifest seed hash differs from adapter environment")
    seed_bundle = _read_json(seed_path, "formal seed bundle")
    seed_report = _toolchain_call(toolchain.validate_seed_bundle, seed_bundle)
    require(seed_report["seed_bundle_hash"] == expected_seed_hash, "formal seed bundle canonical hash mismatch")
    require(row.get("seed_bundle_id") == seed_bundle.get("seed_bundle_id"), "row seed bundle ID mismatch")
    require(row.get("seed_bundle_hash") == expected_seed_hash, "row seed bundle hash mismatch")
    return seed_path, seed_bundle


def _validate_formal_assets(
    freeze: Mapping[str, Any],
    row: Mapping[str, Any],
    case_manifest: Optional[Mapping[str, Any]],
) -> tuple[Mapping[str, str], Mapping[str, Any], Mapping[str, Mapping[str, str]]]:
    """Resolve every asset from the freeze and bind it to the case manifest."""
    simulator = freeze.get("simulator_assets")
    require(isinstance(simulator, Mapping), "formal freeze lacks simulator_assets")
    simulator_report = _toolchain_call(toolchain.validate_formal_simulator_assets, simulator)
    map_path = Path(str(simulator_report["map_path"]))
    world_path = Path(str(simulator_report["world_path"]))
    world_geometry_path = Path(str(simulator_report["world_geometry_path"]))
    robot_path = Path(str(simulator_report["robot_model_path"]))
    world = simulator_report["world_geometry"]

    paths = freeze.get("paths")
    require(isinstance(paths, Mapping), "formal freeze lacks path registry")
    path_entry = paths.get(row["path_id"])
    require(isinstance(path_entry, Mapping), f"formal freeze lacks path {row['path_id']}")
    require(path_entry.get("source_mode") == "frozen_json_replay", "formal adapter requires frozen_json_replay paths")
    reject_development_or_rejected_lineage(path_entry, "formal path entry")
    source_path = _bound_file(path_entry, "source_path", "source_path_hash", "formal source path")
    sim_path = _bound_file(path_entry, "sim_path", "sim_path_hash", "formal replay path")
    transform_path = _bound_file(path_entry, "transform_path", "transform_hash", "formal path transform")
    fit_path = _bound_file(path_entry, "fit_clearance_report_path", "fit_clearance_report_hash", "formal path fit/clearance report")
    source_document = _read_json(source_path, "formal source path")
    sim_document = _read_json(sim_path, "formal replay path")
    require(source_document.get("path_id") == row["path_id"], "frozen source JSON path_id differs from planned row")
    require(sim_document.get("path_id") == row["path_id"], "frozen replay JSON path_id differs from planned row")
    require(source_document.get("source_mode") == "frozen_json_replay", "formal source JSON is not frozen_json_replay")
    reject_development_or_rejected_lineage(source_document, "formal source JSON")
    reject_development_or_rejected_lineage(sim_document, "formal replay JSON")
    transform = _read_json(transform_path, "formal path transform")
    _toolchain_call(
        toolchain.validate_path_replay,
        source_path,
        sim_path,
        transform,
        world,
        float(path_entry.get("clearance_m")),
    )

    containers = freeze.get("containers")
    require(isinstance(containers, Mapping), "formal freeze lacks container registry")
    container = containers.get(row["container_id"])
    require(isinstance(container, Mapping), f"formal freeze lacks container {row['container_id']}")
    _toolchain_call(toolchain.validate_formal_container_entry, container, str(row["container_id"]))
    parameter_path = _bound_file(container, "physical_parameter_file", "physical_parameter_hash", "formal container parameters")
    container_manifest_path = _bound_file(container, "container_manifest_path", "container_manifest_hash", "formal container manifest")

    configs = freeze.get("effective_configs")
    require(isinstance(configs, Mapping), "formal freeze lacks effective_configs")
    config_entry = configs.get(row["condition_id"])
    require(isinstance(config_entry, Mapping), f"formal freeze lacks condition config {row['condition_id']}")
    config_path = _bound_file(config_entry, "effective_config_path", "effective_config_file_hash", "formal effective config")
    config = _read_json(config_path, "formal effective config")
    require(config_entry.get("effective_config") == config, "formal config embedded/file content mismatch")
    config_report = _toolchain_call(toolchain.validate_effective_config, config, str(row["condition_id"]))
    require(config_entry.get("effective_config_hash") == config_report["effective_config_hash"], "formal config canonical hash mismatch")
    reject_development_or_rejected_lineage(config, "formal effective config")
    reject_legacy_default_bslosh(config, "formal effective config")
    config_contract = _effective_config_contract(config, config_path, _sha256_file(config_path))

    assets: Dict[str, Mapping[str, str]] = {
        "map": {"path": str(map_path), "sha256": str(simulator["map_hash"])},
        # ``world`` is the actual Gazebo SDF/XML artifact.  Clearance geometry
        # is separately bound below and cannot be renamed as the launched world.
        "world": {"path": str(world_path), "sha256": str(simulator["world_hash"])},
        "world_geometry": {"path": str(world_geometry_path), "sha256": str(simulator["world_geometry_hash"])},
        "robot": {"path": str(robot_path), "sha256": str(simulator["robot_model_hash"])},
        "source_path": {"path": str(source_path), "sha256": str(path_entry["source_path_hash"])},
        "sim_path": {"path": str(sim_path), "sha256": str(path_entry["sim_path_hash"])},
        "transform": {"path": str(transform_path), "sha256": str(path_entry["transform_hash"])},
        "fit_clearance_report": {"path": str(fit_path), "sha256": str(path_entry["fit_clearance_report_hash"])},
        "container_parameters": {"path": str(parameter_path), "sha256": str(container["physical_parameter_hash"])},
        "container_manifest": {"path": str(container_manifest_path), "sha256": str(container["container_manifest_hash"])},
        "effective_config": {"path": str(config_path), "sha256": str(config_entry["effective_config_file_hash"])},
    }

    if row["condition_id"] == "FixedProfile":
        stratum = f"{row['path_id']}_{row['container_id']}"
        profiles = freeze.get("fixed_profiles")
        require(isinstance(profiles, Mapping) and isinstance(profiles.get(stratum), Mapping), f"formal freeze lacks FixedProfile {stratum}")
        profile = profiles[stratum]
        _toolchain_call(toolchain.validate_fixed_profile, profile, True)
        profile_path = _absolute_existing_file(profile.get("profile_path"), f"FixedProfile {stratum}")
        assets["fixed_profile"] = {"path": str(profile_path), "sha256": str(profile["profile_hash"])}
        # Explicitly refuse generated/selected profiles even if a caller adds
        # tempting metadata beside an otherwise valid registry entry.
        reject_development_or_rejected_lineage(profile, "FixedProfile registry")
        reject_legacy_default_bslosh(profile, "FixedProfile registry")

    if row["condition_id"] == "Bslosh":
        _toolchain_call(toolchain.validate_formal_bslosh_release, freeze.get("formal_bslosh_release"), freeze, config)

    capability = freeze.get("liquid_plant_capability")
    truth = _toolchain_call(toolchain.validate_formal_liquid_plant_capability, capability)
    require(truth.get("eligible") is True, "formal adapter requires independent liquid plant capability")
    require(isinstance(capability, Mapping), "formal liquid plant capability is missing")
    for file_key, hash_key, asset_key in (
        ("plant_code_path", "plant_code_hash", "plant_code"),
        ("plant_parameter_path", "plant_parameter_hash", "plant_parameters"),
        ("plant_input_schema_path", "plant_input_schema_hash", "plant_input_schema"),
        ("plant_output_schema_path", "plant_output_schema_hash", "plant_output_schema"),
        ("fidelity_report_path", "fidelity_report_hash", "plant_fidelity_report"),
    ):
        path = _bound_file(capability, file_key, hash_key, "formal liquid plant asset")
        assets[asset_key] = {"path": str(path), "sha256": str(capability[hash_key])}

    expected_hashes = _toolchain_call(
        toolchain.expected_row_frozen_asset_hashes,
        freeze,
        str(row["path_id"]),
        str(row["container_id"]),
        str(row["condition_id"]),
    )
    require(row.get("frozen_asset_hashes") == expected_hashes, "planned row frozen asset hashes differ from freeze")
    if case_manifest is not None:
        manifest_hashes = case_manifest.get("hashes")
        require(isinstance(manifest_hashes, Mapping), "case manifest lacks runtime input hashes")
        for key, expected_hash in expected_hashes.items():
            if key in {"source_path_hash", "transform_hash"}:
                continue
            actual_key = "path_hash" if key == "sim_path_hash" else key
            require(manifest_hashes.get(actual_key) == expected_hash, f"case manifest {actual_key} differs from frozen row asset")
        require(manifest_hashes.get("effective_config_hash") == config_contract["effective_config_hash"], "case manifest config hash differs from bound config")
        require(manifest_hashes.get("observer_policy_hash") == config_contract["observer_policy_hash"], "case manifest observer policy hash differs from bound config")
        require(manifest_hashes.get("delay_policy_hash") == config_contract["delay_policy_hash"], "case manifest delay policy hash differs from bound config")
    return expected_hashes, config_contract, assets


def preflight_case(
    *,
    environment: Optional[Mapping[str, str]] = None,
    sim_root: Path = DEFAULT_SIM_ROOT,
) -> FormalRuntimeContext:
    """Offline, fail-closed proof that one case can consume only frozen inputs.

    A return value is intentionally not an execution authorization.  It means
    only that the static assets and the 8-command ABI are internally bound.
    The active runtime commands remain NO-GO until an independent plant and a
    real launch backend are implemented and reviewed.
    """
    env: Mapping[str, str] = os.environ if environment is None else environment
    missing = [key for key in REQUIRED_ENVIRONMENT if not env.get(key)]
    require(not missing, "formal adapter missing required environment: " + ", ".join(missing))
    root = Path(sim_root).resolve()
    require(root.is_dir(), f"simulation root does not exist: {root}")

    try:
        case_manifest_path, case_manifest, case_manifest_hash = _bound_json_from_environment(
            env,
            "SMPCC_CASE_LAUNCH_MANIFEST_PATH",
            "SMPCC_CASE_LAUNCH_MANIFEST_SHA256",
            "formal case launch manifest",
        )
        case_manifest_path = _ensure_below(case_manifest_path, root, "formal case launch manifest")
        require(case_manifest.get("formal") is True, "formal adapter refuses non-formal case manifests")

        freeze_path, freeze, freeze_file_hash = _bound_json_from_environment(
            env,
            "SMPCC_FORMAL_FREEZE_PATH",
            "SMPCC_FORMAL_FREEZE_FILE_SHA256",
            "formal freeze",
        )
        master_path, master, master_file_hash = _bound_json_from_environment(
            env,
            "SMPCC_FORMAL_MASTER_PATH",
            "SMPCC_FORMAL_MASTER_FILE_SHA256",
            "formal master",
        )
        reject_development_or_rejected_lineage(freeze, "formal freeze")
        reject_development_or_rejected_lineage(master, "formal master")

        freeze_report = _toolchain_call(toolchain.validate_formal_freeze, freeze)
        require(freeze_report.get("status") == "PASS", "FORMAL_SIM_NO_GO: " + "; ".join(freeze_report.get("errors", [])))
        master_report = _toolchain_call(toolchain.validate_master, master, require_formal=True)
        require(master_report.get("status") == "PASS", "formal master validation failed: " + "; ".join(master_report.get("errors", [])))
        try:
            source_separation.require_execution_identity(freeze, master)
        except source_separation.SourceSeparationError as exc:
            raise AdapterError(str(exc)) from exc
        formal_freeze_hash = _canonical_hash(freeze)
        formal_master_hash = master.get("master_hash")
        require(_is_sha256(formal_master_hash), "formal master has invalid master_hash")
        require(case_manifest.get("formal_freeze_hash") == formal_freeze_hash, "case manifest formal freeze hash mismatch")
        require(case_manifest.get("formal_master_hash") == formal_master_hash, "case manifest formal master hash mismatch")
        require(master.get("freeze_hash") == formal_freeze_hash, "formal master freeze hash mismatch")
        require(master.get("formal_freeze_path") == str(freeze_path), "formal master is bound to a different formal freeze path")
        require(master.get("formal_freeze_file_hash") == freeze_file_hash, "formal master formal freeze file hash mismatch")

        row = _select_row(master, case_manifest)
        _validate_row_identity(row, case_manifest)
        case_dir = _validate_case_location(root, case_manifest_path, case_manifest, row)
        seed_path, seed_bundle = _validate_seed_binding(env, case_manifest, row, case_dir)

        ledger = freeze.get("dataset_ledger")
        require(isinstance(ledger, Mapping), "formal freeze lacks dataset ledger")
        require(case_manifest.get("dataset_root") == ledger.get("ledger_root"), "case manifest dataset root differs from frozen ledger")
        require(case_manifest.get("dataset_ledger_id") == ledger.get("ledger_id"), "case manifest ledger ID differs from frozen ledger")
        require(case_manifest.get("dataset_ledger_identity_hash") == ledger.get("ledger_identity_hash"), "case manifest ledger identity differs from frozen ledger")

        launch_contract = _toolchain_call(toolchain.validate_frozen_runtime_launch_contract, freeze.get("runtime_launch_contract"))
        require(case_manifest.get("runtime_launch_contract_id") == launch_contract.get("contract_id"), "case manifest runtime launch contract ID mismatch")
        require(case_manifest.get("runtime_launch_contract_hash") == launch_contract.get("contract_hash"), "case manifest runtime launch contract hash mismatch")
        validate_adapter_command_abi(launch_contract)
        backend = validate_formal_runtime_backend(freeze, launch_contract)

        expected_hashes, config_contract, assets = _validate_formal_assets(freeze, row, case_manifest)
        return FormalRuntimeContext(
            sim_root=root,
            case_dir=case_dir,
            case_manifest_path=case_manifest_path,
            case_manifest_hash=case_manifest_hash,
            case_manifest=case_manifest,
            formal_freeze_path=freeze_path,
            formal_freeze_file_hash=freeze_file_hash,
            formal_freeze_hash=formal_freeze_hash,
            formal_freeze=freeze,
            formal_master_path=master_path,
            formal_master_file_hash=master_file_hash,
            formal_master_hash=str(formal_master_hash),
            formal_master=master,
            row=row,
            seed_bundle_path=seed_path,
            seed_bundle=seed_bundle,
            expected_hashes=expected_hashes,
            effective_config_contract=config_contract,
            launch_contract=launch_contract,
            backend=backend,
            assets=assets,
        )
    except AdapterError:
        raise
    except Exception as exc:  # defensive conversion to a clear, fail-closed ABI result
        raise AdapterError(f"formal adapter preflight failed: {exc!r}") from exc


@dataclass(frozen=True)
class FormalRowPreparation:
    """A no-write formal-row launch request, fully derived from frozen inputs."""

    sim_root: Path
    output_root: Path
    formal_freeze_path: Path
    formal_freeze_file_hash: str
    formal_freeze_hash: str
    formal_freeze: Mapping[str, Any]
    formal_master_path: Path
    formal_master_file_hash: str
    formal_master_hash: str
    formal_master: Mapping[str, Any]
    row: Mapping[str, Any]
    attempt_id: str
    seed_bundle: Mapping[str, Any]
    expected_hashes: Mapping[str, str]
    effective_config_contract: Mapping[str, Any]
    launch_contract: Mapping[str, Any]
    backend: FormalRuntimeBackend
    assets: Mapping[str, Mapping[str, str]]
    stage_entry_evidence: Optional[Mapping[str, Any]]
    retry_authorization: Optional[str]


def _formal_row_case_dir(output_root: Path, row: Mapping[str, Any], attempt_id: str) -> Path:
    attempt_number = _toolchain_call(toolchain.parse_attempt_number, attempt_id)
    require(
        _toolchain_call(toolchain.attempt_prefix, attempt_id) == row.get("planned_row_id"),
        "formal attempt ID is not derived from the planned row",
    )
    return (
        output_root
        / str(row["stage"])
        / str(row["block_id"])
        / f"p{int(row['order_position']):02d}_{row['condition_id']}"
        / f"r{attempt_number:02d}"
    ).resolve()


def prepare_formal_row(
    *,
    formal_freeze_path: Path,
    formal_master_path: Path,
    planned_row_id: str,
    output_root: Path,
    attempt_id: Optional[str] = None,
    stage_entry_evidence: Optional[Mapping[str, Any]] = None,
    retry_authorization: Optional[str] = None,
    sim_root: Path = DEFAULT_SIM_ROOT,
) -> FormalRowPreparation:
    """Build an actual formal-row request without creating files or processes.

    This is the formal outer entry point.  Unlike an ad-hoc ``toolchain run``
    spec, every executable, asset, ACK path and timing value is derived from
    the master/freeze/backend manifest.  It deliberately rejects a different
    output root, a pre-existing attempt directory, or a non-default simulator
    root before the generic runner can create a case directory.
    """
    try:
        root = Path(sim_root).resolve()
        require(root == DEFAULT_SIM_ROOT.resolve(), "formal runtime sim_root must be the isolated formal simulation root")
        require(root.is_dir(), f"formal simulation root does not exist: {root}")
        freeze_path = _absolute_existing_file(str(formal_freeze_path), "formal freeze")
        master_path = _absolute_existing_file(str(formal_master_path), "formal master")
        freeze = _read_json(freeze_path, "formal freeze")
        master = _read_json(master_path, "formal master")
        reject_development_or_rejected_lineage(freeze, "formal freeze")
        reject_development_or_rejected_lineage(master, "formal master")
        freeze_report = _toolchain_call(toolchain.validate_formal_freeze, freeze)
        require(freeze_report.get("status") == "PASS", "FORMAL_SIM_NO_GO: " + "; ".join(freeze_report.get("errors", [])))
        master_report = _toolchain_call(toolchain.validate_master, master, require_formal=True)
        require(master_report.get("status") == "PASS", "formal master validation failed: " + "; ".join(master_report.get("errors", [])))
        try:
            source_separation.require_execution_identity(freeze, master)
        except source_separation.SourceSeparationError as exc:
            raise AdapterError(str(exc)) from exc
        freeze_file_hash = _sha256_file(freeze_path)
        freeze_hash = _canonical_hash(freeze)
        master_file_hash = _sha256_file(master_path)
        master_hash = master.get("master_hash")
        require(_is_sha256(master_hash), "formal master has invalid master_hash")
        require(master.get("freeze_hash") == freeze_hash, "formal master freeze hash mismatch")
        require(master.get("formal_freeze_path") == str(freeze_path), "formal master is bound to a different formal freeze path")
        require(master.get("formal_freeze_file_hash") == freeze_file_hash, "formal master formal freeze file hash mismatch")
        require(isinstance(planned_row_id, str) and planned_row_id, "formal planned_row_id is missing")
        row = _select_row(master, {"planned_row_id": planned_row_id})
        _validate_row_identity(row, {})
        launch_contract = _toolchain_call(toolchain.validate_frozen_runtime_launch_contract, freeze.get("runtime_launch_contract"))
        validate_adapter_command_abi(launch_contract)
        backend = validate_formal_runtime_backend(freeze, launch_contract)
        seed_bundles = master.get("seed_bundles")
        require(isinstance(seed_bundles, Mapping), "formal master lacks frozen seed bundles")
        seed_key = f"{row['stage']}:{row['block_id']}"
        seed_bundle = seed_bundles.get(seed_key)
        seed_report = _toolchain_call(toolchain.validate_seed_bundle, seed_bundle)
        require(
            seed_bundle.get("seed_bundle_id") == row.get("seed_bundle_id")
            and seed_report.get("seed_bundle_hash") == row.get("seed_bundle_hash"),
            "formal row seed bundle differs from the frozen master",
        )
        expected_hashes, config_contract, assets = _validate_formal_assets(freeze, row, None)
        ledger = _toolchain_call(toolchain.validate_formal_dataset_ledger, freeze.get("dataset_ledger"), freeze)
        destination = Path(output_root).resolve()
        require(destination == Path(str(ledger["ledger_root"])).resolve(), "formal output_root differs from frozen dataset ledger root")
        _ensure_below(destination, root, "formal output root")
        actual_attempt_id = attempt_id or f"{row['planned_row_id']}_r01"
        require(isinstance(actual_attempt_id, str) and actual_attempt_id, "formal attempt_id is invalid")
        case_dir = _formal_row_case_dir(destination, row, actual_attempt_id)
        _ensure_below(case_dir, destination, "formal case directory")
        require(not case_dir.exists(), f"formal attempt output already exists: {case_dir}")
        if stage_entry_evidence is not None:
            require(isinstance(stage_entry_evidence, Mapping), "formal stage_entry_evidence must be an object")
        _toolchain_call(toolchain.validate_stage_entry, master, row, stage_entry_evidence)
        if retry_authorization is not None:
            retry_path = _absolute_existing_file(retry_authorization, "formal retry authorization")
            retry_authorization = str(retry_path)
        require(
            (actual_attempt_id.endswith("_r01") and retry_authorization is None)
            or (not actual_attempt_id.endswith("_r01") and retry_authorization is not None),
            "formal r01 must not use retry authorization and r02+ must provide one",
        )
        return FormalRowPreparation(
            sim_root=root,
            output_root=destination,
            formal_freeze_path=freeze_path,
            formal_freeze_file_hash=freeze_file_hash,
            formal_freeze_hash=freeze_hash,
            formal_freeze=freeze,
            formal_master_path=master_path,
            formal_master_file_hash=master_file_hash,
            formal_master_hash=str(master_hash),
            formal_master=master,
            row=row,
            attempt_id=actual_attempt_id,
            seed_bundle=seed_bundle,
            expected_hashes=expected_hashes,
            effective_config_contract=config_contract,
            launch_contract=launch_contract,
            backend=backend,
            assets=assets,
            stage_entry_evidence=None if stage_entry_evidence is None else dict(stage_entry_evidence),
            retry_authorization=retry_authorization,
        )
    except AdapterError:
        raise
    except Exception as exc:  # normalize toolchain/path errors to the adapter ABI
        raise AdapterError(f"formal row preparation failed: {exc!r}") from exc


def formal_runner_spec(preparation: FormalRowPreparation) -> Dict[str, Any]:
    """Derive the only admissible ``run_single_row`` spec for a formal row."""
    case_dir = _formal_row_case_dir(preparation.output_root, preparation.row, preparation.attempt_id)
    _ensure_below(case_dir, preparation.output_root, "formal derived case directory")
    artifact_paths = {
        key: str((case_dir / filename).resolve())
        for key, filename in preparation.backend.artifacts.items()
    }
    for key, path_value in artifact_paths.items():
        require(Path(path_value).parent == case_dir, f"formal backend {key} artifact escaped its derived case directory")
    assets: Dict[str, Any] = {
        "map_file": preparation.assets["map"]["path"],
        "world_file": preparation.assets["world"]["path"],
        "world_geometry_file": preparation.assets["world_geometry"]["path"],
        "robot_model_file": preparation.assets["robot"]["path"],
        "path_file": preparation.assets["sim_path"]["path"],
        "physical_parameter_file": preparation.assets["container_parameters"]["path"],
        "effective_config_file": preparation.assets["effective_config"]["path"],
    }
    if preparation.row["condition_id"] == "FixedProfile":
        stratum = f"{preparation.row['path_id']}_{preparation.row['container_id']}"
        profiles = preparation.formal_freeze.get("fixed_profiles")
        require(isinstance(profiles, Mapping) and isinstance(profiles.get(stratum), Mapping), "formal FixedProfile registry disappeared after preparation")
        assets["fixed_profile"] = dict(profiles[stratum])
    recording = preparation.formal_freeze.get("recording_policy")
    require(isinstance(recording, Mapping), "formal recording policy disappeared after preparation")
    firewall = preparation.formal_freeze.get("controller_firewall")
    require(isinstance(firewall, Mapping) and isinstance(firewall.get("controller_nodes"), list), "formal controller firewall disappeared after preparation")
    spec: Dict[str, Any] = {
        "formal_freeze": dict(preparation.formal_freeze),
        "formal_master": str(preparation.formal_master_path),
        "attempt_id": preparation.attempt_id,
        "retry_authorization": preparation.retry_authorization,
        "stage_entry_evidence": preparation.stage_entry_evidence,
        "assets": assets,
        "liquid_plant_capability": dict(preparation.formal_freeze["liquid_plant_capability"]),
        "controller_nodes": list(firewall["controller_nodes"]),
        "ros_master_uri": preparation.launch_contract["ros_master_uri"],
        "gazebo_master_uri": preparation.launch_contract["gazebo_master_uri"],
        "startup_timeout_sec": preparation.launch_contract["startup_timeout_sec"],
        "command_timeout_sec": preparation.launch_contract["command_timeout_sec"],
        "settle_sec": recording["settle_sec"],
        "goal_timeout_sec": recording["goal_timeout_sec"],
        "tail_sec": recording["tail_sec"],
        "post_shutdown_sec": recording["post_shutdown_sec"],
        "recorder_ready_timeout_sec": recording["recorder_ready_timeout_sec"],
        "recorder_artifact": artifact_paths["recorder_artifact"],
        "runtime_ack_path": artifact_paths["runtime_ack"],
        "motion_release_ack_path": artifact_paths["motion_release_ack"],
        "motion_stop_ack_path": artifact_paths["motion_stop_ack"],
        # The generic runner checks this before it creates a case directory;
        # adapter delegates independently re-derive the same manifest from
        # SMPCC_FORMAL_FREEZE_PATH at each ABI command as a second binding.
        "formal_runtime_backend": {
            "backend_id": preparation.backend.backend_id,
            "backend_hash": preparation.backend.backend_hash,
            "manifest_path": str(preparation.backend.manifest_path),
            "manifest_file_hash": preparation.backend.manifest_file_hash,
            "case_artifacts": dict(preparation.backend.artifacts),
        },
    }
    for field, command in preparation.launch_contract["commands"].items():
        spec[field] = list(command)
    require(spec["settle_sec"] == 30.0, "derived formal spec settle is not exactly 30 seconds")
    require(spec["goal_timeout_sec"] == 60.0, "derived formal spec effective-motion window is not exactly 60 seconds")
    require(spec["tail_sec"] > 0.0, "derived formal spec has no frozen tail")
    return spec


def formal_row_preparation_report(preparation: FormalRowPreparation) -> Dict[str, Any]:
    """Human/machine evidence for a ready-but-not-executed formal row."""
    spec = formal_runner_spec(preparation)
    return {
        "schema_version": ADAPTER_SCHEMA_VERSION,
        "adapter_id": ADAPTER_ID,
        "status": "FORMAL_ROW_PREPARED_NOT_EXECUTED",
        "formal": True,
        "formal_execution_authorized": False,
        "planned_row_id": preparation.row["planned_row_id"],
        "attempt_id": preparation.attempt_id,
        "formal_freeze_hash": preparation.formal_freeze_hash,
        "formal_master_hash": preparation.formal_master_hash,
        "seed_bundle_hash": preparation.seed_bundle["seed_bundle_hash"],
        "runtime_backend_id": preparation.backend.backend_id,
        "runtime_backend_hash": preparation.backend.backend_hash,
        "effective_motion_window_sec": spec["goal_timeout_sec"],
        "settle_sec": spec["settle_sec"],
        "tail_sec": spec["tail_sec"],
        "runtime_artifacts": {
            key: spec[spec_key]
            for key, spec_key in (
                ("recorder_artifact", "recorder_artifact"),
                ("runtime_ack", "runtime_ack_path"),
                ("motion_release_ack", "motion_release_ack_path"),
                ("motion_stop_ack", "motion_stop_ack_path"),
            )
        },
        "reason": "all inputs are derived from formal artifacts, but this report does not start ROS/Gazebo or consume a planned row",
    }


def execute_prepared_formal_row(
    preparation: FormalRowPreparation,
    *,
    authorize_execution: bool = False,
) -> Dict[str, Any]:
    """Run the one-row lifecycle only after an explicit caller-side authorization.

    The explicit boolean is intentionally inconvenient: importing this module,
    calling ``prepare_formal_row``, or printing a report can never start a
    formal row.  Once authorized, the generic runner enforces all fresh-master
    and ACK/tail semantics; the eight adapter commands then only exec their
    hash-bound delegates.
    """
    require(authorize_execution is True, "formal row execution requires explicit authorize_execution=True")
    try:
        source_separation.require_execution_identity(
            preparation.formal_freeze, preparation.formal_master
        )
    except source_separation.SourceSeparationError as exc:
        raise AdapterError(str(exc)) from exc
    spec = formal_runner_spec(preparation)
    try:
        return _toolchain_call(
            toolchain.run_single_row,
            preparation.row,
            spec,
            preparation.output_root,
            preparation.sim_root,
        )
    except AdapterError:
        raise
    except Exception as exc:
        raise AdapterError(f"formal row execution failed: {exc!r}") from exc


def preflight_report(context: FormalRuntimeContext) -> Dict[str, Any]:
    """Return static evidence without starting a formal row."""
    return {
        "schema_version": ADAPTER_SCHEMA_VERSION,
        "adapter_id": ADAPTER_ID,
        "status": "FORMAL_RUNTIME_BACKEND_BOUND_NOT_EXECUTED",
        "formal": True,
        "formal_execution_authorized": False,
        "runtime_backend_implemented": True,
        "can_dispatch_hash_bound_backend": True,
        "reason": "static master/freeze/case/seed/assets/backend binding passed; this report neither starts ROS/Gazebo nor records a formal attempt",
        "case_manifest_hash": context.case_manifest_hash,
        "formal_freeze_hash": context.formal_freeze_hash,
        "formal_master_hash": context.formal_master_hash,
        "planned_row_id": context.row["planned_row_id"],
        "condition_id": context.row["condition_id"],
        "path_id": context.row["path_id"],
        "container_id": context.row["container_id"],
        "seed_bundle_hash": context.seed_bundle.get("seed_bundle_hash"),
        "expected_frozen_asset_hashes": dict(context.expected_hashes),
        "effective_config_contract": dict(context.effective_config_contract),
        "assets": {key: dict(value) for key, value in context.assets.items()},
        "abi": dict(ABI_SUBCOMMANDS),
        "runtime_backend": {
            "backend_id": context.backend.backend_id,
            "backend_hash": context.backend.backend_hash,
            "manifest_path": str(context.backend.manifest_path),
            "manifest_file_hash": context.backend.manifest_file_hash,
            "case_artifacts": dict(context.backend.artifacts),
            "lifecycle": dict(context.backend.lifecycle),
        },
        "process_safety": "no process is started or signalled by preflight",
    }


def case_runtime_artifact_paths(context: FormalRuntimeContext) -> Dict[str, Path]:
    """Resolve the backend's frozen artifact names beneath this exact case only."""
    output: Dict[str, Path] = {}
    for key, filename in context.backend.artifacts.items():
        path = (context.case_dir / filename).resolve()
        require(path.parent == context.case_dir.resolve(), f"formal backend {key} escaped its case directory")
        output[key] = path
    return output


def resolve_backend_delegate(
    context: FormalRuntimeContext,
    subcommand: str,
    *,
    environment: Optional[Mapping[str, str]] = None,
) -> Dict[str, Any]:
    """Return the sole hash-bound exec target for one active ABI command.

    This function does not spawn anything.  It is separately testable so a
    review can prove that no caller-controlled command or asset selector leaks
    from environment/CLI into the eventual process image.
    """
    require(subcommand in SUBCOMMAND_TO_FIELD, f"unknown formal ABI subcommand: {subcommand}")
    field = SUBCOMMAND_TO_FIELD[subcommand]
    source: Mapping[str, str] = os.environ if environment is None else environment
    expected = {
        "SMPCC_CASE_LAUNCH_MANIFEST_PATH": str(context.case_manifest_path),
        "SMPCC_CASE_LAUNCH_MANIFEST_SHA256": context.case_manifest_hash,
        "SMPCC_SEED_BUNDLE_PATH": str(context.seed_bundle_path),
        "SMPCC_SEED_BUNDLE_SHA256": str(context.seed_bundle.get("seed_bundle_hash")),
        "SMPCC_FORMAL_FREEZE_PATH": str(context.formal_freeze_path),
        "SMPCC_FORMAL_FREEZE_FILE_SHA256": context.formal_freeze_file_hash,
        "SMPCC_FORMAL_MASTER_PATH": str(context.formal_master_path),
        "SMPCC_FORMAL_MASTER_FILE_SHA256": context.formal_master_file_hash,
        "ROS_MASTER_URI": _toolchain_call(toolchain.http_uri, str(context.launch_contract["ros_master_uri"])),
        "GAZEBO_MASTER_URI": _toolchain_call(toolchain.http_uri, str(context.launch_contract["gazebo_master_uri"])),
    }
    for key in BACKEND_REQUIRED_ENVIRONMENT:
        require(source.get(key) == expected[key], f"formal backend {field} environment {key} differs from the bound case")
    dispatch_environment = {str(key): str(value) for key, value in source.items()}
    additions = {
        "SMPCC_FORMAL_RUNTIME_BACKEND_PATH": str(context.backend.manifest_path),
        "SMPCC_FORMAL_RUNTIME_BACKEND_FILE_SHA256": context.backend.manifest_file_hash,
        "SMPCC_FORMAL_RUNTIME_BACKEND_ID": context.backend.backend_id,
        "SMPCC_FORMAL_RUNTIME_BACKEND_HASH": context.backend.backend_hash,
        "SMPCC_FORMAL_RUNTIME_BACKEND_COMMAND_FIELD": field,
    }
    for key, value in additions.items():
        existing = dispatch_environment.get(key)
        require(existing in (None, value), f"formal backend {field} received an untrusted {key} override")
        dispatch_environment[key] = value
    command = list(context.backend.commands[field])
    return {
        "field": field,
        "subcommand": subcommand,
        "command": command,
        "environment": dispatch_environment,
        "case_artifacts": {key: str(path) for key, path in case_runtime_artifact_paths(context).items()},
    }


def dispatch_backend_command(
    context: FormalRuntimeContext,
    subcommand: str,
    *,
    environment: Optional[Mapping[str, str]] = None,
    executor=None,
) -> None:
    """Exec exactly one validated backend delegate; never shell out or kill broadly."""
    resolved = resolve_backend_delegate(context, subcommand, environment=environment)
    exec_fn = os.execve if executor is None else executor
    try:
        exec_fn(str(resolved["command"][0]), list(resolved["command"]), dict(resolved["environment"]))
    except OSError as exc:
        raise AdapterError(f"formal backend {resolved['field']} exec failed: {exc}") from exc
    # ``execve`` must not return.  Treat a test/double or a broken wrapper that
    # returns as a protocol failure rather than falling through as a success.
    raise AdapterError(f"formal backend {resolved['field']} exec returned unexpectedly")


def owned_pid_cleanup_plan(records: Sequence[Mapping[str, Any]], context: FormalRuntimeContext) -> list[Dict[str, Any]]:
    """Validate a future adapter's owned process records without killing anything.

    The only admissible cleanup target is the process group whose leader PID
    was recorded by this adapter for this exact case manifest.  The function
    returns a declarative TERM-then-KILL plan; execution is intentionally out
    of scope for this non-running scaffold.
    """
    plan: list[Dict[str, Any]] = []
    seen: set[int] = set()
    for index, record in enumerate(records):
        require(isinstance(record, Mapping), f"owned PID record {index} must be an object")
        require(record.get("adapter_id") == ADAPTER_ID, f"owned PID record {index} is not owned by this adapter")
        require(record.get("case_manifest_hash") == context.case_manifest_hash, f"owned PID record {index} belongs to another case")
        pid = record.get("pid")
        pgid = record.get("process_group_id")
        require(isinstance(pid, int) and pid > 1, f"owned PID record {index} has unsafe pid")
        require(isinstance(pgid, int) and pgid == pid and pgid > 1, f"owned PID record {index} must target its own process-group leader")
        require(pid not in seen, f"owned PID record {index} duplicates pid {pid}")
        command = record.get("command")
        require(isinstance(command, list) and command and all(isinstance(item, str) for item in command), f"owned PID record {index} lacks argv")
        _toolchain_call(toolchain.command_from_spec, command, f"owned PID record {index} command")
        expected_command_hash = _canonical_hash(command)
        require(record.get("command_hash") == expected_command_hash, f"owned PID record {index} command hash mismatch")
        seen.add(pid)
        plan.append(
            {
                "pid": pid,
                "process_group_id": pgid,
                "signals": ["SIGTERM", "SIGKILL_IF_STILL_OWNED_AFTER_TIMEOUT"],
                "scope": "adapter-owned process group only",
            }
        )
    return plan


def describe() -> Dict[str, Any]:
    return {
        "schema_version": ADAPTER_SCHEMA_VERSION,
        "adapter_id": ADAPTER_ID,
        "status": "HASH_BOUND_BACKEND_DISPATCH_IMPLEMENTED_FAIL_CLOSED",
        "formal_execution_authorized": False,
        "runtime_backend_implemented": True,
        "abi": dict(ABI_SUBCOMMANDS),
        "required_environment": list(REQUIRED_ENVIRONMENT),
        "backend_required_environment": list(BACKEND_REQUIRED_ENVIRONMENT),
        "refuses": [
            "H0/H0b/H0s",
            "runtime_s_curve",
            "W5_S10",
            "legacy default B_slosh/B_ours variants",
            "shared-target R1--R7 release identity",
            "missing hash-bound formal artifacts",
            "missing/hash-mismatched FROZEN formal runtime backend manifest",
            "fixture/development backend manifests",
            "mutable CLI condition/path/config/profile/container selectors",
            "untrusted backend environment overrides",
        ],
        "does_not": [
            "generate or select Bslosh",
            "generate paths/configs/profiles",
            "contain an embedded ROS/Gazebo/H0 runtime fallback",
            "convert a development backend into a formal delegate",
            "send signals or use broad process cleanup",
        ],
    }


def _emit(value: Mapping[str, Any]) -> None:
    print(json.dumps(value, ensure_ascii=False, sort_keys=True))


def command_run_formal_row(args: argparse.Namespace) -> int:
    """Prepare, and only with an explicit flag execute, one formal row."""
    stage_entry_evidence: Optional[Mapping[str, Any]] = None
    try:
        if args.stage_entry_evidence is not None:
            stage_entry_evidence = _read_json(Path(args.stage_entry_evidence), "formal stage-entry evidence")
        preparation = prepare_formal_row(
            formal_freeze_path=Path(args.formal_freeze),
            formal_master_path=Path(args.formal_master),
            planned_row_id=str(args.planned_row_id),
            output_root=Path(args.output_root),
            attempt_id=args.attempt_id,
            stage_entry_evidence=stage_entry_evidence,
            retry_authorization=None if args.retry_authorization is None else str(args.retry_authorization),
            sim_root=Path(args.sim_root),
        )
        if not args.execute_formal_row:
            _emit(formal_row_preparation_report(preparation))
            return 2
        report = execute_prepared_formal_row(preparation, authorize_execution=True)
        _emit(report)
        return 0 if report.get("status") == "PASS" else 2
    except AdapterError as exc:
        _emit(
            {
                "schema_version": ADAPTER_SCHEMA_VERSION,
                "adapter_id": ADAPTER_ID,
                "status": "NO_GO",
                "formal_execution_authorized": False,
                "error": str(exc),
            }
        )
        return 2


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="fail-closed hash-bound formal SMPCC-SIM runtime adapter")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("describe", help="describe the fail-closed 8-command backend-dispatch ABI")
    preflight = sub.add_parser("preflight", help="offline hash/case/config/asset preflight only")
    preflight.add_argument("--sim-root", type=Path, default=DEFAULT_SIM_ROOT)
    for command in ABI_SUBCOMMANDS.values():
        item = sub.add_parser(command, help=f"formal ABI command ({command}; dispatches only a hash-bound frozen backend)")
        item.add_argument("--sim-root", type=Path, default=DEFAULT_SIM_ROOT)
    run = sub.add_parser("run-formal-row", help="prepare and, only with explicit authorization, execute one frozen formal row")
    run.add_argument("--formal-freeze", type=Path, required=True)
    run.add_argument("--formal-master", type=Path, required=True)
    run.add_argument("--planned-row-id", required=True)
    run.add_argument("--output-root", type=Path, required=True)
    run.add_argument("--attempt-id")
    run.add_argument("--retry-authorization", type=Path)
    run.add_argument("--stage-entry-evidence", type=Path)
    run.add_argument("--sim-root", type=Path, default=DEFAULT_SIM_ROOT)
    run.add_argument(
        "--execute-formal-row",
        action="store_true",
        help="required to start a formal row; without it, only a no-write preparation report is emitted",
    )
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = _parser().parse_args(list(argv) if argv is not None else None)
    try:
        if args.command == "describe":
            _emit(describe())
            return 0
        if args.command == "run-formal-row":
            return command_run_formal_row(args)
        context = preflight_case(sim_root=args.sim_root)
        if args.command == "preflight":
            _emit(preflight_report(context))
            return 0
        dispatch_backend_command(context, str(args.command))
    except AdapterError as exc:
        _emit(
            {
                "schema_version": ADAPTER_SCHEMA_VERSION,
                "adapter_id": ADAPTER_ID,
                "status": "NO_GO",
                "formal_execution_authorized": False,
                "error": str(exc),
            }
        )
        return 2
    return 2


if __name__ == "__main__":
    sys.exit(main())
