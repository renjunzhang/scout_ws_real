#!/usr/bin/env python3
"""Sequentially generate a real independent-Plant recovery dataset.

The C++ sampler owns motion, the motion-only controller liquid observer,
candidate pulses, command queues, and external-Plant recovery labels.  This
script only enforces split/seed isolation, invokes one complete seed batch at
a time, validates the exact fitter schema, and publishes an immutable merged
CSV plus audit/manifest sidecars.
"""

import argparse
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

import yaml


SESSION_SCHEMA = "spmpc_formal_simulation_session_v1"
MANIFEST_SCHEMA = "spmpc_phase_rejoin_recovery_rollout_dataset_manifest_v1"
SPLITS = ("fit", "tune", "held_out")
SESSION_SEED_KEYS = {
    "fit": "recovery_fit",
    "tune": "recovery_tune",
    "held_out": "recovery_held_out",
}
DATASET_COLUMNS = (
    "split",
    "rollout_id",
    "seed",
    "phase_index",
    "recovered",
    "x",
    "y",
    "yaw",
    "v",
    "omega",
    "eta_x",
    "eta_x_dot",
    "eta_y",
    "eta_y_dot",
    "linear_output",
    "angular_output",
    "linear_pending_0",
    "linear_pending_1",
    "linear_pending_2",
    "linear_pending_3",
    "linear_pending_4",
    "angular_pending_0",
    "angular_pending_1",
    "angular_pending_2",
    "angular_pending_3",
    "angular_pending_4",
    "angular_pending_5",
    "angular_pending_6",
)


class DatasetGenerationError(RuntimeError):
    """Fail-closed session, subprocess, or dataset error."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_yaml(path: Path) -> Mapping[str, Any]:
    try:
        value = yaml.safe_load(path.read_text("utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise DatasetGenerationError("cannot load {}: {}".format(path, error))
    if not isinstance(value, dict):
        raise DatasetGenerationError("{} is not a mapping".format(path))
    return value


def _canonical_seed(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise DatasetGenerationError("{} is not an integer seed".format(label))
    if value < 0 or value > (1 << 32) - 1:
        raise DatasetGenerationError("{} is outside uint32".format(label))
    return value


def load_session_seeds(path: Path) -> Mapping[str, Tuple[int, ...]]:
    session = _load_yaml(path)
    scope = session.get("scope")
    if (
        session.get("schema") != SESSION_SCHEMA
        or session.get("formal_trials_started") is not False
        or not isinstance(scope, dict)
        or scope.get("simulation_only") is not True
        or scope.get("formal_robot_release") is not False
        or scope.get("real_robot_enforce_allowed") is not False
        or scope.get("plant_truth_visible_to_controller") is not False
        or scope.get("physical_parameter_claim") is not False
    ):
        raise DatasetGenerationError("formal simulation session scope is unsafe")
    raw_seeds = session.get("seeds")
    if not isinstance(raw_seeds, dict) or raw_seeds.get("locked") is not True:
        raise DatasetGenerationError("session recovery seeds are not locked")
    owners: Dict[int, str] = {}
    result: Dict[str, Tuple[int, ...]] = {}
    for split in SPLITS:
        key = SESSION_SEED_KEYS[split]
        values = raw_seeds.get(key)
        if not isinstance(values, list) or not values:
            raise DatasetGenerationError("{} seed list is empty".format(key))
        seeds = tuple(
            _canonical_seed(value, "{}[{}]".format(key, index))
            for index, value in enumerate(values)
        )
        if len(set(seeds)) != len(seeds):
            raise DatasetGenerationError("{} contains a duplicate seed".format(key))
        for seed in seeds:
            previous = owners.get(seed)
            if previous is not None:
                raise DatasetGenerationError(
                    "seed {} crosses {} and {}".format(seed, previous, split)
                )
            owners[seed] = split
        result[split] = seeds
    return result


def load_frozen_plant_identity(path: Path) -> Mapping[str, str]:
    config = _load_yaml(path)
    scope = config.get("scope")
    provenance = config.get("provenance")
    if (
        config.get("schema") != "spmpc_independent_simulation_config_v1"
        or config.get("status") != "formal_simulation_release"
        or not isinstance(config.get("freeze_id"), str)
        or not config["freeze_id"]
        or not isinstance(scope, dict)
        or scope.get("simulation_only") is not True
        or scope.get("formal_robot_release") is not False
        or scope.get("real_robot_enforce_allowed") is not False
        or scope.get("result_claim_authorized") is not False
        or scope.get("physical_parameter_claim") is not False
        or not isinstance(provenance, dict)
        or provenance.get("source_limitations_acknowledged") is not True
        or provenance.get("physical_parameter_claim") is not False
    ):
        raise DatasetGenerationError(
            "Plant is not a frozen simulation-only, non-physical parameter image"
        )
    return {
        "freeze_id": config["freeze_id"],
        "status": config["status"],
    }


def load_profile_count(path: Path) -> int:
    config = _load_yaml(path)
    sampling = config.get("sampling")
    profiles = sampling.get("profiles") if isinstance(sampling, dict) else None
    if not isinstance(profiles, list) or not profiles:
        raise DatasetGenerationError("sampling profile table is empty")
    identifiers = []
    for index, profile in enumerate(profiles):
        if not isinstance(profile, dict) or not isinstance(
            profile.get("profile_id"), str
        ):
            raise DatasetGenerationError("invalid sampling profile {}".format(index))
        identifiers.append(profile["profile_id"])
    if len(set(identifiers)) != len(identifiers):
        raise DatasetGenerationError("sampling profile IDs are not unique")
    return len(profiles)


def _read_csv(path: Path) -> Tuple[Tuple[str, ...], List[Mapping[str, str]]]:
    try:
        with path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            header = tuple(reader.fieldnames or ())
            rows = list(reader)
    except (OSError, UnicodeError, csv.Error) as error:
        raise DatasetGenerationError("cannot read {}: {}".format(path, error))
    if any(None in row for row in rows):
        raise DatasetGenerationError("{} contains extra columns".format(path))
    return header, rows


def _validate_partial(
    split: str,
    seed: int,
    phase_begin: int,
    phase_end: int,
    profile_count: int,
    dataset_path: Path,
    audit_path: Path,
) -> Tuple[List[Mapping[str, str]], Tuple[str, ...], List[Mapping[str, str]]]:
    header, rows = _read_csv(dataset_path)
    if header != DATASET_COLUMNS:
        raise DatasetGenerationError("C++ sampler dataset header mismatch")
    expected_count = (phase_end - phase_begin + 1) * profile_count
    if len(rows) != expected_count:
        raise DatasetGenerationError(
            "C++ sampler row count {} != {}".format(len(rows), expected_count)
        )
    rollout_ids = set()
    phase_counts = {phase: 0 for phase in range(phase_begin, phase_end + 1)}
    for row in rows:
        if row["split"] != split or row["seed"] != str(seed):
            raise DatasetGenerationError("partial row split/seed mismatch")
        try:
            phase = int(row["phase_index"])
        except ValueError as error:
            raise DatasetGenerationError("non-integer phase index") from error
        if phase not in phase_counts or row["recovered"] not in ("0", "1"):
            raise DatasetGenerationError("partial row phase/label mismatch")
        phase_counts[phase] += 1
        if row["rollout_id"] in rollout_ids:
            raise DatasetGenerationError("duplicate partial rollout_id")
        rollout_ids.add(row["rollout_id"])
    if any(count != profile_count for count in phase_counts.values()):
        raise DatasetGenerationError("profile coverage differs across phases")

    audit_header, audits = _read_csv(audit_path)
    if not audit_header or len(audits) != len(rows):
        raise DatasetGenerationError("C++ sampler audit cardinality mismatch")
    audit_ids = {row.get("rollout_id") for row in audits}
    if audit_ids != rollout_ids:
        raise DatasetGenerationError("dataset/audit rollout identities differ")
    for audit in audits:
        if (
            audit.get("external_liquid_truth_visible_to_candidate_policy") != "0"
            or audit.get("external_liquid_truth_used_for_features") != "0"
            or audit.get("external_liquid_truth_used_for_label") != "1"
        ):
            raise DatasetGenerationError("C++ sampler truth isolation failed")
    return rows, audit_header, audits


def _csv_bytes(
    header: Sequence[str], rows: Iterable[Mapping[str, str]]
) -> bytes:
    output = io.StringIO(newline="")
    writer = csv.DictWriter(output, fieldnames=header, lineterminator="\n")
    writer.writeheader()
    for row in rows:
        writer.writerow({name: row[name] for name in header})
    return output.getvalue().encode("utf-8")


def _json_bytes(value: Any) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8")


def _write_all(descriptor: int, contents: bytes) -> None:
    offset = 0
    while offset < len(contents):
        count = os.write(descriptor, contents[offset:])
        if count <= 0:
            raise OSError("zero-length output write")
        offset += count
    os.fsync(descriptor)


def _publish_exclusive_bundle(outputs: Sequence[Tuple[Path, bytes]]) -> None:
    """Durably stage a bundle, then publish every member without overwrite.

    Hard-link publication gives every target an exclusive create operation.
    If any link or directory sync fails, all targets created by this call are
    removed; pre-existing targets are never removed.  Temporary files live in
    each target directory so publication does not cross filesystems.
    """
    if not outputs:
        raise DatasetGenerationError("output bundle is empty")
    resolved = [path.resolve(strict=False) for path, _ in outputs]
    if len(set(resolved)) != len(resolved):
        raise DatasetGenerationError("output bundle paths alias")
    for path in resolved:
        path.parent.mkdir(parents=True, exist_ok=True)
    if any(path.exists() for path in resolved):
        raise DatasetGenerationError("an output already exists")

    staged: List[Path] = []
    published: List[Path] = []
    try:
        for path, (_, contents) in zip(resolved, outputs):
            descriptor, temporary = tempfile.mkstemp(
                prefix=".{}.tmp.".format(path.name), dir=str(path.parent)
            )
            temporary_path = Path(temporary)
            staged.append(temporary_path)
            try:
                os.fchmod(descriptor, 0o644)
                _write_all(descriptor, contents)
            finally:
                os.close(descriptor)
        for temporary_path, path in zip(staged, resolved):
            os.link(str(temporary_path), str(path))
            published.append(path)
        for parent in sorted({path.parent for path in resolved}, key=str):
            descriptor = os.open(str(parent), os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
    except Exception:
        for path in reversed(published):
            try:
                path.unlink()
            except OSError:
                pass
        raise
    finally:
        for path in staged:
            try:
                path.unlink()
            except OSError:
                pass


def generate(args: argparse.Namespace) -> Mapping[str, Any]:
    inputs = (
        args.sampler_bin,
        args.session,
        args.plant_config,
        args.offline_plan,
        args.offline_plan_report,
        args.path_json,
        args.sampling_config,
    )
    for path in inputs:
        if not path.is_file():
            raise DatasetGenerationError("input is not a file: {}".format(path))
    if not os.access(args.sampler_bin, os.X_OK):
        raise DatasetGenerationError("sampler binary is not executable")
    if args.phase_begin < 0 or args.phase_end < args.phase_begin:
        raise DatasetGenerationError("invalid phase range")
    output_paths = (args.output, args.audit_output, args.manifest_output)
    if len({path.resolve(strict=False) for path in output_paths}) != 3:
        raise DatasetGenerationError("output paths alias")
    if any(path.exists() for path in output_paths):
        raise DatasetGenerationError("an output already exists")

    seeds = load_session_seeds(args.session)
    plant_identity = load_frozen_plant_identity(args.plant_config)
    profile_count = load_profile_count(args.sampling_config)
    dataset_rows: List[Mapping[str, str]] = []
    audit_rows: List[Mapping[str, str]] = []
    audit_header: Tuple[str, ...] = ()
    invocations = []
    with tempfile.TemporaryDirectory(prefix="spmpc_recovery_dataset.") as temporary:
        root = Path(temporary)
        for split in SPLITS:
            for seed in seeds[split]:
                partial_dataset = root / "{}-{}.csv".format(split, seed)
                partial_audit = root / "{}-{}-audit.csv".format(split, seed)
                command = [
                    str(args.sampler_bin),
                    "--plant-config",
                    str(args.plant_config),
                    "--offline-plan",
                    str(args.offline_plan),
                    "--offline-plan-report",
                    str(args.offline_plan_report),
                    "--path-json",
                    str(args.path_json),
                    "--sampling-config",
                    str(args.sampling_config),
                    "--split",
                    split,
                    "--seed",
                    str(seed),
                    "--phase-begin",
                    str(args.phase_begin),
                    "--phase-end",
                    str(args.phase_end),
                    "--dataset-output",
                    str(partial_dataset),
                    "--audit-output",
                    str(partial_audit),
                ]
                completed = subprocess.run(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    check=False,
                )
                if completed.returncode != 0:
                    raise DatasetGenerationError(
                        "sampler failed for {}/{}: {}".format(
                            split, seed, completed.stderr.strip()
                        )
                    )
                rows, current_audit_header, audits = _validate_partial(
                    split,
                    seed,
                    args.phase_begin,
                    args.phase_end,
                    profile_count,
                    partial_dataset,
                    partial_audit,
                )
                if audit_header and current_audit_header != audit_header:
                    raise DatasetGenerationError("audit headers differ by seed")
                audit_header = current_audit_header
                dataset_rows.extend(rows)
                audit_rows.extend(audits)
                invocations.append(
                    {
                        "split": split,
                        "seed": seed,
                        "stdout": completed.stdout.strip(),
                    }
                )

    rollout_ids = [row["rollout_id"] for row in dataset_rows]
    if len(set(rollout_ids)) != len(rollout_ids):
        raise DatasetGenerationError("rollout identity crosses seed/split batches")
    phase_sets = {
        split: {
            int(row["phase_index"])
            for row in dataset_rows
            if row["split"] == split
        }
        for split in SPLITS
    }
    if not (phase_sets["fit"] == phase_sets["tune"] == phase_sets["held_out"]):
        raise DatasetGenerationError("phase coverage differs across splits")

    dataset_contents = _csv_bytes(DATASET_COLUMNS, dataset_rows)
    audit_contents = _csv_bytes(audit_header, audit_rows)
    recovered_by_split = {
        split: sum(
            row["recovered"] == "1"
            for row in dataset_rows
            if row["split"] == split
        )
        for split in SPLITS
    }
    manifest = {
        "schema": MANIFEST_SCHEMA,
        "status": "PILOT_DATASET",
        "simulation_only": True,
        "formal_robot_release": False,
        "physical_parameter_claim": False,
        "source_limitations_acknowledged": True,
        "paper_main_result_eligible": False,
        "nominal_source": "offline_plan_replayed_with_compiled_22d_transition",
        "candidate_policy_reads_external_liquid_truth": False,
        "features_use_external_liquid_truth": False,
        "label_uses_external_liquid_truth": True,
        "synthetic_labels": False,
        "plant": plant_identity,
        "parallel_invocations": 1,
        "phase_range": {
            "begin": args.phase_begin,
            "end_inclusive": args.phase_end,
        },
        "profile_count": profile_count,
        "row_count": len(dataset_rows),
        "recovered_by_split": recovered_by_split,
        "seeds": {split: list(seeds[split]) for split in SPLITS},
        "inputs": {
            "sampler_binary_sha256": sha256_file(args.sampler_bin),
            "session_sha256": sha256_file(args.session),
            "plant_config_sha256": sha256_file(args.plant_config),
            "offline_plan_sha256": sha256_file(args.offline_plan),
            "offline_plan_report_sha256": sha256_file(args.offline_plan_report),
            "path_sha256": sha256_file(args.path_json),
            "sampling_config_sha256": sha256_file(args.sampling_config),
        },
        "outputs": {
            "dataset": {
                "path": args.output.name,
                "sha256": hashlib.sha256(dataset_contents).hexdigest(),
            },
            "audit": {
                "path": args.audit_output.name,
                "sha256": hashlib.sha256(audit_contents).hexdigest(),
            },
        },
        "invocations": invocations,
    }
    manifest_contents = _json_bytes(manifest)
    _publish_exclusive_bundle(
        (
            (args.output, dataset_contents),
            (args.audit_output, audit_contents),
            (args.manifest_output, manifest_contents),
        )
    )
    return manifest


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sampler-bin", type=Path, required=True)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--plant-config", type=Path, required=True)
    parser.add_argument("--offline-plan", type=Path, required=True)
    parser.add_argument("--offline-plan-report", type=Path, required=True)
    parser.add_argument("--path-json", type=Path, required=True)
    parser.add_argument("--sampling-config", type=Path, required=True)
    parser.add_argument("--phase-begin", type=int, required=True)
    parser.add_argument("--phase-end", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--audit-output", type=Path, required=True)
    parser.add_argument("--manifest-output", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str]) -> int:
    try:
        args = parse_args(argv)
        manifest = generate(args)
    except DatasetGenerationError as error:
        print("recovery dataset rejected: {}".format(error), file=sys.stderr)
        return 2
    except OSError as error:
        print("recovery dataset I/O failed: {}".format(error), file=sys.stderr)
        return 3
    print(
        "status={} rows={} recovered_by_split={} synthetic_labels=false".format(
            manifest["status"],
            manifest["row_count"],
            manifest["recovered_by_split"],
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
