#!/usr/bin/env python3
"""Generate a fail-closed Stage-I freeze draft or final GO manifest."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import random
import stat
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, MutableMapping, Sequence, Tuple

import yaml


PROTOCOL_ID = "SMPCC-REAL-40-88-v2.0"
SCOPE = "S1_CORE_40"
METHODS = ("B0", "Bsmooth", "SmoothMatch", "FixedProfile", "Bslosh")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_hash(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def resolve_path(value: Any, repo_root: Path) -> Path:
    if not isinstance(value, str) or not value.strip():
        return Path("")
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (repo_root / path).resolve()


def artifact(path: Path) -> Mapping[str, str]:
    return {"path": str(path), "sha256": sha256_file(path)}


def git_output(repo_root: Path, *args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(repo_root), *args],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    ).stdout.strip()


def deterministic_permutation(values: Sequence[Any], seed: str, tag: str) -> List[Any]:
    return sorted(
        values,
        key=lambda value: hashlib.sha256(
            (seed + "|" + tag + "|" + repr(value)).encode("utf-8")
        ).digest(),
    )


def stage1_orders(seed: str) -> List[List[str]]:
    labels = deterministic_permutation(list(METHODS), seed, "method-map")
    latin = [[labels[(row + column) % 5] for column in range(5)] for row in range(5)]
    reverse = [list(reversed(row)) for row in latin]
    first_five = deterministic_permutation(latin, seed, "latin-row-order")
    extra = deterministic_permutation(reverse, seed, "reverse-row-order")[:3]
    orders = deterministic_permutation(first_five + extra, seed, "block-order")
    if len(orders) != 8 or any(set(row) != set(METHODS) for row in orders):
        raise RuntimeError("internal randomization construction failed")
    for position in range(5):
        counts = {method: sum(row[position] == method for row in orders) for method in METHODS}
        if max(counts.values()) - min(counts.values()) > 1:
            raise RuntimeError("randomization position balance failed")
    return orders


def write_randomization(path: Path, seed: str) -> None:
    orders = stage1_orders(seed)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=(
                "stage",
                "planned_unit",
                "block",
                "order_position",
                "condition",
                "path_id",
                "container_id",
                "planned_block_segment_id",
                "run_label",
            ),
        )
        writer.writeheader()
        planned = 0
        for block_index, order in enumerate(orders, 1):
            for position, method in enumerate(order, 1):
                planned += 1
                writer.writerow(
                    {
                        "stage": "S1_CORE",
                        "planned_unit": "{:02d}".format(planned),
                        "block": "{:02d}".format(block_index),
                        "order_position": "{:02d}".format(position),
                        "condition": method,
                        "path_id": "H1",
                        "container_id": "C1",
                        "planned_block_segment_id": "S1_b{:02d}_seg01".format(block_index),
                        "run_label": "S1_H1_C1_{}_b{:02d}_p{:02d}_a01".format(
                            method, block_index, position
                        ),
                    }
                )


def load_json_report(path: Path) -> Mapping[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise RuntimeError("cannot parse JSON report {}: {}".format(path, exc)) from exc
    if not isinstance(data, Mapping):
        raise RuntimeError("JSON report root is not an object: {}".format(path))
    return data


def evidence_status_ok(name: str, report: Mapping[str, Any]) -> Tuple[bool, str]:
    status = str(report.get("status", ""))
    if name == "g2s_report":
        formal = report.get("decision_is_formal_release") is True or report.get("formal_release") is True
        return formal and status in {"PASS", "FORMAL_PASS"}, "status={!r}, formal_release={!r}".format(status, formal)
    if name in {"g2c_report", "g3_report", "g4_report", "g5_report"}:
        return status == "PASS", "status={!r}".format(status)
    return False, "unknown evidence kind"


def collect_artifacts(
    config: Mapping[str, Any], repo_root: Path, failures: List[str]
) -> Tuple[Mapping[str, Any], Mapping[str, Any]]:
    output: MutableMapping[str, Any] = {}
    reports: MutableMapping[str, Any] = {}
    sections = ("software", "methods", "path", "container", "vision", "analysis")
    for section_name in sections:
        section = config.get(section_name, {})
        if not isinstance(section, Mapping):
            failures.append("{} must be a mapping".format(section_name))
            continue
        encoded = {}
        for key, value in section.items():
            if key in {"id", "radius_m", "liquid_height_m", "damping_ratio"}:
                encoded[key] = value
                continue
            path = resolve_path(value, repo_root)
            if not str(path) or not path.is_file():
                failures.append("{}.{} artifact is missing: {}".format(section_name, key, value or "<empty>"))
                encoded[key] = {"path": str(path) if str(path) else "", "sha256": ""}
            else:
                encoded[key] = artifact(path)
        output[section_name] = encoded

    evidence = config.get("evidence", {})
    encoded_evidence = {}
    if not isinstance(evidence, Mapping):
        failures.append("evidence must be a mapping")
        evidence = {}
    for name in ("g2s_report", "g2c_report", "g3_report", "g4_report", "g5_report"):
        path = resolve_path(evidence.get(name), repo_root)
        if not str(path) or not path.is_file():
            failures.append("evidence.{} is missing: {}".format(name, evidence.get(name) or "<empty>"))
            encoded_evidence[name] = {"path": str(path) if str(path) else "", "sha256": "", "status": "MISSING"}
            continue
        report = load_json_report(path)
        ok, detail = evidence_status_ok(name, report)
        if not ok:
            failures.append("evidence.{} is not a formal PASS ({})".format(name, detail))
        encoded_evidence[name] = {
            **artifact(path),
            "status": report.get("status"),
            "formal_gate_pass": ok,
        }
        reports[name] = report
    output["evidence"] = encoded_evidence
    return output, reports


def build_manifest(
    config: Mapping[str, Any],
    repo_root: Path,
    out_dir: Path,
    finalize: bool,
) -> Tuple[Mapping[str, Any], List[str]]:
    failures: List[str] = []
    if config.get("schema_version") != 1:
        failures.append("input schema_version must be 1")
    if config.get("protocol_id") != PROTOCOL_ID:
        failures.append("protocol_id must be {}".format(PROTOCOL_ID))
    if config.get("scope") != SCOPE:
        failures.append("scope must be {}".format(SCOPE))
    seed = str(config.get("randomization_seed", ""))
    if not seed:
        failures.append("randomization_seed is empty")
    settle = config.get("t_settle_sec")
    if not isinstance(settle, (int, float)) or not math.isfinite(float(settle)) or float(settle) <= 0.0:
        failures.append("t_settle_sec must be positive")

    artifact_sections, _reports = collect_artifacts(config, repo_root, failures)
    source = config.get("source", {})
    if not isinstance(source, Mapping):
        source = {}
        failures.append("source must be a mapping")
    if source.get("nominal_observer_source") != "processed_imu":
        failures.append("nominal_observer_source must be processed_imu")
    if source.get("fallback_source") != "odom" or source.get("fallback_is_method_failure") is not True:
        failures.append("processed-IMU fallback contract must be odom + method failure")
    if float(source.get("final_w_slosh", math.nan)) != 5.0:
        failures.append("final_w_slosh must be 5.0")

    try:
        head = git_output(repo_root, "rev-parse", "HEAD")
        dirty = git_output(repo_root, "status", "--porcelain", "--untracked-files=normal")
    except (OSError, subprocess.CalledProcessError) as exc:
        failures.append("git inspection failed: {}".format(exc))
        head = ""
        dirty = "unknown"
    if dirty:
        failures.append("Git worktree is not clean")

    randomization = out_dir / "S1_CORE_40_randomization.csv"
    write_randomization(randomization, seed)
    random_artifact = artifact(randomization)

    h1_entry = artifact_sections.get("path", {}).get("h1_file", {})
    g3_entry = artifact_sections.get("evidence", {}).get("g3_report", {})
    if isinstance(h1_entry, Mapping) and h1_entry.get("sha256"):
        g3_report_path = Path(str(g3_entry.get("path", "")))
        if g3_report_path.is_file():
            g3 = load_json_report(g3_report_path)
            g3_paths = {
                item.get("bag_sha256") for item in g3.get("dataset_index", []) if item.get("bag_sha256")
            }
            # The path hash is stored in trial sidecars rather than the aggregate;
            # compare explicitly with the known development H0 when available.
            h0 = Path("/home/geist/fixed_paths/real/20260727_spmpc_development/H0/H0_G2.json")
            if h0.is_file() and h1_entry.get("sha256") == sha256_file(h0):
                failures.append("formal H1 is identical to viewed G3 H0; held-out path isolation fails")

    identity = {
        "protocol_id": PROTOCOL_ID,
        "scope": SCOPE,
        "git_revision": head,
        "t_settle_sec": settle,
        "source": dict(source),
        "artifacts": artifact_sections,
        "randomization": random_artifact,
        "randomization_seed_sha256": hashlib.sha256(seed.encode("utf-8")).hexdigest(),
    }
    identity_sha = canonical_hash(identity)
    freeze_id = "SMPCC40-{}".format(identity_sha[:16])
    gates = {
        "git_clean": not bool(dirty),
        "g2s_formal_source_selection": not any("g2s_report" in item for item in failures),
        "g2c_final_candidate": not any("g2c_report" in item for item in failures),
        "g3_rgb_efficacy": not any("g3_report" in item for item in failures),
        "g4_trajectory_replay": not any("g4_report" in item for item in failures),
        "g5_comparator_fairness": not any("g5_report" in item for item in failures),
        "h1_held_out_path": not any("H1" in item or "h1" in item for item in failures),
        "all_artifacts_authenticated": not any("artifact is missing" in item for item in failures),
    }
    manifest = {
        "schema_version": 2,
        "protocol_id": PROTOCOL_ID,
        "scope": SCOPE,
        "freeze_id": freeze_id,
        "status": "GO" if finalize and not failures else "NO-GO",
        "identity_sha256": identity_sha,
        "identity": identity,
        "gates": gates,
        "failures": failures,
    }
    return manifest, failures


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inputs", required=True)
    parser.add_argument("--repo-root", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--finalize", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    inputs_path = Path(args.inputs).expanduser().resolve()
    repo_root = Path(args.repo_root).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load(inputs_path.read_text(encoding="utf-8"))
    manifest, failures = build_manifest(config, repo_root, out_dir, args.finalize)

    readiness = {
        "schema_version": 1,
        "report_type": "G6_FREEZE_READINESS",
        "status": "PASS" if not failures else "NO-GO",
        "finalize_requested": bool(args.finalize),
        "freeze_id_candidate": manifest["freeze_id"],
        "failures": failures,
        "inputs": str(inputs_path),
        "inputs_sha256": sha256_file(inputs_path),
        "manifest_status": manifest["status"],
    }
    readiness_path = out_dir / "G6_FREEZE_READINESS_REPORT.json"
    readiness_path.write_text(
        json.dumps(readiness, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    draft_path = out_dir / "freeze_manifest.v2.draft.yaml"
    draft_path.write_text(yaml.safe_dump(manifest, sort_keys=True), encoding="utf-8")

    if args.finalize and not failures:
        manifest_path = out_dir / "freeze_manifest.yaml"
        manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=True), encoding="utf-8")
        os.chmod(manifest_path, stat.S_IRUSR | stat.S_IRGRP | stat.S_IROTH)
        (out_dir / "freeze_manifest_sha256.txt").write_text(
            "{}  {}\n".format(sha256_file(manifest_path), manifest_path), encoding="utf-8"
        )
        print("[G6] GO {}".format(manifest["freeze_id"]))
        print("  manifest={}".format(manifest_path))
        return 0

    print("[G6] NO-GO draft={}".format(draft_path))
    print("  readiness={}".format(readiness_path))
    for failure in failures:
        print("  BLOCKER: {}".format(failure), file=sys.stderr)
    return 2 if args.finalize else 1


if __name__ == "__main__":
    raise SystemExit(main())
