#!/usr/bin/env python3
"""Static gate for the frozen U3 Stage-4 numerical contract v1."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import stat
import sys
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator


ROOT = Path(__file__).resolve().parent.parent
CONTRACT = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_numerical_contract_v1.json"
SCHEMA = ROOT / "schema/target_host_u3_stage4_numerical_contract_v1.json"
MAX_BYTES = 4 * 1024 * 1024


class ContractError(ValueError):
    pass


def read_regular(path: Path) -> tuple[bytes, dict[str, Any]]:
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    digest = hashlib.sha256()
    raw = bytearray()
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or not 0 < metadata.st_size <= MAX_BYTES:
            raise ContractError(f"unsafe contract input: {path}")
        while True:
            block = os.read(descriptor, 1 << 20)
            if not block:
                break
            digest.update(block)
            raw.extend(block)
    finally:
        os.close(descriptor)
    return bytes(raw), {"path": str(path), "size_bytes": len(raw), "sha256": digest.hexdigest()}


def read_json(path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    raw, identity = read_regular(path)
    try:
        value = json.loads(raw.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise ContractError(f"invalid JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ContractError(f"JSON root differs: {path}")
    return value, identity


def assert_deep_closed(value: Any, path: str = "$") -> None:
    if isinstance(value, dict):
        kind = value.get("type")
        if kind == "object" or (isinstance(kind, list) and "object" in kind):
            if value.get("additionalProperties") is not False:
                raise ContractError(f"schema object is not closed at {path}")
        for key, item in value.items():
            assert_deep_closed(item, f"{path}/{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            assert_deep_closed(item, f"{path}/{index}")


def self_check() -> dict[str, Any]:
    contract, contract_identity = read_json(CONTRACT)
    schema, schema_identity = read_json(SCHEMA)
    Draft202012Validator.check_schema(schema)
    assert_deep_closed(schema)
    errors = sorted(Draft202012Validator(schema).iter_errors(contract), key=lambda item: list(item.absolute_path))
    if errors:
        first = errors[0]
        raise ContractError(f"contract schema failure at {list(first.absolute_path)}: {first.message}")
    parents: dict[str, Any] = {}
    for name, expected in contract["parents"].items():
        _raw, observed = read_regular(Path(expected["path"]))
        if observed["sha256"] != expected["sha256"]:
            raise ContractError(f"parent hash drift: {name}")
        parents[name] = observed
    s4a, _ = read_json(Path(contract["parents"]["s4a_result"]["path"]))
    if s4a.get("verdict", {}).get("status") != contract["s4a_adjudication"]["status"]:
        raise ContractError("S4-A status differs")
    comparison = s4a.get("comparison", {})
    if comparison.get("candidate_to_baseline_primary_score_ratio") != contract["s4a_adjudication"]["primary_normalized_score_ratio"]:
        raise ContractError("S4-A score ratio differs")
    if comparison.get("primary_metrics_meaningfully_improved") != 0:
        raise ContractError("S4-A primary improvement count differs")
    probe = contract["remediation_probe"]
    if probe["single_delta"]["other_numerical_parameters_changed"]:
        raise ContractError("remediation is not single-variable")
    if contract["execution_limits"]["network"] or contract["execution_limits"]["project_workspace_write_bind_count"]:
        raise ContractError("execution boundary allows network/project writes")
    if not contract["result_boundary"]["missing_measured_physical_inputs_prevent_formal_stage4_pass"]:
        raise ContractError("physical claim ceiling drifted")
    return {
        "status": "PASS_U3_STAGE4_NUMERICAL_CONTRACT_V1_SELF_CHECK",
        "contract": contract_identity,
        "schema": schema_identity,
        "parents": parents,
        "s4a_status": s4a["verdict"]["status"],
        "single_delta": probe["single_delta"],
        "network_used": False,
        "solver_executed": False,
        "candidate_executed": False,
        "files_modified": False,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        result = self_check()
    except (ContractError, OSError, ValueError) as exc:
        print(json.dumps({"status": "FAIL_U3_STAGE4_NUMERICAL_CONTRACT_V1", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
