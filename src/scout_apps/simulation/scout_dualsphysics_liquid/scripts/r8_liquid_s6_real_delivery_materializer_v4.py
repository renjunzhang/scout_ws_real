#!/usr/bin/env python3
"""Materialize an exact, create-new S6 v4 runtime contract from real file facts.

The static template is always NOT_ADMITTED.  This module can only replace its
null placeholders with identities calculated from opened regular, single-link
files and an exact finalized S5B0 inventory.  Caller supplied hashes are never
accepted as facts.  It does not read an optional bag or execute a solver/GPU.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import stat
import sys
from copy import deepcopy
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
TEMPLATE = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s6_real_delivery_execution_contract_v4.template.json"
SCHEMA = ROOT / "schema/target_host_s6_real_delivery_execution_contract_v4.json"
ATTEMPT = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01"
REQUIRED = ("execution_receipt.json", "result_qc.json", "frame_manifest.json", "native_gauge_manifest.json", "checksums.sha256")
PROBES = tuple(f"s5b0_p{i:02d}" for i in range(16))
MAX_BYTES = 128 * 1024 * 1024


class S6MaterializerV4Error(ValueError):
    pass


def canonical_json(value: Any) -> bytes:
    try:
        return (json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":")) + "\n").encode()
    except (TypeError, ValueError) as exc:
        raise S6MaterializerV4Error("non-canonical JSON value") from exc


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_bytes(), parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
    if not isinstance(value, dict):
        raise S6MaterializerV4Error("JSON root is not object")
    return value


def assert_deep_closed(node: Any, location: str = "$") -> None:
    if isinstance(node, dict):
        if node.get("type") == "object" and node.get("additionalProperties") is not False:
            raise S6MaterializerV4Error(f"open schema object: {location}")
        for key, child in node.items():
            assert_deep_closed(child, f"{location}/{key}")
    elif isinstance(node, list):
        for index, child in enumerate(node):
            assert_deep_closed(child, f"{location}/{index}")


def load_contracts() -> tuple[dict[str, Any], dict[str, Any]]:
    template, schema = _json(TEMPLATE), _json(SCHEMA)
    Draft202012Validator.check_schema(schema)
    assert_deep_closed(schema)
    Draft202012Validator(schema).validate(template)
    for key in ("s6_engine_v3", "selected_signal_v5", "figure_media_v1", "bi4_reader_v1", "finalized_frame_reader_v1"):
        path = ROOT / template["dependencies"][f"{key}_path"]
        if sha256_bytes(path.read_bytes()) != template["dependencies"][f"{key}_sha256"]:
            raise S6MaterializerV4Error(f"dependency hash drift: {path.name}")
    return template, schema


def _components(path: Path) -> None:
    if not path.is_absolute() or str(path) != os.path.normpath(str(path)):
        raise S6MaterializerV4Error("path is not exact normalized absolute")
    cursor = Path(path.anchor)
    for part in path.parts[1:]:
        cursor /= part
        if stat.S_ISLNK(os.lstat(cursor).st_mode):
            raise S6MaterializerV4Error("symlink path component")


def read_exact_file(path: Path, *, maximum: int = MAX_BYTES) -> tuple[bytes, dict[str, Any]]:
    path = Path(path)
    _components(path)
    before = os.lstat(path)
    if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1 or before.st_size < 1 or before.st_size > maximum:
        raise S6MaterializerV4Error("file is not bounded regular single-link")
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        opened = os.fstat(fd)
        if (opened.st_dev, opened.st_ino) != (before.st_dev, before.st_ino):
            raise S6MaterializerV4Error("file changed before open")
        chunks, total = [], 0
        while True:
            block = os.read(fd, min(1 << 20, maximum + 1 - total))
            if not block:
                break
            total += len(block)
            if total > maximum:
                raise S6MaterializerV4Error("file exceeded bound")
            chunks.append(block)
        after = os.fstat(fd)
    finally:
        os.close(fd)
    fields = lambda row: (row.st_dev, row.st_ino, row.st_size, row.st_mtime_ns, row.st_ctime_ns, row.st_nlink)
    if fields(before) != fields(after) or total != before.st_size:
        raise S6MaterializerV4Error("file TOCTOU detected")
    raw = b"".join(chunks)
    return raw, {"path": str(path), "sha256": sha256_bytes(raw), "size_bytes": len(raw), "mode": f"{stat.S_IMODE(after.st_mode):04o}", "device": after.st_dev, "inode": after.st_ino, "nlink": after.st_nlink, "mtime_ns": after.st_mtime_ns, "ctime_ns": after.st_ctime_ns}


def _safe_relative(value: str) -> str:
    pure = PurePosixPath(value)
    if not value or pure.is_absolute() or str(pure) != value or any(part in ("", ".", "..") for part in pure.parts):
        raise S6MaterializerV4Error("unsafe inventory path")
    return value


def scan_exact_package(root: Path) -> tuple[dict[str, bytes], list[dict[str, Any]]]:
    root = Path(root)
    _components(root)
    if not stat.S_ISDIR(os.lstat(root).st_mode):
        raise S6MaterializerV4Error("S5B0 root is not directory")
    payloads, identities = {}, []
    root_fd = os.open(root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC)
    try:
        root_before = os.fstat(root_fd)
        names = []
        for current, directories, files in os.walk(root, followlinks=False):
            for directory in directories:
                if stat.S_ISLNK(os.lstat(Path(current) / directory).st_mode):
                    raise S6MaterializerV4Error("symlink package directory")
            for name in files:
                names.append(_safe_relative(str((Path(current) / name).relative_to(root))))
        for relative in sorted(names):
            raw, identity = read_exact_file(root / relative)
            payloads[relative] = raw
            identities.append({"relative_path": relative, **{key: value for key, value in identity.items() if key != "path"}})
        root_after = os.fstat(root_fd)
    finally:
        os.close(root_fd)
    root_fields = lambda row: (row.st_dev, row.st_ino, row.st_mtime_ns, row.st_ctime_ns)
    if root_fields(root_before) != root_fields(root_after) or not set(REQUIRED).issubset(payloads):
        raise S6MaterializerV4Error("package changed or required parents absent")
    lines = payloads["checksums.sha256"].decode("ascii").splitlines()
    expected = {}
    for line in lines:
        if len(line) < 67 or line[64:66] != "  ":
            raise S6MaterializerV4Error("checksums syntax")
        digest, relative = line[:64], _safe_relative(line[66:])
        if digest != sha256_bytes(payloads.get(relative, b"")) or relative in expected:
            raise S6MaterializerV4Error("checksums hash/duplicate")
        expected[relative] = digest
    if set(expected) != set(payloads) - {"checksums.sha256"}:
        raise S6MaterializerV4Error("checksums incomplete")
    execution = json.loads(payloads["execution_receipt.json"])
    qc = json.loads(payloads["result_qc.json"])
    frames = json.loads(payloads["frame_manifest.json"])
    gauges = json.loads(payloads["native_gauge_manifest.json"])
    if execution.get("status") != "S5B0_PRIMARY_R7_EXECUTED_MOTION_REPLAY_PASS_DEVELOPMENT_ONLY" or execution.get("finalized") is not True or execution.get("attempt_id") != ATTEMPT:
        raise S6MaterializerV4Error("execution parent not finalized/exact")
    if qc.get("status") != "PASS_S5B0_REPLAY_RESULT_QC_V2" or qc.get("pass") is not True or qc.get("attempt_id") != ATTEMPT:
        raise S6MaterializerV4Error("QC parent differs")
    if frames.get("status") != "PASS_S5B0_FINALIZED_SOLVER_FRAMES_MANIFEST_V1" or frames.get("attempt_id") != ATTEMPT or not frames.get("integrity_pass"):
        raise S6MaterializerV4Error("frame manifest differs")
    files = gauges.get("files")
    if gauges.get("attempt_id") != ATTEMPT or not isinstance(files, list) or [row.get("probe_name") for row in files] != list(PROBES):
        raise S6MaterializerV4Error("native Gauge manifest differs")
    for row in files:
        relative = _safe_relative(row.get("relative_path", ""))
        if relative not in payloads or row.get("sha256") != sha256_bytes(payloads[relative]) or row.get("size_bytes") != len(payloads[relative]):
            raise S6MaterializerV4Error("native Gauge CSV identity differs")
    return payloads, identities


def materialize(*, s5a0_receipt: Path, s5a1_transfer_manifest: Path, primary_bag: Path,
                s5b0_package: Path, final_root: Path, external_ledger_path: Path,
                expected_previous_sha256: str, windows: Mapping[str, Mapping[str, float]]) -> dict[str, Any]:
    template, schema = load_contracts()
    parent_rows = []
    for path in (s5a0_receipt, s5a1_transfer_manifest, primary_bag):
        _, identity = read_exact_file(path)
        parent_rows.append(identity)
    payloads, inventory = scan_exact_package(s5b0_package)
    final_root, ledger = Path(final_root), Path(external_ledger_path)
    if not final_root.is_absolute() or not ledger.is_absolute() or len(expected_previous_sha256) != 64:
        raise S6MaterializerV4Error("runtime output/ledger identity invalid")
    if set(windows) != {"first15", "full_motion", "recorded_tail", "solver_tail"}:
        raise S6MaterializerV4Error("four frozen windows required")
    result = deepcopy(template)
    result["status"] = "ADMITTED_EXACT_FINALIZED_S5B0_PRIMARY_RUNTIME_MATERIALIZED_V4"
    result["parents"] = {"s5a0_receipt": parent_rows[0], "s5a1_transfer_manifest": parent_rows[1], "primary_bag": parent_rows[2], "s5b0_package": {"root": str(Path(s5b0_package)), "inventory_sha256": sha256_bytes(canonical_json(inventory)), "inventory": inventory, "required_parents": {name.replace(".json", "_sha256").replace("checksums.sha256", "checksums_sha256"): sha256_bytes(payloads[name]) for name in REQUIRED}}}
    result["runtime"].update({"final_root": str(final_root), "external_ledger_path": str(ledger), "external_ledger_expected_previous_sha256": expected_previous_sha256, "windows": {name: {"start_s": float(row["start_s"]), "end_s": float(row["end_s"])} for name, row in windows.items()}})
    result["authorization"] = {"exact_parents_materialized": True, "runtime_read_authorized": True, "create_new_publish_authorized": True}
    Draft202012Validator(schema).validate(result)
    return result


def static_receipt() -> dict[str, Any]:
    template, _ = load_contracts()
    return {"status": template["status"], "contract_sha256": sha256_bytes(canonical_json(template)), "parents_materialized": False, "files_written": False, "optional_bag_read": False, "solver_executed": False, "gpu_exposed": False, "stage6_pass": False}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        print(json.dumps(static_receipt(), sort_keys=True, separators=(",", ":")))
        return 0
    except Exception as exc:
        print(json.dumps({"status": "S6_REAL_DELIVERY_MATERIALIZER_V4_FAILED", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["S6MaterializerV4Error", "assert_deep_closed", "canonical_json", "load_contracts", "materialize", "read_exact_file", "scan_exact_package", "sha256_bytes", "static_receipt"]
