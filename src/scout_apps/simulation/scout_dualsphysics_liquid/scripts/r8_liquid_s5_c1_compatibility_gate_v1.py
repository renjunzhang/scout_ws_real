#!/usr/bin/env python3
"""Static, read-only gate for the Stage-5 C1-to-C1M replay contract."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import stat
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
CONTRACT = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s5_c1_compatibility_v1.json"
SCHEMA = ROOT / "schema/target_host_s5_c1_compatibility_v1.json"
MAX_BYTES = 16 * 1024 * 1024


class CompatibilityError(ValueError):
    """Raised when a parent, schema, geometry, mount, or particle invariant drifts."""


def read_regular(path: Path, maximum: int = MAX_BYTES) -> tuple[bytes, dict[str, Any]]:
    flags = os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC
    descriptor = os.open(path, flags)
    digest = hashlib.sha256()
    data = bytearray()
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise CompatibilityError(f"unsafe regular-file identity: {path}")
        if not 0 < metadata.st_size <= maximum:
            raise CompatibilityError(f"file size outside 1..{maximum}: {path}")
        remaining = metadata.st_size
        while remaining:
            block = os.read(descriptor, min(remaining, 1024 * 1024))
            if not block:
                raise CompatibilityError(f"short read: {path}")
            digest.update(block)
            data.extend(block)
            remaining -= len(block)
    finally:
        os.close(descriptor)
    return bytes(data), {
        "path": str(path),
        "sha256": digest.hexdigest(),
        "size_bytes": len(data),
        "mode": f"{stat.S_IMODE(metadata.st_mode):04o}",
        "inode": metadata.st_ino,
        "nlink": metadata.st_nlink,
    }


def read_json(path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    raw, identity = read_regular(path)
    try:
        value = json.loads(raw.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise CompatibilityError(f"invalid JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise CompatibilityError(f"JSON root differs: {path}")
    return value, identity


def assert_deep_closed(value: Any, path: str = "$") -> None:
    if isinstance(value, dict):
        kind = value.get("type")
        if kind == "object" or (isinstance(kind, list) and "object" in kind):
            if value.get("additionalProperties") is not False:
                raise CompatibilityError(f"schema object is not closed at {path}")
        for key, item in value.items():
            assert_deep_closed(item, f"{path}/{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            assert_deep_closed(item, f"{path}/{index}")


def _float(value: str | None, label: str) -> float:
    try:
        return float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError) as exc:
        raise CompatibilityError(f"invalid XML numeric field: {label}") from exc


def validate_c1_yaml(raw: bytes, contract: dict[str, Any]) -> None:
    text = raw.decode("utf-8")
    required = (
        "sim_container_condition: C1",
        "sim_container_config_id: SIM_ONLY_C1_D37_H58_UNVALIDATED",
        "formal: false",
        "physical_primary_eligible: false",
        "container_radius: 0.0185",
        "liquid_height: 0.058",
        "liquid_density: 1000.0",
    )
    if any(text.count(token) != 1 for token in required):
        raise CompatibilityError("R7 C1 YAML semantic token differs")
    expected = contract["r7_c1_model"]
    if expected["formal"] or expected["physical_primary_eligible"]:
        raise CompatibilityError("R7 C1 claim ceiling drifted")


def validate_mount_sources(launch_raw: bytes, runner_raw: bytes, contract: dict[str, Any]) -> None:
    launch = launch_raw.decode("utf-8")
    if launch.count('<arg name="offset_x" default="0.0"/>') != 1:
        raise CompatibilityError("R7 H_proxy offset_x default differs")
    if launch.count('<arg name="offset_y" default="0.0"/>') != 1:
        raise CompatibilityError("R7 H_proxy offset_y default differs")
    runner = runner_raw.decode("utf-8")
    start = runner.find("def _hproxy_command(")
    end = runner.find("\ndef ", start + 1)
    if start < 0 or end < 0:
        raise CompatibilityError("R7 runner H_proxy function is missing")
    function = runner[start:end]
    if "smpcc_sim_h_proxy_monitor.launch" not in function:
        raise CompatibilityError("R7 runner H_proxy launch differs")
    if "offset_x:=" in function or "offset_y:=" in function:
        raise CompatibilityError("R7 runner overrides the frozen zero-offset convention")
    mount = contract["mount_contract"]
    if mount["translation_m"] != [0.0, 0.0, 0.0] or mount["quaternion_xyzw"] != [0.0, 0.0, 0.0, 1.0]:
        raise CompatibilityError("development mount is not the frozen identity transform")
    if mount["physical_mount_validated"] or not mount["development_assumption"]:
        raise CompatibilityError("mount evidence was promoted beyond the partial source")


def validate_xml(raw: bytes, contract: dict[str, Any]) -> dict[str, Any]:
    try:
        root = ET.fromstring(raw)
    except ET.ParseError as exc:
        raise CompatibilityError(f"C1M XML parse failure: {exc}") from exc
    definition = root.find("./casedef/geometry/definition")
    if definition is None or _float(definition.get("dp"), "definition.dp") != 0.002:
        raise CompatibilityError("C1M particle spacing differs")
    cylinders = root.findall("./casedef/geometry/commands/mainlist/drawcylinder")
    if len(cylinders) != 2:
        raise CompatibilityError("C1M cylinder count differs")
    fluid, boundary = cylinders
    if fluid.attrib != {"radius": "0.0185"} or boundary.attrib != {"radius": "0.0185", "mask": "2"}:
        raise CompatibilityError("C1M cylinder definitions differ")
    fluid_points = fluid.findall("point")
    boundary_points = boundary.findall("point")
    if [point.attrib for point in fluid_points] != [
        {"x": "0", "y": "0", "z": "0"}, {"x": "0", "y": "0", "z": "0.058"}
    ]:
        raise CompatibilityError("C1M fluid cylinder extent differs")
    if [point.attrib for point in boundary_points] != [
        {"x": "0", "y": "0", "z": "0"}, {"x": "0", "y": "0", "z": "0.066"}
    ]:
        raise CompatibilityError("C1M boundary cylinder extent differs")
    gravity = root.find("./casedef/constantsdef/gravity")
    density = root.find("./casedef/constantsdef/rhop0")
    if gravity is None or density is None:
        raise CompatibilityError("C1M constants are missing")
    gravity_value = [_float(gravity.get(axis), f"gravity.{axis}") for axis in ("x", "y", "z")]
    if gravity_value != [0.0, 0.0, -9.81] or _float(density.get("value"), "rhop0") != 1000.0:
        raise CompatibilityError("C1M density/gravity differs")
    particles = root.find("./execution/particles")
    moving = root.find("./execution/particles/moving")
    liquid = root.find("./execution/particles/fluid")
    if particles is None or moving is None or liquid is None:
        raise CompatibilityError("C1M particle metadata is missing")
    observed = {
        "particle_count": int(particles.get("np", "-1")),
        "moving_boundary_count": int(moving.get("count", "-1")),
        "moving_boundary_id_first": int(moving.get("begin", "-1")),
        "fluid_count": int(liquid.get("count", "-1")),
        "fluid_id_first": int(liquid.get("begin", "-1")),
    }
    expected = contract["replay_c1m"]
    for key, value in observed.items():
        if expected[key] != value:
            raise CompatibilityError(f"C1M XML particle field differs: {key}")
    motions = root.findall(".//motion/objreal")
    if len(motions) != 2 or any(item.attrib != {"ref": "0"} for item in motions):
        raise CompatibilityError("C1M motion reference differs")
    if any(len(item.findall("mvnull")) != 1 for item in motions):
        raise CompatibilityError("C1M zero-motion seed differs")
    return observed


def load_bi4_reader(path: Path):
    spec = importlib.util.spec_from_file_location("r8_liquid_s5_c1_bi4_reader_v1", path)
    if spec is None or spec.loader is None:
        raise CompatibilityError("cannot load the frozen BI4 reader")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _particle_map(module: Any, path: Path, expected_sha256: str) -> tuple[Any, dict[int, tuple[float, float, float]]]:
    secure = module.read_regular_file(path, expected_sha256=expected_sha256)
    root = module.parse_jpartdata_bi4(secure.data)
    if len(root.items) != 1:
        raise CompatibilityError("BI4 particle item count differs")
    part = root.items[0]
    ids = part.arrays["Idp"].records()
    positions = part.arrays["Posd"].records()
    if len(ids) != len(positions) or len(set(ids)) != len(ids):
        raise CompatibilityError("BI4 particle ID set is invalid")
    mapping = dict(zip(ids, positions))
    return root, mapping


def _moving_position_hash(positions: dict[int, tuple[float, float, float]], count: int) -> str:
    digest = hashlib.sha256()
    for particle_id in range(count):
        if particle_id not in positions:
            raise CompatibilityError("moving-boundary particle ID is missing")
        digest.update(struct.pack("<Iddd", particle_id, *positions[particle_id]))
    return digest.hexdigest()


def validate_particles(contract: dict[str, Any]) -> dict[str, Any]:
    parents = contract["parents"]
    module = load_bi4_reader(Path(parents["bi4_reader"]["path"]))
    base_root, base_positions = _particle_map(
        module, Path(parents["c1m_case_bi4"]["path"]), parents["c1m_case_bi4"]["sha256"]
    )
    settled_root, settled_positions = _particle_map(
        module, Path(parents["settled_checkpoint"]["path"]), parents["settled_checkpoint"]["sha256"]
    )
    expected = contract["replay_c1m"]
    wanted_ids = set(range(expected["particle_count"]))
    if set(base_positions) != wanted_ids or set(settled_positions) != wanted_ids:
        raise CompatibilityError("C1M/settled particle ID sets differ")
    for root, label in ((base_root, "base"), (settled_root, "settled")):
        values = root.values
        counts = (values.get("CaseNp"), values.get("CaseNmoving"), values.get("CaseNfluid"))
        if counts != (9078, 2669, 6409):
            raise CompatibilityError(f"{label} particle classes differ")
        if float(values.get("Dp", -1.0)) != 0.002 or float(values.get("Rhop0", -1.0)) != 1000.0:
            raise CompatibilityError(f"{label} dp/density differs")
    moving_count = expected["moving_boundary_count"]
    base_hash = _moving_position_hash(base_positions, moving_count)
    settled_hash = _moving_position_hash(settled_positions, moving_count)
    if base_hash != expected["moving_boundary_position_sha256"] or settled_hash != base_hash:
        raise CompatibilityError("moving-boundary positions drifted through settle")
    settled_part = settled_root.items[0].values
    if settled_part.get("Cpart") != 901 or settled_part.get("TimeStep") != expected["settled_time_s"]:
        raise CompatibilityError("settled Part_0901 identity differs")
    if settled_part.get("Nout") != 0:
        raise CompatibilityError("settled checkpoint reports output particles")
    return {"particle_ids": 9078, "moving_boundary_position_sha256": base_hash}


def self_check() -> dict[str, Any]:
    contract, contract_identity = read_json(CONTRACT)
    schema, schema_identity = read_json(SCHEMA)
    Draft202012Validator.check_schema(schema)
    assert_deep_closed(schema)
    errors = sorted(Draft202012Validator(schema).iter_errors(contract), key=lambda item: list(item.absolute_path))
    if errors:
        first = errors[0]
        raise CompatibilityError(f"contract schema failure at {list(first.absolute_path)}: {first.message}")
    observed: dict[str, dict[str, Any]] = {}
    raw: dict[str, bytes] = {}
    for name, expected in contract["parents"].items():
        content, identity = read_regular(Path(expected["path"]))
        if identity["sha256"] != expected["sha256"]:
            raise CompatibilityError(f"parent hash drift: {name}")
        observed[name] = identity
        raw[name] = content
    validate_c1_yaml(raw["r7_c1_model_yaml"], contract)
    validate_mount_sources(raw["r7_hproxy_launch"], raw["r7_runner_source"], contract)
    xml_fields = validate_xml(raw["c1m_case_xml"], contract)
    settled = json.loads(raw["settled_receipt"])
    if settled.get("verdict", {}).get("status") != "U3_SETTLED_STATE_FROZEN":
        raise CompatibilityError("settled receipt status differs")
    file_ref = settled.get("settled_checkpoint", {}).get("file", {})
    if file_ref.get("path") != contract["parents"]["settled_checkpoint"]["path"] or file_ref.get("sha256") != contract["parents"]["settled_checkpoint"]["sha256"]:
        raise CompatibilityError("settled receipt does not bind the exact checkpoint")
    if settled.get("settled_checkpoint", {}).get("case_bi4_sha256") != contract["parents"]["c1m_case_bi4"]["sha256"]:
        raise CompatibilityError("settled receipt does not bind the exact C1M case")
    particles = validate_particles(contract)
    if contract["claim_ceiling"]["gpu_replay_authorized"]:
        raise CompatibilityError("compatibility contract improperly authorizes GPU replay")
    return {
        "status": "PASS_S5_C1_TO_C1M_DEVELOPMENT_REPLAY_COMPATIBILITY_V1",
        "contract": contract_identity,
        "schema": schema_identity,
        "verified_parent_count": len(observed),
        "xml_particle_fields": xml_fields,
        "particle_evidence": particles,
        "mount_basis": contract["mount_contract"]["basis"],
        "source_provenance": "PARTIAL_BAG_ONLY",
        "source_outcome": "UNKNOWN",
        "gpu_replay_authorized": False,
        "solver_executed": False,
        "gpu_exposed": False,
        "network_used": False,
        "files_modified": False,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        result = self_check()
    except (CompatibilityError, OSError, ValueError, KeyError) as exc:
        print(json.dumps({"status": "FAIL_S5_C1_COMPATIBILITY_V1", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
