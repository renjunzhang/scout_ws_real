#!/usr/bin/env python3
"""Read-only QC v2 for one or two completed U3 DualSPHysics CPU output trees.

The tool never executes DualSPHysics and never writes into an input tree.  It
prints a machine-readable report to stdout.  The particle-surface quantity is
explicitly a development proxy (sector-wise q99 of fluid-particle Z), not a
DualSPHysics SWL gauge and not a physical liquid-level measurement.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import stat
import struct
import sys
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable

import r8_liquid_bi4_reader_v1 as bi4


SCHEMA_VERSION = "r8-liquid-u3-solver-output-qc-v2"
MAX_TEXT_BYTES = 8 * 1024 * 1024
MAX_PART_BYTES = 16 * 1024 * 1024
MAX_PART_FILES = 10_000
PART_RE = re.compile(r"Part_([0-9]{4})\.bi4")
PARTOUT_RE = re.compile(r"PartOut_([0-9]{3})\.obi4")


class QcError(ValueError):
    """Raised when an input tree is structurally unsafe or uninterpretable."""


@dataclass(frozen=True)
class Thresholds:
    minimum_settle_time_s: float = 3.0
    tail_window_s: float = 1.0
    physical_time_tolerance_s: float = 1.0e-6
    max_tail_speed_rms_m_s: float = 1.0e-3
    max_tail_speed_p95_m_s: float = 1.0e-3
    max_tail_speed_m_s: float = 5.0e-3
    max_tail_specific_kinetic_energy_j_kg: float = 5.0e-7
    max_surface_bias_dp: float = 0.5
    max_surface_spread_dp: float = 0.5
    max_surface_temporal_range_dp: float = 0.25
    max_surface_drift_dp_per_s: float = 0.05
    max_density_mean_relative_range: float = 0.005
    surface_sector_count: int = 16
    surface_quantile: float = 0.99
    surface_inner_margin_dp: float = 2.0
    minimum_surface_particles_per_sector: int = 4


def _finite_float(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise QcError(f"invalid float for {label}: {value!r}") from exc
    if not math.isfinite(result):
        raise QcError(f"non-finite float for {label}")
    return result


def _uint_text(value: str, label: str) -> int:
    cleaned = value.strip().replace(",", "")
    if not re.fullmatch(r"[0-9]+", cleaned):
        raise QcError(f"invalid unsigned integer for {label}: {value!r}")
    return int(cleaned)


def _required_uint(values: dict[str, Any], name: str) -> int:
    try:
        return bi4.require_int(values, name)
    except bi4.Bi4FormatError as exc:
        raise QcError(str(exc)) from exc


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _same_float(left: float, right: float) -> bool:
    return math.isclose(left, right, rel_tol=1.0e-12, abs_tol=1.0e-15)


def _float32_roundtrip(value: float, label: str) -> float:
    """Return the exact binary64 value produced by a binary32 round-trip."""

    finite = _finite_float(value, label)
    try:
        result = struct.unpack("<f", struct.pack("<f", finite))[0]
    except (OverflowError, struct.error) as exc:
        raise QcError(f"{label} cannot be represented as IEEE-754 binary32") from exc
    if not math.isfinite(result):
        raise QcError(f"{label} becomes non-finite as IEEE-754 binary32")
    return result


def _same_binary64_bits(left: float, right: float) -> bool:
    """Compare finite Python floats by their serialized binary64 bit pattern."""

    return struct.pack("<d", left) == struct.pack("<d", right)


def _require_directory(path: str | os.PathLike[str], label: str) -> Path:
    result = Path(path)
    try:
        metadata = os.lstat(result)
    except OSError as exc:
        raise QcError(f"cannot inspect {label} directory {result}: {exc}") from exc
    if not stat.S_ISDIR(metadata.st_mode):
        raise QcError(f"{label} is not a real directory: {result}")
    return result


def _read(path: Path, *, maximum: int, expected_sha256: str | None = None) -> bi4.SecureFile:
    try:
        return bi4.read_regular_file(
            path, expected_sha256=expected_sha256, max_bytes=maximum
        )
    except (bi4.Bi4FormatError, ValueError) as exc:
        raise QcError(str(exc)) from exc


def _quantile(values: Iterable[float], q: float) -> float:
    ordered = sorted(values)
    if not ordered:
        raise QcError("cannot calculate a quantile of an empty collection")
    position = (len(ordered) - 1) * q
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _median(values: Iterable[float]) -> float:
    return _quantile(values, 0.5)


def _slope(xs: list[float], ys: list[float]) -> float:
    if len(xs) < 2 or len(xs) != len(ys):
        return 0.0
    xmean = sum(xs) / len(xs)
    ymean = sum(ys) / len(ys)
    denominator = sum((value - xmean) ** 2 for value in xs)
    if denominator == 0:
        return 0.0
    return sum((x - xmean) * (y - ymean) for x, y in zip(xs, ys)) / denominator


def _xml_parameter(parameters: dict[str, str], key: str) -> float:
    if key not in parameters:
        raise QcError(f"case XML lacks required parameter {key}")
    return _finite_float(parameters[key], f"case XML parameter {key}")


def parse_case_xml(source: bi4.SecureFile) -> dict[str, Any]:
    if b"<!DOCTYPE" in source.data.upper() or b"<!ENTITY" in source.data.upper():
        raise QcError("case XML contains a prohibited DTD/entity declaration")
    try:
        root = ET.fromstring(source.data)
    except ET.ParseError as exc:
        raise QcError(f"invalid case XML: {exc}") from exc
    particles = root.find("./execution/particles")
    constants = root.find("./execution/constants")
    motion = root.find("./execution/motion")
    if particles is None or constants is None or motion is None:
        raise QcError("case XML lacks particles/constants/motion")
    if motion.attrib or list(motion) or (motion.text and motion.text.strip()):
        raise QcError("U3 case motion is not empty")
    fixed = particles.find("./fixed")
    fluid = particles.find("./fluid")
    if fixed is None or fluid is None:
        raise QcError("case XML lacks fixed/fluid particle blocks")
    parameters = {
        node.attrib.get("key", ""): node.attrib.get("value", "")
        for node in root.findall("./execution/parameters/parameter")
    }
    dp_node = constants.find("./dp")
    mass_node = constants.find("./massfluid")
    rhop0_node = constants.find("./rhop0")
    hswl_node = root.find("./casedef/constantsdef/hswl")
    if dp_node is None or mass_node is None or rhop0_node is None:
        raise QcError("case XML lacks dp/massfluid/rhop0")
    cylinders = root.findall("./casedef/geometry/commands/mainlist/drawcylinder")
    radius = None
    if cylinders and "radius" in cylinders[0].attrib:
        radius = _finite_float(cylinders[0].attrib["radius"], "fluid cylinder radius")
    domain_min = domain_max = None
    domain = root.find("./execution/parameters/simulationdomain")
    if domain is not None:
        pmin, pmax = domain.find("./posmin"), domain.find("./posmax")
        if pmin is not None and pmax is not None:
            domain_min = [_finite_float(pmin.attrib[a], f"domain min {a}") for a in "xyz"]
            domain_max = [_finite_float(pmax.attrib[a], f"domain max {a}") for a in "xyz"]
    return {
        "sha256": source.sha256,
        "size_bytes": source.size_bytes,
        "total": _uint_text(particles.attrib.get("np", ""), "particles.np"),
        "fixed": _uint_text(fixed.attrib.get("count", ""), "fixed.count"),
        "fluid": _uint_text(fluid.attrib.get("count", ""), "fluid.count"),
        "dp_m": _finite_float(dp_node.attrib.get("value"), "dp"),
        "massfluid_kg": _finite_float(mass_node.attrib.get("value"), "massfluid"),
        "rhop0_kg_m3": _finite_float(rhop0_node.attrib.get("value"), "rhop0"),
        "hswl_m": (
            _finite_float(hswl_node.attrib.get("value"), "hswl")
            if hswl_node is not None
            else None
        ),
        "time_max_xml_s": _xml_parameter(parameters, "TimeMax"),
        "time_out_xml_s": _xml_parameter(parameters, "TimeOut"),
        "min_fluid_stop": _xml_parameter(parameters, "MinFluidStop"),
        "rhop_out_min_kg_m3": _xml_parameter(parameters, "RhopOutMin"),
        "rhop_out_max_kg_m3": _xml_parameter(parameters, "RhopOutMax"),
        "fluid_radius_m": radius,
        "domain_min_m": domain_min,
        "domain_max_m": domain_max,
        "motion_empty": True,
    }


def load_fixed_case(
    case_bi4: str | os.PathLike[str],
    case_xml: str | os.PathLike[str],
    *,
    expected_bi4_sha256: str | None = None,
    expected_xml_sha256: str | None = None,
) -> dict[str, Any]:
    bi4_source = _read(Path(case_bi4), maximum=MAX_PART_BYTES, expected_sha256=expected_bi4_sha256)
    xml_source = _read(Path(case_xml), maximum=MAX_TEXT_BYTES, expected_sha256=expected_xml_sha256)
    try:
        root = bi4.parse_jpartdata_bi4(bi4_source.data)
        particles = bi4.extract_u3_particles(root)
    except bi4.Bi4FormatError as exc:
        raise QcError(f"invalid fixed case BI4: {exc}") from exc
    xml = parse_case_xml(xml_source)
    counts = particles["counts"]
    massfluid = _finite_float(root.values.get("MassFluid"), "case BI4 MassFluid")
    solver_massfluid = _float32_roundtrip(
        massfluid, "case BI4 MassFluid for solver serialization"
    )
    dp = _finite_float(root.values.get("Dp"), "case BI4 Dp")
    errors = []
    if xml["total"] != particles["particle_count"]:
        errors.append("XML/BI4 total particle count mismatch")
    if xml["fixed"] != counts["fixed_boundary"] or xml["fluid"] != counts["fluid"]:
        errors.append("XML/BI4 particle class mismatch")
    if counts["moving_boundary"] or counts["floating"]:
        errors.append("U3 fixed case unexpectedly has moving/floating particles")
    if not _same_float(dp, xml["dp_m"]) or not _same_float(massfluid, xml["massfluid_kg"]):
        errors.append("XML/BI4 dp or particle mass mismatch")
    if errors:
        raise QcError("; ".join(errors))
    if any(not math.isfinite(v) for vec in particles["positions_m"] for v in vec):
        raise QcError("fixed case contains non-finite positions")
    if any(not math.isfinite(v) for vec in particles["velocities_m_s"] for v in vec):
        raise QcError("fixed case contains non-finite velocities")
    return {
        "bi4": {
            "sha256": bi4_source.sha256,
            "size_bytes": bi4_source.size_bytes,
            "mode": oct(bi4_source.mode),
        },
        "xml": xml,
        "root_values": root.values,
        "particle_count": particles["particle_count"],
        "counts": counts,
        "dp_m": dp,
        "massfluid_kg": massfluid,
        "solver_effective_massfluid_kg": solver_massfluid,
        "initial_requested_case_fluid_mass_kg": counts["fluid"] * massfluid,
        "initial_solver_effective_fluid_mass_kg": counts["fluid"] * solver_massfluid,
        "positions_by_id": dict(zip(particles["ids"], particles["positions_m"])),
        "velocities_by_id": dict(zip(particles["ids"], particles["velocities_m_s"])),
        "densities_by_id": dict(zip(particles["ids"], particles["densities_kg_m3"])),
    }


def _part_arrays(root: bi4.JbdItem) -> tuple[bi4.JbdItem, list[int], list[Any], list[Any], list[float]]:
    if len(root.items) != 1:
        raise QcError(f"PART BI4 must contain one particle item, found {len(root.items)}")
    part = root.items[0]
    expected = {"Idp": 8, "Posd": 23, "Vel": 22, "Rhop": 11}
    for name, type_code in expected.items():
        array = part.arrays.get(name)
        if array is None or array.type_code != type_code:
            raise QcError(f"PART array {name} is missing or has the wrong type")
    try:
        ids = part.arrays["Idp"].records()
        positions = part.arrays["Posd"].records()
        velocities = part.arrays["Vel"].records()
        densities = part.arrays["Rhop"].records()
    except bi4.Bi4FormatError as exc:
        raise QcError(str(exc)) from exc
    if len({len(ids), len(positions), len(velocities), len(densities)}) != 1:
        raise QcError("PART particle arrays have inconsistent lengths")
    return part, ids, positions, velocities, densities


def _surface_proxy(
    fluid_positions: list[tuple[float, float, float]],
    case: dict[str, Any],
    thresholds: Thresholds,
) -> dict[str, Any]:
    sectors: list[list[float]] = [[] for _ in range(thresholds.surface_sector_count)]
    radius = case["xml"]["fluid_radius_m"]
    if radius is None:
        radius = max((math.hypot(p[0], p[1]) for p in fluid_positions), default=0.0)
    core_radius = radius - thresholds.surface_inner_margin_dp * case["dp_m"]
    if core_radius <= 0:
        core_radius = radius
    for x, y, z in fluid_positions:
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        if math.hypot(x, y) > core_radius:
            continue
        angle = math.atan2(y, x)
        if angle < 0:
            angle += 2 * math.pi
        index = min(
            thresholds.surface_sector_count - 1,
            int(angle * thresholds.surface_sector_count / (2 * math.pi)),
        )
        sectors[index].append(z)
    heights = [
        _quantile(values, thresholds.surface_quantile)
        for values in sectors
        if len(values) >= thresholds.minimum_surface_particles_per_sector
    ]
    valid = len(heights) == thresholds.surface_sector_count
    return {
        "method": "fluid_z_q99_by_azimuth_sector_development_proxy",
        "is_dualsphysics_swl_gauge": False,
        "sector_count": thresholds.surface_sector_count,
        "valid_sector_count": len(heights),
        "sector_particle_counts": [len(values) for values in sectors],
        "core_radius_m": core_radius,
        "valid": valid,
        "median_height_m": _median(heights) if heights else None,
        "spatial_spread_m": max(heights) - min(heights) if heights else None,
        "sector_heights_m": heights,
    }


def _root_case_invariants(
    values: dict[str, Any], case: dict[str, Any]
) -> dict[str, dict[str, Any]]:
    """Compare only the fixed case invariants carried into a PART root."""

    report: dict[str, dict[str, Any]] = {}
    count_fields = (
        ("CaseNp", case["particle_count"]),
        ("CaseNfixed", case["counts"]["fixed_boundary"]),
        ("CaseNmoving", 0),
        ("CaseNfloat", 0),
        ("CaseNfluid", case["counts"]["fluid"]),
    )
    for name, expected in count_fields:
        observed = _required_uint(values, name)
        report[name] = {
            "expected_case": expected,
            "expected_solver_serialized": expected,
            "observed": observed,
            "comparison_mode": "unsigned_integer_exact",
            "match": observed == expected,
        }

    expected_dp = case["dp_m"]
    observed_dp = _finite_float(values.get("Dp"), "PART root Dp")
    report["Dp"] = {
        "expected_case": expected_dp,
        "expected_solver_serialized": expected_dp,
        "observed": observed_dp,
        "comparison_mode": "binary64_rel_1e-12_abs_1e-15_retained_from_v1",
        "match": _same_float(observed_dp, expected_dp),
    }

    expected_mass = case["massfluid_kg"]
    expected_solver_mass = case["solver_effective_massfluid_kg"]
    observed_mass = _finite_float(values.get("MassFluid"), "PART root MassFluid")
    report["MassFluid"] = {
        "expected_case": expected_mass,
        "expected_solver_serialized": expected_solver_mass,
        "observed": observed_mass,
        "comparison_mode": (
            "case_binary64_to_ieee754_binary32_little_endian_roundtrip_"
            "then_binary64_bits_exact"
        ),
        "expected_solver_serialized_float32_le_hex": struct.pack(
            "<f", expected_mass
        ).hex(),
        "expected_solver_serialized_binary64_le_hex": struct.pack(
            "<d", expected_solver_mass
        ).hex(),
        "observed_binary64_le_hex": struct.pack("<d", observed_mass).hex(),
        "match": _same_binary64_bits(observed_mass, expected_solver_mass),
    }
    return report


def parse_part_file(
    source: bi4.SecureFile,
    filename_index: int,
    case: dict[str, Any],
    thresholds: Thresholds,
) -> dict[str, Any]:
    try:
        root = bi4.parse_jpartdata_bi4(source.data)
    except bi4.Bi4FormatError as exc:
        raise QcError(f"invalid PART BI4 {source.path.name}: {exc}") from exc
    part, ids, positions, velocities, densities = _part_arrays(root)
    cpart = _required_uint(part.values, "Cpart")
    npok = _required_uint(part.values, "Npok")
    nout = _required_uint(part.values, "Nout")
    step = _required_uint(part.values, "Step")
    timestep = _finite_float(part.values.get("TimeStep"), "PART TimeStep")
    if cpart != filename_index:
        raise QcError(f"PART Cpart {cpart} disagrees with filename index {filename_index}")
    if npok != len(ids):
        raise QcError("PART Npok disagrees with array length")
    expected_total = case["particle_count"]
    id_unique = len(set(ids)) == len(ids)
    id_in_range = all(0 <= value < expected_total for value in ids)
    missing_ids = sorted(set(range(expected_total)).difference(ids)) if id_unique else []
    fixed_limit = case["counts"]["fixed_boundary"]
    fluid_ids = [value for value in ids if value >= fixed_limit and value < expected_total]
    fixed_ids = [value for value in ids if value < fixed_limit]
    finite_pos = all(math.isfinite(value) for vector in positions for value in vector)
    finite_vel = all(math.isfinite(value) for vector in velocities for value in vector)
    finite_rho = all(math.isfinite(value) for value in densities)
    position_by_id = dict(zip(ids, positions))
    velocity_by_id = dict(zip(ids, velocities))
    density_by_id = dict(zip(ids, densities))
    fixed_positions_exact = id_unique and all(
        position_by_id.get(value) == case["positions_by_id"][value]
        for value in range(fixed_limit)
    )
    fixed_velocities_zero = id_unique and all(
        velocity_by_id.get(value) == (0.0, 0.0, 0.0)
        for value in range(fixed_limit)
    )
    initial_arrays_exact = id_unique and len(ids) == expected_total and all(
        position_by_id.get(value) == case["positions_by_id"][value]
        and velocity_by_id.get(value) == case["velocities_by_id"][value]
        and density_by_id.get(value) == case["densities_by_id"][value]
        for value in range(expected_total)
    )
    root_invariants = _root_case_invariants(root.values, case)
    root_invariants_match = all(item["match"] for item in root_invariants.values())
    fluid_indices = [index for index, value in enumerate(ids) if value >= fixed_limit]
    fluid_positions = [positions[index] for index in fluid_indices]
    fluid_velocities = [velocities[index] for index in fluid_indices]
    fluid_densities = [densities[index] for index in fluid_indices]
    finite_speeds = [
        math.sqrt(sum(component * component for component in vector))
        for vector in fluid_velocities
        if all(math.isfinite(component) for component in vector)
    ]
    speed_rms = (
        math.sqrt(sum(value * value for value in finite_speeds) / len(finite_speeds))
        if finite_speeds
        else None
    )
    finite_density_values = [value for value in fluid_densities if math.isfinite(value)]
    rhop_min = case["xml"]["rhop_out_min_kg_m3"]
    rhop_max = case["xml"]["rhop_out_max_kg_m3"]
    density_outside = sum(
        value < rhop_min or value > rhop_max for value in finite_density_values
    )
    domain_outside = 0
    dmin, dmax = case["xml"]["domain_min_m"], case["xml"]["domain_max_m"]
    if dmin is not None and dmax is not None:
        domain_outside = sum(
            any(value < dmin[axis] or value > dmax[axis] for axis, value in enumerate(pos))
            for pos in fluid_positions
            if all(math.isfinite(value) for value in pos)
        )
    fluid_count = len(fluid_ids)
    requested_case_fluid_mass = fluid_count * case["massfluid_kg"]
    solver_effective_fluid_mass = (
        fluid_count * case["solver_effective_massfluid_kg"]
    )
    initial_mass = case["initial_solver_effective_fluid_mass_kg"]
    return {
        "name": source.path.name,
        "sha256": source.sha256,
        "size_bytes": source.size_bytes,
        "cpart": cpart,
        "time_s": timestep,
        "step": step,
        "npok": npok,
        "nout": nout,
        "ids_unique": id_unique,
        "ids_in_range": id_in_range,
        "missing_id_count": len(missing_ids),
        "missing_ids_prefix": missing_ids[:32],
        "counts": {"fixed_boundary": len(fixed_ids), "fluid": fluid_count},
        "fluid_mass_kg": solver_effective_fluid_mass,
        "fluid_mass_requested_case_kg": requested_case_fluid_mass,
        "fluid_mass_solver_effective_kg": solver_effective_fluid_mass,
        "mass_relative_error": (
            solver_effective_fluid_mass - initial_mass
        ) / initial_mass,
        "finite": {"position": finite_pos, "velocity": finite_vel, "density": finite_rho},
        "fixed_positions_exact": fixed_positions_exact,
        "fixed_velocities_zero": fixed_velocities_zero,
        "matches_fixed_case_initial_arrays": initial_arrays_exact,
        "root_case_invariants_match": root_invariants_match,
        "root_case_invariants": root_invariants,
        "domain_outside_count": domain_outside,
        "density_outside_count": density_outside,
        "speed": {
            "rms_m_s": speed_rms,
            "p95_m_s": _quantile(finite_speeds, 0.95) if finite_speeds else None,
            "max_m_s": max(finite_speeds) if finite_speeds else None,
            "specific_kinetic_energy_j_kg": (
                0.5 * sum(value * value for value in finite_speeds) / len(finite_speeds)
                if finite_speeds
                else None
            ),
        },
        "density": {
            "min_kg_m3": min(finite_density_values) if finite_density_values else None,
            "p01_kg_m3": _quantile(finite_density_values, 0.01) if finite_density_values else None,
            "mean_kg_m3": (
                sum(finite_density_values) / len(finite_density_values)
                if finite_density_values
                else None
            ),
            "p99_kg_m3": _quantile(finite_density_values, 0.99) if finite_density_values else None,
            "max_kg_m3": max(finite_density_values) if finite_density_values else None,
        },
        "surface_proxy": _surface_proxy(fluid_positions, case, thresholds),
    }


def parse_partout(source: bi4.SecureFile) -> dict[str, Any]:
    data = source.data
    if len(data) < bi4.JBD_HEADER_SIZE + 4:
        raise QcError(f"PartOut file is too small: {source.path.name}")
    header = data[: bi4.JBD_HEADER_SIZE]
    try:
        title = header[:58].decode("ascii").rstrip(" ")
    except UnicodeDecodeError as exc:
        raise QcError("PartOut header is not ASCII") from exc
    if title != "#FileJBD JPartOutBi4" or header[58:60] != b"\n\0":
        raise QcError(f"invalid PartOut header: {source.path.name}")
    byte_order, si64, reserved2, reserved3 = header[60:64]
    if byte_order not in (0, 10) or si64 not in (0, 1):
        raise QcError("unsupported PartOut byte order/size mode")
    if bool(byte_order == 10) != bool(si64) or reserved2 or reserved3:
        raise QcError("inconsistent PartOut header flags")
    reader = bi4._Reader(data[bi4.JBD_HEADER_SIZE :])  # same frozen v1 codec
    items: list[bi4.JbdItem] = []
    try:
        while reader.remaining:
            items.append(bi4._read_item(reader, si64=bool(si64), depth=0))
    except bi4.Bi4FormatError as exc:
        raise QcError(f"invalid PartOut item stream: {exc}") from exc
    if not items or items[0].name != "JPartOutBi4":
        raise QcError("PartOut stream lacks the root item")
    appended = items[1:]
    nout_values = []
    for item in appended:
        if not re.fullmatch(r"PART_[0-9]{4}", item.name):
            raise QcError(f"unexpected appended PartOut item {item.name!r}")
        nout = _required_uint(item.values, "Nout")
        if nout <= 0:
            raise QcError("appended PartOut item has zero Nout")
        expected_types = {"Idp": 8, "Vel": 22, "Rhop": 11, "Motive": 4}
        for name, type_code in expected_types.items():
            if name not in item.arrays:
                raise QcError(f"PartOut item lacks {name}")
            if item.arrays[name].type_code != type_code:
                raise QcError(f"PartOut array {name} has the wrong type")
            if item.arrays[name].count != nout:
                raise QcError(f"PartOut array {name} count disagrees with Nout")
        if "Posd" not in item.arrays and "Pos" not in item.arrays:
            raise QcError("PartOut item lacks position data")
        if "Posd" in item.arrays and item.arrays["Posd"].type_code != 23:
            raise QcError("PartOut Posd array has the wrong type")
        if "Pos" in item.arrays and item.arrays["Pos"].type_code != 22:
            raise QcError("PartOut Pos array has the wrong type")
        nout_values.append(nout)
    return {
        "name": source.path.name,
        "sha256": source.sha256,
        "size_bytes": source.size_bytes,
        "appended_part_count": len(appended),
        "total_nout": sum(nout_values),
    }


def parse_run_csv(source: bi4.SecureFile) -> dict[str, Any]:
    try:
        text = source.data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise QcError("Run.csv is not UTF-8") from exc
    lines = [line for line in text.splitlines() if line.strip()]
    if len(lines) != 2:
        raise QcError("Run.csv must contain exactly one header and one data row")
    delimiter = ";" if lines[0].count(";") >= lines[0].count(",") else ","
    rows = list(csv.reader(lines, delimiter=delimiter))
    if len(rows[0]) != len(rows[1]) or len(set(rows[0])) != len(rows[0]):
        raise QcError("Run.csv columns are malformed or duplicated")
    values = dict(zip(rows[0], rows[1]))
    required = (
        "Np", "PhysicalTime", "PartFiles", "PartsOut", "Nbound", "Nfixed",
        "Dp", "H", "PartsOutRho", "PartsOutVel",
    )
    missing = [name for name in required if name not in values]
    if missing:
        raise QcError(f"Run.csv lacks columns: {', '.join(missing)}")
    parsed = {
        "Np": _uint_text(values["Np"], "Run.csv Np"),
        "PhysicalTime": _finite_float(values["PhysicalTime"], "Run.csv PhysicalTime"),
        "PartFiles": _uint_text(values["PartFiles"], "Run.csv PartFiles"),
        "PartsOut": _uint_text(values["PartsOut"], "Run.csv PartsOut"),
        "Nbound": _uint_text(values["Nbound"], "Run.csv Nbound"),
        "Nfixed": _uint_text(values["Nfixed"], "Run.csv Nfixed"),
        "Dp": _finite_float(values["Dp"], "Run.csv Dp"),
        "H": _finite_float(values["H"], "Run.csv H"),
        "PartsOutRho": _uint_text(values["PartsOutRho"], "Run.csv PartsOutRho"),
        "PartsOutVel": _uint_text(values["PartsOutVel"], "Run.csv PartsOutVel"),
    }
    for optional in ("Steps", "MaxParticles", "MaxCells"):
        if optional in values:
            parsed[optional] = _uint_text(values[optional], f"Run.csv {optional}")
    return {
        "sha256": source.sha256,
        "delimiter": delimiter,
        "physics_fields": parsed,
        "excluded_comparison_fields": [
            name for name in ("DateTime", "TSimul", "TSeg", "TTotal", "GPIPS") if name in values
        ],
    }


def parse_run_out(source: bi4.SecureFile) -> dict[str, Any]:
    try:
        text = source.data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise QcError("Run.out is not UTF-8") from exc
    lower = text.lower()
    completion = "Finished execution (code=0)." in text and "[Simulation finished" in text
    bad_patterns = {
        "exception": bool(re.search(r"\*\*\*\s*exception", text, re.IGNORECASE)),
        "warning": bool(re.search(r"\bwarning\b", text, re.IGNORECASE)),
        "nan": bool(re.search(r"(?<![A-Za-z])nan(?![A-Za-z])", lower)),
        "inf": bool(re.search(r"(?<![A-Za-z])[-+]?inf(?:inity)?(?![A-Za-z])", lower)),
        "minimum_fluid_stop": "minimum number of fluid particles" in lower,
    }
    excluded = re.search(r"Excluded particles\.*:\s*([0-9,]+)", text)
    excluded_count = _uint_text(excluded.group(1), "Run.out excluded particles") if excluded else None
    return {
        "sha256": source.sha256,
        "completion_markers_pass": completion,
        "bad_patterns": bad_patterns,
        "excluded_particles": excluded_count,
    }


def _tail_metrics(parts: list[dict[str, Any]], case: dict[str, Any], thresholds: Thresholds) -> dict[str, Any]:
    final_time = parts[-1]["time_s"]
    tail = [part for part in parts if part["time_s"] >= final_time - thresholds.tail_window_s - 1e-12]
    coverage = tail[-1]["time_s"] - tail[0]["time_s"] if len(tail) > 1 else 0.0
    speed_rms = [part["speed"]["rms_m_s"] for part in tail]
    speed_p95 = [part["speed"]["p95_m_s"] for part in tail]
    speed_max = [part["speed"]["max_m_s"] for part in tail]
    kinetic = [part["speed"]["specific_kinetic_energy_j_kg"] for part in tail]
    density_means = [part["density"]["mean_kg_m3"] for part in tail]
    valid_surface = [part for part in tail if part["surface_proxy"]["valid"]]
    surface_times = [part["time_s"] for part in valid_surface]
    surface_values = [part["surface_proxy"]["median_height_m"] for part in valid_surface]
    surface_spreads = [part["surface_proxy"]["spatial_spread_m"] for part in valid_surface]
    hswl = case["xml"]["hswl_m"]
    density_relative_range = (
        (max(density_means) - min(density_means)) / case["xml"]["rhop0_kg_m3"]
        if density_means and all(value is not None for value in density_means)
        else None
    )
    surface_mean = sum(surface_values) / len(surface_values) if surface_values else None
    return {
        "part_count": len(tail),
        "start_time_s": tail[0]["time_s"],
        "end_time_s": tail[-1]["time_s"],
        "coverage_s": coverage,
        "max_speed_rms_m_s": max(speed_rms) if speed_rms and None not in speed_rms else None,
        "max_speed_p95_m_s": max(speed_p95) if speed_p95 and None not in speed_p95 else None,
        "max_speed_m_s": max(speed_max) if speed_max and None not in speed_max else None,
        "max_specific_kinetic_energy_j_kg": max(kinetic) if kinetic and None not in kinetic else None,
        "density_mean_relative_range": density_relative_range,
        "surface_proxy": {
            "valid_part_count": len(valid_surface),
            "all_tail_parts_valid": len(valid_surface) == len(tail),
            "mean_height_m": surface_mean,
            "bias_from_hswl_m": surface_mean - hswl if surface_mean is not None and hswl is not None else None,
            "max_spatial_spread_m": max(surface_spreads) if surface_spreads else None,
            "temporal_range_m": max(surface_values) - min(surface_values) if surface_values else None,
            "linear_drift_m_s": _slope(surface_times, surface_values) if surface_values else None,
            "is_development_proxy": True,
        },
    }


def analyze_run(run_dir: str | os.PathLike[str], case: dict[str, Any], thresholds: Thresholds) -> dict[str, Any]:
    root = _require_directory(run_dir, "run")
    data_dir = _require_directory(root / "data", "run data")
    run_csv = parse_run_csv(_read(root / "Run.csv", maximum=MAX_TEXT_BYTES))
    run_out = parse_run_out(_read(root / "Run.out", maximum=MAX_TEXT_BYTES))
    part_paths: list[tuple[int, Path]] = []
    partout_paths: list[tuple[int, Path]] = []
    for path in data_dir.iterdir():
        match = PART_RE.fullmatch(path.name)
        if match:
            part_paths.append((int(match.group(1)), path))
        out_match = PARTOUT_RE.fullmatch(path.name)
        if out_match:
            partout_paths.append((int(out_match.group(1)), path))
    part_paths.sort()
    partout_paths.sort()
    if not part_paths or len(part_paths) > MAX_PART_FILES:
        raise QcError("PART file count is outside the allowed range")
    indices = [index for index, _path in part_paths]
    if indices != list(range(indices[-1] + 1)):
        raise QcError("PART filenames are not contiguous from Part_0000")
    if not partout_paths or [index for index, _ in partout_paths] != list(range(len(partout_paths))):
        raise QcError("PartOut filenames are not contiguous from PartOut_000")
    parts = [
        parse_part_file(_read(path, maximum=MAX_PART_BYTES), index, case, thresholds)
        for index, path in part_paths
    ]
    partouts = [parse_partout(_read(path, maximum=MAX_PART_BYTES)) for _index, path in partout_paths]
    times = [part["time_s"] for part in parts]
    steps = [part["step"] for part in parts]
    times_strict = all(right > left for left, right in zip(times, times[1:]))
    steps_monotonic = all(right >= left for left, right in zip(steps, steps[1:]))
    first_matches_case = (
        parts[0]["matches_fixed_case_initial_arrays"]
        and parts[0]["root_case_invariants_match"]
        and parts[0]["missing_id_count"] == 0
    )
    final_time = parts[-1]["time_s"]
    csv_fields = run_csv["physics_fields"]
    total_part_nout = sum(part["nout"] for part in parts)
    total_partout_nout = sum(item["total_nout"] for item in partouts)
    structural_checks = {
        "part_times_strictly_increasing": times_strict,
        "part_steps_monotonic": steps_monotonic,
        "first_snapshot_matches_fixed_case": first_matches_case,
        "all_particle_arrays_finite": all(all(part["finite"].values()) for part in parts),
        "all_ids_unique_and_in_range": all(part["ids_unique"] and part["ids_in_range"] for part in parts),
        "all_expected_ids_present": all(part["missing_id_count"] == 0 for part in parts),
        "class_counts_constant": all(
            part["counts"]["fixed_boundary"] == case["counts"]["fixed_boundary"]
            and part["counts"]["fluid"] == case["counts"]["fluid"]
            for part in parts
        ),
        "all_part_root_invariants_match_fixed_case": all(
            part["root_case_invariants_match"] for part in parts
        ),
        "fixed_boundary_unchanged": all(part["fixed_positions_exact"] and part["fixed_velocities_zero"] for part in parts),
        "zero_part_nout": all(part["nout"] == 0 for part in parts),
        "zero_partout_records": sum(item["total_nout"] for item in partouts) == 0,
        "zero_run_csv_partsout": csv_fields["PartsOut"] == csv_fields["PartsOutRho"] == csv_fields["PartsOutVel"] == 0,
        "zero_run_out_excluded": run_out["excluded_particles"] == 0,
        "exclusion_counts_cross_consistent": (
            total_part_nout
            == total_partout_nout
            == csv_fields["PartsOut"]
            == run_out["excluded_particles"]
        ),
        "mass_exact": all(part["mass_relative_error"] == 0 for part in parts),
        "positions_inside_domain": all(part["domain_outside_count"] == 0 for part in parts),
        "density_inside_xml_limits": all(part["density_outside_count"] == 0 for part in parts),
        "run_csv_identity_matches_case": (
            csv_fields["Np"] == case["particle_count"]
            and csv_fields["Nbound"] == case["counts"]["fixed_boundary"]
            and csv_fields["Nfixed"] == case["counts"]["fixed_boundary"]
            and _same_float(csv_fields["Dp"], case["dp_m"])
        ),
        "physical_time_consistent": abs(csv_fields["PhysicalTime"] - final_time) <= thresholds.physical_time_tolerance_s,
        "run_out_success_clean": run_out["completion_markers_pass"] and not any(run_out["bad_patterns"].values()),
    }
    tail = _tail_metrics(parts, case, thresholds)
    dp = case["dp_m"]
    surface = tail["surface_proxy"]
    tail_checks = {
        "tail_window_covered": tail["coverage_s"] >= thresholds.tail_window_s * 0.95,
        "speed_rms_below_threshold": tail["max_speed_rms_m_s"] is not None and tail["max_speed_rms_m_s"] <= thresholds.max_tail_speed_rms_m_s,
        "speed_p95_below_threshold": tail["max_speed_p95_m_s"] is not None and tail["max_speed_p95_m_s"] <= thresholds.max_tail_speed_p95_m_s,
        "speed_max_below_threshold": tail["max_speed_m_s"] is not None and tail["max_speed_m_s"] <= thresholds.max_tail_speed_m_s,
        "kinetic_energy_below_threshold": tail["max_specific_kinetic_energy_j_kg"] is not None and tail["max_specific_kinetic_energy_j_kg"] <= thresholds.max_tail_specific_kinetic_energy_j_kg,
        "surface_proxy_valid": surface["all_tail_parts_valid"],
        "surface_bias_below_threshold": surface["bias_from_hswl_m"] is not None and abs(surface["bias_from_hswl_m"]) <= thresholds.max_surface_bias_dp * dp,
        "surface_spread_below_threshold": surface["max_spatial_spread_m"] is not None and surface["max_spatial_spread_m"] <= thresholds.max_surface_spread_dp * dp,
        "surface_temporal_range_below_threshold": surface["temporal_range_m"] is not None and surface["temporal_range_m"] <= thresholds.max_surface_temporal_range_dp * dp,
        "surface_drift_below_threshold": surface["linear_drift_m_s"] is not None and abs(surface["linear_drift_m_s"]) <= thresholds.max_surface_drift_dp_per_s * dp,
        "density_mean_stable": tail["density_mean_relative_range"] is not None and tail["density_mean_relative_range"] <= thresholds.max_density_mean_relative_range,
    }
    duration_eligible = final_time + thresholds.physical_time_tolerance_s >= thresholds.minimum_settle_time_s
    structural_pass = all(structural_checks.values())
    tail_pass = all(tail_checks.values())
    if not structural_pass:
        status = "FAIL_U3_SOLVER_OUTPUT_QC"
    elif not duration_eligible:
        status = "PASS_U3_SOLVER_OUTPUT_SMOKE_ONLY" if tail_pass else "PASS_U3_SOLVER_RUNTIME_SMOKE_WITH_UNSETTLED_TAIL"
    elif not tail_pass:
        status = "FAIL_U3_DEVELOPMENT_SETTLE_TAIL_QC"
    else:
        status = "PASS_U3_DEVELOPMENT_SETTLE_SINGLE_OUTPUT_QC"
    return {
        "run_dir": str(root),
        "inventory": {
            "part_file_count": len(parts),
            "partout_file_count": len(partouts),
            "part_sha256": {part["name"]: part["sha256"] for part in parts},
            "partout_sha256": {item["name"]: item["sha256"] for item in partouts},
        },
        "run_csv": run_csv,
        "run_out": run_out,
        "parts": parts,
        "partout": partouts,
        "tail_metrics": tail,
        "checks": {"structural": structural_checks, "tail": tail_checks},
        "structural_pass": structural_pass,
        "tail_pass": tail_pass,
        "duration_eligible_for_settle_qc": duration_eligible,
        "short_duration_smoke": not duration_eligible,
        "numeric_settle_qc_pass": structural_pass and duration_eligible and tail_pass,
        "status": status,
    }


def compare_runs(first: dict[str, Any], second: dict[str, Any]) -> dict[str, Any]:
    first_parts = first["inventory"]["part_sha256"]
    second_parts = second["inventory"]["part_sha256"]
    first_out = first["inventory"]["partout_sha256"]
    second_out = second["inventory"]["partout_sha256"]
    physics_a = first["run_csv"]["physics_fields"]
    physics_b = second["run_csv"]["physics_fields"]
    return {
        "part_filename_sets_equal": list(first_parts) == list(second_parts),
        "all_part_sha256_exact": first_parts == second_parts,
        "partout_filename_sets_equal": list(first_out) == list(second_out),
        "all_partout_sha256_exact": first_out == second_out,
        "run_csv_physics_fields_exact": physics_a == physics_b,
        "excluded_run_csv_fields": ["DateTime", "TSimul", "TSeg", "TTotal", "GPIPS"],
    }


def build_report(
    run_dir: str | os.PathLike[str],
    case_bi4: str | os.PathLike[str],
    case_xml: str | os.PathLike[str],
    *,
    compare_run_dir: str | os.PathLike[str] | None = None,
    expected_case_bi4_sha256: str | None = None,
    expected_case_xml_sha256: str | None = None,
    thresholds: Thresholds | None = None,
) -> dict[str, Any]:
    limits = thresholds or Thresholds()
    case = load_fixed_case(
        case_bi4,
        case_xml,
        expected_bi4_sha256=expected_case_bi4_sha256,
        expected_xml_sha256=expected_case_xml_sha256,
    )
    first = analyze_run(run_dir, case, limits)
    second = analyze_run(compare_run_dir, case, limits) if compare_run_dir is not None else None
    comparison = compare_runs(first, second) if second is not None else None
    comparison_pass = comparison is not None and all(
        comparison[key]
        for key in (
            "part_filename_sets_equal", "all_part_sha256_exact",
            "partout_filename_sets_equal", "all_partout_sha256_exact",
            "run_csv_physics_fields_exact",
        )
    )
    two_run_numeric_pass = bool(
        second is not None
        and first["numeric_settle_qc_pass"]
        and second["numeric_settle_qc_pass"]
        and comparison_pass
    )
    any_hard_failure = not first["structural_pass"] or (
        second is not None and not second["structural_pass"]
    )
    if any_hard_failure:
        status = "FAIL_U3_SOLVER_OUTPUT_QC"
    elif second is None:
        status = first["status"]
    elif first["short_duration_smoke"] or second["short_duration_smoke"]:
        status = "PASS_U3_TWO_OUTPUTS_SMOKE_ONLY" if comparison_pass else "FAIL_U3_SMOKE_REPRODUCIBILITY_QC"
    elif two_run_numeric_pass:
        status = "PASS_U3_DEVELOPMENT_SETTLE_OUTPUT_QC_TWO_RUNS"
    else:
        status = "FAIL_U3_DEVELOPMENT_SETTLE_REPRODUCIBILITY_QC"
    return {
        "schema_version": SCHEMA_VERSION,
        "document_type": "SMPCC_R8_LIQUID_U3_SOLVER_OUTPUT_QC_V2",
        "thresholds": asdict(limits),
        "fixed_case": {
            "bi4": case["bi4"],
            "xml": case["xml"],
            "particle_count": case["particle_count"],
            "counts": case["counts"],
            "dp_m": case["dp_m"],
            "requested_case_massfluid_kg": case["massfluid_kg"],
            "solver_effective_massfluid_kg": case[
                "solver_effective_massfluid_kg"
            ],
            "initial_requested_case_fluid_mass_kg": case[
                "initial_requested_case_fluid_mass_kg"
            ],
            "initial_solver_effective_fluid_mass_kg": case[
                "initial_solver_effective_fluid_mass_kg"
            ],
            "massfluid_solver_serialization": {
                "mode": "ieee754_binary32_little_endian_roundtrip",
                "requested_case_float32_le_hex": struct.pack(
                    "<f", case["massfluid_kg"]
                ).hex(),
            },
        },
        "run": first,
        "comparison_run": second,
        "comparison": comparison,
        "verdict": {
            "status": status,
            "two_run_numeric_settle_qc_pass": two_run_numeric_pass,
            "development_only": True,
            "formal": False,
            "fidelity_validation_status": "UNVALIDATED",
            "physical_primary_eligible": False,
            "settled_state_claim_allowed": False,
            "settled_state_freeze_eligible": False,
            "reason_settled_state_not_frozen": [
                "surface metric is a particle q99 proxy, not an admitted SWL gauge",
                "offline output QC does not prove two distinct cold-start processes",
                "fresh-clone restart equivalence is not checked by this tool",
                "physical vessel/fluid inputs remain unvalidated",
            ],
        },
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--compare-run-dir")
    parser.add_argument("--case-bi4", required=True)
    parser.add_argument("--case-xml", required=True)
    parser.add_argument("--expected-case-bi4-sha256", required=True)
    parser.add_argument("--expected-case-xml-sha256", required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        report = build_report(
            args.run_dir,
            args.case_bi4,
            args.case_xml,
            compare_run_dir=args.compare_run_dir,
            expected_case_bi4_sha256=args.expected_case_bi4_sha256,
            expected_case_xml_sha256=args.expected_case_xml_sha256,
        )
    except QcError as exc:
        json.dump(
            {"schema_version": SCHEMA_VERSION, "status": "ERROR_UNSAFE_OR_INVALID_INPUT", "error": str(exc)},
            sys.stdout,
            ensure_ascii=False,
            sort_keys=True,
        )
        sys.stdout.write("\n")
        return 2
    json.dump(report, sys.stdout, ensure_ascii=False, indent=2, sort_keys=True)
    sys.stdout.write("\n")
    return 0 if not report["verdict"]["status"].startswith("FAIL_") else 1


if __name__ == "__main__":
    raise SystemExit(main())
