#!/usr/bin/env python3
"""Deterministically render the frozen U3 Stage-4 synthetic-motion XMLs.

Only the two identical ``motion`` blocks in the sealed generated C1M XML are
replaced.  All other bytes remain identical to the frozen base case.  Output
is create-new (O_EXCL), regular, non-symlink and mode 0640; this tool never
executes DualSPHysics, exposes a GPU, or writes outside the case directory.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import stat
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_stage4_synthetic_motion_contract_v1.json"
CONTRACT_PATH = PACKAGE_ROOT / "config/target_hosts/liquid_zrj_msi_u2404_u3_stage4_synthetic_motion_contract_20260811T135543Z_v20.json"
CASE_ROOT = PACKAGE_ROOT / "config/cases"
BASE_CASE_PATH = Path("/home/zrj/scout_liquid_lab/cases/u3_c1m_gencase_v8_20260808T153753Z.partial/output/C1M_zero.xml")
MAX_JSON_BYTES = 4 * 1024 * 1024
MAX_XML_BYTES = 1024 * 1024
BASE_MOTION_BLOCK = b"""        <motion>
            <objreal ref="0">
                <begin mov="1" start="0" />
                <mvnull id="1" />
            </objreal>
        </motion>"""

EXPECTED_CASES = {
    "zero": {
        "case_id": "SETTLED_ZERO_REPLAY",
        "run_id": "u3_c1m_gpu_settled_zero_replay_20260811T135543Z_v20",
        "output_xml": str(CASE_ROOT / "u3_c1m_settled_zero_replay_20260811T135543Z_v20.xml"),
        "family": "SETTLED_REPLAY_ZERO",
        "motion_kind": "ZERO",
        "start_time_s": 45.05001991890928,
        "active_duration_s": 0.0,
        "tail_duration_s": 1.0,
        "end_time_s": 46.05001991890928,
        "frequency_hz": 0.0,
        "phase_rad": 0.0,
        "translation_axis": "NONE",
        "translation_amplitude_m": 0.0,
        "yaw_axis": "NONE",
        "yaw_amplitude_deg": 0.0,
        "part_first": 901,
        "part_last": 921,
        "part_count": 21,
        "maximum_output_bytes": 16777216,
    },
    "translation": {
        "case_id": "SYNTHETIC_TRANSLATION_X",
        "run_id": "u3_c1m_gpu_synthetic_translation_20260811T135543Z_v21",
        "output_xml": str(CASE_ROOT / "u3_c1m_synthetic_translation_20260811T135543Z_v21.xml"),
        "family": "SYNTHETIC_TRANSLATION",
        "motion_kind": "TRANSLATION",
        "start_time_s": 45.05001991890928,
        "active_duration_s": 2.0,
        "tail_duration_s": 1.0,
        "end_time_s": 48.05001991890928,
        "frequency_hz": 1.0,
        "phase_rad": 0.0,
        "translation_axis": "X",
        "translation_amplitude_m": 0.002,
        "yaw_axis": "NONE",
        "yaw_amplitude_deg": 0.0,
        "part_first": 901,
        "part_last": 961,
        "part_count": 61,
        "maximum_output_bytes": 33554432,
    },
    "yaw": {
        "case_id": "SYNTHETIC_YAW_Z",
        "run_id": "u3_c1m_gpu_synthetic_yaw_20260811T135543Z_v22",
        "output_xml": str(CASE_ROOT / "u3_c1m_synthetic_yaw_20260811T135543Z_v22.xml"),
        "family": "SYNTHETIC_YAW",
        "motion_kind": "YAW",
        "start_time_s": 45.05001991890928,
        "active_duration_s": 2.0,
        "tail_duration_s": 1.0,
        "end_time_s": 48.05001991890928,
        "frequency_hz": 1.0,
        "phase_rad": 0.0,
        "translation_axis": "NONE",
        "translation_amplitude_m": 0.0,
        "yaw_axis": "Z",
        "yaw_amplitude_deg": 2.0,
        "part_first": 901,
        "part_last": 961,
        "part_count": 61,
        "maximum_output_bytes": 33554432,
    },
}


class MotionCaseError(ValueError):
    """Drifted parent, invalid contract/XML, collision, or unsafe output."""


def _number(value: float) -> str:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise MotionCaseError("motion numeric value is invalid")
    return json.dumps(value, allow_nan=False, separators=(",", ":"))


def _sha256(path: Path, maximum: int) -> str:
    metadata = os.lstat(path)
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
        raise MotionCaseError(f"unsafe regular-file identity: {path}")
    if not 1 <= metadata.st_size <= maximum:
        raise MotionCaseError(f"file size is outside the bound: {path}")
    digest = hashlib.sha256()
    with path.open("rb", buffering=0) as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def identity(path: Path, maximum: int = MAX_XML_BYTES) -> dict[str, Any]:
    digest = _sha256(path, maximum)
    metadata = os.lstat(path)
    return {
        "path": str(path),
        "sha256": digest,
        "size_bytes": metadata.st_size,
        "mode": f"{stat.S_IMODE(metadata.st_mode):04o}",
        "inode": metadata.st_ino,
        "nlink": metadata.st_nlink,
    }


def _read_json(path: Path) -> dict[str, Any]:
    source = path.read_bytes()
    if not 1 <= len(source) <= MAX_JSON_BYTES:
        raise MotionCaseError(f"JSON size differs: {path}")
    value = json.loads(source)
    if not isinstance(value, dict):
        raise MotionCaseError(f"JSON root is not an object: {path}")
    return value


def semantic_validate(contract: dict[str, Any], contract_path: Path) -> None:
    if contract_path.resolve(strict=True) != CONTRACT_PATH or contract_path != CONTRACT_PATH:
        raise MotionCaseError("contract path differs from the frozen exact path")
    if contract["contract_id"] != "u3_c1m_stage4_synthetic_motion_20260811T135543Z_v20":
        raise MotionCaseError("contract identity differs")
    if contract["cases"] != EXPECTED_CASES:
        raise MotionCaseError("one or more exact motion-case fields differ")
    fixed = contract["fixed_numerics"]
    if fixed != {
        "dp_m": 0.002,
        "cfl": 0.1,
        "shifting": "None",
        "viscosity_model": "ARTIFICIAL",
        "viscoart": 0.3,
        "output_period_s": 0.05,
        "coordinate_frame": "CASE_XYZ_Z_UP_CONTAINER_BODY",
        "container_axis": "Z",
        "container_radius_m": 0.0185,
        "nominal_surface_z_m": 0.058,
    }:
        raise MotionCaseError("fixed numerical/geometry fields differ")
    checkpoint = contract["checkpoint"]
    if (
        checkpoint["part_index"] != 901
        or checkpoint["part_first"] != 901
        or checkpoint["time_s"] != 45.05001991890928
        or checkpoint["particle_count"] != 9078
        or checkpoint["moving_count"] != 2669
        or checkpoint["fluid_count"] != 6409
    ):
        raise MotionCaseError("settled checkpoint semantics differ")
    numerical = _read_json(Path(contract["parents"]["numerical_contract"]["path"]))
    synthetic = numerical.get("synthetic_contract")
    if synthetic != {
        "translation": {"axis": "x", "amplitude_m": 0.002, "frequency_hz": 1.0, "excitation_duration_s": 2.0, "tail_duration_s": 1.0},
        "yaw": {"axis": "z", "amplitude_degrees": 2.0, "frequency_hz": 1.0, "excitation_duration_s": 2.0, "tail_duration_s": 1.0},
        "output_period_s": 0.05,
        "minimum_azimuth_sectors": 16,
        "minimum_valid_fraction": 0.95,
        "zero_nout_required": True,
        "finite_required": True,
    }:
        raise MotionCaseError("parent numerical synthetic contract differs")
    expected_surface = {
        "method": "FLUID_PARTICLE_Z_QUANTILE_PER_CONTAINER_FRAME_AZIMUTH_SECTOR",
        "sector_count": 16,
        "annulus_inner_radius_fraction": 0.45,
        "annulus_outer_radius_fraction": 0.95,
        "surface_quantile": 0.98,
        "minimum_particles_per_sector": 128,
        "minimum_valid_fraction": 0.95,
        "frame": "INVERSE_ACTUAL_BOUNDARY_RIGID_TRANSFORM_CONTAINER_BODY",
        "center_radius_fraction": 0.25,
        "mid_annulus_inner_fraction": 0.25,
        "mid_annulus_outer_fraction": 0.55,
    }
    if contract["surface_proxy"] != expected_surface:
        raise MotionCaseError("surface-proxy contract differs")
    acceptance = contract["acceptance"]
    if acceptance["translation_surface_first_harmonic_min_m"] != 0.00001:
        raise MotionCaseError("translation response threshold differs")
    if acceptance["free_decay_final_to_initial_peak_ke_ratio_max"] != 0.95:
        raise MotionCaseError("free-decay threshold differs")
    if contract["execution_boundary"] != {
        "runner_revision": "r8_liquid_u3_stage4_experiment_runner_v1",
        "backend": "GPU",
        "gpu_index": 0,
        "parallel_runs": 1,
        "network": False,
        "sudo": False,
        "apparmor": False,
        "workspace_write_bind_count": 0,
        "candidate_source_executed": False,
        "combined_motion_authorized": False,
        "stage5_authorized": False,
        "formal_physical_claim_authorized": False,
    }:
        raise MotionCaseError("execution boundary differs")


def load_contract(path: Path = CONTRACT_PATH) -> dict[str, Any]:
    schema = _read_json(SCHEMA_PATH)
    contract = _read_json(path)
    Draft202012Validator.check_schema(schema)
    errors = sorted(Draft202012Validator(schema).iter_errors(contract), key=lambda error: list(error.absolute_path))
    if errors:
        first = errors[0]
        raise MotionCaseError(f"contract schema failure at {list(first.absolute_path)}: {first.message}")
    semantic_validate(contract, path)
    return contract


def _verify_parents(contract: dict[str, Any]) -> dict[str, Any]:
    verified: dict[str, Any] = {}
    for name, spec in contract["parents"].items():
        maximum = MAX_XML_BYTES if name in {"base_case_xml", "motion_parser_source", "motion_writer_source"} else MAX_JSON_BYTES
        observed = identity(Path(spec["path"]), maximum)
        if observed["sha256"] != spec["sha256"]:
            raise MotionCaseError(f"parent hash differs: {name}")
        for key in ("size_bytes", "mode", "inode", "nlink"):
            if key in spec and observed[key] != spec[key]:
                raise MotionCaseError(f"parent identity differs: {name}.{key}")
        verified[name] = observed
    for name in ("part", "head"):
        spec = contract["checkpoint"][name]
        observed = identity(Path(spec["path"]), MAX_XML_BYTES)
        if any(observed[key] != spec[key] for key in ("sha256", "size_bytes", "mode", "inode", "nlink")):
            raise MotionCaseError(f"checkpoint identity differs: {name}")
        verified[f"checkpoint_{name}"] = observed
    settled = _read_json(Path(contract["parents"]["settled_qc"]["path"]))
    if settled.get("verdict", {}).get("status") != "U3_SETTLED_STATE_FROZEN":
        raise MotionCaseError("settled-state parent is not frozen")
    parity = _read_json(Path(contract["parents"]["backend_parity_qc"]["path"]))
    if parity.get("verdict", {}).get("status") != "PASS_CPU_GPU_PARITY_DEVELOPMENT_ONLY":
        raise MotionCaseError("CPU/GPU parity parent is not PASS")
    return verified


def motion_block(case: dict[str, Any]) -> bytes:
    start = _number(case["start_time_s"])
    if case["motion_kind"] == "ZERO":
        lines = [
            "        <motion>",
            "            <objreal ref=\"0\">",
            f"                <begin mov=\"1\" start=\"{start}\" />",
            "                <mvnull id=\"1\" />",
            "            </objreal>",
            "        </motion>",
        ]
    elif case["motion_kind"] == "TRANSLATION":
        lines = [
            "        <motion>",
            "            <objreal ref=\"0\">",
            f"                <begin mov=\"1\" start=\"{start}\" />",
            f"                <mvrectsinu id=\"1\" duration=\"{_number(case['active_duration_s'])}\" next=\"2\" anglesunits=\"radians\">",
            f"                    <freq x=\"{_number(case['frequency_hz'])}\" y=\"0\" z=\"0\" units_comment=\"1/s\" />",
            f"                    <ampl x=\"{_number(case['translation_amplitude_m'])}\" y=\"0\" z=\"0\" units_comment=\"metres (m)\" />",
            f"                    <phase x=\"{_number(case['phase_rad'])}\" y=\"0\" z=\"0\" units_comment=\"radians\" />",
            "                </mvrectsinu>",
            "                <mvnull id=\"2\" />",
            "            </objreal>",
            "        </motion>",
        ]
    elif case["motion_kind"] == "YAW":
        phase_deg = math.degrees(case["phase_rad"])
        lines = [
            "        <motion>",
            "            <objreal ref=\"0\">",
            f"                <begin mov=\"1\" start=\"{start}\" />",
            f"                <mvrotsinu id=\"1\" duration=\"{_number(case['active_duration_s'])}\" next=\"2\" anglesunits=\"degrees\">",
            f"                    <freq v=\"{_number(case['frequency_hz'])}\" units_comment=\"1/s\" />",
            f"                    <ampl v=\"{_number(case['yaw_amplitude_deg'])}\" units_comment=\"degrees\" />",
            f"                    <phase v=\"{_number(phase_deg)}\" units_comment=\"degrees\" />",
            "                    <axisp1 x=\"0\" y=\"0\" z=\"0\" />",
            "                    <axisp2 x=\"0\" y=\"0\" z=\"1\" />",
            "                </mvrotsinu>",
            "                <mvnull id=\"2\" />",
            "            </objreal>",
            "        </motion>",
        ]
    else:
        raise MotionCaseError("unsupported motion kind")
    return "\n".join(lines).encode("utf-8")


def _motion_elements(source: bytes) -> list[ET.Element]:
    try:
        root = ET.fromstring(source)
    except ET.ParseError as exc:
        raise MotionCaseError(f"rendered XML is invalid: {exc}") from exc
    elements = root.findall("./casedef/motion") + root.findall("./execution/motion")
    if len(elements) != 2:
        raise MotionCaseError("rendered XML does not contain two motion blocks")
    return elements


def validate_rendered(source: bytes, case: dict[str, Any], base: bytes) -> None:
    block = motion_block(case)
    if source.count(block) != 2 or source.replace(block, BASE_MOTION_BLOCK) != base:
        raise MotionCaseError("bytes outside the two motion blocks differ")
    for motion in _motion_elements(source):
        obj = motion.find("objreal")
        if obj is None or obj.attrib != {"ref": "0"}:
            raise MotionCaseError("motion object identity differs")
        children = list(obj)
        if not children or children[0].tag != "begin" or children[0].attrib != {
            "mov": "1",
            "start": _number(case["start_time_s"]),
        }:
            raise MotionCaseError("motion begin event differs")
        tags = [child.tag for child in children]
        expected_tags = {
            "ZERO": ["begin", "mvnull"],
            "TRANSLATION": ["begin", "mvrectsinu", "mvnull"],
            "YAW": ["begin", "mvrotsinu", "mvnull"],
        }[case["motion_kind"]]
        if tags != expected_tags:
            raise MotionCaseError("motion element sequence differs")


def render_case(base: bytes, case: dict[str, Any]) -> bytes:
    if base.count(BASE_MOTION_BLOCK) != 2:
        raise MotionCaseError("sealed base XML motion-block cardinality differs")
    rendered = base.replace(BASE_MOTION_BLOCK, motion_block(case))
    validate_rendered(rendered, case, base)
    return rendered


def _write_exclusive(path: Path, data: bytes) -> dict[str, Any]:
    if path.parent != CASE_ROOT or path.resolve(strict=False).parent != CASE_ROOT:
        raise MotionCaseError("output XML leaves the exact case directory")
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
    try:
        os.fchmod(descriptor, 0o640)
        view = memoryview(data)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise MotionCaseError("short XML write")
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return identity(path)


def rendered_cases(contract: dict[str, Any]) -> dict[str, bytes]:
    base = BASE_CASE_PATH.read_bytes()
    if hashlib.sha256(base).hexdigest() != contract["parents"]["base_case_xml"]["sha256"]:
        raise MotionCaseError("sealed base XML bytes differ")
    return {name: render_case(base, case) for name, case in contract["cases"].items()}


def self_check() -> dict[str, Any]:
    contract = load_contract()
    verified = _verify_parents(contract)
    rendered = rendered_cases(contract)
    return {
        "status": "PASS_U3_STAGE4_MOTION_CASE_GENERATOR_V1_SELF_CHECK",
        "contract": identity(CONTRACT_PATH, MAX_JSON_BYTES),
        "schema": identity(SCHEMA_PATH, MAX_JSON_BYTES),
        "script": identity(SCRIPT_PATH, MAX_JSON_BYTES),
        "verified_parents": verified,
        "rendered": {
            name: {"path": contract["cases"][name]["output_xml"], "size_bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}
            for name, data in rendered.items()
        },
        "files_written": False,
        "solver_executed": False,
        "gpu_exposed": False,
        "network_used": False,
    }


def generate() -> dict[str, Any]:
    contract = load_contract()
    _verify_parents(contract)
    rendered = rendered_cases(contract)
    paths = {name: Path(contract["cases"][name]["output_xml"]) for name in rendered}
    collisions = [str(path) for path in paths.values() if os.path.lexists(path)]
    if collisions:
        raise MotionCaseError(f"create-new XML collision: {collisions}")
    produced = {name: _write_exclusive(paths[name], rendered[name]) for name in ("zero", "translation", "yaw")}
    return {
        "status": "U3_STAGE4_SYNTHETIC_MOTION_XMLS_FROZEN",
        "contract": identity(CONTRACT_PATH, MAX_JSON_BYTES),
        "outputs": produced,
        "solver_executed": False,
        "gpu_exposed": False,
        "network_used": False,
    }


def verify() -> dict[str, Any]:
    contract = load_contract()
    _verify_parents(contract)
    rendered = rendered_cases(contract)
    outputs: dict[str, Any] = {}
    for name, expected_bytes in rendered.items():
        path = Path(contract["cases"][name]["output_xml"])
        observed = identity(path)
        if path.read_bytes() != expected_bytes or observed["mode"] != "0640":
            raise MotionCaseError(f"materialized XML differs: {name}")
        outputs[name] = observed
    return {
        "status": "PASS_U3_STAGE4_SYNTHETIC_MOTION_XML_VERIFY",
        "outputs": outputs,
        "solver_executed": False,
        "gpu_exposed": False,
        "network_used": False,
    }


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("command", choices=("self-check", "generate", "verify"))
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        result = {"self-check": self_check, "generate": generate, "verify": verify}[args.command]()
    except (MotionCaseError, OSError, ValueError, json.JSONDecodeError) as exc:
        print(json.dumps({"status": "FAIL_U3_STAGE4_MOTION_CASE_GENERATOR_V1", "error_type": type(exc).__name__, "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
