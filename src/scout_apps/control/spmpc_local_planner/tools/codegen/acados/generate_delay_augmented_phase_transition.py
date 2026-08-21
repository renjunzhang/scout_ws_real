#!/usr/bin/env python3
"""Generate the deterministic CasADi C transition used by WP3 consistency tests."""

import argparse
import hashlib
import json
import math
import os
import pathlib

import casadi as ca
import numpy as np
import scipy.linalg
import yaml

from spmpc_delay_augmented_phase_model import export_transition_functions


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT = PACKAGE_ROOT / "generated" / "casadi"
MODAL_ROOTS = (1.8412, 5.3314, 8.5363, 11.7060, 14.8636)


def _load_yaml(relative_path):
    with (PACKAGE_ROOT / relative_path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _resolve_channel(delay, dt, time_constant, output_min, output_max):
    integer_steps = int(math.floor(delay / dt))
    remainder = delay - integer_steps * dt
    tolerance = max(1e-12, dt * 1e-12)
    if remainder <= tolerance:
        remainder = 0.0
    elif dt - remainder <= tolerance:
        integer_steps += 1
        remainder = 0.0
    return {
        "delay_sec": float(delay),
        "time_constant_sec": float(time_constant),
        "positive_gain": 1.0,
        "negative_gain": 1.0,
        "deadzone": 0.0,
        "output_min": float(output_min),
        "output_max": float(output_max),
        "integer_delay_steps": integer_steps,
        "fractional_delay_sec": remainder,
    }


def _slosh_matrices(slosh, duration):
    modal_root = MODAL_ROOTS[int(slosh["mode_index"]) - 1]
    omega_sq = (
        9.81
        * modal_root
        / float(slosh["container_radius"])
        * math.tanh(
            modal_root
            * float(slosh["liquid_height"])
            / float(slosh["container_radius"])
        )
    )
    omega_n = math.sqrt(max(0.0, omega_sq))
    damping = 2.0 * float(slosh["damping_ratio"]) * omega_n
    continuous_a = np.zeros((4, 4))
    continuous_a[0, 1] = 1.0
    continuous_a[1, 0] = -omega_sq
    continuous_a[1, 1] = -damping
    continuous_a[2, 3] = 1.0
    continuous_a[3, 2] = -omega_sq
    continuous_a[3, 3] = -damping
    continuous_b = np.zeros((4, 2))
    continuous_b[1, 0] = -1.0
    continuous_b[3, 1] = -1.0
    augmented = np.zeros((6, 6))
    augmented[:4, :4] = continuous_a * duration
    augmented[:4, 4:] = continuous_b * duration
    discretized = scipy.linalg.expm(augmented)
    return {
        "ad": discretized[:4, :4].tolist(),
        "bd": discretized[:4, 4:].tolist(),
    }


def load_contract():
    common = _load_yaml("config/planner/common.yaml")
    platform = _load_yaml("config/platforms/scout_mini.yaml")["robot"]
    slosh = _load_yaml("config/containers/tube_default.yaml")["slosh"]
    delay = common["delay_phase"]
    dt = float(common["dt"])
    linear = _resolve_channel(
        float(delay["linear_delay_sec"]),
        dt,
        float(delay["linear_time_constant_sec"]),
        0.0,
        float(platform["v_max"]),
    )
    angular = _resolve_channel(
        float(delay["angular_delay_sec"]),
        dt,
        float(delay["angular_time_constant_sec"]),
        -float(platform["omega_max"]),
        float(platform["omega_max"]),
    )
    events = sorted({
        0.0,
        dt,
        linear["fractional_delay_sec"],
        angular["fractional_delay_sec"],
    })
    front_steps = max(
        linear["integer_delay_steps"]
        + (1 if linear["fractional_delay_sec"] > 1e-12 else 0),
        angular["integer_delay_steps"]
        + (1 if angular["fractional_delay_sec"] > 1e-12 else 0),
    )
    liquid_steps = int(common["phase_rejoin"]["liquid_horizon_steps"])
    contract = {
        "schema_version": 1,
        "contract_id": "delay_augmented_phase_codegen_candidate_v1",
        "dt": dt,
        "linear": linear,
        "angular": angular,
        "events": events,
        "execution_front_steps": front_steps,
        "liquid_horizon_steps": liquid_steps,
        "horizon_steps": front_steps + liquid_steps,
        "slosh": {
            "container_radius": float(slosh["container_radius"]),
            "liquid_height": float(slosh["liquid_height"]),
            "liquid_density": float(slosh["liquid_density"]),
            "damping_ratio": float(slosh["damping_ratio"]),
            "mode_index": int(slosh["mode_index"]),
        },
    }
    contract["slosh_segment_matrices"] = [
        _slosh_matrices(slosh, events[index + 1] - events[index])
        for index in range(len(events) - 1)
    ]
    semantic = dict(contract)
    semantic.pop("slosh_segment_matrices")
    contract["contract_hash"] = hashlib.sha256(
        json.dumps(
            semantic, sort_keys=True, separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    ).hexdigest()
    return contract


def _cpp_float(value):
    return format(float(value), ".17g")


def emit_manifest(contract, layout, output_dir):
    output_dir = pathlib.Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / "spmpc_delay_augmented_phase_manifest.h"
    content = f"""// Generated by tools/codegen/acados/generate_delay_augmented_phase_transition.py.
// Candidate consistency contract only; not a formal Scout release artifact.
#pragma once

namespace spmpc_local_planner {{
namespace delay_augmented_phase_manifest {{

constexpr int kSchemaVersion = {contract['schema_version']};
constexpr const char kContractId[] = "{contract['contract_id']}";
constexpr const char kContractHash[] = "{contract['contract_hash']}";
constexpr double kDt = {_cpp_float(contract['dt'])};
constexpr int kStateCount = {layout['state_width']};
constexpr int kControlCount = {layout['control_width']};
constexpr int kLinearBufferOffset = {layout['linear_buffer_offset']};
constexpr int kLinearBufferCount = {layout['linear_buffer_count']};
constexpr int kAngularBufferOffset = {layout['angular_buffer_offset']};
constexpr int kAngularBufferCount = {layout['angular_buffer_count']};
constexpr int kExecutionFrontSteps = {contract['execution_front_steps']};
constexpr int kLiquidHorizonSteps = {contract['liquid_horizon_steps']};
constexpr int kHorizonSteps = {contract['horizon_steps']};
constexpr double kLinearDelaySec = {_cpp_float(contract['linear']['delay_sec'])};
constexpr double kAngularDelaySec = {_cpp_float(contract['angular']['delay_sec'])};
constexpr double kLinearFractionalDelaySec = {_cpp_float(contract['linear']['fractional_delay_sec'])};
constexpr double kAngularFractionalDelaySec = {_cpp_float(contract['angular']['fractional_delay_sec'])};
constexpr int kLinearIntegerDelaySteps = {contract['linear']['integer_delay_steps']};
constexpr int kAngularIntegerDelaySteps = {contract['angular']['integer_delay_steps']};
constexpr double kLinearTimeConstantSec = {_cpp_float(contract['linear']['time_constant_sec'])};
constexpr double kAngularTimeConstantSec = {_cpp_float(contract['angular']['time_constant_sec'])};
constexpr double kLinearOutputMin = {_cpp_float(contract['linear']['output_min'])};
constexpr double kLinearOutputMax = {_cpp_float(contract['linear']['output_max'])};
constexpr double kAngularOutputMin = {_cpp_float(contract['angular']['output_min'])};
constexpr double kAngularOutputMax = {_cpp_float(contract['angular']['output_max'])};
constexpr double kContainerRadius = {_cpp_float(contract['slosh']['container_radius'])};
constexpr double kLiquidHeight = {_cpp_float(contract['slosh']['liquid_height'])};
constexpr double kLiquidDensity = {_cpp_float(contract['slosh']['liquid_density'])};
constexpr double kDampingRatio = {_cpp_float(contract['slosh']['damping_ratio'])};
constexpr int kModeIndex = {contract['slosh']['mode_index']};

}}  // namespace delay_augmented_phase_manifest
}}  // namespace spmpc_local_planner
"""
    path.write_text(content, encoding="utf-8")
    return path


def _normalize_generated_text(path):
    content = path.read_text(encoding="utf-8")
    normalized = "\n".join(line.rstrip() for line in content.splitlines())
    path.write_text(normalized + "\n", encoding="utf-8")


def generate(output_dir):
    output_dir = pathlib.Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    contract = load_contract()
    exported = export_transition_functions(contract)
    c_path = output_dir / "spmpc_delay_augmented_phase_transition.c"
    generator = ca.CodeGenerator(c_path.name, {"with_header": True})
    generator.add(exported["step"])
    generator.add(exported["step_jacobian"])
    generator.add(exported["terminal_jacobian"])
    generator.generate(str(output_dir) + os.sep)
    _normalize_generated_text(c_path)
    _normalize_generated_text(c_path.with_suffix(".h"))
    manifest_path = emit_manifest(contract, exported["layout"], output_dir)
    return c_path, c_path.with_suffix(".h"), manifest_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    contract = load_contract()
    exported = export_transition_functions(contract)
    if args.check:
        print(
            "[check] delay-augmented phase transition",
            f"nx={exported['layout']['state_width']}",
            f"nu={exported['layout']['control_width']}",
            f"N_e={contract['horizon_steps']}",
            f"hash={contract['contract_hash']}",
        )
        return 0
    paths = generate(args.output_dir)
    for path in paths:
        print(f"[ok] generated {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
