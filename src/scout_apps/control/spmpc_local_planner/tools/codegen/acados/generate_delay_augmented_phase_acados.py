#!/usr/bin/env python3
"""Generate the delay-augmented Phase-Rejoin DISCRETE OCP.

The generated capsule is owned by the explicit opt-in online development
Phase-Rejoin backend. It freezes the WP3 dynamics, nominal-relative cost
parameter image, command envelope, terminal 9D empirical gate, and terminal
execution-compatibility box. Formal use still requires a separately frozen
recovery artifact; generated capability is not release evidence.
"""

import argparse
import hashlib
import json
import math
import os
import pathlib
import sys

import casadi as ca
import numpy as np
import yaml

from generate_delay_augmented_phase_transition import load_contract
from spmpc_delay_augmented_phase_model import (
    GATE_RADIUS_NAMES,
    WEIGHT_NAMES,
    nominal_relative_cost,
    nominal_relative_residual,
    parameter_layout,
    published_command_constraints,
    state_layout,
    terminal_recovery_constraints,
    transition_expression,
)


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT = PACKAGE_ROOT / "generated" / "acados"
MODEL_NAME = "spmpc_delay_augmented_phase"
SOLVER_CAPABILITY_SCHEMA_VERSION = 3
SOLVER_ID = "delay_augmented_phase_acados_online_v2"
PARAMETER_SCHEMA_VERSION = 2
PARAMETER_SCHEMA_ID = "delay_augmented_phase_parameter_image_v2"
MAX_EQUALITY_RESIDUAL = 1.0e-6
MAX_INEQUALITY_RESIDUAL = 1.0e-6
MAX_CAUSAL_STATE_ERROR = 1.0e-6
MIN_RECOVERY_DENOMINATOR = 1.0e-9
PUBLISHED_CONSISTENCY_TOLERANCE = 1.0e-9

CAP_DISCRETE_DYNAMICS = 1 << 0
CAP_AUGMENTED_INITIAL_STATE = 1 << 1
CAP_PUBLISHED_COMMAND_BOUNDS = 1 << 2
CAP_ROBOT_SPEED_BOUNDS = 1 << 3
CAP_PUBLISHED_RATE_BOUNDS = 1 << 4
CAP_TERMINAL_EMPIRICAL_GATE = 1 << 5
CAP_EXECUTION_COMPATIBILITY_SET = 1 << 6
CAP_PUBLISHED_RESIDUAL_BOUNDS = 1 << 7

WP3C_CAPABILITIES = (
    CAP_DISCRETE_DYNAMICS
    | CAP_AUGMENTED_INITIAL_STATE
    | CAP_PUBLISHED_COMMAND_BOUNDS
    | CAP_ROBOT_SPEED_BOUNDS
    | CAP_PUBLISHED_RATE_BOUNDS
)
FORMAL_REQUIRED_CAPABILITIES = (
    WP3C_CAPABILITIES
    | CAP_TERMINAL_EMPIRICAL_GATE
    | CAP_EXECUTION_COMPATIBILITY_SET
    | CAP_PUBLISHED_RESIDUAL_BOUNDS
)


def _load_yaml(relative_path):
    with (PACKAGE_ROOT / relative_path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def load_solver_spec():
    contract = load_contract()
    layout = state_layout(contract)
    platform = _load_yaml("config/platforms/scout_mini.yaml")["robot"]
    common = _load_yaml("config/planner/common.yaml")
    experiment = _load_yaml("config/experiments/fixed_path.yaml")["experiment"]
    slosh = _load_yaml("config/containers/tube_default.yaml")["slosh"]
    use_linear_model = bool(slosh.get("use_linear_model", True))
    use_parabola_term = bool(slosh.get("use_parabola_term", False))
    if use_parabola_term:
        raise ValueError(
            "delay-augmented OCP does not implement the parabolic height "
            "term; set slosh/use_parabola_term=false before codegen")
    parameters = parameter_layout(layout)
    modal_root = (1.8412, 5.3314, 8.5363, 11.7060, 14.8636)[
        int(slosh["mode_index"]) - 1
    ]
    frequency_argument = (
        modal_root * float(slosh["liquid_height"])
        / float(slosh["container_radius"])
    )
    modal_mass_ratio = (
        2.0 * float(slosh["container_radius"])
        * math.tanh(frequency_argument)
        / (
            modal_root * float(slosh["liquid_height"])
            * (modal_root * modal_root - 1.0)
        )
    )
    if use_linear_model:
        height_coeff = (
            4.0 * float(slosh["liquid_height"]) * modal_mass_ratio
            / float(slosh["container_radius"])
        )
    else:
        height_coeff = (
            modal_root * modal_root * float(slosh["liquid_height"])
            * modal_mass_ratio / float(slosh["container_radius"])
        )
    omega_n = math.sqrt(
        9.81 * modal_root / float(slosh["container_radius"])
        * math.tanh(frequency_argument)
    )
    eta_scale = float(slosh["slosh_height_ref"]) / height_coeff
    cost_scales = {
        "position": 0.5 * float(experiment["corridor_width"]),
        "yaw": 1.0,
        "progress": max(0.1, float(platform["v_max"]) * contract["dt"]),
        "v": float(platform["v_max"]),
        "omega": float(platform["omega_max"]),
        "eta": eta_scale,
        "eta_dot": omega_n * eta_scale,
        "a": float(platform["a_max"]),
        "alpha": float(platform.get("alpha_max", 1.2)),
        "v_s": float(platform["v_max"]),
    }
    parameter_semantic = {
        "schema_version": PARAMETER_SCHEMA_VERSION,
        "schema_id": PARAMETER_SCHEMA_ID,
        "names": parameters["names"],
        "state_width": layout["state_width"],
        "control_width": layout["control_width"],
        "horizon_steps": contract["horizon_steps"],
        "execution_contract_hash": contract["contract_hash"],
        "cost_contract": "nominal_relative_augmented_nls_v2",
        "cost_scales": cost_scales,
        "control_bounds": {
            "acceleration_max": float(platform["a_max"]),
            "angular_acceleration_max": float(
                platform.get("alpha_max", 1.2)),
            "progress_rate_max": float(platform["v_max"]),
        },
        "derived_state_bound_contract":
            "validated_initial_plus_bounded_publish_invariance_v1",
        "terminal_gate_contract": "phase_indexed_empirical_9d_ellipsoid_v1",
        "execution_compatibility_contract": "phase_indexed_execution_box_v1",
    }
    parameter_hash = hashlib.sha256(
        json.dumps(
            parameter_semantic,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    ).hexdigest()
    return {
        "model_name": MODEL_NAME,
        "solver_id": SOLVER_ID,
        "capability_schema_version": SOLVER_CAPABILITY_SCHEMA_VERSION,
        "capabilities": FORMAL_REQUIRED_CAPABILITIES,
        "formal_required_capabilities": FORMAL_REQUIRED_CAPABILITIES,
        "contract": contract,
        "layout": layout,
        "parameters": parameters,
        "parameter_schema_version": PARAMETER_SCHEMA_VERSION,
        "parameter_schema_id": PARAMETER_SCHEMA_ID,
        "parameter_schema_hash": parameter_hash,
        "cost_contract": parameter_semantic["cost_contract"],
        "terminal_gate_contract": parameter_semantic["terminal_gate_contract"],
        "execution_compatibility_contract": parameter_semantic[
            "execution_compatibility_contract"
        ],
        "cost_scales": cost_scales,
        "slosh_height_ref": float(slosh["slosh_height_ref"]),
        "slosh_eta_dot_ratio": float(slosh["slosh_eta_dot_ratio"]),
        "use_linear_model": use_linear_model,
        "use_parabola_term": use_parabola_term,
        "a_max": float(platform["a_max"]),
        "alpha_max": float(platform.get("alpha_max", 1.2)),
        "vs_max": float(platform["v_max"]),
        "max_equality_residual": MAX_EQUALITY_RESIDUAL,
        "max_inequality_residual": MAX_INEQUALITY_RESIDUAL,
        "max_causal_state_error": MAX_CAUSAL_STATE_ERROR,
        "state_bound_count": 0,
    }


def build_symbolic_spec(spec):
    contract = spec["contract"]
    layout = spec["layout"]
    x = ca.SX.sym("x", layout["state_width"])
    q = ca.SX.sym("q", layout["control_width"])
    p = ca.SX.sym("p", spec["parameters"]["parameter_width"])
    x_next, published = transition_expression(x, q, contract, layout)
    stage_constraints, stage_lower, stage_upper = (
        published_command_constraints(
            published,
            p,
            spec["parameters"],
            contract["linear"]["output_min"],
            contract["linear"]["output_max"],
            contract["angular"]["output_min"],
            contract["angular"]["output_max"],
        )
    )
    stage_cost = nominal_relative_cost(
        x, q, p, layout, spec["parameters"], spec["cost_scales"]
    )
    terminal_cost = nominal_relative_cost(
        x, q, p, layout, spec["parameters"], spec["cost_scales"],
        terminal=True,
    )
    stage_residual = nominal_relative_residual(
        x, q, p, layout, spec["parameters"], spec["cost_scales"]
    )
    terminal_residual = nominal_relative_residual(
        x, q, p, layout, spec["parameters"], spec["cost_scales"],
        terminal=True,
    )
    terminal_constraints = terminal_recovery_constraints(
        x, p, layout, spec["parameters"]
    )

    # No path-stage box is needed for actuator outputs or pending queues.
    # The initial augmented state is validated and fixed; every new published
    # command is hard-bounded below; queue entries are exact shifts of those
    # commands; and the first-order actuator update is a convex combination
    # with a saturated target.  Repeating the same bounds on all of those
    # derived states makes the zero-command trajectory highly degenerate for
    # the interior-point QP without strengthening the feasible set.
    state_indices = []
    state_lower = []
    state_upper = []

    return {
        "x": x,
        "q": q,
        "p": p,
        "x_next": x_next,
        "published": published,
        "stage_constraints": stage_constraints,
        "stage_cost": stage_cost,
        "terminal_cost": terminal_cost,
        "stage_residual": stage_residual,
        "terminal_residual": terminal_residual,
        "terminal_constraints": terminal_constraints,
        "idxbu": np.array([0, 1, 2], dtype=int),
        "lbu": np.array(
            [-spec["a_max"], -spec["alpha_max"], 0.0]),
        "ubu": np.array(
            [spec["a_max"], spec["alpha_max"], spec["vs_max"]]),
        "idxbx": np.array(state_indices, dtype=int),
        "lbx": np.array(state_lower),
        "ubx": np.array(state_upper),
        "lh": np.array(stage_lower),
        "uh": np.array(stage_upper),
    }


def _cpp_float(value):
    return format(float(value), ".17g")


def emit_solver_manifest(spec, output_root):
    output_root = pathlib.Path(output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    contract = spec["contract"]
    layout = spec["layout"]
    path = output_root / "spmpc_delay_augmented_phase_solver_manifest.h"
    content = f"""// Generated by tools/codegen/acados/generate_delay_augmented_phase_acados.py.
// Development online solver contract only; not a formal Scout release artifact.
#pragma once

#include <cstdint>

namespace spmpc_local_planner {{
namespace delay_augmented_phase_solver_manifest {{

constexpr int kCapabilitySchemaVersion = {spec['capability_schema_version']};
constexpr const char kSolverId[] = "{spec['solver_id']}";
constexpr const char kModelName[] = "{spec['model_name']}";
constexpr const char kIntegratorType[] = "DISCRETE";
constexpr int kExecutionContractSchemaVersion = {contract['schema_version']};
constexpr const char kContractId[] = "{contract['contract_id']}";
constexpr const char kContractHash[] = "{contract['contract_hash']}";
constexpr int kStateCount = {layout['state_width']};
constexpr int kControlCount = {layout['control_width']};
constexpr int kParameterSchemaVersion = {spec['parameter_schema_version']};
constexpr const char kParameterSchemaId[] = "{spec['parameter_schema_id']}";
constexpr const char kParameterSchemaHash[] = "{spec['parameter_schema_hash']}";
constexpr const char kCostContract[] = "{spec['cost_contract']}";
constexpr const char kTerminalGateContract[] = "{spec['terminal_gate_contract']}";
constexpr const char kExecutionCompatibilityContract[] = "{spec['execution_compatibility_contract']}";
constexpr int kParameterCount = {spec['parameters']['parameter_width']};
constexpr int kNominalStateOffset = {spec['parameters']['nominal_state_offset']};
constexpr int kNominalControlOffset = {spec['parameters']['nominal_control_offset']};
constexpr int kNominalPublishOffset = {spec['parameters']['nominal_publish_offset']};
constexpr int kResidualBoundOffset = {spec['parameters']['residual_bound_offset']};
constexpr int kWeightOffset = {spec['parameters']['weight_offset']};
constexpr int kGateRadiusOffset = {spec['parameters']['gate_radius_offset']};
constexpr int kExecutionBoundOffset = {spec['parameters']['execution_bound_offset']};
constexpr int kWeightCount = {len(WEIGHT_NAMES)};
constexpr int kGateRadiusCount = {len(GATE_RADIUS_NAMES)};
constexpr int kExecutionBoundCount = {len(layout['execution_indices'])};
constexpr int kHorizonSteps = {contract['horizon_steps']};
constexpr int kExecutionFrontSteps = {contract['execution_front_steps']};
constexpr int kLiquidHorizonSteps = {contract['liquid_horizon_steps']};
constexpr int kLinearBufferOffset = {layout['linear_buffer_offset']};
constexpr int kLinearBufferCount = {layout['linear_buffer_count']};
constexpr int kAngularBufferOffset = {layout['angular_buffer_offset']};
constexpr int kAngularBufferCount = {layout['angular_buffer_count']};
constexpr int kStateBoundCount = {spec['state_bound_count']};
constexpr int kInitialStateBoundCount = {layout['state_width']};
constexpr int kTerminalStateBoundCount = kStateBoundCount;
constexpr int kControlBoundCount = {layout['control_width']};
constexpr int kPublishedCommandConstraintCount = 6;
constexpr int kTerminalPublishedCommandConstraintCount = 0;
constexpr int kTerminalRecoveryConstraintCount = {1 + len(layout['execution_indices'])};
constexpr double kDt = {_cpp_float(contract['dt'])};
constexpr double kLinearDelaySec = {_cpp_float(contract['linear']['delay_sec'])};
constexpr double kAngularDelaySec = {_cpp_float(contract['angular']['delay_sec'])};
constexpr int kLinearIntegerDelaySteps = {contract['linear']['integer_delay_steps']};
constexpr int kAngularIntegerDelaySteps = {contract['angular']['integer_delay_steps']};
constexpr double kLinearFractionalDelaySec = {_cpp_float(contract['linear']['fractional_delay_sec'])};
constexpr double kAngularFractionalDelaySec = {_cpp_float(contract['angular']['fractional_delay_sec'])};
constexpr double kLinearTimeConstantSec = {_cpp_float(contract['linear']['time_constant_sec'])};
constexpr double kAngularTimeConstantSec = {_cpp_float(contract['angular']['time_constant_sec'])};
constexpr double kLinearPositiveGain = {_cpp_float(contract['linear']['positive_gain'])};
constexpr double kLinearNegativeGain = {_cpp_float(contract['linear']['negative_gain'])};
constexpr double kAngularPositiveGain = {_cpp_float(contract['angular']['positive_gain'])};
constexpr double kAngularNegativeGain = {_cpp_float(contract['angular']['negative_gain'])};
constexpr double kLinearDeadzone = {_cpp_float(contract['linear']['deadzone'])};
constexpr double kAngularDeadzone = {_cpp_float(contract['angular']['deadzone'])};
constexpr double kLinearOutputMin = {_cpp_float(contract['linear']['output_min'])};
constexpr double kLinearOutputMax = {_cpp_float(contract['linear']['output_max'])};
constexpr double kAngularOutputMin = {_cpp_float(contract['angular']['output_min'])};
constexpr double kAngularOutputMax = {_cpp_float(contract['angular']['output_max'])};
constexpr double kAccelerationMax = {_cpp_float(spec['a_max'])};
constexpr double kAngularAccelerationMax = {_cpp_float(spec['alpha_max'])};
constexpr double kProgressRateMax = {_cpp_float(spec['vs_max'])};
constexpr double kMaxEqualityResidual = {_cpp_float(spec['max_equality_residual'])};
constexpr double kMaxInequalityResidual = {_cpp_float(spec['max_inequality_residual'])};
constexpr double kMaxCausalStateError = {_cpp_float(spec['max_causal_state_error'])};
constexpr double kMinimumRecoveryDenominator = {_cpp_float(MIN_RECOVERY_DENOMINATOR)};
constexpr double kPublishedConsistencyTolerance = {_cpp_float(PUBLISHED_CONSISTENCY_TOLERANCE)};
constexpr double kPositionScale = {_cpp_float(spec['cost_scales']['position'])};
constexpr double kYawScale = {_cpp_float(spec['cost_scales']['yaw'])};
constexpr double kProgressScale = {_cpp_float(spec['cost_scales']['progress'])};
constexpr double kVelocityScale = {_cpp_float(spec['cost_scales']['v'])};
constexpr double kAngularVelocityScale = {_cpp_float(spec['cost_scales']['omega'])};
constexpr double kEtaScale = {_cpp_float(spec['cost_scales']['eta'])};
constexpr double kEtaDotScale = {_cpp_float(spec['cost_scales']['eta_dot'])};
constexpr double kAccelerationScale = {_cpp_float(spec['cost_scales']['a'])};
constexpr double kAngularAccelerationScale = {_cpp_float(spec['cost_scales']['alpha'])};
constexpr double kProgressRateScale = {_cpp_float(spec['cost_scales']['v_s'])};
constexpr double kSloshHeightRef = {_cpp_float(spec['slosh_height_ref'])};
constexpr double kSloshEtaDotRatio = {_cpp_float(spec['slosh_eta_dot_ratio'])};
constexpr bool kUseLinearModel = {str(spec['use_linear_model']).lower()};
constexpr bool kUseParabolaTerm = {str(spec['use_parabola_term']).lower()};
constexpr double kContainerRadius = {_cpp_float(contract['slosh']['container_radius'])};
constexpr double kLiquidHeight = {_cpp_float(contract['slosh']['liquid_height'])};
constexpr double kLiquidDensity = {_cpp_float(contract['slosh']['liquid_density'])};
constexpr double kDampingRatio = {_cpp_float(contract['slosh']['damping_ratio'])};
constexpr int kModeIndex = {contract['slosh']['mode_index']};

constexpr const char* kParameterNames[kParameterCount] = {{
{chr(10).join('    "' + name + '",' for name in spec['parameters']['names'])}
}};

constexpr std::uint32_t kDiscreteDynamics = {CAP_DISCRETE_DYNAMICS}u;
constexpr std::uint32_t kAugmentedInitialState = {CAP_AUGMENTED_INITIAL_STATE}u;
constexpr std::uint32_t kPublishedCommandBounds = {CAP_PUBLISHED_COMMAND_BOUNDS}u;
constexpr std::uint32_t kRobotSpeedBounds = {CAP_ROBOT_SPEED_BOUNDS}u;
constexpr std::uint32_t kPublishedRateBounds = {CAP_PUBLISHED_RATE_BOUNDS}u;
constexpr std::uint32_t kTerminalEmpiricalGate = {CAP_TERMINAL_EMPIRICAL_GATE}u;
constexpr std::uint32_t kExecutionCompatibilitySet = {CAP_EXECUTION_COMPATIBILITY_SET}u;
constexpr std::uint32_t kPublishedResidualBounds = {CAP_PUBLISHED_RESIDUAL_BOUNDS}u;
constexpr std::uint32_t kCapabilities = {spec['capabilities']}u;
constexpr std::uint32_t kFormalRequiredCapabilities = {spec['formal_required_capabilities']}u;

}}  // namespace delay_augmented_phase_solver_manifest
}}  // namespace spmpc_local_planner
"""
    path.write_text(content, encoding="utf-8")
    return path


def _import_acados_template():
    try:
        from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
        return AcadosModel, AcadosOcp, AcadosOcpSolver
    except ImportError:
        source = os.environ.get("ACADOS_SOURCE_DIR", "")
        candidate = pathlib.Path(source) / "interfaces" / "acados_template"
        if source and candidate.is_dir():
            sys.path.insert(0, str(candidate))
            from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
            return AcadosModel, AcadosOcp, AcadosOcpSolver
        raise


def build_ocp(spec):
    AcadosModel, AcadosOcp, _ = _import_acados_template()
    symbolic = build_symbolic_spec(spec)
    model = AcadosModel()
    model.name = MODEL_NAME
    model.x = symbolic["x"]
    model.u = symbolic["q"]
    model.p = symbolic["p"]
    model.disc_dyn_expr = symbolic["x_next"]

    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = spec["contract"]["horizon_steps"]
    ocp.solver_options.tf = (
        spec["contract"]["horizon_steps"] * spec["contract"]["dt"])
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.model.cost_y_expr = symbolic["stage_residual"]
    ocp.model.cost_y_expr_e = symbolic["terminal_residual"]
    stage_residual_count = int(symbolic["stage_residual"].shape[0])
    terminal_residual_count = int(
        symbolic["terminal_residual"].shape[0])
    ocp.cost.W = np.eye(stage_residual_count)
    ocp.cost.W_e = np.eye(terminal_residual_count)
    ocp.cost.yref = np.zeros(stage_residual_count)
    ocp.cost.yref_e = np.zeros(terminal_residual_count)
    parameter_defaults = np.ones(spec["parameters"]["parameter_width"])
    parameter_defaults[
        spec["parameters"]["nominal_state_offset"]:
        spec["parameters"]["nominal_control_offset"]
    ] = 0.0
    parameter_defaults[
        spec["parameters"]["nominal_control_offset"]:
        spec["parameters"]["weight_offset"]
    ] = 0.0
    ocp.parameter_values = parameter_defaults

    ocp.constraints.idxbu = symbolic["idxbu"]
    ocp.constraints.lbu = symbolic["lbu"]
    ocp.constraints.ubu = symbolic["ubu"]
    ocp.constraints.idxbx = symbolic["idxbx"]
    ocp.constraints.lbx = symbolic["lbx"]
    ocp.constraints.ubx = symbolic["ubx"]
    ocp.constraints.idxbx_e = symbolic["idxbx"]
    ocp.constraints.lbx_e = symbolic["lbx"]
    ocp.constraints.ubx_e = symbolic["ubx"]
    ocp.constraints.x0 = np.zeros(spec["layout"]["state_width"])
    ocp.model.con_h_expr = symbolic["stage_constraints"]
    ocp.model.con_h_expr_0 = symbolic["stage_constraints"]
    ocp.constraints.lh = symbolic["lh"]
    ocp.constraints.uh = symbolic["uh"]
    ocp.constraints.lh_0 = symbolic["lh"]
    ocp.constraints.uh_0 = symbolic["uh"]
    ocp.model.con_h_expr_e = symbolic["terminal_constraints"]
    terminal_count = int(symbolic["terminal_constraints"].shape[0])
    # These are genuinely one-sided upper constraints.  acados masks a bound
    # whose magnitude exceeds ACADOS_INFTY before passing the QP to HPIPM; a
    # finite mathematical minimum such as -1 would instead create a redundant,
    # zero-gradient active lower constraint at the nominal terminal state.
    ocp.constraints.lh_e = np.full(terminal_count, -1.0e15)
    ocp.constraints.uh_e = np.zeros(terminal_count)

    ocp.solver_options.integrator_type = "DISCRETE"
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hpipm_mode = "SPEED_ABS"
    ocp.solver_options.qp_solver_iter_max = 100
    ocp.solver_options.qp_solver_tol_stat = MAX_EQUALITY_RESIDUAL
    ocp.solver_options.qp_solver_tol_eq = MAX_EQUALITY_RESIDUAL
    ocp.solver_options.qp_solver_tol_ineq = MAX_INEQUALITY_RESIDUAL
    ocp.solver_options.qp_solver_tol_comp = MAX_INEQUALITY_RESIDUAL
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.regularize_method = "PROJECT"
    ocp.solver_options.levenberg_marquardt = 1.0e-3
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.tol_eq = spec["max_equality_residual"]
    ocp.solver_options.tol_ineq = spec["max_inequality_residual"]
    return ocp


def generate(output_root, build=True):
    _, _, AcadosOcpSolver = _import_acados_template()
    output_root = pathlib.Path(output_root).resolve()
    spec = load_solver_spec()
    manifest = emit_solver_manifest(spec, output_root)
    ocp = build_ocp(spec)
    export_dir = output_root / MODEL_NAME
    export_dir.mkdir(parents=True, exist_ok=True)
    ocp.code_gen_opts.code_export_directory = str(export_dir)
    json_path = export_dir / f"acados_ocp_{MODEL_NAME}.json"
    AcadosOcpSolver.generate(ocp, json_file=str(json_path), verbose=True)
    if build:
        AcadosOcpSolver.build(str(export_dir), with_cython=False, verbose=True)
    return manifest, json_path


def check():
    spec = load_solver_spec()
    symbolic = build_symbolic_spec(spec)
    if symbolic["x_next"].shape != (spec["layout"]["state_width"], 1):
        raise RuntimeError("delay-augmented discrete state width mismatch")
    if symbolic["published"].shape != (2, 1):
        raise RuntimeError("published-command constraint width mismatch")
    if symbolic["stage_constraints"].shape != (6, 1):
        raise RuntimeError("published residual constraint width mismatch")
    if symbolic["terminal_constraints"].shape != (
            1 + len(spec["layout"]["execution_indices"]), 1):
        raise RuntimeError("terminal recovery constraint width mismatch")
    if len(symbolic["idxbx"]) != spec["state_bound_count"]:
        raise RuntimeError("derived-state bound contract mismatch")
    if spec["capabilities"] != spec["formal_required_capabilities"]:
        raise RuntimeError("invalid solver capability mask")
    print(
        "[check] delay-augmented Phase-Rejoin DISCRETE OCP",
        f"nx={spec['layout']['state_width']}",
        f"nu={spec['layout']['control_width']}",
        f"N={spec['contract']['horizon_steps']}",
        f"np={spec['parameters']['parameter_width']}",
        f"nbx={len(symbolic['idxbx'])}",
        "nh=6",
        f"capabilities=0x{spec['capabilities']:x}",
        f"execution_hash={spec['contract']['contract_hash']}",
        f"parameter_hash={spec['parameter_schema_hash']}",
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--no-build", action="store_true")
    args = parser.parse_args()
    if args.check:
        check()
        print("[ok] solver specification validated; no files written")
        return 0
    generated = generate(args.output_dir, build=not args.no_build)
    for path in generated:
        print(f"[ok] generated {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
