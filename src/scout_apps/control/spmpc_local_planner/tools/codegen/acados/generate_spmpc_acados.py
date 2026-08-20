#!/usr/bin/env python3
"""SPMPC 连续 MPCC —— acados 求解器生成入口（orchestrator）。

职责（方案 §5.1 / §11.4）：
  - 唯一的 IO 边界：从包内 config/*.yaml 读取维度、bounds、默认权重；
  - 组装 AcadosModel / AcadosOcp，设置 EXTERNAL cost、约束、SQP-RTI 求解器选项；
  - codegen 输出到 generated/acados/<model_name>/（生成物不手改）。

依赖：本文件需要 acados_template；model/cost/constraints 只需 CasADi/numpy。
用法：
  python3 generate_spmpc_acados.py                         # 默认生成 alpha-state B0
  python3 generate_spmpc_acados.py --model slosh            # 生成 alpha-state slosh
  python3 generate_spmpc_acados.py --model b0_direct_omega_legacy
  python3 generate_spmpc_acados.py --model slosh_direct_omega
  python3 generate_spmpc_acados.py --check                  # 只校验 CasADi 模型/代价装配
"""

import argparse
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from spmpc_acados_model import (  # noqa: E402
    export_spmpc_b0_symbols,
    export_spmpc_b0_direct_omega_legacy_symbols,
    export_spmpc_slosh_symbols,
    export_spmpc_phase_rejoin_symbols,
    export_spmpc_slosh_direct_omega_symbols,
    PARAM_NAMES,
    PIDX,
    PARAM_NAMES_DIRECT_OMEGA_LEGACY,
    PIDX_DIRECT_OMEGA_LEGACY,
    PARAM_NAMES_SLOSH,
    PIDX_SLOSH,
    PARAM_NAMES_SLOSH_DIRECT_OMEGA,
    PIDX_SLOSH_DIRECT_OMEGA,
)
from spmpc_acados_cost import stage_cost_expr, terminal_cost_expr  # noqa: E402
from spmpc_acados_constraints import (  # noqa: E402
    set_constraints,
    set_constraints_direct_omega_legacy,
    set_constraints_slosh,
    slosh_nonlinear_constraint_expr,
)

MODELS = {
    "b0": {"export": export_spmpc_b0_symbols, "with_slosh": False},
    "slosh": {"export": export_spmpc_slosh_symbols, "with_slosh": True},
    "phase_rejoin": {
        "export": export_spmpc_phase_rejoin_symbols,
        "with_slosh": True,
        "short_phase_horizon": True,
    },
    "b0_direct_omega_legacy": {
        "export": export_spmpc_b0_direct_omega_legacy_symbols,
        "with_slosh": False,
        "direct_omega_legacy": True,
    },
    "slosh_direct_omega": {
        "export": export_spmpc_slosh_direct_omega_symbols,
        "with_slosh": True,
        "direct_omega_legacy": True,
    },
}

PKG_DIR = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))


def _load_yaml(rel_path):
    with open(os.path.join(PKG_DIR, rel_path), "r") as f:
        return yaml.safe_load(f)


def load_config():
    """从包内同一套 config 派生生成参数，避免脚本里另写一份口径。"""
    common = _load_yaml("config/planner/common.yaml")
    platform = _load_yaml("config/platforms/scout_mini.yaml")["robot"]
    experiment = _load_yaml("config/experiments/fixed_path.yaml")["experiment"]
    variants = _load_yaml("config/planner/variants.yaml")["variants"]
    b0 = variants["B0"]
    b_slosh = variants.get("B_slosh", b0)

    dt = float(common["dt"])
    n_steps = int(common["horizon_steps"])
    v_max = float(platform["v_max"])
    corridor_width = float(experiment["corridor_width"])

    cfg = {
        "dt": dt,
        "N": n_steps,
        "Tf": dt * n_steps,
        "v_max": v_max,
        "omega_max": float(platform["omega_max"]),
        "a_max": float(platform["a_max"]),
        # 转向角加速度上限(|d(omega)/dt|<=alpha_max)，与 TEB/DWA acc_lim_theta 同口径(默认 1.2)。
        "alpha_max": float(platform.get("alpha_max", 1.2)),
        "vs_max": v_max,
        # 非对称 anti-creep 相对 w_v 的增益（只罚 v/v_s 低于 v_ref 的亏空，防弯处停滞）。
        "anticreep_gain": float(common.get("anticreep_gain", 8.0)),
        "e_c_ref": 0.5 * corridor_width,
        # §4.4 建议 e_l_ref = v_ref*dt；置下限避免过小导致 lag 权重失真，最终由 wrapper 运行时设定。
        "e_l_ref": max(0.1, v_max * dt),
        "w_contour": float(b0["w_contour"]),
        "w_lag": float(b0["w_lag"]),
        "w_progress": float(b0["w_progress"]),
        "w_v": float(b0.get("w_v", 1.0)),
        "w_vs": float(b0.get("w_vs", 0.3)),
        "v_ref": float(b0.get("v_ref", 0.25)),
        "w_control": float(b0["w_control"]),
        "w_smooth": float(b0["w_smooth"]),
        "w_alpha": float(b0.get("w_alpha", b0["w_smooth"])),
        "w_du_a": float(b0.get("w_du_a", b0["w_smooth"])),
        "w_du_vs": float(b0.get("w_du_vs", b0["w_smooth"])),
        # slosh codegen 默认值（运行时由 C++ 包装层从同一套 slosh_dynamics 覆盖，§4.3）。
        "w_slosh": float(b_slosh.get("w_slosh", 5.0)),
        "phase_rejoin_N": int(common.get("phase_rejoin", {}).get(
            "liquid_horizon_steps", 3)),
    }
    return cfg


def default_parameter_values(cfg, with_slosh, direct_omega_legacy=False):
    """codegen 默认参数；运行时由 C++ 包装层每周期覆盖。"""
    if direct_omega_legacy:
        names = PARAM_NAMES_SLOSH_DIRECT_OMEGA if with_slosh else PARAM_NAMES_DIRECT_OMEGA_LEGACY
        idx = PIDX_SLOSH_DIRECT_OMEGA if with_slosh else PIDX_DIRECT_OMEGA_LEGACY
    else:
        names = PARAM_NAMES_SLOSH if with_slosh else PARAM_NAMES
        idx = PIDX_SLOSH if with_slosh else PIDX
    p = np.zeros(len(names))
    # 占位参考：x_ref(s)=s, y_ref(s)=0（直线），wrapper 每周期用 ReferenceSpline 拟合覆盖。
    p[idx["rx1"]] = 1.0
    p[idx["w_contour"]] = cfg["w_contour"]
    p[idx["w_lag"]] = cfg["w_lag"]
    p[idx["w_progress"]] = cfg["w_progress"]
    p[idx["w_a"]] = cfg["w_control"]
    p[idx["w_omega"]] = cfg["w_control"]
    p[idx["w_v"]] = cfg["w_v"]
    p[idx["w_vs"]] = cfg["w_vs"]
    if direct_omega_legacy:
        p[idx["w_du_omega"]] = cfg["w_smooth"]
    else:
        p[idx["w_alpha"]] = cfg["w_alpha"]   # 转向角加速度权重(抗 chatter，所有 stage 生效)
    p[idx["v_ref"]] = cfg["v_ref"]
    p[idx["w_du_a"]] = cfg["w_du_a"]
    p[idx["w_du_vs"]] = cfg["w_du_vs"]
    p[idx["e_c_ref"]] = cfg["e_c_ref"]
    p[idx["e_l_ref"]] = cfg["e_l_ref"]
    if with_slosh:
        # 占位 slosh 物理（运行时由 slosh_dynamics 核覆盖）：ω_n≈5, ζ≈0.05。
        omega_n = 5.0
        zeta = 0.05
        eta_ref = 0.01
        p[idx["two_zeta_omega_n"]] = 2.0 * zeta * omega_n
        p[idx["omega_n_sq"]] = omega_n * omega_n
        p[idx["kappa_x"]] = 1.0
        p[idx["kappa_y"]] = 1.0
        p[idx["eta_ref"]] = eta_ref
        p[idx["eta_dot_ref"]] = omega_n * eta_ref
        p[idx["w_slosh_eta"]] = cfg["w_slosh"]
        p[idx["w_slosh_eta_dot"]] = cfg["w_slosh"]
        if "eta_max_sq" in idx:
            p[idx["eta_max_sq"]] = 1e12
        if "phase_rejoin_active" in idx:
            # Off/monitor-compatible default: no nominal-relative term and no
            # empirical gate.  Unit radii keep every inactive stage finite;
            # enforce mode replaces the N_l radius vector from its artifact.
            p[idx["phase_rejoin_active"]] = 0.0
            p[idx["empirical_gate_active"]] = 0.0
            for name in (
                "gate_r_x", "gate_r_y", "gate_r_yaw", "gate_r_v",
                "gate_r_omega", "gate_r_eta_x", "gate_r_eta_x_dot",
                "gate_r_eta_y", "gate_r_eta_y_dot",
            ):
                p[idx[name]] = 1.0
    return p


def _cpp_enum(names, max_name):
    lines = []
    for index, name in enumerate(names):
        lines.append(f"    {name.upper()} = {index},")
    lines.append(f"    {max_name} = {len(names)},")
    return "\n".join(lines)


def _cpp_name_array(names, symbol):
    values = ",\n".join(f'    "{name}"' for name in names)
    return (
        f"constexpr const char* {symbol}[] = {{\n"
        f"{values}\n"
        "};")


def emit_cpp_manifest(cfg, output_root):
    """Emit the C++ parameter/dimension contract from the Python model truth.

    The wrapper includes this file directly.  It must never carry a second
    handwritten enum matching PARAM_NAMES/PARAM_NAMES_SLOSH.
    """
    os.makedirs(output_root, exist_ok=True)
    path = os.path.join(output_root, "spmpc_parameter_manifest.h")
    temporary = path + ".tmp"
    mainline_enum = _cpp_enum(PARAM_NAMES_SLOSH, "PARAM_MAX")
    legacy_enum = _cpp_enum(
        PARAM_NAMES_SLOSH_DIRECT_OMEGA, "PARAM_LEGACY_MAX")
    mainline_names = _cpp_name_array(
        PARAM_NAMES_SLOSH, "kParameterNames")
    legacy_names = _cpp_name_array(
        PARAM_NAMES_SLOSH_DIRECT_OMEGA, "kParameterNames")
    content = f"""// Generated by tools/codegen/acados/generate_spmpc_acados.py.
// Source of truth: spmpc_acados_model.py parameter-name lists and planner YAML.
// Do not edit by hand.
#pragma once

namespace spmpc_local_planner {{
namespace acados_manifest {{

namespace mainline {{
enum ParameterIndex : int {{
{mainline_enum}
}};
constexpr int kB0ParameterCount = {len(PARAM_NAMES)};
constexpr int kSloshParameterCount = {len(PARAM_NAMES_SLOSH)};
constexpr int kB0StateCount = 6;
constexpr int kSloshStateCount = 10;
constexpr int kControlCount = 3;
constexpr int kSloshNonlinearConstraintCount = 2;
{mainline_names}
}}  // namespace mainline

namespace direct_omega_legacy {{
enum ParameterIndex : int {{
{legacy_enum}
}};
constexpr int kB0ParameterCount = {len(PARAM_NAMES_DIRECT_OMEGA_LEGACY)};
constexpr int kSloshParameterCount = {len(PARAM_NAMES_SLOSH_DIRECT_OMEGA)};
constexpr int kB0StateCount = 5;
constexpr int kSloshStateCount = 9;
constexpr int kControlCount = 3;
{legacy_names}
}}  // namespace direct_omega_legacy

namespace generated_bounds {{
constexpr double kVMax = {float(cfg['v_max']):.17g};
constexpr double kOmegaMax = {float(cfg['omega_max']):.17g};
constexpr double kAMax = {float(cfg['a_max']):.17g};
constexpr double kAlphaMax = {float(cfg['alpha_max']):.17g};
constexpr double kVsMax = {float(cfg['vs_max']):.17g};
constexpr int kMainHorizonSteps = {int(cfg['N'])};
constexpr int kPhaseRejoinHorizonSteps = {int(cfg['phase_rejoin_N'])};
}}  // namespace generated_bounds

}}  // namespace acados_manifest
}}  // namespace spmpc_local_planner
"""
    with open(temporary, "w", encoding="utf-8") as stream:
        stream.write(content)
    os.replace(temporary, path)
    print(f"[ok] C++ parameter manifest generated -> {path}")
    return path


def build_check(cfg, model_key):
    """不依赖 acados：装配 CasADi 符号与代价表达式并打印维度，做语法/形状自检。"""
    import casadi as ca

    model_cfg = MODELS[model_key]
    sym = model_cfg["export"]()
    stage = stage_cost_expr(sym, cfg)
    terminal = terminal_cost_expr(sym, cfg)
    p_default = default_parameter_values(
        cfg, model_cfg["with_slosh"], model_cfg.get("direct_omega_legacy", False))

    print(f"[check] 模型 '{model_key}' ({sym['name']}) 装配成功")
    print(f"  nx={sym['nx']} nu={sym['nu']} np={sym['np']} (= len(param_default)={len(p_default)})")
    print(f"  f_expl shape = {sym['f_expl'].shape}")
    print(f"  stage_cost   shape = {ca.SX(stage).shape}")
    print(f"  terminal_cost shape = {ca.SX(terminal).shape}")
    print(f"  N={cfg['N']} dt={cfg['dt']} Tf={cfg['Tf']}")
    print(f"  bounds: v_max={cfg['v_max']} omega_max={cfg['omega_max']} a_max={cfg['a_max']} vs_max={cfg['vs_max']}")
    print(f"  e_c_ref={cfg['e_c_ref']:.4f} e_l_ref={cfg['e_l_ref']:.4f}")
    print(f"  path-speed: w_progress={cfg['w_progress']:.4f} w_v={cfg['w_v']:.4f} w_vs={cfg['w_vs']:.4f} v_ref={cfg['v_ref']:.4f}")
    if model_key in ("slosh", "phase_rejoin"):
        h_expr = slosh_nonlinear_constraint_expr(sym, PIDX_SLOSH)
        if h_expr.shape != (2, 1):
            raise RuntimeError(
                f"slosh nonlinear constraint shape must be (2, 1), got {h_expr.shape}")
        x0 = np.zeros(sym["nx"])
        h_fun = ca.Function("spmpc_slosh_h_check", [sym["x"], sym["p"]], [h_expr])
        inactive = np.asarray(h_fun(x0, p_default)).reshape(-1)
        if not np.isfinite(inactive).all() or abs(inactive[1] + 1.0) > 1e-12:
            raise RuntimeError(
                f"inactive empirical gate must be finite and equal -1, got {inactive[1]}")

        p_active = p_default.copy()
        p_active[PIDX_SLOSH["empirical_gate_active"]] = 1.0
        p_active[PIDX_SLOSH["nom_x"]] = 0.0
        boundary_x = x0.copy()
        boundary_x[0] = p_active[PIDX_SLOSH["gate_r_x"]]
        boundary = np.asarray(h_fun(boundary_x, p_active)).reshape(-1)
        if not np.isfinite(boundary).all() or abs(boundary[1]) > 1e-12:
            raise RuntimeError(
                f"one-radius empirical gate boundary must equal 0, got {boundary[1]}")
        print("  nonlinear h shape = (2, 1): slosh cap + stage-selective empirical gate")
        print("  empirical gate self-check: inactive=-1, one-radius boundary=0")


def generate(cfg, output_root, model_key):
    try:
        from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
    except ImportError:
        sys.stderr.write(
            "[err] 未找到 acados_template。请先安装 acados 并设置 ACADOS_SOURCE_DIR，\n"
            "      或先用 `--check` 验证 CasADi 模型（不依赖 acados）。\n")
        return 2

    model_cfg = MODELS[model_key]
    with_slosh = model_cfg["with_slosh"]
    direct_omega_legacy = model_cfg.get("direct_omega_legacy", False)
    sym = model_cfg["export"]()
    cfg["nx"] = sym["nx"]

    model = AcadosModel()
    model.name = sym["name"]
    model.x = sym["x"]
    model.u = sym["u"]
    model.p = sym["p"]
    model.xdot = sym["xdot"]
    model.f_expl_expr = sym["f_expl"]
    model.f_impl_expr = sym["f_impl"]

    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = cfg["N"]
    ocp.solver_options.tf = cfg["Tf"]

    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"
    ocp.model.cost_expr_ext_cost = stage_cost_expr(sym, cfg)
    ocp.model.cost_expr_ext_cost_e = terminal_cost_expr(sym, cfg)

    ocp.parameter_values = default_parameter_values(cfg, with_slosh, direct_omega_legacy)
    if direct_omega_legacy:
        set_constraints_direct_omega_legacy(ocp, cfg)
    elif with_slosh:
        set_constraints_slosh(ocp, cfg, PIDX_SLOSH)
    else:
        set_constraints(ocp, cfg)

    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "EXACT"
    ocp.solver_options.regularize_method = "PROJECT"
    ocp.solver_options.levenberg_marquardt = 1e-3
    ocp.solver_options.nlp_solver_type = "SQP_RTI"

    export_dir = os.path.join(output_root, sym["name"])
    os.makedirs(export_dir, exist_ok=True)
    ocp.code_gen_opts.code_export_directory = export_dir
    json_path = os.path.join(export_dir, f"acados_ocp_{sym['name']}.json")

    AcadosOcpSolver(ocp, json_file=json_path)
    emit_cpp_manifest(cfg, output_root)
    print(f"[ok] acados 求解器 '{model_key}' 已生成 -> {export_dir}")
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", choices=list(MODELS.keys()), default="b0",
                        help="生成哪个模型：b0（6维alpha-state）| slosh（10维alpha-state）| "
                             "b0_direct_omega_legacy（5维诊断）| slosh_direct_omega（9维诊断）")
    parser.add_argument("--check", action="store_true",
                        help="不依赖 acados，仅校验 CasADi 模型/代价装配")
    parser.add_argument("--emit-manifest-only", action="store_true",
                        help="仅从Python模型与YAML生成C++参数/边界manifest")
    parser.add_argument("--output-dir", default=os.path.join(PKG_DIR, "generated", "acados"),
                        help="codegen 输出根目录（默认 <pkg>/generated/acados）")
    args = parser.parse_args()

    cfg = load_config()
    if MODELS[args.model].get("short_phase_horizon", False):
        cfg["N"] = cfg["phase_rejoin_N"]
        cfg["Tf"] = cfg["dt"] * cfg["N"]
    if args.emit_manifest_only:
        emit_cpp_manifest(cfg, args.output_dir)
        return 0
    if args.check:
        build_check(cfg, args.model)
        return 0
    return generate(cfg, args.output_dir, args.model)


if __name__ == "__main__":
    sys.exit(main())
