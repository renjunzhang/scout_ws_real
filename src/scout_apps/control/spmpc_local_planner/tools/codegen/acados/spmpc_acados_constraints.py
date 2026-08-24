"""SPMPC 连续 MPCC —— acados 约束（控制、状态和 slosh hard cap）。

只设置 acados ocp.constraints 的边界数组（numpy），不 import acados_template。
B0 alpha-state 主线只做简单 bound：
  控制: a∈[-a_max,a_max], alpha∈[-alpha_max,alpha_max], v_s∈[0,vs_max]
  状态: v∈[0,v_max], omega∈[-omega_max,omega_max]
direct-omega 诊断模型使用单独的 direct-omega bounds；slosh hard variant 追加模态高度上限。
obstacle / costmap / hard corridor 不在 B0 引入。
"""

import casadi as ca
import numpy as np


def slosh_nonlinear_constraint_expr(sym, pidx):
    """Return [modal cap, empirical gate, independent BT phase window].

    The second constraint is stage-selective through empirical_gate_active.
    For inactive stages it evaluates to -1 (strictly feasible) instead of 0,
    avoiding an always-active zero-Jacobian inequality.  With activation=1 it
    is exactly the same nine-dimensional diagonal ellipsoid metric used by
    EmpiricalRecoveryGate in C++: metric - 1 <= 0, including wrapped yaw.

    This is an empirical gate only; it is not a robust invariant set or a
    recovery certificate.
    """
    x = sym["x"]
    p = sym["p"]

    eta_x = x[6]
    eta_y = x[8]
    eta_max_sq = p[pidx["eta_max_sq"]]
    h_slosh = eta_x * eta_x + eta_y * eta_y - eta_max_sq

    active = p[pidx["empirical_gate_active"]]
    errors = [
        x[0] - p[pidx["nom_x"]],
        x[1] - p[pidx["nom_y"]],
        ca.atan2(
            ca.sin(x[2] - p[pidx["nom_yaw"]]),
            ca.cos(x[2] - p[pidx["nom_yaw"]])),
        x[3] - p[pidx["nom_v"]],
        x[5] - p[pidx["nom_omega"]],
        x[6] - p[pidx["nom_eta_x"]],
        x[7] - p[pidx["nom_eta_x_dot"]],
        x[8] - p[pidx["nom_eta_y"]],
        x[9] - p[pidx["nom_eta_y_dot"]],
    ]
    radius_names = [
        "gate_r_x", "gate_r_y", "gate_r_yaw", "gate_r_v",
        "gate_r_omega", "gate_r_eta_x", "gate_r_eta_x_dot",
        "gate_r_eta_y", "gate_r_eta_y_dot",
    ]
    metric = 0.0
    for error, radius_name in zip(errors, radius_names):
        # Wrapper only injects active in {0,1}.  Blending the denominator keeps
        # inactive stages finite even before their unit default radii are set.
        radius = active * p[pidx[radius_name]] + (1.0 - active)
        metric = metric + (error / radius) ** 2
    h_empirical_gate = active * metric - 1.0

    bt_active = p[pidx["bt_reference_active"]]
    half_width = (
        bt_active * p[pidx["bt_phase_half_width"]]
        + (1.0 - bt_active))
    progress_error = (x[4] - p[pidx["nom_s"]]) / half_width
    # Inactive evaluates to -1 with a finite denominator.  Active is the hard
    # |s-nom_s| <= phase_half_width contract for the full BT clock.
    h_bt_phase_window = bt_active * progress_error ** 2 - 1.0
    return ca.vertcat(h_slosh, h_empirical_gate, h_bt_phase_window)


def set_constraints_direct_omega_legacy(ocp, cfg):
    """诊断 legacy B0：u[1]=omega 直接受限，状态只约束 v。"""
    a_max = cfg["a_max"]
    omega_max = cfg["omega_max"]
    vs_max = cfg["vs_max"]
    v_max = cfg["v_max"]

    ocp.constraints.idxbu = np.array([0, 1, 2])
    ocp.constraints.lbu = np.array([-a_max, -omega_max, 0.0])
    ocp.constraints.ubu = np.array([a_max, omega_max, vs_max])

    ocp.constraints.idxbx = np.array([3])
    ocp.constraints.lbx = np.array([0.0])
    ocp.constraints.ubx = np.array([v_max])
    ocp.constraints.x0 = np.zeros(cfg["nx"])



def set_constraints(ocp, cfg):
    a_max = cfg["a_max"]
    omega_max = cfg["omega_max"]
    vs_max = cfg["vs_max"]
    v_max = cfg["v_max"]
    alpha_max = cfg["alpha_max"]

    # 控制 bounds: u = [a, alpha, v_s]；alpha=d(omega)/dt 硬约束在 ±alpha_max。
    ocp.constraints.idxbu = np.array([0, 1, 2])
    ocp.constraints.lbu = np.array([-a_max, -alpha_max, 0.0])
    ocp.constraints.ubu = np.array([a_max, alpha_max, vs_max])

    # 状态 bound: v 是 x[3] ∈ [0, v_max]；omega 是 x[5] ∈ [-omega_max, omega_max]。
    ocp.constraints.idxbx = np.array([3, 5])
    ocp.constraints.lbx = np.array([0.0, -omega_max])
    ocp.constraints.ubx = np.array([v_max, omega_max])

    # 初始状态由 wrapper 每周期通过 set("x0", ...) 设定；
    # 这里给出占位 x0，维度需匹配 nx。
    ocp.constraints.x0 = np.zeros(cfg["nx"])



def set_constraints_slosh(ocp, cfg, pidx):
    """Mainline slosh: box bounds, height cap, and stage-selective empirical gate.

    约束写成 eta_x^2 + eta_y^2 - eta_max_sq <= 0，其中 eta_max_sq 是参数，
    运行时由 C++ 用 slosh_height_max / heightCoeff() 注入；非 hard variant 用大阈值禁用。
    stage 0 使用同一表达式但放宽上界，避免当前实测/估计液面已超阈值时立即不可行。

    第二个非线性约束由逐 stage 参数激活；wrapper 只允许它在
    k=N_l 为 1，其他 stage 为 0。
    """
    set_constraints(ocp, cfg)

    sym = {"x": ocp.model.x, "p": ocp.model.p}
    h_expr = slosh_nonlinear_constraint_expr(sym, pidx)
    ocp.model.con_h_expr = h_expr
    ocp.model.con_h_expr_e = h_expr
    ocp.model.con_h_expr_0 = h_expr

    # h_slosh <= 0 for stages 1..N and terminal.  Keep the lower bound far below
    # the disabled-runtime value (eta_max_sq=1e12 => h_slosh≈-1e12), otherwise
    # soft-only slosh variants become infeasible even though the cap is disabled.
    ocp.constraints.lh = np.array([-1e15, -1e15, -1e15])
    ocp.constraints.uh = np.array([0.0, 0.0, 0.0])
    ocp.constraints.lh_e = np.array([-1e15, -1e15, -1e15])
    ocp.constraints.uh_e = np.array([0.0, 0.0, 0.0])

    # Do not reject a cycle solely because the measured initial slosh state already
    # violates the cap; constrain predicted future nodes instead.
    ocp.constraints.lh_0 = np.array([-1e15, -1e15, -1e15])
    ocp.constraints.uh_0 = np.array([1e15, 0.0, 0.0])
