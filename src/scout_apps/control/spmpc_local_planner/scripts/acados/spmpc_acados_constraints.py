"""SPMPC 连续 MPCC —— B0 约束（控制与状态 bounds）。

只设置 acados ocp.constraints 的边界数组（numpy），不 import acados_template。
B0 第一版只做简单 bound（方案 §7 Phase C）：
  控制: a∈[-a_max,a_max], omega∈[-omega_max,omega_max], v_s∈[0,vs_max]
  状态: v∈[0,v_max]
obstacle / costmap / hard corridor 不在 B0 引入。
"""

import casadi as ca
import numpy as np


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
    """Mainline alpha-state slosh: box bounds + predicted slosh-height hard cap.

    约束写成 eta_x^2 + eta_y^2 - eta_max_sq <= 0，其中 eta_max_sq 是参数，
    运行时由 C++ 用 slosh_height_max / heightCoeff() 注入；非 hard variant 用大阈值禁用。
    stage 0 使用同一表达式但放宽上界，避免当前实测/估计液面已超阈值时立即不可行。
    """
    set_constraints(ocp, cfg)

    x = ocp.model.x
    p = ocp.model.p
    eta_x = x[6]
    eta_y = x[8]
    eta_max_sq = p[pidx["eta_max_sq"]]
    h_slosh = ca.vertcat(eta_x * eta_x + eta_y * eta_y - eta_max_sq)

    ocp.model.con_h_expr = h_slosh
    ocp.model.con_h_expr_e = h_slosh
    ocp.model.con_h_expr_0 = h_slosh

    # h_slosh <= 0 for stages 1..N and terminal.  Keep the lower bound far below
    # the disabled-runtime value (eta_max_sq=1e12 => h_slosh≈-1e12), otherwise
    # soft-only slosh variants become infeasible even though the cap is disabled.
    ocp.constraints.lh = np.array([-1e15])
    ocp.constraints.uh = np.array([0.0])
    ocp.constraints.lh_e = np.array([-1e15])
    ocp.constraints.uh_e = np.array([0.0])

    # Do not reject a cycle solely because the measured initial slosh state already
    # violates the cap; constrain predicted future nodes instead.
    ocp.constraints.lh_0 = np.array([-1e15])
    ocp.constraints.uh_0 = np.array([1e15])
