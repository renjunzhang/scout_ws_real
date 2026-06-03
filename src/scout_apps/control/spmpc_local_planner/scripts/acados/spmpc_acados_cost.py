"""SPMPC 连续 MPCC —— B0 外部代价（EXTERNAL cost，纯 CasADi）。

只产出 CasADi 代价表达式，不依赖 acados。表达式按 §4.4 口径：
  - 误差类项无量纲化（除以参考尺度）；
  - 所有逐步累加项除以 N，保证不同 horizon / 与 primitive 后端权重可迁移；
  - 进度奖励单独处理（对 v_s 的负向奖励，推动 s 前进）。

contour / lag 用 s 的参考多项式解析计算（局部 MPCC）。
"""

import casadi as ca

from spmpc_acados_model import PIDX, PIDX_SLOSH


def _reference_terms(x, p):
    """返回 (x_ref, y_ref, phi_ref)：参考点与参考切向，均为 s 的函数。"""
    s = x[4]
    rx = [p[PIDX["rx0"]], p[PIDX["rx1"]], p[PIDX["rx2"]], p[PIDX["rx3"]]]
    ry = [p[PIDX["ry0"]], p[PIDX["ry1"]], p[PIDX["ry2"]], p[PIDX["ry3"]]]

    x_ref = rx[0] + rx[1] * s + rx[2] * s * s + rx[3] * s * s * s
    y_ref = ry[0] + ry[1] * s + ry[2] * s * s + ry[3] * s * s * s
    dx_ref = rx[1] + 2.0 * rx[2] * s + 3.0 * rx[3] * s * s
    dy_ref = ry[1] + 2.0 * ry[2] * s + 3.0 * ry[3] * s * s
    phi_ref = ca.atan2(dy_ref, dx_ref)
    return x_ref, y_ref, phi_ref


def _tracking_cost(x, p):
    """contour + lag 跟踪代价（无量纲化）。"""
    px, py = x[0], x[1]
    x_ref, y_ref, phi_ref = _reference_terms(x, p)

    # Liniger contour/lag 投影
    e_contour = ca.sin(phi_ref) * (px - x_ref) - ca.cos(phi_ref) * (py - y_ref)
    e_lag = -ca.cos(phi_ref) * (px - x_ref) - ca.sin(phi_ref) * (py - y_ref)

    e_c_ref = p[PIDX["e_c_ref"]]
    e_l_ref = p[PIDX["e_l_ref"]]
    j_contour = p[PIDX["w_contour"]] * (e_contour / e_c_ref) ** 2
    j_lag = p[PIDX["w_lag"]] * (e_lag / e_l_ref) ** 2
    return j_contour + j_lag


def _slosh_cost(x, p):
    """液体模态代价（仅 9 维 slosh 模型）：η/η̇ 无量纲化（§4.4）。"""
    eta_x, eta_x_dot = x[5], x[6]
    eta_y, eta_y_dot = x[7], x[8]
    eta_ref = p[PIDX_SLOSH["eta_ref"]]
    eta_dot_ref = p[PIDX_SLOSH["eta_dot_ref"]]
    j_eta = p[PIDX_SLOSH["w_slosh_eta"]] * (eta_x * eta_x + eta_y * eta_y) / (eta_ref * eta_ref)
    j_eta_dot = p[PIDX_SLOSH["w_slosh_eta_dot"]] * \
        (eta_x_dot * eta_x_dot + eta_y_dot * eta_y_dot) / (eta_dot_ref * eta_dot_ref)
    return j_eta + j_eta_dot


def stage_cost_expr(sym, cfg):
    """stage k 的 EXTERNAL 代价表达式。"""
    x = sym["x"]
    u = sym["u"]
    p = sym["p"]
    a, omega, v_s = u[0], u[1], u[2]

    a_max = cfg["a_max"]
    omega_max = cfg["omega_max"]
    vs_max = cfg["vs_max"]
    n_steps = float(cfg["N"])

    j_track = _tracking_cost(x, p)

    # 进度奖励：推动 v_s 前进（负向，单独处理）
    j_progress = -p[PIDX["w_progress"]] * (v_s / vs_max)

    # 控制幅值
    j_control = (
        p[PIDX["w_a"]] * (a / a_max) ** 2
        + p[PIDX["w_omega"]] * (omega / omega_max) ** 2
        + p[PIDX["w_vs"]] * (v_s / vs_max) ** 2
    )

    # 控制变化率：相对 u_prev。w_du_* 仅在 stage 0 由 wrapper 置非零，
    # 实现跨周期第一帧连续性（§4.5）；horizon 内 k>0 的 Δu 平滑见模块说明。
    du_a = (a - p[PIDX["a_prev"]]) / a_max
    du_omega = (omega - p[PIDX["omega_prev"]]) / omega_max
    du_vs = (v_s - p[PIDX["vs_prev"]]) / vs_max
    j_smooth = (
        p[PIDX["w_du_a"]] * du_a ** 2
        + p[PIDX["w_du_omega"]] * du_omega ** 2
        + p[PIDX["w_du_vs"]] * du_vs ** 2
    )

    j_slosh = _slosh_cost(x, p) if sym.get("with_slosh") else 0.0
    return (j_track + j_progress + j_control + j_smooth + j_slosh) / n_steps


def terminal_cost_expr(sym, cfg):
    """stage N 的 EXTERNAL 终端代价：跟踪 + 残余模态能量（§5.4），无控制项。"""
    x = sym["x"]
    p = sym["p"]
    j = _tracking_cost(x, p)
    if sym.get("with_slosh"):
        j = j + _slosh_cost(x, p)
    return j
