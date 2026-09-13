"""SPMPC 连续 MPCC —— acados 外部代价（EXTERNAL cost，纯 CasADi）。

只产出 CasADi 代价表达式，不依赖 acados。覆盖显式执行器主线和
RouteB direct-omega 诊断模型。主线采用：
  - 误差类项无量纲化（除以参考尺度）；
  - running 项除以 N、终端项计一次，acados scaling 显式设为 1；
  - 速度/路径进度单独处理（v_s 负向奖励 + v/v_s 对 v_ref 的跟踪，避免弯处 creep）。

contour / lag 用 s 的参考多项式解析计算（局部 MPCC）。
"""

import casadi as ca

from spmpc_acados_model import (
    ACCEL_MEMORY_INDEX, LINEAR_QUEUE_START, ANGULAR_QUEUE_START,
    LINEAR_DELAY_STEPS, ANGULAR_DELAY_STEPS,
    PIDX, PIDX_SLOSH, PIDX_DIRECT_OMEGA_LEGACY, PIDX_SLOSH_DIRECT_OMEGA,
)


def _reference_terms(x, p, pidx=PIDX):
    """返回 (x_ref, y_ref, phi_ref)：参考点与参考切向，均为 s 的函数。"""
    s = x[4]
    rx = [p[pidx["rx0"]], p[pidx["rx1"]], p[pidx["rx2"]], p[pidx["rx3"]]]
    ry = [p[pidx["ry0"]], p[pidx["ry1"]], p[pidx["ry2"]], p[pidx["ry3"]]]

    x_ref = rx[0] + rx[1] * s + rx[2] * s * s + rx[3] * s * s * s
    y_ref = ry[0] + ry[1] * s + ry[2] * s * s + ry[3] * s * s * s
    dx_ref = rx[1] + 2.0 * rx[2] * s + 3.0 * rx[3] * s * s
    dy_ref = ry[1] + 2.0 * ry[2] * s + 3.0 * ry[3] * s * s
    phi_ref = ca.atan2(dy_ref, dx_ref)
    return x_ref, y_ref, phi_ref


def _tracking_cost(x, p, pidx=PIDX):
    """contour + lag 跟踪代价（无量纲化）。"""
    px, py = x[0], x[1]
    x_ref, y_ref, phi_ref = _reference_terms(x, p, pidx)

    # Liniger contour/lag 投影
    e_contour = ca.sin(phi_ref) * (px - x_ref) - ca.cos(phi_ref) * (py - y_ref)
    e_lag = -ca.cos(phi_ref) * (px - x_ref) - ca.sin(phi_ref) * (py - y_ref)

    e_c_ref = p[pidx["e_c_ref"]]
    e_l_ref = p[pidx["e_l_ref"]]
    j_contour = p[pidx["w_contour"]] * (e_contour / e_c_ref) ** 2
    j_lag = p[pidx["w_lag"]] * (e_lag / e_l_ref) ** 2
    return j_contour + j_lag


def _curvature_limited_vref(x, p, cfg, pidx=PIDX):
    """曲率自适应参考速度（纯几何，不引入新参数）。

    用已有的参考多项式系数(rx*/ry*)解析出曲率 kappa(s)，按角速度上限给参考限速:
      物理上跟踪曲率 kappa 的路径需要 omega = v*kappa <= omega_max，故 v <= omega_max/|kappa|。
    平滑形式  v_eff = v_cruise / sqrt(1 + (v_cruise*|kappa|/omega_max)^2):
      kappa->0 -> v_cruise; kappa 很大 -> omega_max/|kappa|。
    再夹一个下限(0.3*v_cruise)，保证弯中 v_ref 仍 >0、anti-creep 不失效。

    只读 cost 模块已有的参数与 cfg 常量，不改参数向量契约、不碰 wrapper。
    """
    s = x[4]
    rx1, rx2, rx3 = p[pidx["rx1"]], p[pidx["rx2"]], p[pidx["rx3"]]
    ry1, ry2, ry3 = p[pidx["ry1"]], p[pidx["ry2"]], p[pidx["ry3"]]
    dx = rx1 + 2.0 * rx2 * s + 3.0 * rx3 * s * s
    dy = ry1 + 2.0 * ry2 * s + 3.0 * ry3 * s * s
    ddx = 2.0 * rx2 + 6.0 * rx3 * s
    ddy = 2.0 * ry2 + 6.0 * ry3 * s
    speed_sq = dx * dx + dy * dy + 1e-6
    kappa = (dx * ddy - dy * ddx) / speed_sq ** 1.5
    v_cruise = p[pidx["v_ref"]]
    omega_max = cfg["omega_max"]
    ratio = v_cruise * ca.fabs(kappa) / omega_max
    v_eff = v_cruise / ca.sqrt(1.0 + ratio * ratio)
    return ca.fmax(0.3 * v_cruise, v_eff)


def _path_speed_cost(x, u, p, cfg, pidx=PIDX):
    """仅供 legacy：固定参考速度下的进度奖励与物理/虚拟速度跟踪。"""
    v = x[3]
    v_s = u[2]
    v_max = cfg["v_max"]
    vs_max = cfg["vs_max"]
    v_ref = p[pidx["v_ref"]]
    j_progress = -p[pidx["w_progress"]] * (v_s / vs_max)
    j_v = p[pidx["w_v"]] * ((v - v_ref) / v_max) ** 2
    j_vs = p[pidx["w_vs"]] * ((v_s - v_ref) / vs_max) ** 2
    return j_progress + j_v + j_vs


def _slosh_cost(x, p, pidx_slosh=PIDX_SLOSH, eta_base=6):
    """液体模态代价：η/η̇ 无量纲化（§4.4）。

    eta_base 指定 eta 在状态向量里的起始下标：
      explicit-actuator mainline -> sym["eta_base"]；direct-omega(9 维) -> 5。
    """
    eta_x, eta_x_dot = x[eta_base], x[eta_base + 1]
    eta_y, eta_y_dot = x[eta_base + 2], x[eta_base + 3]
    eta_ref = p[pidx_slosh["eta_ref"]]
    eta_dot_ref = p[pidx_slosh["eta_dot_ref"]]
    j_eta = p[pidx_slosh["w_slosh_eta"]] * (eta_x * eta_x + eta_y * eta_y) / (eta_ref * eta_ref)
    j_eta_dot = p[pidx_slosh["w_slosh_eta_dot"]] * \
        (eta_x_dot * eta_x_dot + eta_y_dot * eta_y_dot) / (eta_dot_ref * eta_dot_ref)
    return j_eta + j_eta_dot


def stage_cost_expr_direct_omega_legacy(sym, cfg):
    """direct-omega B0/slosh 的 stage EXTERNAL 代价（RouteB 诊断/legacy）。

    omega 是直接控制；with_slosh 时液体模态在 x[5..8]，参数用 PIDX_SLOSH_DIRECT_OMEGA。
    转向 chatter 不在此硬约束，由 wrapper 出口 omega-rate 限幅压制。
    """
    x = sym["x"]
    u = sym["u"]
    p = sym["p"]
    a, omega, v_s = u[0], u[1], u[2]

    a_max = cfg["a_max"]
    omega_max = cfg["omega_max"]
    vs_max = cfg["vs_max"]
    n_steps = float(cfg["N"])
    with_slosh = sym.get("with_slosh", False)
    pidx = PIDX_SLOSH_DIRECT_OMEGA if with_slosh else PIDX_DIRECT_OMEGA_LEGACY

    j_track = _tracking_cost(x, p, pidx)
    j_path_speed = _path_speed_cost(x, u, p, cfg, pidx)

    j_control = (
        p[pidx["w_a"]] * (a / a_max) ** 2
        + p[pidx["w_omega"]] * (omega / omega_max) ** 2
    )

    du_a = (a - p[pidx["a_prev"]]) / a_max
    du_omega = (omega - p[pidx["omega_prev"]]) / omega_max
    du_vs = (v_s - p[pidx["vs_prev"]]) / vs_max
    j_smooth = (
        p[pidx["w_du_a"]] * du_a ** 2
        + p[pidx["w_du_omega"]] * du_omega ** 2
        + p[pidx["w_du_vs"]] * du_vs ** 2
    )

    j_slosh = _slosh_cost(x, p, PIDX_SLOSH_DIRECT_OMEGA, 5) if with_slosh else 0.0

    return (j_track + j_path_speed + j_control + j_smooth + j_slosh) / n_steps



def terminal_cost_expr_direct_omega_legacy(sym, cfg):
    """direct-omega B0/slosh 的终端代价：跟踪 +（slosh 时）残余模态能量。"""
    with_slosh = sym.get("with_slosh", False)
    pidx = PIDX_SLOSH_DIRECT_OMEGA if with_slosh else PIDX_DIRECT_OMEGA_LEGACY
    j = _tracking_cost(sym["x"], sym["p"], pidx)
    if with_slosh:
        j = j + _slosh_cost(sym["x"], sym["p"], PIDX_SLOSH_DIRECT_OMEGA, 5)
    return j



# Stable component ABI for generated C diagnostics. Terminal terms are evaluated
# separately; slack/stop have their own totals including terminal contributions.
COST_COMPONENT_NAMES = ("contour", "lag", "progress", "v_actual", "v_s",
                        "anti_creep", "control", "smooth", "slosh_eta", "slosh_eta_dot", "slack", "stop")


def cost_components(sym, cfg, terminal=False):
    """Single definition consumed by both the objective and runtime diagnostics."""
    x, u, p = sym["x"], sym["u"], sym["p"]
    idx = PIDX_SLOSH if sym.get("with_slosh") else PIDX
    rx, ry, phi = _reference_terms(x, p, idx)
    ec = ca.sin(phi) * (x[0] - rx) - ca.cos(phi) * (x[1] - ry)
    el = -ca.cos(phi) * (x[0] - rx) - ca.sin(phi) * (x[1] - ry)
    terms = [p[idx["w_contour"]] * (ec / p[idx["e_c_ref"]])**2,
             p[idx["w_lag"]] * (el / p[idx["e_l_ref"]])**2] + [ca.SX(0)] * 10
    if sym.get("with_slosh"):
        b = sym["eta_base"]
        terms[8] = p[idx["w_slosh_eta"]] * (x[b]**2 + x[b+2]**2) / p[idx["eta_ref"]]**2
        terms[9] = p[idx["w_slosh_eta_dot"]] * (x[b+1]**2 + x[b+3]**2) / p[idx["eta_dot_ref"]]**2
        # Eliminate epsilon analytically: the minimizing nonnegative slack is
        # max(||eta||^2 - target^2, 0). A separate hard cap bounds its budget.
        # Positive linear/quadratic penalties make this equivalent to an explicit
        # bounded slack variable, while retaining the existing state/control ABI.
        slack = ca.fmax(0, x[b]**2 + x[b+2]**2 - p[idx["eta_target_sq"]]) / p[idx["eta_ref"]]**2
        terms[10] = p[idx["slack_linear_weight"]]*slack + p[idx["slack_quadratic_weight"]]*slack**2
    remaining = ca.fmax(0, p[idx["stop_goal_s"]] - x[4])
    brake = p[idx["stop_brake_accel"]]
    delay = p[idx["stop_delay_margin"]]
    # Includes a delay/actuator/jerk margin, with finite derivatives at rest.
    stop_v = ca.sqrt((brake*delay)**2 + 2*brake*remaining) - brake*delay
    nominal_vref = _curvature_limited_vref(x, p, cfg, idx)
    stopping = p[idx["stop_active"]]
    vref = (1-stopping)*nominal_vref + stopping*ca.fmin(nominal_vref, stop_v)
    near_goal = stopping*ca.fmax(0, 1-remaining/ca.fmax(.1, p[idx["v_ref"]]*delay))
    # Clear command/actual motion and the delayed command tail at the true goal.
    stop_energy = (x[3]**2+x[6]**2)/cfg["v_max"]**2 + (x[5]**2+x[7]**2)/cfg["omega_max"]**2
    stop_energy += ca.sumsqr(x[LINEAR_QUEUE_START:ANGULAR_QUEUE_START])/(LINEAR_DELAY_STEPS*cfg["v_max"]**2) + ca.sumsqr(x[ANGULAR_QUEUE_START:ACCEL_MEMORY_INDEX])/(ANGULAR_DELAY_STEPS*cfg["omega_max"]**2)
    stop_energy += (x[ACCEL_MEMORY_INDEX]/cfg["a_max"])**2
    terms[11] = p[idx["stop_velocity_weight"]]*near_goal*stop_energy
    if sym.get("with_slosh"):
        b=sym["eta_base"]
        residual_energy = x[b]**2+x[b+2]**2+(x[b+1]**2+x[b+3]**2)/p[idx["omega_n_sq"]]
        terms[11] += near_goal*p[idx["w_slosh_eta"]]*residual_energy/p[idx["eta_ref"]]**2
    if not terminal:
        terms[2] = -p[idx["w_progress"]] * u[2] / cfg["vs_max"] * ((1-stopping) + stopping*ca.fmin(1, stop_v/ca.fmax(1e-6,nominal_vref)))
        terms[3] = p[idx["w_v"]] * ((x[3] - vref) / cfg["v_max"])**2
        terms[4] = p[idx["w_vs"]] * ((u[2] - vref) / cfg["vs_max"])**2
        terms[5] = p[idx["anticreep_gain"]] * p[idx["w_v"]] * (
            (ca.fmax(0, vref-x[3]) / cfg["v_max"])**2 +
            (ca.fmax(0, vref-u[2]) / cfg["vs_max"])**2)
        terms[6] = (p[idx["w_a"]] * (u[0]/cfg["a_max"])**2 +
                    p[idx["w_omega"]] * (x[5]/cfg["omega_max"])**2 +
                    p[idx["w_alpha"]] * (u[1]/cfg["alpha_max"])**2)
        terms[7] = (p[idx["w_du_a"]] * ((u[0]-x[ACCEL_MEMORY_INDEX])/cfg["a_max"])**2 +
                    p[idx["w_du_vs"]] * ((u[2]-p[idx["vs_prev"]])/cfg["vs_max"])**2)
    # Average running cost + one terminal cost. acados scaling is explicitly 1.
    return ca.vertcat(*terms) / (1. if terminal else float(cfg["N"]))


def stage_cost_expr(sym, cfg):
    if sym.get("direct_omega_legacy"):
        return stage_cost_expr_direct_omega_legacy(sym, cfg)
    return ca.sum1(cost_components(sym, cfg))


def terminal_cost_expr(sym, cfg):
    if sym.get("direct_omega_legacy"):
        return terminal_cost_expr_direct_omega_legacy(sym, cfg)
    return ca.sum1(cost_components(sym, cfg, terminal=True))
