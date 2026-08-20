"""SPMPC 连续 MPCC —— acados 外部代价（EXTERNAL cost，纯 CasADi）。

只产出 CasADi 代价表达式，不依赖 acados。覆盖 alpha-state 主线和
RouteB direct-omega 诊断模型。表达式按 §4.4 口径：
  - 误差类项无量纲化（除以参考尺度）；
  - 所有逐步累加项除以 N，保证不同 horizon / 与 primitive 后端权重可迁移；
  - 速度/路径进度单独处理（v_s 负向奖励 + v/v_s 对 v_ref 的跟踪，避免弯处 creep）。

contour / lag 用 s 的参考多项式解析计算（局部 MPCC）。
"""

import casadi as ca

from spmpc_acados_model import (
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


def _path_speed_cost(x, u, p, cfg, pidx=PIDX, curvature_aware=False, anticreep=False):
    """物理速度/虚拟路径进度速度代价：线性进度奖励 + v_ref 跟踪惩罚。

    curvature_aware=True 时 v_ref 随路径曲率自适应（弯前预减速、弯中仍 >0）。
    anticreep=True 时对 "v / v_s 低于 v_ref 的亏空" 额外重罚（非对称 relu，仅惩罚低于侧）：
      使 "塌到 v_ref 以下" 比 "弯处那点 contour 误差" 更贵，从源头堵住停滞；
      高于 v_ref 不额外罚（上侧仍由 j_v/j_vs 处理）。权重复用 w_v × 常量增益，不引入新参数。
    两个开关默认 False，legacy 诊断后端调用不传 → 行为字节级不变。
    """
    v = x[3]
    v_s = u[2]
    v_max = cfg["v_max"]
    vs_max = cfg["vs_max"]
    v_ref = _curvature_limited_vref(x, p, cfg, pidx) if curvature_aware else p[pidx["v_ref"]]
    j_progress = -p[pidx["w_progress"]] * (v_s / vs_max)
    j_v = p[pidx["w_v"]] * ((v - v_ref) / v_max) ** 2
    j_vs = p[pidx["w_vs"]] * ((v_s - v_ref) / vs_max) ** 2
    cost = j_progress + j_v + j_vs
    if anticreep:
        gain = cfg.get("anticreep_gain", 8.0)   # 亏空相对 w_v 的放大倍数（codegen 常量，可调）
        short_v = ca.fmax(0.0, v_ref - v) / v_max
        short_vs = ca.fmax(0.0, v_ref - v_s) / vs_max
        cost = cost + gain * p[pidx["w_v"]] * (short_v * short_v + short_vs * short_vs)
    return cost


def _slosh_cost(x, p, pidx_slosh=PIDX_SLOSH, eta_base=6):
    """液体模态代价：η/η̇ 无量纲化（§4.4）。

    eta_base 指定 eta 在状态向量里的起始下标：
      omega-state(10 维) -> 6；direct-omega(9 维) -> 5。

    phase_rejoin_active=1 时改为相对 j_f+k 名义液体状态的偏差代价；
    为 0 时精确保留原点模态能量代价。direct-omega 后端没有该参数，
    因而仍保持原行为。
    """
    eta_x, eta_x_dot = x[eta_base], x[eta_base + 1]
    eta_y, eta_y_dot = x[eta_base + 2], x[eta_base + 3]
    if "phase_rejoin_active" in pidx_slosh:
        active = p[pidx_slosh["phase_rejoin_active"]]
        eta_x = eta_x - active * p[pidx_slosh["nom_eta_x"]]
        eta_x_dot = eta_x_dot - active * p[pidx_slosh["nom_eta_x_dot"]]
        eta_y = eta_y - active * p[pidx_slosh["nom_eta_y"]]
        eta_y_dot = eta_y_dot - active * p[pidx_slosh["nom_eta_y_dot"]]
    eta_ref = p[pidx_slosh["eta_ref"]]
    eta_dot_ref = p[pidx_slosh["eta_dot_ref"]]
    j_eta = p[pidx_slosh["w_slosh_eta"]] * (eta_x * eta_x + eta_y * eta_y) / (eta_ref * eta_ref)
    j_eta_dot = p[pidx_slosh["w_slosh_eta_dot"]] * \
        (eta_x_dot * eta_x_dot + eta_y_dot * eta_y_dot) / (eta_dot_ref * eta_dot_ref)
    return j_eta + j_eta_dot


def _phase_rejoin_relative_cost(x, u, p, cfg):
    """Alpha-state slosh 主线的 nominal-relative 状态/控制偏差。

    该项只在 wrapper 将 phase_rejoin_active 显式设为 1 的短窗口 stage
    生效；使用现有权重和物理尺度，不引入第二套未标定权重。
    几何位置仍由 MPCC contour/lag 项负责；nom_x/y/yaw 仅用于
    stage N_l 的 9 维经验 gate。
    """
    active = p[PIDX_SLOSH["phase_rejoin_active"]]
    a, alpha, v_s = u[0], u[1], u[2]
    v, omega = x[3], x[5]
    j_state = (
        p[PIDX_SLOSH["w_v"]] *
        ((v - p[PIDX_SLOSH["nom_v"]]) / cfg["v_max"]) ** 2
        + p[PIDX_SLOSH["w_omega"]] *
        ((omega - p[PIDX_SLOSH["nom_omega"]]) / cfg["omega_max"]) ** 2
    )
    j_control = (
        p[PIDX_SLOSH["w_a"]] *
        ((a - p[PIDX_SLOSH["nom_a"]]) / cfg["a_max"]) ** 2
        + p[PIDX_SLOSH["w_alpha"]] *
        ((alpha - p[PIDX_SLOSH["nom_alpha"]]) / cfg["alpha_max"]) ** 2
        + p[PIDX_SLOSH["w_vs"]] *
        ((v_s - p[PIDX_SLOSH["nom_v_s"]]) / cfg["vs_max"]) ** 2
    )
    return active * (j_state + j_control)


def _phase_rejoin_terminal_relative_cost(x, p, cfg):
    """N_l 与整个 OCP 终端重合时的 v/omega 名义偏差项。"""
    active = p[PIDX_SLOSH["phase_rejoin_active"]]
    return active * (
        p[PIDX_SLOSH["w_v"]] *
        ((x[3] - p[PIDX_SLOSH["nom_v"]]) / cfg["v_max"]) ** 2
        + p[PIDX_SLOSH["w_omega"]] *
        ((x[5] - p[PIDX_SLOSH["nom_omega"]]) / cfg["omega_max"]) ** 2
    )


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



def stage_cost_expr(sym, cfg):
    """stage k 的 EXTERNAL 代价表达式。"""
    if sym.get("direct_omega_legacy"):
        return stage_cost_expr_direct_omega_legacy(sym, cfg)
    x = sym["x"]
    u = sym["u"]
    p = sym["p"]
    a, alpha, v_s = u[0], u[1], u[2]
    omega = x[5]   # omega 现在是状态

    a_max = cfg["a_max"]
    omega_max = cfg["omega_max"]
    vs_max = cfg["vs_max"]
    alpha_max = cfg["alpha_max"]
    n_steps = float(cfg["N"])

    j_track = _tracking_cost(x, p)
    j_path_speed = _path_speed_cost(x, u, p, cfg, curvature_aware=True, anticreep=True)  # 曲率v_ref + 非对称anti-creep

    # 幅值：a 是控制，omega 是状态(转向幅值)，alpha 是转向角加速度控制；v_s 单独由 path-speed 项处理。
    # w_alpha 在所有 stage 生效 -> horizon 内 Δomega 平滑，直接抑制直道甩舵 chattering。
    j_control = (
        p[PIDX["w_a"]] * (a / a_max) ** 2
        + p[PIDX["w_omega"]] * (omega / omega_max) ** 2
        + p[PIDX["w_alpha"]] * (alpha / alpha_max) ** 2
    )

    # 控制变化率：a/v_s 相对 u_prev，仅 stage 0 由 wrapper 置非零 -> 跨周期第一帧连续性（§4.5）。
    # omega 已是状态(初值=实测 omega)，跨周期连续性由状态保证，不再需要 du_omega。
    du_a = (a - p[PIDX["a_prev"]]) / a_max
    du_vs = (v_s - p[PIDX["vs_prev"]]) / vs_max
    j_smooth = (
        p[PIDX["w_du_a"]] * du_a ** 2
        + p[PIDX["w_du_vs"]] * du_vs ** 2
    )

    j_slosh = _slosh_cost(x, p) if sym.get("with_slosh") else 0.0
    j_phase_relative = (
        _phase_rejoin_relative_cost(x, u, p, cfg)
        if sym.get("with_slosh") else 0.0
    )
    # Phase mode replaces the baseline speed/control objective instead of
    # stacking a zero-centred control prior on top of a nominal-centred prior.
    # Geometry stays active, while liquid cost already switches from the origin
    # to nominal-relative coordinates inside _slosh_cost().
    phase_active = (
        p[PIDX_SLOSH["phase_rejoin_active"]]
        if sym.get("with_slosh") else 0.0
    )
    baseline = (1.0 - phase_active) * (j_path_speed + j_control + j_smooth)
    return (j_track + baseline + j_slosh + j_phase_relative) / n_steps


def terminal_cost_expr(sym, cfg):
    """stage N 的 EXTERNAL 终端代价：跟踪 + 残余模态能量（§5.4），无控制项。"""
    if sym.get("direct_omega_legacy"):
        return terminal_cost_expr_direct_omega_legacy(sym, cfg)
    x = sym["x"]
    p = sym["p"]
    j = _tracking_cost(x, p)
    if sym.get("with_slosh"):
        j = j + _slosh_cost(x, p)
        j = j + _phase_rejoin_terminal_relative_cost(x, p, cfg)
    return j
