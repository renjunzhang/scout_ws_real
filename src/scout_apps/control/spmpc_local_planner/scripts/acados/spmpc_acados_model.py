"""SPMPC 连续 MPCC —— B0 模型符号定义（纯 CasADi，不依赖 acados）。

设计约束（见方案 §5.1 / §11.1 / §11.4）：
  - 本模块只产出 CasADi 符号与显式 ODE，不 import acados_template；
  - AcadosModel 的组装、求解器选项、codegen 全部在 generate_spmpc_acados.py 完成；
  - 参数向量布局是 python 建模 与 C++ 包装层（Phase C）的契约，改这里必须同步 wrapper。

B0（无 slosh）—— 2026-06-08 起 omega 进入状态、alpha=omegȧ 成为控制：
  状态 x = [px, py, theta, v, s, omega]          （6 维）
  控制 u = [a, alpha, v_s]                        （3 维, alpha = d(omega)/dt）
  动力学（显式 ODE，由 acados 积分器离散）：
    px_dot    = v cos(theta)
    py_dot    = v sin(theta)
    theta_dot = omega          # omega 现在是状态
    v_dot     = a
    s_dot     = v_s
    omega_dot = alpha          # 转向角加速度作为控制

把 omega 提为状态、alpha 作受约束控制(|alpha|<=alpha_max)，是为了在 OCP 内
硬约束转向角加速度，消掉旧模型里 omega 可逐帧瞬变造成的直道甩舵 chattering，
并与 TEB/DWA 的 acc_lim_theta 同口径对齐（见 SOP 20260603）。

参考几何以 s 的三次多项式按参数传入（局部 MPCC 参考），
contour / lag 在 cost 模块中据此解析计算。
"""

import casadi as ca

NX = 6  # [px, py, theta, v, s, omega]
NU = 3  # [a, alpha, v_s]

# 参数向量布局：改顺序 / 增删 必须同步 C++ 包装层与 spmpc_acados_cost.py。
# 2026-06-08：omega 变状态后，删去 w_du_omega / omega_prev（omega 连续性由状态初值保证），
# 新增 w_alpha（转向角加速度幅值权重，所有 stage 生效 -> horizon 内 Δomega 平滑/抗 chatter）。
PARAM_NAMES = [
    "rx0", "rx1", "rx2", "rx3",   # x_ref(s) = rx0 + rx1 s + rx2 s^2 + rx3 s^3
    "ry0", "ry1", "ry2", "ry3",   # y_ref(s) = ry0 + ry1 s + ry2 s^2 + ry3 s^3
    "w_contour", "w_lag", "w_progress",   # 跟踪 / 进度权重（运行时可调，变体切换用）
    "w_a", "w_omega", "w_v", "w_vs", "w_alpha",  # 幅值/速度: a / omega / v / v_s / alpha
    "w_du_a", "w_du_vs",                  # 控制变化率（仅 stage 0 置非零 -> a/v_s 跨周期第一帧连续性）
    "a_prev", "vs_prev",                  # 上一控制周期实际下发的 a / v_s（§4.5）
    "e_c_ref", "e_l_ref",                 # contour / lag 归一化尺度
    "v_ref",                               # 物理速度/虚拟路径进度参考速度：用于 v 和 v_s tracking 防 creep
]
NP = len(PARAM_NAMES)
PIDX = {name: i for i, name in enumerate(PARAM_NAMES)}

# 诊断用 legacy direct-omega B0：保留当前 v/v_s/v_ref 防 creep 速度项，
# 但恢复 u[1]=omega 与 stage-0 du_omega/omega_prev。不要复用/重排上面的
# alpha-state PARAM_NAMES，否则会破坏当前 continuous_mpcc_acados 的 Python/C++/generated 契约。
NX_DIRECT_OMEGA_LEGACY = 5  # [px, py, theta, v, s]
PARAM_NAMES_DIRECT_OMEGA_LEGACY = [
    "rx0", "rx1", "rx2", "rx3",
    "ry0", "ry1", "ry2", "ry3",
    "w_contour", "w_lag", "w_progress",
    "w_a", "w_omega", "w_v", "w_vs",
    "w_du_a", "w_du_omega", "w_du_vs",
    "a_prev", "omega_prev", "vs_prev",
    "e_c_ref", "e_l_ref",
    "v_ref",
]
NP_DIRECT_OMEGA_LEGACY = len(PARAM_NAMES_DIRECT_OMEGA_LEGACY)
PIDX_DIRECT_OMEGA_LEGACY = {name: i for i, name in enumerate(PARAM_NAMES_DIRECT_OMEGA_LEGACY)}

# slosh（10 维）模型在 B0 参数之后追加的物理/权重参数。
# 共享前缀索引与 B0 完全一致，故 cost 模块可对两模型复用同一套 contour/lag 取值。
# slosh 物理参数（two_zeta_omega_n / omega_n_sq / kappa_x / kappa_y）必须来自
# 同一套 slosh_dynamics 核（C++ 包装层运行时注入），不在此另写隐式增益（§4.3）。
SLOSH_EXTRA_NAMES = [
    "two_zeta_omega_n",  # 2 ζ ω_n
    "omega_n_sq",        # ω_n^2
    "kappa_x",           # 纵向加速度到模态增益
    "kappa_y",           # 横向加速度到模态增益
    "eta_ref",           # η 归一化尺度（= slosh_height_ref / c_h，与 primitive 一致）
    "eta_dot_ref",       # η̇ 归一化尺度
    "w_slosh_eta",       # 模态位移权重
    "w_slosh_eta_dot",   # 模态速度权重
]
SLOSH_HARD_EXTRA_NAMES = [
    "eta_max_sq",        # 硬约束阈值: eta_x^2 + eta_y^2 <= eta_max_sq（mainline slosh only）
]
PARAM_NAMES_SLOSH = PARAM_NAMES + SLOSH_EXTRA_NAMES + SLOSH_HARD_EXTRA_NAMES
NP_SLOSH = len(PARAM_NAMES_SLOSH)
PIDX_SLOSH = {name: i for i, name in enumerate(PARAM_NAMES_SLOSH)}

NX_SLOSH = 10  # [px, py, theta, v, s, omega, eta_x, eta_x_dot, eta_y, eta_y_dot]

# direct-omega + slosh（路 B 诊断）：legacy direct-omega 参数 + slosh 物理/软代价权重。
# 不追加 mainline hard-constraint 参数，避免诊断后端共享不支持的硬约束布局。
PARAM_NAMES_SLOSH_DIRECT_OMEGA = PARAM_NAMES_DIRECT_OMEGA_LEGACY + SLOSH_EXTRA_NAMES
NP_SLOSH_DIRECT_OMEGA = len(PARAM_NAMES_SLOSH_DIRECT_OMEGA)
PIDX_SLOSH_DIRECT_OMEGA = {name: i for i, name in enumerate(PARAM_NAMES_SLOSH_DIRECT_OMEGA)}

NX_SLOSH_DIRECT_OMEGA = 9  # [px, py, theta, v, s, eta_x, eta_x_dot, eta_y, eta_y_dot]（omega 是控制, 无 omega 状态）


def export_spmpc_b0_symbols(name="spmpc_b0"):
    """返回 B0 的 CasADi 符号与显式/隐式动力学，供 orchestrator 组装 AcadosModel。"""
    px = ca.SX.sym("px")
    py = ca.SX.sym("py")
    theta = ca.SX.sym("theta")
    v = ca.SX.sym("v")
    s = ca.SX.sym("s")
    omega = ca.SX.sym("omega")
    x = ca.vertcat(px, py, theta, v, s, omega)

    a = ca.SX.sym("a")
    alpha = ca.SX.sym("alpha")
    v_s = ca.SX.sym("v_s")
    u = ca.vertcat(a, alpha, v_s)

    p = ca.SX.sym("p", NP)
    xdot = ca.SX.sym("xdot", NX)

    f_expl = ca.vertcat(
        v * ca.cos(theta),
        v * ca.sin(theta),
        omega,
        a,
        v_s,
        alpha,
    )

    return {
        "name": name,
        "x": x,
        "u": u,
        "p": p,
        "xdot": xdot,
        "f_expl": f_expl,
        "f_impl": xdot - f_expl,
        "nx": NX,
        "nu": NU,
        "np": NP,
    }



def export_spmpc_b0_direct_omega_legacy_symbols(name="spmpc_b0_direct_omega_legacy"):
    """诊断用旧式 B0：omega 作为直接控制量，非 formal 主线模型。"""
    px = ca.SX.sym("px")
    py = ca.SX.sym("py")
    theta = ca.SX.sym("theta")
    v = ca.SX.sym("v")
    s = ca.SX.sym("s")
    x = ca.vertcat(px, py, theta, v, s)

    a = ca.SX.sym("a")
    omega = ca.SX.sym("omega")
    v_s = ca.SX.sym("v_s")
    u = ca.vertcat(a, omega, v_s)

    p = ca.SX.sym("p", NP_DIRECT_OMEGA_LEGACY)
    xdot = ca.SX.sym("xdot", NX_DIRECT_OMEGA_LEGACY)

    f_expl = ca.vertcat(
        v * ca.cos(theta),
        v * ca.sin(theta),
        omega,
        a,
        v_s,
    )

    return {
        "name": name,
        "x": x,
        "u": u,
        "p": p,
        "xdot": xdot,
        "f_expl": f_expl,
        "f_impl": xdot - f_expl,
        "nx": NX_DIRECT_OMEGA_LEGACY,
        "nu": NU,
        "np": NP_DIRECT_OMEGA_LEGACY,
        "direct_omega_legacy": True,
    }



def export_spmpc_slosh_symbols(name="spmpc_slosh"):
    """返回 10 维 slosh 模型符号（B0 机器人状态 + 二维一阶液体模态）。

    模态连续动力学（§4.3）：
      η̈_i + 2ζω_n η̇_i + ω_n^2 η_i = -κ_i a_i,  a_x=a, a_y=v·ω（ω 现在是状态）
    """
    px = ca.SX.sym("px")
    py = ca.SX.sym("py")
    theta = ca.SX.sym("theta")
    v = ca.SX.sym("v")
    s = ca.SX.sym("s")
    omega = ca.SX.sym("omega")
    eta_x = ca.SX.sym("eta_x")
    eta_x_dot = ca.SX.sym("eta_x_dot")
    eta_y = ca.SX.sym("eta_y")
    eta_y_dot = ca.SX.sym("eta_y_dot")
    x = ca.vertcat(px, py, theta, v, s, omega, eta_x, eta_x_dot, eta_y, eta_y_dot)

    a = ca.SX.sym("a")
    alpha = ca.SX.sym("alpha")
    v_s = ca.SX.sym("v_s")
    u = ca.vertcat(a, alpha, v_s)

    p = ca.SX.sym("p", NP_SLOSH)
    xdot = ca.SX.sym("xdot", NX_SLOSH)

    two_zeta_omega_n = p[PIDX_SLOSH["two_zeta_omega_n"]]
    omega_n_sq = p[PIDX_SLOSH["omega_n_sq"]]
    kappa_x = p[PIDX_SLOSH["kappa_x"]]
    kappa_y = p[PIDX_SLOSH["kappa_y"]]

    a_x = a
    a_y = v * omega

    f_expl = ca.vertcat(
        v * ca.cos(theta),
        v * ca.sin(theta),
        omega,
        a,
        v_s,
        alpha,
        eta_x_dot,
        -two_zeta_omega_n * eta_x_dot - omega_n_sq * eta_x - kappa_x * a_x,
        eta_y_dot,
        -two_zeta_omega_n * eta_y_dot - omega_n_sq * eta_y - kappa_y * a_y,
    )

    return {
        "name": name,
        "x": x,
        "u": u,
        "p": p,
        "xdot": xdot,
        "f_expl": f_expl,
        "f_impl": xdot - f_expl,
        "nx": NX_SLOSH,
        "nu": NU,
        "np": NP_SLOSH,
        "with_slosh": True,
    }


def export_spmpc_slosh_direct_omega_symbols(name="spmpc_slosh_direct_omega"):
    """路 B：direct-omega + slosh（9 维）。omega 作为直接控制（u[1]），液体模态在 x[5..8]。

    模态连续动力学（§4.3）：η̈_i + 2ζω_n η̇_i + ω_n^2 η_i = -κ_i a_i, a_x=a, a_y=v·ω（ω 是控制）。
    本模型解算稳定（单次 RTI 即可跑完）；转向 chatter 由 wrapper 出口 omega-rate 限幅压制。
    """
    px = ca.SX.sym("px")
    py = ca.SX.sym("py")
    theta = ca.SX.sym("theta")
    v = ca.SX.sym("v")
    s = ca.SX.sym("s")
    eta_x = ca.SX.sym("eta_x")
    eta_x_dot = ca.SX.sym("eta_x_dot")
    eta_y = ca.SX.sym("eta_y")
    eta_y_dot = ca.SX.sym("eta_y_dot")
    x = ca.vertcat(px, py, theta, v, s, eta_x, eta_x_dot, eta_y, eta_y_dot)

    a = ca.SX.sym("a")
    omega = ca.SX.sym("omega")
    v_s = ca.SX.sym("v_s")
    u = ca.vertcat(a, omega, v_s)

    p = ca.SX.sym("p", NP_SLOSH_DIRECT_OMEGA)
    xdot = ca.SX.sym("xdot", NX_SLOSH_DIRECT_OMEGA)

    two_zeta_omega_n = p[PIDX_SLOSH_DIRECT_OMEGA["two_zeta_omega_n"]]
    omega_n_sq = p[PIDX_SLOSH_DIRECT_OMEGA["omega_n_sq"]]
    kappa_x = p[PIDX_SLOSH_DIRECT_OMEGA["kappa_x"]]
    kappa_y = p[PIDX_SLOSH_DIRECT_OMEGA["kappa_y"]]

    a_x = a
    a_y = v * omega

    f_expl = ca.vertcat(
        v * ca.cos(theta),
        v * ca.sin(theta),
        omega,
        a,
        v_s,
        eta_x_dot,
        -two_zeta_omega_n * eta_x_dot - omega_n_sq * eta_x - kappa_x * a_x,
        eta_y_dot,
        -two_zeta_omega_n * eta_y_dot - omega_n_sq * eta_y - kappa_y * a_y,
    )

    return {
        "name": name,
        "x": x,
        "u": u,
        "p": p,
        "xdot": xdot,
        "f_expl": f_expl,
        "f_impl": xdot - f_expl,
        "nx": NX_SLOSH_DIRECT_OMEGA,
        "nu": NU,
        "np": NP_SLOSH_DIRECT_OMEGA,
        "direct_omega_legacy": True,
        "with_slosh": True,
    }
