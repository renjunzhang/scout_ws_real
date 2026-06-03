"""SPMPC 连续 MPCC —— B0 模型符号定义（纯 CasADi，不依赖 acados）。

设计约束（见方案 §5.1 / §11.1 / §11.4）：
  - 本模块只产出 CasADi 符号与显式 ODE，不 import acados_template；
  - AcadosModel 的组装、求解器选项、codegen 全部在 generate_spmpc_acados.py 完成；
  - 参数向量布局是 python 建模 与 C++ 包装层（Phase C）的契约，改这里必须同步 wrapper。

B0（无 slosh）：
  状态 x = [px, py, theta, v, s]                （5 维）
  控制 u = [a, omega, v_s]                      （3 维）
  动力学（显式 ODE，由 acados 积分器离散）：
    px_dot    = v cos(theta)
    py_dot    = v sin(theta)
    theta_dot = omega
    v_dot     = a
    s_dot     = v_s

参考几何以 s 的三次多项式按参数传入（局部 MPCC 参考），
contour / lag 在 cost 模块中据此解析计算。
"""

import casadi as ca

NX = 5  # [px, py, theta, v, s]
NU = 3  # [a, omega, v_s]

# 参数向量布局：改顺序 / 增删 必须同步 C++ 包装层与 spmpc_acados_cost.py。
PARAM_NAMES = [
    "rx0", "rx1", "rx2", "rx3",   # x_ref(s) = rx0 + rx1 s + rx2 s^2 + rx3 s^3
    "ry0", "ry1", "ry2", "ry3",   # y_ref(s) = ry0 + ry1 s + ry2 s^2 + ry3 s^3
    "w_contour", "w_lag", "w_progress",   # 跟踪 / 进度权重（运行时可调，变体切换用）
    "w_a", "w_omega", "w_vs",             # 控制幅值权重
    "w_du_a", "w_du_omega", "w_du_vs",    # 控制变化率权重（仅 stage 0 置非零 -> 跨周期连续性）
    "a_prev", "omega_prev", "vs_prev",    # 上一控制周期实际下发的控制（§4.5）
    "e_c_ref", "e_l_ref",                 # contour / lag 归一化尺度
]
NP = len(PARAM_NAMES)
PIDX = {name: i for i, name in enumerate(PARAM_NAMES)}

# slosh（9 维）模型在 B0 参数之后追加的物理/权重参数。
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
PARAM_NAMES_SLOSH = PARAM_NAMES + SLOSH_EXTRA_NAMES
NP_SLOSH = len(PARAM_NAMES_SLOSH)
PIDX_SLOSH = {name: i for i, name in enumerate(PARAM_NAMES_SLOSH)}

NX_SLOSH = 9  # [px, py, theta, v, s, eta_x, eta_x_dot, eta_y, eta_y_dot]


def export_spmpc_b0_symbols(name="spmpc_b0"):
    """返回 B0 的 CasADi 符号与显式/隐式动力学，供 orchestrator 组装 AcadosModel。"""
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

    p = ca.SX.sym("p", NP)
    xdot = ca.SX.sym("xdot", NX)

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
        "nx": NX,
        "nu": NU,
        "np": NP,
    }


def export_spmpc_slosh_symbols(name="spmpc_slosh"):
    """返回 9 维 slosh 模型符号（B0 机器人状态 + 二维一阶液体模态）。

    模态连续动力学（§4.3）：
      η̈_i + 2ζω_n η̇_i + ω_n^2 η_i = -κ_i a_i,  a_x=a, a_y=v·ω
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
