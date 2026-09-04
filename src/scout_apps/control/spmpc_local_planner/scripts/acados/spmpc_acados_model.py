"""SPMPC 连续 MPCC —— 模型符号定义（纯 CasADi，不依赖 acados）。

设计约束（见方案 §5.1 / §11.1 / §11.4）：
  - 本模块只产出 CasADi 符号与显式 ODE，不 import acados_template；
  - AcadosModel 的组装、求解器选项、codegen 全部在 generate_spmpc_acados.py 完成；
  - 参数向量布局是 Python 建模与 C++ 包装层的契约，改这里必须同步 wrapper。

B0（无 slosh）采用显式 command/actual 执行器模型：
  基础状态 = [px, py, theta, v_actual, s, omega_actual, v_cmd, omega_cmd]
  延迟状态 = 5 拍线速度 FIFO + 10 拍角速度 FIFO
  连续性状态 = [a_cmd_memory]，满足 a_cmd_memory(k+1) = a_cmd(k)
  控制 u   = [a_cmd, alpha_cmd, v_s]

每个 OCP interval 内以固定 FIFO 头作为延迟输入，用 RK4 离散连续 FOPDT；
interval 末端把新 command state 写入 FIFO。车体和液体只消费 actual，
/cmd_vel 则从下一拍 command state 提取。

参考几何以 s 的三次多项式按参数传入（局部 MPCC 参考），
contour / lag 在 cost 模块中据此解析计算。
"""

import casadi as ca

LINEAR_DELAY_STEPS = 5
ANGULAR_DELAY_STEPS = 10
ACTUATOR_CORE_NX = 8
LINEAR_QUEUE_START = ACTUATOR_CORE_NX
ANGULAR_QUEUE_START = LINEAR_QUEUE_START + LINEAR_DELAY_STEPS
ACCEL_MEMORY_INDEX = ANGULAR_QUEUE_START + ANGULAR_DELAY_STEPS
SLOSH_STATE_OFFSET = ACCEL_MEMORY_INDEX + 1

NX = SLOSH_STATE_OFFSET
NU = 3  # [a, alpha, v_s]

# 参数向量布局：改顺序 / 增删 必须同步 C++ 包装层与 spmpc_acados_cost.py。
# 2026-06-08：omega 变状态后，删去 w_du_omega / omega_prev（omega 连续性由状态初值保证），
# 新增 w_alpha（转向角加速度幅值权重，所有 stage 生效 -> horizon 内 Δomega 平滑/抗 chatter）。
PARAM_NAMES = [
    "rx0", "rx1", "rx2", "rx3",   # x_ref(s) = rx0 + rx1 s + rx2 s^2 + rx3 s^3
    "ry0", "ry1", "ry2", "ry3",   # y_ref(s) = ry0 + ry1 s + ry2 s^2 + ry3 s^3
    "w_contour", "w_lag", "w_progress",   # 跟踪 / 进度权重（运行时可调，变体切换用）
    "w_a", "w_omega", "w_v", "w_vs", "w_alpha",  # 幅值/速度: a / omega / v / v_s / alpha
    "w_du_a", "w_du_vs",                  # a_cmd 全时域连续性 / v_s 跨周期第一帧连续性
    "a_prev", "vs_prev",                  # a_prev 仅保留 ABI/诊断兼容；vs_prev 供 stage 0 使用
    "e_c_ref", "e_l_ref",                 # contour / lag 归一化尺度
    "v_ref",                               # 物理速度/虚拟路径进度参考速度：用于 v 和 v_s tracking 防 creep
    "actuator_dt",                         # 离散 OCP 步长
    "actuator_tau_v",                      # 线速度一阶惯性时间常数
    "actuator_tau_omega",                  # 角速度一阶惯性时间常数
    "actuator_gain_v",                     # 线速度稳态增益
    "actuator_gain_omega",                 # 角速度稳态增益
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

NX_SLOSH = SLOSH_STATE_OFFSET + 4

# direct-omega + slosh（路 B 诊断）：legacy direct-omega 参数 + slosh 物理/软代价权重。
# 不追加 mainline hard-constraint 参数，避免诊断后端共享不支持的硬约束布局。
PARAM_NAMES_SLOSH_DIRECT_OMEGA = PARAM_NAMES_DIRECT_OMEGA_LEGACY + SLOSH_EXTRA_NAMES
NP_SLOSH_DIRECT_OMEGA = len(PARAM_NAMES_SLOSH_DIRECT_OMEGA)
PIDX_SLOSH_DIRECT_OMEGA = {name: i for i, name in enumerate(PARAM_NAMES_SLOSH_DIRECT_OMEGA)}

NX_SLOSH_DIRECT_OMEGA = 9  # [px, py, theta, v, s, eta_x, eta_x_dot, eta_y, eta_y_dot]（omega 是控制, 无 omega 状态）


def _export_explicit_actuator_symbols(name, with_slosh):
    """构造 command/actual 分离、固定离散 FIFO 延迟的 OCP 模型。"""
    px = ca.SX.sym("px")
    py = ca.SX.sym("py")
    theta = ca.SX.sym("theta")
    v_actual = ca.SX.sym("v_actual")
    s = ca.SX.sym("s")
    omega_actual = ca.SX.sym("omega_actual")
    v_cmd = ca.SX.sym("v_cmd")
    omega_cmd = ca.SX.sym("omega_cmd")
    q_v = ca.SX.sym("q_v", LINEAR_DELAY_STEPS)
    q_omega = ca.SX.sym("q_omega", ANGULAR_DELAY_STEPS)
    a_cmd_memory = ca.SX.sym("a_cmd_memory")

    eta = []
    if with_slosh:
        eta = [
            ca.SX.sym("eta_x"),
            ca.SX.sym("eta_x_dot"),
            ca.SX.sym("eta_y"),
            ca.SX.sym("eta_y_dot"),
        ]
    x = ca.vertcat(
        px, py, theta, v_actual, s, omega_actual, v_cmd, omega_cmd,
        q_v, q_omega, a_cmd_memory, *eta)

    a_cmd = ca.SX.sym("a_cmd")
    alpha_cmd = ca.SX.sym("alpha_cmd")
    v_s = ca.SX.sym("v_s")
    u = ca.vertcat(a_cmd, alpha_cmd, v_s)

    np_dim = NP_SLOSH if with_slosh else NP
    p = ca.SX.sym("p", np_dim)
    pidx = PIDX_SLOSH if with_slosh else PIDX

    dt = p[pidx["actuator_dt"]]
    tau_v = p[pidx["actuator_tau_v"]]
    tau_omega = p[pidx["actuator_tau_omega"]]
    gain_v = p[pidx["actuator_gain_v"]]
    gain_omega = p[pidx["actuator_gain_omega"]]

    def rhs(z):
        delayed_v_cmd = z[LINEAR_QUEUE_START]
        delayed_omega_cmd = z[ANGULAR_QUEUE_START]
        a_actual = (gain_v * delayed_v_cmd - z[3]) / tau_v
        alpha_actual = (gain_omega * delayed_omega_cmd - z[5]) / tau_omega
        values = [
            z[3] * ca.cos(z[2]),
            z[3] * ca.sin(z[2]),
            z[5],
            a_actual,
            u[2],
            alpha_actual,
            u[0],
            u[1],
        ]
        values.extend([0.0] * (LINEAR_DELAY_STEPS + ANGULAR_DELAY_STEPS))
        # a_cmd_memory 是离散记忆状态，interval 末由 disc_dyn 直接覆盖为当前 a_cmd。
        values.append(0.0)
        if with_slosh:
            eta_x = z[SLOSH_STATE_OFFSET]
            eta_x_dot = z[SLOSH_STATE_OFFSET + 1]
            eta_y = z[SLOSH_STATE_OFFSET + 2]
            eta_y_dot = z[SLOSH_STATE_OFFSET + 3]
            two_zeta_omega_n = p[pidx["two_zeta_omega_n"]]
            omega_n_sq = p[pidx["omega_n_sq"]]
            kappa_x = p[pidx["kappa_x"]]
            kappa_y = p[pidx["kappa_y"]]
            values.extend([
                eta_x_dot,
                -two_zeta_omega_n * eta_x_dot - omega_n_sq * eta_x
                - kappa_x * a_actual,
                eta_y_dot,
                -two_zeta_omega_n * eta_y_dot - omega_n_sq * eta_y
                - kappa_y * z[3] * z[5],
            ])
        return ca.vertcat(*values)

    k1 = rhs(x)
    k2 = rhs(x + 0.5 * dt * k1)
    k3 = rhs(x + 0.5 * dt * k2)
    k4 = rhs(x + dt * k3)
    integrated = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    next_q_v = ca.vertcat(
        x[LINEAR_QUEUE_START + 1:ANGULAR_QUEUE_START], integrated[6])
    next_q_omega = ca.vertcat(
        x[ANGULAR_QUEUE_START + 1:ACCEL_MEMORY_INDEX], integrated[7])
    next_parts = [
        integrated[0:ACTUATOR_CORE_NX], next_q_v, next_q_omega, a_cmd]
    if with_slosh:
        next_parts.append(integrated[SLOSH_STATE_OFFSET:SLOSH_STATE_OFFSET + 4])
    disc_dyn = ca.vertcat(*next_parts)

    return {
        "name": name,
        "x": x,
        "u": u,
        "p": p,
        "disc_dyn": disc_dyn,
        "discrete": True,
        "nx": NX_SLOSH if with_slosh else NX,
        "nu": NU,
        "np": np_dim,
        "with_slosh": with_slosh,
        "accel_memory_index": ACCEL_MEMORY_INDEX,
        "eta_base": SLOSH_STATE_OFFSET,
        "linear_delay_steps": LINEAR_DELAY_STEPS,
        "angular_delay_steps": ANGULAR_DELAY_STEPS,
    }


def export_spmpc_b0_symbols(name="spmpc_b0"):
    return _export_explicit_actuator_symbols(name, False)



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
    return _export_explicit_actuator_symbols(name, True)


def export_spmpc_slosh_direct_omega_symbols(name="spmpc_slosh_direct_omega"):
    """路 B：direct-omega + slosh（9 维）。omega 作为直接控制（u[1]），液体模态在 x[5..8]。

    模态连续动力学（§4.3）：η̈_i + 2ζω_n η̇_i + ω_n^2 η_i = -κ_i a_i, a_x=a, a_y=v·ω（ω 是控制）。
    该模型提供 codegen / link 入口；formal 使用仍需单独验证。转向 chatter 由 wrapper 出口 omega-rate 限幅压制。
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
