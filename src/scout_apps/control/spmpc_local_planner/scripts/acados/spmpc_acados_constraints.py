"""SPMPC 连续 MPCC —— acados 约束（控制、状态和液体恢复预算边界）。

只设置 acados ocp.constraints 的边界数组（numpy），不 import acados_template。
B0 显式执行器主线约束 actual / command 速度和控制，并保留全时域 jerk 行：
  控制: a∈[-a_max,a_max], alpha∈[-alpha_max,alpha_max], v_s∈[0,vs_max]
  状态: v∈[0,v_max], omega∈[-omega_max,omega_max]
direct-omega 诊断模型使用独立 bounds；slosh 追加模态高度约束。
严格模式以性能目标为上限，恢复模式以上限加预算（再受物理边界截断）为硬边界。
obstacle / costmap / hard corridor 不在 B0 引入。
"""

import casadi as ca
import numpy as np

from spmpc_acados_model import ACCEL_MEMORY_INDEX, PIDX
from planning_terms import region_reference_constraints


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


def set_region_reference_constraints(ocp, pidx):
    """Common region/footprint and interpolation-domain rows for both models."""
    rows = region_reference_constraints(ocp.model.x, ocp.model.p, pidx)
    for suffix in ("", "_0", "_e"):
        setattr(ocp.model, "con_h_expr"+suffix, rows)
        # acados masks bounds at -ACADOS_INFTY. A large finite lower bound
        # instead creates an unnecessary, badly scaled barrier inequality.
        setattr(ocp.constraints, "lh"+suffix, np.full(int(rows.numel()), -1e15))
        setattr(ocp.constraints, "uh"+suffix, np.zeros(int(rows.numel())))


def set_constraints(ocp, cfg, explicit_actuator=False):
    a_max = cfg["a_max"]
    omega_max = cfg["omega_max"]
    vs_max = cfg["vs_max"]
    v_max = cfg["v_max"]
    alpha_max = cfg["alpha_max"]

    # 控制 bounds: u = [a, alpha, v_s]；alpha=d(omega)/dt 硬约束在 ±alpha_max。
    ocp.constraints.idxbu = np.array([0, 1, 2])
    ocp.constraints.lbu = np.array([-a_max, -alpha_max, 0.0])
    ocp.constraints.ubu = np.array([a_max, alpha_max, vs_max])

    if explicit_actuator:
        # actual 与 command 分别受限；command bounds directly protect /cmd_vel.
        # Actual motion, commands, both FIFOs and acceleration memory are
        # bounded at EVERY future node, including the terminal node. Runtime
        # can impose an exactly drained vehicle state at/after the deadline.
        ocp.constraints.idxbx = np.array([3, 5] + list(range(6, 24)))
        ocp.constraints.lbx = np.array([0.,-omega_max,0.,-omega_max]+[0.]*5+[-omega_max]*10+[-a_max])
        ocp.constraints.ubx = np.array([v_max,omega_max,v_max,omega_max]+[v_max]*5+[omega_max]*10+[a_max])
        ocp.constraints.idxbx_e = ocp.constraints.idxbx.copy()
        ocp.constraints.lbx_e = ocp.constraints.lbx.copy()
        ocp.constraints.ubx_e = ocp.constraints.ubx.copy()
        # One linear mixed state/control row at EVERY control stage, including
        # stage 0: -delta_a_max <= a_cmd - a_cmd_memory <= delta_a_max.
        # Always generate the row so the runtime switch does not need codegen.
        # Bounds are disabled by default and replaced by jerk_max * dt in C++.
        ocp.constraints.C = np.zeros((2, cfg["nx"]))
        ocp.constraints.C[0, ACCEL_MEMORY_INDEX] = -1.0
        # Row 1: actuator-output delta acceleration, filled with runtime
        # tau/gain/dt. Always generated; zero bound configuration disables it.
        ocp.constraints.D = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        ocp.constraints.lg = np.array([-1e15, -1e15])
        ocp.constraints.ug = np.array([1e15, 1e15])
    else:
        ocp.constraints.idxbx = np.array([3, 5])
        ocp.constraints.lbx = np.array([0.0, -omega_max])
        ocp.constraints.ubx = np.array([v_max, omega_max])

    # 初始状态由 wrapper 每周期通过 set("x0", ...) 设定；
    # 这里给出占位 x0，维度需匹配 nx。
    ocp.constraints.x0 = np.zeros(cfg["nx"])
    if explicit_actuator:
        set_region_reference_constraints(ocp, PIDX)



def set_constraints_slosh(ocp, cfg, pidx, eta_base=6,
                          explicit_actuator=False):
    """Normalized hard recovery/physical cap at every node, including x0.

    The performance target and its analytically eliminated slack live in the
    cost expression. eta_max_sq encodes min(target+budget, physical boundary)^2.
    The wrapper retains the real liquid x0 and rejects an exhausted budget.
    """
    set_constraints(ocp, cfg, explicit_actuator=explicit_actuator)

    x = ocp.model.x
    p = ocp.model.p
    eta_x = x[eta_base]
    eta_y = x[eta_base + 2]
    eta_max_sq = p[pidx["eta_max_sq"]]
    h_slosh = ca.vertcat((eta_x * eta_x + eta_y * eta_y) / eta_max_sq - 1.0)

    # q^2/cap^2 - 1 is always >= -1. A finite inactive lower bound avoids
    # injecting a 1e15 barrier range into the recovery QP.
    for suffix in ("", "_0", "_e"):
        common = region_reference_constraints(x, p, pidx) if explicit_actuator else ca.SX.zeros(0)
        rows = ca.vertcat(h_slosh, common)
        setattr(ocp.model, "con_h_expr"+suffix, rows)
        setattr(ocp.constraints, "lh"+suffix, np.r_[-2.0, np.full(int(common.numel()), -1e15)])
        setattr(ocp.constraints, "uh"+suffix, np.zeros(int(rows.numel())))
