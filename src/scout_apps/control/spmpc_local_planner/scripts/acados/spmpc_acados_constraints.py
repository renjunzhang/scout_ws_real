"""SPMPC 连续 MPCC —— B0 约束（控制与状态 bounds）。

只设置 acados ocp.constraints 的边界数组（numpy），不 import acados_template。
B0 第一版只做简单 bound（方案 §7 Phase C）：
  控制: a∈[-a_max,a_max], omega∈[-omega_max,omega_max], v_s∈[0,vs_max]
  状态: v∈[0,v_max]
obstacle / costmap / hard corridor 不在 B0 引入。
"""

import numpy as np


def set_constraints(ocp, cfg):
    a_max = cfg["a_max"]
    omega_max = cfg["omega_max"]
    vs_max = cfg["vs_max"]
    v_max = cfg["v_max"]

    # 控制 bounds: u = [a, omega, v_s]
    ocp.constraints.idxbu = np.array([0, 1, 2])
    ocp.constraints.lbu = np.array([-a_max, -omega_max, 0.0])
    ocp.constraints.ubu = np.array([a_max, omega_max, vs_max])

    # 状态 bound: v 是 x[3]
    ocp.constraints.idxbx = np.array([3])
    ocp.constraints.lbx = np.array([0.0])
    ocp.constraints.ubx = np.array([v_max])

    # 初始状态由 wrapper 每周期通过 set("x0", ...) 设定；
    # 这里给出占位 x0，维度需匹配 nx。
    ocp.constraints.x0 = np.zeros(cfg["nx"])
