#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""换容器 / 换液体后，重算晃动模态派生量并同步 oscrs_container.yaml。

==============================================================================
背景
==============================================================================
Scout 晃动模型（Ferrari 2026 RA-L）只需要 4 个一手物理量：
  R   容器内半径 (m)
  h   静止液面到容器底高度 (m)
  ρ   液体密度 (kg/m³)
  ν   液体动力黏度 (Pa·s)

以下派生量都是从这 4 个值闭式得到的（ξ = ξ_{1,1} = 1.841 是常数）：
  m_F          = ρ·π·R²·h
  m_n          = m_F · 2R·tanh(ξh/R) / (ξh·(ξ²-1))                   ← 一阶模态等效质量
  ω_n          = sqrt( g·(ξ/R)·tanh(ξh/R) )                          ← Ferrari 式(2)
  height_coeff (observer)  = 4·h·m_n / (m_F·R)                       ← 在线 observer 标定口径
  height_coeff (Ferrari)   = ξ²·h·m_n / (m_F·R)                      ← Ferrari 式(13)
  ζ (Ferrari)  = 0.92·sqrt( ν/ρ/(g·R³) )
                · [1 + 0.318/sinh(ξh/R) · (1 + (1−h/R)/cosh(ξh/R))]   ← Ferrari 式(3)

oscrs_container.yaml 中以下字段是这些派生量的"硬写值"，必须保持与 R/h 一致：
  slosh_score.omega_n      ← 必须 = ω_n
  slosh_score.height_coeff ← 必须 = height_coeff (observer)

ζ 在 yaml 中的 slosh.damping_ratio 默认是 observer 拟合值（不是 Ferrari 物理值），
因为容器黏性壁面 / 表面张力等效应不在 Ferrari 半经验公式里。本脚本只把
Ferrari 物理 ζ 打印出来供参考，**不会自动覆盖 damping_ratio**——除非用户加
--write-zeta-ferrari 强制写。

==============================================================================
用法
==============================================================================
1. 显示当前 yaml 里的派生量是否与物理一致（不写文件）：

     python3 compute_modal_params.py \
         --yaml src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml

   输出会列出 ω_n / height_coeff 的"yaml 现值 vs 物理值"，差异 > 1e-6 会标 ⚠。

2. 用命令行覆盖物理量并查看派生量（不写文件）：

     python3 compute_modal_params.py --R 0.025 --h 0.07 --rho 900 --nu 5.0e-2

3. 改 yaml 里的容器物理量（手工改 R/h/ρ/ν），然后让本脚本同步派生量：

     # 先手工编辑 yaml: slosh.container_radius / liquid_height / liquid_density
     #                  / liquid_dynamic_viscosity
     python3 compute_modal_params.py \
         --yaml src/scout_apps/control/scout_local_planner/config/oscrs_container.yaml \
         --write

   会就地修改 slosh_score.omega_n 与 slosh_score.height_coeff 两行，
   注释 / 缩进 / 行尾备注全部保留。

4. 强制把 Ferrari 物理 ζ 也写入 slosh.damping_ratio（一般不建议，仅作 ablation）：

     python3 compute_modal_params.py --yaml ...oscrs_container.yaml \
         --write --write-zeta-ferrari

5. 把派生量另存为 yaml 块（不动原文件）方便手工 review：

     python3 compute_modal_params.py --R 0.025 --h 0.07 --emit-yaml /tmp/derived.yaml

==============================================================================
注意
==============================================================================
本脚本只算"由物理量推导"的派生项；下面这些参数它不管，要换容器仍要单独评估：
  - oscrs.eta_lim_mm        任务级安全门，按容器自由空间 60-80% 选
  - oscrs.height_coeff_mode / damping_ratio_mode  方案选项，不是物理量
  - slosh.offset_x / offset_y                     CAD / 机身测量
"""

import argparse
import math
import re
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.stderr.write("error: PyYAML 未安装；运行 `pip install pyyaml`\n")
    sys.exit(2)

G = 9.81
XI_11 = 1.841


def compute(R, h, rho, nu, xi=XI_11):
    """返回所有由 R/h/ρ/ν 推导的派生量。"""
    if R <= 0 or h <= 0 or rho <= 0 or nu <= 0:
        raise ValueError("R / h / ρ / ν 必须均为正数")
    xi_h_R = xi * h / R
    m_F = rho * math.pi * R * R * h
    m_n = m_F * (2.0 * R * math.tanh(xi_h_R)) / (xi * h * (xi * xi - 1.0))
    omega_n = math.sqrt(G * (xi / R) * math.tanh(xi_h_R))
    height_coeff_observer = (4.0 * h * m_n) / (m_F * R)
    height_coeff_ferrari = (xi * xi * h * m_n) / (m_F * R)
    zeta_ferrari = (
        0.92
        * math.sqrt(nu / rho / (G * R ** 3))
        * (1.0 + (0.318 / math.sinh(xi_h_R)) * (1.0 + (1.0 - h / R) / math.cosh(xi_h_R)))
    )
    return {
        "R": R,
        "h": h,
        "rho": rho,
        "nu": nu,
        "xi": xi,
        "xi_h_R": xi_h_R,
        "m_F": m_F,
        "m_n": m_n,
        "omega_n": omega_n,
        "height_coeff_observer": height_coeff_observer,
        "height_coeff_ferrari": height_coeff_ferrari,
        "zeta_ferrari": zeta_ferrari,
    }


def read_yaml_inputs(yaml_path):
    """从 oscrs_container.yaml 读 R/h/ρ/ν + 当前 yaml 写死的 ω_n / height_coeff / ζ。"""
    with open(yaml_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    slosh = data.get("slosh", {}) or {}
    score = data.get("slosh_score", {}) or {}
    return {
        "R": float(slosh.get("container_radius", 0.0185)),
        "h": float(slosh.get("liquid_height", 0.058)),
        "rho": float(slosh.get("liquid_density", 1000.0)),
        "nu": float(slosh.get("liquid_dynamic_viscosity", 1.0e-3)),
        "yaml_omega_n": float(score.get("omega_n", float("nan"))),
        "yaml_height_coeff": float(score.get("height_coeff", float("nan"))),
        "yaml_damping_ratio": float(slosh.get("damping_ratio", float("nan"))),
        "yaml_damping_mode": str(slosh.get("damping_ratio_mode", "manual")),
        "yaml_height_mode": str((data.get("oscrs", {}) or {}).get("height_coeff_mode", "observer_linear")),
    }


def replace_scalar_inline(text, key, new_value, indent_spaces=2):
    """将一行形如 `  omega_n: 31.25  # 注释` 的标量替换为新值，保留缩进与注释。

    匹配条件: 行首恰好 indent_spaces 个空格 + key + ':' + 数值 (含科学计数);
    返回 (替换后整段, 实际替换次数)。整段中 key 重名时只替换缩进匹配的那行。
    """
    pattern = re.compile(
        r"^(?P<indent>" + " " * indent_spaces + r")"
        r"(?P<key>" + re.escape(key) + r")"
        r"(?P<sep>:\s+)"
        r"(?P<value>[+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)"
        r"(?P<suffix>.*)$",
        flags=re.MULTILINE,
    )
    formatted = format_scalar(new_value)

    n_replaced = [0]
    def _sub(m):
        n_replaced[0] += 1
        return m.group("indent") + m.group("key") + m.group("sep") + formatted + m.group("suffix")
    return pattern.sub(_sub, text), n_replaced[0]


def format_scalar(v):
    """把浮点数格式化成 yaml 友好字符串：
       - 0 的显示成 0.0
       - 整数显示成 X.0
       - 其它保留 12 位有效数字（够覆盖 ω_n / height_coeff 的精度）
    """
    if v == 0.0:
        return "0.0"
    s = f"{v:.12g}"
    if "." not in s and "e" not in s and "E" not in s:
        s += ".0"
    return s


def write_yaml_inplace(yaml_path, omega_n, height_coeff_observer,
                       zeta_ferrari=None):
    text = Path(yaml_path).read_text(encoding="utf-8")
    text, n_omega = replace_scalar_inline(text, "omega_n", omega_n)
    text, n_hc = replace_scalar_inline(text, "height_coeff", height_coeff_observer)
    n_zeta = 0
    if zeta_ferrari is not None:
        text, n_zeta = replace_scalar_inline(text, "damping_ratio", zeta_ferrari)
    if n_omega != 1 or n_hc != 1:
        raise RuntimeError(
            f"yaml 文件 {yaml_path} 中 omega_n/height_coeff 行匹配数异常: "
            f"omega_n={n_omega}, height_coeff={n_hc}（期望各 1）"
        )
    if zeta_ferrari is not None and n_zeta != 1:
        raise RuntimeError(
            f"damping_ratio 行匹配数异常: {n_zeta}（期望 1）"
        )
    Path(yaml_path).write_text(text, encoding="utf-8")
    return {"omega_n": n_omega, "height_coeff": n_hc, "damping_ratio": n_zeta}


def emit_yaml(out_path, derived):
    block = {
        "slosh_score": {
            "omega_n": float(derived["omega_n"]),
            "height_coeff": float(derived["height_coeff_observer"]),
        },
        "slosh": {
            "damping_ratio_ferrari_reference": float(derived["zeta_ferrari"]),
        },
        "oscrs_height_coeff_modes": {
            "observer_linear": float(derived["height_coeff_observer"]),
            "ferrari_closed_form": float(derived["height_coeff_ferrari"]),
        },
        "inputs_used": {
            "R": float(derived["R"]),
            "h": float(derived["h"]),
            "rho": float(derived["rho"]),
            "nu": float(derived["nu"]),
        },
    }
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(block, handle, sort_keys=False, allow_unicode=True)


def parse_args():
    parser = argparse.ArgumentParser(
        description="重算 Scout 晃动模态派生量并同步 oscrs_container.yaml"
    )
    parser.add_argument("--R", type=float, default=None, help="容器内半径 (m)")
    parser.add_argument("--h", type=float, default=None, help="静止液面高度 (m)")
    parser.add_argument("--rho", type=float, default=None, help="液体密度 (kg/m³)")
    parser.add_argument("--nu", type=float, default=None, help="动力黏度 (Pa·s)")
    parser.add_argument("--yaml", default="", help="oscrs_container.yaml 路径；提供时默认从其读取 R/h/ρ/ν")
    parser.add_argument("--write", action="store_true",
                        help="把 ω_n / height_coeff 写回 --yaml 指定的文件（in-place，保留注释）")
    parser.add_argument("--write-zeta-ferrari", action="store_true",
                        help="把 Ferrari 物理 ζ 写回 slosh.damping_ratio（一般不建议；仅 ablation）")
    parser.add_argument("--emit-yaml", default="",
                        help="把所有派生量另存为这个 yaml 路径（不动原文件）")
    return parser.parse_args()


def main():
    args = parse_args()

    yaml_inputs = None
    if args.yaml:
        yaml_inputs = read_yaml_inputs(args.yaml)

    R = args.R if args.R is not None else (yaml_inputs["R"] if yaml_inputs else 0.0185)
    h = args.h if args.h is not None else (yaml_inputs["h"] if yaml_inputs else 0.058)
    rho = args.rho if args.rho is not None else (yaml_inputs["rho"] if yaml_inputs else 1000.0)
    nu = args.nu if args.nu is not None else (yaml_inputs["nu"] if yaml_inputs else 1.0e-3)

    d = compute(R, h, rho, nu)

    print("==== 输入物理量 ====")
    print(f"  R       = {R:.6f} m")
    print(f"  h       = {h:.6f} m")
    print(f"  ρ       = {rho:.3f} kg/m³")
    print(f"  ν       = {nu:.3e} Pa·s")
    print(f"  ξ_{{1,1}} = {XI_11} (常数)")
    print(f"  ξh/R    = {d['xi_h_R']:.4f}")
    print()
    print("==== 派生量 ====")
    print(f"  m_F (总液体质量)             = {d['m_F']:.6e} kg")
    print(f"  m_n (一阶模态等效质量)       = {d['m_n']:.6e} kg")
    print(f"  ω_n (一阶模态频率, Ferrari)  = {d['omega_n']:.6f} rad/s")
    print(f"  height_coeff (observer)      = {d['height_coeff_observer']:.6f}")
    print(f"  height_coeff (ferrari)       = {d['height_coeff_ferrari']:.6f}")
    print(f"  ζ_n (Ferrari 物理推导)       = {d['zeta_ferrari']:.6f}")
    print()

    if yaml_inputs:
        print("==== 与 yaml 当前值对比 ====")
        report_match("slosh_score.omega_n", yaml_inputs["yaml_omega_n"], d["omega_n"])
        # yaml 的 height_coeff 是 observer 口径
        report_match("slosh_score.height_coeff", yaml_inputs["yaml_height_coeff"], d["height_coeff_observer"])
        print(f"  slosh.damping_ratio          = {yaml_inputs['yaml_damping_ratio']:.6f}"
              f"  (yaml; mode={yaml_inputs['yaml_damping_mode']})")
        print(f"     vs Ferrari 物理 ζ         = {d['zeta_ferrari']:.6f}"
              f"  (差 {abs(yaml_inputs['yaml_damping_ratio']-d['zeta_ferrari']):.4f}"
              f"，决定是否启用 ferrari_physics 或重新 observer 拟合)")
        print(f"  oscrs.height_coeff_mode     = {yaml_inputs['yaml_height_mode']}")
        print()

    if args.write:
        if not args.yaml:
            sys.stderr.write("error: --write 需要 --yaml 指定目标文件\n")
            sys.exit(2)
        zeta_to_write = d["zeta_ferrari"] if args.write_zeta_ferrari else None
        result = write_yaml_inplace(args.yaml, d["omega_n"], d["height_coeff_observer"], zeta_to_write)
        print(f"==== 已就地更新 {args.yaml} ====")
        print(f"  slosh_score.omega_n      ← {format_scalar(d['omega_n'])} ({result['omega_n']} 行)")
        print(f"  slosh_score.height_coeff ← {format_scalar(d['height_coeff_observer'])} ({result['height_coeff']} 行)")
        if args.write_zeta_ferrari:
            print(f"  slosh.damping_ratio      ← {format_scalar(d['zeta_ferrari'])} ({result['damping_ratio']} 行)")
        print()

    if args.emit_yaml:
        emit_yaml(args.emit_yaml, d)
        print(f"==== 派生量另存到 {args.emit_yaml} ====")


def report_match(name, yaml_value, physics_value, tol=1e-6):
    if math.isnan(yaml_value):
        flag = "(yaml 缺该字段)"
    elif abs(yaml_value - physics_value) <= tol * max(1.0, abs(physics_value)):
        flag = "OK"
    else:
        flag = f"⚠ 差 {yaml_value - physics_value:+.6e}"
    print(f"  {name:30s} yaml={yaml_value:.6f}  physics={physics_value:.6f}  {flag}")


if __name__ == "__main__":
    main()
