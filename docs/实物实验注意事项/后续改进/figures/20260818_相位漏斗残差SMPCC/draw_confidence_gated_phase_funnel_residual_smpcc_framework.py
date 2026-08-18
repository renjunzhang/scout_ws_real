#!/usr/bin/env python3
"""绘制普通 MPCC 与相位漏斗残差式 S-MPCC 的论文框架对比图。"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


OUT_DIR = Path(__file__).resolve().parent
STEM = "20260818_confidence_gated_phase_funnel_residual_smpcc_framework"

COLORS = {
    "ink": "#233B5D",
    "text": "#26323B",
    "line": "#52606D",
    "common_fill": "#FFFFFF",
    "common_edge": "#66727C",
    "ordinary_fill": "#ECEFF2",
    "ordinary_panel": "#F7F8FA",
    "ordinary_panel_edge": "#AAB2BA",
    "blue": "#0077BB",
    "blue_fill": "#E5F0FA",
    "teal": "#009988",
    "teal_fill": "#DDF3EE",
    "orange": "#EE7733",
    "orange_fill": "#FFF0E7",
    "red": "#CC3311",
    "red_fill": "#FDEBE8",
    "proposed_panel": "#F2FAF8",
    "proposed_panel_edge": "#53A9A1",
}


def rounded_panel(ax, x, y, w, h, face, edge):
    patch = FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.02,rounding_size=0.18",
        linewidth=1.1,
        linestyle=(0, (4, 3)),
        edgecolor=edge,
        facecolor=face,
        zorder=0,
    )
    ax.add_patch(patch)


def box(
    ax,
    cx,
    cy,
    w,
    h,
    title,
    body=(),
    face=None,
    edge=None,
    linewidth=1.2,
    title_size=7.5,
    body_size=6.55,
    title_offset=0.18,
    body_offset=-0.14,
):
    face = face or COLORS["common_fill"]
    edge = edge or COLORS["common_edge"]
    patch = FancyBboxPatch(
        (cx - w / 2, cy - h / 2),
        w,
        h,
        boxstyle="round,pad=0.04,rounding_size=0.15",
        linewidth=linewidth,
        edgecolor=edge,
        facecolor=face,
        zorder=3,
    )
    ax.add_patch(patch)
    if body:
        ax.text(
            cx,
            cy + title_offset,
            title,
            ha="center",
            va="center",
            fontsize=title_size,
            fontweight="bold",
            color=COLORS["text"],
            zorder=4,
        )
        ax.text(
            cx,
            cy + body_offset,
            "\n".join(body),
            ha="center",
            va="center",
            fontsize=body_size,
            color=COLORS["text"],
            linespacing=1.20,
            zorder=4,
        )
    else:
        ax.text(
            cx,
            cy,
            title,
            ha="center",
            va="center",
            fontsize=7.0,
            fontweight="bold",
            color=COLORS["text"],
            zorder=4,
        )
    return patch


def orthogonal_arrow(ax, points, color=None, linewidth=1.2, dashed=False, zorder=2):
    """用显式水平/垂直折线和末端箭头绘制正交连接。"""
    color = color or COLORS["line"]
    linestyle = (0, (4, 3)) if dashed else "solid"
    for (x0, y0), (x1, y1) in zip(points[:-2], points[1:-1]):
        if x0 != x1 and y0 != y1:
            raise ValueError("连接线必须横平竖直")
        ax.add_line(
            Line2D(
                [x0, x1],
                [y0, y1],
                color=color,
                linewidth=linewidth,
                linestyle=linestyle,
                zorder=zorder,
            )
        )
    (x0, y0), (x1, y1) = points[-2], points[-1]
    if x0 != x1 and y0 != y1:
        raise ValueError("连接线必须横平竖直")
    arrow = FancyArrowPatch(
        (x0, y0),
        (x1, y1),
        arrowstyle="-|>",
        mutation_scale=7.5,
        linewidth=linewidth,
        linestyle=linestyle,
        color=color,
        shrinkA=0,
        shrinkB=0,
        zorder=zorder,
    )
    ax.add_patch(arrow)


def main():
    matplotlib.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": ["Noto Sans CJK JP", "WenQuanYi Micro Hei", "DejaVu Sans"],
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "svg.fonttype": "none",
            "figure.facecolor": "white",
            "savefig.facecolor": "white",
        }
    )

    fig = plt.figure(figsize=(7.2, 4.8), dpi=300)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 10.6)
    ax.axis("off")

    ax.text(
        8,
        10.22,
        "普通 MPCC 与本文相位漏斗残差式 S-MPCC 的结构对比",
        ha="center",
        va="center",
        fontsize=10.5,
        fontweight="bold",
        color=COLORS["ink"],
    )

    # 两个方法按相同的五阶段结构上下对齐。
    rounded_panel(ax, 0.35, 6.45, 15.30, 3.30, COLORS["ordinary_panel"], COLORS["ordinary_panel_edge"])
    rounded_panel(ax, 0.35, 1.45, 15.30, 4.72, COLORS["proposed_panel"], COLORS["proposed_panel_edge"])
    ax.text(0.65, 9.45, "（a）普通 MPCC：只规划机器人运动", fontsize=8.6, color="#4B5563", va="center")
    ax.text(0.65, 5.87, "（b）本文方法：机器人—液体联合规划", fontsize=8.6, color="#006F66", va="center")

    xcols = [1.70, 4.80, 7.90, 11.00, 14.10]
    w = 2.45

    # (a) 普通 MPCC。
    y_a, h_a = 8.15, 1.15
    box(ax, xcols[0], y_a, w, h_a, "在线输入", ("全局路径、局部障碍", "里程计"))
    box(ax, xcols[1], y_a, w, h_a, "机器人状态构造", ("位姿、速度", "路径进度"))
    box(
        ax,
        xcols[2],
        y_a,
        w,
        h_a,
        "普通 MPCC",
        ("只预测机器人运动", "跟踪、进度、避障与平滑"),
        face=COLORS["ordinary_fill"],
        edge="#5B6770",
        linewidth=1.6,
    )
    box(ax, xcols[3], y_a, w, h_a, "首拍控制与安全执行", ("限幅", "碰撞检查"))
    box(ax, xcols[4], y_a, w, h_a, "移动底盘", ("执行机器人运动",))

    for left, right in zip(xcols[:-1], xcols[1:]):
        orthogonal_arrow(ax, [(left + w / 2, y_a), (right - w / 2, y_a)])

    # 普通 MPCC 的里程计闭环固定走面板底部。
    y_fb_a = 6.82
    orthogonal_arrow(
        ax,
        [
            (xcols[4], y_a - h_a / 2),
            (xcols[4], y_fb_a),
            (xcols[1], y_fb_a),
            (xcols[1], y_a - h_a / 2),
        ],
        dashed=True,
    )
    ax.text(9.3, y_fb_a + 0.10, "里程计反馈", fontsize=6.2, color=COLORS["line"], ha="center")

    # (b) 本文方法：离线支路 + 与普通 MPCC 对齐的在线主链。
    box(
        ax,
        7.90,
        5.25,
        4.15,
        0.74,
        "离线防晃参考生成",
        ("完整抵消序列 + 相位可恢复漏斗",),
        face=COLORS["blue_fill"],
        edge=COLORS["blue"],
        linewidth=1.6,
    )

    y_b, h_b = 3.82, 1.28
    box(ax, xcols[0], y_b, w, h_b, "在线输入", ("全局路径、局部障碍", "里程计 + IMU"))
    box(
        ax,
        xcols[1],
        y_b,
        w,
        h_b,
        "机器人—液体状态构造",
        ("状态时间对齐", "置信度与液体相位"),
        face=COLORS["orange_fill"],
        edge=COLORS["orange"],
        linewidth=1.6,
    )
    box(
        ax,
        xcols[2],
        y_b,
        w,
        h_b,
        "相位漏斗残差式 S-MPCC",
        ("普通 MPCC 骨架", "+ 液体可信短时预测", "+ 相位恢复约束"),
        face=COLORS["teal_fill"],
        edge=COLORS["teal"],
        linewidth=1.8,
        title_size=6.55,
        body_size=5.95,
        title_offset=0.29,
        body_offset=-0.20,
    )
    box(ax, xcols[3], y_b, w, h_b, "首拍控制与安全执行", ("限幅、碰撞检查", "与安全降级"))
    box(ax, xcols[4], y_b, w, h_b, "移动底盘 + 开放液体", ("实际运动", "产生物理晃动"))

    for left, right in zip(xcols[:-1], xcols[1:]):
        orthogonal_arrow(ax, [(left + w / 2, y_b), (right - w / 2, y_b)])

    # 离线参考分别进入状态相位对齐与在线优化器。
    y_branch = 4.70
    orthogonal_arrow(
        ax,
        [(7.90, 4.88), (7.90, y_branch), (xcols[1], y_branch), (xcols[1], y_b + h_b / 2)],
        color=COLORS["blue"],
        linewidth=1.35,
    )
    orthogonal_arrow(
        ax,
        [(7.90, 4.88), (7.90, y_b + h_b / 2)],
        color=COLORS["blue"],
        linewidth=1.35,
    )

    # 安全降级支路。
    box(
        ax,
        8.05,
        2.25,
        3.60,
        0.70,
        "低置信度或漏斗不可满足",
        ("减速、受控停车或重新对齐",),
        face=COLORS["red_fill"],
        edge=COLORS["red"],
        linewidth=1.5,
    )
    orthogonal_arrow(
        ax,
        [(xcols[2], y_b - h_b / 2), (xcols[2], 2.60)],
        color=COLORS["red"],
        linewidth=1.25,
    )
    orthogonal_arrow(
        ax,
        [(9.85, 2.25), (xcols[3], 2.25), (xcols[3], y_b - h_b / 2)],
        color=COLORS["red"],
        linewidth=1.25,
    )

    # 物理反馈与独立 RGB 评价分开表达，避免误解 RGB 已进入控制器。
    box(
        ax,
        13.15,
        2.25,
        3.55,
        0.70,
        "实物独立评价",
        ("RGB 液面与任务指标（不反馈）",),
        face="#FFFFFF",
        edge="#7A838C",
    )
    orthogonal_arrow(
        ax,
        [(xcols[4], y_b - h_b / 2), (xcols[4], 2.60)],
        dashed=True,
    )
    y_fb_b = 1.67
    orthogonal_arrow(
        ax,
        [
            (xcols[4] - 0.35, y_b - h_b / 2),
            (xcols[4] - 0.35, y_fb_b),
            (xcols[1], y_fb_b),
            (xcols[1], y_b - h_b / 2),
        ],
        dashed=True,
    )
    # 对比结论条：一眼给出相对普通 MPCC 的新增结构。
    diff = FancyBboxPatch(
        (1.25, 0.66),
        13.50,
        0.60,
        boxstyle="round,pad=0.03,rounding_size=0.16",
        linewidth=1.4,
        edgecolor=COLORS["blue"],
        facecolor="#EEF5FC",
        zorder=3,
    )
    ax.add_patch(diff)
    ax.text(
        8,
        0.96,
        "相对普通 MPCC 新增：① 显式液体状态   ② 离线防晃参考与相位漏斗   ③ 置信度门控预测时域   ④ 相位恢复与安全降级",
        ha="center",
        va="center",
        fontsize=7.0,
        color="#174A73",
        fontweight="bold",
        zorder=4,
    )
    ax.text(
        8,
        0.30,
        "灰色：两种方法的共同骨架    彩色：本文新增的防晃结构    虚线：物理反馈或独立评价",
        ha="center",
        va="center",
        fontsize=6.2,
        color="#4B5563",
    )

    for suffix in ("png", "svg", "pdf"):
        kwargs = {"dpi": 300} if suffix == "png" else {}
        fig.savefig(OUT_DIR / f"{STEM}.{suffix}", **kwargs)
    plt.close(fig)


if __name__ == "__main__":
    main()
