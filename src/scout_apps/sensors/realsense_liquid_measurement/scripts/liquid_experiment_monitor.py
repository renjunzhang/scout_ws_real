#!/usr/bin/env python3
"""实物实验现场监控前端(纯订阅，隔离)。

定位(见方案 docs/Claude/修改方案-时间-简介/2026-06-03_RGB在线识别前端隔离方案.md)：
  - 只订阅已有话题做显示 + 本地导出，绝不新发控制/感知话题、不开相机、不碰录包；
  - 不依赖任何控制包；杀掉本前端不影响 planner；
  - 在线 /liquid/* 与模型 /spmpc/* 都是 proxy，论文真值仍以离线推断为准。

一屏 2x2 + 底部按钮：
  [RGB overlay]            [液面高度曲线 (max-LCR / L/C/R / 模型)]
  [状态灯]                 [控制/模型数值]
  [开始]  [结束]  [归零]  [导出]

实验流程：路径跟踪开始时点"开始"，到达终点点"结束"，再点"导出"得到该段 CSV+SVG。
按钮均为纯显示/本地导出(不发任何 ROS 话题)：
  开始: 清会话 + 归零基线 + 全程记录该段
  结束: 冻结(停止记录)
  归零: 把当前液面设为基线 h0, 画相对静止偏差
  导出: 该段会话 -> CSV(数据) + SVG(矢量曲线图), 存 ~export_dir, 带时间戳

依赖：rospy, cv_bridge, matplotlib, numpy。退出：按 q 或关窗，只关本前端。
"""

import csv
import os
import threading
import time
from collections import deque
from datetime import datetime

import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray, String

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

BG = "#eceff4"; PANEL = "#ffffff"; INK = "#2e3440"; MUTE = "#6b7280"; GRID = "#e1e5ea"
C_MAX = "#1565c0"; C_L = "#90caf9"; C_C = "#80cbc4"; C_R = "#ffb74d"; C_MODEL = "#7e57c2"
OK = "#2e9e5b"; BAD = "#e0413a"; WARN = "#e8a33d"

CSV_HEADER = ["t_s", "max_lcr_mm", "L_mm", "C_mm", "R_mm", "model_h_mm", "cmd_v", "odom_v", "solver_ms"]


class Monitor:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/liquid/debug_image")
        self.history_sec = float(rospy.get_param("~history_sec", 20.0))     # 显示窗口
        self.max_session_sec = float(rospy.get_param("~max_session_sec", 300.0))  # 会话上限(防爆内存)
        self.solver_warn_ms = float(rospy.get_param("~solver_warn_ms", 33.0))
        self.stale_sec = float(rospy.get_param("~stale_sec", 1.0))
        self.refresh_hz = float(rospy.get_param("~refresh_hz", 5.0))
        self.export_dir = os.path.expanduser(rospy.get_param("~export_dir", "~/liquid_monitor_exports"))

        self.lock = threading.Lock()
        self.bridge = CvBridge()

        self.img = None
        self.t = {}
        self.liquid_max = float("nan")
        self.liquid_lcr = [float("nan")] * 3
        self.model_h = float("nan")
        self.h_peak_pred = float("nan")
        self.status = "-"
        self.backend = "-"
        self.solver_ms = float("nan")
        self.cmd_v = float("nan")
        self.odom_v = float("nan")
        self.prev_odom_v = None
        self.prev_odom_t = None
        self.ax_est = float("nan")

        self.session = deque()       # (t, max, l, c, r, model, cmd_v, odom_v, solver_ms) 全程记录
        self.running = True
        self.h0_max = 0.0
        self.h0_model = 0.0
        self.zeroed = False
        self.last_export = ""

        rospy.Subscriber(self.image_topic, Image, self._img_cb, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber("/liquid/height", Float32, self._height_cb, queue_size=10)
        rospy.Subscriber("/liquid/height_lcr", Float32MultiArray, self._lcr_cb, queue_size=10)
        rospy.Subscriber("/spmpc/slosh_height", Float32, self._mk("model_h", "model_h"), queue_size=10)
        rospy.Subscriber("/spmpc/solver_time_ms", Float32, self._mk("solver_ms", "solver_ms"), queue_size=10)
        rospy.Subscriber("/spmpc/slosh_horizon_summary", Float32MultiArray, self._summary_cb, queue_size=5)
        rospy.Subscriber("/spmpc/status", String, self._str_cb("status"), queue_size=5)
        rospy.Subscriber("/spmpc/solver_backend", String, self._str_cb("backend"), queue_size=5)
        rospy.Subscriber("/cmd_vel", Twist, self._cmd_cb, queue_size=5)
        rospy.Subscriber("/odom", Odometry, self._odom_cb, queue_size=5)

    def _mk(self, key, attr):
        def cb(msg):
            with self.lock:
                setattr(self, attr, float(msg.data)); self.t[key] = time.time()
        return cb

    def _str_cb(self, attr):
        def cb(msg):
            with self.lock:
                setattr(self, attr, msg.data); self.t[attr] = time.time()
        return cb

    def _img_cb(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:  # noqa: BLE001
            return
        with self.lock:
            self.img = bgr[:, :, ::-1].copy(); self.t["img"] = time.time()

    def _lcr_cb(self, msg):
        vals = [float(x) for x in msg.data[:3]]
        with self.lock:
            self.liquid_lcr = vals + [float("nan")] * (3 - len(vals)); self.t["liquid"] = time.time()

    def _height_cb(self, msg):
        # /liquid/height 是主信号; 在它的回调里按相机帧率记录会话(比 5Hz 显示刷新更细)。
        with self.lock:
            self.liquid_max = float(msg.data); self.t["liquid"] = time.time()
            if self.running:
                now = time.time()
                self.session.append((now, self.liquid_max, self.liquid_lcr[0], self.liquid_lcr[1],
                                     self.liquid_lcr[2], self.model_h, self.cmd_v, self.odom_v, self.solver_ms))
                while self.session and now - self.session[0][0] > self.max_session_sec:
                    self.session.popleft()

    def _summary_cb(self, msg):
        if len(msg.data) >= 2:
            with self.lock:
                self.h_peak_pred = float(msg.data[0]); self.t["spmpc"] = time.time()

    def _cmd_cb(self, msg):
        with self.lock:
            self.cmd_v = float(msg.linear.x); self.t["cmd"] = time.time()

    def _odom_cb(self, msg):
        now = time.time(); v = float(msg.twist.twist.linear.x)
        with self.lock:
            if self.prev_odom_t is not None and now > self.prev_odom_t:
                self.ax_est = (v - self.prev_odom_v) / (now - self.prev_odom_t)
            self.prev_odom_v, self.prev_odom_t = v, now
            self.odom_v = v; self.t["odom"] = time.time()

    def fresh(self, key):
        return (time.time() - self.t.get(key, 0.0)) < self.stale_sec

    # ---- 按钮(纯显示/本地导出) ----
    def on_start(self, _):
        with self.lock:
            self.session.clear(); self.h0_max = 0.0; self.h0_model = 0.0
            self.zeroed = False; self.running = True
        rospy.loginfo("[monitor] 开始记录新会话")

    def on_stop(self, _):
        with self.lock:
            self.running = False
        rospy.loginfo("[monitor] 结束记录, 会话样本数=%d", len(self.session))

    def on_zero(self, _):
        with self.lock:
            if not np.isnan(self.liquid_max):
                self.h0_max = self.liquid_max
            if not np.isnan(self.model_h):
                self.h0_model = self.model_h
            self.zeroed = True

    def on_export(self, _):
        with self.lock:
            samples = list(self.session)
            h0m, h0mod = self.h0_max, self.h0_model
        if not samples:
            rospy.logwarn("[monitor] 会话为空, 先点开始记录一段再导出。")
            return
        os.makedirs(self.export_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.join(self.export_dir, "liquid_session_%s" % stamp)
        t0 = samples[0][0]

        def cell(x):
            return "" if (x is None or (isinstance(x, float) and np.isnan(x))) else "%.5f" % x

        with open(base + ".csv", "w", newline="") as f:
            w = csv.writer(f); w.writerow(CSV_HEADER)
            for s in samples:
                w.writerow(["%.3f" % (s[0] - t0)] + [cell(v) for v in s[1:]])

        fig = plt.figure(figsize=(9, 4.5))
        ax = fig.add_subplot(111)
        ts = [s[0] - t0 for s in samples]
        ax.plot(ts, [s[1] - h0m for s in samples], color=C_MAX, lw=2.0, label="RGB max-LCR")
        ax.plot(ts, [s[2] - h0m for s in samples], color=C_L, lw=0.9, label="L")
        ax.plot(ts, [s[3] - h0m for s in samples], color=C_C, lw=0.9, label="C")
        ax.plot(ts, [s[4] - h0m for s in samples], color=C_R, lw=0.9, label="R")
        ax.plot(ts, [s[5] - h0mod for s in samples], color=C_MODEL, lw=1.3, ls="--", label="model h")
        ax.axhline(0, color=MUTE, lw=0.6, ls=":")
        ax.set_xlabel("t (s)"); ax.set_ylabel("liquid height (mm)" + ("  [zeroed]" if (h0m or h0mod) else ""))
        ax.set_title("liquid_session_%s" % stamp); ax.grid(True, alpha=0.3); ax.legend(loc="upper left", fontsize=8, ncol=2)
        fig.tight_layout(); fig.savefig(base + ".svg"); plt.close(fig)

        with self.lock:
            self.last_export = base
        rospy.loginfo("[monitor] 已导出: %s.csv / .svg (%d 样本)", base, len(samples))


def _style_panel(ax, title):
    ax.set_facecolor(PANEL)
    ax.set_title(title, color=INK, fontsize=11, fontweight="bold", pad=6)
    for s in ax.spines.values():
        s.set_color(GRID)


def main():
    rospy.init_node("liquid_experiment_monitor")
    m = Monitor()

    plt.rcParams["axes.edgecolor"] = GRID
    fig = plt.figure("liquid experiment monitor", figsize=(12, 8.2), facecolor=BG)
    ax_img = fig.add_axes([0.035, 0.55, 0.44, 0.40]); ax_img.axis("off")
    ax_curve = fig.add_axes([0.55, 0.55, 0.42, 0.40])
    ax_state = fig.add_axes([0.035, 0.16, 0.44, 0.34]); ax_state.axis("off")
    ax_ctrl = fig.add_axes([0.55, 0.16, 0.42, 0.34]); ax_ctrl.axis("off")
    ax_img.set_title("RGB overlay", color=INK, fontsize=11, fontweight="bold")

    specs = [([0.06, 0.04, 0.12, 0.06], "开始", "#d7f0df", "#bfe6cd", m.on_start),
             ([0.20, 0.04, 0.12, 0.06], "结束", "#fbdcda", "#f6c6c2", m.on_stop),
             ([0.34, 0.04, 0.12, 0.06], "归零", "#e7e0f6", "#d6c9f0", m.on_zero),
             ([0.48, 0.04, 0.12, 0.06], "导出", "#d6e4f7", "#bcd4f2", m.on_export)]
    buttons = []
    for rect, lab, col, hov, cb in specs:
        b = Button(fig.add_axes(rect), lab, color=col, hovercolor=hov)
        b.label.set_fontsize(12); b.label.set_color(INK); b.on_clicked(cb)
        buttons.append(b)

    def on_key(event):
        if event.key in ("q", "escape"):
            rospy.signal_shutdown("user quit"); plt.close(fig)
    fig.canvas.mpl_connect("key_press_event", on_key)

    def badge(ax, x, y, name, txt, color):
        ax.text(x, y, name, fontsize=12, va="center", color=INK)
        ax.text(x + 0.5, y, "  %s  " % txt, fontsize=12, va="center", color="white",
                fontweight="bold", bbox=dict(boxstyle="round,pad=0.35", fc=color, ec="none"))

    def update(_):
        with m.lock:
            img = m.img; running = m.running; zeroed = m.zeroed
            h0m, h0mod = m.h0_max, m.h0_model
            liquid_max = m.liquid_max
            model_h, h_peak = m.model_h, m.h_peak_pred
            status, backend, solver_ms = m.status, m.backend, m.solver_ms
            cmd_v, odom_v, ax_est = m.cmd_v, m.odom_v, m.ax_est
            n_sess = len(m.session); last_export = m.last_export
            now = time.time()
            win = [s for s in m.session if now - s[0] <= m.history_sec]
            img_fresh = m.fresh("img"); liquid_fresh = m.fresh("liquid"); status_fresh = m.fresh("status")

        ax_img.clear(); ax_img.axis("off")
        ax_img.set_title("RGB overlay", color=INK, fontsize=11, fontweight="bold")
        if img is not None and img_fresh:
            ax_img.imshow(img)
        else:
            ax_img.text(0.5, 0.5, "no /liquid/debug_image\n(online publish_debug:=true ?)",
                        ha="center", va="center", color=BAD, fontsize=11)

        ax_curve.clear(); _style_panel(ax_curve, "液面高度 (mm)" + ("  [归零]" if zeroed else ""))
        ax_curve.grid(True, color=GRID, lw=0.8)
        if win:
            t0 = win[-1][0]; ts = [s[0] - t0 for s in win]
            ax_curve.plot(ts, [s[1] - h0m for s in win], color=C_MAX, lw=2.2, label="RGB max-LCR")
            ax_curve.plot(ts, [s[2] - h0m for s in win], color=C_L, lw=0.9, label="L")
            ax_curve.plot(ts, [s[3] - h0m for s in win], color=C_C, lw=0.9, label="C")
            ax_curve.plot(ts, [s[4] - h0m for s in win], color=C_R, lw=0.9, label="R")
            ax_curve.plot(ts, [s[5] - h0mod for s in win], color=C_MODEL, lw=1.4, ls="--", label="model h")
            ax_curve.axhline(0, color=MUTE, lw=0.6, ls=":")
            ax_curve.set_xlabel("t (s, 相对当前)", color=MUTE, fontsize=9)
            ax_curve.legend(loc="upper left", fontsize=8, framealpha=0.9, ncol=2)
        ax_curve.tick_params(colors=MUTE, labelsize=8)

        ax_state.clear(); ax_state.axis("off")
        ax_state.text(0.0, 1.02, "状态", fontsize=13, fontweight="bold", color=INK, va="top")
        liquid_ok = liquid_fresh and not np.isnan(liquid_max)
        liquid_txt, liquid_c = ("OK", OK) if liquid_ok else (("NAN", WARN) if liquid_fresh else ("LOST", BAD))
        solver_ok = (not np.isnan(solver_ms)) and solver_ms < m.solver_warn_ms
        rows = [("OVERLAY", ("OK", OK) if img_fresh else ("--", BAD)),
                ("LIQUID", (liquid_txt, liquid_c)),
                ("SPMPC", ("OK", OK) if (status_fresh and "FAIL" not in status.upper()) else ("--", BAD)),
                ("SOLVER", ("< %dms" % m.solver_warn_ms, OK) if solver_ok else ("SLOW", BAD))]
        for i, (name, (txt, c)) in enumerate(rows):
            badge(ax_state, 0.0, 0.80 - i * 0.20, name, txt, c)
        run_txt, run_c = ("RUNNING", OK) if running else ("FROZEN", WARN)
        ax_state.text(0.0, -0.02, "采集", fontsize=11, color=MUTE, va="center")
        ax_state.text(0.5, -0.02, "  %s  " % run_txt, fontsize=11, va="center", color="white",
                      fontweight="bold", bbox=dict(boxstyle="round,pad=0.3", fc=run_c, ec="none"))
        ax_state.text(0.0, -0.13, "会话样本 %d" % n_sess, fontsize=10, color=MUTE, va="center")

        ax_ctrl.clear(); ax_ctrl.axis("off")
        ax_ctrl.text(0.0, 1.02, "控制 / 模型", fontsize=13, fontweight="bold", color=INK, va="top")

        def fnum(x, u=""):
            return "NA" if (x is None or (isinstance(x, float) and np.isnan(x))) else ("%.3f%s" % (x, u))
        lines = [("status", status), ("backend", backend), ("solver_ms", fnum(solver_ms)),
                 ("cmd_v", fnum(cmd_v, " m/s")), ("odom_v", fnum(odom_v, " m/s")),
                 ("ax_est", fnum(ax_est, " m/s2")), ("model h", fnum(model_h, " mm")),
                 ("h_peak_pred", fnum(h_peak, " mm"))]
        for i, (k, val) in enumerate(lines):
            y = 0.84 - i * 0.108
            ax_ctrl.text(0.0, y, k, fontsize=11, color=MUTE, va="center")
            ax_ctrl.text(0.42, y, str(val), fontsize=11, color=INK, va="center")
        if last_export:
            ax_ctrl.text(0.0, -0.05, "✓ 已导出: %s.csv/.svg" % os.path.basename(last_export),
                         fontsize=9, color=OK, va="center")
        return []

    ani = FuncAnimation(fig, update, interval=int(1000.0 / max(1.0, m.refresh_hz)), cache_frame_data=False)
    plt.show()
    del ani, buttons


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
