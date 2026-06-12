#!/usr/bin/env python3
"""Field monitor dashboard for RGB liquid debugging (subscribe-only, isolated).

Role:
  - Subscribe to existing topics for display and local export only.
  - Never publish control/perception topics, never open the camera, never record bags.
  - Independent from control packages; closing this dashboard does not affect the planner.
  - Topic names are configurable through ROS private parameters.
  - Online /liquid/* and model /spmpc/* are proxies; paper metrics use offline bag inference.

Layout:
  [RGB overlay]            [liquid height trend: max-LCR / L/C/R / model]
  [key metrics]            [topic health]
  [START] [STOP] [ZERO] [EXPORT]

Controls are local UI actions only:
  START: clear session and record from now.
  STOP: freeze current session.
  ZERO: use current liquid height as baseline h0.
  EXPORT: save current session as CSV + SVG in export_dir with timestamp.

Dependencies: rospy, cv_bridge, matplotlib, numpy. Exit with q, Esc, or by closing the window.
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

BG = "#0f172a"; PANEL = "#111827"; PANEL2 = "#172033"; CARD = "#1f2937"
INK = "#f8fafc"; MUTE = "#94a3b8"; GRID = "#334155"; FAINT = "#1e293b"
C_MAX = "#38bdf8"; C_L = "#93c5fd"; C_C = "#2dd4bf"; C_R = "#fb923c"; C_MODEL = "#c084fc"
OK = "#22c55e"; BAD = "#ef4444"; WARN = "#f59e0b"

CSV_HEADER = ["t_s", "max_lcr_mm", "L_mm", "C_mm", "R_mm", "model_h_mm", "cmd_v", "odom_v", "solver_ms"]


class Monitor:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/liquid/debug_image")
        self.liquid_height_topic = rospy.get_param("~liquid_height_topic", "/liquid/height")
        self.liquid_lcr_topic = rospy.get_param("~liquid_lcr_topic", "/liquid/height_lcr")
        self.model_height_topic = rospy.get_param("~model_height_topic", "/spmpc/slosh_height")
        self.solver_time_topic = rospy.get_param("~solver_time_topic", "/spmpc/solver_time_ms")
        self.slosh_summary_topic = rospy.get_param("~slosh_summary_topic", "/spmpc/slosh_horizon_summary")
        self.status_topic = rospy.get_param("~status_topic", "/spmpc/status")
        self.backend_topic = rospy.get_param("~backend_topic", "/spmpc/solver_backend")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.history_sec = float(rospy.get_param("~history_sec", 20.0))     # display window
        self.max_session_sec = float(rospy.get_param("~max_session_sec", 300.0))  # session cap
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

        self.session = deque()       # (t, max, l, c, r, model, cmd_v, odom_v, solver_ms) session log
        self.running = True
        self.h0_max = 0.0
        self.h0_model = 0.0
        self.zeroed = False
        self.last_export = ""

        rospy.Subscriber(self.image_topic, Image, self._img_cb, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber(self.liquid_height_topic, Float32, self._height_cb, queue_size=10)
        rospy.Subscriber(self.liquid_lcr_topic, Float32MultiArray, self._lcr_cb, queue_size=10)
        rospy.Subscriber(self.model_height_topic, Float32, self._mk("model_h", "model_h"), queue_size=10)
        rospy.Subscriber(self.solver_time_topic, Float32, self._mk("solver_ms", "solver_ms"), queue_size=10)
        rospy.Subscriber(self.slosh_summary_topic, Float32MultiArray, self._summary_cb, queue_size=5)
        rospy.Subscriber(self.status_topic, String, self._str_cb("status"), queue_size=5)
        rospy.Subscriber(self.backend_topic, String, self._str_cb("backend"), queue_size=5)
        rospy.Subscriber(self.cmd_topic, Twist, self._cmd_cb, queue_size=5)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=5)
        rospy.loginfo("[monitor] subscribed liquid=(%s,%s) spmpc=(%s,%s,%s,%s,%s) cmd=%s odom=%s",
                      self.liquid_height_topic, self.liquid_lcr_topic,
                      self.model_height_topic, self.solver_time_topic, self.slosh_summary_topic,
                      self.status_topic, self.backend_topic, self.cmd_topic, self.odom_topic)

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
        # /liquid/height is the primary signal; record samples at camera/message rate.
        with self.lock:
            self.liquid_max = float(msg.data); self.t["liquid"] = time.time()
            if self.running:
                now = time.time()
                self.session.append((now, self.liquid_max, self.liquid_lcr[0], self.liquid_lcr[1],
                                     self.liquid_lcr[2], self.model_h, self.cmd_v, self.odom_v, self.solver_ms))
                if self.max_session_sec > 0:
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

    # ---- UI callbacks: display/local export only ----
    def on_start(self, _):
        with self.lock:
            self.session.clear(); self.h0_max = 0.0; self.h0_model = 0.0
            self.zeroed = False; self.running = True
        rospy.loginfo("[monitor] started a new session")

    def on_stop(self, _):
        with self.lock:
            self.running = False
        rospy.loginfo("[monitor] stopped recording, session samples=%d", len(self.session))

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
            rospy.logwarn("[monitor] session is empty; press START and record a segment before export.")
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
        rospy.loginfo("[monitor] exported: %s.csv / .svg (%d samples)", base, len(samples))


def _isnan(x):
    return x is None or (isinstance(x, float) and np.isnan(x))


def _fmt(x, unit="", digits=2):
    if _isnan(x):
        return "--"
    return (("%%.%df" % digits) % x) + unit


def _style_panel(ax, title=None):
    ax.set_facecolor(PANEL)
    for s in ax.spines.values():
        s.set_color(GRID)
        s.set_linewidth(1.0)
    if title:
        ax.set_title(title, color=INK, fontsize=12, fontweight="bold", pad=10, loc="left")


def _prep_text_panel(ax):
    ax.clear()
    ax.set_facecolor(PANEL)
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    for s in ax.spines.values():
        s.set_visible(False)


def _pill(ax, x, y, text, color, size=10, fg="white", pad=0.28):
    ax.text(x, y, " %s " % text, fontsize=size, color=fg, va="center", ha="left", fontweight="bold",
            bbox=dict(boxstyle="round,pad=%.2f,rounding_size=0.20" % pad, fc=color, ec="none"))


def _card(ax, x, y, w, h, title, value, unit="", color=C_MAX, sub=""):
    ax.add_patch(plt.Rectangle((x, y), w, h, facecolor=CARD, edgecolor=GRID, linewidth=0.8,
                               transform=ax.transAxes, clip_on=False))
    ax.add_patch(plt.Rectangle((x, y + h - 0.012), w, 0.012, facecolor=color, edgecolor="none",
                               transform=ax.transAxes, clip_on=False))
    ax.text(x + 0.030, y + h - 0.055, title, color=MUTE, fontsize=8, va="top", ha="left")
    ax.text(x + 0.030, y + 0.095, value, color=INK, fontsize=13, va="bottom", ha="left", fontweight="bold")
    if unit:
        ax.text(x + w - 0.030, y + 0.097, unit, color=MUTE, fontsize=8, va="bottom", ha="right")
    if sub:
        ax.text(x + 0.030, y + 0.024, sub, color=MUTE, fontsize=7, va="bottom", ha="left")


def _metric_cell(ax, x, y, w, h, label, value, color=C_MAX):
    ax.add_patch(plt.Rectangle((x, y), w, h, facecolor=CARD, edgecolor=GRID, linewidth=0.8,
                               transform=ax.transAxes, clip_on=False))
    ax.add_patch(plt.Rectangle((x, y + h - 0.010), w, 0.010, facecolor=color, edgecolor="none",
                               transform=ax.transAxes, clip_on=False))
    ax.text(x + 0.025, y + h - 0.045, label, color=MUTE, fontsize=7.5, va="top", ha="left")
    ax.text(x + w * 0.50, y + h * 0.24, value, color=INK, fontsize=11.5,
            va="center", ha="center", fontweight="bold")


def _status_row(ax, y, name, ok_text, ok_color, age=None):
    ax.text(0.04, y, name, color=INK, fontsize=10, va="center", ha="left", fontweight="bold")
    ax.scatter([0.35], [y], s=95, c=[ok_color], marker="o", edgecolors="none")
    ax.text(0.40, y, ok_text, color=ok_color, fontsize=10, va="center", ha="left", fontweight="bold")
    if age is not None:
        ax.text(0.96, y, age, color=MUTE, fontsize=9, va="center", ha="right")


def _age_text(m, key):
    last = m.t.get(key, 0.0)
    if last <= 0.0:
        return "no msg"
    return "%.1fs" % (time.time() - last)


def main():
    rospy.init_node("liquid_experiment_monitor")
    m = Monitor()

    plt.rcParams["axes.edgecolor"] = GRID
    plt.rcParams["axes.labelcolor"] = MUTE
    plt.rcParams["xtick.color"] = MUTE
    plt.rcParams["ytick.color"] = MUTE
    plt.rcParams["text.color"] = INK

    fig = plt.figure("RGB liquid online monitor", figsize=(14.5, 8.8), facecolor=BG)
    ax_header = fig.add_axes([0.025, 0.905, 0.95, 0.075])
    ax_img = fig.add_axes([0.025, 0.455, 0.43, 0.425])
    ax_curve = fig.add_axes([0.485, 0.455, 0.49, 0.425])
    ax_kpi = fig.add_axes([0.025, 0.205, 0.60, 0.205])
    ax_state = fig.add_axes([0.65, 0.205, 0.325, 0.205])
    ax_footer = fig.add_axes([0.025, 0.055, 0.95, 0.105])

    button_specs = [([0.055, 0.010, 0.105, 0.045], "START", OK, "#16a34a", m.on_start),
                    ([0.175, 0.010, 0.105, 0.045], "STOP", BAD, "#dc2626", m.on_stop),
                    ([0.295, 0.010, 0.105, 0.045], "ZERO", C_MODEL, "#a855f7", m.on_zero),
                    ([0.415, 0.010, 0.105, 0.045], "EXPORT", C_MAX, "#0ea5e9", m.on_export)]
    buttons = []
    for rect, lab, col, hov, cb in button_specs:
        b = Button(fig.add_axes(rect), lab, color=col, hovercolor=hov)
        b.label.set_fontsize(11); b.label.set_color("white"); b.label.set_fontweight("bold"); b.on_clicked(cb)
        buttons.append(b)

    def on_key(event):
        if event.key in ("q", "escape"):
            rospy.signal_shutdown("user quit"); plt.close(fig)
    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_):
        with m.lock:
            img = m.img; running = m.running; zeroed = m.zeroed
            h0m, h0mod = m.h0_max, m.h0_model
            liquid_max = m.liquid_max; lcr = list(m.liquid_lcr)
            model_h, h_peak = m.model_h, m.h_peak_pred
            status, backend, solver_ms = m.status, m.backend, m.solver_ms
            cmd_v, odom_v, ax_est = m.cmd_v, m.odom_v, m.ax_est
            n_sess = len(m.session); last_export = m.last_export
            now = time.time()
            win = [s for s in m.session if now - s[0] <= m.history_sec]
            img_fresh = m.fresh("img"); liquid_fresh = m.fresh("liquid")
            status_fresh = m.fresh("status"); cmd_fresh = m.fresh("cmd"); odom_fresh = m.fresh("odom")
            solver_fresh = m.fresh("solver_ms")

        # Header
        _prep_text_panel(ax_header)
        ax_header.set_facecolor(BG)
        ax_header.text(0.00, 0.66, "RGB Liquid Online Monitor", fontsize=18, fontweight="bold", ha="left", va="center")
        ax_header.text(0.00, 0.22, "Subscribe-only dashboard · offline RGB max-LCR for paper metrics",
                       fontsize=10, color=MUTE, ha="left", va="center")
        status_short = status if len(status) <= 22 else status[:19] + "..."
        _pill(ax_header, 0.60, 0.66, "RUNNING" if running else "FROZEN", OK if running else WARN, size=9)
        _pill(ax_header, 0.70, 0.66, "ZEROED" if zeroed else "ABS", C_MODEL if zeroed else GRID, size=9)
        st_color = OK if (status_fresh and "FAIL" not in status.upper()) else BAD
        _pill(ax_header, 0.79, 0.66, status_short if status_short and status_short != "-" else "NO STATUS", st_color, size=8)
        ax_header.text(0.60, 0.22, "backend: %s" % (backend or "--"), fontsize=9, color=MUTE, ha="left", va="center")
        ax_header.text(0.98, 0.22, "samples: %d" % n_sess, fontsize=9, color=MUTE, ha="right", va="center")

        # RGB image
        ax_img.clear(); ax_img.axis("off"); ax_img.set_facecolor(PANEL)
        ax_img.set_title("RGB Overlay / Online Liquid Detection", color=INK, fontsize=12, fontweight="bold", loc="left", pad=10)
        if img is not None and img_fresh:
            ax_img.imshow(img)
            ax_img.text(0.02, 0.04, "LIVE", transform=ax_img.transAxes, color="white", fontsize=10, fontweight="bold",
                        bbox=dict(boxstyle="round,pad=0.30", fc=OK, ec="none", alpha=0.90))
        else:
            ax_img.text(0.5, 0.56, "NO DEBUG IMAGE", ha="center", va="center", color=BAD, fontsize=18, fontweight="bold")
            ax_img.text(0.5, 0.43, "Check online_liquid_height.launch publish_debug:=true\nor override image_topic",
                        ha="center", va="center", color=MUTE, fontsize=10)
        for s in ax_img.spines.values():
            s.set_visible(True); s.set_color(GRID)

        # Curve
        ax_curve.clear(); _style_panel(ax_curve, "Height Trend (last %.0fs)" % m.history_sec)
        ax_curve.grid(True, color=GRID, lw=0.7, alpha=0.75)
        if win:
            t0 = win[-1][0]; ts = [s[0] - t0 for s in win]
            ax_curve.fill_between(ts, [s[1] - h0m for s in win], 0, color=C_MAX, alpha=0.10)
            ax_curve.plot(ts, [s[1] - h0m for s in win], color=C_MAX, lw=2.6, label="max")
            ax_curve.plot(ts, [s[2] - h0m for s in win], color=C_L, lw=1.0, alpha=0.85, label="L")
            ax_curve.plot(ts, [s[3] - h0m for s in win], color=C_C, lw=1.0, alpha=0.85, label="C")
            ax_curve.plot(ts, [s[4] - h0m for s in win], color=C_R, lw=1.0, alpha=0.85, label="R")
            ax_curve.plot(ts, [s[5] - h0mod for s in win], color=C_MODEL, lw=1.6, ls="--", label="model")
            ax_curve.axhline(0, color=MUTE, lw=0.8, ls=":")
            ax_curve.set_xlim(-m.history_sec, 0.0)
            ax_curve.set_xlabel("t (s, now=0)", fontsize=8)
            ax_curve.set_ylabel("delta h (mm)" if zeroed else "height (mm)", fontsize=8)
            leg = ax_curve.legend(loc="upper right", fontsize=7, framealpha=0.85, ncol=1, borderpad=0.35, labelspacing=0.25, handlelength=1.2)
            leg.get_frame().set_facecolor(PANEL2); leg.get_frame().set_edgecolor(GRID)
            for txt in leg.get_texts():
                txt.set_color(INK)
        else:
            ax_curve.text(0.5, 0.52, "Waiting for /liquid/height", transform=ax_curve.transAxes,
                          ha="center", va="center", color=MUTE, fontsize=12)
        ax_curve.tick_params(colors=MUTE, labelsize=8)

        # KPI cards
        _prep_text_panel(ax_kpi)
        ax_kpi.text(0.02, 0.99, "Key Metrics", fontsize=13, fontweight="bold", va="top")
        rgb_value = _fmt(liquid_max - h0m if not _isnan(liquid_max) else liquid_max, " mm", 1)
        model_value = _fmt(model_h - h0mod if not _isnan(model_h) else model_h, " mm", 1)
        solver_color = OK if (solver_fresh and not _isnan(solver_ms) and solver_ms < m.solver_warn_ms) else WARN if solver_fresh else BAD
        _metric_cell(ax_kpi, 0.02, 0.47, 0.18, 0.28, "RGB MAX", rgb_value, C_MAX)
        _metric_cell(ax_kpi, 0.22, 0.47, 0.12, 0.28, "LEFT", _fmt(lcr[0] - h0m, " mm", 0), C_L)
        _metric_cell(ax_kpi, 0.36, 0.47, 0.12, 0.28, "CENTER", _fmt(lcr[1] - h0m, " mm", 0), C_C)
        _metric_cell(ax_kpi, 0.50, 0.47, 0.12, 0.28, "RIGHT", _fmt(lcr[2] - h0m, " mm", 0), C_R)
        _metric_cell(ax_kpi, 0.64, 0.47, 0.16, 0.28, "MODEL", model_value, C_MODEL)
        _metric_cell(ax_kpi, 0.82, 0.47, 0.15, 0.28, "SOLVER", _fmt(solver_ms, " ms", 1), solver_color)
        _metric_cell(ax_kpi, 0.02, 0.08, 0.18, 0.24, "CMD V", _fmt(cmd_v, " m/s", 2), OK if cmd_fresh else WARN)
        _metric_cell(ax_kpi, 0.22, 0.08, 0.18, 0.24, "ODOM V", _fmt(odom_v, " m/s", 2), OK if odom_fresh else WARN)
        _metric_cell(ax_kpi, 0.42, 0.08, 0.18, 0.24, "H PEAK", _fmt(h_peak, " mm", 1), WARN)
        _metric_cell(ax_kpi, 0.62, 0.08, 0.16, 0.24, "AX EST", _fmt(ax_est, "", 2), C_MAX)
        _metric_cell(ax_kpi, 0.80, 0.08, 0.17, 0.24, "MODE", "ZERO" if zeroed else "ABS", C_MODEL if zeroed else GRID)

        # Status panel
        _prep_text_panel(ax_state)
        ax_state.text(0.04, 0.93, "Topic Health", fontsize=13, fontweight="bold", va="top")
        liquid_ok = liquid_fresh and not _isnan(liquid_max)
        liquid_txt, liquid_c = ("OK", OK) if liquid_ok else (("NAN", WARN) if liquid_fresh else ("LOST", BAD))
        _status_row(ax_state, 0.73, "overlay", "OK" if img_fresh else "LOST", OK if img_fresh else BAD, _age_text(m, "img"))
        _status_row(ax_state, 0.55, "liquid", liquid_txt, liquid_c, _age_text(m, "liquid"))
        _status_row(ax_state, 0.37, "spmpc", "OK" if (status_fresh and "FAIL" not in status.upper()) else "WARN", OK if status_fresh else BAD, _age_text(m, "status"))
        _status_row(ax_state, 0.19, "odom/cmd", "OK" if (odom_fresh and cmd_fresh) else "WAIT", OK if (odom_fresh and cmd_fresh) else WARN, _age_text(m, "odom"))

        # Footer / operation hints
        _prep_text_panel(ax_footer)
        ax_footer.text(0.54, 0.64, "Actions: START clears and records; STOP freezes; ZERO uses current liquid as baseline; EXPORT saves CSV+SVG", fontsize=10, color=MUTE, va="center")
        ax_footer.text(0.54, 0.34, "Shortcuts: q / Esc exits. This dashboard only subscribes and exports locally; planner is unaffected.", fontsize=9, color=MUTE, va="center")
        if last_export:
            ax_footer.text(0.54, 0.10, "✓ Exported %s.csv/.svg" % os.path.basename(last_export), fontsize=9, color=OK, va="center")
        else:
            ax_footer.text(0.54, 0.10, "export_dir: %s" % os.path.expanduser(m.export_dir), fontsize=9, color=MUTE, va="center")
        return []

    ani = FuncAnimation(fig, update, interval=int(1000.0 / max(1.0, m.refresh_hz)), cache_frame_data=False)
    plt.show()
    del ani, buttons


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
