#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Simulate Scout's 2nd-order slosh ODE as an input-output system.

The model is a pair of decoupled mass-spring-damper systems:

    eta_x'' + 2*zeta*omega_n*eta_x' + omega_n^2*eta_x = -a_x
    eta_y'' + 2*zeta*omega_n*eta_y' + omega_n^2*eta_y = -a_y

Important interpretation:
    input   : a_x, a_y
    memory  : eta_x, eta_x_dot, eta_y, eta_y_dot
    outputs : slosh state for MPC, plus h_model as a model-risk proxy

Examples:
    python3 simulate_slosh_ode.py --scenario brake_turn
    python3 simulate_slosh_ode.py --scenario pulse --init-eta-x-mm 2.0
    python3 simulate_slosh_ode.py --scenario sine --out /tmp/slosh_ode.png --csv /tmp/slosh_ode.csv
"""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import expm


DEFAULT_OUT_DIR = Path("/data/a/Obsidian/vaults/StudyVault/attachments/projects/MPC")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--omega", type=float, default=31.25, help="Modal natural frequency omega_n (rad/s)")
    parser.add_argument("--zeta", type=float, default=0.05, help="Damping ratio")
    parser.add_argument("--h-coeff", type=float, default=1.0, help="Height coefficient for h_model")
    parser.add_argument("--dt", type=float, default=1.0 / 30.0, help="Discretisation step (s)")
    parser.add_argument("--duration", type=float, default=6.0, help="Simulation duration (s)")
    parser.add_argument(
        "--scenario",
        choices=("pulse", "brake_turn", "sine", "zero"),
        default="brake_turn",
        help="Acceleration input profile",
    )
    parser.add_argument("--amp-x", type=float, default=2.0, help="Longitudinal acceleration amplitude (m/s^2)")
    parser.add_argument("--amp-y", type=float, default=1.2, help="Lateral acceleration amplitude (m/s^2)")
    parser.add_argument("--input-start", type=float, default=0.3, help="Input starts at this time (s)")
    parser.add_argument("--input-end", type=float, default=1.8, help="Input is zero after this time for pulse/brake_turn (s)")
    parser.add_argument("--sine-freq", type=float, default=1.0, help="Sine input frequency for --scenario sine (Hz)")
    parser.add_argument("--init-eta-x-mm", type=float, default=0.0, help="Initial eta_x residual slosh (mm)")
    parser.add_argument("--init-eta-y-mm", type=float, default=0.0, help="Initial eta_y residual slosh (mm)")
    parser.add_argument("--init-eta-x-dot-mm-s", type=float, default=0.0, help="Initial eta_x_dot (mm/s)")
    parser.add_argument("--init-eta-y-dot-mm-s", type=float, default=0.0, help="Initial eta_y_dot (mm/s)")
    parser.add_argument("--out", default="", help="Output plot path")
    parser.add_argument("--csv", default="", help="Optional CSV output path with the simulated time series")
    parser.add_argument("--no-plot", action="store_true", help="Only print numeric summary")
    return parser.parse_args()


def discrete_matrices(omega_n, zeta, dt):
    """ZOH discretisation of one slosh channel."""
    if omega_n <= 0.0:
        raise ValueError("omega_n must be positive")
    if zeta < 0.0:
        raise ValueError("zeta must be non-negative")
    if dt <= 0.0:
        raise ValueError("dt must be positive")

    wn2 = omega_n * omega_n
    damp = 2.0 * zeta * omega_n
    a2 = np.array([[0.0, 1.0], [-wn2, -damp]])
    b2 = np.array([[0.0], [-1.0]])

    m = np.zeros((3, 3))
    m[:2, :2] = a2 * dt
    m[:2, 2] = b2[:, 0] * dt
    em = expm(m)
    return em[:2, :2], em[:2, 2]


def raised_cosine(t, start, end, amp):
    """Smooth 0 -> amp -> 0 bump over [start, end]."""
    y = np.zeros_like(t)
    if end <= start:
        return y
    mask = (t >= start) & (t <= end)
    phase = (t[mask] - start) / (end - start)
    y[mask] = amp * 0.5 * (1.0 - np.cos(2.0 * np.pi * phase))
    return y


def smooth_step(t, start, end, amp):
    """Smooth 0 -> amp -> 0 plateau with cosine ramps."""
    y = np.zeros_like(t)
    if end <= start:
        return y
    ramp = min(0.3, 0.25 * (end - start))
    up = (t >= start) & (t < start + ramp)
    hold = (t >= start + ramp) & (t < end - ramp)
    down = (t >= end - ramp) & (t <= end)
    if ramp > 0.0:
        y[up] = amp * 0.5 * (1.0 - np.cos(np.pi * (t[up] - start) / ramp))
        y[down] = amp * 0.5 * (1.0 + np.cos(np.pi * (t[down] - (end - ramp)) / ramp))
    y[hold] = amp
    return y


def build_inputs(t, args):
    """Return a_x, a_y profiles for controlled ODE experiments."""
    if args.scenario == "zero":
        return np.zeros_like(t), np.zeros_like(t)

    if args.scenario == "pulse":
        ax = raised_cosine(t, args.input_start, args.input_end, args.amp_x)
        ay = np.zeros_like(t)
        return ax, ay

    if args.scenario == "sine":
        ax = args.amp_x * np.sin(2.0 * np.pi * args.sine_freq * t)
        ay = args.amp_y * np.sin(2.0 * np.pi * args.sine_freq * t + 0.5 * np.pi)
        return ax, ay

    ax = -smooth_step(t, args.input_start, args.input_end, abs(args.amp_x))
    ay = smooth_step(t, args.input_start + 0.15, args.input_end + 0.35, abs(args.amp_y))
    return ax, ay


def simulate(ax, ay, ad, bd, h_coeff, initial_state):
    """Run the discrete slosh ODE.

    initial_state is [eta_x, eta_x_dot, eta_y, eta_y_dot] in SI units.
    """
    sx = np.array([initial_state[0], initial_state[1]], dtype=float)
    sy = np.array([initial_state[2], initial_state[3]], dtype=float)

    eta_x = []
    eta_x_dot = []
    eta_y = []
    eta_y_dot = []
    h_model = []

    for a_x, a_y in zip(ax, ay):
        sx = ad @ sx + bd * a_x
        sy = ad @ sy + bd * a_y
        eta_x.append(sx[0])
        eta_x_dot.append(sx[1])
        eta_y.append(sy[0])
        eta_y_dot.append(sy[1])
        h_model.append(h_coeff * np.hypot(sx[0], sy[0]))

    return {
        "eta_x": np.asarray(eta_x),
        "eta_x_dot": np.asarray(eta_x_dot),
        "eta_y": np.asarray(eta_y),
        "eta_y_dot": np.asarray(eta_y_dot),
        "h_model": np.asarray(h_model),
    }


def last_nonzero_time(t, ax, ay):
    mag = np.hypot(ax, ay)
    idx = np.where(mag > 1.0e-6)[0]
    if len(idx) == 0:
        return 0.0
    return float(t[idx[-1]])


def summarise(t, ax, ay, state, args):
    h_mm = state["h_model"] * 1000.0
    eta_norm_mm = np.hypot(state["eta_x"], state["eta_y"]) * 1000.0
    input_end = last_nonzero_time(t, ax, ay)
    free_mask = t >= input_end
    if np.any(free_mask):
        free_peak_mm = float(np.max(h_mm[free_mask]))
    else:
        free_peak_mm = 0.0

    tau = float("inf") if args.zeta == 0.0 else 1.0 / (args.zeta * args.omega)
    peak_idx = int(np.argmax(h_mm))
    summary = {
        "scenario": args.scenario,
        "omega_n_rad_s": args.omega,
        "zeta": args.zeta,
        "dt_s": args.dt,
        "tau_s": tau,
        "input_end_s": input_end,
        "peak_h_model_mm": float(np.max(h_mm)),
        "peak_eta_norm_mm": float(np.max(eta_norm_mm)),
        "peak_time_s": float(t[peak_idx]),
        "free_decay_peak_h_model_mm": free_peak_mm,
        "max_abs_ax_m_s2": float(np.max(np.abs(ax))),
        "max_abs_ay_m_s2": float(np.max(np.abs(ay))),
    }
    return summary


def print_summary(summary, t, state):
    print("==== Slosh ODE input-output summary ====")
    for key, value in summary.items():
        if isinstance(value, float):
            print(f"{key}: {value:.6g}")
        else:
            print(f"{key}: {value}")

    tau = summary["tau_s"]
    if np.isfinite(tau) and tau > 0.0:
        print("\nFree-decay checkpoints:")
        for n in range(0, 7):
            ti = summary["input_end_s"] + n * tau
            if ti > t[-1]:
                break
            hi = np.interp(ti, t, state["h_model"] * 1000.0)
            pct = 100.0 * np.exp(-n)
            print(f"  t={ti:.3f}s  {n}tau  h_model={hi:.4f}mm  envelope={pct:.1f}%")


def save_csv(path, t, ax, ay, state):
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    data = np.column_stack(
        [
            t,
            ax,
            ay,
            state["eta_x"] * 1000.0,
            state["eta_x_dot"] * 1000.0,
            state["eta_y"] * 1000.0,
            state["eta_y_dot"] * 1000.0,
            state["h_model"] * 1000.0,
        ]
    )
    header = "t_s,ax_m_s2,ay_m_s2,eta_x_mm,eta_x_dot_mm_s,eta_y_mm,eta_y_dot_mm_s,h_model_mm"
    np.savetxt(out, data, delimiter=",", header=header, comments="", fmt="%.9g")
    print(f"CSV saved: {out}")


def plot_result(path, t, ax, ay, state, summary):
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)

    h_mm = state["h_model"] * 1000.0
    eta_x_mm = state["eta_x"] * 1000.0
    eta_y_mm = state["eta_y"] * 1000.0
    eta_x_dot_mm_s = state["eta_x_dot"] * 1000.0
    eta_y_dot_mm_s = state["eta_y_dot"] * 1000.0

    fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=False)
    ax0, ax1, ax2, ax3 = axes

    ax0.plot(t, ax, color="#c62828", lw=1.5, label="a_x input")
    ax0.plot(t, ay, color="#1565c0", lw=1.5, label="a_y input")
    ax0.axvline(summary["input_end_s"], color="gray", ls=":", lw=1.0, label="input ends")
    ax0.set_ylabel("accel (m/s^2)")
    ax0.set_title("Input: container acceleration")
    ax0.legend(loc="upper right")
    ax0.grid(True, alpha=0.3)

    ax1.plot(t, eta_x_mm, color="#ef6c00", lw=1.3, label="eta_x")
    ax1.plot(t, eta_y_mm, color="#00897b", lw=1.3, label="eta_y")
    ax1.plot(t, eta_x_dot_mm_s, color="#ef6c00", ls="--", lw=0.9, alpha=0.8, label="eta_x_dot")
    ax1.plot(t, eta_y_dot_mm_s, color="#00897b", ls="--", lw=0.9, alpha=0.8, label="eta_y_dot")
    ax1.axvline(summary["input_end_s"], color="gray", ls=":", lw=1.0)
    ax1.set_ylabel("state (mm, mm/s)")
    ax1.set_title("Memory state: eta and eta_dot")
    ax1.legend(loc="upper right", ncol=2, fontsize=8)
    ax1.grid(True, alpha=0.3)

    ax2.plot(t, h_mm, color="#6a1b9a", lw=1.6, label="h_model")
    ax2.axvline(summary["input_end_s"], color="gray", ls=":", lw=1.0)
    ax2.scatter([summary["peak_time_s"]], [summary["peak_h_model_mm"]], color="#6a1b9a", s=25, zorder=3)
    ax2.annotate(
        f"peak {summary['peak_h_model_mm']:.3f} mm",
        xy=(summary["peak_time_s"], summary["peak_h_model_mm"]),
        xytext=(summary["peak_time_s"] + 0.15, summary["peak_h_model_mm"] + 0.05),
        fontsize=8,
        arrowprops={"arrowstyle": "->", "lw": 0.8},
    )
    ax2.set_ylabel("h_model (mm)")
    ax2.set_title("Output: model height proxy, not RGB truth")
    ax2.legend(loc="upper right")
    ax2.grid(True, alpha=0.3)

    ax3.plot(eta_x_mm, eta_x_dot_mm_s, color="#ef6c00", lw=1.1, label="x channel phase")
    ax3.plot(eta_y_mm, eta_y_dot_mm_s, color="#00897b", lw=1.1, label="y channel phase")
    ax3.scatter([eta_x_mm[0]], [eta_x_dot_mm_s[0]], color="#ef6c00", marker="o", s=20)
    ax3.scatter([eta_y_mm[0]], [eta_y_dot_mm_s[0]], color="#00897b", marker="o", s=20)
    ax3.set_xlabel("eta (mm)")
    ax3.set_ylabel("eta_dot (mm/s)")
    ax3.set_title("State-space memory: same input can differ with residual initial state")
    ax3.legend(loc="upper right")
    ax3.grid(True, alpha=0.3)

    fig.suptitle(
        "Scout slosh ODE input-output simulation "
        f"(scenario={summary['scenario']}, omega_n={summary['omega_n_rad_s']:.3g}, "
        f"zeta={summary['zeta']:.3g}, dt={summary['dt_s']:.4g}s)",
        fontsize=12,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Plot saved: {out}")


def default_output_path(args):
    init = abs(args.init_eta_x_mm) + abs(args.init_eta_y_mm) + abs(args.init_eta_x_dot_mm_s) + abs(args.init_eta_y_dot_mm_s)
    suffix = "_residual" if init > 0.0 else ""
    return DEFAULT_OUT_DIR / f"slosh_ode_io_{args.scenario}_w{args.omega:.0f}_z{args.zeta:.3f}{suffix}.png"


def main():
    args = parse_args()
    t = np.arange(0.0, args.duration + 0.5 * args.dt, args.dt)
    ad, bd = discrete_matrices(args.omega, args.zeta, args.dt)
    ax, ay = build_inputs(t, args)
    initial_state = np.array(
        [
            args.init_eta_x_mm / 1000.0,
            args.init_eta_x_dot_mm_s / 1000.0,
            args.init_eta_y_mm / 1000.0,
            args.init_eta_y_dot_mm_s / 1000.0,
        ],
        dtype=float,
    )

    state = simulate(ax, ay, ad, bd, args.h_coeff, initial_state)
    summary = summarise(t, ax, ay, state, args)
    print_summary(summary, t, state)

    if args.csv:
        save_csv(args.csv, t, ax, ay, state)

    if not args.no_plot:
        plot_result(args.out or default_output_path(args), t, ax, ay, state, summary)


if __name__ == "__main__":
    main()
