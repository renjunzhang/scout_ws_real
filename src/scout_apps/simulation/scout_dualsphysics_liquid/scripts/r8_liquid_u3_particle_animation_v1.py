#!/usr/bin/env python3
"""Render real DualSPHysics PART frames and compact time-series QC.

The output is intentionally labelled SIM_ONLY_UNVALIDATED.  This renderer is
read-only with respect to the solver tree and refuses to overwrite an existing
visualization directory.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
import numpy as np

import r8_liquid_bi4_reader_v1 as bi4


PART_RE = re.compile(r"Part_([0-9]{4})\.bi4\Z")
EXPECTED_PARTICLES = 9_078
MOVING_END = 2_669
FLUID_BEGIN = 2_669
DOMAIN_XY = (-0.021, 0.021)
DOMAIN_Z = (-0.002, 0.070)


@dataclass(frozen=True)
class FrameData:
    index: int
    time_s: float
    step: int
    ids: np.ndarray
    positions: np.ndarray
    speeds: np.ndarray
    npok: int
    nout: int
    sha256: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--title", default="C1M real liquid simulation")
    parser.add_argument("--status-note", default="settling not yet validated")
    parser.add_argument("--expected-frames", type=int)
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--dpi", type=int, default=120)
    return parser.parse_args()


def _uint(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{label} is not an unsigned integer")
    return value


def _finite(value: Any, label: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} is not finite")
    return number


def read_frame(path: Path, expected_index: int) -> FrameData:
    source = bi4.read_regular_file(path, max_bytes=2 * 1024 * 1024)
    root = bi4.parse_jpartdata_bi4(source.data)
    if len(root.items) != 1:
        raise ValueError(f"{path.name}: expected one PART item")
    part = root.items[0]
    expected_arrays = {"Idp": 8, "Posd": 23, "Vel": 22, "Rhop": 11}
    for name, type_code in expected_arrays.items():
        array = part.arrays.get(name)
        if array is None or array.type_code != type_code:
            raise ValueError(f"{path.name}: missing or invalid {name} array")
    ids = np.asarray(part.arrays["Idp"].records(), dtype=np.int64)
    positions = np.asarray(part.arrays["Posd"].records(), dtype=np.float64)
    velocities = np.asarray(part.arrays["Vel"].records(), dtype=np.float64)
    if positions.shape != (len(ids), 3) or velocities.shape != positions.shape:
        raise ValueError(f"{path.name}: particle array shapes differ")
    if not np.all(np.isfinite(positions)) or not np.all(np.isfinite(velocities)):
        raise ValueError(f"{path.name}: non-finite particle arrays")
    if len(ids) != EXPECTED_PARTICLES or len(np.unique(ids)) != EXPECTED_PARTICLES:
        raise ValueError(f"{path.name}: particle count or IDs differ")
    if int(ids.min()) != 0 or int(ids.max()) != EXPECTED_PARTICLES - 1:
        raise ValueError(f"{path.name}: particle ID range differs")
    cpart = _uint(part.values.get("Cpart"), "Cpart")
    npok = _uint(part.values.get("Npok"), "Npok")
    nout = _uint(part.values.get("Nout"), "Nout")
    if cpart != expected_index or npok != EXPECTED_PARTICLES or nout != 0:
        raise ValueError(f"{path.name}: Cpart/Npok/Nout differs")
    return FrameData(
        index=cpart,
        time_s=_finite(part.values.get("TimeStep"), "TimeStep"),
        step=_uint(part.values.get("Step"), "Step"),
        ids=ids,
        positions=positions,
        speeds=np.linalg.norm(velocities, axis=1),
        npok=npok,
        nout=nout,
        sha256=source.sha256,
    )


def discover_frames(run_dir: Path, max_frames: int | None) -> list[Path]:
    data_dir = run_dir / "data"
    if not data_dir.is_dir():
        raise ValueError(f"missing solver data directory: {data_dir}")
    indexed: list[tuple[int, Path]] = []
    for path in data_dir.iterdir():
        match = PART_RE.fullmatch(path.name)
        if match and path.is_file():
            indexed.append((int(match.group(1)), path))
    indexed.sort()
    if max_frames is not None:
        indexed = indexed[:max_frames]
    if not indexed or [index for index, _ in indexed] != list(range(len(indexed))):
        raise ValueError("PART frames are absent or not contiguous from zero")
    return [path for _, path in indexed]


def surface_proxy(positions: np.ndarray) -> tuple[float, float, float, float]:
    angles = np.mod(np.arctan2(positions[:, 1], positions[:, 0]), 2 * np.pi)
    sectors = np.minimum((angles / (2 * np.pi) * 8).astype(int), 7)
    heights = [
        float(np.quantile(positions[sectors == sector, 2], 0.99))
        for sector in range(8)
        if int(np.count_nonzero(sectors == sector)) >= 4
    ]
    if len(heights) != 8:
        raise ValueError("surface proxy lacks all eight azimuth sectors")
    return (
        float(np.median(heights)),
        min(heights),
        max(heights),
        max(heights) - min(heights),
    )


def metrics_for(frame: FrameData) -> dict[str, Any]:
    fluid = frame.ids >= FLUID_BEGIN
    speeds = frame.speeds[fluid]
    surface_median, surface_min, surface_max, surface_spread = surface_proxy(
        frame.positions[fluid]
    )
    return {
        "part": frame.index,
        "time_s": frame.time_s,
        "step": frame.step,
        "particle_count": frame.npok,
        "excluded_count": frame.nout,
        "speed_rms_m_s": float(np.sqrt(np.mean(speeds * speeds))),
        "speed_p95_m_s": float(np.quantile(speeds, 0.95)),
        "speed_max_m_s": float(np.max(speeds)),
        "specific_kinetic_energy_j_kg": float(0.5 * np.mean(speeds * speeds)),
        "surface_proxy_median_m": surface_median,
        "surface_proxy_sector_min_m": surface_min,
        "surface_proxy_sector_max_m": surface_max,
        "surface_proxy_spread_m": surface_spread,
        "source_sha256": frame.sha256,
    }


def set_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 8.5,
            "axes.titlesize": 10,
            "axes.labelsize": 9,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
            "legend.fontsize": 7.5,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "figure.facecolor": "white",
            "savefig.facecolor": "white",
        }
    )


def padded_limits(values: list[float], *, floor_zero: bool = False) -> tuple[float, float]:
    low, high = min(values), max(values)
    if floor_zero:
        low = 0.0
    span = high - low
    pad = span * 0.08 if span > 0 else max(abs(high) * 0.08, 1e-6)
    return (max(0.0, low - pad) if floor_zero else low - pad, high + pad)


def render_frame(
    frame: FrameData,
    metrics: list[dict[str, Any]],
    upto: int,
    output: Path,
    *,
    title: str,
    status_note: str,
    speed_vmax: float,
    dpi: int,
) -> None:
    set_style()
    fig, axes = plt.subplots(2, 2, figsize=(12.0, 7.2), constrained_layout=True)
    layout_engine = fig.get_layout_engine()
    if layout_engine is not None:
        layout_engine.set(rect=(0.0, 0.055, 1.0, 0.88))
    side, top, speed_ax, surface_ax = axes.ravel()
    moving = frame.ids < MOVING_END
    fluid = frame.ids >= FLUID_BEGIN
    norm = Normalize(vmin=0.0, vmax=speed_vmax)
    side.scatter(
        frame.positions[moving, 0], frame.positions[moving, 2],
        s=2.0, c="#737373", alpha=0.55, linewidths=0, rasterized=True,
        label="moving boundary",
    )
    fluid_side = side.scatter(
        frame.positions[fluid, 0], frame.positions[fluid, 2],
        s=2.6, c=frame.speeds[fluid], cmap="viridis", norm=norm,
        alpha=0.82, linewidths=0, rasterized=True, label="fluid",
    )
    top.scatter(
        frame.positions[moving, 0], frame.positions[moving, 1],
        s=2.0, c="#737373", alpha=0.55, linewidths=0, rasterized=True,
    )
    top.scatter(
        frame.positions[fluid, 0], frame.positions[fluid, 1],
        s=2.6, c=frame.speeds[fluid], cmap="viridis", norm=norm,
        alpha=0.82, linewidths=0, rasterized=True,
    )
    side.set(xlim=DOMAIN_XY, ylim=DOMAIN_Z, xlabel="x (m)", ylabel="z (m)", title="Side view (x-z)")
    top.set(xlim=DOMAIN_XY, ylim=DOMAIN_XY, xlabel="x (m)", ylabel="y (m)", title="Top view (x-y)")
    side.set_aspect("equal", adjustable="box")
    top.set_aspect("equal", adjustable="box")
    colorbar = fig.colorbar(fluid_side, ax=(side, top), shrink=0.78, pad=0.02)
    colorbar.set_label("Speed |v| (m s$^{-1}$), capped at run p99")

    shown = metrics[: upto + 1]
    times = [float(row["time_s"]) for row in shown]
    full_times = [float(row["time_s"]) for row in metrics]
    palette = ("#0072B2", "#E69F00", "#009E73")
    for key, label, color, linestyle in (
        ("speed_rms_m_s", "RMS", palette[0], "-"),
        ("speed_p95_m_s", "P95", palette[1], "--"),
        ("speed_max_m_s", "Max", palette[2], ":"),
    ):
        speed_ax.plot(times, [row[key] for row in shown], label=label, color=color, linestyle=linestyle, linewidth=1.7)
    speed_ax.set(
        xlim=(0.0, max(full_times)),
        ylim=padded_limits([float(row["speed_max_m_s"]) for row in metrics], floor_zero=True),
        xlabel="Simulation time (s)",
        ylabel="Speed (m s$^{-1}$)",
        title="Fluid speed metrics",
    )
    speed_ax.legend(frameon=False, ncol=3, loc="upper left")
    surface_ax.fill_between(
        times,
        [row["surface_proxy_sector_min_m"] for row in shown],
        [row["surface_proxy_sector_max_m"] for row in shown],
        color="#56B4E9", alpha=0.25, label="sector range",
    )
    surface_ax.plot(
        times, [row["surface_proxy_median_m"] for row in shown],
        color="#0072B2", linewidth=1.8, label="sector median",
    )
    surface_values = [
        float(row[key])
        for row in metrics
        for key in ("surface_proxy_sector_min_m", "surface_proxy_sector_max_m")
    ]
    surface_ax.set(
        xlim=(0.0, max(full_times)),
        ylim=padded_limits(surface_values),
        xlabel="Simulation time (s)",
        ylabel="Surface q99 proxy (m)",
        title="Liquid-surface development proxy",
    )
    surface_ax.legend(frameon=False, loc="best")
    for axis in axes.ravel():
        axis.grid(True, color="#d9d9d9", linewidth=0.5, alpha=0.65)
    fig.suptitle(
        f"{title} — t={frame.time_s:.3f} s, frame {frame.index:04d}",
        y=0.985, fontsize=13, weight="bold",
    )
    fig.text(
        0.5, 0.012,
        f"SIM_ONLY_UNVALIDATED · {status_note}\n"
        f"grey=boundary · viridis=fluid speed · N={frame.npok:,} · excluded={frame.nout}",
        ha="center", va="bottom", color="#A33A00", fontsize=8.0, weight="bold",
    )
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def render_qc(metrics: list[dict[str, Any]], output: Path, *, title: str, status_note: str, dpi: int) -> None:
    set_style()
    fig, axes = plt.subplots(3, 1, figsize=(7.2, 7.2), sharex=True, constrained_layout=True)
    layout_engine = fig.get_layout_engine()
    if layout_engine is not None:
        layout_engine.set(rect=(0.0, 0.075, 1.0, 0.855))
    times = [float(row["time_s"]) for row in metrics]
    colors = ("#0072B2", "#E69F00", "#009E73")
    for key, label, color, linestyle in (
        ("speed_rms_m_s", "RMS", colors[0], "-"),
        ("speed_p95_m_s", "P95", colors[1], "--"),
        ("speed_max_m_s", "Max", colors[2], ":"),
    ):
        axes[0].plot(times, [row[key] for row in metrics], label=label, color=color, linestyle=linestyle, marker="o", markersize=2.6)
    axes[0].set(ylabel="Speed (m s$^{-1}$)", title="a  Fluid speed")
    axes[0].legend(frameon=False, ncol=3)
    axes[1].plot(
        times, [row["specific_kinetic_energy_j_kg"] for row in metrics],
        color="#CC79A7", marker="s", markersize=2.6, linewidth=1.6,
    )
    axes[1].set(ylabel="Specific KE (J kg$^{-1}$)", title="b  Fluid specific kinetic energy")
    axes[2].fill_between(
        times,
        [row["surface_proxy_sector_min_m"] for row in metrics],
        [row["surface_proxy_sector_max_m"] for row in metrics],
        color="#56B4E9", alpha=0.28, label="sector range",
    )
    axes[2].plot(
        times, [row["surface_proxy_median_m"] for row in metrics],
        color="#0072B2", marker="o", markersize=2.6, linewidth=1.6,
        label="sector median",
    )
    axes[2].set(xlabel="Simulation time (s)", ylabel="Surface q99 proxy (m)", title="c  Liquid-surface development proxy")
    axes[2].legend(frameon=False)
    for axis in axes:
        axis.grid(True, color="#d9d9d9", linewidth=0.5, alpha=0.65)
    fig.suptitle(
        f"{title} — quick settle/QC trends", y=0.985,
        fontsize=12, weight="bold",
    )
    fig.text(
        0.5, 0.003,
        f"SIM_ONLY_UNVALIDATED · {status_note}\n"
        f"N={EXPECTED_PARTICLES:,} retained · excluded=0 in every rendered frame",
        ha="center", va="bottom", color="#A33A00", fontsize=7.6, weight="bold",
    )
    fig.savefig(output, dpi=dpi)
    plt.close(fig)


def write_metrics(path: Path, metrics: list[dict[str, Any]]) -> None:
    with path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(metrics[0]))
        writer.writeheader()
        writer.writerows(metrics)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    args = parse_args()
    if args.output_dir.exists():
        raise ValueError(f"refusing to overwrite: {args.output_dir}")
    frame_paths = discover_frames(args.run_dir, args.max_frames)
    if args.expected_frames is not None and len(frame_paths) != args.expected_frames:
        raise ValueError(f"expected {args.expected_frames} frames, found {len(frame_paths)}")
    frames = [read_frame(path, index) for index, path in enumerate(frame_paths)]
    if any(right.time_s <= left.time_s for left, right in zip(frames, frames[1:])):
        raise ValueError("frame times are not strictly increasing")
    metrics = [metrics_for(frame) for frame in frames]
    speed_vmax = max(float(np.quantile(frame.speeds[frame.ids >= FLUID_BEGIN], 0.99)) for frame in frames)
    speed_vmax = max(speed_vmax, 1e-6)

    frames_dir = args.output_dir / "frames"
    data_dir = args.output_dir / "data"
    figures_dir = args.output_dir / "figures"
    reports_dir = args.output_dir / "reports"
    for path in (frames_dir, data_dir, figures_dir, reports_dir):
        path.mkdir(parents=True, exist_ok=False)
    metrics_path = data_dir / "particle_animation_metrics.csv"
    write_metrics(metrics_path, metrics)
    qc_path = figures_dir / "quick_settle_qc.png"
    render_qc(metrics, qc_path, title=args.title, status_note=args.status_note, dpi=max(args.dpi, 300))
    for index, frame in enumerate(frames):
        render_frame(
            frame,
            metrics,
            index,
            frames_dir / f"frame_{index:04d}.png",
            title=args.title,
            status_note=args.status_note,
            speed_vmax=speed_vmax,
            dpi=args.dpi,
        )
    report = {
        "status": "SIM_ONLY_UNVALIDATED",
        "source_run": str(args.run_dir),
        "frame_count": len(frames),
        "first_time_s": frames[0].time_s,
        "last_time_s": frames[-1].time_s,
        "particle_count_all_frames": EXPECTED_PARTICLES,
        "excluded_count_all_frames": 0,
        "speed_color_vmax_m_s": speed_vmax,
        "surface_metric": "median_of_eight_azimuth_sector_fluid_z_q99_development_proxy",
        "settling_validated": False,
        "output_frames": str(frames_dir),
        "metrics_csv": {
            "path": str(metrics_path),
            "sha256": sha256_file(metrics_path),
        },
        "quick_qc": {"path": str(qc_path), "sha256": sha256_file(qc_path)},
    }
    report_path = reports_dir / "animation_report.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
