#!/usr/bin/env python3
"""Replay the current slosh model from a bag using recorded /slosh estimator inputs.

Important: this replays the current engineering model used in the repo.
`--use-nonlinear-model` only switches the height mapping coefficient from L to NL.
It does not turn the state update into the full nonlinear EOM from the paper.
"""

import argparse
import bisect
import csv
import json
import math
import statistics
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag
import yaml
from scipy.linalg import expm


MODAL_ROOTS = [1.8412, 5.3314, 8.5363, 11.7060, 14.8636]
DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[3]
    / "control"
    / "scout_local_planner"
    / "config"
    / "mpc_params.yaml"
)
DEFAULT_SLOSH_BAG_ROOT = Path("/data/a/slosh_bags")
DEFAULT_DEBUG_ROOT = Path("/data/a/realsense_validation_v2/debug")


@dataclass
class SloshReplayParams:
    container_radius_m: float
    liquid_height_m: float
    liquid_density: float
    damping_ratio: float
    mode_index: int
    offset_x_m: float
    offset_y_m: float
    gravity_mps2: float
    dt_s: float
    use_linear_model: bool
    use_parabola_term: bool
    replay_mode: str


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Replay the current slosh model from a rosbag using recorded "
            "/slosh/ax_est, /slosh/ay_est, /slosh/omega_est_used, and optional "
            "/slosh/alpha_est. Supports parameter overrides such as liquid_height "
            "and linear/nonlinear output mapping."
        )
    )
    parser.add_argument("--bag", required=True, help="Path to the rosbag containing /slosh/* topics.")
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG),
        help="Planner YAML used as the default parameter source. Defaults to scout_local_planner/config/mpc_params.yaml.",
    )
    parser.add_argument(
        "--liquid-csv",
        default="",
        help="Optional liquid_height_v2.csv to overlay RealSense main center output.",
    )
    parser.add_argument(
        "--center-column",
        default="height_center_rel_mm_bias_corrected_v2",
        help="RealSense center column to overlay when --liquid-csv is provided.",
    )
    parser.add_argument(
        "--liquid-filter",
        choices=["reportable", "valid", "all"],
        default="reportable",
        help="Which RealSense frames to use when overlaying liquid_height_v2.csv.",
    )
    parser.add_argument(
        "--out-dir",
        default="",
        help=(
            "Output directory. Defaults to "
            "/data/a/realsense_validation_v2/debug/<bag_batch>/slosh_replay/<bag_stem> "
            "when the bag lives under /data/a/slosh_bags; otherwise falls back to "
            "<bag_stem>_slosh_replay next to the bag."
        ),
    )
    parser.add_argument(
        "--initial-zero-align",
        action="store_true",
        help="Zero-align each compared height series using early samples before plotting/metrics.",
    )
    parser.add_argument(
        "--initial-align-window-sec",
        type=float,
        default=1.0,
        help="Early time window used when --initial-zero-align is enabled.",
    )
    parser.add_argument(
        "--initial-align-max-samples",
        type=int,
        default=30,
        help="Maximum early samples used when --initial-zero-align is enabled.",
    )
    parser.add_argument(
        "--init-mode",
        choices=["bag_state", "zero"],
        default="bag_state",
        help="Initial slosh state source for replay. bag_state uses the first /slosh/state sample, zero starts from [0,0,0,0].",
    )
    parser.add_argument("--container-radius-m", type=float, default=None, help="Override slosh/container_radius.")
    parser.add_argument("--liquid-height-m", type=float, default=None, help="Override slosh/liquid_height.")
    parser.add_argument("--liquid-density", type=float, default=None, help="Override slosh/liquid_density.")
    parser.add_argument("--damping-ratio", type=float, default=None, help="Override slosh/damping_ratio.")
    parser.add_argument("--mode-index", type=int, default=None, help="Override slosh/mode_index.")
    parser.add_argument("--offset-x-m", type=float, default=None, help="Override slosh/offset_x.")
    parser.add_argument("--offset-y-m", type=float, default=None, help="Override slosh/offset_y.")
    parser.add_argument("--gravity-mps2", type=float, default=None, help="Override gravity constant.")
    parser.add_argument("--dt-s", type=float, default=None, help="Override replay dt. Defaults to planner mpc/dt.")
    parser.add_argument(
        "--use-linear-model",
        dest="use_linear_model",
        action="store_true",
        default=None,
        help="Force L output mapping.",
    )
    parser.add_argument(
        "--use-nonlinear-model",
        dest="use_linear_model",
        action="store_false",
        help="Force NL output mapping.",
    )
    parser.add_argument(
        "--use-parabola-term",
        dest="use_parabola_term",
        action="store_true",
        default=None,
        help="Force parabola term on.",
    )
    parser.add_argument(
        "--disable-parabola-term",
        dest="use_parabola_term",
        action="store_false",
        help="Force parabola term off.",
    )
    parser.add_argument(
        "--replay-mode",
        choices=["linear_engineering", "paper_nl", "both"],
        default="linear_engineering",
        help="Replay mode: linear_engineering (Lp code) or paper_nl (Eq 11 full RK4) or both.",
    )
    return parser.parse_args()


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def infer_default_out_dir(bag_path: Path) -> Path:
    try:
        rel = bag_path.resolve().relative_to(DEFAULT_SLOSH_BAG_ROOT.resolve())
    except ValueError:
        return bag_path.with_name(f"{bag_path.stem}_slosh_replay")

    batch_name = rel.parts[0] if len(rel.parts) >= 2 else "misc"
    return DEFAULT_DEBUG_ROOT / batch_name / "slosh_replay" / bag_path.stem


def load_yaml(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"config is not a mapping: {path}")
    return data


def derive_params(args) -> Tuple[SloshReplayParams, Dict]:
    config_path = Path(args.config).expanduser().resolve()
    config_data = load_yaml(config_path)
    slosh_cfg = config_data.get("slosh", {}) or {}
    mpc_cfg = config_data.get("mpc", {}) or {}

    params = SloshReplayParams(
        container_radius_m=float(args.container_radius_m if args.container_radius_m is not None else slosh_cfg.get("container_radius", 0.014)),
        liquid_height_m=float(args.liquid_height_m if args.liquid_height_m is not None else slosh_cfg.get("liquid_height", 0.055)),
        liquid_density=float(args.liquid_density if args.liquid_density is not None else slosh_cfg.get("liquid_density", 1000.0)),
        damping_ratio=float(args.damping_ratio if args.damping_ratio is not None else slosh_cfg.get("damping_ratio", 0.12)),
        mode_index=int(args.mode_index if args.mode_index is not None else slosh_cfg.get("mode_index", 1)),
        offset_x_m=float(args.offset_x_m if args.offset_x_m is not None else slosh_cfg.get("offset_x", 0.0)),
        offset_y_m=float(args.offset_y_m if args.offset_y_m is not None else slosh_cfg.get("offset_y", 0.0)),
        gravity_mps2=float(args.gravity_mps2 if args.gravity_mps2 is not None else slosh_cfg.get("gravity", 9.81)),
        dt_s=float(args.dt_s if args.dt_s is not None else mpc_cfg.get("dt", 0.05)),
        use_linear_model=bool(args.use_linear_model if args.use_linear_model is not None else slosh_cfg.get("use_linear_model", True)),
        use_parabola_term=bool(args.use_parabola_term if args.use_parabola_term is not None else slosh_cfg.get("use_parabola_term", True)),
        replay_mode=args.replay_mode,
    )
    return params, config_data


def finite_float(raw_value) -> float:
    if raw_value is None:
        return math.nan
    try:
        value = float(raw_value)
    except (TypeError, ValueError):
        return math.nan
    return value if math.isfinite(value) else math.nan


def parse_optional_mm(raw_value) -> float:
    if raw_value is None:
        return math.nan
    text = str(raw_value).strip()
    if not text:
        return math.nan
    return finite_float(text)


def nearest_index(times: Sequence[float], ts: float) -> Optional[int]:
    if not times:
        return None
    idx = bisect.bisect_left(times, ts)
    if idx <= 0:
        return 0
    if idx >= len(times):
        return len(times) - 1
    left_dt = abs(ts - times[idx - 1])
    right_dt = abs(times[idx] - ts)
    return idx - 1 if left_dt <= right_dt else idx


def interpolate_scalar(sample_times: Sequence[float], series_times: Sequence[float], series_values: Sequence[float]) -> List[float]:
    if not series_times:
        return [math.nan] * len(sample_times)
    if len(series_times) == 1:
        only = float(series_values[0])
        return [only] * len(sample_times)
    out = []
    for ts in sample_times:
        idx = bisect.bisect_left(series_times, ts)
        if idx <= 0:
            out.append(float(series_values[0]))
            continue
        if idx >= len(series_times):
            out.append(float(series_values[-1]))
            continue
        t0 = float(series_times[idx - 1])
        t1 = float(series_times[idx])
        v0 = float(series_values[idx - 1])
        v1 = float(series_values[idx])
        if t1 <= t0:
            out.append(v1)
            continue
        ratio = (ts - t0) / (t1 - t0)
        out.append(v0 + ratio * (v1 - v0))
    return out


def compute_modal_root(mode_index: int) -> float:
    if mode_index < 1 or mode_index > len(MODAL_ROOTS):
        return MODAL_ROOTS[0]
    return MODAL_ROOTS[mode_index - 1]


def compute_modal_params(params: SloshReplayParams) -> Dict[str, float]:
    xi_1n = compute_modal_root(params.mode_index)
    m_f = params.liquid_density * math.pi * params.container_radius_m * params.container_radius_m * params.liquid_height_m
    arg = xi_1n * params.liquid_height_m / params.container_radius_m
    omega_sq = params.gravity_mps2 * (xi_1n / params.container_radius_m) * math.tanh(arg)
    omega_n = math.sqrt(max(omega_sq, 0.0))
    numerator = 2.0 * params.container_radius_m * math.tanh(arg)
    denominator = xi_1n * params.liquid_height_m * (xi_1n * xi_1n - 1.0)
    if abs(denominator) < 1e-9:
        raise RuntimeError("modal mass denominator is near zero")
    m_n = m_f * numerator / denominator
    k_n = m_n * omega_n * omega_n
    c_n = 2.0 * params.damping_ratio * math.sqrt(k_n * m_n)
    height_coeff_l = (4.0 * params.liquid_height_m * m_n) / (m_f * params.container_radius_m)
    height_coeff_nl = (xi_1n * xi_1n * params.liquid_height_m * m_n) / (m_f * params.container_radius_m)
    if params.use_linear_model:
        height_coeff = height_coeff_l
    else:
        height_coeff = height_coeff_nl
    return {
        "xi_1n": xi_1n,
        "m_F": m_f,
        "m_n": m_n,
        "omega_n": omega_n,
        "k_n": k_n,
        "c_n": c_n,
        "height_coeff": height_coeff,
        "height_coeff_nl": height_coeff_nl,
    }


def build_discrete_matrices(params: SloshReplayParams, modal: Dict[str, float]) -> Tuple[np.ndarray, np.ndarray]:
    omega_n = float(modal["omega_n"])
    two_zeta_omega = 2.0 * params.damping_ratio * omega_n
    omega_sq = omega_n * omega_n

    a_cont = np.zeros((4, 4), dtype=np.float64)
    a_cont[0, 1] = 1.0
    a_cont[1, 0] = -omega_sq
    a_cont[1, 1] = -two_zeta_omega
    a_cont[2, 3] = 1.0
    a_cont[3, 2] = -omega_sq
    a_cont[3, 3] = -two_zeta_omega

    b_cont = np.zeros((4, 2), dtype=np.float64)
    b_cont[1, 0] = -1.0
    b_cont[3, 1] = -1.0

    m_aug = np.zeros((6, 6), dtype=np.float64)
    m_aug[:4, :4] = a_cont * params.dt_s
    m_aug[:4, 4:] = b_cont * params.dt_s
    em = expm(m_aug)
    a_disc = em[:4, :4]
    b_disc = em[:4, 4:]
    if not np.isfinite(a_disc).all() or not np.isfinite(b_disc).all():
        raise RuntimeError("non-finite discrete matrices from expm")
    return a_disc, b_disc


def compute_height_mm(state: np.ndarray, omega_z: float, params: SloshReplayParams, modal: Dict[str, float]) -> Tuple[float, float, float]:
    eta_x = float(state[0])
    eta_y = float(state[2])
    eta_modal_m = float(modal["height_coeff"]) * math.hypot(eta_x, eta_y)
    eta_parabola_m = 0.0
    if params.use_parabola_term:
        eta_parabola_m = (params.container_radius_m * params.container_radius_m * omega_z * omega_z) / (4.0 * params.gravity_mps2)
    total_m = eta_modal_m + eta_parabola_m
    return eta_modal_m * 1000.0, eta_parabola_m * 1000.0, total_m * 1000.0


def load_bag_series(bag_path: Path) -> Dict[str, List]:
    topics = {
        "/slosh/height": [],
        "/slosh/height_pred_max": [],
        "/slosh/ax_est": [],
        "/slosh/ay_est": [],
        "/slosh/omega_est_used": [],
        "/slosh/alpha_est": [],
        "/slosh/state": [],
        "/cmd_vel": [],
        "/odom": [],
    }
    start_time = None
    with rosbag.Bag(str(bag_path)) as bag:
        start_time = bag.get_start_time()
        for topic, msg, t in bag.read_messages(topics=list(topics.keys())):
            ts = t.to_sec()
            if topic == "/slosh/height":
                topics[topic].append((ts, float(msg.data) * 1000.0))
            elif topic == "/slosh/height_pred_max":
                topics[topic].append((ts, float(msg.data) * 1000.0))
            elif topic in ("/slosh/ax_est", "/slosh/ay_est", "/slosh/omega_est_used", "/slosh/alpha_est"):
                topics[topic].append((ts, float(msg.data)))
            elif topic == "/slosh/state":
                values = list(msg.data)
                if len(values) >= 4:
                    topics[topic].append((ts, [float(values[0]), float(values[1]), float(values[2]), float(values[3])]))
            elif topic == "/cmd_vel":
                topics[topic].append((ts, (float(msg.linear.x), float(msg.angular.z))))
            elif topic == "/odom":
                topics[topic].append((ts, (float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z))))
    if start_time is None:
        raise RuntimeError(f"failed to read bag start time: {bag_path}")
    if not topics["/slosh/height"]:
        raise RuntimeError("bag is missing /slosh/height")
    if not topics["/slosh/ax_est"]:
        raise RuntimeError("bag is missing /slosh/ax_est")
    if not topics["/slosh/ay_est"]:
        raise RuntimeError("bag is missing /slosh/ay_est")
    return {"start_time": start_time, "topics": topics}


def maybe_fallback_series(raw_topics: Dict[str, List]) -> Dict[str, Tuple[List[float], List[float]]]:
    scalar = {}
    for name in ("/slosh/height", "/slosh/height_pred_max", "/slosh/ax_est", "/slosh/ay_est", "/slosh/omega_est_used", "/slosh/alpha_est"):
        entries = raw_topics.get(name, [])
        scalar[name] = ([ts for ts, _ in entries], [value for _, value in entries])

    if not scalar["/slosh/omega_est_used"][0]:
        cmd_entries = raw_topics.get("/cmd_vel", [])
        if cmd_entries:
            scalar["/slosh/omega_est_used"] = (
                [ts for ts, _ in cmd_entries],
                [value[1] for _, value in cmd_entries],
            )
        else:
            odom_entries = raw_topics.get("/odom", [])
            scalar["/slosh/omega_est_used"] = (
                [ts for ts, _ in odom_entries],
                [value[1] for _, value in odom_entries],
            )

    if not scalar["/slosh/alpha_est"][0]:
        omega_times, omega_values = scalar["/slosh/omega_est_used"]
        alpha_values = []
        prev_t = None
        prev_omega = None
        for ts, omega in zip(omega_times, omega_values):
            if prev_t is None:
                alpha_values.append(0.0)
            else:
                dt = ts - prev_t
                alpha_values.append((omega - prev_omega) / dt if dt > 1e-6 else 0.0)
            prev_t = ts
            prev_omega = omega
        scalar["/slosh/alpha_est"] = (omega_times, alpha_values)

    return scalar


def load_liquid_center_series(path: Path, center_column: str, liquid_filter: str) -> Tuple[List[float], List[float]]:
    times = []
    values = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames or []
        if "stamp" not in fields:
            raise RuntimeError("liquid csv is missing stamp")
        if center_column not in fields:
            raise RuntimeError(f"liquid csv is missing {center_column}")
        for row in reader:
            if liquid_filter == "reportable" and row.get("accept_for_peak_report_v2") != "1":
                continue
            if liquid_filter == "valid" and row.get("valid_v2") != "1":
                continue
            value = parse_optional_mm(row.get(center_column, ""))
            if math.isnan(value):
                continue
            times.append(float(row["stamp"]))
            values.append(value)
    return times, values


def initial_offset(rel_times: Sequence[float], values: Sequence[float], window_sec: float, max_samples: int) -> float:
    pairs = [(t, v) for t, v in zip(rel_times, values) if not math.isnan(v)]
    if not pairs:
        return 0.0
    selected = [v for t, v in pairs if t <= window_sec]
    if len(selected) < 3:
        selected = [v for _, v in pairs[: max(1, int(max_samples))]]
    else:
        selected = selected[: max(1, int(max_samples))]
    if not selected:
        return 0.0
    return float(statistics.median(selected))


def subtract_offset(values: Sequence[float], offset: float) -> List[float]:
    return [math.nan if math.isnan(v) else (v - offset) for v in values]


def paired_values(a: Sequence[float], b: Sequence[float]) -> List[Tuple[float, float]]:
    return [(x, y) for x, y in zip(a, b) if not math.isnan(x) and not math.isnan(y)]


def mae(a: Sequence[float], b: Sequence[float]) -> float:
    pairs = paired_values(a, b)
    if not pairs:
        return math.nan
    return sum(abs(x - y) for x, y in pairs) / len(pairs)


def rmse(a: Sequence[float], b: Sequence[float]) -> float:
    pairs = paired_values(a, b)
    if not pairs:
        return math.nan
    return math.sqrt(sum((x - y) ** 2 for x, y in pairs) / len(pairs))


def corr(a: Sequence[float], b: Sequence[float]) -> float:
    pairs = paired_values(a, b)
    if len(pairs) < 2:
        return math.nan
    xs = [x for x, _ in pairs]
    ys = [y for _, y in pairs]
    mean_x = statistics.mean(xs)
    mean_y = statistics.mean(ys)
    num = sum((x - mean_x) * (y - mean_y) for x, y in pairs)
    den_x = math.sqrt(sum((x - mean_x) ** 2 for x in xs))
    den_y = math.sqrt(sum((y - mean_y) ** 2 for y in ys))
    if den_x <= 0.0 or den_y <= 0.0:
        return math.nan
    return num / (den_x * den_y)


def metric_bundle(a: Sequence[float], b: Sequence[float]) -> Dict[str, float]:
    return {
        "mae_mm": mae(a, b),
        "rmse_mm": rmse(a, b),
        "corr": corr(a, b),
        "paired_count": len(paired_values(a, b)),
    }


def safe_stats(values: Sequence[float]) -> Dict[str, float]:
    clean = [float(v) for v in values if not math.isnan(v)]
    if not clean:
        return {"median": math.nan, "p90": math.nan, "max": math.nan}
    clean_sorted = sorted(clean)
    idx90 = int(round((len(clean_sorted) - 1) * 0.9))
    return {
        "median": float(statistics.median(clean)),
        "p90": float(clean_sorted[idx90]),
        "max": float(max(clean)),
    }



def integrate_rk4_paper_nl(state: np.ndarray, ax: float, ay: float, params: SloshReplayParams, modal: Dict[str, float], dt: float) -> np.ndarray:
    def derivs(s: np.ndarray) -> np.ndarray:
        x, y, dx, dy = s[0], s[1], s[2], s[3]
        alpha_n = 0.58
        w = 2.0
        r2 = x*x + y*y
        omega_sq = modal["omega_n"]**2
        two_zeta_omega = 2.0 * params.damping_ratio * modal["omega_n"]
        C_n_sq = (params.container_radius_m * omega_sq / params.gravity_mps2)**2
        
        M11 = 1.0 + C_n_sq * x * x
        M12 = C_n_sq * x * y
        M21 = M12
        M22 = 1.0 + C_n_sq * y * y
        det_M = 1.0 + C_n_sq * r2
        
        Fx = -ax / params.container_radius_m \
             - two_zeta_omega * (dx + C_n_sq * (x * x * dx + x * y * dy)) \
             - C_n_sq * x * (dx * dx + dy * dy) \
             - omega_sq * x * (1.0 + alpha_n * (r2 ** (w - 1.0)))
             
        Fy = -ay / params.container_radius_m \
             - two_zeta_omega * (dy + C_n_sq * (y * y * dy + x * y * dx)) \
             - C_n_sq * y * (dx * dx + dy * dy) \
             - omega_sq * y * (1.0 + alpha_n * (r2 ** (w - 1.0)))
             
        ddx = (M22 * Fx - M12 * Fy) / det_M
        ddy = (-M21 * Fx + M11 * Fy) / det_M
        
        return np.array([dx, dy, ddx, ddy], dtype=np.float64)

    max_dt = 0.01
    steps = max(1, int(dt / max_dt) + 1)
    step_dt = dt / steps
    
    curr_state = state.copy()
    for _ in range(steps):
        k1 = derivs(curr_state)
        k2 = derivs(curr_state + 0.5 * step_dt * k1)
        k3 = derivs(curr_state + 0.5 * step_dt * k2)
        k4 = derivs(curr_state + step_dt * k3)
        curr_state += (step_dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4)
        
        # simple limit to prevent overflow
        curr_state = np.clip(curr_state, -1000.0, 1000.0)
        
    return curr_state

def replay_series(raw: Dict, params: SloshReplayParams, modal: Dict[str, float], a_disc: np.ndarray, b_disc: np.ndarray, init_mode: str) -> Dict[str, List]:
    topics = raw["topics"]
    scalar = maybe_fallback_series(topics)
    out_times = scalar["/slosh/height"][0]
    bag_height_mm = scalar["/slosh/height"][1]
    pred_interp_mm = interpolate_scalar(out_times, *scalar["/slosh/height_pred_max"]) if scalar["/slosh/height_pred_max"][0] else [math.nan] * len(out_times)
    ax_interp = interpolate_scalar(out_times, *scalar["/slosh/ax_est"])
    ay_interp = interpolate_scalar(out_times, *scalar["/slosh/ay_est"])
    omega_interp = interpolate_scalar(out_times, *scalar["/slosh/omega_est_used"])
    alpha_interp = interpolate_scalar(out_times, *scalar["/slosh/alpha_est"])

    state_entries = topics["/slosh/state"]
    state_times = [ts for ts, _ in state_entries]
    state_values = [np.array(value, dtype=np.float64) for _, value in state_entries]

    if init_mode == "bag_state" and state_times:
        idx0 = nearest_index(state_times, out_times[0])
        replay_state = state_values[idx0].copy()
    else:
        replay_state = np.zeros(4, dtype=np.float64)

    bag_state_interp = []
    replay_state_rows = []
    modal_only_mm = []
    parabola_mm = []
    replay_height_mm = []

    nl_state_rows = []
    nl_modal_mm = []
    nl_total_mm = []

    if init_mode == "bag_state" and state_times:
        idx0 = nearest_index(state_times, out_times[0])
        init_bag = state_values[idx0]
        R = params.container_radius_m
        nl_state = np.array([init_bag[0]/R, init_bag[2]/R, init_bag[1]/R, init_bag[3]/R], dtype=np.float64)
    else:
        nl_state = np.zeros(4, dtype=np.float64)

    for idx, ts in enumerate(out_times):
        if state_times:
            state_idx = nearest_index(state_times, ts)
            bag_state = state_values[state_idx].copy()
            bag_state_interp.append(bag_state)
        else:
            bag_state_interp.append(np.full(4, np.nan, dtype=np.float64))

        if idx > 0:
            ax = float(ax_interp[idx])
            ay = float(ay_interp[idx])
            omega = float(omega_interp[idx]) if not math.isnan(omega_interp[idx]) else 0.0
            alpha = float(alpha_interp[idx]) if not math.isnan(alpha_interp[idx]) else 0.0
            
            a_cx = ax - alpha * params.offset_y_m - omega * omega * params.offset_x_m
            a_cy = ay + alpha * params.offset_x_m - omega * omega * params.offset_y_m
            
            replay_state = a_disc.dot(replay_state) + b_disc.dot(np.array([a_cx, a_cy], dtype=np.float64))
            
            dt = ts - out_times[idx - 1]
            if dt > 0.0:
                nl_state = integrate_rk4_paper_nl(nl_state, a_cx, a_cy, params, modal, dt)

        modal_mm, parab_mm, total_mm = compute_height_mm(
            replay_state,
            float(omega_interp[idx]) if not math.isnan(omega_interp[idx]) else 0.0,
            params,
            modal,
        )
        replay_state_rows.append(replay_state.copy())
        modal_only_mm.append(modal_mm)
        parabola_mm.append(parab_mm)
        replay_height_mm.append(total_mm)

        nl_state_rows.append(nl_state.copy())
        nl_r_m = math.hypot(nl_state[0], nl_state[1]) * params.container_radius_m
        nl_modal_h = modal["height_coeff_nl"] * nl_r_m
        nl_modal_mm.append(nl_modal_h * 1000.0)
        nl_total_mm.append(nl_modal_h * 1000.0 + parab_mm)

    return {
        "times": out_times,
        "bag_height_mm": bag_height_mm,
        "bag_pred_max_mm": pred_interp_mm,
        "ax_est_mps2": ax_interp,
        "ay_est_mps2": ay_interp,
        "omega_est_radps": omega_interp,
        "alpha_est_radps2": alpha_interp,
        "bag_state": bag_state_interp,
        "replay_state": replay_state_rows,
        "replay_modal_only_mm": modal_only_mm,
        "replay_parabola_mm": parabola_mm,
        "replay_height_mm": replay_height_mm,
        "paper_nl_modal_height": nl_modal_mm,
        "paper_nl_total_height": nl_total_mm,
    }


def write_aligned_csv(path: Path, rel_times: Sequence[float], replay: Dict[str, List], realsense_interp_mm: Optional[Sequence[float]], zero_align_enabled: bool, aligned: Dict[str, List]):
    ensure_dir(path.parent)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "stamp",
                "relative_time_s",
                "bag_slosh_height_mm",
                "bag_slosh_height_pred_max_mm",
                "recomputed_slosh_height_mm",
                "recomputed_modal_only_mm",
                "recomputed_parabola_mm",
                "paper_nl_modal_height",
                "paper_nl_total_height",
                "ax_est_mps2",
                "ay_est_mps2",
                "omega_est_radps",
                "alpha_est_radps2",
                "bag_eta_x",
                "bag_eta_x_dot",
                "bag_eta_y",
                "bag_eta_y_dot",
                "recomputed_eta_x",
                "recomputed_eta_x_dot",
                "recomputed_eta_y",
                "recomputed_eta_y_dot",
                "realsense_center_mm_interp",
                "bag_slosh_height_mm_aligned",
                "recomputed_slosh_height_mm_aligned",
                "realsense_center_mm_interp_aligned",
            ]
        )
        for idx, ts in enumerate(replay["times"]):
            bag_state = replay["bag_state"][idx]
            replay_state = replay["replay_state"][idx]
            row = [
                f"{ts:.9f}",
                f"{rel_times[idx]:.9f}",
                f"{replay['bag_height_mm'][idx]:.9f}",
                f"{replay['bag_pred_max_mm'][idx]:.9f}" if not math.isnan(replay["bag_pred_max_mm"][idx]) else "",
                f"{replay['replay_height_mm'][idx]:.9f}",
                f"{replay['replay_modal_only_mm'][idx]:.9f}",
                f"{replay['replay_parabola_mm'][idx]:.9f}",
                f"{replay['paper_nl_modal_height'][idx]:.9f}" if not math.isnan(replay["paper_nl_modal_height"][idx]) else "",
                f"{replay['paper_nl_total_height'][idx]:.9f}" if not math.isnan(replay["paper_nl_total_height"][idx]) else "",
                f"{replay['ax_est_mps2'][idx]:.9f}" if not math.isnan(replay["ax_est_mps2"][idx]) else "",
                f"{replay['ay_est_mps2'][idx]:.9f}" if not math.isnan(replay["ay_est_mps2"][idx]) else "",
                f"{replay['omega_est_radps'][idx]:.9f}" if not math.isnan(replay["omega_est_radps"][idx]) else "",
                f"{replay['alpha_est_radps2'][idx]:.9f}" if not math.isnan(replay["alpha_est_radps2"][idx]) else "",
                f"{bag_state[0]:.9f}" if np.isfinite(bag_state[0]) else "",
                f"{bag_state[1]:.9f}" if np.isfinite(bag_state[1]) else "",
                f"{bag_state[2]:.9f}" if np.isfinite(bag_state[2]) else "",
                f"{bag_state[3]:.9f}" if np.isfinite(bag_state[3]) else "",
                f"{replay_state[0]:.9f}",
                f"{replay_state[1]:.9f}",
                f"{replay_state[2]:.9f}",
                f"{replay_state[3]:.9f}",
                f"{realsense_interp_mm[idx]:.9f}" if realsense_interp_mm is not None and not math.isnan(realsense_interp_mm[idx]) else "",
                f"{aligned['bag'][idx]:.9f}" if zero_align_enabled and not math.isnan(aligned["bag"][idx]) else "",
                f"{aligned['replay'][idx]:.9f}" if zero_align_enabled and not math.isnan(aligned["replay"][idx]) else "",
                f"{aligned['realsense'][idx]:.9f}"
                if zero_align_enabled and aligned["realsense"] is not None and not math.isnan(aligned["realsense"][idx])
                else "",
            ]
            writer.writerow(row)


def plot_comparison(
    out_path: Path,
    rel_times: Sequence[float],
    bag_height_mm: Sequence[float],
    replay_height_mm: Sequence[float],
    paper_nl_total_height_mm: Sequence[float],
    pred_max_mm: Sequence[float],
    realsense_times_rel: Optional[Sequence[float]],
    realsense_values_mm: Optional[Sequence[float]],
    aligned: Dict[str, List],
    zero_align_enabled: bool,
    params: SloshReplayParams,
):
    ensure_dir(out_path.parent)
    fig, axes = plt.subplots(2, 1, figsize=(14, 9), sharex=True, constrained_layout=True)
    ax1, ax2 = axes

    bag_series = aligned["bag"] if zero_align_enabled else bag_height_mm
    replay_series = aligned["replay"] if zero_align_enabled else replay_height_mm
    paper_nl_series = aligned.get("paper_nl_total_height", []) if zero_align_enabled else paper_nl_total_height_mm
    pred_series = aligned["pred"] if zero_align_enabled else pred_max_mm
    rs_series = realsense_values_mm if not zero_align_enabled else None
    rs_series_aligned = [v - aligned.get("realsense_offset", 0.0) for v in realsense_values_mm] if zero_align_enabled else None

    ax1.plot(rel_times, bag_series, color="#1f77b4", linewidth=1.7, label="bag /slosh/height")
    ax1.plot(rel_times, replay_series, color="#d62728", linewidth=1.5, label="recomputed slosh height")
    if params.replay_mode in ("paper_nl", "both") and paper_nl_series:
        ax1.plot(rel_times, paper_nl_series, color="#ff7f0e", linewidth=1.5, label="paper_nl total height", linestyle="--")
    if any(not math.isnan(v) for v in pred_series):
        ax1.plot(rel_times, pred_series, color="#9467bd", linewidth=1.0, linestyle="--", label="bag /slosh/height_pred_max")
    if realsense_times_rel is not None and realsense_values_mm is not None:
        series = rs_series_aligned if zero_align_enabled else rs_series
        ax1.scatter(realsense_times_rel, series, s=10, color="#2ca02c", alpha=0.7, label="RealSense center")
    ax1.axhline(0.0, color="#808080", linewidth=1.0, linestyle=":")
    ax1.set_ylabel("height [mm]")
    ax1.set_title(
        "Slosh model replay from bag"
        f" | h={params.liquid_height_m:.3f} m"
        f" | {'L' if params.use_linear_model else 'NL'}"
        f" | parabola={'on' if params.use_parabola_term else 'off'}"
        + (" | initial-zero-aligned" if zero_align_enabled else "")
    )
    ax1.grid(True, alpha=0.25)
    ax1.legend(loc="upper right")

    replay_err = [
        math.nan if math.isnan(a) or math.isnan(b) else (b - a)
        for a, b in zip(bag_series, replay_series)
    ]
    ax2.plot(rel_times, replay_err, color="#d62728", linewidth=1.2, label="recomputed - bag")
    if params.replay_mode in ("paper_nl", "both") and paper_nl_series:
        nl_err = [math.nan if math.isnan(a) or math.isnan(b) else (b - a) for a, b in zip(bag_series, paper_nl_series)]
        ax2.plot(rel_times, nl_err, color="#ff7f0e", linewidth=1.2, label="paper_nl - bag", linestyle="--")
    if realsense_times_rel is not None:
        rs_interp = aligned["realsense"] if zero_align_enabled else aligned["realsense_raw_interp"]
        rs_err = [
            math.nan if rs_interp is None or math.isnan(a) or math.isnan(b) else (b - a)
            for a, b in zip(bag_series, rs_interp if rs_interp is not None else [math.nan] * len(bag_series))
        ]
        ax2.plot(rel_times, rs_err, color="#2ca02c", linewidth=1.0, label="RealSense center - bag")
    ax2.axhline(0.0, color="#808080", linewidth=1.0, linestyle=":")
    ax2.set_xlabel("relative time [s]")
    ax2.set_ylabel("error [mm]")
    ax2.grid(True, alpha=0.25)
    ax2.legend(loc="upper right")
    fig.savefig(str(out_path), dpi=180)
    plt.close(fig)


def main():
    args = parse_args()
    bag_path = Path(args.bag).expanduser().resolve()
    if not bag_path.is_file():
        raise SystemExit(f"[ERROR] bag not found: {bag_path}")

    out_dir = Path(args.out_dir).expanduser().resolve() if args.out_dir else infer_default_out_dir(bag_path)
    ensure_dir(out_dir)

    params, config_data = derive_params(args)
    modal = compute_modal_params(params)
    a_disc, b_disc = build_discrete_matrices(params, modal)
    raw = load_bag_series(bag_path)
    replay = replay_series(raw, params, modal, a_disc, b_disc, args.init_mode)

    start_time = float(raw["start_time"])
    rel_times = [ts - start_time for ts in replay["times"]]

    realsense_times = None
    realsense_values = None
    realsense_times_rel = None
    realsense_interp = None
    if args.liquid_csv:
        liquid_csv = Path(args.liquid_csv).expanduser().resolve()
        realsense_times, realsense_values = load_liquid_center_series(liquid_csv, args.center_column, args.liquid_filter)
        realsense_times_rel = [ts - start_time for ts in realsense_times]
        realsense_interp = interpolate_scalar(replay["times"], realsense_times, realsense_values)

    zero_align_enabled = bool(args.initial_zero_align)
    aligned = {
        "bag": list(replay["bag_height_mm"]),
        "replay": list(replay["replay_height_mm"]),
        "paper_nl_total_height": list(replay["paper_nl_total_height"]),
        "pred": list(replay["bag_pred_max_mm"]),
        "realsense": None,
        "realsense_raw_interp": realsense_interp,
    }
    offsets = {"bag": 0.0, "replay": 0.0, "pred": 0.0, "realsense": 0.0, "paper_nl_total_height": 0.0}
    if zero_align_enabled:
        offsets["bag"] = initial_offset(rel_times, replay["bag_height_mm"], args.initial_align_window_sec, args.initial_align_max_samples)
        offsets["replay"] = initial_offset(rel_times, replay["replay_height_mm"], args.initial_align_window_sec, args.initial_align_max_samples)
        offsets["paper_nl_total_height"] = initial_offset(rel_times, replay["paper_nl_total_height"], args.initial_align_window_sec, args.initial_align_max_samples)
        offsets["pred"] = initial_offset(rel_times, replay["bag_pred_max_mm"], args.initial_align_window_sec, args.initial_align_max_samples)
        aligned["bag"] = subtract_offset(replay["bag_height_mm"], offsets["bag"])
        aligned["replay"] = subtract_offset(replay["replay_height_mm"], offsets["replay"])
        aligned["paper_nl_total_height"] = subtract_offset(replay["paper_nl_total_height"], offsets["paper_nl_total_height"])
        aligned["pred"] = subtract_offset(replay["bag_pred_max_mm"], offsets["pred"])
        if realsense_interp is not None:
            offsets["realsense"] = initial_offset(rel_times, realsense_interp, args.initial_align_window_sec, args.initial_align_max_samples)
            aligned["realsense"] = subtract_offset(realsense_interp, offsets["realsense"])
            aligned["realsense_offset"] = offsets["realsense"]

    bag_metric_series = aligned["bag"] if zero_align_enabled else replay["bag_height_mm"]
    replay_metric_series = aligned["replay"] if zero_align_enabled else replay["replay_height_mm"]
    paper_nl_metric_series = aligned["paper_nl_total_height"] if zero_align_enabled else replay["paper_nl_total_height"]
    pred_metric_series = aligned["pred"] if zero_align_enabled else replay["bag_pred_max_mm"]
    metrics = {
        "bag_vs_recomputed": metric_bundle(bag_metric_series, replay_metric_series),
        "bag_height_stats_mm": safe_stats(aligned["bag"] if zero_align_enabled else replay["bag_height_mm"]),
        "recomputed_height_stats_mm": safe_stats(aligned["replay"] if zero_align_enabled else replay["replay_height_mm"]),
        "bag_pred_max_stats_mm": safe_stats(pred_metric_series),
    }
    if params.replay_mode in ("paper_nl", "both"):
        metrics.update(
            {
                "bag_vs_paper_nl": metric_bundle(bag_metric_series, paper_nl_metric_series),
                "paper_nl_total_height_stats_mm": safe_stats(paper_nl_metric_series),
                "paper_nl_modal_height_stats_mm": safe_stats(replay["paper_nl_modal_height"]),
            }
        )
    if realsense_interp is not None:
        rs_metric_series = aligned["realsense"] if zero_align_enabled else realsense_interp
        metrics.update(
            {
                "bag_vs_realsense": metric_bundle(bag_metric_series, rs_metric_series),
                "recomputed_vs_realsense": metric_bundle(replay_metric_series, rs_metric_series),
            }
        )
        if params.replay_mode in ("paper_nl", "both"):
            metrics["paper_nl_vs_realsense"] = metric_bundle(paper_nl_metric_series, rs_metric_series)

    png_path = out_dir / "slosh_recomputed_compare.png"
    csv_path = out_dir / "slosh_recomputed_aligned.csv"
    json_path = out_dir / "slosh_recomputed_summary.json"

    write_aligned_csv(csv_path, rel_times, replay, realsense_interp, zero_align_enabled, aligned)
    plot_comparison(
        png_path,
        rel_times,
        replay["bag_height_mm"],
        replay["replay_height_mm"],
        replay["paper_nl_total_height"],
        replay["bag_pred_max_mm"],
        realsense_times_rel,
        realsense_values,
        aligned,
        zero_align_enabled,
        params,
    )

    summary = {
        "bag": str(bag_path),
        "config": str(Path(args.config).expanduser().resolve()),
        "out_dir": str(out_dir),
        "init_mode": args.init_mode,
        "initial_zero_align": zero_align_enabled,
        "offsets_mm": offsets,
        "params": asdict(params),
        "modal": modal,
        "metrics": metrics,
        "liquid_csv": str(Path(args.liquid_csv).expanduser().resolve()) if args.liquid_csv else "",
        "liquid_filter": args.liquid_filter if args.liquid_csv else "",
        "center_column": args.center_column if args.liquid_csv else "",
        "raw_config_excerpt": {
            "mpc": config_data.get("mpc", {}),
            "slosh": config_data.get("slosh", {}),
        },
    }
    with json_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)

    print(f"[OK] out dir: {out_dir}")
    print(f"[OK] comparison png: {png_path}")
    print(f"[OK] aligned csv: {csv_path}")
    print(f"[OK] summary json: {json_path}")
    print(
        "[INFO] replay params: "
        f"h={params.liquid_height_m:.6f} m, "
        f"zeta={params.damping_ratio:.6f}, "
        f"mode={params.mode_index}, "
        f"model={'L' if params.use_linear_model else 'NL'}, "
        f"parabola={'on' if params.use_parabola_term else 'off'}, "
        f"replay_mode={params.replay_mode}"
    )
    print(
        "[INFO] bag vs recomputed: "
        f"MAE={metrics['bag_vs_recomputed']['mae_mm']:.6f} mm, "
        f"RMSE={metrics['bag_vs_recomputed']['rmse_mm']:.6f} mm, "
        f"Corr={metrics['bag_vs_recomputed']['corr']:.6f}, "
        f"n={metrics['bag_vs_recomputed']['paired_count']}"
    )
    if params.replay_mode in ("paper_nl", "both"):
        print(
            "[INFO] bag vs paper_nl: "
            f"MAE={metrics['bag_vs_paper_nl']['mae_mm']:.6f} mm, "
            f"RMSE={metrics['bag_vs_paper_nl']['rmse_mm']:.6f} mm, "
            f"Corr={metrics['bag_vs_paper_nl']['corr']:.6f}, "
            f"n={metrics['bag_vs_paper_nl']['paired_count']}"
        )
    if realsense_interp is not None:
        print(
            "[INFO] bag vs RealSense center: "
            f"MAE={metrics['bag_vs_realsense']['mae_mm']:.6f} mm, "
            f"RMSE={metrics['bag_vs_realsense']['rmse_mm']:.6f} mm, "
            f"Corr={metrics['bag_vs_realsense']['corr']:.6f}, "
            f"n={metrics['bag_vs_realsense']['paired_count']}"
        )
        print(
            "[INFO] recomputed vs RealSense center: "
            f"MAE={metrics['recomputed_vs_realsense']['mae_mm']:.6f} mm, "
            f"RMSE={metrics['recomputed_vs_realsense']['rmse_mm']:.6f} mm, "
            f"Corr={metrics['recomputed_vs_realsense']['corr']:.6f}, "
            f"n={metrics['recomputed_vs_realsense']['paired_count']}"
        )
        if params.replay_mode in ("paper_nl", "both"):
            print(
                "[INFO] paper_nl vs RealSense center: "
                f"MAE={metrics['paper_nl_vs_realsense']['mae_mm']:.6f} mm, "
                f"RMSE={metrics['paper_nl_vs_realsense']['rmse_mm']:.6f} mm, "
                f"Corr={metrics['paper_nl_vs_realsense']['corr']:.6f}, "
                f"n={metrics['paper_nl_vs_realsense']['paired_count']}"
            )


if __name__ == "__main__":
    main()
