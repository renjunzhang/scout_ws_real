"""NumPy stopping predicates and measurement windows, independent of ROS and OCP."""
import numpy as np

_CLEAR = 2e-6


def _series(times, X, U):
    t = np.asarray(times, dtype=float)
    x = np.asarray(X, dtype=float)
    u = np.asarray(U, dtype=float)
    if t.ndim != 1 or x.ndim != 2 or u.ndim != 2 or x.shape[0] != t.size or u.shape[0] != t.size:
        raise ValueError("times, X and U must have matching sample counts")
    if x.shape[1] != 28 or u.shape[1] not in (2, 3) or t.size == 0 or not np.isfinite(t).all() or not np.isfinite(x).all() or not np.isfinite(u).all():
        raise ValueError("invalid sample arrays")
    if np.any(np.diff(t) <= 0):
        raise ValueError("times must be strictly increasing")
    return t, x, u


def _yaw_error(a, b):
    return np.abs(np.arctan2(np.sin(a - b), np.cos(a - b)))


def stop_mask(X, U, task, clear_tolerance=_CLEAR):
    """Return per-sample predicate for the complete, physically stopped goal."""
    x = np.asarray(X, dtype=float); u = np.asarray(U, dtype=float)
    if not np.isfinite(x).all() or not np.isfinite(u).all() or not np.isfinite(clear_tolerance) or clear_tolerance <= 0:
        raise ValueError("X, U and clear_tolerance must be finite")
    if x.ndim != 2 or x.shape[1] != 28 or u.ndim != 2 or u.shape[0] != x.shape[0] or u.shape[1] not in (2, 3):
        raise ValueError("X must be (N,28), and U must have N rows")
    required = ("goal_pose", "goal_position_tolerance", "goal_yaw_tolerance",
                "stop_speed_tolerance", "stop_omega_tolerance")
    if any(k not in task for k in required):
        raise ValueError("task is missing stopping tolerances or goal_pose")
    goal = np.asarray(task["goal_pose"], dtype=float)
    if goal.shape != (3,) or not np.isfinite(goal).all(): raise ValueError("goal_pose must have length 3")
    if any(not np.isfinite(task[k]) or task[k] <= 0 for k in required[1:]):
        raise ValueError("stopping tolerances must be finite and positive")
    pos = np.linalg.norm(x[:, :2] - goal[:2], axis=1) <= float(task["goal_position_tolerance"])
    yaw = _yaw_error(x[:, 2], goal[2]) <= float(task["goal_yaw_tolerance"])
    motion = np.abs(x[:, 3]) <= float(task["stop_speed_tolerance"])
    motion &= np.abs(x[:, 5]) <= float(task["stop_omega_tolerance"])
    quiet = np.max(np.abs(x[:, 6:24]), axis=1) <= clear_tolerance
    # The third control is virtual path progress, not a physical command.
    quiet &= np.max(np.abs(u[:, :2]), axis=1) <= clear_tolerance
    return pos & yaw & motion & quiet


def find_stop_time(times, X, U, task, clear_tolerance=_CLEAR):
    """Return first timestamp whose sample and every later sample satisfy stop_mask."""
    t, x, u = _series(times, X, U)
    good = stop_mask(x, u, task, clear_tolerance)
    suffix = np.logical_and.accumulate(good[::-1])[::-1]
    indices = np.flatnonzero(suffix)
    return None if indices.size == 0 else float(t[indices[0]])


def stopping_windows(times, heights_m, t_stop, window_sec, max_sample_gap_sec=None):
    """Report transport peak/P95/RMS and the peak on [t_stop,t_stop+W].

    Boundary heights use linear interpolation; the transport supremum includes
    the left limit at t_stop. Missing endpoints produce None, not extrapolation.
    P95 and RMS use recorded samples strictly in [0,t_stop), without adding
    interpolated endpoints. Incomplete or empty transport windows yield None.
    Sampling coverage is not a proof of continuous-time constraint satisfaction.
    """
    t = np.asarray(times, dtype=float); h = np.asarray(heights_m, dtype=float)
    if t.ndim != 1 or h.ndim != 1 or t.size != h.size or t.size == 0 or not np.isfinite(t).all() or not np.isfinite(h).all() or np.any(np.diff(t) <= 0):
        raise ValueError("times and heights_m must be finite, nonempty, increasing 1-D arrays")
    if t_stop is None or not np.isfinite(t_stop) or t_stop < 0 or not np.isfinite(window_sec) or window_sec <= 0:
        raise ValueError("invalid t_stop/window_sec")
    if max_sample_gap_sec is not None and (not np.isfinite(max_sample_gap_sec) or max_sample_gap_sec <= 0):
        raise ValueError("invalid max_sample_gap_sec")
    if np.any(h < 0):
        raise ValueError("heights_m must be nonnegative")
    w = float(window_sec); ts = float(t_stop)
    end = ts + w
    def interp(at):
        return float(np.interp(at, t, h)) if t[0] <= at <= t[-1] else None
    def covered(lo, hi):
        if t[0] > lo or t[-1] < hi:
            return False
        if max_sample_gap_sec is None or hi == lo:
            return True
        # Use actual bracketing samples, including any gap crossing a boundary.
        left = max(0, int(np.searchsorted(t, lo, side="right")) - 1)
        right = min(len(t), int(np.searchsorted(t, hi, side="left")) + 1)
        return bool(np.all(np.diff(t[left:right]) <= max_sample_gap_sec + 1e-12))
    def peak(lo, hi):
        if lo == hi:
            return None
        if t[0] > lo or t[-1] < hi:
            return None
        vals = h[(t >= lo) & (t <= hi)]
        boundary = [v for v in (interp(lo), interp(hi)) if v is not None]
        vals = np.concatenate((vals, boundary))
        return None if vals.size == 0 else float(np.max(vals))
    tc = covered(0.0, ts)
    wc = covered(ts, end)
    transport = h[(t >= 0) & (t < ts)]
    statistics_available = tc and transport.size > 0
    return {"transport_peak_m": peak(0.0, ts),
            "transport_p95_m": float(np.percentile(transport, 95)) if statistics_available else None,
            "transport_rms_m": float(np.sqrt(np.mean(transport**2))) if statistics_available else None,
            "transport_samples": int(transport.size),
            "tail_peak_m": peak(ts, end),
            "transport_covered": tc, "tail_covered": wc,
            "covered": bool(tc and wc), "t_stop": ts, "window_sec": w}
