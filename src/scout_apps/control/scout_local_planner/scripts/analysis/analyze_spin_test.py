#!/usr/bin/env python3
"""analyze_spin_test.py

Compare two in-place spin bags recorded by record_spin_test.sh (before vs after
the wheel-contact fix). Reports pose drift (Gazebo ground truth) and IMU
angular-velocity spectral energy in the 5-20 Hz band.

Targets (from 2026-05-19_仿真skid-steer接触模型修复方案 fix A):
  - pose drift peak  : drop >= 50%
  - imu wz 5-20 Hz energy: drop >= 70%

Usage:
  analyze_spin_test.py --before BAG1 --after BAG2 [--model-name scout/]
"""

import argparse
import json
import sys
from pathlib import Path
from typing import List, Tuple

import numpy as np
import rosbag
from scipy.signal import welch


GAZEBO_TOPIC = '/gazebo/model_states'
IMU_TOPIC = '/imu/data_raw'
CMD_TOPIC = '/cmd_vel'

BAND_LO_HZ = 5.0
BAND_HI_HZ = 20.0
ANAL_LEAD_S = 0.5  # trim transient after cmd onset


def load_meta(bag_path: str) -> dict:
    meta_path = Path(bag_path).with_suffix('.meta.json')
    if meta_path.exists():
        return json.loads(meta_path.read_text())
    return {}


def find_spin_window(bag_path: str) -> Tuple[float, float]:
    t_start = None
    t_end = None
    with rosbag.Bag(bag_path) as bag:
        for _, msg, t in bag.read_messages(topics=[CMD_TOPIC]):
            if abs(msg.angular.z) > 1e-3:
                t_sec = t.to_sec()
                if t_start is None:
                    t_start = t_sec
                t_end = t_sec
    if t_start is None:
        raise RuntimeError(f"no /cmd_vel angular.z!=0 in {bag_path}")
    return t_start, t_end


def pose_drift(bag_path: str, model_name: str,
               t_lo: float, t_hi: float) -> Tuple[float, float]:
    samples: List[Tuple[float, float, float]] = []
    with rosbag.Bag(bag_path) as bag:
        for _, msg, t in bag.read_messages(topics=[GAZEBO_TOPIC]):
            if model_name not in msg.name:
                continue
            idx = msg.name.index(model_name)
            p = msg.pose[idx].position
            samples.append((t.to_sec(), p.x, p.y))
    if not samples:
        raise RuntimeError(f"no '{model_name}' in {GAZEBO_TOPIC} of {bag_path}")
    arr = np.asarray(samples)
    mask = (arr[:, 0] >= t_lo) & (arr[:, 0] <= t_hi)
    arr = arr[mask]
    if arr.shape[0] < 2:
        raise RuntimeError(f"too few pose samples in window for {bag_path}")
    x0, y0 = arr[0, 1], arr[0, 2]
    dxy = np.hypot(arr[:, 1] - x0, arr[:, 2] - y0)
    return float(np.max(dxy)), float(np.sqrt(np.mean(dxy ** 2)))


def imu_band_energy(bag_path: str,
                    t_lo: float, t_hi: float) -> Tuple[float, float, float]:
    times: List[float] = []
    wz: List[float] = []
    with rosbag.Bag(bag_path) as bag:
        for _, msg, t in bag.read_messages(topics=[IMU_TOPIC]):
            stamp = msg.header.stamp.to_sec()
            t_sec = stamp if stamp > 0 else t.to_sec()
            if t_lo <= t_sec <= t_hi:
                times.append(t_sec)
                wz.append(msg.angular_velocity.z)
    if len(wz) < 32:
        raise RuntimeError(f"too few {IMU_TOPIC} samples ({len(wz)}) in {bag_path}")
    t_arr = np.asarray(times)
    wz_arr = np.asarray(wz)
    fs = 1.0 / float(np.median(np.diff(t_arr)))
    # Remove commanded mean so PSD reflects deviations (stick-slip / jitter)
    wz_dev = wz_arr - np.mean(wz_arr)
    nperseg = min(256, len(wz_dev))
    f, psd = welch(wz_dev, fs=fs, nperseg=nperseg)
    band = (f >= BAND_LO_HZ) & (f <= BAND_HI_HZ)
    band_e = float(np.trapz(psd[band], f[band]))
    rms_dev = float(np.sqrt(np.mean(wz_dev ** 2)))
    return band_e, rms_dev, fs


def drop_pct(before: float, after: float) -> float:
    if before <= 0:
        return float('nan')
    return (before - after) / before * 100.0


def analyze_one(bag_path: str, model_name: str) -> dict:
    t0, t1 = find_spin_window(bag_path)
    t_lo = t0 + ANAL_LEAD_S
    peak, rms_xy = pose_drift(bag_path, model_name, t_lo, t1)
    band_e, wz_rms, fs = imu_band_energy(bag_path, t_lo, t1)
    return {
        'bag': bag_path,
        'meta': load_meta(bag_path),
        'duration_s': t1 - t0,
        'pose_peak_m': peak,
        'pose_rms_m': rms_xy,
        'imu_band_energy': band_e,
        'imu_wz_rms_rad_s': wz_rms,
        'imu_fs_hz': fs,
    }


def report(before: dict, after: dict) -> None:
    print(f"== before: {before['bag']}")
    print(f"== after : {after['bag']}")
    if before['meta']:
        print(f"   before meta: omega={before['meta'].get('omega_rad_s')} "
              f"dur={before['meta'].get('duration_s')}s "
              f"git={before['meta'].get('git_hash')}")
    if after['meta']:
        print(f"   after  meta: omega={after['meta'].get('omega_rad_s')} "
              f"dur={after['meta'].get('duration_s')}s "
              f"git={after['meta'].get('git_hash')}")
    print()
    fmt = "{:<26} {:>14} {:>14} {:>10} {:>12}"
    print(fmt.format('metric', 'before', 'after', 'drop %', 'target'))
    print('-' * 80)
    rows = [
        ('spin duration [s]',       'duration_s',         '{:.2f}',  None),
        ('pose drift peak [m]',     'pose_peak_m',        '{:.4f}',  '>=50%'),
        ('pose drift RMS  [m]',     'pose_rms_m',         '{:.4f}',  None),
        ('imu wz 5-20Hz energy',    'imu_band_energy',    '{:.4e}',  '>=70%'),
        ('imu wz dev RMS [rad/s]',  'imu_wz_rms_rad_s',   '{:.4f}',  None),
        ('imu fs [Hz]',             'imu_fs_hz',          '{:.2f}',  None),
    ]
    for name, key, val_fmt, target in rows:
        bv = before[key]
        av = after[key]
        if target is None:
            drop_s = ''
        else:
            d = drop_pct(bv, av)
            drop_s = 'n/a' if np.isnan(d) else f"{d:+.1f}"
        print(fmt.format(name, val_fmt.format(bv), val_fmt.format(av),
                         drop_s, target or ''))
    print()
    print(f"analysis band : {BAND_LO_HZ:.1f}-{BAND_HI_HZ:.1f} Hz")
    print(f"analysis window: [{ANAL_LEAD_S:.2f}s after first cmd_vel.angular.z != 0, "
          f"last command]")
    # pass/fail summary on the two declared targets
    pose_drop = drop_pct(before['pose_peak_m'], after['pose_peak_m'])
    imu_drop = drop_pct(before['imu_band_energy'], after['imu_band_energy'])
    pose_ok = (not np.isnan(pose_drop)) and pose_drop >= 50.0
    imu_ok = (not np.isnan(imu_drop)) and imu_drop >= 70.0
    print()
    print(f"pose drift peak: {pose_drop:+.1f}%  [{'PASS' if pose_ok else 'FAIL'} vs >=50%]")
    print(f"imu band energy: {imu_drop:+.1f}%  [{'PASS' if imu_ok else 'FAIL'} vs >=70%]")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--before', required=True, help='bag recorded before fix A')
    ap.add_argument('--after',  required=True, help='bag recorded after fix A')
    ap.add_argument('--model-name', default='scout/',
                    help="Gazebo model name to track (default: 'scout/')")
    args = ap.parse_args()
    before = analyze_one(args.before, args.model_name)
    after = analyze_one(args.after, args.model_name)
    report(before, after)
    return 0


if __name__ == '__main__':
    sys.exit(main())
