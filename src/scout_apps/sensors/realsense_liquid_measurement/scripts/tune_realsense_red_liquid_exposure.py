#!/usr/bin/env python3
"""
Tune RealSense RGB settings for red-liquid visual measurement.

The RealSense color camera has a fixed aperture on the common D4xx devices used in
this project, so this script sweeps the controllable exposure-triangle knobs:

  exposure, gain, white balance

For each candidate setting it captures live ROS images, scores ROI sharpness,
over/under exposure, and red-liquid separability, then writes a ranked CSV and
recommended roslaunch/dynparam settings.
"""

import argparse
import csv
import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml

try:
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    from dynamic_reconfigure.client import Client as DynClient
except ImportError as exc:  # pragma: no cover - ROS-only script
    raise SystemExit(
        "This script must run in a sourced ROS environment with cv_bridge and "
        "dynamic_reconfigure available."
    ) from exc


Setting = Tuple[int, int, int]  # exposure, gain, white_balance


def parse_csv_ints(text: str) -> List[int]:
    vals: List[int] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        vals.append(int(part))
    if not vals:
        raise argparse.ArgumentTypeError("expected at least one integer")
    return vals


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Sweep RealSense RGB exposure/gain/white-balance for red-liquid segmentation."
    )
    p.add_argument("--image-topic", default="/camera/color/image_raw")
    p.add_argument("--dynparam-ns", default="/camera/rgb_camera",
                   help="dynamic_reconfigure namespace for the color sensor.")
    p.add_argument("--calibration", default="",
                   help="Optional red_liquid_calibrate.py YAML. Uses ROI/tube if present.")
    p.add_argument("--roi", default="",
                   help="Manual ROI as x,y,w,h if no calibration YAML is available.")
    p.add_argument("--out-dir", required=True)
    p.add_argument("--exposures", type=parse_csv_ints,
                   default=parse_csv_ints("3000,5000,7000,9000,12000,16000"),
                   help="Comma-separated exposure candidates. Unit follows RealSense driver.")
    p.add_argument("--gains", type=parse_csv_ints,
                   default=parse_csv_ints("16,32,64"),
                   help="Comma-separated gain candidates.")
    p.add_argument("--white-balances", type=parse_csv_ints,
                   default=parse_csv_ints("3800,4200,4600,5000"),
                   help="Comma-separated white-balance candidates in Kelvin-like driver units.")
    p.add_argument("--frames-per-setting", type=int, default=20)
    p.add_argument("--settle-sec", type=float, default=0.8)
    p.add_argument("--timeout-sec", type=float, default=5.0)
    p.add_argument("--topk-previews", type=int, default=6)
    p.add_argument("--keep-best", action="store_true",
                   help="Leave the camera at the best setting. Default restores auto modes.")
    p.add_argument("--no-restore", action="store_true",
                   help="Do not restore the original dynamic_reconfigure values.")
    p.add_argument("--hue1-low", type=int, default=0)
    p.add_argument("--hue1-high", type=int, default=12)
    p.add_argument("--hue2-low", type=int, default=168)
    p.add_argument("--hue2-high", type=int, default=179)
    p.add_argument("--sat-min", type=int, default=90)
    p.add_argument("--val-min", type=int, default=60)
    return p.parse_args()


def load_roi(args: argparse.Namespace) -> Optional[Tuple[int, int, int, int]]:
    if args.calibration:
        path = Path(args.calibration).expanduser()
        with path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        roi = data.get("roi", {}) if isinstance(data, dict) else {}
        vals = [roi.get(k) for k in ("x", "y", "w", "h")]
        if all(v is not None for v in vals):
            return tuple(int(v) for v in vals)  # type: ignore[return-value]
    if args.roi:
        vals = [int(v.strip()) for v in args.roi.split(",")]
        if len(vals) != 4:
            raise ValueError("--roi must be x,y,w,h")
        return tuple(vals)  # type: ignore[return-value]
    return None


class ImageBuffer:
    def __init__(self, topic: str) -> None:
        self.bridge = CvBridge()
        self.latest: Optional[np.ndarray] = None
        self.stamp = 0.0
        self.sub = rospy.Subscriber(topic, Image, self._cb, queue_size=1)

    def _cb(self, msg: Image) -> None:
        self.latest = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.stamp = msg.header.stamp.to_sec() if msg.header.stamp else time.time()

    def collect(self, n: int, timeout: float) -> List[np.ndarray]:
        frames: List[np.ndarray] = []
        start = time.time()
        last_stamp = -1.0
        while len(frames) < n and (time.time() - start) < timeout and not rospy.is_shutdown():
            if self.latest is not None and self.stamp != last_stamp:
                frames.append(self.latest.copy())
                last_stamp = self.stamp
            rospy.sleep(0.02)
        return frames


def clip_roi(img: np.ndarray, roi: Optional[Tuple[int, int, int, int]]) -> np.ndarray:
    if roi is None:
        return img
    x, y, w, h = roi
    h_img, w_img = img.shape[:2]
    x1 = max(0, min(w_img - 1, x))
    y1 = max(0, min(h_img - 1, y))
    x2 = max(x1 + 1, min(w_img, x + w))
    y2 = max(y1 + 1, min(h_img, y + h))
    return img[y1:y2, x1:x2]


def red_mask(hsv: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    h, s, v = cv2.split(hsv)
    hue = ((h >= args.hue1_low) & (h <= args.hue1_high)) | (
        (h >= args.hue2_low) & (h <= args.hue2_high)
    )
    return (hue & (s >= args.sat_min) & (v >= args.val_min)).astype(np.uint8)


def frame_metrics(img: np.ndarray, roi: Optional[Tuple[int, int, int, int]], args: argparse.Namespace) -> Dict[str, float]:
    crop = clip_roi(img, roi)
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    mask = red_mask(hsv, args).astype(bool)

    sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    over_frac = float(np.mean(v >= 245))
    under_frac = float(np.mean(v <= 20))
    red_frac = float(np.mean(mask))

    if np.any(mask):
        red_s = float(np.median(s[mask]))
        red_v = float(np.median(v[mask]))
        bg_s = float(np.median(s[~mask])) if np.any(~mask) else red_s
        bg_v = float(np.median(v[~mask])) if np.any(~mask) else red_v
        red_contrast = max(0.0, red_s - bg_s) / 255.0 + max(0.0, red_v - bg_v) / 255.0
    else:
        red_s = red_v = red_contrast = 0.0

    red_frac_score = math.exp(-((red_frac - 0.18) / 0.22) ** 2)
    red_quality = 0.45 * (red_s / 255.0) + 0.25 * (red_v / 255.0) + 0.20 * red_contrast + 0.10 * red_frac_score
    exposure_quality = 1.0 - min(1.0, 3.0 * over_frac + 2.0 * under_frac)

    return {
        "sharpness": sharpness,
        "over_frac": over_frac,
        "under_frac": under_frac,
        "red_frac": red_frac,
        "red_s_median": red_s,
        "red_v_median": red_v,
        "red_contrast": red_contrast,
        "red_quality": red_quality,
        "exposure_quality": exposure_quality,
    }


def mean_metrics(frames: Sequence[np.ndarray], roi: Optional[Tuple[int, int, int, int]], args: argparse.Namespace) -> Dict[str, float]:
    if not frames:
        return {"valid_frames": 0.0}
    rows = [frame_metrics(f, roi, args) for f in frames]
    out = {"valid_frames": float(len(frames))}
    for key in rows[0]:
        out[key] = float(np.mean([r[key] for r in rows]))
    return out


def try_update(client: DynClient, params: Dict[str, object]) -> None:
    try:
        client.update_configuration(params)
    except Exception as exc:  # pragma: no cover - hardware-specific
        raise RuntimeError(f"dynamic_reconfigure update failed for {params}: {exc}") from exc


def save_preview(img: np.ndarray, roi: Optional[Tuple[int, int, int, int]], path: Path, label: str) -> None:
    vis = img.copy()
    if roi is not None:
        x, y, w, h = roi
        cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 255), 2)
    cv2.putText(vis, label, (30, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2, cv2.LINE_AA)
    cv2.imwrite(str(path), vis)


def normalize_scores(rows: List[Dict[str, object]]) -> None:
    sharp_vals = [float(r["sharpness"]) for r in rows if float(r.get("valid_frames", 0.0)) > 0]
    if not sharp_vals:
        return
    lo, hi = float(np.percentile(sharp_vals, 5)), float(np.percentile(sharp_vals, 95))
    span = max(1e-6, hi - lo)
    for r in rows:
        sharp_norm = max(0.0, min(1.0, (float(r["sharpness"]) - lo) / span))
        score = (
            0.40 * sharp_norm
            + 0.35 * float(r["red_quality"])
            + 0.25 * float(r["exposure_quality"])
        )
        if float(r["over_frac"]) > 0.01:
            score -= min(0.25, 4.0 * float(r["over_frac"]))
        if float(r["red_frac"]) < 0.005:
            score -= 0.20
        r["sharpness_norm"] = sharp_norm
        r["score"] = score


def main() -> None:
    args = parse_args()
    out_dir = Path(args.out_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    roi = load_roi(args)

    rospy.init_node("tune_realsense_red_liquid_exposure", anonymous=True)
    img_buf = ImageBuffer(args.image_topic)
    client = DynClient(args.dynparam_ns, timeout=5.0)
    original = dict(client.get_configuration())

    print(f"[INFO] image_topic={args.image_topic}")
    print(f"[INFO] dynparam_ns={args.dynparam_ns}")
    print(f"[INFO] roi={roi if roi is not None else 'full image'}")
    print("[INFO] RealSense color aperture is fixed; sweeping exposure/gain/white_balance.")

    rows: List[Dict[str, object]] = []
    last_frames: Dict[Setting, np.ndarray] = {}
    settings: List[Setting] = [
        (e, g, wb)
        for e in args.exposures
        for g in args.gains
        for wb in args.white_balances
    ]

    sweep_ok = False
    try:
        try_update(client, {"enable_auto_exposure": False, "enable_auto_white_balance": False})
        for idx, (exposure, gain, wb) in enumerate(settings, start=1):
            print(f"[{idx:03d}/{len(settings):03d}] exposure={exposure} gain={gain} white_balance={wb}")
            try_update(client, {"exposure": exposure, "gain": gain, "white_balance": wb})
            rospy.sleep(args.settle_sec)
            frames = img_buf.collect(args.frames_per_setting, args.timeout_sec)
            metrics = mean_metrics(frames, roi, args)
            row: Dict[str, object] = {
                "exposure": exposure,
                "gain": gain,
                "white_balance": wb,
                **metrics,
            }
            rows.append(row)
            if frames:
                last_frames[(exposure, gain, wb)] = frames[-1]
        sweep_ok = True
    finally:
        if not args.no_restore and not (args.keep_best and sweep_ok):
            restore = {
                k: original[k]
                for k in ("enable_auto_exposure", "enable_auto_white_balance", "exposure", "gain", "white_balance")
                if k in original
            }
            if restore:
                try_update(client, restore)

    normalize_scores(rows)
    rows.sort(key=lambda r: float(r.get("score", -1e9)), reverse=True)

    csv_path = out_dir / "realsense_red_liquid_exposure_sweep.csv"
    fieldnames = [
        "score", "exposure", "gain", "white_balance", "valid_frames",
        "sharpness", "sharpness_norm", "red_quality", "exposure_quality",
        "over_frac", "under_frac", "red_frac", "red_s_median", "red_v_median", "red_contrast",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in fieldnames})

    if not rows:
        raise SystemExit("[ERROR] no candidate rows were collected")
    best = rows[0]
    best_setting = (int(best["exposure"]), int(best["gain"]), int(best["white_balance"]))

    for rank, row in enumerate(rows[: max(0, args.topk_previews)], start=1):
        setting = (int(row["exposure"]), int(row["gain"]), int(row["white_balance"]))
        frame = last_frames.get(setting)
        if frame is None:
            continue
        label = f"rank={rank} score={float(row['score']):.3f} exp={setting[0]} gain={setting[1]} wb={setting[2]}"
        save_preview(frame, roi, out_dir / f"rank{rank:02d}_exp{setting[0]}_gain{setting[1]}_wb{setting[2]}.png", label)

    report_path = out_dir / "REALSENSE_RED_LIQUID_EXPOSURE_RECOMMENDATION.md"
    with report_path.open("w", encoding="utf-8") as f:
        f.write("# RealSense Red-Liquid Exposure Recommendation\n\n")
        f.write("说明：常见 RealSense D4xx 彩色镜头光圈固定，本脚本只筛选 exposure / gain / white_balance。\n\n")
        f.write("## Best Setting\n\n")
        f.write("```text\n")
        f.write(f"score={float(best['score']):.4f}\n")
        f.write(f"exposure={best_setting[0]}\n")
        f.write(f"gain={best_setting[1]}\n")
        f.write(f"white_balance={best_setting[2]}\n")
        f.write(f"sharpness={float(best['sharpness']):.2f}\n")
        f.write(f"red_quality={float(best['red_quality']):.4f}\n")
        f.write(f"over_frac={float(best['over_frac']):.4f}\n")
        f.write(f"under_frac={float(best['under_frac']):.4f}\n")
        f.write("```\n\n")
        f.write("## Apply After Camera Launch\n\n")
        f.write("```bash\n")
        f.write(f"rosrun dynamic_reconfigure dynparam set {args.dynparam_ns} enable_auto_exposure false\n")
        f.write(f"rosrun dynamic_reconfigure dynparam set {args.dynparam_ns} exposure {best_setting[0]}\n")
        f.write(f"rosrun dynamic_reconfigure dynparam set {args.dynparam_ns} gain {best_setting[1]}\n")
        f.write(f"rosrun dynamic_reconfigure dynparam set {args.dynparam_ns} enable_auto_white_balance false\n")
        f.write(f"rosrun dynamic_reconfigure dynparam set {args.dynparam_ns} white_balance {best_setting[2]}\n")
        f.write("```\n\n")
        f.write("## Output\n\n")
        f.write(f"- CSV: `{csv_path}`\n")
        f.write("- Rank preview images are saved in this directory.\n")

    if args.keep_best:
        try_update(client, {
            "enable_auto_exposure": False,
            "exposure": best_setting[0],
            "gain": best_setting[1],
            "enable_auto_white_balance": False,
            "white_balance": best_setting[2],
        })

    print(f"[OK] CSV: {csv_path}")
    print(f"[OK] report: {report_path}")
    print(
        "[BEST] "
        f"score={float(best['score']):.4f} exposure={best_setting[0]} "
        f"gain={best_setting[1]} white_balance={best_setting[2]}"
    )


if __name__ == "__main__":
    main()
