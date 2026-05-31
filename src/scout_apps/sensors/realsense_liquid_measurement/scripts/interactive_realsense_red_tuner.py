#!/usr/bin/env python3
"""
Interactive RealSense RGB tuner for red-liquid visual measurement.

Run this after the RealSense node is already launched. The window shows live RGB,
ROI, red mask, and overlay panels. Trackbars adjust exposure, gain, white balance,
and HSV thresholds. Press `s` to save the current setting before recording bags.

Keys:
  s  save current parameters, preview image, and report
  r  restore auto exposure / auto white balance
  a  apply current trackbar values immediately
  q  quit; Esc is intentionally ignored in the main tuner window
"""

import argparse
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import yaml

try:
    import rospy
    from cv_bridge import CvBridge
    from dynamic_reconfigure.client import Client as DynClient
    from sensor_msgs.msg import Image
except ImportError as exc:  # pragma: no cover - ROS-only script
    raise SystemExit(
        "This script must run in a sourced ROS environment with cv_bridge and "
        "dynamic_reconfigure available."
    ) from exc


WIN = "realsense_red_liquid_tuner"
MASK_WIN = "red_mask_overlay"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Interactively tune RealSense color settings for red-liquid segmentation."
    )
    p.add_argument("--image-topic", default="/camera/color/image_raw")
    p.add_argument("--dynparam-ns", default="/camera/rgb_camera")
    p.add_argument("--calibration", default="",
                   help="Optional red_liquid_calibrate.py YAML. Uses ROI if present.")
    p.add_argument("--roi", default="", help="Manual ROI x,y,w,h if no calibration YAML is available.")
    p.add_argument("--select-roi", action="store_true",
                   help="Select a temporary ROI from the first live frame before tuning.")
    p.add_argument("--out-dir", required=True)
    p.add_argument("--exp-min", type=int, default=1000)
    p.add_argument("--exp-max", type=int, default=33000)
    p.add_argument("--gain-min", type=int, default=16)
    p.add_argument("--gain-max", type=int, default=128)
    p.add_argument("--wb-min", type=int, default=2800)
    p.add_argument("--wb-max", type=int, default=6500)
    p.add_argument("--wb-step", type=int, default=50)
    p.add_argument("--init-exposure", type=int, default=7000)
    p.add_argument("--init-gain", type=int, default=32)
    p.add_argument("--init-white-balance", type=int, default=4200)
    p.add_argument("--hue1-low", type=int, default=0)
    p.add_argument("--hue1-high", type=int, default=12)
    p.add_argument("--hue2-low", type=int, default=168)
    p.add_argument("--hue2-high", type=int, default=179)
    p.add_argument("--sat-min", type=int, default=90)
    p.add_argument("--val-min", type=int, default=60)
    p.add_argument("--morph-kernel", type=int, default=5)
    p.add_argument("--panel-width", type=int, default=760)
    p.add_argument("--panel-height", type=int, default=420)
    p.add_argument("--apply-period", type=float, default=0.25)
    p.add_argument("--no-live-apply", action="store_true",
                   help="Only apply camera parameters when pressing `a`.")
    p.add_argument("--no-guide-lines", action="store_true",
                   help="Disable display-only center and target liquid-level guide lines.")
    p.add_argument("--liquid-target-frac", type=float, default=0.425,
                   help="Display-only target static liquid level as a fraction of ROI height from the top.")
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


def wait_for_first_frame(img_buf: ImageBuffer, timeout: float = 10.0) -> Optional[np.ndarray]:
    start = time.time()
    while not rospy.is_shutdown() and time.time() - start < timeout:
        if img_buf.latest is not None:
            return img_buf.latest.copy()
        rospy.sleep(0.03)
    return None


def select_roi_from_frame(frame: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
    print("[INFO] Select temporary ROI around the red liquid, then press Enter/Space.")
    print("[INFO] Press c in the ROI window to cancel and use full image; avoid Esc on unstable RealSense setups.")
    selected = cv2.selectROI("select_red_liquid_roi", frame, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow("select_red_liquid_roi")
    x, y, w, h = (int(v) for v in selected)
    if w <= 0 or h <= 0:
        return None
    return x, y, w, h


def make_red_mask(crop_bgr: np.ndarray, params: Dict[str, int]) -> np.ndarray:
    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    hue = ((h >= params["hue1_low"]) & (h <= params["hue1_high"])) | (
        (h >= params["hue2_low"]) & (h <= params["hue2_high"])
    )
    mask = (hue & (s >= params["sat_min"]) & (v >= params["val_min"])).astype(np.uint8) * 255
    k = int(params.get("morph_kernel", 0))
    if k >= 3:
        if k % 2 == 0:
            k += 1
        kernel = np.ones((k, k), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def compute_metrics(crop_bgr: np.ndarray, mask: np.ndarray) -> Dict[str, float]:
    gray = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    _, s, v = cv2.split(hsv)
    m = mask > 0
    sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    over_frac = float(np.mean(v >= 245))
    under_frac = float(np.mean(v <= 20))
    red_frac = float(np.mean(m))
    if np.any(m):
        red_s = float(np.median(s[m]))
        red_v = float(np.median(v[m]))
        bg_s = float(np.median(s[~m])) if np.any(~m) else red_s
        bg_v = float(np.median(v[~m])) if np.any(~m) else red_v
        contrast = max(0.0, red_s - bg_s) / 255.0 + max(0.0, red_v - bg_v) / 255.0
    else:
        red_s = red_v = contrast = 0.0
    exposure_quality = 1.0 - min(1.0, 3.0 * over_frac + 2.0 * under_frac)
    red_quality = 0.45 * (red_s / 255.0) + 0.25 * (red_v / 255.0) + 0.20 * contrast
    score = 0.35 * min(1.0, sharpness / 120.0) + 0.35 * red_quality + 0.30 * exposure_quality
    return {
        "sharpness": sharpness,
        "over_frac": over_frac,
        "under_frac": under_frac,
        "red_frac": red_frac,
        "red_s_median": red_s,
        "red_v_median": red_v,
        "red_contrast": contrast,
        "red_quality": red_quality,
        "exposure_quality": exposure_quality,
        "score": score,
    }


def draw_top_boundary(overlay: np.ndarray, mask: np.ndarray) -> None:
    h, w = mask.shape[:2]
    top_points = []
    for x in range(w):
        ys = np.where(mask[:, x] > 0)[0]
        if ys.size:
            top_points.append((x, int(ys.min())))
    if len(top_points) < max(5, int(0.05 * w)):
        return
    pts = np.array(top_points, dtype=np.int32)
    cv2.polylines(overlay, [pts], isClosed=False, color=(0, 255, 255), thickness=2)


def resize_panel(img: np.ndarray, width: int, height: int) -> np.ndarray:
    return cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)


def put_lines(img: np.ndarray, lines: Tuple[str, ...], origin: Tuple[int, int] = (16, 28)) -> None:
    x, y = origin
    for line in lines:
        cv2.putText(img, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2, cv2.LINE_AA)
        y += 25


def draw_guides(
    img: np.ndarray,
    roi: Optional[Tuple[int, int, int, int]],
    target_frac: float,
) -> None:
    h_img, w_img = img.shape[:2]
    cx = w_img // 2
    cy = h_img // 2
    cv2.line(img, (cx, 0), (cx, h_img - 1), (255, 255, 0), 1, cv2.LINE_AA)
    cv2.line(img, (0, cy), (w_img - 1, cy), (255, 255, 0), 1, cv2.LINE_AA)

    frac = max(0.05, min(0.95, float(target_frac)))
    if roi is None:
        x1, x2 = 0, w_img - 1
        y_target = int(round(frac * h_img))
    else:
        x, y, w, h = roi
        x1 = max(0, min(w_img - 1, x))
        x2 = max(x1, min(w_img - 1, x + w))
        y_target = int(round(y + frac * h))
    y_target = max(0, min(h_img - 1, y_target))
    cv2.line(img, (x1, y_target), (x2, y_target), (0, 255, 0), 2, cv2.LINE_AA)
    put_lines(img, (f"target static liquid {frac:.3f}H",), origin=(max(8, x1 + 8), max(24, y_target - 8)))


def read_trackbar(args: argparse.Namespace) -> Dict[str, int]:
    exposure = args.exp_min + cv2.getTrackbarPos("exposure", WIN)
    gain = args.gain_min + cv2.getTrackbarPos("gain", WIN)
    wb = args.wb_min + cv2.getTrackbarPos("white_balance", WIN) * args.wb_step
    return {
        "exposure": exposure,
        "gain": gain,
        "white_balance": wb,
        "hue1_low": cv2.getTrackbarPos("hue1_low", WIN),
        "hue1_high": cv2.getTrackbarPos("hue1_high", WIN),
        "hue2_low": cv2.getTrackbarPos("hue2_low", WIN),
        "hue2_high": cv2.getTrackbarPos("hue2_high", WIN),
        "sat_min": cv2.getTrackbarPos("sat_min", WIN),
        "val_min": cv2.getTrackbarPos("val_min", WIN),
        "morph_kernel": cv2.getTrackbarPos("morph", WIN),
    }


def apply_camera(client: DynClient, params: Dict[str, int]) -> None:
    client.update_configuration({
        "enable_auto_exposure": False,
        "exposure": int(params["exposure"]),
        "gain": int(params["gain"]),
        "enable_auto_white_balance": False,
        "white_balance": int(params["white_balance"]),
    })


def save_current(
    out_dir: Path,
    frame: np.ndarray,
    roi: Optional[Tuple[int, int, int, int]],
    params: Dict[str, int],
    metrics: Dict[str, float],
    dynparam_ns: str,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    yaml_path = out_dir / f"realsense_red_tuning_{stamp}.yaml"
    md_path = out_dir / f"REALSENSE_RED_TUNING_{stamp}.md"
    img_path = out_dir / f"realsense_red_tuning_{stamp}.png"

    data = {
        "dynparam_ns": dynparam_ns,
        "roi": {"x": roi[0], "y": roi[1], "w": roi[2], "h": roi[3]} if roi else None,
        "camera": {
            "enable_auto_exposure": False,
            "exposure": int(params["exposure"]),
            "gain": int(params["gain"]),
            "enable_auto_white_balance": False,
            "white_balance": int(params["white_balance"]),
        },
        "hsv": {
            "hue1_low": int(params["hue1_low"]),
            "hue1_high": int(params["hue1_high"]),
            "hue2_low": int(params["hue2_low"]),
            "hue2_high": int(params["hue2_high"]),
            "sat_min": int(params["sat_min"]),
            "val_min": int(params["val_min"]),
            "morph_kernel": int(params["morph_kernel"]),
            "guide_lines": True,
            "liquid_target_frac": float(params.get("liquid_target_frac", 0.425)),
        },
        "metrics": {k: float(v) for k, v in metrics.items()},
    }
    with yaml_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)

    preview = frame.copy()
    if roi is not None:
        x, y, w, h = roi
        cv2.rectangle(preview, (x, y), (x + w, y + h), (0, 255, 255), 2)
    draw_guides(preview, roi, float(params.get("liquid_target_frac", 0.425)))
    put_lines(preview, (
        f"exp={params['exposure']} gain={params['gain']} wb={params['white_balance']}",
        f"HSV h1=[{params['hue1_low']},{params['hue1_high']}] h2=[{params['hue2_low']},{params['hue2_high']}]",
        f"sat>={params['sat_min']} val>={params['val_min']} score={metrics['score']:.3f}",
    ))
    cv2.imwrite(str(img_path), preview)

    with md_path.open("w", encoding="utf-8") as f:
        f.write("# RealSense Red-Liquid Interactive Tuning\n\n")
        f.write("## Apply Camera Parameters\n\n")
        f.write("```bash\n")
        f.write(f"rosrun dynamic_reconfigure dynparam set {dynparam_ns} enable_auto_exposure false\n")
        f.write(f"rosrun dynamic_reconfigure dynparam set {dynparam_ns} exposure {params['exposure']}\n")
        f.write(f"rosrun dynamic_reconfigure dynparam set {dynparam_ns} gain {params['gain']}\n")
        f.write(f"rosrun dynamic_reconfigure dynparam set {dynparam_ns} enable_auto_white_balance false\n")
        f.write(f"rosrun dynamic_reconfigure dynparam set {dynparam_ns} white_balance {params['white_balance']}\n")
        f.write("```\n\n")
        f.write("## HSV Args\n\n")
        f.write("```bash\n")
        f.write(
            f"--hue1-low {params['hue1_low']} --hue1-high {params['hue1_high']} "
            f"--hue2-low {params['hue2_low']} --hue2-high {params['hue2_high']} "
            f"--sat-min {params['sat_min']} --val-min {params['val_min']} "
            f"--morph-kernel {params['morph_kernel']}\n"
        )
        f.write("```\n\n")
        f.write("## Metrics\n\n")
        f.write("```text\n")
        for key, val in metrics.items():
            f.write(f"{key}={val:.6f}\n")
        f.write("```\n")

    print(f"[OK] saved {yaml_path}")
    print(f"[OK] saved {md_path}")
    print(f"[OK] saved {img_path}")


def create_trackbars(args: argparse.Namespace) -> None:
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.namedWindow(MASK_WIN, cv2.WINDOW_NORMAL)
    cv2.createTrackbar("exposure", WIN, max(0, args.init_exposure - args.exp_min), args.exp_max - args.exp_min, lambda _v: None)
    cv2.createTrackbar("gain", WIN, max(0, args.init_gain - args.gain_min), args.gain_max - args.gain_min, lambda _v: None)
    wb_pos = max(0, int(round((args.init_white_balance - args.wb_min) / float(args.wb_step))))
    cv2.createTrackbar("white_balance", WIN, wb_pos, max(1, int((args.wb_max - args.wb_min) / args.wb_step)), lambda _v: None)
    cv2.createTrackbar("hue1_low", WIN, args.hue1_low, 179, lambda _v: None)
    cv2.createTrackbar("hue1_high", WIN, args.hue1_high, 179, lambda _v: None)
    cv2.createTrackbar("hue2_low", WIN, args.hue2_low, 179, lambda _v: None)
    cv2.createTrackbar("hue2_high", WIN, args.hue2_high, 179, lambda _v: None)
    cv2.createTrackbar("sat_min", WIN, args.sat_min, 255, lambda _v: None)
    cv2.createTrackbar("val_min", WIN, args.val_min, 255, lambda _v: None)
    cv2.createTrackbar("morph", WIN, args.morph_kernel, 21, lambda _v: None)


def main() -> None:
    args = parse_args()
    out_dir = Path(args.out_dir).expanduser()
    roi = load_roi(args)

    rospy.init_node("interactive_realsense_red_tuner", anonymous=True)
    img_buf = ImageBuffer(args.image_topic)
    client = DynClient(args.dynparam_ns, timeout=5.0)
    original = dict(client.get_configuration())

    if roi is None and args.select_roi:
        first = wait_for_first_frame(img_buf)
        if first is None:
            raise SystemExit(f"[ERROR] timed out waiting for image topic: {args.image_topic}")
        roi = select_roi_from_frame(first)

    create_trackbars(args)
    print(f"[INFO] image_topic={args.image_topic}")
    print(f"[INFO] dynparam_ns={args.dynparam_ns}")
    print(f"[INFO] roi={roi if roi is not None else 'full image'}")
    print("[KEYS] s=save  r=restore auto  a=apply  q=quit  Esc=ignored")
    print("[INFO] Use q to quit cleanly. Do not use Esc if the RealSense node is unstable.")
    print("[INFO] Guide lines are display-only: cyan=image center, green=target static liquid level.")
    print("[INFO] RealSense color aperture is fixed; tune exposure/gain/white_balance.")

    last_applied: Optional[Tuple[int, int, int]] = None
    last_apply_time = 0.0
    current_metrics: Dict[str, float] = {"score": 0.0}

    while not rospy.is_shutdown():
        frame = img_buf.latest
        params = read_trackbar(args)
        params["liquid_target_frac"] = float(args.liquid_target_frac)
        now = time.time()
        setting = (params["exposure"], params["gain"], params["white_balance"])
        if not args.no_live_apply and (setting != last_applied) and (now - last_apply_time >= args.apply_period):
            apply_camera(client, params)
            last_applied = setting
            last_apply_time = now

        if frame is None:
            blank = np.zeros((300, 900, 3), dtype=np.uint8)
            put_lines(blank, ("Waiting for image topic...", args.image_topic))
            cv2.imshow(WIN, blank)
            key = cv2.waitKey(30) & 0xFF
            if key == ord("q"):
                break
            if key == 27:
                print("[WARN] Esc ignored. Press q to quit cleanly.")
            continue

        crop = clip_roi(frame, roi)
        mask = make_red_mask(crop, params)
        current_metrics = compute_metrics(crop, mask)

        full = frame.copy()
        if roi is not None:
            x, y, w, h = roi
            cv2.rectangle(full, (x, y), (x + w, y + h), (0, 255, 255), 2)
        if not args.no_guide_lines:
            draw_guides(full, roi, args.liquid_target_frac)

        overlay = crop.copy()
        red_overlay = np.zeros_like(overlay)
        red_overlay[:, :, 2] = mask
        overlay = cv2.addWeighted(overlay, 0.72, red_overlay, 0.28, 0.0)
        draw_top_boundary(overlay, mask)
        if not args.no_guide_lines:
            draw_guides(overlay, None, args.liquid_target_frac)

        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        h, w = args.panel_height, args.panel_width
        p1 = resize_panel(full, w, h)
        p2 = resize_panel(crop, w, h)
        p3 = resize_panel(mask_bgr, w, h)
        p4 = resize_panel(overlay, w, h)

        put_lines(p1, ("RGB + ROI", f"exp={params['exposure']} gain={params['gain']} wb={params['white_balance']}"))
        put_lines(p2, ("ROI", f"sharp={current_metrics['sharpness']:.1f} score={current_metrics['score']:.3f}"))
        put_lines(p3, ("HSV red mask", f"red_frac={current_metrics['red_frac']:.3f} S={current_metrics['red_s_median']:.0f} V={current_metrics['red_v_median']:.0f}"))
        put_lines(p4, ("Overlay + top boundary", f"over={current_metrics['over_frac']:.3f} under={current_metrics['under_frac']:.3f}"))

        grid = np.vstack((np.hstack((p1, p2)), np.hstack((p3, p4))))
        cv2.imshow(WIN, grid)
        cv2.imshow(MASK_WIN, p4)

        key = cv2.waitKey(20) & 0xFF
        if key == ord("q"):
            break
        if key == 27:
            print("[WARN] Esc ignored. Press q to quit cleanly.")
        if key == ord("a"):
            apply_camera(client, params)
            last_applied = setting
            last_apply_time = time.time()
            print(f"[APPLY] exposure={setting[0]} gain={setting[1]} white_balance={setting[2]}")
        elif key == ord("r"):
            restore = {
                k: original[k]
                for k in ("enable_auto_exposure", "enable_auto_white_balance", "exposure", "gain", "white_balance")
                if k in original
            }
            if restore:
                client.update_configuration(restore)
            print("[RESTORE] restored original camera configuration")
        elif key == ord("s"):
            save_current(out_dir, frame, roi, params, current_metrics, args.dynparam_ns)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
