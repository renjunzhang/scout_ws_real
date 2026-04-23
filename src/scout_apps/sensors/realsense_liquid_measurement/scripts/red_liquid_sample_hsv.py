#!/usr/bin/env python3
"""
HSV range sampler for red-liquid segmentation calibration.

Click on red liquid body pixels across one or more reference frames.
The script collects HSV samples and outputs suggested CLI thresholds for
red_liquid_infer_from_bag.py.

Usage
-----
  python red_liquid_sample_hsv.py --images frame1.png frame2.png ...
  python red_liquid_sample_hsv.py --images frame1.png --calibration calib.yaml

Keys
----
  Left click   sample pixel (+ patch_radius neighbourhood)
  c            clear all clicks on current image
  Space        next image
  Enter        finish and print suggested thresholds
  q / Esc      quit and print suggested thresholds
"""

import argparse
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

WIN = "red_liquid_sample_hsv"


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Sample HSV values from red liquid pixels to calibrate segmentation thresholds."
    )
    p.add_argument("--images", nargs="+", required=True,
                   help="One or more reference frame images (PNG/JPG).")
    p.add_argument("--calibration", default="",
                   help="Optional YAML from red_liquid_calibrate.py (shows ROI overlay).")
    p.add_argument("--zoom", type=float, default=1.5,
                   help="Display zoom factor. Default: 1.5")
    p.add_argument("--patch-radius", type=int, default=2,
                   help="Sample all pixels within this radius around each click. Default: 2")
    p.add_argument("--h-margin", type=int, default=6,
                   help="Hue margin added beyond observed range. Default: 6")
    p.add_argument("--sv-margin", type=int, default=20,
                   help="Saturation/Value margin subtracted from 5th percentile. Default: 20")
    return p.parse_args()


# ── Calibration overlay ───────────────────────────────────────────────────────

def _load_roi(calib_path: str) -> Optional[Tuple[int, int, int, int]]:
    if not calib_path:
        return None
    path = Path(calib_path).expanduser().resolve()
    if not path.exists():
        return None
    with path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    roi = data.get("roi", {})
    x, y, w, h = roi.get("x"), roi.get("y"), roi.get("w"), roi.get("h")
    if all(v is not None for v in (x, y, w, h)):
        return int(x), int(y), int(w), int(h)
    return None


def _load_tube(calib_path: str) -> Optional[Tuple[int, int]]:
    if not calib_path:
        return None
    path = Path(calib_path).expanduser().resolve()
    if not path.exists():
        return None
    with path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    ti = data.get("tube_inner", {})
    xl, xr = ti.get("x_left"), ti.get("x_right")
    if xl is not None and xr is not None:
        return int(xl), int(xr)
    return None


# ── Sampler state ─────────────────────────────────────────────────────────────

class _State:
    """Accumulates HSV samples and per-image click display positions."""

    def __init__(self) -> None:
        self.all_samples: List[Tuple[int, int, int]] = []   # (H, S, V) across all images
        self.disp_clicks: List[Tuple[int, int, int, int, int]] = []  # (dx, dy, H, S, V)
        self._img_start: int = 0  # index into disp_clicks at start of current image

    def begin_image(self) -> None:
        self._img_start = len(self.disp_clicks)

    def add(self, hsv_patch: np.ndarray, disp_x: int, disp_y: int) -> None:
        h_c = int(np.median(hsv_patch[:, 0]))
        s_c = int(np.median(hsv_patch[:, 1]))
        v_c = int(np.median(hsv_patch[:, 2]))
        for row in hsv_patch:
            self.all_samples.append((int(row[0]), int(row[1]), int(row[2])))
        self.disp_clicks.append((disp_x, disp_y, h_c, s_c, v_c))

    def clear_current_image(self) -> None:
        del self.all_samples[self._img_start * 0:]   # clear only pixels from this image
        # recount pixels: not straightforward since patches vary in size
        # Simpler: track per-image pixel counts separately
        # For now just remove clicks from display; samples already accumulated but we'll re-tally
        del self.disp_clicks[self._img_start:]
        # Rebuild all_samples from remaining clicks (each click stores representative H/S/V)
        # We re-derive from disp_clicks which store median values only — OK for display,
        # but for statistics use all_samples_per_click stored separately.
        pass  # see below

    def current_image_clicks(self) -> List[Tuple[int, int, int, int, int]]:
        return self.disp_clicks[self._img_start:]


# Cleaner state that tracks per-sample ownership:
class _SamplerState:
    def __init__(self) -> None:
        self.samples: List[Tuple[int, int, int]] = []   # (H, S, V) raw pixels
        self.clicks:  List[Tuple[int, int, int, int, int]] = []  # (dx, dy, H, S, V) repr
        self._img_start_samples: int = 0
        self._img_start_clicks:  int = 0

    def begin_image(self) -> None:
        self._img_start_samples = len(self.samples)
        self._img_start_clicks  = len(self.clicks)

    def add(self, hsv_patch: np.ndarray, disp_x: int, disp_y: int) -> None:
        h_c = int(np.median(hsv_patch[:, 0]))
        s_c = int(np.median(hsv_patch[:, 1]))
        v_c = int(np.median(hsv_patch[:, 2]))
        for row in hsv_patch:
            self.samples.append((int(row[0]), int(row[1]), int(row[2])))
        self.clicks.append((disp_x, disp_y, h_c, s_c, v_c))
        print(f"  → H={h_c:3d}  S={s_c:3d}  V={v_c:3d}   "
              f"({len(hsv_patch)} px, total {len(self.samples)})")

    def clear_image(self) -> None:
        del self.samples[self._img_start_samples:]
        del self.clicks[self._img_start_clicks:]
        print(f"[INFO] cleared current image clicks (total samples now {len(self.samples)})")

    def current_clicks(self) -> List[Tuple[int, int, int, int, int]]:
        return self.clicks[self._img_start_clicks:]


# ── Mouse callback ────────────────────────────────────────────────────────────

class _MouseCB:
    def __init__(self, state: _SamplerState, zoom: float, patch_radius: int) -> None:
        self.state        = state
        self.zoom         = zoom
        self.patch_radius = patch_radius
        self._img_bgr: Optional[np.ndarray] = None

    def set_image(self, img_bgr: np.ndarray) -> None:
        self._img_bgr = img_bgr

    def __call__(self, event: int, x: int, y: int, flags: int, param) -> None:  # noqa: ARG002
        if event != cv2.EVENT_LBUTTONDOWN or self._img_bgr is None:
            return
        z  = max(0.01, self.zoom)
        ix = int(x / z)
        iy = int(y / z)
        h_img, w_img = self._img_bgr.shape[:2]
        if not (0 <= ix < w_img and 0 <= iy < h_img):
            return

        r  = self.patch_radius
        y1, y2 = max(0, iy - r), min(h_img, iy + r + 1)
        x1, x2 = max(0, ix - r), min(w_img, ix + r + 1)
        hsv_img = cv2.cvtColor(self._img_bgr, cv2.COLOR_BGR2HSV)
        patch   = hsv_img[y1:y2, x1:x2].reshape(-1, 3)
        self.state.add(patch, x, y)


# ── Display ───────────────────────────────────────────────────────────────────

def _bgr_from_hsv(h: int, s: int, v: int) -> Tuple[int, int, int]:
    px = np.array([[[h, s, v]]], dtype=np.uint8)
    bgr = cv2.cvtColor(px, cv2.COLOR_HSV2BGR)[0, 0]
    return int(bgr[0]), int(bgr[1]), int(bgr[2])


def _draw_frame(
    img_bgr: np.ndarray,
    roi:     Optional[Tuple[int, int, int, int]],
    tube:    Optional[Tuple[int, int]],
    clicks:  List[Tuple[int, int, int, int, int]],
    zoom:    float,
    img_idx: int,
    n_imgs:  int,
    n_total: int,
) -> np.ndarray:
    # Apply zoom first (all overlay coordinates are in display space)
    h_img, w_img = img_bgr.shape[:2]
    if abs(zoom - 1.0) > 0.01:
        nw = max(1, int(w_img * zoom))
        nh = max(1, int(h_img * zoom))
        canvas = cv2.resize(img_bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
    else:
        canvas = img_bgr.copy()
    h_c, w_c = canvas.shape[:2]

    def to_d(x: int, y: int) -> Tuple[int, int]:
        return int(x * zoom), int(y * zoom)

    # ROI rectangle
    if roi:
        rx, ry, rw, rh = roi
        p1 = to_d(rx, ry)
        p2 = to_d(rx + rw, ry + rh)
        cv2.rectangle(canvas, p1, p2, (0, 230, 230), 2)

    # Tube inner walls
    if roi and tube:
        rx, ry, _, rh = roi
        xl, xr = tube
        cv2.line(canvas, to_d(rx + xl, ry), to_d(rx + xl, ry + rh), (0, 180, 180), 1)
        cv2.line(canvas, to_d(rx + xr, ry), to_d(rx + xr, ry + rh), (0, 180, 180), 1)

    # Clicked sample points
    for dx, dy, h, s, v in clicks:
        bcol = _bgr_from_hsv(h, s, v)
        cv2.circle(canvas, (dx, dy), 6, (0, 0, 0), -1)
        cv2.circle(canvas, (dx, dy), 5, bcol, -1)
        label = f"H{h} S{s} V{v}"
        tx = min(dx + 8, w_c - 90)
        ty = max(dy - 6, 12)
        cv2.putText(canvas, label, (tx, ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 0, 0), 2)
        cv2.putText(canvas, label, (tx, ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, bcol, 1)

    # Status bar
    lines = [
        f"Image {img_idx + 1}/{n_imgs}   Samples: {n_total}",
        "Click red liquid body pixels.  c=clear  Space=next  Enter=finish  q=quit",
    ]
    for i, line in enumerate(lines):
        yp = h_c - 8 - (len(lines) - 1 - i) * 18
        cv2.putText(canvas, line, (6, yp), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (30, 30, 30), 2)
        cv2.putText(canvas, line, (6, yp), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (210, 210, 210), 1)

    return canvas


# ── Threshold suggestion ──────────────────────────────────────────────────────

def suggest_thresholds(
    samples:   List[Tuple[int, int, int]],
    h_margin:  int,
    sv_margin: int,
) -> Dict:
    if not samples:
        print("[WARN] No samples collected — cannot suggest thresholds.")
        return {}

    arr = np.array(samples, dtype=np.float32)
    hs  = arr[:, 0]
    ss  = arr[:, 1]
    vs  = arr[:, 2]

    low_h  = hs[hs <= 90]
    high_h = hs[hs > 90]

    if len(low_h) > 0:
        h1_lo = max(0,   int(np.percentile(low_h,  2)) - h_margin)
        h1_hi = min(90,  int(np.percentile(low_h, 98)) + h_margin)
    else:
        h1_lo, h1_hi = 0, 12

    if len(high_h) > 0:
        h2_lo = max(91,  int(np.percentile(high_h,  2)) - h_margin)
        h2_hi = min(179, int(np.percentile(high_h, 98)) + h_margin)
    else:
        h2_lo, h2_hi = 168, 179

    s_min = max(0, int(np.percentile(ss, 5)) - sv_margin)
    v_min = max(0, int(np.percentile(vs, 5)) - sv_margin)

    sep = "=" * 62
    print(f"\n{sep}")
    print(f"  Samples: {len(samples)}  "
          f"(low-H ≤90: {len(low_h)}  high-H >90: {len(high_h)})")
    print()
    print(f"  H   p2={np.percentile(hs, 2):.0f}  p50={np.percentile(hs,50):.0f}"
          f"  p98={np.percentile(hs,98):.0f}")
    print(f"  S   p5={np.percentile(ss, 5):.0f}  p50={np.percentile(ss,50):.0f}"
          f"  p95={np.percentile(ss,95):.0f}")
    print(f"  V   p5={np.percentile(vs, 5):.0f}  p50={np.percentile(vs,50):.0f}"
          f"  p95={np.percentile(vs,95):.0f}")
    print()
    print(f"  Suggested ranges (with margins h±{h_margin}, sv-{sv_margin}):")
    print(f"    hue1: [{h1_lo}, {h1_hi}]   hue2: [{h2_lo}, {h2_hi}]")
    print(f"    sat_min: {s_min}   val_min: {v_min}")
    print()
    print("  Copy-paste CLI args:")
    cli = (f"  --hue1-low {h1_lo} --hue1-high {h1_hi} "
           f"--hue2-low {h2_lo} --hue2-high {h2_hi} "
           f"--sat-min {s_min} --val-min {v_min}")
    print(cli)
    print(sep)

    return {
        "hue1_low": h1_lo, "hue1_high": h1_hi,
        "hue2_low": h2_lo, "hue2_high": h2_hi,
        "sat_min": s_min,  "val_min": v_min,
    }


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> int:
    args    = parse_args()
    roi     = _load_roi(args.calibration)
    tube    = _load_tube(args.calibration)
    state   = _SamplerState()
    zoom    = max(0.1, args.zoom)

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    mouse_cb = _MouseCB(state, zoom, args.patch_radius)
    cv2.setMouseCallback(WIN, mouse_cb)

    img_paths = [Path(p).expanduser().resolve() for p in args.images]
    missing   = [str(p) for p in img_paths if not p.exists()]
    if missing:
        print(f"[ERROR] image(s) not found: {missing}", file=sys.stderr)
        return 1

    print(f"[INFO] {len(img_paths)} image(s)  patch_radius={args.patch_radius}")
    print("Click on red liquid body pixels (NOT reflections, NOT labels).")

    img_idx = 0
    while img_idx < len(img_paths):
        img_bgr = cv2.imread(str(img_paths[img_idx]))
        if img_bgr is None:
            print(f"[WARN] failed to load: {img_paths[img_idx]}", file=sys.stderr)
            img_idx += 1
            continue

        state.begin_image()
        mouse_cb.set_image(img_bgr)
        print(f"\n[Image {img_idx + 1}/{len(img_paths)}] {img_paths[img_idx].name}")

        done = False
        while True:
            frame = _draw_frame(
                img_bgr, roi, tube,
                state.current_clicks(), zoom,
                img_idx, len(img_paths), len(state.samples),
            )
            cv2.imshow(WIN, frame)
            key = cv2.waitKey(30) & 0xFF

            if key in (ord('q'), 27):
                done = True
                break
            elif key == ord('c'):
                state.clear_image()
            elif key == ord(' '):
                img_idx += 1
                break
            elif key == 13:  # Enter
                img_idx = len(img_paths)  # finish all
                done = True
                break

        if done:
            break

    suggest_thresholds(state.samples, args.h_margin, args.sv_margin)
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
