#!/usr/bin/env python3
"""
Interactive calibration tool for the red-liquid three-ruler measurement chain.

Outputs a YAML consumed by red_liquid_infer_from_bag.py.

Phase flow
----------
  ROI_DRAG  drag a rectangle around the tube region
            (or pass --roi-x/y/w/h to skip)
  WALLS     click left inner wall, then right inner wall (x matters, y ignored)
  LEFT_X    click anywhere on the LEFT ruler column to set its x position
  LEFT      click 0mm → 2mm → … marks on the left ruler (bottom→top in image)
  CENTER_X  click anywhere on the CENTER ruler column
  CENTER    click height marks on center ruler
  RIGHT_X   click anywhere on the RIGHT ruler column
  RIGHT     click height marks on right ruler
  Enter     save YAML

Keys
----
  u      undo last click (works across phase boundaries)
  r      reset current phase
  q/Esc  quit without saving

Design notes
------------
  Ruler x positions are determined by explicit user clicks (not fraction-derived),
  so the calibration bands in red_liquid_infer_from_bag.py always match the physical
  ruler sticker positions.

  _write_yaml enforces that for each ruler, y values are strictly decreasing as mm
  increases (higher liquid = smaller y in image).  Saving is refused otherwise.

Output YAML
-----------
  roi: {x, y, w, h}
  tube_inner: {x_left, x_right}
  rotation_deg: 0.0
  rulers:
    left:
      x_roi: <float>          # x as clicked by user
      reference_points:
        - {y_px_roi: <float>, h_mm: 0.0}
        - {y_px_roi: <float>, h_mm: 2.0}
        ...
    center: ...
    right:  ...
"""

import argparse
import sys
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import yaml

WIN = "red_liquid_calibrate"

RULER_NAMES: Tuple[str, str, str] = ("left", "center", "right")
RULER_COLORS: List[Tuple[int, int, int]] = [
    (0, 165, 255),   # orange – left
    (0, 210,   0),   # green  – center
    (200, 80, 255),  # purple – right
]


# ── Phase state machine ───────────────────────────────────────────────────────

class Phase(Enum):
    ROI_DRAG  = 0
    WALLS     = 1
    LEFT_X    = 2   # click to set left ruler column x
    LEFT      = 3   # click height marks on left ruler
    CENTER_X  = 4
    CENTER    = 5
    RIGHT_X   = 6
    RIGHT     = 7
    DONE      = 8


RULER_X_PHASES  = (Phase.LEFT_X,  Phase.CENTER_X, Phase.RIGHT_X)
RULER_HT_PHASES = (Phase.LEFT,    Phase.CENTER,   Phase.RIGHT)

_NEXT_PHASE: dict = {
    Phase.WALLS:    Phase.LEFT_X,
    Phase.LEFT_X:   Phase.LEFT,
    Phase.LEFT:     Phase.CENTER_X,
    Phase.CENTER_X: Phase.CENTER,
    Phase.CENTER:   Phase.RIGHT_X,
    Phase.RIGHT_X:  Phase.RIGHT,
    Phase.RIGHT:    Phase.DONE,
}


# ── Calibration state ─────────────────────────────────────────────────────────

@dataclass
class _St:
    phase:           Phase = Phase.ROI_DRAG
    drag_start:      Optional[Tuple[int, int]] = None
    drag_cur:        Optional[Tuple[int, int]] = None
    dragging:        bool = False
    roi:             Optional[Tuple[int, int, int, int]] = None  # (x,y,w,h) full-image

    # Tube walls: ROI-relative x coords, sorted [left, right]
    wall_xs: List[int] = field(default_factory=list)

    # Ruler x positions (ROI-relative), set by explicit user click in RULER_X phases
    ruler_actual_xs: List[Optional[float]] = field(default_factory=lambda: [None, None, None])

    # Per-ruler y clicks (ROI-relative), list of 3 lists
    ruler_ys: List[List[float]] = field(default_factory=lambda: [[], [], []])

    # ---- index helpers -------------------------------------------------------
    def ruler_x_idx(self) -> int:
        return {Phase.LEFT_X: 0, Phase.CENTER_X: 1, Phase.RIGHT_X: 2}.get(self.phase, -1)

    def ruler_ht_idx(self) -> int:
        return {Phase.LEFT: 0, Phase.CENTER: 1, Phase.RIGHT: 2}.get(self.phase, -1)

    def x_left(self) -> Optional[float]:
        return float(self.wall_xs[0]) if len(self.wall_xs) >= 1 else None

    def x_right(self) -> Optional[float]:
        return float(self.wall_xs[1]) if len(self.wall_xs) >= 2 else None

    def ruler_x(self, idx: int) -> Optional[float]:
        return self.ruler_actual_xs[idx] if 0 <= idx < 3 else None

    # ---- edit actions -------------------------------------------------------
    def undo(self) -> None:
        if self.phase in RULER_HT_PHASES:
            ridx = self.ruler_ht_idx()
            if self.ruler_ys[ridx]:
                self.ruler_ys[ridx].pop()
                return
            # No y clicks → go back to ruler X phase and clear its x
            self.phase = RULER_X_PHASES[ridx]
            self.ruler_actual_xs[ridx] = None

        elif self.phase in RULER_X_PHASES:
            xidx = self.ruler_x_idx()
            if xidx > 0:
                # Go back to previous HT phase and undo its last click
                self.phase = RULER_HT_PHASES[xidx - 1]
                if self.ruler_ys[xidx - 1]:
                    self.ruler_ys[xidx - 1].pop()
            else:
                # Back to WALLS
                self.phase = Phase.WALLS
                if self.wall_xs:
                    self.wall_xs.pop()

        elif self.phase == Phase.DONE:
            self.phase = Phase.RIGHT
            if self.ruler_ys[2]:
                self.ruler_ys[2].pop()

        elif self.phase == Phase.WALLS:
            if self.wall_xs:
                self.wall_xs.pop()

    def reset_phase(self) -> None:
        if self.phase in RULER_HT_PHASES:
            self.ruler_ys[self.ruler_ht_idx()].clear()
        elif self.phase in RULER_X_PHASES:
            xidx = self.ruler_x_idx()
            self.ruler_actual_xs[xidx] = None
        elif self.phase == Phase.WALLS:
            self.wall_xs.clear()
        elif self.phase == Phase.ROI_DRAG:
            self.roi = None
            self.drag_start = self.drag_cur = None
            self.dragging = False


# ── Overlay renderer ──────────────────────────────────────────────────────────

def _draw_overlay(
    full_img: np.ndarray,
    state: _St,
    heights_mm: List[float],
    zoom: float,
    cursor_roi_x: Optional[int] = None,
    cursor_roi_y: Optional[int] = None,
) -> np.ndarray:

    # ---- ROI drag phase: full image -----------------------------------------
    if state.phase == Phase.ROI_DRAG:
        canvas = full_img.copy()
        if state.drag_start and state.drag_cur:
            cv2.rectangle(canvas, state.drag_start, state.drag_cur, (0, 220, 220), 2)
        if state.roi:
            rx, ry, rw, rh = state.roi
            cv2.rectangle(canvas, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 2)
        cv2.putText(
            canvas,
            "ROI: drag to select tube region. Release to confirm.  r=reset  q=quit",
            (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1,
        )
        return canvas

    if state.roi is None:
        return full_img.copy()

    # ---- All other phases: ROI crop -----------------------------------------
    ox, oy, rw, rh = state.roi
    canvas = full_img[oy:oy + rh, ox:ox + rw].copy()
    h_c, w_c = canvas.shape[:2]

    # Tube walls
    for x_val in [state.x_left(), state.x_right()]:
        if x_val is not None:
            cv2.line(canvas, (int(x_val), 0), (int(x_val), h_c), (0, 220, 220), 1)

    # Ruler columns and confirmed height marks
    for ridx, (rname, rcol) in enumerate(zip(RULER_NAMES, RULER_COLORS)):
        rx_pos = state.ruler_x(ridx)
        x_phase = RULER_X_PHASES[ridx]
        ht_phase = RULER_HT_PHASES[ridx]
        is_x_active  = (state.phase == x_phase)
        is_ht_active = (state.phase == ht_phase)
        is_past = (state.phase.value > ht_phase.value)

        # Draw confirmed ruler x line
        if rx_pos is not None:
            col   = rcol if (is_ht_active or is_past) else tuple(int(c * 0.55) for c in rcol)
            thick = 2 if is_ht_active else 1
            cv2.line(canvas, (int(rx_pos), 0), (int(rx_pos), h_c), col, thick)
            cv2.putText(canvas, rname[0].upper(), (int(rx_pos) + 2, 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, col, 1)

        # Cursor vertical guide during RULER_X phase
        if is_x_active and cursor_roi_x is not None:
            overlay = canvas.copy()
            cv2.line(overlay, (cursor_roi_x, 0), (cursor_roi_x, h_c), rcol, 2)
            cv2.addWeighted(overlay, 0.45, canvas, 0.55, 0, canvas)

        # Confirmed height marks
        for click_idx, y_val in enumerate(state.ruler_ys[ridx]):
            if click_idx >= len(heights_mm):
                break
            h_val  = heights_mm[click_idx]
            ix     = int(rx_pos) if rx_pos is not None else w_c // 2
            iy     = int(y_val)
            cv2.circle(canvas, (ix, iy), 4, rcol, -1)
            cv2.line(canvas, (max(0, ix - 16), iy), (min(w_c, ix + 16), iy), rcol, 1)
            cv2.putText(canvas, f"{h_val:.0f}mm", (ix + 6, iy - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, rcol, 1)

        # Next-mark prompt for active HT phase
        if is_ht_active:
            n_done = len(state.ruler_ys[ridx])
            if n_done < len(heights_mm):
                next_h = heights_mm[n_done]
                cv2.putText(
                    canvas,
                    f"→ {rname} {next_h:.0f}mm  ({n_done + 1}/{len(heights_mm)})",
                    (5, 14 + (ridx + 1) * 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, rcol, 1,
                )

    # Cursor horizontal guide during HT phases
    if cursor_roi_y is not None and state.phase in RULER_HT_PHASES:
        ridx = state.ruler_ht_idx()
        cv2.line(canvas, (0, cursor_roi_y), (w_c, cursor_roi_y),
                 RULER_COLORS[ridx], 1)

    # Status bar
    ph = state.phase
    if ph == Phase.WALLS:
        msg = ("WALLS: click LEFT inner wall"
               if len(state.wall_xs) == 0
               else "WALLS: click RIGHT inner wall")
    elif ph in RULER_X_PHASES:
        xidx = state.ruler_x_idx()
        msg = f"Click anywhere on the {RULER_NAMES[xidx].upper()} ruler column to set its x position"
    elif ph in RULER_HT_PHASES:
        ridx = state.ruler_ht_idx()
        n = len(state.ruler_ys[ridx])
        if n < len(heights_mm):
            msg = (f"{RULER_NAMES[ridx].upper()}: click {heights_mm[n]:.0f}mm mark"
                   f"  ({n + 1}/{len(heights_mm)})")
        else:
            msg = f"{RULER_NAMES[ridx].upper()}: done"
    elif ph == Phase.DONE:
        msg = "All marks done.  Press Enter to save YAML."
    else:
        msg = ""

    hint = "u=undo  r=reset phase  q=quit  Enter=save"
    for i, line in enumerate([msg, hint]):
        if not line:
            continue
        cv2.putText(canvas, line, (5, h_c - 6 - (1 - i) * 17),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (210, 210, 210), 1)

    # Apply zoom
    if abs(zoom - 1.0) > 0.01:
        nw = max(1, int(w_c * zoom))
        nh = max(1, int(h_c * zoom))
        canvas = cv2.resize(canvas, (nw, nh), interpolation=cv2.INTER_LINEAR)

    return canvas


# ── Mouse callback ────────────────────────────────────────────────────────────

class _MouseCB:
    def __init__(self, state: _St, heights_mm: List[float], zoom: float) -> None:
        self.state      = state
        self.heights_mm = heights_mm
        self.zoom       = zoom
        self.cursor_roi_x: Optional[int] = None
        self.cursor_roi_y: Optional[int] = None

    def _unzoom(self, x: int, y: int) -> Tuple[int, int]:
        z = max(0.01, self.zoom)
        return int(x / z), int(y / z)

    def __call__(self, event: int, x: int, y: int, flags: int, param) -> None:  # noqa: ARG002
        st = self.state

        # ---- ROI drag -------------------------------------------------------
        if st.phase == Phase.ROI_DRAG:
            if event == cv2.EVENT_LBUTTONDOWN:
                st.drag_start = (x, y)
                st.drag_cur   = (x, y)
                st.dragging   = True
            elif event == cv2.EVENT_MOUSEMOVE and st.dragging:
                st.drag_cur = (x, y)
            elif event == cv2.EVENT_LBUTTONUP and st.dragging:
                st.drag_cur = (x, y)
                st.dragging = False
                if st.drag_start:
                    x1, y1 = st.drag_start
                    x2, y2 = x, y
                    irx, iry = min(x1, x2), min(y1, y2)
                    irw, irh = abs(x2 - x1), abs(y2 - y1)
                    if irw >= 4 and irh >= 4:
                        st.roi = (irx, iry, irw, irh)
                        st.phase = Phase.WALLS
                        print(f"[INFO] ROI confirmed: x={irx} y={iry} w={irw} h={irh}")
            return

        # ---- Other phases: convert display → ROI coords ---------------------
        rx, ry = self._unzoom(x, y)

        if event == cv2.EVENT_MOUSEMOVE:
            self.cursor_roi_x = rx
            self.cursor_roi_y = ry
            return

        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if st.phase == Phase.WALLS:
            if len(st.wall_xs) < 2:
                st.wall_xs.append(rx)
                print(f"[INFO] wall click {len(st.wall_xs)}: x_roi={rx}")
                if len(st.wall_xs) == 2:
                    if st.wall_xs[0] > st.wall_xs[1]:
                        st.wall_xs[0], st.wall_xs[1] = st.wall_xs[1], st.wall_xs[0]
                    st.phase = Phase.LEFT_X
                    print(f"[INFO] walls: x_left={st.wall_xs[0]}  x_right={st.wall_xs[1]}")

        elif st.phase in RULER_X_PHASES:
            xidx = st.ruler_x_idx()
            st.ruler_actual_xs[xidx] = float(rx)
            st.phase = _NEXT_PHASE[st.phase]
            print(f"[INFO] {RULER_NAMES[xidx]} ruler x_roi = {rx}")

        elif st.phase in RULER_HT_PHASES:
            ridx = st.ruler_ht_idx()
            ys   = st.ruler_ys[ridx]
            if len(ys) < len(self.heights_mm):
                ys.append(float(ry))
                h_val = self.heights_mm[len(ys) - 1]
                print(f"[INFO] {RULER_NAMES[ridx]} {h_val}mm → y_roi={ry}")
                if len(ys) >= len(self.heights_mm):
                    st.phase = _NEXT_PHASE[st.phase]
                    print(f"[INFO] → phase {st.phase.name}")


# ── YAML writer ───────────────────────────────────────────────────────────────

def _write_yaml(
    state: _St,
    heights_mm: List[float],
    out_path: Path,
    rotation_deg: float,
) -> None:
    if state.roi is None:
        raise ValueError("ROI not set.")
    xl = state.x_left()
    xr = state.x_right()
    if xl is None or xr is None:
        raise ValueError("Tube walls not set.")

    rulers = {}
    for ridx, rname in enumerate(RULER_NAMES):
        rx_pos = state.ruler_x(ridx)
        if rx_pos is None:
            raise ValueError(f"Ruler {rname}: x position not clicked.")
        ys = state.ruler_ys[ridx]
        if len(ys) < len(heights_mm):
            raise ValueError(
                f"Ruler {rname}: only {len(ys)}/{len(heights_mm)} height marks clicked."
            )
        ref_pts = [
            {"y_px_roi": round(float(y), 2), "h_mm": float(h)}
            for y, h in zip(ys, heights_mm)
        ]
        rulers[rname] = {
            "x_roi": round(float(rx_pos), 1),
            "reference_points": ref_pts,
        }

    # ---- Monotonicity check (higher mm = smaller y_px_roi in image) --------
    for rname in RULER_NAMES:
        pts = rulers[rname]["reference_points"]
        ys_check = [p["y_px_roi"] for p in pts]
        for i in range(len(ys_check) - 1):
            if ys_check[i] <= ys_check[i + 1]:
                h_a = heights_mm[i]
                h_b = heights_mm[i + 1]
                raise ValueError(
                    f"Ruler {rname}: y({h_a:.0f}mm)={ys_check[i]:.1f} px should be "
                    f"GREATER than y({h_b:.0f}mm)={ys_check[i+1]:.1f} px "
                    f"(higher liquid = smaller y in image). "
                    f"Click heights in ascending order from bottom to top."
                )

    rx, ry, rw, rh = state.roi
    data = {
        "roi":        {"x": rx, "y": ry, "w": rw, "h": rh},
        "tube_inner": {"x_left": int(round(xl)), "x_right": int(round(xr))},
        "rotation_deg": rotation_deg,
        "rulers":     rulers,
    }

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    print(f"[OK] saved YAML: {out_path}")


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Interactive calibration for red-liquid three-ruler measurement."
    )
    p.add_argument("--image",        required=True, help="Reference frame PNG/JPG.")
    p.add_argument("--output-yaml",  required=True, help="Output YAML path.")
    p.add_argument("--output-image", default="",    help="Optional annotated preview image path.")
    p.add_argument(
        "--ruler-heights-mm", default="0,2,4,6,8",
        help="Comma-separated height marks in mm, bottom→top. Default: 0,2,4,6,8",
    )
    p.add_argument(
        "--zoom", type=float, default=1.5,
        help="Display zoom factor for ROI view. Default: 1.5",
    )
    p.add_argument(
        "--rotation-deg", type=float, default=0.0,
        help="Tube axis rotation written to YAML (degrees). Default: 0.0",
    )
    # ROI shortcut
    p.add_argument("--roi-x", type=int, default=None)
    p.add_argument("--roi-y", type=int, default=None)
    p.add_argument("--roi-w", type=int, default=None)
    p.add_argument("--roi-h", type=int, default=None)
    p.add_argument(
        "--wall-left-x", type=int, default=None,
        help="Optional ROI-relative left inner-wall x; requires --wall-right-x and a CLI ROI.",
    )
    p.add_argument(
        "--wall-right-x", type=int, default=None,
        help="Optional ROI-relative right inner-wall x; requires --wall-left-x and a CLI ROI.",
    )
    return p.parse_args()


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> int:
    args = parse_args()

    img_path = Path(args.image).expanduser().resolve()
    if not img_path.exists():
        print(f"[ERROR] image not found: {img_path}", file=sys.stderr)
        return 1
    full_img = cv2.imread(str(img_path))
    if full_img is None:
        print(f"[ERROR] failed to load: {img_path}", file=sys.stderr)
        return 1

    try:
        heights_mm = [float(v) for v in args.ruler_heights_mm.split(",")]
    except ValueError:
        print("[ERROR] --ruler-heights-mm: expected comma-separated floats.", file=sys.stderr)
        return 1
    if len(heights_mm) < 2:
        print("[ERROR] need at least 2 height marks.", file=sys.stderr)
        return 1

    state = _St()

    if all(v is not None for v in [args.roi_x, args.roi_y, args.roi_w, args.roi_h]):
        state.roi   = (args.roi_x, args.roi_y, args.roi_w, args.roi_h)
        state.phase = Phase.WALLS
        print(f"[INFO] ROI from CLI: {state.roi}")

    wall_args = (args.wall_left_x, args.wall_right_x)
    if any(value is not None for value in wall_args):
        if state.roi is None or not all(value is not None for value in wall_args):
            print(
                "[ERROR] --wall-left-x/--wall-right-x require each other and a complete CLI ROI.",
                file=sys.stderr,
            )
            return 1
        wall_left, wall_right = sorted(int(value) for value in wall_args)
        if wall_left < 0 or wall_right > state.roi[2] or wall_left >= wall_right:
            print(
                f"[ERROR] invalid preset walls for ROI width {state.roi[2]}: "
                f"{wall_left}, {wall_right}",
                file=sys.stderr,
            )
            return 1
        state.wall_xs = [wall_left, wall_right]
        state.phase = Phase.LEFT_X
        print(f"[INFO] walls from CLI: x_left={wall_left} x_right={wall_right}")

    zoom = max(0.1, args.zoom)

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    mouse_cb = _MouseCB(state, heights_mm, zoom)
    cv2.setMouseCallback(WIN, mouse_cb)

    print(f"[INFO] heights: {heights_mm} mm")
    print("Controls:  u=undo  r=reset phase  Enter=save  q/Esc=quit")
    print("Flow: ROI → WALLS → LEFT_X → LEFT → CENTER_X → CENTER → RIGHT_X → RIGHT → Enter")

    while True:
        frame = _draw_overlay(
            full_img, state, heights_mm, zoom,
            cursor_roi_x=mouse_cb.cursor_roi_x,
            cursor_roi_y=mouse_cb.cursor_roi_y,
        )
        cv2.imshow(WIN, frame)
        key = cv2.waitKey(30) & 0xFF

        if key in (ord('q'), 27):
            print("[INFO] quit without saving.")
            break

        elif key == ord('u'):
            state.undo()

        elif key == ord('r'):
            state.reset_phase()

        elif key == 13:  # Enter
            if state.phase == Phase.DONE:
                try:
                    _write_yaml(state, heights_mm, Path(args.output_yaml), args.rotation_deg)
                    if args.output_image:
                        preview = _draw_overlay(full_img, state, heights_mm, zoom=1.0)
                        cv2.imwrite(args.output_image, preview)
                        print(f"[OK] preview saved: {args.output_image}")
                    break
                except ValueError as exc:
                    print(f"[ERROR] {exc}", file=sys.stderr)
            else:
                phases_left = []
                if state.phase.value <= Phase.WALLS.value:
                    wall_left = max(0, 2 - len(state.wall_xs))
                    if wall_left:
                        phases_left.append(f"walls({wall_left})")
                for i in range(len(RULER_X_PHASES)):
                    if state.ruler_x(i) is None:
                        phases_left.append(f"{RULER_NAMES[i]}_x")
                    ht_left = max(0, len(heights_mm) - len(state.ruler_ys[i]))
                    if ht_left:
                        phases_left.append(f"{RULER_NAMES[i]}_ht({ht_left})")
                print(f"[WARN] not done yet — remaining: {', '.join(phases_left)}")

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
