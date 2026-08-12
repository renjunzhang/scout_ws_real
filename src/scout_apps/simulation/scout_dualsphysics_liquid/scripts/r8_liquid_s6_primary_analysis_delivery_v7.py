#!/usr/bin/env python3
"""Pure single-bag S6 analysis, rendering, media, and transaction composer v7.

The public CLI is static-only.  Runtime library calls accept already-admitted
canonical Gauge/BI4 bytes and exact selected signals.  No solver, candidate,
GPU, sudo, profile, network, optional bag, or path discovery surface exists.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import io
import json
import math
import sys
import tempfile
import warnings
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

import r8_liquid_bi4_reader_v1 as bi4  # noqa: E402
import r8_liquid_s6_canonical_replay_input_gate_v7 as canonical_gate  # noqa: E402
import r8_liquid_s6_primary_selected_signal_reader_v7 as selected_reader  # noqa: E402
import r8_liquid_s6_real_runtime_transaction_v7 as transaction  # noqa: E402


POLICY_PATH = ROOT / "config/target_hosts/liquid_zrj_msi_u2404_s6_real_runtime_delivery_policy_v7.json"
POLICY_SCHEMA_PATH = ROOT / "schema/target_host_s6_real_runtime_delivery_policy_v7.json"
RESULT_SCHEMA_PATH = ROOT / "schema/target_host_s6_real_runtime_result_v7.json"
ATTEMPT_ID = "SIM-S1_CORE_H1_C1_Bsmooth_b01_r01"
FINAL_STATUS = "S6_PRIMARY_R7_BAG_REPLAY_AND_MODEL_COMPARISON_PASS_DEVELOPMENT_ONLY"
SURFACES = ("H_crest", "H_abs", "H_peak_to_peak")
SECONDARIES = ("H_proxy", "H_modal")
WINDOWS = ("first15", "full_motion", "recorded_tail", "solver_tail")
H0_M = 0.058


class S6PrimaryV7Error(ValueError):
    """A frozen input, numeric, rendering, media, or transaction gate failed."""


def canonical_json(value: Any) -> bytes:
    return (json.dumps(value, allow_nan=False, ensure_ascii=False, sort_keys=True,
                       separators=(",", ":")) + "\n").encode("utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_json(value: Any) -> str:
    return sha256_bytes(canonical_json(value))


def assert_deep_closed(value: Any, location: str = "$") -> None:
    if isinstance(value, dict):
        kind = value.get("type")
        if (kind == "object" or isinstance(kind, list) and "object" in kind) and value.get("additionalProperties") is not False:
            raise S6PrimaryV7Error(f"schema object is open at {location}")
        for key, child in value.items(): assert_deep_closed(child, f"{location}/{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value): assert_deep_closed(child, f"{location}/{index}")


def load_policy() -> dict[str, Any]:
    policy = json.loads(POLICY_PATH.read_bytes())
    schema = json.loads(POLICY_SCHEMA_PATH.read_bytes())
    Draft202012Validator.check_schema(schema); assert_deep_closed(schema)
    Draft202012Validator(schema).validate(policy)
    if policy["selection"] != {
        "attempt_id": ATTEMPT_ID, "role": "PRIMARY_BASELINE_ONLY", "planned_denominator": 1,
        "source_outcome": "UNKNOWN", "optional_bag_read": False, "paired_ranking": False,
        "cross_method_ranking": False, "selected_trajectory_cpu_comparison": False,
    } or policy["analysis_contract"]["h0_m"] != H0_M:
        raise S6PrimaryV7Error("policy one-bag selection or h0 drift")
    return policy


def _parse_gauge(raw: bytes, times: Sequence[float]) -> list[float]:
    reader = csv.DictReader(io.StringIO(raw.decode("utf-8-sig"), newline=""), strict=True)
    if tuple(reader.fieldnames or ()) != ("time_s", "zsurf_m"):
        raise S6PrimaryV7Error("Gauge CSV header differs")
    values: list[float] = []
    for index, row in enumerate(reader):
        if index >= len(times) or set(row) != {"time_s", "zsurf_m"}:
            raise S6PrimaryV7Error("Gauge CSV row count/shape differs")
        observed_time = float(row["time_s"])
        if not math.isfinite(observed_time) or abs(observed_time - times[index]) > 1e-9:
            raise S6PrimaryV7Error("Gauge CSV time grid differs")
        try: value = float(row["zsurf_m"])
        except ValueError as exc: raise S6PrimaryV7Error("Gauge value missing/invalid") from exc
        if not math.isfinite(value): raise S6PrimaryV7Error("Gauge value non-finite")
        values.append(value)
    if len(values) != len(times): raise S6PrimaryV7Error("Gauge CSV time slots differ")
    return values


def derive_surface(canonical: Mapping[str, Any], gauge_csv_bytes: Mapping[str, bytes]) -> dict[str, Any]:
    if canonical.get("status") != "PASS_S6_CANONICAL_REPLAY_INPUT_ADMISSION_V7":
        raise S6PrimaryV7Error("canonical replay input is not admitted")
    files = canonical["native_gauges"]["files"]
    if [row["probe_name"] for row in files] != list(canonical_gate.PROBES) or list(gauge_csv_bytes) != list(canonical_gate.PROBES):
        raise S6PrimaryV7Error("Gauge set/order differs")
    times: list[float] | None = None
    matrix: list[list[float]] = []
    for row in files:
        raw = gauge_csv_bytes[row["probe_name"]]
        if sha256_bytes(raw) != row["sha256"] or len(raw) != row["size_bytes"]:
            raise S6PrimaryV7Error("Gauge bytes differ from canonical admission")
        # The actual grid is frozen by the CSV, not reconstructed from row count.
        local_times = [float(line.split(",", 1)[0]) for line in raw.decode("utf-8-sig").splitlines()[1:]]
        if times is None: times = local_times
        if local_times != times: raise S6PrimaryV7Error("Gauge grids differ")
        matrix.append(_parse_gauge(raw, times))
    if sha256_json(times) != canonical["native_gauges"]["time_grid_sha256"]:
        raise S6PrimaryV7Error("actual Gauge grid hash differs from canonical admission")
    rows = []
    for index, time_s in enumerate(times or []):
        heights = [series[index] for series in matrix]
        eta = [height - H0_M for height in heights]
        rows.append({"time_s": time_s, "H_crest_m": max(eta),
                     "H_abs_m": max(abs(value) for value in eta),
                     "H_peak_to_peak_m": max(heights) - min(heights),
                     "valid_probe_count": len(heights), "probe_heights_m": heights})
    return {"time_grid_sha256": sha256_json(times), "rows": rows,
            "probe_names": list(canonical_gate.PROBES), "h0_m": H0_M,
            "fact_source": "SIXTEEN_RAW_NATIVE_JGAUGESWL_CSV"}


def _interpolate(times: Sequence[float], values: Sequence[float], query: float) -> float:
    index = bisect.bisect_left(times, query)
    if index < len(times) and abs(times[index] - query) <= 1e-12: return float(values[index])
    if index == 0 or index == len(times): raise S6PrimaryV7Error("extrapolation forbidden")
    fraction = (query - times[index - 1]) / (times[index] - times[index - 1])
    return float(values[index - 1]) + fraction * (float(values[index]) - float(values[index - 1]))


def _uniform(rows: Sequence[Mapping[str, Any]], key: str) -> tuple[list[float], list[float]]:
    times = [float(row["time_s"]) for row in rows]
    values = [float(row[key]) for row in rows]
    if len(times) < 4: raise S6PrimaryV7Error("metric grid too short")
    dt = (times[-1] - times[0]) / (len(times) - 1)
    if dt <= 0 or any(abs((times[i] - times[i-1]) - dt) > max(1e-9, dt * 1e-6) for i in range(1, len(times))):
        raise S6PrimaryV7Error("metric grid is not uniform")
    return times, values


def _metrics(rows: Sequence[Mapping[str, Any]], key: str) -> dict[str, float]:
    times, values = _uniform(rows, key)
    mean = sum(values) / len(values); centered = [value - mean for value in values]
    best = (0, 0.0, 0.0, -1.0)
    for k in range(1, len(values)//2 + 1):
        real = sum(value * math.cos(2*math.pi*k*i/len(values)) for i, value in enumerate(centered))
        imag = -sum(value * math.sin(2*math.pi*k*i/len(values)) for i, value in enumerate(centered))
        power = real*real + imag*imag
        if power > best[3]: best = (k, real, imag, power)
    peaks = [(times[i], abs(centered[i])) for i in range(1, len(values)-1)
             if abs(centered[i]) >= abs(centered[i-1]) and abs(centered[i]) > abs(centered[i+1]) and abs(centered[i]) > 0]
    damping = 0.0
    if len(peaks) >= 2:
        xs = [row[0] for row in peaks]; ys = [math.log(row[1]) for row in peaks]
        xm = sum(xs)/len(xs); ym = sum(ys)/len(ys); denominator = sum((x-xm)**2 for x in xs)
        damping = -sum((x-xm)*(y-ym) for x, y in zip(xs, ys))/denominator if denominator else 0.0
    return {"amplitude_m": max(abs(value) for value in centered),
            "frequency_hz": best[0]/(len(values)*(times[1]-times[0])),
            "damping_per_s": damping, "phase_rad": math.atan2(best[2], best[1])}


def analyze(surface: Mapping[str, Any], selected: Mapping[str, Any],
            windows: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    load_policy()
    try:
        selected = selected_reader.validate_result(selected)
    except Exception as exc:
        raise S6PrimaryV7Error("selected signals fail the closed single-bag admission") from exc
    if set(windows) != set(WINDOWS): raise S6PrimaryV7Error("four exact windows required")
    normalized = {}
    for name in WINDOWS:
        row = windows[name]
        if set(row) != {"start_index", "end_index", "start_s", "end_s"}:
            raise S6PrimaryV7Error("window is not closed")
        start, end = int(row["start_index"]), int(row["end_index"])
        if not 0 <= start <= end < len(surface["rows"]): raise S6PrimaryV7Error("window index bounds differ")
        if abs(float(row["start_s"]) - surface["rows"][start]["time_s"]) > 1e-9 or abs(float(row["end_s"]) - surface["rows"][end]["time_s"]) > 1e-9:
            raise S6PrimaryV7Error("window times differ from frozen grid")
        normalized[name] = dict(row)
    first, motion, recorded, solver = (normalized[name] for name in WINDOWS)
    if first["start_index"] != motion["start_index"] or first["end_s"] > min(motion["end_s"], first["start_s"] + 15.0) + 1e-9:
        raise S6PrimaryV7Error("first15 window topology differs")
    if recorded["start_index"] < motion["end_index"] or solver["start_index"] < recorded["end_index"]:
        raise S6PrimaryV7Error("tail window topology differs")
    selected_series = {}
    for name in SECONDARIES:
        samples = selected["series"][name]["samples"]
        selected_series[name] = ([float(row["time_since_odom_origin_s"]) for row in samples],
                                 [float(row["value_comparison_mm"])/1000.0 for row in samples])
    overlap_start = selected["time_alignment"]["overlap_start_s"]
    overlap_end = selected["time_alignment"]["overlap_end_s"]
    solver_rows, comparison_rows = [], []
    for source in surface["rows"]:
        row = {key: source[key] for key in ("time_s", "H_crest_m", "H_abs_m", "H_peak_to_peak_m", "valid_probe_count")}
        inside = overlap_start - 1e-12 <= row["time_s"] <= overlap_end + 1e-12
        for name in SECONDARIES:
            times, values = selected_series[name]
            row[f"{name}_m"] = _interpolate(times, values, row["time_s"]) if inside else None
            row[f"{name}_coverage"] = "IN_REGISTERED_OVERLAP" if inside else "NA_OUTSIDE_REGISTERED_OVERLAP"
        solver_rows.append(row)
        if inside: comparison_rows.append(row)
    if len(comparison_rows) < 4: raise S6PrimaryV7Error("comparison overlap too short")
    window_metrics = {}
    for name, limits in normalized.items():
        subset = solver_rows[limits["start_index"]:limits["end_index"]+1]
        window_metrics[name] = {surface_name: _metrics(subset, f"{surface_name}_m") for surface_name in SURFACES}
    series_metrics = {name: _metrics(comparison_rows, f"{name}_m") for name in (*SURFACES, *SECONDARIES)}
    comparisons = []
    for surface_name in SURFACES:
        x = [float(row[f"{surface_name}_m"]) for row in comparison_rows]
        for secondary in SECONDARIES:
            y = [float(row[f"{secondary}_m"]) for row in comparison_rows]
            xm, ym = sum(x)/len(x), sum(y)/len(y)
            cov = sum((a-xm)*(b-ym) for a, b in zip(x, y)); xv = sum((a-xm)**2 for a in x); yv = sum((b-ym)**2 for b in y)
            comparisons.append({"surface": surface_name, "secondary": secondary,
                "amplitude_error_m": series_metrics[secondary]["amplitude_m"]-series_metrics[surface_name]["amplitude_m"],
                "frequency_error_hz": series_metrics[secondary]["frequency_hz"]-series_metrics[surface_name]["frequency_hz"],
                "damping_error_per_s": series_metrics[secondary]["damping_per_s"]-series_metrics[surface_name]["damping_per_s"],
                "phase_error_rad": (series_metrics[secondary]["phase_rad"]-series_metrics[surface_name]["phase_rad"]+math.pi)%(2*math.pi)-math.pi,
                "correlation": cov/math.sqrt(xv*yv) if xv and yv else 0.0,
                "rmse_m": math.sqrt(sum((a-b)**2 for a, b in zip(x, y))/len(x)), "ranking_claimed": False})
    return {"schema_version": "smpcc-r8-liquid-s6-primary-analysis-v7", "status": "PASS_S6_PRIMARY_ANALYSIS_V7",
        "attempt_id": ATTEMPT_ID, "planned_denominator": 1, "source_outcome": "UNKNOWN", "h0_m": H0_M,
        "time_grid_sha256": surface["time_grid_sha256"], "windows": normalized,
        "solver_rows": solver_rows, "comparison_rows": comparison_rows,
        "window_metrics": window_metrics, "series_metrics": series_metrics, "comparisons": comparisons,
        "claims": {"stage6_pass": False, "development_only": True, "physical_reference_pending": True,
            "physical_fidelity_validated": False, "paired_ranking": False, "cross_method_ranking": False,
            "selected_trajectory_cpu_comparison": False, "formal": False, "production": False}}


def render_three_panel(analysis: Mapping[str, Any]) -> dict[str, Any]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image
    rows = analysis["solver_rows"]; times = [row["time_s"] for row in rows]
    colors = {"H_crest":"#0072B2","H_abs":"#E69F00","H_peak_to_peak":"#009E73","H_proxy":"#56B4E9","H_modal":"#CC79A7"}
    styles = {"H_crest":"-","H_abs":"--","H_peak_to_peak":":","H_proxy":"-.","H_modal":(0,(5,2))}
    def draw(gray: bool):
        fig, axes = plt.subplots(3,1,figsize=(7.2,6.6),sharex=True)
        fig.subplots_adjust(left=.14,right=.97,bottom=.12,top=.91,hspace=.28)
        for name in SURFACES: axes[0].plot(times,[row[f"{name}_m"]*1000 for row in rows],label=name,color="black" if gray else colors[name],linestyle=styles[name],linewidth=1.25)
        for name in SECONDARIES:
            values=[math.nan if row[f"{name}_m"] is None else row[f"{name}_m"]*1000 for row in rows]
            axes[1].plot(times,values,label=name,color="black" if gray else colors[name],linestyle=styles[name],linewidth=1.25)
            residual=[math.nan if row[f"{name}_m"] is None else (row["H_crest_m"]-row[f"{name}_m"])*1000 for row in rows]
            axes[2].plot(times,residual,label=f"H_crest - {name}",color="black" if gray else colors[name],linestyle=styles[name],linewidth=1.25)
        for i, axis in enumerate(axes):
            axis.set_ylabel("Height (mm)" if i<2 else "Residual (mm)");axis.grid(True,linewidth=.45,alpha=.65);axis.legend(frameon=False);axis.spines[["top","right"]].set_visible(False)
            axis.margins(x=.02)
        axes[-1].set_xlabel("Time since /odom.header.stamp origin (s)")
        axes[0].set_title("Primary R7 liquid/model comparison — physical reference PENDING")
        axes[-1].set_xlim(times[0], times[-1])
        return fig,axes
    artifacts={}; color,axes=draw(False)
    # Constrained layout needs a second draw after tick locators settle.
    color.canvas.draw()
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        color.canvas.draw()
    glyph_warnings=[str(row.message) for row in caught if "Glyph" in str(row.message)]
    shared=all(axes[0].get_shared_x_axes().joined(axes[0],a) for a in axes[1:])
    renderer=color.canvas.get_renderer();figure_box=color.bbox
    figure_bounds=figure_box.extents;clip_tolerance_px=1.0
    no_clipping=all(figure_bounds[0]-clip_tolerance_px<=corner[0]<=figure_bounds[2]+clip_tolerance_px
        and figure_bounds[1]-clip_tolerance_px<=corner[1]<=figure_bounds[3]+clip_tolerance_px
        for axis in axes for artist in
        (axis.xaxis.label,axis.yaxis.label,axis.title,*axis.get_xticklabels(),*axis.get_yticklabels())
        if artist.get_visible() for corner in (artist.get_window_extent(renderer).p0,artist.get_window_extent(renderer).p1))
    stream=io.BytesIO();color.savefig(stream,format="png",dpi=300);artifacts["figures/primary_shared_x_timeseries.png"]=stream.getvalue()
    stream=io.BytesIO();color.savefig(stream,format="pdf");artifacts["figures/primary_shared_x_timeseries.pdf"]=stream.getvalue()
    stream=io.BytesIO();color.savefig(stream,format="svg");artifacts["figures/primary_shared_x_timeseries.svg"]=stream.getvalue();plt.close(color)
    gray,_=draw(True);gray.canvas.draw();stream=io.BytesIO();gray.savefig(stream,format="png",dpi=300);artifacts["figures/primary_shared_x_timeseries_grayscale.png"]=stream.getvalue();plt.close(gray)
    sizes=[]
    for name in ("figures/primary_shared_x_timeseries.png","figures/primary_shared_x_timeseries_grayscale.png"):
        with Image.open(io.BytesIO(artifacts[name])) as image: image.verify()
        with Image.open(io.BytesIO(artifacts[name])) as image: sizes.append(image.size)
    if (not shared or not no_clipping or glyph_warnings or len(set(sizes))!=1
            or not artifacts["figures/primary_shared_x_timeseries.pdf"].startswith(b"%PDF")
        or b"<svg" not in artifacts["figures/primary_shared_x_timeseries.svg"][:2048]):
        raise S6PrimaryV7Error("figure programmatic QA failed")
    return {"artifacts":artifacts,"qa":{"color_render_pass":True,"grayscale_render_pass":True,
        "svg_render_pass":True,"no_clipping":True,"no_missing_glyphs":True,
        "no_dual_y_axis":True,"source_data_hash_bound":True,
        "multimodal_visual_review":False}}


def render_particle_frames(frame_manifest: Mapping[str, Any], frame_bytes: Mapping[str, bytes],
                           probe_grid: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    images=[]; manifest=[]; grid_hash=sha256_json(list(probe_grid))
    for source in frame_manifest["frames"]:
        relative=source["identity"]["relative_path"];raw=frame_bytes.get(relative)
        if raw is None or sha256_bytes(raw)!=source["identity"]["sha256"]: raise S6PrimaryV7Error("BI4 frame bytes differ")
        try: particles=bi4.extract_u3_particles(bi4.parse_jpartdata_bi4(raw))
        except bi4.Bi4FormatError as exc: raise S6PrimaryV7Error("invalid canonical BI4") from exc
        if particles["particle_count"]!=source["particle_count"] or particles["counts"]!=source["class_counts"]: raise S6PrimaryV7Error("BI4 particle classes differ")
        fig,axes=plt.subplots(1,2,figsize=(8,4),constrained_layout=True);palette={"fixed_boundary":"#666666","moving_boundary":"#E69F00","floating":"#CC79A7","fluid":"#0072B2"}
        for class_name in ("fixed_boundary","moving_boundary","floating","fluid"):
            chosen=[p for p,c in zip(particles["positions_m"],particles["classes"]) if c==class_name]
            if chosen:
                axes[0].scatter([p[0] for p in chosen],[p[2] for p in chosen],s=2,c=palette[class_name],label=class_name)
                axes[1].scatter([p[0] for p in chosen],[p[1] for p in chosen],s=2,c=palette[class_name])
        axes[1].scatter([p["x_m"] for p in probe_grid],[p["y_m"] for p in probe_grid],s=14,facecolors="none",edgecolors="#D55E00",marker="o",label="16 native probes")
        axes[0].set(xlabel="container x (m)",ylabel="z (m)",title=f"t={source['time_s']:.3f} s");axes[1].set(xlabel="container x (m)",ylabel="container y (m)",title="MOVING_CONTAINER_REFERENCE_REF_0")
        for axis in axes: axis.set_aspect("equal",adjustable="datalim");axis.grid(True,linewidth=.3,alpha=.5)
        axes[0].legend(frameon=False,fontsize=6);axes[1].legend(frameon=False,fontsize=6)
        stream=io.BytesIO();fig.savefig(stream,format="png",dpi=120);plt.close(fig);image=stream.getvalue();images.append(image)
        manifest.append({"index":source["index"],"time_s":source["time_s"],"source_bi4_sha256":source["identity"]["sha256"],"rendered_png_sha256":sha256_bytes(image),"probe_grid_sha256":grid_hash,"attachment_frame":"MOVING_CONTAINER_REFERENCE_REF_0"})
    return {"frames":images,"manifest":manifest,"frame_manifest_sha256":sha256_json(frame_manifest),"numeric_fact_source":False}


def encode_media(rendered: Mapping[str, Any], fps: int=10) -> dict[str, Any]:
    import cv2
    import numpy as np
    from PIL import Image
    if len(rendered["frames"])<3 or not 1<=fps<=30: raise S6PrimaryV7Error("media frame/fps contract differs")
    images=[];arrays=[];size=None
    for raw in rendered["frames"]:
        with Image.open(io.BytesIO(raw)) as image:
            rgb=image.convert("RGB");size=size or rgb.size
            if rgb.size!=size: raise S6PrimaryV7Error("rendered frame size drift")
            images.append(rgb.copy());arrays.append(cv2.cvtColor(np.asarray(rgb),cv2.COLOR_RGB2BGR))
    with tempfile.TemporaryDirectory(prefix="r8-s6-v7-media-") as temporary:
        mp4=Path(temporary)/"primary.mp4";gif=Path(temporary)/"primary.gif";writer=cv2.VideoWriter(str(mp4),cv2.VideoWriter_fourcc(*"mp4v"),fps,size)
        if not writer.isOpened(): raise S6PrimaryV7Error("MP4 encoder unavailable")
        for array in arrays: writer.write(array)
        writer.release();images[0].save(gif,save_all=True,append_images=images[1:],duration=round(1000/fps),loop=0,disposal=2,optimize=False)
        capture=cv2.VideoCapture(str(mp4));decoded=0
        observed_fps=float(capture.get(cv2.CAP_PROP_FPS))
        while True:
            ok,frame=capture.read()
            if not ok:break
            if (frame.shape[1],frame.shape[0])!=size:raise S6PrimaryV7Error("MP4 dimensions differ")
            decoded+=1
        capture.release()
        with Image.open(gif) as source:
            gif_frames=int(getattr(source,"n_frames",1))
            gif_duration_ms=0
            if source.size!=size:raise S6PrimaryV7Error("GIF dimensions differ")
            for index in range(gif_frames):
                source.seek(index);source.load();gif_duration_ms+=int(source.info.get("duration",0))
        mp4_raw=mp4.read_bytes();gif_raw=gif.read_bytes()
    expected_duration=len(images)/fps
    if (decoded!=len(images) or gif_frames!=len(images) or abs(observed_fps-fps)>0.01
            or abs(decoded/observed_fps-expected_duration)>max(.01,1/fps)
            or abs(gif_duration_ms/1000-expected_duration)>max(.02,1/fps)):
        raise S6PrimaryV7Error("media complete decode/timing differs")
    keyframes={}
    for label,index in zip(("first","middle","last"),(0,len(images)//2,len(images)-1)):
        stream=io.BytesIO();images[index].save(stream,format="PNG");raw=stream.getvalue()
        with Image.open(io.BytesIO(raw)) as reopened:
            reopened.load()
            if reopened.size!=size:raise S6PrimaryV7Error("keyframe decode dimensions differ")
        source_sha=rendered["manifest"][index]["rendered_png_sha256"]
        if source_sha!=sha256_bytes(rendered["frames"][index]):
            raise S6PrimaryV7Error("keyframe source hash binding differs")
        keyframes[f"keyframes/primary_{label}.png"]=raw
    artifacts={"animation/primary.mp4":mp4_raw,"animation/primary_preview.gif":gif_raw,**keyframes}
    return {"artifacts":artifacts,"manifest":{"schema_version":"smpcc-r8-liquid-s6-media-manifest-v7","attempt_id":ATTEMPT_ID,"source_frame_manifest_sha256":rendered["frame_manifest_sha256"],"frames":rendered["manifest"],"fps":fps,"frame_count":len(images),"duration_s":expected_duration,"decoded_mp4_fps":observed_fps,"decoded_mp4_duration_s":decoded/observed_fps,"decoded_gif_duration_s":gif_duration_ms/1000,"keyframes":{name:{"sha256":sha256_bytes(raw),"source_index":index,"source_rendered_png_sha256":rendered["manifest"][index]["rendered_png_sha256"]} for (name,raw),index in zip(keyframes.items(),(0,len(images)//2,len(images)-1))},"numeric_fact_source":False},"qa":{"mp4_complete_decode":True,"gif_complete_decode":True,"frame_count_matches_manifest":True,"duration_matches_manifest":True,"dimensions_match_manifest":True,"keyframes_complete_decode":True,"media_is_numeric_fact_source":False}}


def build_artifacts(analysis: Mapping[str, Any], selected: Mapping[str, Any], surface: Mapping[str, Any],
                    figures: Mapping[str, Any], media: Mapping[str, Any],
                    visual_review: Mapping[str, Any]) -> dict[str, bytes]:
    rows=analysis["solver_rows"]
    output=io.StringIO(newline="");fields=("time_s","H_crest_m","H_abs_m","H_peak_to_peak_m","H_proxy_m","H_modal_m");writer=csv.DictWriter(output,fieldnames=fields,lineterminator="\n");writer.writeheader()
    for row in rows:writer.writerow({field:"NA" if row.get(field) is None else row[field] for field in fields})
    metrics_csv=io.StringIO(newline="");metric_fields=("surface","secondary","amplitude_error_m","frequency_error_hz","damping_error_per_s","phase_error_rad","correlation","rmse_m");metric_writer=csv.DictWriter(metrics_csv,fieldnames=metric_fields,lineterminator="\n");metric_writer.writeheader()
    for row in analysis["comparisons"]:metric_writer.writerow({key:row[key] for key in metric_fields})
    comparison={"schema_version":"smpcc-r8-liquid-s6-comparison-manifest-v7","status":FINAL_STATUS,"attempt_id":ATTEMPT_ID,"planned_denominator":1,"source_outcome":"UNKNOWN","compared_secondaries":list(SECONDARIES),"paired_ranking":False,"cross_method_ranking":False,"selected_trajectory_cpu_comparison":False,"physical_reference_pending":True,"physical_fidelity_validated":False,"formal":False,"production":False}
    required_review={"schema_version":"smpcc-r8-liquid-s6-multimodal-visual-qa-v7","status":"PASS_S6_MULTIMODAL_VISUAL_QA_V7","reviewed_preview_sha256":sha256_bytes(figures["artifacts"]["figures/primary_shared_x_timeseries.png"]),"reviewed_grayscale_sha256":sha256_bytes(figures["artifacts"]["figures/primary_shared_x_timeseries_grayscale.png"]),"no_clipping":True,"no_missing_glyphs":True,"no_legend_occlusion":True,"panel_alignment":True,"grayscale_distinguishable":True,"data_not_visually_clipped":True,"cross_panel_units_consistent":True}
    if dict(visual_review)!=required_review:raise S6PrimaryV7Error("multimodal visual review evidence differs")
    figure_qa={**figures["qa"],"multimodal_visual_review":True}
    figure_manifest={"schema_version":"smpcc-r8-liquid-s6-figure-manifest-v7","source_analysis_sha256":sha256_json(analysis),"layout":"THREE_VERTICAL_SHARED_X_PANELS","dual_y_axes":False,"palette":"OKABE_ITO","redundant_line_styles":True,"formats":["PNG","PDF","SVG","GRAYSCALE_PNG"],"artifacts":{name:{"sha256":sha256_bytes(raw),"size_bytes":len(raw)} for name,raw in sorted(figures["artifacts"].items())},"qa":figure_qa}
    media_manifest={**media["manifest"],"artifacts":{name:{"sha256":sha256_bytes(raw),"size_bytes":len(raw)} for name,raw in sorted(media["artifacts"].items())}}
    base={"data/surface_timeseries.csv":output.getvalue().encode(),"data/surface_timeseries.json":canonical_json(surface),"data/selected_signals.json":canonical_json(selected),"data/metrics.csv":metrics_csv.getvalue().encode(),"data/metrics.json":canonical_json({"window_metrics":analysis["window_metrics"],"series_metrics":analysis["series_metrics"],"comparisons":analysis["comparisons"]}),"reports/analysis_result.json":canonical_json(analysis),"reports/eda_report.json":canonical_json({"schema_version":"smpcc-r8-liquid-s6-eda-v7","row_count":len(rows),"time_min_s":rows[0]["time_s"],"time_max_s":rows[-1]["time_s"],"missing_selected_outside_overlap":sum(row["H_proxy_m"] is None for row in rows),"probe_count":16,"h0_m":H0_M}),"reports/quality_control.json":canonical_json({"canonical_grid":True,"valid_probes_per_slot":16,"optional_unread":True,"PHYSICAL_REFERENCE_PENDING":True,"visual_qa_programmatic":True,"visual_qa_human_pending":False}),"reports/visual_qa.json":canonical_json(required_review),"reports/figure_manifest.json":canonical_json(figure_manifest),"reports/media_manifest.json":canonical_json(media_manifest),"comparison_manifest.json":canonical_json(comparison),**figures["artifacts"],**media["artifacts"]}
    evidence={"schema_version":"smpcc-r8-liquid-s6-evidence-index-v7","attempt_id":ATTEMPT_ID,"planned_denominator":1,"source_outcome":"UNKNOWN","entries":[{"relative_path":name,"sha256":sha256_bytes(raw),"size_bytes":len(raw)} for name,raw in sorted(base.items())],"excluded_self_referential_paths":["checksums.sha256","evidence_index.json"],"optional_unread":True,"physical_reference_pending":True};base["evidence_index.json"]=canonical_json(evidence)
    base["checksums.sha256"]="".join(f"{sha256_bytes(base[name])}  {name}\n" for name in sorted(base)).encode("ascii")
    return base


def precommit_bundle(runtime_contract: Mapping[str, Any], artifacts: Mapping[str, bytes],
                     canonical_inputs: Mapping[str, Any]) -> dict[str, Any]:
    normal=transaction._normalise_artifacts(artifacts);inventory=transaction._expected_inventory(normal)
    required=json.loads(transaction.ARTIFACT_BUNDLE_SCHEMA_PATH.read_bytes())["$defs"]["requiredArtifacts"]["const"]
    bundle={"schema_version":"smpcc-r8-liquid-s6-real-runtime-artifact-bundle-v7","document_type":"SMPCC_R8_LIQUID_S6_REAL_RUNTIME_ARTIFACT_BUNDLE_V7","status":"S6_PRIMARY_ARTIFACT_BUNDLE_PRECOMMIT_ADMISSION_PASS_V7","attempt_id":ATTEMPT_ID,"planned_denominator":1,"source_outcome":"UNKNOWN","contract":{"sha256":sha256_bytes(canonical_json(runtime_contract)),"size_bytes":len(canonical_json(runtime_contract))},"canonical_inputs":dict(canonical_inputs),"required_artifacts":required,"inventory":inventory,"inventory_sha256":sha256_json(inventory),"quality":{"analysis_status":"PASS_S6_PRIMARY_ANALYSIS_V7","figure_manifest_sha256":sha256_bytes(normal["reports/figure_manifest.json"]),"media_manifest_sha256":sha256_bytes(normal["reports/media_manifest.json"]),"visual_qa_sha256":sha256_bytes(normal["reports/visual_qa.json"]),"comparison_manifest_sha256":sha256_bytes(normal["comparison_manifest.json"]),"evidence_index_sha256":sha256_bytes(normal["evidence_index.json"]),"checksums_sha256":sha256_bytes(normal["checksums.sha256"]),"figure_formats":["PNG","PDF","SVG","GRAYSCALE_PNG"],"keyframe_count":3,"mp4_complete_decode":True,"gif_complete_decode":True,"media_timing_verified":True,"programmatic_visual_qa":True,"multimodal_visual_review":True,"no_clipping":True,"no_missing_glyphs":True,"no_legend_occlusion":True,"panel_alignment":True,"grayscale_distinguishable":True,"no_dual_y_axis":True},"claims":{"stage6_pass":False,"development_only":True,"physical_reference_pending":True,"physical_fidelity_validated":False,"paired_ranking":False,"cross_method_ranking":False,"selected_trajectory_cpu_comparison":False,"formal":False,"production":False,"physical_primary":False}}
    schema=json.loads(transaction.ARTIFACT_BUNDLE_SCHEMA_PATH.read_bytes());Draft202012Validator(schema).validate(bundle);return bundle


def publish(spec: transaction.TransactionSpec, artifacts: Mapping[str, bytes],
            runtime_contract: Mapping[str, Any], artifact_bundle: Mapping[str, Any]) -> dict[str, Any]:
    return transaction.execute_transaction(spec, artifacts, runtime_contract, artifact_bundle)


def result_from_transaction(spec: transaction.TransactionSpec, transaction_report: Mapping[str, Any],
                            runtime_contract_sha256: str, artifact_bundle: Mapping[str, Any]) -> dict[str, Any]:
    bundle_sha=transaction.precommit_bundle_sha256(artifact_bundle)
    if (transaction_report.get("status")!="COMMITTED_RECEIPT_CONSISTENT"
            or not transaction_report["consumer_acceptance"]["accepted"]
            or runtime_contract_sha256!=spec.runtime_contract_sha256
            or transaction_report.get("runtime_contract_sha256")!=runtime_contract_sha256
            or transaction_report.get("precommit_bundle_sha256")!=bundle_sha):raise S6PrimaryV7Error("transaction consumer identity did not accept")
    receipt_sha=transaction_report["receipt"]["sha256"]
    quality=artifact_bundle["quality"]
    result={"schema_version":"smpcc-r8-liquid-s6-real-runtime-result-v7","document_type":"SMPCC_R8_LIQUID_S6_REAL_RUNTIME_RESULT_V7","status":FINAL_STATUS,"attempt_id":ATTEMPT_ID,"planned_denominator":1,"source_outcome":"UNKNOWN","runtime_contract_sha256":runtime_contract_sha256,"precommit_bundle_sha256":bundle_sha,"transaction_receipt_sha256":receipt_sha,"publication":{"root":str(spec.final_root),"final_root_present":True,"exact_inventory":True,"inventory_sha256":transaction_report["publication"]["final_root_inventory_sha256"],"atomic_noreplace":True},"ledger":{"prepared_present":True,"prepared_stage6_pass":False,"prepared_entry_sha256":transaction_report["ledger"]["prepared_entry_sha256"],"committed_present":True,"committed_entry_sha256":transaction_report["ledger"]["committed_entry_sha256"]},"receipt":{"final_receipt_present":True,"receipt_sha256":receipt_sha,"matches_committed":True},"checks":{"three_way_transaction_consistent":True,"canonical_finalized_frame_manifest":True,"native_gauge_grid_exact":True,"frame_gauge_time_grid_same":True,"analysis_complete":quality["analysis_status"]=="PASS_S6_PRIMARY_ANALYSIS_V7","figure_complete":quality["programmatic_visual_qa"] and quality["multimodal_visual_review"] and quality["no_clipping"] and quality["no_missing_glyphs"],"media_complete_decode":quality["mp4_complete_decode"] and quality["gif_complete_decode"] and quality["media_timing_verified"],"evidence_index_complete":bool(quality["evidence_index_sha256"]),"checksums_complete":bool(quality["checksums_sha256"]),"optional_unread":True},"claims":{"stage6_pass":True,"development_only":True,"physical_reference_pending":True,"physical_fidelity_validated":False,"paired_ranking":False,"cross_method_ranking":False,"selected_trajectory_cpu_comparison":False,"formal":False,"production":False,"physical_primary":False}}
    schema=json.loads(RESULT_SCHEMA_PATH.read_bytes());Draft202012Validator.check_schema(schema);assert_deep_closed(schema);Draft202012Validator(schema).validate(result);return result


def self_check() -> dict[str, Any]:
    policy=load_policy();schemas=[POLICY_SCHEMA_PATH,RESULT_SCHEMA_PATH]
    for path in schemas:
        schema=json.loads(path.read_bytes());Draft202012Validator.check_schema(schema);assert_deep_closed(schema)
    return {"status":"PASS_S6_PRIMARY_ANALYSIS_DELIVERY_V7_STATIC_ONLY","policy_sha256":sha256_bytes(POLICY_PATH.read_bytes()),"h0_m":H0_M,"windows":list(WINDOWS),"surfaces":list(SURFACES),"secondaries":list(SECONDARIES),"metrics":policy["analysis_contract"]["metrics"],"planned_denominator":1,"real_input_read":False,"optional_bag_read":False,"external_write":False,"media_executed":False,"candidate_executed":False,"solver_or_gpu_executed":False,"stage6_pass":False}


def main(argv: Sequence[str]|None=None)->int:
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("command",choices=("self-check",),nargs="?",default="self-check");parser.parse_args(argv)
    try:print(json.dumps(self_check(),sort_keys=True,separators=(",",":")));return 0
    except Exception as exc:print(json.dumps({"status":"FAIL_S6_PRIMARY_ANALYSIS_DELIVERY_V7","error":str(exc)},sort_keys=True),file=sys.stderr);return 2


if __name__=="__main__":raise SystemExit(main())


__all__=["S6PrimaryV7Error","analyze","build_artifacts","derive_surface","encode_media","load_policy","precommit_bundle","publish","render_particle_frames","render_three_panel","result_from_transaction","self_check"]
