#!/usr/bin/env python3
"""Validate, export and visualize one immutable U3 GenCase BI4 case.

The source case is opened read-only with SHA-256 pinning.  Every derived file
is written into a new output directory; existing output paths are refused.
The 2-D orthographic projections are the geometry-validation view.  The 3-D
views are explicitly contextual because perspective can distort distances.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import stat
import sys
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from r8_liquid_bi4_reader_v1 import (
    Bi4FormatError,
    SecureFile,
    extract_u3_particles,
    parse_jpartdata_bi4,
    read_regular_file,
)


FORMAT_VERSION = "r8-liquid-u3-case-visualization-v1"
STATUS_LABEL = "SIM_ONLY_UNVALIDATED"
MAX_XML_BYTES = 1024 * 1024
MAX_OUT_BYTES = 1024 * 1024

EXPECTED_CLASSES = ("fixed_boundary", "moving_boundary", "floating", "fluid")
CLASS_STYLE = {
    "fixed_boundary": {"label": "Fixed boundary", "color": "#555555", "marker": "x"},
    "moving_boundary": {"label": "Moving boundary", "color": "#E69F00", "marker": "s"},
    "floating": {"label": "Floating", "color": "#009E73", "marker": "^"},
    "fluid": {"label": "Fluid", "color": "#0072B2", "marker": "o"},
}


def _strict_float(value: str, label: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise Bi4FormatError(f"invalid floating-point value for {label}: {value!r}") from exc
    if not math.isfinite(parsed):
        raise Bi4FormatError(f"non-finite floating-point value for {label}")
    return parsed


def _strict_int(value: str, label: str) -> int:
    if not re.fullmatch(r"[0-9]+", value or ""):
        raise Bi4FormatError(f"invalid integer value for {label}: {value!r}")
    return int(value)


def _parse_triplet(node: ET.Element, label: str) -> tuple[float, float, float]:
    return tuple(_strict_float(node.attrib[axis], f"{label}.{axis}") for axis in "xyz")


def parse_case_xml(source: SecureFile) -> dict[str, Any]:
    """Extract the exact XML facts that must agree with BI4 and OUT."""

    try:
        root = ET.fromstring(source.data)
    except ET.ParseError as exc:
        raise Bi4FormatError(f"invalid case XML: {exc}") from exc
    if root.tag != "case":
        raise Bi4FormatError(f"unexpected XML root {root.tag!r}")
    particles = root.find("./execution/particles")
    constants = root.find("./execution/constants")
    if particles is None or constants is None:
        raise Bi4FormatError("case XML lacks execution particles/constants")
    summary = particles.find("./_summary")
    fixed = particles.find("./fixed")
    fluid = particles.find("./fluid")
    positions = summary.find("./positions") if summary is not None else None
    if None in (summary, fixed, fluid, positions):
        raise Bi4FormatError("case XML lacks particle summary/fixed/fluid records")
    pos_min = positions.find("./posmin")
    pos_max = positions.find("./posmax")
    dp = constants.find("./dp")
    rhop0 = constants.find("./rhop0")
    rhopgradient = root.find("./casedef/constantsdef/rhopgradient")
    parameters = {
        node.attrib.get("key", ""): node.attrib.get("value", "")
        for node in root.findall("./execution/parameters/parameter")
    }
    motion = root.find("./execution/motion")
    if None in (pos_min, pos_max, dp, rhop0, rhopgradient, motion):
        raise Bi4FormatError("case XML lacks required positions/constants/motion")
    if "RhopOutMin" not in parameters or "RhopOutMax" not in parameters:
        raise Bi4FormatError("case XML lacks density output limits")
    if motion.attrib or (motion.text and motion.text.strip()) or list(motion):
        raise Bi4FormatError("U3 static XML motion element is not uniquely empty")

    total = _strict_int(particles.attrib.get("np", ""), "particles.np")
    fixed_count = _strict_int(fixed.attrib.get("count", ""), "fixed.count")
    fluid_count = _strict_int(fluid.attrib.get("count", ""), "fluid.count")
    fixed_begin = _strict_int(fixed.attrib.get("begin", ""), "fixed.begin")
    fluid_begin = _strict_int(fluid.attrib.get("begin", ""), "fluid.begin")
    if fixed_begin != 0 or fluid_begin != fixed_count:
        raise Bi4FormatError("XML particle ID ranges are not contiguous fixed->fluid")
    return {
        "case_app": root.attrib.get("app"),
        "case_date": root.attrib.get("date"),
        "total": total,
        "counts": {
            "fixed_boundary": fixed_count,
            "moving_boundary": 0,
            "floating": 0,
            "fluid": fluid_count,
        },
        "limits_m": {
            axis: [
                _parse_triplet(pos_min, "positions.posmin")[idx],
                _parse_triplet(pos_max, "positions.posmax")[idx],
            ]
            for idx, axis in enumerate("xyz")
        },
        "dp_m": _strict_float(dp.attrib.get("value", ""), "constants.dp"),
        "rhop0_kg_m3": _strict_float(rhop0.attrib.get("value", ""), "constants.rhop0"),
        "rhopgradient": _strict_float(
            rhopgradient.attrib.get("value", ""), "constantsdef.rhopgradient"
        ),
        "rhop_out_min_kg_m3": _strict_float(parameters["RhopOutMin"], "RhopOutMin"),
        "rhop_out_max_kg_m3": _strict_float(parameters["RhopOutMax"], "RhopOutMax"),
        "motion_empty": True,
    }


def _number(text: str) -> int:
    return int(text.replace(",", ""))


def _required_match(pattern: str, text: str, label: str, flags: int = 0) -> re.Match[str]:
    match = re.search(pattern, text, flags)
    if match is None:
        raise Bi4FormatError(f"GenCase OUT lacks required {label} record")
    return match


def parse_gencase_out(source: SecureFile) -> dict[str, Any]:
    try:
        text = source.data.decode("ascii")
    except UnicodeDecodeError as exc:
        raise Bi4FormatError("GenCase OUT is not strict ASCII") from exc
    fixed = _required_match(
        r"Fixed\.+:\s*([0-9,]+)\s+id:\(([0-9]+)-([0-9]+)\)", text, "fixed summary"
    )
    moving = _required_match(r"Moving\.+:\s*([0-9,]+)", text, "moving summary")
    floating = _required_match(r"Floating\.:\s*([0-9,]+)", text, "floating summary")
    fluid = _required_match(
        r"Fluid\.+:\s*([0-9,]+)\s+id:\(([0-9]+)-([0-9]+)\)", text, "fluid summary"
    )
    total = _required_match(r"Total particles:\s*([0-9,]+)", text, "total summary")
    dp = _required_match(
        r"Distance between points \(Dp\):\s*([-+0-9.eE]+)", text, "Dp"
    )
    limits: dict[str, list[float]] = {}
    for axis in "XYZ":
        match = _required_match(
            rf"{axis} range:\s*([-+0-9.eE]+)\s+to\s+([-+0-9.eE]+)\s+\[m\]",
            text,
            f"{axis} limits",
        )
        limits[axis.lower()] = [
            _strict_float(match.group(1), f"OUT {axis} min"),
            _strict_float(match.group(2), f"OUT {axis} max"),
        ]
    if "Finished execution (code=0)." not in text:
        raise Bi4FormatError("GenCase OUT does not record successful completion")

    fixed_count = _number(fixed.group(1))
    fluid_count = _number(fluid.group(1))
    if (int(fixed.group(2)), int(fixed.group(3))) != (0, fixed_count - 1):
        raise Bi4FormatError("OUT fixed ID range is inconsistent")
    if (int(fluid.group(2)), int(fluid.group(3))) != (
        fixed_count,
        fixed_count + fluid_count - 1,
    ):
        raise Bi4FormatError("OUT fluid ID range is inconsistent")
    return {
        "total": _number(total.group(1)),
        "counts": {
            "fixed_boundary": fixed_count,
            "moving_boundary": _number(moving.group(1)),
            "floating": _number(floating.group(1)),
            "fluid": fluid_count,
        },
        "limits_m": limits,
        "dp_m": _strict_float(dp.group(1), "OUT Dp"),
        "finished_code": 0,
    }


def _close(a: float, b: float, tolerance: float = 1e-12) -> bool:
    return math.isclose(a, b, rel_tol=0.0, abs_tol=tolerance)


def validate_cross_sources(
    root_values: dict[str, Any], particles: dict[str, Any], xml: dict[str, Any], out: dict[str, Any]
) -> dict[str, Any]:
    checks: dict[str, bool] = {
        "bi4_xml_total": particles["particle_count"] == xml["total"],
        "bi4_out_total": particles["particle_count"] == out["total"],
        "bi4_xml_counts": particles["counts"] == xml["counts"],
        "bi4_out_counts": particles["counts"] == out["counts"],
        "xml_out_counts": xml["counts"] == out["counts"],
        "bi4_xml_dp": _close(float(root_values["Dp"]), xml["dp_m"]),
        "bi4_out_dp": _close(float(root_values["Dp"]), out["dp_m"]),
        "xml_out_dp": _close(xml["dp_m"], out["dp_m"]),
        "xml_motion_empty": xml["motion_empty"],
    }
    for axis in "xyz":
        for index, side in enumerate(("min", "max")):
            checks[f"bi4_xml_{axis}_{side}"] = _close(
                particles["limits_m"][axis][index], xml["limits_m"][axis][index]
            )
            checks[f"bi4_out_{axis}_{side}"] = _close(
                particles["limits_m"][axis][index], out["limits_m"][axis][index]
            )

    velocity_components = [value for triple in particles["velocities_m_s"] for value in triple]
    densities = particles["densities_kg_m3"]
    positions = [value for triple in particles["positions_m"] for value in triple]
    fixed_count = particles["counts"]["fixed_boundary"]
    checks.update(
        {
            "positions_finite": all(math.isfinite(value) for value in positions),
            "velocities_finite": all(math.isfinite(value) for value in velocity_components),
            "static_velocity_zero": all(value == 0.0 for value in velocity_components),
            "densities_finite": all(math.isfinite(value) for value in densities),
            "density_within_xml_output_limits": all(
                xml["rhop_out_min_kg_m3"] <= float(value) <= xml["rhop_out_max_kg_m3"]
                for value in densities
            ),
            "fixed_boundary_density_matches_rhop0": all(
                _close(float(value), float(root_values["Rhop0"]), 1e-6)
                for value in densities[:fixed_count]
            ),
            "fluid_density_not_below_rhop0": all(
                float(value) + 1e-6 >= float(root_values["Rhop0"])
                for value in densities[fixed_count:]
            ),
        }
    )
    failed = sorted(name for name, passed in checks.items() if not passed)
    if failed:
        raise Bi4FormatError(f"cross-source validation failed: {', '.join(failed)}")
    return {"status": "PASS", "checks": checks, "failed": failed}


def _write_bytes(path: Path, data: bytes, mode: int = 0o640) -> None:
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0)
    fd = os.open(path, flags, mode)
    try:
        view = memoryview(data)
        while view:
            written = os.write(fd, view)
            if written <= 0:
                raise OSError(f"short write to {path}")
            view = view[written:]
        os.fsync(fd)
    finally:
        os.close(fd)


def _json_clean(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _json_clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_clean(item) for item in value]
    if hasattr(value, "item"):
        return _json_clean(value.item())
    return value


def _write_json(path: Path, value: Any) -> None:
    payload = json.dumps(
        _json_clean(value), ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False
    ).encode("utf-8") + b"\n"
    _write_bytes(path, payload)


def _write_text(path: Path, text: str) -> None:
    _write_bytes(path, text.encode("utf-8"))


def write_particles_csv(path: Path, particles: dict[str, Any]) -> None:
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0)
    fd = os.open(path, flags, 0o640)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(
                [
                    "particle_id",
                    "particle_class",
                    "x_m",
                    "y_m",
                    "z_m",
                    "vx_m_s",
                    "vy_m_s",
                    "vz_m_s",
                    "rhop_kg_m3",
                ]
            )
            for row in zip(
                particles["ids"],
                particles["classes"],
                particles["positions_m"],
                particles["velocities_m_s"],
                particles["densities_kg_m3"],
                strict=True,
            ):
                particle_id, particle_class, position, velocity, density = row
                writer.writerow(
                    [
                        particle_id,
                        particle_class,
                        *(format(value, ".17g") for value in position),
                        *(format(value, ".9g") for value in velocity),
                        format(density, ".9g"),
                    ]
                )
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        # fd is owned by fdopen once entered; no deletion is attempted on failure.
        raise


def _load_skill_modules(skill_scripts: Path):
    if not skill_scripts.is_dir():
        raise RuntimeError(f"SciPilot scripts directory does not exist: {skill_scripts}")
    required = (
        "setup_style.py",
        "layout_tools.py",
        "visual_qa.py",
        "export_figure.py",
        "check_figure.py",
        "profile_data.py",
    )
    missing = [name for name in required if not (skill_scripts / name).is_file()]
    if missing:
        raise RuntimeError(f"SciPilot scripts missing: {', '.join(missing)}")
    sys.path.insert(0, str(skill_scripts))
    from check_figure import check_figure
    from export_figure import export_figure
    from layout_tools import add_panel_labels, finalize_figure
    from profile_data import profile_data, render_report
    from setup_style import setup_style
    from visual_qa import audit_layout, render_preview

    return {
        "setup_style": setup_style,
        "add_panel_labels": add_panel_labels,
        "finalize_figure": finalize_figure,
        "audit_layout": audit_layout,
        "render_preview": render_preview,
        "export_figure": export_figure,
        "check_figure": check_figure,
        "profile_data": profile_data,
        "render_report": render_report,
    }


def _active_classes(particles: dict[str, Any]) -> list[str]:
    return [name for name in EXPECTED_CLASSES if particles["counts"].get(name, 0)]


def _scatter_projection(ax, coords, classes, x_idx: int, y_idx: int, active: list[str]) -> None:
    import numpy as np

    for name in reversed(active):
        mask = np.asarray([value == name for value in classes])
        style = CLASS_STYLE[name]
        is_boundary = name != "fluid"
        ax.scatter(
            coords[mask, x_idx],
            coords[mask, y_idx],
            s=4.0 if is_boundary else 3.0,
            marker=style["marker"],
            c=style["color"],
            alpha=0.58 if is_boundary else 0.28,
            linewidths=0.35 if is_boundary else 0.0,
            rasterized=False,
        )


def _portable_figure_info(
    info: dict[str, Any], path: Path, output_root: Path
) -> dict[str, Any]:
    """Replace an ephemeral pre-publication path with a package-relative path."""

    portable = dict(info)
    portable["path"] = path.relative_to(output_root).as_posix()
    return portable


def render_figures(
    output: Path,
    particles: dict[str, Any],
    root_values: dict[str, Any],
    skill: dict[str, Any],
) -> tuple[list[Path], dict[str, Any]]:
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.lines import Line2D
    from PIL import Image

    style_info = skill["setup_style"](
        journal="general", lang="en", use_sciplots=False, constrained_layout=False
    )
    figures_dir = output / "figures"
    reports_dir = output / "reports"
    coords = np.asarray(particles["positions_m"], dtype=np.float64) * 1000.0
    classes = np.asarray(particles["classes"], dtype=object)
    active = _active_classes(particles)
    dp_mm = float(root_values["Dp"]) * 1000.0
    limits = {
        axis: np.asarray(particles["limits_m"][axis], dtype=float) * 1000.0 for axis in "xyz"
    }
    legend_handles = [
        Line2D(
            [0],
            [0],
            linestyle="none",
            marker=CLASS_STYLE[name]["marker"],
            markerfacecolor=CLASS_STYLE[name]["color"],
            markeredgecolor=CLASS_STYLE[name]["color"],
            markersize=5,
            label=f"{CLASS_STYLE[name]['label']} (n={particles['counts'][name]:,})",
        )
        for name in active
    ]
    qa: dict[str, Any] = {"style": style_info, "figures": {}}
    products: list[Path] = []

    # Primary validation is split after the three-panel layout failed visual QA:
    # two vertical sections plus one independent top view preserve scale/readability.
    fig, axes = plt.subplots(1, 2, figsize=(5.6, 4.2), constrained_layout=False)
    specs = (
        (axes[0], 0, 2, "X-Z longitudinal", "X (mm)", "Z (mm)", "x", "z"),
        (axes[1], 1, 2, "Y-Z transverse", "Y (mm)", "Z (mm)", "y", "z"),
    )
    for ax, x_idx, y_idx, title, xlabel, ylabel, x_key, y_key in specs:
        _scatter_projection(ax, coords, classes, x_idx, y_idx, active)
        ax.set_title(title, pad=6)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.xaxis.labelpad = 2.0
        ax.yaxis.labelpad = 1.0
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlim(limits[x_key][0] - dp_mm, limits[x_key][1] + dp_mm)
        ax.set_ylim(limits[y_key][0] - dp_mm, limits[y_key][1] + dp_mm)
        ax.grid(True, color="#D9D9D9", linewidth=0.35, linestyle=":", zorder=0)
        ax.minorticks_off()
        ax.tick_params(top=False, right=False, pad=2)
    fig.suptitle(
        f"U3 C1 particle geometry — vertical sections (dp={dp_mm:g} mm)", y=0.975
    )
    fig.legend(
        handles=legend_handles,
        loc="lower center",
        bbox_to_anchor=(0.5, 0.015),
        ncol=max(1, len(legend_handles)),
        frameon=False,
    )
    fig.subplots_adjust(left=0.105, right=0.975, bottom=0.19, top=0.84, wspace=0.30)
    skill["add_panel_labels"](
        fig,
        axes=axes,
        style="nature",
        x_offset_pt=-10,
        y_offset_pt=4,
        ha="right",
        va="bottom",
    )
    fig.canvas.draw()
    sections_preview = figures_dir / "_preview_vertical_sections.png"
    skill["render_preview"](fig, str(sections_preview), dpi=150)
    sections_issues = skill["audit_layout"](fig)
    if any(severity == "FAIL" for severity, _ in sections_issues):
        raise RuntimeError(f"vertical-sections visual QA failed: {sections_issues}")
    base = figures_dir / "u3_c1_particle_vertical_sections"
    exported = skill["export_figure"](
        fig,
        str(base),
        formats=["pdf", "svg", "png"],
        size_inches=(5.6, 4.2),
        dpi=300,
        grayscale_preview=False,
        tight=False,
    )
    grayscale = figures_dir / "u3_c1_particle_vertical_sections_grayscale.png"
    with Image.open(base.with_suffix(".png")) as image:
        image.convert("L").save(grayscale, dpi=(300, 300))
    plt.close(fig)
    section_products = [Path(path) for path in exported] + [grayscale, sections_preview]
    products.extend(section_products)
    qa["figures"]["vertical_sections"] = {
        "role": "primary_geometry_validation",
        "programmatic_issues": sections_issues,
        "size_inches": [5.6, 4.2],
    }

    fig, ax = plt.subplots(figsize=(4.4, 4.4), constrained_layout=False)
    _scatter_projection(ax, coords, classes, 0, 1, active)
    ax.set_title("X-Y top projection", pad=6)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.xaxis.labelpad = 2.0
    ax.yaxis.labelpad = 1.0
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(limits["x"][0] - dp_mm, limits["x"][1] + dp_mm)
    ax.set_ylim(limits["y"][0] - dp_mm, limits["y"][1] + dp_mm)
    ax.grid(True, color="#D9D9D9", linewidth=0.35, linestyle=":", zorder=0)
    ax.minorticks_off()
    ax.tick_params(top=False, right=False, pad=2)
    fig.suptitle(f"U3 C1 particle geometry — top view (dp={dp_mm:g} mm)", y=0.975)
    fig.legend(
        handles=legend_handles,
        loc="lower center",
        bbox_to_anchor=(0.5, 0.015),
        ncol=max(1, len(legend_handles)),
        frameon=False,
    )
    fig.subplots_adjust(left=0.14, right=0.97, bottom=0.19, top=0.84)
    fig.canvas.draw()
    top_preview = figures_dir / "_preview_top_view.png"
    skill["render_preview"](fig, str(top_preview), dpi=150)
    top_issues = skill["audit_layout"](fig)
    if any(severity == "FAIL" for severity, _ in top_issues):
        raise RuntimeError(f"top-view visual QA failed: {top_issues}")
    top_base = figures_dir / "u3_c1_particle_top_view"
    top_exported = skill["export_figure"](
        fig,
        str(top_base),
        formats=["pdf", "svg", "png"],
        size_inches=(4.4, 4.4),
        dpi=300,
        grayscale_preview=False,
        tight=False,
    )
    top_gray = figures_dir / "u3_c1_particle_top_view_grayscale.png"
    with Image.open(top_base.with_suffix(".png")) as image:
        image.convert("L").save(top_gray, dpi=(300, 300))
    plt.close(fig)
    top_products = [Path(path) for path in top_exported] + [top_gray, top_preview]
    products.extend(top_products)
    qa["figures"]["top_view"] = {
        "role": "primary_geometry_validation",
        "programmatic_issues": top_issues,
        "size_inches": [4.4, 4.4],
    }

    # Context-only 3-D view.  Validation conclusions come from the projections above.
    fig = plt.figure(figsize=(5.2, 4.8), constrained_layout=True)
    ax = fig.add_subplot(111, projection="3d")
    for name in reversed(active):
        mask = classes == name
        style = CLASS_STYLE[name]
        is_boundary = name != "fluid"
        ax.scatter(
            coords[mask, 0],
            coords[mask, 1],
            coords[mask, 2],
            s=3.0 if is_boundary else 2.2,
            marker=style["marker"],
            c=style["color"],
            alpha=0.55 if is_boundary else 0.20,
            linewidths=0.3 if is_boundary else 0.0,
            depthshade=False,
        )
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_xlim(limits["x"][0] - dp_mm, limits["x"][1] + dp_mm)
    ax.set_ylim(limits["y"][0] - dp_mm, limits["y"][1] + dp_mm)
    ax.set_zlim(limits["z"][0] - dp_mm, limits["z"][1] + dp_mm)
    spans = [limits[axis][1] - limits[axis][0] for axis in "xyz"]
    ax.set_box_aspect(spans)
    ax.view_init(elev=22, azim=-52)
    ax.set_xticks([-15, 0, 15])
    ax.set_yticks([-15, 0, 15])
    ax.set_zticks([0, 20, 40, 60])
    ax.minorticks_off()
    ax.grid(False)
    fig.suptitle("U3 C1 particle geometry — 3D context only")
    fig.legend(
        handles=legend_handles,
        loc="outside lower center",
        ncol=max(1, len(legend_handles)),
        frameon=False,
    )
    skill["finalize_figure"](fig)
    context_preview = figures_dir / "_preview_3d_context.png"
    skill["render_preview"](fig, str(context_preview), dpi=150)
    context_issues = skill["audit_layout"](fig)
    if any(severity == "FAIL" for severity, _ in context_issues):
        raise RuntimeError(f"3D visual QA failed: {context_issues}")
    context_base = figures_dir / "u3_c1_particle_3d_context"
    context_exported = skill["export_figure"](
        fig,
        str(context_base),
        formats=["pdf", "svg", "png"],
        size_inches=(5.2, 4.8),
        dpi=300,
        grayscale_preview=False,
        tight=False,
    )
    context_gray = figures_dir / "u3_c1_particle_3d_context_grayscale.png"
    with Image.open(context_base.with_suffix(".png")) as image:
        image.convert("L").save(context_gray, dpi=(300, 300))
    plt.close(fig)
    context_products = [Path(path) for path in context_exported] + [context_gray, context_preview]
    products.extend(context_products)
    qa["figures"]["3d_context"] = {
        "role": "context_only_not_metric_validation",
        "programmatic_issues": context_issues,
        "size_inches": [5.2, 4.8],
    }

    # Self-contained interactive 3-D HTML for local inspection on either computer.
    import plotly.graph_objects as go

    traces = []
    for name in reversed(active):
        mask = classes == name
        style = CLASS_STYLE[name]
        traces.append(
            go.Scatter3d(
                x=coords[mask, 0],
                y=coords[mask, 1],
                z=coords[mask, 2],
                mode="markers",
                name=f"{style['label']} (n={particles['counts'][name]:,})",
                marker={
                    "size": 2.2 if name == "fluid" else 2.8,
                    "color": style["color"],
                    "symbol": "circle" if name == "fluid" else "x",
                    "opacity": 0.42 if name == "fluid" else 0.72,
                },
                hovertemplate=(
                    f"{style['label']}<br>X=%{{x:.3f}} mm<br>Y=%{{y:.3f}} mm"
                    "<br>Z=%{z:.3f} mm<extra></extra>"
                ),
            )
        )
    interactive = go.Figure(data=traces)
    interactive.update_layout(
        template="plotly_white",
        title="U3 C1 particle geometry — interactive 3D context",
        font={"family": "Arial, Helvetica, sans-serif", "size": 12},
        scene={
            "xaxis_title": "X (mm)",
            "yaxis_title": "Y (mm)",
            "zaxis_title": "Z (mm)",
            "aspectmode": "data",
        },
        margin={"l": 0, "r": 0, "b": 0, "t": 50},
        legend={"orientation": "h", "y": 0.99, "x": 0.01},
    )
    interactive_path = figures_dir / "u3_c1_particle_3d_interactive.html"
    if interactive_path.exists():
        raise FileExistsError(interactive_path)
    interactive.write_html(
        str(interactive_path), include_plotlyjs=True, full_html=True, auto_open=False
    )
    os.chmod(interactive_path, 0o640)
    products.append(interactive_path)

    # Programmatic publication checks for final static exports.
    compliance: dict[str, Any] = {}
    target_sizes = {
        "u3_c1_particle_vertical_sections": (5.6, 4.2),
        "u3_c1_particle_top_view": (4.4, 4.4),
        "u3_c1_particle_3d_context": (5.2, 4.8),
    }
    for path in products:
        if path.suffix.lower() not in {".pdf", ".svg", ".png"} or path.name.startswith("_preview"):
            continue
        target = next((size for stem, size in target_sizes.items() if path.name.startswith(stem)), None)
        issues, info = skill["check_figure"](
            str(path), min_dpi=300, target_inches=target
        )
        # The partial directory is atomically renamed at publication. Keep QA
        # paths package-relative so they remain valid locally and after handoff.
        info = _portable_figure_info(info, path, output)
        compliance[path.name] = {"issues": issues, "info": info}
        if any(severity == "FAIL" for severity, _ in issues):
            raise RuntimeError(f"figure compliance failed for {path}: {issues}")
    qa["compliance"] = compliance
    _write_json(reports_dir / "figure_programmatic_qa.json", qa)
    products.append(reports_dir / "figure_programmatic_qa.json")
    return products, qa


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _harden_products(output_root: Path, products: list[Path]) -> None:
    """Freeze derived products before their metadata enters the manifest."""

    unique_products = sorted(set(products))
    for product in unique_products:
        try:
            product.relative_to(output_root)
        except ValueError as exc:
            raise RuntimeError(f"derived product escapes output root: {product}") from exc
        metadata = product.lstat()
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise RuntimeError(
                f"derived product is not a regular single-link file: {product}"
            )
        os.chmod(product, 0o640)
    for directory in (output_root / "data", output_root / "figures", output_root / "reports"):
        metadata = directory.lstat()
        if not stat.S_ISDIR(metadata.st_mode) or metadata.st_nlink < 2:
            raise RuntimeError(f"derived output directory is invalid: {directory}")
        os.chmod(directory, 0o750)
    os.chmod(output_root, 0o750)


def _source_record(source: SecureFile) -> dict[str, Any]:
    return {
        "path": str(source.path),
        "basename": source.path.name,
        "sha256": source.sha256,
        "size_bytes": source.size_bytes,
        "mode_octal": f"{source.mode:04o}",
        "uid": source.uid,
        "gid": source.gid,
    }


def _default_skill_scripts() -> Path:
    configured = os.environ.get("SCIPILOT_FIGURE_SKILL_SCRIPTS")
    if configured:
        return Path(configured)
    return Path.home() / (
        ".local/share/codex-gpt-pro/codex-home/skills/"
        "scipilot-figure-skill/scripts"
    )


def run(args: argparse.Namespace) -> Path:
    final_output = Path(args.output_dir)
    if final_output.exists() or final_output.is_symlink():
        raise FileExistsError(f"refusing to overwrite output: {final_output}")
    parent = final_output.parent.resolve(strict=True)
    if not parent.is_dir():
        raise NotADirectoryError(parent)
    partial = parent / f"{final_output.name}.partial.{os.getpid()}"
    os.mkdir(partial, 0o700)
    for name in ("data", "figures", "reports"):
        os.mkdir(partial / name, 0o700)

    bi4 = read_regular_file(args.bi4, expected_sha256=args.expected_bi4_sha256)
    xml_source = read_regular_file(
        args.xml, expected_sha256=args.expected_xml_sha256, max_bytes=MAX_XML_BYTES
    )
    out_source = read_regular_file(
        args.out, expected_sha256=args.expected_out_sha256, max_bytes=MAX_OUT_BYTES
    )
    root = parse_jpartdata_bi4(bi4.data)
    particles = extract_u3_particles(root)
    xml = parse_case_xml(xml_source)
    out = parse_gencase_out(out_source)
    cross = validate_cross_sources(root.values, particles, xml, out)

    csv_path = partial / "data" / "particles.csv"
    write_particles_csv(csv_path, particles)
    profile = {
        "format_version": FORMAT_VERSION,
        "status_label": STATUS_LABEL,
        "case_id": args.case_id,
        "bi4_root": root.name,
        "bi4_part": particles["part_name"],
        "particle_count": particles["particle_count"],
        "counts": particles["counts"],
        "limits_m": particles["limits_m"],
        "dp_m": float(root.values["Dp"]),
        "rhop0_kg_m3": float(root.values["Rhop0"]),
        "density_range_kg_m3": [
            min(particles["densities_kg_m3"]),
            max(particles["densities_kg_m3"]),
        ],
        "velocity_component_range_m_s": [
            min(value for row in particles["velocities_m_s"] for value in row),
            max(value for row in particles["velocities_m_s"] for value in row),
        ],
        "arrays": particles["array_metadata"],
        "xml": xml,
        "out": out,
        "cross_validation": cross,
    }
    _write_json(partial / "reports" / "case_profile.json", profile)

    skill = _load_skill_modules(Path(args.skill_scripts))
    eda = skill["profile_data"](str(csv_path), group_cols=["particle_class"])
    eda["source"] = "data/particles.csv (derived read-only from pinned BI4)"
    _write_json(partial / "reports" / "scipilot_eda.json", eda)
    _write_text(partial / "reports" / "scipilot_eda.md", skill["render_report"](eda) + "\n")
    decision = {
        "argument": "Verify that the fresh U3 GenCase output contains the intended static vessel boundary and fluid fill before solver admission.",
        "data_profile": {
            "rows": particles["particle_count"],
            "classes": particles["counts"],
            "coordinate_limits_m": particles["limits_m"],
            "all_velocities_zero": cross["checks"]["static_velocity_zero"],
            "density_range_kg_m3": [
                min(particles["densities_kg_m3"]),
                max(particles["densities_kg_m3"]),
            ],
            "density_within_xml_output_limits": cross["checks"][
                "density_within_xml_output_limits"
            ],
        },
        "primary_chart": "two split equal-scale figures: vertical X-Z/Y-Z sections and an independent X-Y top projection",
        "primary_reason": "Projections preserve metric geometry and expose the bottom, wall, fill height and circular footprint without perspective distortion.",
        "secondary_chart": "static and self-contained interactive 3-D point clouds",
        "secondary_reason": "Useful for spatial orientation and hover inspection, but explicitly not used for metric validation.",
        "encoding": "Okabe-Ito-compatible dark gray boundary and blue fluid, with x/circle marker redundancy and grayscale exports.",
        "rejected_as_primary": [
            "3-D-only validation because perspective distorts distances",
            "density heatmap because density is constant in this initial static case",
            "mean/bar summaries because the objective is geometry, not group statistics",
        ],
    }
    _write_json(partial / "reports" / "visualization_decision.json", decision)

    figure_products, _ = render_figures(partial, particles, root.values, skill)
    products = [
        csv_path,
        partial / "reports" / "case_profile.json",
        partial / "reports" / "scipilot_eda.json",
        partial / "reports" / "scipilot_eda.md",
        partial / "reports" / "visualization_decision.json",
        *figure_products,
    ]
    readme = f"""# U3 C1 GenCase v6 visualization\n\nStatus: `{STATUS_LABEL}`\n\n- Case: `{args.case_id}`\n- Source BI4 SHA-256: `{bi4.sha256}`\n- Particles: `{particles['particle_count']}` (fixed boundary `{particles['counts']['fixed_boundary']}`, fluid `{particles['counts']['fluid']}`)\n- Primary vertical validation: `figures/u3_c1_particle_vertical_sections.*`\n- Primary top validation: `figures/u3_c1_particle_top_view.*`\n- Interactive context: `figures/u3_c1_particle_3d_interactive.html`\n- Parsed data: `data/particles.csv`\n- Cross-source report: `reports/case_profile.json`\n\nThe 3-D views are context only. Use the equal-scale orthographic projections for geometry validation. This case is a development smoke geometry and is not a measurement-validated physical vessel model.\n"""
    _write_text(partial / "README.md", readme)
    products.append(partial / "README.md")

    # Modes are part of the transfer manifest, so harden first and observe the
    # final metadata rather than recording the libraries' creation modes.
    _harden_products(partial, products)

    manifest_products = {}
    for product in sorted(set(products)):
        relative = product.relative_to(partial).as_posix()
        metadata = product.stat()
        manifest_products[relative] = {
            "sha256": _sha256_file(product),
            "size_bytes": metadata.st_size,
            "mode_octal": f"{stat.S_IMODE(metadata.st_mode):04o}",
        }
    manifest = {
        "format_version": FORMAT_VERSION,
        "status": "PASS_U3_C1_GENCASE_V6_CASE_DATA_AND_VISUALIZATION_EXPORT",
        "status_label": STATUS_LABEL,
        "case_id": args.case_id,
        "generated_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "sources": {
            "bi4": _source_record(bi4),
            "xml": _source_record(xml_source),
            "out": _source_record(out_source),
        },
        "cross_validation": cross,
        "products": manifest_products,
        "safety": {
            "source_open_mode": "read-only, O_NOFOLLOW where supported",
            "source_case_modified": False,
            "existing_output_overwritten": False,
            "external_dualsphysics_binary_executed": False,
            "sudo_used": False,
            "system_configuration_changed": False,
        },
    }
    _write_json(partial / "artifact_manifest.json", manifest)
    os.chmod(partial / "artifact_manifest.json", 0o640)
    if final_output.exists() or final_output.is_symlink():
        raise FileExistsError(f"output appeared before finalization: {final_output}")
    os.rename(partial, final_output)
    return final_output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case-id", required=True)
    parser.add_argument("--bi4", required=True)
    parser.add_argument("--xml", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--expected-bi4-sha256", required=True)
    parser.add_argument("--expected-xml-sha256", required=True)
    parser.add_argument("--expected-out-sha256", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--skill-scripts", default=str(_default_skill_scripts()))
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        output = run(args)
    except Exception as exc:
        print(f"FAIL_U3_CASE_VISUALIZATION: {exc}", file=sys.stderr)
        return 2
    print(
        json.dumps(
            {
                "status": "PASS_U3_C1_GENCASE_V6_CASE_DATA_AND_VISUALIZATION_EXPORT",
                "output_dir": str(output),
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
