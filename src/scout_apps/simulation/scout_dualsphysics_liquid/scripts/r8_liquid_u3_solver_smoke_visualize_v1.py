#!/usr/bin/env python3
"""Create a read-only diagnostic package for the U3 C1M solver smoke.

The tool re-runs the frozen offline QC, pins both lifecycle receipts and the
GenCase inputs, and refuses to overwrite an existing destination.  It does
not execute DualSPHysics, use sudo, or change any source/output-tree byte.

The figure deliberately supports only this conclusion: the one-second smoke
has complete, structurally valid data, but settling has not been established.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import stat
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import r8_liquid_bi4_reader_v1 as bi4
import r8_liquid_u3_solver_output_qc_v3 as qc


FORMAT_VERSION = "r8-liquid-u3-solver-smoke-visualization-v1"
DOCUMENT_TYPE = "SMPCC_R8_LIQUID_U3_SOLVER_SMOKE_VISUALIZATION_V1"
STATUS = "PASS_U3_C1M_SOLVER_SMOKE_VISUALIZED_NOT_SETTLED"
STATUS_LABEL = "SMOKE_COMPLETE_SETTLING_NOT_ESTABLISHED"
MAX_RECEIPT_BYTES = 2 * 1024 * 1024

EXPECTED_EXECUTION_STATUS = (
    "PASS_U3_C1M_CPU_SOLVER_V3_SMOKE_EXPORTED_CLOSED_ZERO_UNEXPECTED_"
    "LOGGED_DENIALS_TWO_KNOWN_QUIET_DENY_RULES_CLEANUP_PENDING"
)
EXPECTED_LIFECYCLE_STATUS = (
    "PASS_U3_C1M_CPU_SOLVER_V3_LIFECYCLE_CLEANUP_SMOKE_EXPORT_CLOSED_"
    "ZERO_UNEXPECTED_LOGGED_DENIALS_TWO_KNOWN_QUIET_DENY_RULES"
)


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
        _json_clean(value),
        ensure_ascii=False,
        indent=2,
        sort_keys=True,
        allow_nan=False,
    ).encode("utf-8") + b"\n"
    _write_bytes(path, payload)


def _write_text(path: Path, value: str) -> None:
    _write_bytes(path, value.encode("utf-8"))


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _source_record(source: bi4.SecureFile) -> dict[str, Any]:
    return {
        "path": str(source.path),
        "basename": source.path.name,
        "sha256": source.sha256,
        "size_bytes": source.size_bytes,
        "mode_octal": f"{source.mode:04o}",
        "uid": source.uid,
        "gid": source.gid,
    }


def _read_receipt(
    path: str | os.PathLike[str], expected_sha256: str, expected_status: str, case_id: str
) -> tuple[bi4.SecureFile, dict[str, Any]]:
    source = bi4.read_regular_file(
        path, expected_sha256=expected_sha256, max_bytes=MAX_RECEIPT_BYTES
    )
    try:
        value = json.loads(source.data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid pinned receipt {source.path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"receipt is not a JSON object: {source.path}")
    if value.get("case_id") != case_id or value.get("status") != expected_status:
        raise ValueError(f"receipt identity/status differs: {source.path}")
    if value.get("production_authorized") is not False:
        raise ValueError(f"receipt unexpectedly authorizes production: {source.path}")
    if value.get("settled_state_authorized") is not False:
        raise ValueError(f"receipt unexpectedly authorizes settled state: {source.path}")
    return source, value


def validate_receipts(args: argparse.Namespace) -> dict[str, Any]:
    execution_source, execution = _read_receipt(
        args.execution_receipt,
        args.expected_execution_receipt_sha256,
        EXPECTED_EXECUTION_STATUS,
        args.case_id,
    )
    lifecycle_source, lifecycle = _read_receipt(
        args.lifecycle_receipt,
        args.expected_lifecycle_receipt_sha256,
        EXPECTED_LIFECYCLE_STATUS,
        args.case_id,
    )
    if lifecycle.get("sysctls", {}).get("unchanged") is not True:
        raise ValueError("lifecycle receipt does not prove unchanged sysctl bytes")
    if lifecycle.get("next_allowed_stage") != (
        "SEPARATE_OUTPUT_QC_THEN_FRESH_LONGER_SETTLE_IDENTITIES"
    ):
        raise ValueError("lifecycle next-stage boundary differs")
    presence = lifecycle.get("profiles_after", {}).get("aa_status_exact_presence", {})
    if not presence or any(value is not False for value in presence.values()):
        raise ValueError("lifecycle receipt does not prove both profiles absent")
    if execution.get("export_verification", {}).get("file_count") != 30:
        raise ValueError("execution receipt output inventory is not exactly 30 files")
    return {
        "execution": _source_record(execution_source),
        "lifecycle": _source_record(lifecycle_source),
        "checks": {
            "case_id_exact": True,
            "statuses_exact": True,
            "production_authorized_false": True,
            "settled_state_authorized_false": True,
            "sysctls_unchanged": True,
            "apparmor_profiles_absent_after": True,
            "export_file_count_30": True,
        },
    }


def validate_smoke_report(report: dict[str, Any]) -> None:
    run = report.get("run", {})
    verdict = report.get("verdict", {})
    required = {
        "qc_status": verdict.get("status") == "C1M_ZERO_MOTION_SMOKE_PASS",
        "structural_pass": run.get("structural_pass") is True,
        "short_duration_smoke": run.get("short_duration_smoke") is True,
        "duration_not_eligible": run.get("duration_eligible_for_settle_qc") is False,
        "tail_not_passed": run.get("tail_pass") is False,
        "numeric_settle_not_passed": run.get("numeric_settle_qc_pass") is False,
        "settled_claim_forbidden": verdict.get("settled_state_claim_allowed") is False,
        "settled_freeze_forbidden": verdict.get("settled_state_freeze_eligible") is False,
    }
    failed = [name for name, passed in required.items() if not passed]
    if failed:
        raise ValueError(
            "input is not the admitted complete-but-not-settled smoke: "
            + ", ".join(failed)
        )


def build_metric_rows(report: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for part in report["run"]["parts"]:
        surface = part["surface_proxy"]
        heights = surface["sector_heights_m"]
        if not surface["valid"] or not heights:
            raise ValueError(f"invalid surface proxy in {part['name']}")
        row = {
            "part": part["cpart"],
            "time_s": part["time_s"],
            "step": part["step"],
            "speed_rms_m_s": part["speed"]["rms_m_s"],
            "speed_p95_m_s": part["speed"]["p95_m_s"],
            "speed_max_m_s": part["speed"]["max_m_s"],
            "specific_kinetic_energy_j_kg": part["speed"][
                "specific_kinetic_energy_j_kg"
            ],
            "surface_proxy_median_m": surface["median_height_m"],
            "surface_proxy_sector_min_m": min(heights),
            "surface_proxy_sector_max_m": max(heights),
            "surface_proxy_spread_m": surface["spatial_spread_m"],
            "density_min_kg_m3": part["density"]["min_kg_m3"],
            "density_p01_kg_m3": part["density"]["p01_kg_m3"],
            "density_mean_kg_m3": part["density"]["mean_kg_m3"],
            "density_p99_kg_m3": part["density"]["p99_kg_m3"],
            "density_max_kg_m3": part["density"]["max_kg_m3"],
        }
        if any(
            not math.isfinite(float(value))
            for key, value in row.items()
            if key not in {"part", "step"}
        ):
            raise ValueError(f"non-finite derived metric in {part['name']}")
        rows.append(row)
    times = [float(row["time_s"]) for row in rows]
    if not rows or any(right <= left for left, right in zip(times, times[1:])):
        raise ValueError("derived metric times are not strictly increasing")
    return rows


def write_metrics_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0)
    fd = os.open(path, flags, 0o640)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            for row in rows:
                writer.writerow(
                    {
                        key: (format(value, ".17g") if isinstance(value, float) else value)
                        for key, value in row.items()
                    }
                )
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        raise


def classify_final_particles(
    ids: list[int], positions: list[Any], moving_case: dict[str, Any]
) -> list[str]:
    """Validate a solver-reordered ID array and classify each aligned position."""

    particle_count = int(moving_case["particle_count"])
    counts = moving_case["counts"]
    if len(ids) != particle_count or len(positions) != len(ids):
        raise ValueError("final PART particle arrays have unexpected length")
    if len(set(ids)) != len(ids) or set(ids) != set(range(particle_count)):
        raise ValueError("final PART particle IDs are not the complete unique case range")
    fixed_end = int(counts["fixed_boundary"])
    moving_end = fixed_end + int(counts["moving_boundary"])
    floating_end = moving_end + int(counts["floating"])
    classes = []
    for particle_id in ids:
        if particle_id < fixed_end:
            classes.append("fixed_boundary")
        elif particle_id < moving_end:
            classes.append("moving_boundary")
        elif particle_id < floating_end:
            classes.append("floating")
        else:
            classes.append("fluid")
    for name in ("fixed_boundary", "moving_boundary", "floating", "fluid"):
        if classes.count(name) != int(counts[name]):
            raise ValueError(f"final PART {name} classification differs")
    return classes


def load_final_particles(
    run_dir: str | os.PathLike[str], report: dict[str, Any]
) -> dict[str, Any]:
    run = report["run"]
    final = run["parts"][-1]
    path = Path(run_dir) / "data" / final["name"]
    source = bi4.read_regular_file(
        path, expected_sha256=final["sha256"], max_bytes=qc.MAX_PART_BYTES
    )
    root = bi4.parse_jpartdata_bi4(source.data)
    if len(root.items) != 1:
        raise ValueError("final PART does not contain exactly one particle item")
    part = root.items[0]
    expected_arrays = {"Idp": 8, "Posd": 23}
    for name, type_code in expected_arrays.items():
        array = part.arrays.get(name)
        if array is None or array.type_code != type_code:
            raise ValueError(f"final PART array {name} is missing or has wrong type")
    ids = list(part.arrays["Idp"].records())
    positions = list(part.arrays["Posd"].records())
    classes = classify_final_particles(ids, positions, report["moving_case"])
    return {
        "source": _source_record(source),
        "ids": ids,
        "positions_m": positions,
        "classes": classes,
        "time_s": final["time_s"],
    }


def _default_skill_scripts() -> Path:
    configured = os.environ.get("SCIPILOT_FIGURE_SKILL_SCRIPTS")
    if configured:
        return Path(configured)
    return Path.home() / (
        ".local/share/codex-gpt-pro/codex-home/skills/"
        "scipilot-figure-skill/scripts"
    )


def _load_skill_modules(skill_scripts: Path) -> dict[str, Any]:
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
    from layout_tools import add_panel_labels
    from profile_data import profile_data, render_report
    from setup_style import setup_style
    from visual_qa import audit_layout, render_preview

    return {
        "setup_style": setup_style,
        "add_panel_labels": add_panel_labels,
        "audit_layout": audit_layout,
        "render_preview": render_preview,
        "export_figure": export_figure,
        "check_figure": check_figure,
        "profile_data": profile_data,
        "render_report": render_report,
    }


def create_dashboard(
    report: dict[str, Any],
    rows: list[dict[str, Any]],
    particles: dict[str, Any],
    skill: dict[str, Any],
):
    import matplotlib.pyplot as plt
    import numpy as np

    style = skill["setup_style"](
        journal="general", lang="en", use_sciplots=False, constrained_layout=False
    )
    blue = "#0072B2"
    orange = "#E69F00"
    green = "#009E73"
    vermillion = "#D55E00"
    gray = "#666666"

    fig, axes_grid = plt.subplots(2, 2, figsize=(7.2, 6.0), constrained_layout=False)
    axes = list(axes_grid.flat)
    fig.suptitle(
        "U3 C1M CPU smoke: complete output, settling not established",
        fontsize=10,
        fontweight="bold",
        y=0.975,
    )

    coords = np.asarray(particles["positions_m"], dtype=float) * 1000.0
    classes = np.asarray(particles["classes"], dtype=object)
    dp_mm = float(report["moving_case"]["dp_m"]) * 1000.0
    half_slice_mm = 0.51 * dp_mm
    central = np.abs(coords[:, 1]) <= half_slice_mm
    ax = axes[0]
    for name, label, color, marker, size, alpha in (
        ("fluid", "Fluid", blue, "o", 7.0, 0.52),
        ("moving_boundary", "Moving boundary", orange, "s", 10.0, 0.78),
    ):
        mask = central & (classes == name)
        ax.scatter(
            coords[mask, 0],
            coords[mask, 2],
            s=size,
            marker=marker,
            c=color,
            alpha=alpha,
            linewidths=0.15 if name == "moving_boundary" else 0.0,
            edgecolors="#7A4C00" if name == "moving_boundary" else "none",
            label=(
                f"Fluid (n={int(mask.sum()):,})"
                if name == "fluid"
                else f"Moving wall (n={int(mask.sum()):,})"
            ),
        )
    ax.set_title("Final zero-motion central slice", loc="left")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Z (mm)")
    ax.set_aspect("equal", adjustable="box")
    slice_coords = coords[central]
    ax.set_xlim(slice_coords[:, 0].min() - dp_mm, slice_coords[:, 0].max() + dp_mm)
    ax.set_ylim(slice_coords[:, 2].min() - dp_mm, slice_coords[:, 2].max() + dp_mm)
    ax.legend(
        frameon=False,
        loc="upper left",
        bbox_to_anchor=(1.02, 1.0),
        borderaxespad=0.0,
        fontsize=5.5,
    )
    ax.text(
        1.03,
        0.78,
        f"t = {particles['time_s']:.3f} s\n|Y| <= {half_slice_mm:.2f} mm",
        transform=ax.transAxes,
        fontsize=5.5,
        va="top",
        color=gray,
    )

    time = np.asarray([row["time_s"] for row in rows], dtype=float)
    ax = axes[1]
    speed_specs = (
        ("speed_rms_m_s", "RMS", blue, "-", "o"),
        ("speed_p95_m_s", "P95", orange, "--", "s"),
        ("speed_max_m_s", "Maximum", vermillion, "-.", "^"),
    )
    for key, label, color, linestyle, marker in speed_specs:
        ax.plot(
            time,
            [row[key] for row in rows],
            label=label,
            color=color,
            linestyle=linestyle,
            marker=marker,
            markersize=2.5,
            linewidth=1.0,
            markevery=2,
        )
    thresholds = report["thresholds"]
    ax.axhline(
        thresholds["max_tail_speed_rms_m_s"], color=gray, linestyle=":", linewidth=0.7
    )
    ax.axhline(thresholds["max_tail_speed_m_s"], color=gray, linestyle=":", linewidth=0.7)
    ax.text(
        0.98,
        thresholds["max_tail_speed_rms_m_s"],
        "RMS/P95 limit ",
        color=gray,
        fontsize=5.5,
        va="bottom",
        ha="right",
    )
    ax.text(
        0.98,
        thresholds["max_tail_speed_m_s"],
        "max limit ",
        color=gray,
        fontsize=5.5,
        va="bottom",
        ha="right",
    )
    ax.set_title("Fluid speed diagnostics", loc="left")
    ax.set_xlabel("Physical time (s)")
    ax.set_ylabel("Speed (m/s)")
    ax.set_xlim(0.0, max(time))
    ax.set_ylim(bottom=0.0)
    ax.legend(frameon=False, loc="upper left", ncol=1, fontsize=6)

    ax = axes[2]
    surface_mid = np.asarray([row["surface_proxy_median_m"] for row in rows]) * 1000.0
    surface_min = np.asarray([row["surface_proxy_sector_min_m"] for row in rows]) * 1000.0
    surface_max = np.asarray([row["surface_proxy_sector_max_m"] for row in rows]) * 1000.0
    hswl_mm = float(report["moving_case"]["xml"]["hswl_m"]) * 1000.0
    ax.fill_between(
        time,
        surface_min,
        surface_max,
        color=blue,
        alpha=0.18,
        linewidth=0,
        label="Sector min-max",
    )
    ax.plot(
        time,
        surface_mid,
        color=blue,
        linestyle="-",
        marker="o",
        markersize=2.5,
        markevery=2,
        linewidth=1.0,
        label="Median sector q99",
    )
    ax.axhline(hswl_mm, color=gray, linestyle="--", linewidth=0.8, label="Case HSWL")
    ax.set_title("Surface-height proxy (development only)", loc="left")
    ax.set_xlabel("Physical time (s)")
    ax.set_ylabel("Height (mm)")
    ax.set_xlim(0.0, max(time))
    margin = max(0.05, (surface_max.max() - surface_min.min()) * 0.15)
    ax.set_ylim(min(surface_min.min(), hswl_mm) - margin, max(surface_max.max(), hswl_mm) + margin)
    ax.legend(frameon=False, loc="upper left", fontsize=6)
    ax.text(
        0.98,
        0.03,
        "Particle q99 proxy; not a SWL gauge",
        transform=ax.transAxes,
        fontsize=5.5,
        ha="right",
        va="bottom",
        color=gray,
    )

    ax = axes[3]
    density_p01 = np.asarray([row["density_p01_kg_m3"] for row in rows])
    density_p99 = np.asarray([row["density_p99_kg_m3"] for row in rows])
    density_mean = np.asarray([row["density_mean_kg_m3"] for row in rows])
    rhop0 = float(report["moving_case"]["xml"]["rhop0_kg_m3"])
    ax.fill_between(
        time,
        density_p01,
        density_p99,
        color=green,
        alpha=0.18,
        linewidth=0,
        label="P01-P99 particles",
    )
    ax.plot(
        time,
        density_mean,
        color=green,
        linestyle="-",
        marker="D",
        markersize=2.3,
        markevery=2,
        linewidth=1.0,
        label="Mean",
    )
    ax.axhline(rhop0, color=gray, linestyle="--", linewidth=0.8, label="Reference density")
    ax.set_title("Fluid density (diagnostic scale)", loc="left")
    ax.set_xlabel("Physical time (s)")
    ax.set_ylabel(r"Density (kg m$^{-3}$)")
    ax.set_xlim(0.0, max(time))
    density_margin = max(0.05, (density_p99.max() - density_p01.min()) * 0.12)
    ax.set_ylim(
        min(density_p01.min(), rhop0) - density_margin,
        max(density_p99.max(), rhop0) + density_margin,
    )
    ax.legend(frameon=False, loc="upper right", fontsize=6)

    for axis in axes:
        axis.grid(True, color="#D9D9D9", linewidth=0.35, linestyle=":", zorder=0)
        axis.minorticks_off()
        axis.tick_params(top=False, right=False, pad=2)

    final_time = float(report["run"]["parts"][-1]["time_s"])
    minimum_time = float(report["thresholds"]["minimum_settle_time_s"])
    fig.text(
        0.5,
        0.018,
        (
            "Structural QC PASS; "
            f"duration {final_time:.3f} s < {minimum_time:.3f} s eligibility; "
            "tail criteria not passed."
        ),
        ha="center",
        va="bottom",
        fontsize=7,
        fontweight="bold",
        color=vermillion,
    )
    fig.subplots_adjust(left=0.095, right=0.98, bottom=0.105, top=0.90, hspace=0.36, wspace=0.29)
    skill["add_panel_labels"](
        fig,
        axes=axes,
        style="nature",
        x_offset_pt=-9,
        y_offset_pt=4,
        ha="right",
        va="bottom",
    )
    fig.canvas.draw()
    return fig, {"style": style, "size_inches": [7.2, 6.0]}


def _portable_info(info: dict[str, Any], path: Path, root: Path) -> dict[str, Any]:
    result = dict(info)
    result["path"] = path.relative_to(root).as_posix()
    return result


def audit_pdf_font_embedding(path: Path) -> dict[str, Any]:
    """Resolve Type0/CID descendants that the generic checker cannot inspect."""

    from pypdf import PdfReader

    reader = PdfReader(str(path))
    fonts: dict[str, dict[str, Any]] = {}
    for page in reader.pages:
        resources = page.get("/Resources")
        resources = resources.get_object() if resources is not None else None
        font_map = resources.get("/Font") if resources is not None else None
        font_map = font_map.get_object() if font_map is not None else {}
        for _resource_name, reference in font_map.items():
            parent = reference.get_object()
            descendants = parent.get("/DescendantFonts") or []
            targets = [item.get_object() for item in descendants] or [parent]
            for target in targets:
                base_font = str(target.get("/BaseFont", parent.get("/BaseFont", "?")))
                descriptor_ref = target.get("/FontDescriptor") or parent.get(
                    "/FontDescriptor"
                )
                descriptor = (
                    descriptor_ref.get_object() if descriptor_ref is not None else None
                )
                embedded_files = [
                    name
                    for name in ("/FontFile", "/FontFile2", "/FontFile3")
                    if descriptor is not None and name in descriptor
                ]
                fonts[base_font] = {
                    "base_font": base_font,
                    "parent_subtype": str(parent.get("/Subtype", "?")),
                    "descendant_subtype": str(target.get("/Subtype", "?")),
                    "embedded": bool(embedded_files),
                    "embedded_files": embedded_files,
                    "subset_name": "+" in base_font,
                    "unicode_map_present": "/ToUnicode" in parent,
                    "type3": (
                        str(parent.get("/Subtype", "")) == "/Type3"
                        or str(target.get("/Subtype", "")) == "/Type3"
                    ),
                }
    records = [fonts[name] for name in sorted(fonts)]
    passed = bool(records) and all(
        record["embedded"]
        and record["subset_name"]
        and record["unicode_map_present"]
        and not record["type3"]
        for record in records
    )
    return {
        "status": "PASS_CID_TRUETYPE_FONTS_EMBEDDED_SUBSET_UNICODE"
        if passed
        else "FAIL_PDF_FONT_EMBEDDING_AUDIT",
        "pass": passed,
        "method": (
            "pypdf page resource traversal through Type0 DescendantFonts and "
            "FontDescriptor FontFile entries"
        ),
        "fonts": records,
    }


def resolve_compliance_issues(
    compliance: dict[str, Any], pdf_font_audit: dict[str, Any]
) -> dict[str, Any]:
    resolved: list[dict[str, Any]] = []
    unresolved: list[dict[str, Any]] = []
    for filename, record in compliance.items():
        for severity, message in record["issues"]:
            issue = {"file": filename, "severity": severity, "message": message}
            if (
                filename.endswith(".pdf")
                and severity == "WARN"
                and "可能未嵌入" in message
                and pdf_font_audit.get("pass") is True
            ):
                issue["resolution"] = pdf_font_audit["status"]
                resolved.append(issue)
            elif severity in {"WARN", "FAIL"}:
                unresolved.append(issue)
    return {"resolved": resolved, "unresolved": unresolved}


def render_preview_only(args: argparse.Namespace) -> Path:
    preview = Path(args.preview_file)
    if preview.exists() or preview.is_symlink():
        raise FileExistsError(f"refusing to overwrite preview: {preview}")
    preview.parent.resolve(strict=True)
    report = qc.build_report(
        args.run_dir,
        args.case_bi4,
        args.case_xml,
        expected_case_bi4_sha256=args.expected_case_bi4_sha256,
        expected_case_xml_sha256=args.expected_case_xml_sha256,
    )
    validate_smoke_report(report)
    validate_receipts(args)
    rows = build_metric_rows(report)
    particles = load_final_particles(args.run_dir, report)
    skill = _load_skill_modules(Path(args.skill_scripts))
    fig, _ = create_dashboard(report, rows, particles, skill)
    try:
        skill["render_preview"](fig, str(preview), dpi=150)
        issues = skill["audit_layout"](fig)
    finally:
        import matplotlib.pyplot as plt

        plt.close(fig)
    print(json.dumps({"preview": str(preview), "issues": issues}, ensure_ascii=False))
    if any(severity == "FAIL" for severity, _message in issues):
        raise RuntimeError(f"preview layout QA failed: {issues}")
    return preview


def _harden_products(output_root: Path, products: list[Path]) -> None:
    for product in sorted(set(products)):
        try:
            product.relative_to(output_root)
        except ValueError as exc:
            raise RuntimeError(f"derived product escapes output root: {product}") from exc
        metadata = product.lstat()
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise RuntimeError(f"derived product is not a regular single-link file: {product}")
        os.chmod(product, 0o640)
    for name in ("data", "figures", "reports"):
        directory = output_root / name
        if not stat.S_ISDIR(directory.lstat().st_mode):
            raise RuntimeError(f"derived output directory is invalid: {directory}")
        os.chmod(directory, 0o750)
    os.chmod(output_root, 0o750)


def run_export(args: argparse.Namespace) -> Path:
    if not args.external_preview_reviewed:
        raise ValueError(
            "final export requires --external-preview-reviewed after PNG image inspection"
        )
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

    receipts = validate_receipts(args)
    report = qc.build_report(
        args.run_dir,
        args.case_bi4,
        args.case_xml,
        expected_case_bi4_sha256=args.expected_case_bi4_sha256,
        expected_case_xml_sha256=args.expected_case_xml_sha256,
    )
    validate_smoke_report(report)
    rows = build_metric_rows(report)
    particles = load_final_particles(args.run_dir, report)
    skill = _load_skill_modules(Path(args.skill_scripts))

    metrics_path = partial / "data" / "solver_smoke_metrics.csv"
    write_metrics_csv(metrics_path, rows)
    qc_path = partial / "reports" / "solver_output_qc_v3.json"
    _write_json(qc_path, report)
    receipt_path = partial / "reports" / "receipt_validation.json"
    _write_json(receipt_path, receipts)

    eda = skill["profile_data"](str(metrics_path))
    eda["source"] = "data/solver_smoke_metrics.csv (derived from pinned BI4 outputs)"
    eda_json = partial / "reports" / "scipilot_eda.json"
    eda_md = partial / "reports" / "scipilot_eda.md"
    _write_json(eda_json, eda)
    _write_text(eda_md, skill["render_report"](eda) + "\n")

    decision = {
        "argument": (
            "The one-second C1M CPU run has complete structurally valid output, "
            "while its duration and tail dynamics do not establish settling."
        ),
        "primary_chart": (
            "four-panel diagnostic dashboard: final central X-Z particle slice, "
            "fluid speed, development-only surface proxy, and fluid density"
        ),
        "reason": (
            "The geometry panel supplies spatial context; three separate time-series "
            "axes expose convergence without a misleading dual Y axis."
        ),
        "encoding": (
            "Okabe-Ito colors plus line-style/marker redundancy; quantile envelopes "
            "are labeled as particle ranges, not uncertainty intervals."
        ),
        "rejected": [
            "steady-state wording because duration_eligible=false and tail_pass=false",
            "dual Y axes because their arbitrary scaling can manufacture agreement",
            "3-D geometry as the primary metric view because perspective distorts distance",
            "mean-only bars because the evidence is temporal, not a grouped mean comparison",
        ],
    }
    decision_path = partial / "reports" / "visualization_decision.json"
    _write_json(decision_path, decision)

    fig, figure_meta = create_dashboard(report, rows, particles, skill)
    figures_dir = partial / "figures"
    preview_path = figures_dir / "_preview_solver_smoke_dashboard.png"
    skill["render_preview"](fig, str(preview_path), dpi=150)
    layout_issues = skill["audit_layout"](fig)
    if any(severity == "FAIL" for severity, _message in layout_issues):
        raise RuntimeError(f"dashboard layout QA failed: {layout_issues}")

    base = figures_dir / "u3_c1m_solver_smoke_dashboard"
    exported = [
        Path(path)
        for path in skill["export_figure"](
            fig,
            str(base),
            formats=["pdf", "svg", "png"],
            size_inches=(7.2, 6.0),
            dpi=300,
            grayscale_preview=False,
            tight=False,
        )
    ]
    from PIL import Image

    grayscale = figures_dir / "u3_c1m_solver_smoke_dashboard_grayscale.png"
    with Image.open(base.with_suffix(".png")) as image:
        image.convert("L").save(grayscale, dpi=(300, 300))
    import matplotlib.pyplot as plt

    plt.close(fig)

    compliance: dict[str, Any] = {}
    figure_products = [preview_path, *exported, grayscale]
    for path in exported + [grayscale]:
        issues, info = skill["check_figure"](
            str(path), min_dpi=300, target_inches=(7.2, 6.0)
        )
        compliance[path.name] = {
            "issues": issues,
            "info": _portable_info(info, path, partial),
        }
        if any(severity == "FAIL" for severity, _message in issues):
            raise RuntimeError(f"figure compliance failed for {path}: {issues}")
    pdf_font_audit = audit_pdf_font_embedding(base.with_suffix(".pdf"))
    if pdf_font_audit["pass"] is not True:
        raise RuntimeError(f"PDF font embedding audit failed: {pdf_font_audit}")
    resolution = resolve_compliance_issues(compliance, pdf_font_audit)
    if resolution["unresolved"]:
        raise RuntimeError(
            f"unresolved figure compliance issues: {resolution['unresolved']}"
        )
    font_audit_path = partial / "reports" / "pdf_font_embedding_audit.json"
    _write_json(font_audit_path, pdf_font_audit)
    qa = {
        "figure": figure_meta,
        "layout_issues": layout_issues,
        "compliance": compliance,
        "compliance_resolution": resolution,
        "external_visual_review": (
            "PASS_PREPUBLICATION_PNG_REVIEW_SAME_RENDER_CODE_AND_PINNED_INPUTS"
        ),
    }
    qa_path = partial / "reports" / "figure_programmatic_qa.json"
    _write_json(qa_path, qa)

    readme = f"""# U3 C1M CPU solver smoke visualization\n\nStatus: `{STATUS_LABEL}`\n\n- Runtime identity: `{args.case_id}`\n- Solver output QC: `C1M_ZERO_MOTION_SMOKE_PASS`\n- Structural QC: PASS\n- Settle duration eligible: false\n- Tail QC pass: false\n- Numeric settle QC pass: false\n- Main dashboard: `figures/u3_c1m_solver_smoke_dashboard.png`\n- Vector versions: `.pdf` and `.svg` with the same basename\n- Grayscale check: `figures/u3_c1m_solver_smoke_dashboard_grayscale.png`\n- Derived metrics: `data/solver_smoke_metrics.csv`\n- Full QC: `reports/solver_output_qc_v3.json`\n\nThe surface series is a sector-wise particle q99 development proxy, not a DualSPHysics SWL gauge and not a physical liquid-level measurement.  This package does not authorize settled-state claims, U4, production, or physical replay.\n"""
    readme_path = partial / "README.md"
    _write_text(readme_path, readme)

    products = [
        metrics_path,
        qc_path,
        receipt_path,
        eda_json,
        eda_md,
        decision_path,
        qa_path,
        font_audit_path,
        readme_path,
        *figure_products,
    ]
    _harden_products(partial, products)
    manifest_products: dict[str, Any] = {}
    for product in sorted(set(products)):
        metadata = product.stat()
        manifest_products[product.relative_to(partial).as_posix()] = {
            "sha256": _sha256_file(product),
            "size_bytes": metadata.st_size,
            "mode_octal": f"{stat.S_IMODE(metadata.st_mode):04o}",
        }
    manifest = {
        "format_version": FORMAT_VERSION,
        "document_type": DOCUMENT_TYPE,
        "status": STATUS,
        "status_label": STATUS_LABEL,
        "case_id": args.case_id,
        "generated_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "sources": {
            "run_dir": str(Path(args.run_dir)),
            "case_bi4": report["moving_case"]["bi4"],
            "case_xml": report["moving_case"]["xml"],
            "final_part": particles["source"],
            "receipts": receipts,
        },
        "qc_verdict": report["verdict"],
        "products": manifest_products,
        "safety": {
            "sources_opened_read_only": True,
            "source_case_modified": False,
            "solver_output_modified": False,
            "existing_output_overwritten": False,
            "dualsphysics_executed": False,
            "gpu_used": False,
            "sudo_used": False,
            "system_configuration_changed": False,
        },
    }
    manifest_path = partial / "artifact_manifest.json"
    _write_json(manifest_path, manifest)
    os.chmod(manifest_path, 0o640)
    if final_output.exists() or final_output.is_symlink():
        raise FileExistsError(f"output appeared before publication: {final_output}")
    os.rename(partial, final_output)
    return final_output


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case-id", required=True)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--case-bi4", required=True)
    parser.add_argument("--case-xml", required=True)
    parser.add_argument("--expected-case-bi4-sha256", required=True)
    parser.add_argument("--expected-case-xml-sha256", required=True)
    parser.add_argument("--execution-receipt", required=True)
    parser.add_argument("--lifecycle-receipt", required=True)
    parser.add_argument("--expected-execution-receipt-sha256", required=True)
    parser.add_argument("--expected-lifecycle-receipt-sha256", required=True)
    destination = parser.add_mutually_exclusive_group(required=True)
    destination.add_argument("--preview-file")
    destination.add_argument("--output-dir")
    parser.add_argument(
        "--external-preview-reviewed",
        action="store_true",
        help="required only for final export after the preview PNG has been inspected",
    )
    parser.add_argument("--skill-scripts", default=str(_default_skill_scripts()))
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        if args.preview_file:
            output = render_preview_only(args)
            status = "PASS_U3_C1M_SOLVER_SMOKE_PREVIEW_RENDERED"
        else:
            output = run_export(args)
            status = STATUS
    except Exception as exc:
        print(f"FAIL_U3_SOLVER_SMOKE_VISUALIZATION: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"status": status, "output": str(output)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
