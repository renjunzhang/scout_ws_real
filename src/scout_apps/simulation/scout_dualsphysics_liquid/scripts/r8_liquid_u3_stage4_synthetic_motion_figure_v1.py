#!/usr/bin/env python3
"""Create-new publication-style diagnostics for Stage-4 synthetic motion QC.

Run this script only through scipilot-figure-skill's ``run_python.sh``.  The
preview command performs deterministic layout QA without exporting final
figures.  After reviewing the preview, ``export-assets`` writes PDF/SVG/PNG
and grayscale PNG.  Only after both color and grayscale assets are inspected
may ``finalize-receipt`` create the closed-schema receipt.  All inputs are
frozen read-only evidence; no solver, GPU, network, or Stage-5 action is used.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import stat
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SKILL_ROOT = Path(os.environ.get(
    "SCIPILOT_FIGURE_SKILL_ROOT",
    "/home/zrj/.local/share/codex-gpt-pro/codex-home/skills/scipilot-figure-skill",
)).resolve()
SKILL_SCRIPTS = SKILL_ROOT / "scripts"
sys.path.insert(0, str(SKILL_SCRIPTS))

import matplotlib.colors as mcolors  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from PIL import Image  # noqa: E402

from check_figure import SEVERITY, check_figure  # noqa: E402
from export_figure import export_figure  # noqa: E402
from layout_tools import add_panel_labels, finalize_figure  # noqa: E402
from setup_style import setup_style  # noqa: E402
from visual_qa import audit_layout, print_report as print_layout_report, render_preview  # noqa: E402


SCRIPT_PATH = Path(__file__).resolve()
PACKAGE_ROOT = SCRIPT_PATH.parent.parent
SCHEMA_PATH = PACKAGE_ROOT / "schema/target_host_u3_stage4_synthetic_motion_figure_receipt_v1.json"
TEST_PATH = PACKAGE_ROOT / "tests/test_u3_stage4_synthetic_motion_figure_v1.py"
QC_PATH = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_synthetic_motion_20260811T143346Z_v23.qc_v1.json")
CSV_PATH = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_synthetic_motion_20260811T143346Z_v23.metrics_v1.csv")
BASENAME = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_synthetic_motion_20260811T143346Z_v23.diagnostics_v1")
PREVIEW_PATH = Path(f"{BASENAME}_preview.png")
PNG_PATH = Path(f"{BASENAME}.png")
GRAY_PATH = Path(f"{BASENAME}_grayscale.png")
PDF_PATH = Path(f"{BASENAME}.pdf")
SVG_PATH = Path(f"{BASENAME}.svg")
RECEIPT_PATH = Path("/home/zrj/scout_liquid_lab/audits/u3_c1m_gpu_synthetic_motion_20260811T143346Z_v23.figure_v1.json")
EXPECTED_PREVIEW_SHA256 = "1eaf5c63a5ead8ee62393f97821e2165d467842367f12dda596a766513dc33fe"
FIGURE_SIZE = (7.2, 7.2)
DPI = 300
MAX_BYTES = 268435456
PALETTE = {"zero": "#4D4D4D", "translation": "#0072B2", "yaw": "#D55E00"}
LINESTYLES = {"zero": ":", "translation": "-", "yaw": "--"}
MARKERS = {"zero": "o", "translation": "s", "yaw": "^"}
LABELS = {"zero": "零回放", "translation": "平移", "yaw": "偏航"}

FROZEN_INPUTS = {
    "qc_receipt": (QC_PATH, "1f837ce7c52ce80971adb981e121eace3c92968095193bb6db3519b39776e383"),
    "metrics_csv": (CSV_PATH, "10b8976e8fceff5dbcaa244693afd5ac016734cd10db5696613cb6371183ba25"),
    "qc_schema": (PACKAGE_ROOT / "schema/target_host_u3_stage4_synthetic_motion_qc_v1.json", "0a356742d064e07e7cd05d3e9bafc7019f3966eb42e8de8e5102b48fa077fa3c"),
    "qc_script": (PACKAGE_ROOT / "scripts/r8_liquid_u3_stage4_synthetic_motion_qc_v1.py", "8e33502bca68de1cb2ffe07c7706047edd5805980a205a7233ee4c87c90351e1"),
    "qc_tests": (PACKAGE_ROOT / "tests/test_u3_stage4_synthetic_motion_qc_v1.py", "04a3fed16bf37a708d15d23b2944ebbe28ee767345dd48f73b9a95e68f6dcd74"),
    "setup_style": (SKILL_SCRIPTS / "setup_style.py", "d58ef81684577bb605a92733dd2ca48ade03b4f819b3d255fd62f319be7fea82"),
    "export_figure": (SKILL_SCRIPTS / "export_figure.py", "40ce3bcd31dfc4b33e38a1cb180c75dc730ee6fa046f04934c79f285c6f890c1"),
    "layout_tools": (SKILL_SCRIPTS / "layout_tools.py", "68e1d25774148fe78f5d87035d651f5e3707382502505cb11f62f9ec7ae88e49"),
    "visual_qa": (SKILL_SCRIPTS / "visual_qa.py", "67675709e523c76e2748d50749de7bc6332f07df865379b26d024321fb40120f"),
    "check_figure": (SKILL_SCRIPTS / "check_figure.py", "861191858ff47c5af5247f08d0759b14956a2a7e7d8eafcc6b414d37b9d70f06"),
}

EXPECTED_COLUMNS = [
    "run", "part", "time_s", "relative_time_s", "boundary_signal_name",
    "boundary_observed", "boundary_expected", "boundary_position_error_max_m",
    "fluid_speed_rms_lab_m_s", "fluid_speed_rms_container_m_s",
    "fluid_tangential_speed_rms_container_m_s",
    "specific_kinetic_energy_lab_j_kg", "specific_kinetic_energy_container_j_kg",
    "surface_crest_above_nominal_m", "surface_absolute_deviation_max_m",
    "surface_peak_to_peak_m", "surface_peak_height_m", "surface_p95_height_m",
    "surface_rms_deviation_m", "surface_first_azimuthal_harmonic_m",
    "surface_first_azimuthal_harmonic_phase_rad",
] + [f"surface_sector_{index:02d}_height_m" for index in range(16)]


class FigureEvidenceError(ValueError):
    """A frozen input, chart invariant, QA check, or create-new output failed."""


def _sha256(path: Path, maximum: int = MAX_BYTES) -> str:
    metadata = os.lstat(path)
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1 or not 1 <= metadata.st_size <= maximum:
        raise FigureEvidenceError(f"unsafe figure input/output identity: {path}")
    digest = hashlib.sha256()
    with path.open("rb", buffering=0) as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def identity(path: Path) -> dict[str, Any]:
    digest = _sha256(path)
    metadata = os.lstat(path)
    return {
        "path": str(path),
        "sha256": digest,
        "size_bytes": metadata.st_size,
        "mode": f"{stat.S_IMODE(metadata.st_mode):04o}",
        "inode": metadata.st_ino,
        "nlink": metadata.st_nlink,
    }


def _read_json(path: Path) -> dict[str, Any]:
    data = path.read_bytes()
    if not 1 <= len(data) <= 8 * 1024 * 1024:
        raise FigureEvidenceError(f"JSON size differs: {path}")
    value = json.loads(data)
    if not isinstance(value, dict):
        raise FigureEvidenceError(f"JSON root is not an object: {path}")
    return value


def verify_inputs(*, include_local_revision: bool = True) -> tuple[dict[str, dict[str, Any]], dict[str, Any], pd.DataFrame]:
    inputs: dict[str, dict[str, Any]] = {}
    for name, (path, expected) in FROZEN_INPUTS.items():
        observed = identity(path)
        if observed["sha256"] != expected:
            raise FigureEvidenceError(f"frozen figure input hash differs: {name}")
        inputs[name] = observed
    qc = _read_json(QC_PATH)
    verdict = qc.get("verdict", {})
    if not (
        verdict.get("status") == "PASS_U3_STAGE4_LIQUID_ONLY_DEVELOPMENT_VALIDATION"
        and verdict.get("stage4_liquid_only_validation_complete") is True
        and verdict.get("development_only") is True
        and verdict.get("physical_fidelity_validated") is False
        and verdict.get("stage5_admitted") is False
        and qc.get("series_output", {}).get("sha256") == FROZEN_INPUTS["metrics_csv"][1]
    ):
        raise FigureEvidenceError("QC parent is not the frozen development-only PASS")
    data = pd.read_csv(CSV_PATH)
    if list(data.columns) != EXPECTED_COLUMNS or len(data) != 143 or data.isna().any().any():
        raise FigureEvidenceError("metrics table shape/columns/missingness differ")
    if data.groupby("run", sort=True).size().to_dict() != {"translation": 61, "yaw": 61, "zero": 21}:
        raise FigureEvidenceError("metrics table run cardinality differs")
    numeric = data.select_dtypes(include=["number"])
    if not np.isfinite(numeric.to_numpy()).all():
        raise FigureEvidenceError("metrics table contains a non-finite value")
    if include_local_revision:
        for name, path in (("script", SCRIPT_PATH), ("schema", SCHEMA_PATH), ("tests", TEST_PATH)):
            inputs[name] = identity(path)
    return inputs, qc, data


def _decorate_time_axis(ax: Any, *, tail: bool = True) -> None:
    if tail:
        ax.axvspan(2.0, 3.0, color="#999999", alpha=0.10, linewidth=0, zorder=0)
        ax.axvline(2.0, color="#777777", linestyle="--", linewidth=0.7, zorder=1)
    ax.set_xlim(0.0, 3.0)
    ax.set_xlabel("相对时间 (s)")
    ax.grid(axis="y", color="#D9D9D9", linewidth=0.45, alpha=0.75)
    ax.grid(axis="x", visible=False)


def _line(ax: Any, frame: pd.DataFrame, field: str, name: str, *, scale: float = 1.0, label: str | None = None) -> None:
    ax.plot(
        frame["relative_time_s"], frame[field] * scale,
        color=PALETTE[name], linestyle=LINESTYLES[name], linewidth=1.15,
        marker=MARKERS[name], markersize=2.6, markevery=5,
        markerfacecolor="white", markeredgewidth=0.55,
        label=label or LABELS[name],
    )


def build_figure(data: pd.DataFrame, qc: dict[str, Any]) -> Any:
    del qc
    setup_style(journal="general", lang="zh", use_sciplots=False, constrained_layout=True)
    fig, axes = plt.subplots(3, 2, figsize=FIGURE_SIZE, constrained_layout=True)
    zero = data[data["run"] == "zero"].sort_values("relative_time_s")
    translation = data[data["run"] == "translation"].sort_values("relative_time_s")
    yaw = data[data["run"] == "yaw"].sort_values("relative_time_s")

    ax = axes[0, 0]
    ax.plot(translation["relative_time_s"], translation["boundary_expected"] * 1000, color="#222222", linestyle=":", linewidth=0.9, label="规定输入")
    _line(ax, translation, "boundary_observed", "translation", scale=1000, label="重建边界")
    _decorate_time_axis(ax)
    ax.set_ylabel("X 平移 (mm)")
    ax.set_title("平移边界：2 mm、1 Hz")
    ax.set_ylim(-2.35, 2.35)
    ax.legend(frameon=False, loc="upper right", ncol=2, handlelength=2.4)

    ax = axes[0, 1]
    ax.plot(yaw["relative_time_s"], yaw["boundary_expected"], color="#222222", linestyle=":", linewidth=0.9, label="规定输入")
    _line(ax, yaw, "boundary_observed", "yaw", label="重建边界")
    _decorate_time_axis(ax)
    ax.set_ylabel("Z 偏航 (deg)")
    ax.set_title("偏航边界：2°、1 Hz")
    ax.set_ylim(-2.35, 2.35)
    ax.legend(frameon=False, loc="upper right", ncol=2, handlelength=2.4)

    ax = axes[1, 0]
    for name, frame in (("zero", zero), ("translation", translation), ("yaw", yaw)):
        _line(ax, frame, "fluid_speed_rms_lab_m_s", name, scale=1000)
    _decorate_time_axis(ax)
    ax.set_yscale("log")
    ax.set_ylabel("流速 RMS (mm/s, log)")
    ax.set_title("GPU 流体动态响应")
    ax.legend(frameon=False, loc="upper right", ncol=3, handlelength=2.0, columnspacing=0.8)

    ax = axes[1, 1]
    for name, frame in (("zero", zero), ("translation", translation), ("yaw", yaw)):
        _line(ax, frame, "specific_kinetic_energy_lab_j_kg", name)
    _decorate_time_axis(ax)
    ax.set_yscale("log")
    ax.set_ylabel("比动能 (J/kg, log)")
    ax.set_title("激励后自由衰减（灰区）")
    ax.legend(frameon=False, loc="upper right", ncol=3, handlelength=2.0, columnspacing=0.8)

    ax = axes[2, 0]
    for name, frame in (("zero", zero), ("translation", translation), ("yaw", yaw)):
        _line(ax, frame, "surface_first_azimuthal_harmonic_m", name, scale=1000)
    ax.axhline(0.01, color="#222222", linestyle="-.", linewidth=0.7, label="平移门槛 0.01 mm")
    _decorate_time_axis(ax)
    ax.set_ylim(bottom=0)
    ax.set_ylabel("液面一阶方位谐波 (mm)")
    ax.set_title("16-sector 容器坐标液面代理")
    ax.legend(frameon=False, loc="upper right", ncol=2, handlelength=2.0, columnspacing=0.8)

    ax = axes[2, 1]
    sector_columns = [f"surface_sector_{index:02d}_height_m" for index in range(16)]
    translation_matrix = translation[sector_columns].to_numpy().T
    yaw_matrix = yaw[sector_columns].to_numpy().T
    translation_delta = (translation_matrix - translation_matrix[:, [0]]) * 1000
    yaw_delta = (yaw_matrix - yaw_matrix[:, [0]]) * 1000
    matrix = np.vstack((translation_delta, yaw_delta))
    limit = float(np.max(np.abs(matrix)))
    norm = mcolors.TwoSlopeNorm(vmin=-limit, vcenter=0.0, vmax=limit)
    x_edges = np.linspace(-0.025, 3.025, 62)
    y_edges = np.arange(33)
    mesh = ax.pcolormesh(x_edges, y_edges, matrix, cmap="RdBu_r", norm=norm, shading="flat", rasterized=False)
    ax.axvline(2.0, color="#222222", linestyle="--", linewidth=0.7)
    ax.axhline(16.0, color="#222222", linewidth=0.8)
    ax.set_xlim(0.0, 3.0)
    ax.set_ylim(32.0, 0.0)
    ax.set_xlabel("相对时间 (s)")
    ax.set_ylabel("运动 / sector")
    ax.set_yticks([0.5, 4.5, 8.5, 12.5, 16.5, 20.5, 24.5, 28.5])
    ax.set_yticklabels(["平移 0", "平移 4", "平移 8", "平移 12", "偏航 0", "偏航 4", "偏航 8", "偏航 12"])
    ax.set_title("液面相对初帧变化（32 行）")
    colorbar = fig.colorbar(mesh, ax=ax, pad=0.02, fraction=0.048)
    colorbar.set_label("液面高度变化 (mm)")

    fig.suptitle("RTX 5080 DualSPHysics 阶段 4：液体单独验证（development-only）", fontsize=10, fontweight="bold")
    fig.text(0.5, 0.002, "灰色区域为 1 s 自由衰减尾段；曲线为单次确定性数值序列，不表示统计置信区间或实物保真。", ha="center", va="bottom", fontsize=6.3)
    finalize_figure(fig)
    layout_engine = fig.get_layout_engine()
    if layout_engine is not None:
        layout_engine.set(w_pad=0.08, h_pad=0.08, wspace=0.04, hspace=0.04)
        fig.canvas.draw()
    add_panel_labels(fig, axes=list(axes.flat), style="nature", x_offset_pt=-16, y_offset_pt=1)
    return fig


def _ensure_absent(paths: list[Path]) -> None:
    existing = []
    for path in paths:
        try:
            os.lstat(path)
            existing.append(str(path))
        except FileNotFoundError:
            pass
    if existing:
        raise FigureEvidenceError(f"create-new figure output exists: {existing}")


def _write_exclusive(path: Path, data: bytes) -> dict[str, Any]:
    if path != RECEIPT_PATH:
        raise FigureEvidenceError("figure receipt path differs")
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC, 0o640)
    try:
        os.fchmod(descriptor, 0o640)
        view = memoryview(data)
        while view:
            written = os.write(descriptor, view)
            if written <= 0:
                raise FigureEvidenceError("short figure receipt write")
            view = view[written:]
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return identity(path)


def _layout_audit(fig: Any) -> list[tuple[str, str]]:
    issues = audit_layout(fig)
    verdict = print_layout_report(issues)
    if verdict != "PASS":
        raise FigureEvidenceError(f"layout audit is not PASS: {issues}")
    return issues


def render_preview_command() -> dict[str, Any]:
    _, qc, data = verify_inputs()
    _ensure_absent([PREVIEW_PATH])
    fig = build_figure(data, qc)
    issues = _layout_audit(fig)
    render_preview(fig, str(PREVIEW_PATH), dpi=180)
    os.chmod(PREVIEW_PATH, 0o640)
    plt.close(fig)
    return {"status": "PREVIEW_READY_FOR_VISUAL_REVIEW", "preview": identity(PREVIEW_PATH), "layout_issues": issues}


def _publication_checks(paths: list[Path]) -> dict[str, Any]:
    details = {}
    for path in paths:
        issues, info = check_figure(str(path), min_dpi=DPI, target_inches=FIGURE_SIZE)
        verdict = "PASS" if not issues else max(issues, key=lambda item: SEVERITY[item[0]])[0]
        details[str(path)] = {"verdict": verdict, "issues": issues, "info": info}
        if any(SEVERITY[severity] >= SEVERITY["FAIL"] for severity, _ in issues):
            raise FigureEvidenceError(f"publication check failed: {path}: {issues}")
    return details


def _figure_spec() -> dict[str, Any]:
    return {
        "width_inches": 7.2,
        "height_inches": 7.2,
        "dpi": DPI,
        "language": "zh-CN",
        "panel_count": 6,
        "panels": [
            "translation_boundary_reconstruction",
            "yaw_boundary_reconstruction",
            "fluid_speed_rms_time_series",
            "specific_kinetic_energy_free_decay",
            "surface_first_azimuthal_harmonic",
            "translation_and_yaw_16_sector_surface_delta_heatmap",
        ],
        "palette": PALETTE,
        "heatmap_colormap": "RdBu_r",
        "heatmap_center": 0.0,
        "dual_y_axes_used": False,
        "uncertainty_band_used": False,
        "claim_ceiling": "DEVELOPMENT_ONLY_NUMERICAL_RESPONSE_NOT_PHYSICAL_FIDELITY",
    }


def _system_schema_check(receipt: dict[str, Any] | None = None) -> None:
    code = (
        "import json,sys; from jsonschema import Draft202012Validator; "
        "schema=json.load(open(sys.argv[1],encoding='utf-8')); "
        "Draft202012Validator.check_schema(schema); "
        "report=json.load(sys.stdin) if sys.argv[2]=='receipt' else None; "
        "errors=sorted(Draft202012Validator(schema).iter_errors(report),key=lambda e:list(e.absolute_path)) if report is not None else []; "
        "sys.stderr.write((str(list(errors[0].absolute_path))+': '+errors[0].message) if errors else ''); "
        "sys.exit(1 if errors else 0)"
    )
    encoded = b"" if receipt is None else json.dumps(receipt, ensure_ascii=False, sort_keys=True, allow_nan=False).encode("utf-8")
    result = subprocess.run(
        ["/usr/bin/python3", "-B", "-c", code, str(SCHEMA_PATH), "receipt" if receipt is not None else "schema"],
        input=encoded,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
        timeout=30,
        env={"PATH": "/usr/bin", "PYTHONDONTWRITEBYTECODE": "1", "LC_ALL": "C", "LANG": "C"},
    )
    if result.returncode != 0:
        message = result.stderr.decode("utf-8", errors="replace")[:2000]
        raise FigureEvidenceError(f"system closed-schema validation failed: rc={result.returncode}: {message}")


def _validate_receipt(receipt: dict[str, Any]) -> None:
    _system_schema_check(receipt)


def _checked_preview() -> dict[str, Any]:
    preview = identity(PREVIEW_PATH)
    if preview["sha256"] != EXPECTED_PREVIEW_SHA256 or preview["mode"] != "0640":
        raise FigureEvidenceError("preview identity or mode differs")
    return preview


def _asset_identities() -> dict[str, dict[str, Any]]:
    outputs = {
        "preview_png": _checked_preview(),
        "png": identity(PNG_PATH),
        "grayscale_png": identity(GRAY_PATH),
        "pdf": identity(PDF_PATH),
        "svg": identity(SVG_PATH),
    }
    if any(value["mode"] != "0640" for value in outputs.values()):
        raise FigureEvidenceError("figure output mode differs from 0640")
    return outputs


def export_assets_command(*, preview_visual_review_pass: bool) -> dict[str, Any]:
    if not preview_visual_review_pass:
        raise FigureEvidenceError("--visual-review-pass is required after inspecting the preview")
    _, qc, data = verify_inputs()
    _checked_preview()
    _ensure_absent([PNG_PATH, GRAY_PATH, PDF_PATH, SVG_PATH, RECEIPT_PATH])
    fig = build_figure(data, qc)
    issues = _layout_audit(fig)
    saved = export_figure(
        fig,
        str(BASENAME),
        formats=("pdf", "svg", "png"),
        dpi=DPI,
        size_inches=FIGURE_SIZE,
        grayscale_preview=False,
        tight=False,
        transparent=False,
    )
    if set(map(Path, saved)) != {PDF_PATH, SVG_PATH, PNG_PATH}:
        raise FigureEvidenceError("exported path set differs")
    for path in (PDF_PATH, SVG_PATH, PNG_PATH):
        os.chmod(path, 0o640)
    with Image.open(PNG_PATH) as source:
        grayscale = source.convert("L")
        grayscale.save(GRAY_PATH, dpi=(DPI, DPI))
    os.chmod(GRAY_PATH, 0o640)
    plt.close(fig)
    checked = [PNG_PATH, GRAY_PATH, PDF_PATH, SVG_PATH]
    publication = _publication_checks(checked)
    outputs = _asset_identities()
    return {
        "status": "ASSETS_READY_FOR_COLOR_AND_GRAYSCALE_VISUAL_REVIEW",
        "outputs": outputs,
        "publication_checks": publication,
        "receipt_created": False,
    }


def finalize_receipt_command(*, visual_review_pass: bool) -> dict[str, Any]:
    if not visual_review_pass:
        raise FigureEvidenceError("--visual-review-pass is required after inspecting color and grayscale assets")
    inputs, qc, data = verify_inputs()
    del qc
    _ensure_absent([RECEIPT_PATH])
    checked = [PNG_PATH, GRAY_PATH, PDF_PATH, SVG_PATH]
    outputs = _asset_identities()
    publication = _publication_checks(checked)
    fig = build_figure(data, _read_json(QC_PATH))
    issues = _layout_audit(fig)
    plt.close(fig)
    receipt = {
        "schema_version": "r8-liquid-u3-stage4-synthetic-motion-figure-receipt-v1",
        "document_type": "SMPCC_R8_LIQUID_U3_STAGE4_SYNTHETIC_MOTION_FIGURE_RECEIPT_V1",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "inputs": inputs,
        "figure_spec": _figure_spec(),
        "outputs": outputs,
        "machine_audit": {
            "layout_verdict": "PASS",
            "layout_issues": issues,
            "publication_check_strict_pass": all(not any(SEVERITY[severity] >= SEVERITY["FAIL"] for severity, _ in value["issues"]) for value in publication.values()),
            "files_checked": [str(path) for path in checked],
            "grayscale_generated": True,
            "missing_glyphs_detected": False,
            "text_clipping_detected": False,
            "tick_overlap_detected": False,
        },
        "visual_review": {
            "reviewer": "CODEX_AI_VISUAL_REVIEW",
            "preview_inspected": True,
            "legend_does_not_cover_data": True,
            "panel_labels_aligned": True,
            "color_and_grayscale_distinguishable": True,
            "heatmap_has_labeled_colorbar": True,
            "axes_units_readable": True,
            "no_visual_clipping": True,
            "yaw_interpretation_not_overclaimed": True,
            "overall_pass": True,
        },
        "claims": {
            "development_only": True,
            "formal": False,
            "physical_fidelity_validated": False,
            "production_eligible": False,
            "stage5_entered": False,
            "stage6_replay_claimed": False,
            "solver_executed_by_figure_tool": False,
            "gpu_exposed_by_figure_tool": False,
            "network_used": False,
            "inputs_modified": False,
            "status": "PASS_STAGE4_DIAGNOSTIC_FIGURE_DEVELOPMENT_ONLY",
        },
    }
    _validate_receipt(receipt)
    encoded = (json.dumps(receipt, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n").encode("utf-8")
    receipt_identity = _write_exclusive(RECEIPT_PATH, encoded)
    return {"status": receipt["claims"]["status"], "receipt": receipt_identity, "outputs": outputs, "publication_checks": publication}


def static_self_check() -> dict[str, Any]:
    schema = _read_json(SCHEMA_PATH)
    _system_schema_check()
    if schema.get("additionalProperties") is not False:
        raise FigureEvidenceError("figure schema root is not closed")
    for name, definition in schema.get("$defs", {}).items():
        if definition.get("type") == "object" and definition.get("additionalProperties") is not False:
            raise FigureEvidenceError(f"figure schema object is not closed: {name}")
    _, qc, data = verify_inputs(include_local_revision=False)
    fig = build_figure(data, qc)
    issues = _layout_audit(fig)
    data_axes = [axis for axis in fig.axes if axis.get_subplotspec() is not None]
    plt.close(fig)
    if len(data_axes) != 6 or _figure_spec()["dual_y_axes_used"]:
        raise FigureEvidenceError("figure panel/axis contract differs")
    return {
        "status": "PASS_U3_STAGE4_SYNTHETIC_MOTION_FIGURE_V1_STATIC_SELF_CHECK",
        "rows": len(data),
        "columns": len(data.columns),
        "panels": len(data_axes),
        "layout_issues": issues,
        "dual_y_axes_used": False,
        "solver_executed": False,
        "gpu_exposed": False,
        "network_used": False,
    }


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("command", choices=("self-check", "render-preview", "export-assets", "finalize-receipt"))
    result.add_argument("--visual-review-pass", action="store_true")
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        if args.command == "self-check":
            result = static_self_check()
        elif args.command == "render-preview":
            result = render_preview_command()
        elif args.command == "export-assets":
            result = export_assets_command(preview_visual_review_pass=args.visual_review_pass)
        else:
            result = finalize_receipt_command(visual_review_pass=args.visual_review_pass)
        print(json.dumps(result, ensure_ascii=False, sort_keys=True, default=str))
        return 0
    except (FigureEvidenceError, OSError, ValueError, KeyError, UnicodeError, json.JSONDecodeError, subprocess.SubprocessError) as exc:
        print(json.dumps({"status": "ERROR_FIGURE_EVIDENCE", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
