#!/usr/bin/env python3
"""Static S6 visualization/media/evidence planner for synthetic v2 results.

The only data entry point accepts exact in-memory analysis-result bytes.  This
module has no artifact writer or renderer and cannot encode MP4/GIF, append a
ledger, or claim grayscale/layout QA completion.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

from jsonschema import Draft202012Validator
from jsonschema.exceptions import ValidationError


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent.parent
RESULT_SCHEMA_PATH = ROOT / "schema/target_host_s6_primary_analysis_result_v2.json"
PLAN_SCHEMA_PATH = ROOT / "schema/target_host_s6_readonly_delivery_plan_v3.json"
MAXIMUM_RESULT_BYTES = 16 * 1024 * 1024


class S6DeliveryPlanError(ValueError):
    """Input identity, schema, claim ceiling, or static delivery-plan failure."""


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _read_schema(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise S6DeliveryPlanError(f"schema root is not an object: {path.name}")
    return value


def assert_deep_closed(value: Any, location: str = "$") -> None:
    if isinstance(value, dict):
        if value.get("type") == "object" and value.get("additionalProperties") is not False:
            raise S6DeliveryPlanError(f"schema object is open at {location}")
        for key, child in value.items():
            assert_deep_closed(child, f"{location}/{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            assert_deep_closed(child, f"{location}/{index}")


def load_schemas() -> tuple[dict[str, Any], dict[str, Any]]:
    schemas = (_read_schema(RESULT_SCHEMA_PATH), _read_schema(PLAN_SCHEMA_PATH))
    for schema in schemas:
        Draft202012Validator.check_schema(schema)
        assert_deep_closed(schema)
    return schemas


def read_analysis_bytes(raw: bytes, *, expected_sha256: str) -> dict[str, Any]:
    if not isinstance(raw, bytes) or not 0 < len(raw) <= MAXIMUM_RESULT_BYTES:
        raise S6DeliveryPlanError("analysis result bytes are absent or exceed the bound")
    if sha256_bytes(raw) != expected_sha256:
        raise S6DeliveryPlanError("analysis result SHA-256 differs")
    try:
        value = json.loads(raw, parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))
    except (json.JSONDecodeError, ValueError) as exc:
        raise S6DeliveryPlanError("analysis result JSON is invalid or non-finite") from exc
    result_schema, _ = load_schemas()
    try:
        Draft202012Validator(result_schema).validate(value)
    except ValidationError as exc:
        raise S6DeliveryPlanError(f"analysis result schema differs: {exc.message}") from exc
    if (value["planned_denominator"] != 1
            or value["materialization"]["status"] != "NOT_ADMITTED_SYNTHETIC_FIXTURE_ONLY"
            or value["claims"] != {
                "stage6_pass": False,
                "paired_ranking": False,
                "cpu_selected_trajectory_comparison": False,
                "physical_reference_pending": True,
                "physical_fidelity_validated": False,
                "formal": False,
                "production": False,
                "physical_primary": False,
            }):
        raise S6DeliveryPlanError("analysis claim ceiling differs")
    return value


def build_plan(analysis_raw: bytes, *, expected_sha256: str) -> dict[str, Any]:
    analysis = read_analysis_bytes(analysis_raw, expected_sha256=expected_sha256)
    plan = {
        "schema_version": "smpcc-r8-liquid-s6-readonly-delivery-plan-v3",
        "document_type": "SMPCC_R8_LIQUID_S6_READONLY_DELIVERY_PLAN_V3",
        "status": "S6_SYNTHETIC_READONLY_DELIVERY_CONTRACT_VALIDATED_NOT_MATERIALIZED",
        "attempt_id": analysis["attempt_id"],
        "planned_denominator": 1,
        "analysis_parent": {
            "kind": "SYNTHETIC_S6_PRIMARY_ANALYSIS_RESULT_V2",
            "sha256": expected_sha256,
            "status": analysis["status"],
            "real_parent_materialized": False,
        },
        "visualization": {
            "layout": "THREE_VERTICAL_SHARED_X_PANELS",
            "shared_x": True,
            "axes_count": 3,
            "dual_y_axes": False,
            "panels": [
                {"panel_id": "a", "series": ["H_crest", "H_abs", "H_peak_to_peak"], "unit": "mm"},
                {"panel_id": "b", "series": ["H_proxy", "H_modal"], "unit": "mm"},
                {"panel_id": "c", "series": ["H_crest_minus_H_proxy", "H_crest_minus_H_modal"], "unit": "mm"},
            ],
            "palette": {
                "name": "OKABE_ITO_COLORBLIND_SAFE",
                "H_crest": "#0072B2", "H_abs": "#E69F00", "H_peak_to_peak": "#009E73",
                "H_proxy": "#0072B2", "H_modal": "#CC79A7",
                "H_crest_minus_H_proxy": "#000000", "H_crest_minus_H_modal": "#E69F00",
            },
            "line_styles": {
                "H_crest": "-", "H_abs": "--", "H_peak_to_peak": ":",
                "H_proxy": "-", "H_modal": "--",
                "H_crest_minus_H_proxy": "-", "H_crest_minus_H_modal": "--",
            },
            "rainbow_colormap": False,
            "grayscale_qa": {
                "required": True,
                "path": "figures/primary_shared_x_timeseries_grayscale.png",
                "status": "REQUIRED_BEFORE_MATERIALIZATION",
                "executed": False,
                "pass_claimed": False,
            },
            "layout_qa": {
                "checks": ["missing_glyphs", "text_clipping", "tick_overlap", "legend_occlusion", "panel_alignment"],
                "status": "REQUIRED_BEFORE_MATERIALIZATION",
                "executed": False,
                "pass_claimed": False,
            },
        },
        "delivery": {
            "read_only_analysis": True,
            "write_semantics": "CREATE_NEW_ONLY_IF_SEPARATELY_ADMITTED",
            "artifacts_materialized": False,
            "final_artifacts_allowed": False,
            "planned_inventory": [
                "comparison_manifest.json",
                "data/derived_surface_metrics.csv",
                "data/aligned_primary_comparison.csv",
                "reports/primary_summary.json",
                "figures/primary_shared_x_timeseries.png",
                "figures/primary_shared_x_timeseries_grayscale.png",
                "animation/primary.mp4",
                "animation/primary_preview.gif",
                "keyframes/primary_t000.png",
                "evidence_index.json",
                "secondary_ledger_entry.json",
                "checksums.sha256",
            ],
            "animation": {
                "fact_source": "FINALIZED_SOLVER_FRAMES_ONLY",
                "mp4": "animation/primary.mp4",
                "gif": "animation/primary_preview.gif",
                "keyframes": ["keyframes/primary_t000.png"],
                "keyframe_overlay": ["container_coordinates", "time", "particle_classes", "liquid_surface_probes"],
                "encoder_executed": False,
            },
            "checksums": {
                "algorithm": "SHA-256",
                "path": "checksums.sha256",
                "coverage": "ALL_MATERIALIZED_ARTIFACTS_EXCEPT_SELF",
                "status": "PLANNED_NOT_MATERIALIZED",
                "entries": [],
            },
        },
        "evidence": {
            "comparison_manifest": {"path": "comparison_manifest.json", "status": "PLANNED_NOT_MATERIALIZED", "entries": []},
            "evidence_index": {"path": "evidence_index.json", "status": "PLANNED_NOT_MATERIALIZED", "entries": []},
            "secondary_ledger": {"path": "secondary_ledger_entry.json", "status": "PLANNED_NOT_APPENDED", "append_performed": False},
        },
        "claims": dict(analysis["claims"]),
    }
    _, plan_schema = load_schemas()
    try:
        Draft202012Validator(plan_schema).validate(plan)
    except ValidationError as exc:
        raise S6DeliveryPlanError(f"delivery plan schema differs: {exc.message}") from exc
    return plan


def self_check() -> dict[str, Any]:
    result_schema, plan_schema = load_schemas()
    return {
        "status": "S6_READONLY_DELIVERY_PLANNER_V3_SELF_CHECK_OK_NOT_MATERIALIZED",
        "result_schema_id": result_schema["$id"],
        "plan_schema_id": plan_schema["$id"],
        "real_bag_or_bi4_read": False,
        "renderer_executed": False,
        "animation_executed": False,
        "external_write_performed": False,
        "final_artifacts_allowed": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("self-check",))
    parser.parse_args(argv)
    try:
        print(json.dumps(self_check(), ensure_ascii=False, sort_keys=True))
    except (S6DeliveryPlanError, OSError, ValueError, KeyError) as exc:
        print(json.dumps({"status": "S6_DELIVERY_PLANNER_SELF_CHECK_FAILED", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["S6DeliveryPlanError", "build_plan", "load_schemas", "read_analysis_bytes", "self_check"]
