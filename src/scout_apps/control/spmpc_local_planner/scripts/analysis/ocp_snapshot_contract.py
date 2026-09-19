"""ROS-independent ABI checks for PreSolveSnapshot parameter rows.

Schema 7/cost v2 is historical (34/46 parameters); schema 8/cost v3 is the
planning ABI (92/104). Schema 9 adds the actual-jerk bound without changing
parameter rows.  This module deliberately never upgrades a record while
validating it.  ``upgrade_parameter_row`` is an explicit replay adapter and
retains provenance in its return value.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Mapping, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "acados"))
from generate_spmpc_acados import default_parameter_values, load_config  # noqa: E402
from spmpc_acados_model import PARAM_NAMES, PARAM_NAMES_SLOSH  # noqa: E402

OLD = {7: (34, 46, 24, 28, 2)}
NEW = {version: (len(PARAM_NAMES), len(PARAM_NAMES_SLOSH), 24, 28, 3) for version in (8, 9)}


def validate_snapshot(record: Mapping) -> dict:
    schema = int(record.get("schema_version", -1))
    cost = int(record.get("cost_model_version", -1))
    if schema in OLD:
        np_b0, np_sl, nx_b0, nx_sl, expected_cost = OLD[schema]
    elif schema in NEW:
        np_b0, np_sl, nx_b0, nx_sl, expected_cost = NEW[schema]
    else:
        raise ValueError(f"unsupported snapshot schema_version={schema}")
    if cost != expected_cost:
        raise ValueError(f"schema {schema} requires cost_model_version={expected_cost}, got {cost}")
    if int(record.get("liquid_model_version", -1)) != 1:
        raise ValueError("liquid_model_version must be 1")
    if "horizon_steps" not in record or int(record["horizon_steps"]) <= 0:
        raise ValueError("horizon_steps must be positive and present")
    if "state_width" not in record or "control_width" not in record:
        raise ValueError("state_width and control_width are required")
    nx = int(record["state_width"])
    width = int(record.get("parameter_width", -1))
    slosh = nx == nx_sl
    if nx not in (nx_b0, nx_sl) or width != (np_sl if slosh else np_b0):
        raise ValueError(f"invalid dimensions nx={nx}, parameter_width={width}")
    names = list(record.get("parameter_names", ()))
    expected_names = (PARAM_NAMES_SLOSH if slosh else PARAM_NAMES) if schema in NEW else (
        PARAM_NAMES[:34] + (PARAM_NAMES_SLOSH[len(PARAM_NAMES):] if slosh else []))
    if names != expected_names:
        raise ValueError("parameter_names do not match the versioned ABI layout")
    rows = record.get("stage_parameters", ())
    expected_rows = int(record["horizon_steps"]) + 1
    if not isinstance(rows, Sequence) or isinstance(rows, (str, bytes)) or len(rows) != expected_rows * width:
        raise ValueError("stage_parameters length must equal (horizon_steps+1)*parameter_width")
    if any(not math.isfinite(float(x)) for x in rows):
        raise ValueError("stage_parameters contains nonfinite value")
    if int(record["control_width"]) != 3:
        raise ValueError("control_width must be 3")
    if "slosh_enabled" in record and bool(record["slosh_enabled"]) != slosh:
        raise ValueError("slosh_enabled does not match state_width")
    if schema == 9 and ("actual_jerk_max" not in record or not math.isfinite(record["actual_jerk_max"]) or record["actual_jerk_max"] < 0):
        raise ValueError("schema 9 requires a finite nonnegative actual_jerk_max")
    return {"schema_version": schema, "cost_model_version": cost,
            "state_width": nx, "parameter_width": width,
            "source_schema": schema, "source_cost_model": cost}


def upgrade_parameter_row(row: Sequence[float], *, slosh: bool) -> dict:
    """Map one historical row to planning ABI, without changing old values."""
    expected = 46 if slosh else 34
    if len(row) != expected or any(not math.isfinite(float(x)) for x in row):
        raise ValueError(f"historical row must contain {expected} finite values")
    cfg = load_config()
    defaults = list(default_parameter_values(cfg, with_slosh=slosh))
    out = defaults
    out[:34] = [float(x) for x in row[:34]]
    if slosh:
        slosh_offset = len(PARAM_NAMES)
        out[slosh_offset:slosh_offset + 12] = [float(x) for x in row[34:46]]
    return {"parameters": out, "source_schema": 7, "source_cost_model": 2,
            "target_schema": 8, "target_cost_model": 3,
            "purpose": "dynamics_replay_only",
            "cost_reconstruction_supported": False}
