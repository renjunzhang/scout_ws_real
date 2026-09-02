"""Pure Stage 3A layout scaffold for the mainline solver contract.

This module expands the frozen state/control and execution-parameter-prefix
formulae without importing CasADi, acados, ROS, or runtime configuration. It
cannot construct a production layout: Stage 1 has not frozen ``L_max`` and no
full parameter/cost/constraint graph exists yet.
"""

from __future__ import annotations

import math
import re
from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
from types import MappingProxyType
from typing import Any

DEFAULT_DT_SEC = 1.0 / 30.0
DEFAULT_HORIZON_STEPS = 60
EXECUTION_SUBSEGMENT_SLOTS = 3
BASE_STATE_COUNT = 14
CONTROL_COUNT = 3

# This safety bound prevents a malformed synthetic input from expanding an
# unbounded list. It is deliberately present in the scaffold identity; it is
# not an actuator-identification result or a hidden production L_max.
MAX_RETAINED_COMMANDS_PER_CHANNEL = 4096

LAYOUT_SCHEMA_VERSION = "spmpc_mainline_layout_scaffold_v1"
LAYOUT_SCOPE = "STATE_CONTROL_AND_EXECUTION_PARAMETER_PREFIX_ONLY"
MODEL_ID = "spmpc_actuator_slosh_discrete_v1"
DISCRETIZATION_SCHEMA = "zoh_fopdt_piecewise_midpoint_pose_rk4_slosh_v1"
COST_SCHEMA = "right_endpoint_fixed_liquid_weight_v1"

STATE_ROBOT_PROGRESS = (
    "px",
    "py",
    "theta",
    "s",
    "v_actual",
    "omega_actual",
)
STATE_PUBLISHER = (
    "q_prev_v",
    "q_prev_omega",
    "a_prev",
    "alpha_prev",
)
STATE_LIQUID = ("eta_x", "eta_x_dot", "eta_y", "eta_y_dot")
CONTROL_ORDER = ("j_issue_v", "j_issue_omega", "v_s")
EXECUTION_PARAMETER_ORDER_SCHEMA = (
    "act_inv_tau_v",
    "act_gain_v",
    "act_inv_tau_omega",
    "act_gain_omega",
    "act_seg_dt[0:2]",
    "act_sel_v[0:2][0:NQ_v-1]",
    "act_sel_omega[0:2][0:NQ_omega-1]",
)
MISSING_BEFORE_ARTIFACT = (
    "stage1_frozen_lmax_authority",
    "production_resource_budget",
    "full_parameter_layout",
    "cost_expression",
    "constraints",
    "solver_options",
    "generated_solver_library",
    "legacy_generator_retirement",
)

_SCENARIO_ID_RE = re.compile(r"^[a-z0-9][a-z0-9_.-]{0,127}$")


class LayoutError(ValueError):
    """Raised when a scaffold layout is malformed or overclaims authority."""


class LayoutPurpose(str, Enum):
    """The only authority admitted before a frozen Stage 1 contract exists."""

    STAGE3A_SYNTHETIC = "STAGE3A_SYNTHETIC_NO_ARTIFACT"


def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise LayoutError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (OverflowError, ValueError) as exc:
        raise LayoutError(f"{label} must be representable as a finite float") from exc
    if not math.isfinite(result):
        raise LayoutError(f"{label} must be finite")
    return result


def _scenario_id(value: Any) -> str:
    if type(value) is not str or not _SCENARIO_ID_RE.fullmatch(value):
        raise LayoutError("scenario_id must match [a-z0-9][a-z0-9_.-]{0,127}")
    return value


def _retained_command_count(maximum: float, dt_sec: float, label: str) -> int:
    if maximum < 0.0:
        raise LayoutError(f"{label} must be non-negative")
    ratio = maximum / dt_sec
    if not math.isfinite(ratio):
        raise LayoutError(f"{label}/dt is non-finite")
    retained = math.ceil(ratio)
    if retained > MAX_RETAINED_COMMANDS_PER_CHANNEL:
        raise LayoutError(
            f"{label} produces {retained} retained commands; scaffold guard is "
            f"{MAX_RETAINED_COMMANDS_PER_CHANNEL}"
        )
    return retained


def _freeze_mapping(value: Mapping[str, Any]) -> Mapping[str, Any]:
    return MappingProxyType(dict(value))


def _strict_equal(left: Any, right: Any) -> bool:
    """Compare JSON values without Python's bool/int or int/float aliases."""

    if type(left) is not type(right):
        return False
    if type(left) is dict:
        return set(left) == set(right) and all(
            _strict_equal(left[key], right[key]) for key in left
        )
    if type(left) is list:
        return len(left) == len(right) and all(
            _strict_equal(left_item, right_item)
            for left_item, right_item in zip(left, right)
        )
    return bool(left == right)


@dataclass(frozen=True)
class LayoutSpec:
    """Explicit synthetic input to :func:`build_layout`.

    There is intentionally no ``frozen=True`` boolean or production escape
    hatch. A future production constructor must consume a typed Stage 1
    contract reference rather than trusting a caller assertion.
    """

    purpose: LayoutPurpose
    scenario_id: str
    l_max_v_sec: float
    l_max_omega_sec: float
    dt_sec: float = DEFAULT_DT_SEC
    horizon_steps: int = DEFAULT_HORIZON_STEPS


@dataclass(frozen=True)
class MainlineLayoutScaffold:
    """Expanded immutable state/control/execution-prefix layout."""

    purpose: LayoutPurpose
    scenario_id: str
    l_max_v_sec: float
    l_max_omega_sec: float
    dt_sec: float
    horizon_steps: int
    d_v: int
    d_omega: int
    r_v: int
    r_omega: int
    nq_v: int
    nq_omega: int
    nx: int
    nu: int
    np_exec: int
    state_names: tuple[str, ...]
    control_names: tuple[str, ...]
    execution_parameter_names: tuple[str, ...]
    state_offsets: Mapping[str, int]
    control_offsets: Mapping[str, int]
    execution_parameter_offsets: Mapping[str, int]
    selector_offsets: Mapping[str, tuple[tuple[int, ...], ...]]

    @property
    def NX(self) -> int:
        return self.nx

    @property
    def NU(self) -> int:
        return self.nu

    @property
    def NP_exec(self) -> int:
        return self.np_exec

    @property
    def state_group_names(self) -> Mapping[str, tuple[str, ...]]:
        return _freeze_mapping(
            {
                "robot_progress": STATE_ROBOT_PROGRESS,
                "publisher": STATE_PUBLISHER,
                "delay_older": tuple(
                    [f"older_v[{index}]" for index in range(self.d_v)]
                    + [f"older_omega[{index}]" for index in range(self.d_omega)]
                ),
                "liquid": STATE_LIQUID,
            }
        )

    def to_dict(self) -> dict[str, Any]:
        """Return the complete JSON-compatible scaffold identity."""

        return {
            "schema_version": LAYOUT_SCHEMA_VERSION,
            "scope": LAYOUT_SCOPE,
            "authority": {
                "purpose": self.purpose.value,
                "scenario_id": self.scenario_id,
                "artifact_generation_allowed": False,
            },
            "target_contract": {
                "model_id": MODEL_ID,
                "discretization_schema": DISCRETIZATION_SCHEMA,
                "cost_schema": COST_SCHEMA,
            },
            "horizon": {
                "N": self.horizon_steps,
                "dt_sec": self.dt_sec,
            },
            "delay_layout": {
                "L_max_v_sec": self.l_max_v_sec,
                "L_max_omega_sec": self.l_max_omega_sec,
                "v": {"R": self.r_v, "D": self.d_v, "NQ": self.nq_v},
                "omega": {
                    "R": self.r_omega,
                    "D": self.d_omega,
                    "NQ": self.nq_omega,
                },
                "formulae": {
                    "R_c": "ceil(L_max_c/dt)",
                    "D_c": "max(0,R_c-1)",
                    "NQ_c": "R_c+1",
                },
                "generation_resource_guard": {
                    "max_retained_commands_per_channel": (
                        MAX_RETAINED_COMMANDS_PER_CHANNEL
                    )
                },
            },
            "dimensions": {
                "NX": self.nx,
                "NU": self.nu,
                "NP_exec": self.np_exec,
            },
            "state_layout": {
                "stage_semantics": "pre_issue_at_T_k_minus",
                "robot_progress": list(STATE_ROBOT_PROGRESS),
                "publisher": list(STATE_PUBLISHER),
                "delay_older": list(self.state_group_names["delay_older"]),
                "liquid": list(STATE_LIQUID),
                "ordered": list(self.state_names),
                "offsets": dict(self.state_offsets),
                "dimension": self.nx,
            },
            "control_layout": {
                "ordered": list(self.control_names),
                "offsets": dict(self.control_offsets),
                "dimension": self.nu,
            },
            "execution_parameter_schema": {
                "subsegment_slots": EXECUTION_SUBSEGMENT_SLOTS,
                "NP_exec": "7+3*(NQ_v+NQ_omega)",
                "ordered_pattern": list(EXECUTION_PARAMETER_ORDER_SCHEMA),
            },
            "execution_parameter_layout": {
                "ordered": list(self.execution_parameter_names),
                "offsets": dict(self.execution_parameter_offsets),
                "selector_offsets": {
                    channel: [list(slot) for slot in slots]
                    for channel, slots in self.selector_offsets.items()
                },
                "selector_width": {"v": self.nq_v, "omega": self.nq_omega},
                "dimension": self.np_exec,
            },
            "missing_before_artifact": list(MISSING_BEFORE_ARTIFACT),
        }


def build_layout(spec: LayoutSpec) -> MainlineLayoutScaffold:
    """Expand one synthetic Stage 3A layout and validate every dimension."""

    if not isinstance(spec, LayoutSpec):
        raise LayoutError("spec must be a LayoutSpec")
    if spec.purpose is not LayoutPurpose.STAGE3A_SYNTHETIC:
        raise LayoutError("only STAGE3A_SYNTHETIC_NO_ARTIFACT authority is available")
    scenario_id = _scenario_id(spec.scenario_id)
    dt_sec = _finite_number(spec.dt_sec, "dt_sec")
    if dt_sec != DEFAULT_DT_SEC:
        raise LayoutError(
            f"dt_sec must equal the frozen 1/30 s grid, got {spec.dt_sec!r}"
        )
    if isinstance(spec.horizon_steps, bool) or not isinstance(spec.horizon_steps, int):
        raise LayoutError("horizon_steps must be an integer")
    if spec.horizon_steps != DEFAULT_HORIZON_STEPS:
        raise LayoutError(
            f"horizon_steps must equal the frozen {DEFAULT_HORIZON_STEPS}"
        )

    l_max_v_sec = _finite_number(spec.l_max_v_sec, "L_max_v_sec")
    l_max_omega_sec = _finite_number(spec.l_max_omega_sec, "L_max_omega_sec")
    r_v = _retained_command_count(l_max_v_sec, dt_sec, "L_max_v_sec")
    r_omega = _retained_command_count(l_max_omega_sec, dt_sec, "L_max_omega_sec")
    d_v = max(0, r_v - 1)
    d_omega = max(0, r_omega - 1)
    nq_v = r_v + 1
    nq_omega = r_omega + 1
    nx = BASE_STATE_COUNT + d_v + d_omega
    nu = CONTROL_COUNT
    np_exec = 7 + EXECUTION_SUBSEGMENT_SLOTS * (nq_v + nq_omega)

    state_names = list(STATE_ROBOT_PROGRESS) + list(STATE_PUBLISHER)
    state_names.extend(f"older_v[{index}]" for index in range(d_v))
    state_names.extend(f"older_omega[{index}]" for index in range(d_omega))
    state_names.extend(STATE_LIQUID)
    if len(state_names) != nx or len(set(state_names)) != nx:
        raise LayoutError("state expansion does not match unique NX entries")
    state_offsets = {name: index for index, name in enumerate(state_names)}

    control_names = list(CONTROL_ORDER)
    if len(control_names) != nu or len(set(control_names)) != nu:
        raise LayoutError("control expansion does not match unique NU entries")
    control_offsets = {name: index for index, name in enumerate(control_names)}

    parameter_names = [
        "act_inv_tau_v",
        "act_gain_v",
        "act_inv_tau_omega",
        "act_gain_omega",
    ]
    parameter_names.extend(
        f"act_seg_dt[{slot}]" for slot in range(EXECUTION_SUBSEGMENT_SLOTS)
    )
    selector_offsets: dict[str, tuple[tuple[int, ...], ...]] = {}
    for channel, width in (("v", nq_v), ("omega", nq_omega)):
        channel_slots: list[tuple[int, ...]] = []
        for slot in range(EXECUTION_SUBSEGMENT_SLOTS):
            indices: list[int] = []
            for selector in range(width):
                indices.append(len(parameter_names))
                parameter_names.append(f"act_sel_{channel}[{slot}][{selector}]")
            channel_slots.append(tuple(indices))
        selector_offsets[channel] = tuple(channel_slots)

    if len(parameter_names) != np_exec or len(set(parameter_names)) != np_exec:
        raise LayoutError(
            "execution parameter expansion does not match unique NP_exec entries"
        )
    parameter_offsets = {name: index for index, name in enumerate(parameter_names)}

    return MainlineLayoutScaffold(
        purpose=spec.purpose,
        scenario_id=scenario_id,
        l_max_v_sec=l_max_v_sec,
        l_max_omega_sec=l_max_omega_sec,
        dt_sec=dt_sec,
        horizon_steps=spec.horizon_steps,
        d_v=d_v,
        d_omega=d_omega,
        r_v=r_v,
        r_omega=r_omega,
        nq_v=nq_v,
        nq_omega=nq_omega,
        nx=nx,
        nu=nu,
        np_exec=np_exec,
        state_names=tuple(state_names),
        control_names=tuple(control_names),
        execution_parameter_names=tuple(parameter_names),
        state_offsets=_freeze_mapping(state_offsets),
        control_offsets=_freeze_mapping(control_offsets),
        execution_parameter_offsets=_freeze_mapping(parameter_offsets),
        selector_offsets=_freeze_mapping(selector_offsets),
    )


def build_synthetic_layout(
    l_max_v_sec: float,
    l_max_omega_sec: float,
    *,
    scenario_id: str,
    dt_sec: float = DEFAULT_DT_SEC,
    horizon_steps: int = DEFAULT_HORIZON_STEPS,
) -> MainlineLayoutScaffold:
    """Readable constructor whose name preserves the non-production scope."""

    return build_layout(
        LayoutSpec(
            purpose=LayoutPurpose.STAGE3A_SYNTHETIC,
            scenario_id=scenario_id,
            l_max_v_sec=l_max_v_sec,
            l_max_omega_sec=l_max_omega_sec,
            dt_sec=dt_sec,
            horizon_steps=horizon_steps,
        )
    )


def layout_from_dict(value: Any) -> MainlineLayoutScaffold:
    """Rebuild and exactly verify a serialized scaffold layout."""

    if type(value) is not dict:
        raise LayoutError("layout document must be a JSON object")
    expected_top = {
        "schema_version",
        "scope",
        "authority",
        "target_contract",
        "horizon",
        "delay_layout",
        "dimensions",
        "state_layout",
        "control_layout",
        "execution_parameter_schema",
        "execution_parameter_layout",
        "missing_before_artifact",
    }
    if set(value) != expected_top:
        raise LayoutError("layout document keys do not match scaffold schema")
    authority = value.get("authority")
    horizon = value.get("horizon")
    delay = value.get("delay_layout")
    if type(authority) is not dict or set(authority) != {
        "purpose",
        "scenario_id",
        "artifact_generation_allowed",
    }:
        raise LayoutError("layout authority keys do not match scaffold schema")
    if authority["artifact_generation_allowed"] is not False:
        raise LayoutError("Stage 3A layout cannot authorize artifact generation")
    if type(horizon) is not dict or set(horizon) != {"N", "dt_sec"}:
        raise LayoutError("layout horizon keys do not match scaffold schema")
    if type(delay) is not dict or not {
        "L_max_v_sec",
        "L_max_omega_sec",
    }.issubset(delay):
        raise LayoutError("layout delay fields are missing")
    try:
        purpose = LayoutPurpose(authority["purpose"])
    except (TypeError, ValueError) as exc:
        raise LayoutError("layout purpose is not a supported authority") from exc
    rebuilt = build_layout(
        LayoutSpec(
            purpose=purpose,
            scenario_id=authority["scenario_id"],
            l_max_v_sec=delay["L_max_v_sec"],
            l_max_omega_sec=delay["L_max_omega_sec"],
            dt_sec=horizon["dt_sec"],
            horizon_steps=horizon["N"],
        )
    )
    if not _strict_equal(value, rebuilt.to_dict()):
        raise LayoutError("layout document does not match expanded formulae")
    return rebuilt
