"""Pure fixed-width layout derived from the pinned development capacity."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import InitVar, dataclass
from fractions import Fraction
from types import MappingProxyType
from typing import Any

from .development_capacity import (
    DevelopmentCapacityContract,
    DevelopmentCapacityError,
    require_pinned_development_capacity,
)
from .identity import strict_json_equal

DEVELOPMENT_LAYOUT_SCHEMA_VERSION = "spmpc_mainline_development_layout_v1"
DEVELOPMENT_LAYOUT_SCOPE = "DEVELOPMENT_STATE_CONTROL_EXECUTION_PREFIX_ONLY"
HORIZON_STEPS = 60

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
STAGE_SEMANTICS = "pre_issue_at_T_k_minus"

_CONSTRUCTION_TOKEN = object()


class DevelopmentLayoutError(ValueError):
    """Raised when typed capacity or a serialized development layout drifts."""


def _freeze_mapping(value: Mapping[str, Any]) -> Mapping[str, Any]:
    return MappingProxyType(dict(value))


def _require_pinned_capacity(
    capacity: DevelopmentCapacityContract,
) -> DevelopmentCapacityContract:
    try:
        return require_pinned_development_capacity(capacity)
    except DevelopmentCapacityError as exc:
        raise DevelopmentLayoutError(str(exc)) from exc


@dataclass(frozen=True)
class DevelopmentLayout:
    """Immutable state, control, and execution-prefix ordering."""

    capacity_contract_sha256: str
    release_frequency_hz: int
    release_period_sec: Fraction
    horizon_steps: int
    r_v: int
    r_omega: int
    d_v: int
    d_omega: int
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
    _construction_token: InitVar[object] = None

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _CONSTRUCTION_TOKEN:
            raise DevelopmentLayoutError(
                "DevelopmentLayout requires build_development_layout"
            )

    @property
    def NX(self) -> int:
        return self.nx

    @property
    def NU(self) -> int:
        return self.nu

    @property
    def NP_exec(self) -> int:
        return self.np_exec

    def to_dict(self) -> dict[str, Any]:
        """Return the complete JSON-compatible prefix identity."""

        return {
            "schema_version": DEVELOPMENT_LAYOUT_SCHEMA_VERSION,
            "scope": DEVELOPMENT_LAYOUT_SCOPE,
            "capacity_identity": {
                "raw_bytes_sha256": self.capacity_contract_sha256,
                "release_frequency_hz": self.release_frequency_hz,
                "v": {"R": self.r_v, "D": self.d_v, "NQ": self.nq_v},
                "omega": {
                    "R": self.r_omega,
                    "D": self.d_omega,
                    "NQ": self.nq_omega,
                },
            },
            "horizon": {
                "N": self.horizon_steps,
                "period_sec": {
                    "numerator": self.release_period_sec.numerator,
                    "denominator": self.release_period_sec.denominator,
                },
            },
            "dimensions": {
                "NX": self.nx,
                "NU": self.nu,
                "NP_exec": self.np_exec,
            },
            "state_layout": {
                "stage_semantics": STAGE_SEMANTICS,
                "ordered": list(self.state_names),
                "offsets": dict(self.state_offsets),
                "dimension": self.nx,
            },
            "control_layout": {
                "ordered": list(self.control_names),
                "offsets": dict(self.control_offsets),
                "dimension": self.nu,
            },
            "execution_parameter_layout": {
                "subsegment_slots": len(self.selector_offsets["v"]),
                "ordered": list(self.execution_parameter_names),
                "offsets": dict(self.execution_parameter_offsets),
                "selector_offsets": {
                    channel: [list(slot) for slot in slots]
                    for channel, slots in self.selector_offsets.items()
                },
                "selector_width": {
                    "v": self.nq_v,
                    "omega": self.nq_omega,
                },
                "dimension": self.np_exec,
            },
        }


def build_development_layout(
    capacity: DevelopmentCapacityContract,
) -> DevelopmentLayout:
    """Expand the sole Stage 3-D prefix layout from pinned typed capacity."""

    checked = _require_pinned_capacity(capacity)

    state_names = list(STATE_ROBOT_PROGRESS) + list(STATE_PUBLISHER)
    state_names.extend(f"older_v[{index}]" for index in range(checked.D_v))
    state_names.extend(f"older_omega[{index}]" for index in range(checked.D_omega))
    state_names.extend(STATE_LIQUID)
    if len(state_names) != checked.NX or len(set(state_names)) != checked.NX:
        raise DevelopmentLayoutError("state expansion does not match the capacity NX")
    state_offsets = {name: index for index, name in enumerate(state_names)}

    control_names = list(CONTROL_ORDER)
    if len(control_names) != checked.NU or len(set(control_names)) != checked.NU:
        raise DevelopmentLayoutError("control expansion does not match the capacity NU")
    control_offsets = {name: index for index, name in enumerate(control_names)}

    parameter_names = [
        "act_inv_tau_v",
        "act_gain_v",
        "act_inv_tau_omega",
        "act_gain_omega",
    ]
    parameter_names.extend(
        f"act_seg_dt[{slot}]" for slot in range(checked.execution_subsegment_slots)
    )
    selector_offsets: dict[str, tuple[tuple[int, ...], ...]] = {}
    for channel, width in (("v", checked.NQ_v), ("omega", checked.NQ_omega)):
        channel_slots: list[tuple[int, ...]] = []
        for slot in range(checked.execution_subsegment_slots):
            indices: list[int] = []
            for selector in range(width):
                indices.append(len(parameter_names))
                parameter_names.append(f"act_sel_{channel}[{slot}][{selector}]")
            channel_slots.append(tuple(indices))
        selector_offsets[channel] = tuple(channel_slots)

    if (
        len(parameter_names) != checked.NP_exec
        or len(set(parameter_names)) != checked.NP_exec
    ):
        raise DevelopmentLayoutError(
            "execution prefix expansion does not match the capacity NP_exec"
        )
    parameter_offsets = {name: index for index, name in enumerate(parameter_names)}

    return DevelopmentLayout(
        capacity_contract_sha256=checked.contract_sha256,
        release_frequency_hz=checked.release_frequency_hz,
        release_period_sec=checked.release_period_sec,
        horizon_steps=HORIZON_STEPS,
        r_v=checked.R_v,
        r_omega=checked.R_omega,
        d_v=checked.D_v,
        d_omega=checked.D_omega,
        nq_v=checked.NQ_v,
        nq_omega=checked.NQ_omega,
        nx=checked.NX,
        nu=checked.NU,
        np_exec=checked.NP_exec,
        state_names=tuple(state_names),
        control_names=tuple(control_names),
        execution_parameter_names=tuple(parameter_names),
        state_offsets=_freeze_mapping(state_offsets),
        control_offsets=_freeze_mapping(control_offsets),
        execution_parameter_offsets=_freeze_mapping(parameter_offsets),
        selector_offsets=_freeze_mapping(selector_offsets),
        _construction_token=_CONSTRUCTION_TOKEN,
    )


def development_layout_from_dict(
    value: Any,
    capacity: DevelopmentCapacityContract,
) -> DevelopmentLayout:
    """Strictly rebuild a serialized layout against the same typed capacity."""

    if type(value) is not dict:
        raise DevelopmentLayoutError("development layout must be a JSON object")
    expected_keys = {
        "schema_version",
        "scope",
        "capacity_identity",
        "horizon",
        "dimensions",
        "state_layout",
        "control_layout",
        "execution_parameter_layout",
    }
    if set(value) != expected_keys:
        raise DevelopmentLayoutError(
            "development layout keys do not match the v1 schema"
        )
    rebuilt = build_development_layout(capacity)
    if not strict_json_equal(value, rebuilt.to_dict()):
        raise DevelopmentLayoutError(
            "development layout does not exactly match the typed capacity expansion"
        )
    return rebuilt


def validate_development_layout_snapshot(
    layout: DevelopmentLayout,
    capacity: DevelopmentCapacityContract,
) -> DevelopmentLayout:
    """Rebuild and verify every serialized field of one typed layout."""

    if type(layout) is not DevelopmentLayout:
        raise DevelopmentLayoutError("layout must be the exact DevelopmentLayout type")
    rebuilt = build_development_layout(capacity)
    try:
        snapshot = layout.to_dict()
    except (AttributeError, KeyError, TypeError, ValueError) as exc:
        raise DevelopmentLayoutError("typed layout snapshot is malformed") from exc
    if not strict_json_equal(snapshot, rebuilt.to_dict()):
        raise DevelopmentLayoutError(
            "typed layout snapshot does not match the pinned capacity expansion"
        )
    return rebuilt
