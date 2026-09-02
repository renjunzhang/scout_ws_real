"""Pinned development-only queue capacity for Stage 3-D code generation.

The release-interval counts are authoritative.  Seconds and all solver-prefix
dimensions are exact derivatives of those integers; no floating-point
``ceil(L_max / dt)`` participates in the contract.  The diagnostic commit is
provenance metadata only and is never opened or queried by this module.
"""

from __future__ import annotations

from dataclasses import InitVar, dataclass, fields, is_dataclass
from fractions import Fraction
from pathlib import Path
from typing import Any

from .identity import (
    IdentityError,
    read_strict_json,
    require_sha256,
    sha256_bytes,
)

DEVELOPMENT_CAPACITY_SCHEMA_VERSION = "spmpc_mainline_development_capacity_v1"
DEVELOPMENT_CAPACITY_ID = "SPMPC-MAINLINE-DEVELOPMENT-CAPACITY-20260903"
DEVELOPMENT_CAPACITY_STATUS = "DEVELOPMENT_CAPACITY_ONLY"
DEVELOPMENT_ARTIFACT_CLASS = "DEV_UNVALIDATED"
DEVELOPMENT_CAPACITY_SCOPE = "DEVELOPMENT_CODEGEN_QUEUE_CAPACITY_ONLY"
DEVELOPMENT_CAPACITY_SHA256 = (
    "22b1806e0c78d46cc6bf51b30002b30d2a42f29352f492d018aa11ec3ac5bca4"
)

RATIONALE_REPOSITORY_REF = "diag/lt-dwa-collision-tracking"
RATIONALE_COMMIT_SHA = "d601e313deefcaeaacdf027296d3ba0ce376b62e"
RATIONALE_ROLE = "DEVELOPMENT_CAPACITY_RATIONALE_ONLY"

RELEASE_FREQUENCY_HZ = 30
RELEASE_PERIOD_SEC = Fraction(1, RELEASE_FREQUENCY_HZ)
BASE_STATE_COUNT = 14
CONTROL_COUNT = 3
EXECUTION_FIXED_SCALAR_COUNT = 7
EXECUTION_SUBSEGMENT_SLOTS = 3

R_V = 12
R_OMEGA = 24
D_V = R_V - 1
D_OMEGA = R_OMEGA - 1
NQ_V = R_V + 1
NQ_OMEGA = R_OMEGA + 1
NX = BASE_STATE_COUNT + D_V + D_OMEGA
NU = CONTROL_COUNT
NP_EXEC = EXECUTION_FIXED_SCALAR_COUNT + EXECUTION_SUBSEGMENT_SLOTS * (NQ_V + NQ_OMEGA)

_CONSTRUCTION_TOKEN = object()


class DevelopmentCapacityError(ValueError):
    """Raised when the development-capacity snapshot is malformed or drifts."""


def _object(value: Any, keys: set[str], label: str) -> dict[str, Any]:
    if type(value) is not dict:
        raise DevelopmentCapacityError(f"{label} must be a JSON object")
    if set(value) != keys:
        raise DevelopmentCapacityError(f"{label} keys do not match the v1 schema")
    return value


def _strict_int(value: Any, label: str, *, minimum: int) -> int:
    if type(value) is not int or value < minimum:
        raise DevelopmentCapacityError(f"{label} must be an integer >= {minimum}")
    return value


def _literal(value: Any, expected: str, label: str) -> str:
    if type(value) is not str or value != expected:
        raise DevelopmentCapacityError(f"{label} does not match the v1 contract")
    return value


def _exact_rational(
    value: Any,
    label: str,
    *,
    numerator: int,
    denominator: int,
) -> None:
    rational = _object(value, {"numerator", "denominator"}, label)
    actual_numerator = _strict_int(
        rational["numerator"], f"{label}.numerator", minimum=0
    )
    actual_denominator = _strict_int(
        rational["denominator"], f"{label}.denominator", minimum=1
    )
    if (actual_numerator, actual_denominator) != (numerator, denominator):
        raise DevelopmentCapacityError(
            f"{label} must use the exact {numerator}/{denominator} representation"
        )


def _validate_source_identity(value: Any) -> None:
    source = _object(
        value,
        {"repository_ref", "commit_sha", "role"},
        "source_identity",
    )
    _literal(
        source["repository_ref"],
        RATIONALE_REPOSITORY_REF,
        "source_identity.repository_ref",
    )
    _literal(
        source["commit_sha"],
        RATIONALE_COMMIT_SHA,
        "source_identity.commit_sha",
    )
    _literal(source["role"], RATIONALE_ROLE, "source_identity.role")


def _validate_release_grid(value: Any) -> int:
    grid = _object(value, {"frequency_hz", "period_sec"}, "release_grid")
    frequency_hz = _strict_int(
        grid["frequency_hz"], "release_grid.frequency_hz", minimum=1
    )
    if frequency_hz != RELEASE_FREQUENCY_HZ:
        raise DevelopmentCapacityError(
            f"release_grid.frequency_hz must equal {RELEASE_FREQUENCY_HZ}"
        )
    _exact_rational(
        grid["period_sec"],
        "release_grid.period_sec",
        numerator=1,
        denominator=frequency_hz,
    )
    return frequency_hz


def _validate_layout_basis(value: Any) -> tuple[int, int, int, int]:
    basis = _object(
        value,
        {
            "base_state_count",
            "control_count",
            "execution_fixed_scalar_count",
            "execution_subsegment_slots",
        },
        "layout_basis",
    )
    labels_and_expected = (
        ("base_state_count", BASE_STATE_COUNT),
        ("control_count", CONTROL_COUNT),
        ("execution_fixed_scalar_count", EXECUTION_FIXED_SCALAR_COUNT),
        ("execution_subsegment_slots", EXECUTION_SUBSEGMENT_SLOTS),
    )
    parsed: list[int] = []
    for key, expected in labels_and_expected:
        actual = _strict_int(basis[key], f"layout_basis.{key}", minimum=1)
        if actual != expected:
            raise DevelopmentCapacityError(f"layout_basis.{key} must equal {expected}")
        parsed.append(actual)
    return parsed[0], parsed[1], parsed[2], parsed[3]


def _validate_channel(
    value: Any,
    channel: str,
    *,
    expected_release_intervals: int,
    frequency_hz: int,
) -> tuple[int, int, int]:
    entry = _object(
        value,
        {"release_intervals", "derived"},
        f"capacity.{channel}",
    )
    release_intervals = _strict_int(
        entry["release_intervals"],
        f"capacity.{channel}.release_intervals",
        minimum=1,
    )
    if release_intervals != expected_release_intervals:
        raise DevelopmentCapacityError(
            f"capacity.{channel}.release_intervals must equal "
            f"{expected_release_intervals}"
        )

    derived = _object(
        entry["derived"],
        {"D", "NQ", "L_max_sec"},
        f"capacity.{channel}.derived",
    )
    delay_state_count = _strict_int(
        derived["D"], f"capacity.{channel}.derived.D", minimum=0
    )
    selector_width = _strict_int(
        derived["NQ"], f"capacity.{channel}.derived.NQ", minimum=1
    )
    expected_delay_state_count = max(0, release_intervals - 1)
    expected_selector_width = release_intervals + 1
    if delay_state_count != expected_delay_state_count:
        raise DevelopmentCapacityError(
            f"capacity.{channel}.derived.D must equal max(0, R-1)"
        )
    if selector_width != expected_selector_width:
        raise DevelopmentCapacityError(f"capacity.{channel}.derived.NQ must equal R+1")
    _exact_rational(
        derived["L_max_sec"],
        f"capacity.{channel}.derived.L_max_sec",
        numerator=release_intervals,
        denominator=frequency_hz,
    )
    return release_intervals, delay_state_count, selector_width


def _validate_development_capacity(value: Any) -> None:
    """Validate the complete v1 schema without touching external state."""

    document = _object(
        value,
        {
            "schema_version",
            "capacity_id",
            "status",
            "artifact_class",
            "scope",
            "source_identity",
            "release_grid",
            "layout_basis",
            "capacity",
            "derived_dimensions",
        },
        "development_capacity",
    )
    _literal(
        document["schema_version"],
        DEVELOPMENT_CAPACITY_SCHEMA_VERSION,
        "schema_version",
    )
    _literal(document["capacity_id"], DEVELOPMENT_CAPACITY_ID, "capacity_id")
    _literal(document["status"], DEVELOPMENT_CAPACITY_STATUS, "status")
    _literal(
        document["artifact_class"],
        DEVELOPMENT_ARTIFACT_CLASS,
        "artifact_class",
    )
    _literal(document["scope"], DEVELOPMENT_CAPACITY_SCOPE, "scope")
    _validate_source_identity(document["source_identity"])
    frequency_hz = _validate_release_grid(document["release_grid"])
    base_state_count, control_count, fixed_scalars, segment_slots = (
        _validate_layout_basis(document["layout_basis"])
    )

    capacity = _object(document["capacity"], {"v", "omega"}, "capacity")
    _, d_v, nq_v = _validate_channel(
        capacity["v"],
        "v",
        expected_release_intervals=R_V,
        frequency_hz=frequency_hz,
    )
    _, d_omega, nq_omega = _validate_channel(
        capacity["omega"],
        "omega",
        expected_release_intervals=R_OMEGA,
        frequency_hz=frequency_hz,
    )

    dimensions = _object(
        document["derived_dimensions"],
        {"NX", "NU", "NP_exec"},
        "derived_dimensions",
    )
    nx = _strict_int(dimensions["NX"], "derived_dimensions.NX", minimum=1)
    nu = _strict_int(dimensions["NU"], "derived_dimensions.NU", minimum=1)
    np_exec = _strict_int(
        dimensions["NP_exec"], "derived_dimensions.NP_exec", minimum=1
    )
    expected_nx = base_state_count + d_v + d_omega
    expected_nu = control_count
    expected_np_exec = fixed_scalars + segment_slots * (nq_v + nq_omega)
    if nx != expected_nx:
        raise DevelopmentCapacityError(
            "derived_dimensions.NX does not match base_state_count+D_v+D_omega"
        )
    if nu != expected_nu:
        raise DevelopmentCapacityError(
            "derived_dimensions.NU does not match control_count"
        )
    if np_exec != expected_np_exec:
        raise DevelopmentCapacityError(
            "derived_dimensions.NP_exec does not match 7+3*(NQ_v+NQ_omega)"
        )


@dataclass(frozen=True)
class ChannelCapacity:
    """One immutable fixed-width delay channel."""

    release_intervals: int
    delay_state_count: int
    selector_width: int
    l_max_sec: Fraction


@dataclass(frozen=True)
class DevelopmentCapacityContract:
    """Typed view of the pinned v1 development-capacity bytes."""

    contract_sha256: str
    release_frequency_hz: int
    release_period_sec: Fraction
    v: ChannelCapacity
    omega: ChannelCapacity
    base_state_count: int
    control_count: int
    execution_fixed_scalar_count: int
    execution_subsegment_slots: int
    nx: int
    nu: int
    np_exec: int
    _construction_token: InitVar[object] = None

    def __post_init__(self, _construction_token: object) -> None:
        if _construction_token is not _CONSTRUCTION_TOKEN:
            raise DevelopmentCapacityError(
                "DevelopmentCapacityContract requires the pinned loader"
            )

    @property
    def R_v(self) -> int:
        return self.v.release_intervals

    @property
    def R_omega(self) -> int:
        return self.omega.release_intervals

    @property
    def D_v(self) -> int:
        return self.v.delay_state_count

    @property
    def D_omega(self) -> int:
        return self.omega.delay_state_count

    @property
    def NQ_v(self) -> int:
        return self.v.selector_width

    @property
    def NQ_omega(self) -> int:
        return self.omega.selector_width

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
        """Return the exact semantic document; byte identity stays separate."""

        return {
            "schema_version": DEVELOPMENT_CAPACITY_SCHEMA_VERSION,
            "capacity_id": DEVELOPMENT_CAPACITY_ID,
            "status": DEVELOPMENT_CAPACITY_STATUS,
            "artifact_class": DEVELOPMENT_ARTIFACT_CLASS,
            "scope": DEVELOPMENT_CAPACITY_SCOPE,
            "source_identity": {
                "repository_ref": RATIONALE_REPOSITORY_REF,
                "commit_sha": RATIONALE_COMMIT_SHA,
                "role": RATIONALE_ROLE,
            },
            "release_grid": {
                "frequency_hz": self.release_frequency_hz,
                "period_sec": {
                    "numerator": 1,
                    "denominator": self.release_frequency_hz,
                },
            },
            "layout_basis": {
                "base_state_count": self.base_state_count,
                "control_count": self.control_count,
                "execution_fixed_scalar_count": self.execution_fixed_scalar_count,
                "execution_subsegment_slots": self.execution_subsegment_slots,
            },
            "capacity": {
                "v": self._channel_to_dict(self.v),
                "omega": self._channel_to_dict(self.omega),
            },
            "derived_dimensions": {
                "NX": self.nx,
                "NU": self.nu,
                "NP_exec": self.np_exec,
            },
        }

    def _channel_to_dict(self, channel: ChannelCapacity) -> dict[str, Any]:
        return {
            "release_intervals": channel.release_intervals,
            "derived": {
                "D": channel.delay_state_count,
                "NQ": channel.selector_width,
                "L_max_sec": {
                    "numerator": channel.release_intervals,
                    "denominator": self.release_frequency_hz,
                },
            },
        }


def _new_pinned_contract() -> DevelopmentCapacityContract:
    return DevelopmentCapacityContract(
        contract_sha256=DEVELOPMENT_CAPACITY_SHA256,
        release_frequency_hz=RELEASE_FREQUENCY_HZ,
        release_period_sec=RELEASE_PERIOD_SEC,
        v=ChannelCapacity(R_V, D_V, NQ_V, Fraction(R_V, RELEASE_FREQUENCY_HZ)),
        omega=ChannelCapacity(
            R_OMEGA,
            D_OMEGA,
            NQ_OMEGA,
            Fraction(R_OMEGA, RELEASE_FREQUENCY_HZ),
        ),
        base_state_count=BASE_STATE_COUNT,
        control_count=CONTROL_COUNT,
        execution_fixed_scalar_count=EXECUTION_FIXED_SCALAR_COUNT,
        execution_subsegment_slots=EXECUTION_SUBSEGMENT_SLOTS,
        nx=NX,
        nu=NU,
        np_exec=NP_EXEC,
        _construction_token=_CONSTRUCTION_TOKEN,
    )


def _strict_typed_equal(left: Any, right: Any) -> bool:
    if type(left) is not type(right):
        return False
    if is_dataclass(left):
        return all(
            _strict_typed_equal(
                getattr(left, field.name),
                getattr(right, field.name),
            )
            for field in fields(left)
        )
    return bool(left == right)


def require_pinned_development_capacity(
    value: Any,
) -> DevelopmentCapacityContract:
    """Require complete semantic equality with the loader's pinned snapshot.

    A Python-private construction token is only an API guard, not an authority
    boundary.  Consumers call this function so a forged or force-mutated
    dataclass carrying the right digest string cannot alter layout dimensions.
    """

    if type(value) is not DevelopmentCapacityContract:
        raise DevelopmentCapacityError(
            "capacity must come from the pinned development-capacity loader"
        )
    if not _strict_typed_equal(value, _new_pinned_contract()):
        raise DevelopmentCapacityError(
            "typed development capacity does not match the complete pinned snapshot"
        )
    return value


def development_capacity_from_dict(
    value: Any,
    raw_bytes_sha256: str,
) -> DevelopmentCapacityContract:
    """Parse one embedded capacity snapshot against its exact pinned bytes.

    The capacity JSON intentionally does not contain its own digest.  Artifact
    documents carry that digest beside the embedded snapshot, so callers must
    provide it explicitly.  The returned object is the same pinned typed
    authority used by :func:`load_development_capacity`.
    """

    _validate_development_capacity(value)
    try:
        digest = require_sha256(raw_bytes_sha256, "development capacity raw identity")
    except IdentityError as exc:
        raise DevelopmentCapacityError(str(exc)) from exc
    if digest != DEVELOPMENT_CAPACITY_SHA256:
        raise DevelopmentCapacityError(
            "development capacity raw identity is not the pinned immutable v1 snapshot"
        )
    contract = _new_pinned_contract()
    if value != contract.to_dict():
        raise DevelopmentCapacityError(
            "development capacity does not match its typed representation"
        )
    return require_pinned_development_capacity(contract)


def validate_development_capacity_document(
    value: Any,
    raw_bytes_sha256: str,
) -> None:
    """Validate an embedded capacity snapshot and its pinned raw SHA."""

    development_capacity_from_dict(value, raw_bytes_sha256)


def load_development_capacity(
    path: Path | str,
) -> DevelopmentCapacityContract:
    """Load the immutable v1 capacity without inspecting any external data."""

    try:
        value, payload = read_strict_json(path, label="development capacity")
    except IdentityError as exc:
        raise DevelopmentCapacityError(str(exc)) from exc
    digest = sha256_bytes(payload)
    return development_capacity_from_dict(value, digest)


__all__ = [
    "BASE_STATE_COUNT",
    "CONTROL_COUNT",
    "DEVELOPMENT_ARTIFACT_CLASS",
    "DEVELOPMENT_CAPACITY_ID",
    "DEVELOPMENT_CAPACITY_SCHEMA_VERSION",
    "DEVELOPMENT_CAPACITY_SCOPE",
    "DEVELOPMENT_CAPACITY_SHA256",
    "DEVELOPMENT_CAPACITY_STATUS",
    "D_OMEGA",
    "D_V",
    "EXECUTION_FIXED_SCALAR_COUNT",
    "EXECUTION_SUBSEGMENT_SLOTS",
    "NP_EXEC",
    "NQ_OMEGA",
    "NQ_V",
    "NU",
    "NX",
    "R_OMEGA",
    "R_V",
    "ChannelCapacity",
    "DevelopmentCapacityContract",
    "DevelopmentCapacityError",
    "development_capacity_from_dict",
    "load_development_capacity",
    "require_pinned_development_capacity",
    "validate_development_capacity_document",
]
