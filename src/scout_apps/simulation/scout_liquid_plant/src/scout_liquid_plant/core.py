"""Pure-Python dynamics for the development liquid-plant node.

This module deliberately owns its state and equations.  It has no ROS or
controller dependency so unit tests can exercise the plant without a running
ROS master.  The model is an unvalidated multi-mode surrogate; it is not a
claim of physical liquid fidelity.
"""

from __future__ import division

from dataclasses import dataclass
import math
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple


class PlantConfigError(ValueError):
    """Raised when a development-only plant configuration is unsafe to use."""


def _finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PlantConfigError("{} must be a finite number".format(name))
    result = float(value)
    if not math.isfinite(result):
        raise PlantConfigError("{} must be a finite number".format(name))
    return result


def _positive(value: Any, name: str) -> float:
    result = _finite_number(value, name)
    if result <= 0.0:
        raise PlantConfigError("{} must be positive".format(name))
    return result


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise PlantConfigError("{} must be an object".format(name))
    return value


@dataclass(frozen=True)
class ModeParameters:
    """One independent damped liquid mode in the plant surrogate."""

    mode_id: str
    natural_frequency_radps: float
    damping_ratio: float
    input_gain: float
    height_gain: float
    cubic_stiffness_m2s2: float


@dataclass(frozen=True)
class PlantParameters:
    """Validated configuration consumed by :class:`LiquidPlant`."""

    schema_version: int
    condition_template_id: str
    odom_topic: str
    height_topic: str
    state_topic: str
    metadata_topic: str
    integration_step_sec: float
    max_odom_dt_sec: float
    container_radius_m: float
    offset_x_m: float
    offset_y_m: float
    static_liquid_height_m: float
    liquid_density_kgm3: float
    rotation_height_gain_s2: float
    max_modal_displacement_m: float
    max_modal_velocity_mps: float
    height_limit_m: float
    modes: Tuple[ModeParameters, ...]

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any]) -> "PlantParameters":
        """Validate a node parameter mapping and enforce development boundaries."""

        data = _mapping(raw, "plant configuration")
        if data.get("development_only") is not True:
            raise PlantConfigError("development_only must be true")
        if data.get("formal") is not False:
            raise PlantConfigError("formal must be false for this package")
        if data.get("fidelity_validation_status") != "UNVALIDATED":
            raise PlantConfigError(
                "fidelity_validation_status must remain UNVALIDATED for this package"
            )

        schema_version = data.get("schema_version")
        if isinstance(schema_version, bool) or not isinstance(schema_version, int):
            raise PlantConfigError("schema_version must be an integer")
        if schema_version != 1:
            raise PlantConfigError("unsupported schema_version: {}".format(schema_version))

        condition_template_id = data.get("condition_template_id")
        if not isinstance(condition_template_id, str) or not condition_template_id:
            raise PlantConfigError("condition_template_id must be a non-empty string")

        required_topics = {
            "odom_topic": "/odom",
            "height_topic": "/sim_truth/liquid_height",
            "state_topic": "/sim_truth/liquid_state",
            "metadata_topic": "/sim_truth/liquid_metadata",
        }
        topics: Dict[str, str] = {}
        for key, expected in required_topics.items():
            value = data.get(key)
            if value != expected:
                raise PlantConfigError("{} must be {}".format(key, expected))
            topics[key] = value

        container = _mapping(data.get("container"), "container")
        container_radius_m = _positive(
            container.get("container_radius_m"), "container.container_radius_m"
        )
        offset_x_m = _finite_number(container.get("offset_x_m", 0.0), "container.offset_x_m")
        offset_y_m = _finite_number(container.get("offset_y_m", 0.0), "container.offset_y_m")
        static_liquid_height_m = _positive(
            container.get("static_liquid_height_m"), "container.static_liquid_height_m"
        )
        liquid_density_kgm3 = _positive(
            container.get("liquid_density_kgm3"), "container.liquid_density_kgm3"
        )

        modes_value = data.get("modes")
        if not isinstance(modes_value, Sequence) or isinstance(modes_value, (str, bytes)):
            raise PlantConfigError("modes must be an array")
        if len(modes_value) < 2:
            raise PlantConfigError("development liquid plant requires at least two modes")

        modes: List[ModeParameters] = []
        seen_ids = set()
        for index, item in enumerate(modes_value):
            mode = _mapping(item, "modes[{}]".format(index))
            mode_id = mode.get("mode_id")
            if not isinstance(mode_id, str) or not mode_id:
                raise PlantConfigError("modes[{}].mode_id must be a non-empty string".format(index))
            if mode_id in seen_ids:
                raise PlantConfigError("mode_id values must be unique")
            seen_ids.add(mode_id)
            damping = _finite_number(mode.get("damping_ratio"), "modes[{}].damping_ratio".format(index))
            if damping < 0.0 or damping >= 1.0:
                raise PlantConfigError("modes[{}].damping_ratio must be in [0, 1)".format(index))
            cubic = _finite_number(
                mode.get("cubic_stiffness_m2s2", 0.0),
                "modes[{}].cubic_stiffness_m2s2".format(index),
            )
            if cubic < 0.0:
                raise PlantConfigError("modes[{}].cubic_stiffness_m2s2 must be non-negative".format(index))
            modes.append(
                ModeParameters(
                    mode_id=mode_id,
                    natural_frequency_radps=_positive(
                        mode.get("natural_frequency_radps"),
                        "modes[{}].natural_frequency_radps".format(index),
                    ),
                    damping_ratio=damping,
                    input_gain=_positive(
                        mode.get("input_gain"), "modes[{}].input_gain".format(index)
                    ),
                    height_gain=_positive(
                        mode.get("height_gain"), "modes[{}].height_gain".format(index)
                    ),
                    cubic_stiffness_m2s2=cubic,
                )
            )

        integration_step_sec = _positive(data.get("integration_step_sec"), "integration_step_sec")
        max_odom_dt_sec = _positive(data.get("max_odom_dt_sec"), "max_odom_dt_sec")
        if integration_step_sec > max_odom_dt_sec:
            raise PlantConfigError("integration_step_sec cannot exceed max_odom_dt_sec")
        rotation_height_gain_s2 = _finite_number(
            data.get("rotation_height_gain_s2", 0.0), "rotation_height_gain_s2"
        )
        if rotation_height_gain_s2 < 0.0:
            raise PlantConfigError("rotation_height_gain_s2 must be non-negative")

        return cls(
            schema_version=schema_version,
            condition_template_id=condition_template_id,
            odom_topic=topics["odom_topic"],
            height_topic=topics["height_topic"],
            state_topic=topics["state_topic"],
            metadata_topic=topics["metadata_topic"],
            integration_step_sec=integration_step_sec,
            max_odom_dt_sec=max_odom_dt_sec,
            container_radius_m=container_radius_m,
            offset_x_m=offset_x_m,
            offset_y_m=offset_y_m,
            static_liquid_height_m=static_liquid_height_m,
            liquid_density_kgm3=liquid_density_kgm3,
            rotation_height_gain_s2=rotation_height_gain_s2,
            max_modal_displacement_m=_positive(
                data.get("max_modal_displacement_m"), "max_modal_displacement_m"
            ),
            max_modal_velocity_mps=_positive(
                data.get("max_modal_velocity_mps"), "max_modal_velocity_mps"
            ),
            height_limit_m=_positive(data.get("height_limit_m"), "height_limit_m"),
            modes=tuple(modes),
        )

    def public_metadata(self) -> Dict[str, Any]:
        """Return unambiguous, non-promotional metadata for the ROS publisher."""

        return {
            "schema_version": self.schema_version,
            "package": "scout_liquid_plant",
            "plant_model": "independent_multimode_development_surrogate",
            "development_only": True,
            "formal": False,
            "fidelity_validation_status": "UNVALIDATED",
            "physical_primary_eligible": False,
            "condition_template_id": self.condition_template_id,
            "input": {
                "topic": self.odom_topic,
                "message_type": "nav_msgs/Odometry",
                "semantic": "executed_simulated_base_motion_only",
            },
            "outputs": {
                "height_topic": self.height_topic,
                "state_topic": self.state_topic,
                "metadata_topic": self.metadata_topic,
            },
            "state_initialization": "zero_modal_state_at_node_start",
            "container_template": {
                "container_radius_m": self.container_radius_m,
                "static_liquid_height_m": self.static_liquid_height_m,
                "liquid_density_kgm3": self.liquid_density_kgm3,
                "offset_x_m": self.offset_x_m,
                "offset_y_m": self.offset_y_m,
            },
            "integration_step_sec": self.integration_step_sec,
            "max_odom_dt_sec": self.max_odom_dt_sec,
            "mode_ids": [mode.mode_id for mode in self.modes],
            "warning": (
                "UNVALIDATED development signal. It must not be used as a formal "
                "physical-primary outcome or as a controller input."
            ),
        }


@dataclass(frozen=True)
class OdomSample:
    """The executed body motion extracted from one odometry message."""

    stamp_sec: float
    yaw_rad: float
    linear_x_mps: float
    linear_y_mps: float
    yaw_rate_radps: float

    def validate(self) -> None:
        for name in (
            "stamp_sec",
            "yaw_rad",
            "linear_x_mps",
            "linear_y_mps",
            "yaw_rate_radps",
        ):
            _finite_number(getattr(self, name), "odom." + name)


@dataclass
class _ModeState:
    qx_m: float = 0.0
    vx_mps: float = 0.0
    qy_m: float = 0.0
    vy_mps: float = 0.0


@dataclass(frozen=True)
class PlantStep:
    """A pure model output ready for ROS-message conversion."""

    integrated: bool
    initialized: bool
    reason: str
    stamp_sec: float
    dt_sec: float
    ax_body_mps2: float
    ay_body_mps2: float
    yaw_rate_radps: float
    yaw_accel_radps2: float
    liquid_height_m: float
    modal_height_m: float
    rotation_height_m: float
    state_values: Tuple[float, ...]


@dataclass(frozen=True)
class _PreviousMotion:
    stamp_sec: float
    world_vx_mps: float
    world_vy_mps: float
    yaw_rate_radps: float


class LiquidPlant:
    """Independent multi-mode plant driven only by executed odometry motion."""

    _BASE_STATE_FIELDS = (
        "stamp_sec",
        "input_dt_sec",
        "ax_body_mps2",
        "ay_body_mps2",
        "yaw_rate_radps",
        "yaw_accel_radps2",
        "liquid_height_m",
        "modal_height_m",
        "rotation_height_m",
    )

    def __init__(self, parameters: PlantParameters):
        self.parameters = parameters
        self._states = [_ModeState() for _ in parameters.modes]
        self._previous: Optional[_PreviousMotion] = None

    def reset(self) -> None:
        """Reset only the plant's own state; this never touches a controller."""

        self._states = [_ModeState() for _ in self.parameters.modes]
        self._previous = None

    def state_field_names(self) -> List[str]:
        fields = list(self._BASE_STATE_FIELDS)
        for mode in self.parameters.modes:
            prefix = "mode_" + mode.mode_id
            fields.extend(
                [
                    prefix + "_qx_m",
                    prefix + "_vx_mps",
                    prefix + "_qy_m",
                    prefix + "_vy_mps",
                ]
            )
        return fields

    def step(self, sample: OdomSample) -> PlantStep:
        """Advance from actual odometry; no command or controller state is read."""

        sample.validate()
        world_vx, world_vy = self._world_velocity(sample)
        if self._previous is None:
            self._previous = _PreviousMotion(
                stamp_sec=sample.stamp_sec,
                world_vx_mps=world_vx,
                world_vy_mps=world_vy,
                yaw_rate_radps=sample.yaw_rate_radps,
            )
            return self._snapshot(
                sample=sample,
                dt_sec=0.0,
                ax_body=0.0,
                ay_body=0.0,
                yaw_accel=0.0,
                integrated=False,
                initialized=True,
                reason="INITIALIZED_NO_DERIVATIVE",
            )

        previous = self._previous
        dt_sec = sample.stamp_sec - previous.stamp_sec
        if dt_sec <= 0.0:
            return self._snapshot(
                sample=sample,
                dt_sec=dt_sec,
                ax_body=0.0,
                ay_body=0.0,
                yaw_accel=0.0,
                integrated=False,
                initialized=False,
                reason="REJECTED_NONMONOTONIC_ODOM_TIME",
            )
        if dt_sec > self.parameters.max_odom_dt_sec:
            self._previous = _PreviousMotion(
                stamp_sec=sample.stamp_sec,
                world_vx_mps=world_vx,
                world_vy_mps=world_vy,
                yaw_rate_radps=sample.yaw_rate_radps,
            )
            return self._snapshot(
                sample=sample,
                dt_sec=dt_sec,
                ax_body=0.0,
                ay_body=0.0,
                yaw_accel=0.0,
                integrated=False,
                initialized=False,
                reason="REJECTED_ODOM_GAP",
            )

        ax_world = (world_vx - previous.world_vx_mps) / dt_sec
        ay_world = (world_vy - previous.world_vy_mps) / dt_sec
        yaw_cos = math.cos(sample.yaw_rad)
        yaw_sin = math.sin(sample.yaw_rad)
        ax_body = yaw_cos * ax_world + yaw_sin * ay_world
        ay_body = -yaw_sin * ax_world + yaw_cos * ay_world
        yaw_accel = (sample.yaw_rate_radps - previous.yaw_rate_radps) / dt_sec
        container_ax = (
            ax_body
            - yaw_accel * self.parameters.offset_y_m
            - sample.yaw_rate_radps * sample.yaw_rate_radps * self.parameters.offset_x_m
        )
        container_ay = (
            ay_body
            + yaw_accel * self.parameters.offset_x_m
            - sample.yaw_rate_radps * sample.yaw_rate_radps * self.parameters.offset_y_m
        )

        substeps = max(1, int(math.ceil(dt_sec / self.parameters.integration_step_sec)))
        integration_dt = dt_sec / float(substeps)
        for _ in range(substeps):
            for index, mode in enumerate(self.parameters.modes):
                self._states[index] = self._advance_mode(
                    self._states[index], mode, container_ax, container_ay, integration_dt
                )

        self._previous = _PreviousMotion(
            stamp_sec=sample.stamp_sec,
            world_vx_mps=world_vx,
            world_vy_mps=world_vy,
            yaw_rate_radps=sample.yaw_rate_radps,
        )
        return self._snapshot(
            sample=sample,
            dt_sec=dt_sec,
            ax_body=ax_body,
            ay_body=ay_body,
            yaw_accel=yaw_accel,
            integrated=True,
            initialized=False,
            reason="INTEGRATED_EXECUTED_ODOM",
        )

    @staticmethod
    def _world_velocity(sample: OdomSample) -> Tuple[float, float]:
        yaw_cos = math.cos(sample.yaw_rad)
        yaw_sin = math.sin(sample.yaw_rad)
        return (
            yaw_cos * sample.linear_x_mps - yaw_sin * sample.linear_y_mps,
            yaw_sin * sample.linear_x_mps + yaw_cos * sample.linear_y_mps,
        )

    def _advance_mode(
        self,
        state: _ModeState,
        mode: ModeParameters,
        ax_mps2: float,
        ay_mps2: float,
        dt_sec: float,
    ) -> _ModeState:
        """RK4 integrate a mode with independent nonlinear restoring force."""

        initial = (state.qx_m, state.vx_mps, state.qy_m, state.vy_mps)

        def derivative(values: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
            qx, vx, qy, vy = values
            omega = mode.natural_frequency_radps
            damping = 2.0 * mode.damping_ratio * omega
            return (
                vx,
                -damping * vx
                - omega * omega * qx
                - mode.cubic_stiffness_m2s2 * qx * qx * qx
                - mode.input_gain * ax_mps2,
                vy,
                -damping * vy
                - omega * omega * qy
                - mode.cubic_stiffness_m2s2 * qy * qy * qy
                - mode.input_gain * ay_mps2,
            )

        def add(values: Tuple[float, float, float, float], slope: Tuple[float, float, float, float], scale: float) -> Tuple[float, float, float, float]:
            return tuple(values[index] + scale * slope[index] for index in range(4))  # type: ignore[return-value]

        k1 = derivative(initial)
        k2 = derivative(add(initial, k1, 0.5 * dt_sec))
        k3 = derivative(add(initial, k2, 0.5 * dt_sec))
        k4 = derivative(add(initial, k3, dt_sec))
        advanced = tuple(
            initial[index]
            + (dt_sec / 6.0)
            * (k1[index] + 2.0 * k2[index] + 2.0 * k3[index] + k4[index])
            for index in range(4)
        )

        displacement_limit = self.parameters.max_modal_displacement_m
        velocity_limit = self.parameters.max_modal_velocity_mps
        return _ModeState(
            qx_m=max(-displacement_limit, min(displacement_limit, advanced[0])),
            vx_mps=max(-velocity_limit, min(velocity_limit, advanced[1])),
            qy_m=max(-displacement_limit, min(displacement_limit, advanced[2])),
            vy_mps=max(-velocity_limit, min(velocity_limit, advanced[3])),
        )

    def _snapshot(
        self,
        sample: OdomSample,
        dt_sec: float,
        ax_body: float,
        ay_body: float,
        yaw_accel: float,
        integrated: bool,
        initialized: bool,
        reason: str,
    ) -> PlantStep:
        modal_terms = [
            mode.height_gain * math.hypot(state.qx_m, state.qy_m)
            for mode, state in zip(self.parameters.modes, self._states)
        ]
        modal_height = math.sqrt(sum(term * term for term in modal_terms))
        rotation_height = max(
            0.0,
            self.parameters.rotation_height_gain_s2 * sample.yaw_rate_radps * sample.yaw_rate_radps,
        )
        liquid_height = min(self.parameters.height_limit_m, modal_height + rotation_height)
        values: List[float] = [
            sample.stamp_sec,
            dt_sec,
            ax_body,
            ay_body,
            sample.yaw_rate_radps,
            yaw_accel,
            liquid_height,
            modal_height,
            rotation_height,
        ]
        for state in self._states:
            values.extend([state.qx_m, state.vx_mps, state.qy_m, state.vy_mps])
        return PlantStep(
            integrated=integrated,
            initialized=initialized,
            reason=reason,
            stamp_sec=sample.stamp_sec,
            dt_sec=dt_sec,
            ax_body_mps2=ax_body,
            ay_body_mps2=ay_body,
            yaw_rate_radps=sample.yaw_rate_radps,
            yaw_accel_radps2=yaw_accel,
            liquid_height_m=liquid_height,
            modal_height_m=modal_height,
            rotation_height_m=rotation_height,
            state_values=tuple(values),
        )
