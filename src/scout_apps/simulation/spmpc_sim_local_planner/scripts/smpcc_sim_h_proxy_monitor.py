#!/usr/bin/env python3
"""Simulation-owned, record-only H_proxy monitor.

This process deliberately has one input, executed ``nav_msgs/Odometry``.  It
never subscribes to controller state, commands, H_modal, or plant truth, and
it never publishes a command.  The output ``/slosh/height`` is therefore a
modal model proxy for mechanism diagnostics only -- never liquid plant truth
or a physical-primary measurement.

The modal equations and constants are frozen locally in the simulation fork.
That avoids linking the real-stack ``slosh_models`` library while preserving a
stable H_proxy interface for development and source-separated R8 releases.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence


MODAL_ROOTS: Sequence[float] = (1.8412, 5.3314, 8.5363, 11.7060, 14.8636)
GRAVITY = 9.81


class ConfigurationError(RuntimeError):
    """The monitor must not silently run with an invalid proxy model."""


@dataclass
class ModalProxy:
    radius: float
    liquid_height: float
    liquid_density: float
    mode_index: int
    damping_ratio: float
    model_dt: float
    use_linear_model: bool
    use_parabola_term: bool
    offset_x: float = 0.0
    offset_y: float = 0.0

    def __post_init__(self) -> None:
        values = (self.radius, self.liquid_height, self.liquid_density,
                  self.damping_ratio, self.model_dt, self.offset_x,
                  self.offset_y)
        if (not all(math.isfinite(value) for value in values) or
                self.radius <= 0.0 or self.liquid_height <= 0.0 or
                self.liquid_density <= 0.0 or self.model_dt <= 1e-4 or
                self.mode_index < 1 or self.mode_index > len(MODAL_ROOTS)):
            raise ConfigurationError("invalid frozen H_proxy modal parameters")
        self.x: List[float] = [0.0, 0.0, 0.0, 0.0]
        xi = MODAL_ROOTS[self.mode_index - 1]
        tanh_value = math.tanh(xi * self.liquid_height / self.radius)
        self.omega_n = math.sqrt(GRAVITY * (xi / self.radius) * tanh_value)
        liquid_mass = self.liquid_density * math.pi * self.radius * self.radius * self.liquid_height
        modal_mass = liquid_mass * (2.0 * self.radius * tanh_value) / (
            xi * self.liquid_height * (xi * xi - 1.0)
        )
        numerator = 4.0 if self.use_linear_model else xi * xi
        self.height_coeff = numerator * self.liquid_height * modal_mass / (liquid_mass * self.radius)
        if not math.isfinite(self.omega_n) or not math.isfinite(self.height_coeff):
            raise ConfigurationError("non-finite frozen H_proxy modal coefficients")

    def reset(self) -> None:
        self.x = [0.0, 0.0, 0.0, 0.0]

    def _derivative(self, state: Sequence[float], ax: float, ay: float) -> List[float]:
        omega_squared = self.omega_n * self.omega_n
        damping = 2.0 * self.damping_ratio * self.omega_n
        return [
            state[1],
            -omega_squared * state[0] - damping * state[1] - ax,
            state[3],
            -omega_squared * state[2] - damping * state[3] - ay,
        ]

    def update(self, ax: float, ay: float, omega_z: float, alpha_z: float) -> None:
        # Same container-offset excitation correction as the frozen modal
        # equation, followed by deterministic RK4 at the declared model_dt.
        corrected_ax = ax - alpha_z * self.offset_y - omega_z * omega_z * self.offset_x
        corrected_ay = ay + alpha_z * self.offset_x - omega_z * omega_z * self.offset_y
        dt = self.model_dt
        k1 = self._derivative(self.x, corrected_ax, corrected_ay)
        x2 = [value + 0.5 * dt * slope for value, slope in zip(self.x, k1)]
        k2 = self._derivative(x2, corrected_ax, corrected_ay)
        x3 = [value + 0.5 * dt * slope for value, slope in zip(self.x, k2)]
        k3 = self._derivative(x3, corrected_ax, corrected_ay)
        x4 = [value + dt * slope for value, slope in zip(self.x, k3)]
        k4 = self._derivative(x4, corrected_ax, corrected_ay)
        next_state = [
            value + dt * (a + 2.0 * b + 2.0 * c + d) / 6.0
            for value, a, b, c, d in zip(self.x, k1, k2, k3, k4)
        ]
        if all(math.isfinite(value) for value in next_state):
            self.x = next_state

    def height(self, omega_z: float) -> float:
        modal = self.height_coeff * math.hypot(self.x[0], self.x[2])
        parabola = 0.0
        if self.use_parabola_term:
            parabola = self.radius * self.radius * omega_z * omega_z / (4.0 * GRAVITY)
        return max(0.0, modal + parabola)


class HProxyMonitor:
    """ROS adapter around the isolated modal proxy; no controller input ABI."""

    def __init__(self) -> None:
        import rospy  # type: ignore
        from nav_msgs.msg import Odometry  # type: ignore
        from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension  # type: ignore

        self.rospy = rospy
        self.Float32 = Float32
        self.Float32MultiArray = Float32MultiArray
        self.MultiArrayDimension = MultiArrayDimension
        subscribe_cmd_vel_debug = bool(rospy.get_param("~subscribe_cmd_vel_debug", False))
        if subscribe_cmd_vel_debug:
            raise ConfigurationError("H_proxy monitor refuses cmd_vel subscription")
        self.odom_topic = str(rospy.get_param("~odom_topic", "/odom"))
        self.min_dt = float(rospy.get_param("~min_dt", 0.001))
        self.max_dt = float(rospy.get_param("~max_dt", 0.1))
        self.filter_alpha = min(1.0, max(0.0, float(rospy.get_param("~accel_filter_alpha", 0.3))))
        self.model = ModalProxy(
            radius=float(rospy.get_param("~container_radius", 0.0185)),
            liquid_height=float(rospy.get_param("~liquid_height", 0.058)),
            liquid_density=float(rospy.get_param("~liquid_density", 1000.0)),
            mode_index=int(rospy.get_param("~mode_index", 1)),
            damping_ratio=float(rospy.get_param("~damping_ratio", 0.05)),
            model_dt=float(rospy.get_param("~model_dt", 0.02)),
            use_linear_model=bool(rospy.get_param("~use_linear_model", True)),
            use_parabola_term=bool(rospy.get_param("~use_parabola_term", False)),
            offset_x=float(rospy.get_param("~offset_x", 0.0)),
            offset_y=float(rospy.get_param("~offset_y", 0.0)),
        )
        self.height_pub = rospy.Publisher("height", Float32, queue_size=10)
        self.state_pub = rospy.Publisher("state", Float32MultiArray, queue_size=10)
        self.debug_pub = rospy.Publisher("debug", Float32MultiArray, queue_size=10)
        self.previous_stamp = None
        self.previous_v = 0.0
        self.previous_omega = 0.0
        self.ax_filt = 0.0
        self.ay_filt = 0.0
        self.alpha_filt = 0.0
        self.last_dt = 0.0
        self.last_v = 0.0
        self.last_omega = 0.0
        self.update_count = 0
        self.episode_start = rospy.Time.now()
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=50)

    def _filter(self, raw: float, previous: float) -> float:
        return self.filter_alpha * raw + (1.0 - self.filter_alpha) * previous

    def _array(self, label: str, values: Sequence[float]):
        message = self.Float32MultiArray()
        dimension = self.MultiArrayDimension()
        dimension.label, dimension.size, dimension.stride = label, len(values), len(values)
        message.layout.dim = [dimension]
        message.data = [float(value) for value in values]
        return message

    def publish(self, stamp, omega_z: float) -> None:
        height = self.model.height(omega_z)
        self.height_pub.publish(self.Float32(data=float(height)))
        self.state_pub.publish(self._array("x_m,vx_mps,y_m,vy_mps", self.model.x))
        elapsed = max(0.0, (stamp - self.episode_start).to_sec())
        # cmd_vel fields are intentionally fixed to zero: this node has no
        # command subscription and its only source is executed odometry.
        self.debug_pub.publish(self._array(
            "stamp_rel_sec,dt,v_odom,omega_odom,ax_est,ay_est,alpha_est,height_m,height_mm,cmd_v,cmd_omega,update_count,reset_count",
            (elapsed, self.last_dt, self.last_v, self.last_omega,
             self.ax_filt, self.ay_filt, self.alpha_filt, height, 1000.0 * height,
             0.0, 0.0, float(self.update_count), 0.0),
        ))

    def odom_callback(self, message) -> None:
        stamp = message.header.stamp
        if stamp.is_zero():
            stamp = self.rospy.Time.now()
        velocity = float(message.twist.twist.linear.x)
        omega_z = float(message.twist.twist.angular.z)
        self.last_v, self.last_omega = velocity, omega_z
        if not math.isfinite(velocity) or not math.isfinite(omega_z):
            return
        if self.previous_stamp is None:
            self.previous_stamp, self.previous_v, self.previous_omega = stamp, velocity, omega_z
            self.publish(stamp, omega_z)
            return
        dt = (stamp - self.previous_stamp).to_sec()
        self.last_dt = dt
        if self.min_dt <= dt <= self.max_dt:
            raw_ax = (velocity - self.previous_v) / dt
            raw_alpha = (omega_z - self.previous_omega) / dt
            self.ax_filt = self._filter(raw_ax, self.ax_filt)
            self.ay_filt = self._filter(velocity * omega_z, self.ay_filt)
            self.alpha_filt = self._filter(raw_alpha, self.alpha_filt)
            if all(math.isfinite(value) for value in (self.ax_filt, self.ay_filt, self.alpha_filt)):
                self.model.update(self.ax_filt, self.ay_filt, omega_z, self.alpha_filt)
                self.update_count += 1
        self.previous_stamp, self.previous_v, self.previous_omega = stamp, velocity, omega_z
        self.publish(stamp, omega_z)


def main() -> int:
    import rospy  # type: ignore

    rospy.init_node("sim_h_proxy_monitor")
    try:
        HProxyMonitor()
    except ConfigurationError as exc:
        rospy.logfatal("[sim_h_proxy_monitor] %s", exc)
        return 2
    rospy.loginfo("[sim_h_proxy_monitor] simulation-owned Odom-only H_proxy active")
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
