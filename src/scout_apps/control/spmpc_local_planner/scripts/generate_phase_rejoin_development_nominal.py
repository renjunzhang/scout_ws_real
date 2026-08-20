#!/usr/bin/env python3
"""Generate a dynamics-consistent development Phase-Rejoining nominal.

This is deliberately not OfflineSloshOCP.  It turns a frozen ROS Path into a
smooth, uniformly sampled unicycle trajectory, propagates the same linear
liquid model used by S-MPCC, and appends an explicit stop/settle/zero-hold tail.
The resulting V2 artifact is suitable for simulation contract and performance
development only; its metadata permanently excludes hardware/formal/paper use.
"""

import argparse
import csv
import hashlib
import math
import os
from pathlib import Path
import tempfile
from typing import Iterable, List, NamedTuple, Sequence, Tuple


SCHEMA = "phase_rejoin_empirical_v2"
TERMINAL_CONTRACT = "stop_settle_zero_hold_v1"
RECOVERY_CONTRACT = "nominal_command_v1"
EVIDENCE_LEVEL = "development_only"
SOURCE = "development_dynamics_consistent_nominal"

HEADER = (
    "index", "t", "s", "x", "y", "yaw", "v", "omega",
    "eta_x", "eta_x_dot", "eta_y", "eta_y_dot",
    "a", "alpha", "v_s", "u_pub_v", "u_pub_omega",
    "kappa_v", "kappa_omega",
    "r_x", "r_y", "r_yaw", "r_v", "r_omega",
    "r_eta_x", "r_eta_x_dot", "r_eta_y", "r_eta_y_dot",
)


class GenerationError(RuntimeError):
    pass


class Point(NamedTuple):
    x: float
    y: float


class State(NamedTuple):
    x: float
    y: float
    yaw: float
    v: float
    omega: float
    s: float
    eta_x: float
    eta_x_dot: float
    eta_y: float
    eta_y_dot: float


class Control(NamedTuple):
    a: float
    alpha: float
    v_s: float


class LiquidModel(NamedTuple):
    two_zeta_omega_n: float
    omega_n_sq: float
    kappa_x: float
    kappa_y: float


def finite(value: object, label: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise GenerationError(f"{label} is not numeric") from exc
    if not math.isfinite(parsed):
        raise GenerationError(f"{label} is not finite")
    return parsed


def positive(value: object, label: str) -> float:
    parsed = finite(value, label)
    if parsed <= 0.0:
        raise GenerationError(f"{label} must be positive")
    return parsed


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def clean_points(points: Iterable[Point]) -> Tuple[Point, ...]:
    result: List[Point] = []
    for offset, point in enumerate(points):
        x = finite(point.x, f"point[{offset}].x")
        y = finite(point.y, f"point[{offset}].y")
        candidate = Point(x, y)
        if not result or math.hypot(
                candidate.x - result[-1].x,
                candidate.y - result[-1].y) > 1.0e-6:
            result.append(candidate)
    if len(result) < 3:
        raise GenerationError("reference path needs at least three distinct points")
    return tuple(result)


def cumulative_s(points: Sequence[Point]) -> Tuple[float, ...]:
    values = [0.0]
    for previous, current in zip(points, points[1:]):
        values.append(values[-1] + math.hypot(
            current.x - previous.x, current.y - previous.y))
    if values[-1] <= 0.1:
        raise GenerationError("reference path is too short")
    return tuple(values)


def interpolate_position(
    points: Sequence[Point], path_s: Sequence[float], query: float
) -> Point:
    s = clamp(query, 0.0, path_s[-1])
    lo = 0
    hi = len(path_s) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if path_s[mid] <= s:
            lo = mid
        else:
            hi = mid
    length = path_s[lo + 1] - path_s[lo]
    ratio = 0.0 if length <= 1.0e-12 else (s - path_s[lo]) / length
    return Point(
        points[lo].x + ratio * (points[lo + 1].x - points[lo].x),
        points[lo].y + ratio * (points[lo + 1].y - points[lo].y),
    )


def path_heading(
    points: Sequence[Point], path_s: Sequence[float], query: float
) -> float:
    delta = min(0.05, 0.1 * path_s[-1])
    before = interpolate_position(points, path_s, max(0.0, query - delta))
    after = interpolate_position(points, path_s, min(path_s[-1], query + delta))
    return math.atan2(after.y - before.y, after.x - before.x)


def derivative(state: State, control: Control, model: LiquidModel) -> State:
    return State(
        state.v * math.cos(state.yaw),
        state.v * math.sin(state.yaw),
        state.omega,
        control.a,
        control.alpha,
        control.v_s,
        state.eta_x_dot,
        -model.two_zeta_omega_n * state.eta_x_dot
        - model.omega_n_sq * state.eta_x
        - model.kappa_x * control.a,
        state.eta_y_dot,
        -model.two_zeta_omega_n * state.eta_y_dot
        - model.omega_n_sq * state.eta_y
        - model.kappa_y * state.v * state.omega,
    )


def add_scaled(state: State, change: State, scale: float) -> State:
    return State(*(value + scale * delta for value, delta in zip(state, change)))


def rk4_step(
    state: State, control: Control, model: LiquidModel, dt: float
) -> State:
    k1 = derivative(state, control, model)
    k2 = derivative(add_scaled(state, k1, 0.5 * dt), control, model)
    k3 = derivative(add_scaled(state, k2, 0.5 * dt), control, model)
    k4 = derivative(add_scaled(state, k3, dt), control, model)
    return State(*(
        value + dt * (d1 + 2.0 * d2 + 2.0 * d3 + d4) / 6.0
        for value, d1, d2, d3, d4 in zip(state, k1, k2, k3, k4)
    ))


def speed_schedule(
    path_length: float, dt: float, requested_speed: float, ramp_sec: float
) -> Tuple[Tuple[float, ...], Tuple[float, ...]]:
    if path_length / requested_speed <= ramp_sec:
        raise GenerationError("ramp is too long for requested cruise speed")
    cruise_sec = path_length / requested_speed - ramp_sec
    nominal_duration = 2.0 * ramp_sec + cruise_sec
    steps = int(math.ceil(nominal_duration / dt))
    times = tuple(index * dt for index in range(steps + 1))
    shape: List[float] = []
    for time_sec in times:
        if time_sec <= ramp_sec:
            value = 0.5 * (1.0 - math.cos(math.pi * time_sec / ramp_sec))
        elif time_sec <= ramp_sec + cruise_sec:
            value = 1.0
        elif time_sec <= nominal_duration:
            phase = (time_sec - ramp_sec - cruise_sec) / ramp_sec
            value = 0.5 * (1.0 + math.cos(math.pi * phase))
        else:
            value = 0.0
        shape.append(value)
    area = sum(
        0.5 * (shape[index] + shape[index + 1]) * dt
        for index in range(steps)
    )
    actual_speed = path_length / area
    velocities = tuple(actual_speed * value for value in shape)
    progress = [0.0]
    for index in range(steps):
        progress.append(progress[-1] +
                        0.5 * (velocities[index] + velocities[index + 1]) * dt)
    progress[-1] = path_length
    return velocities, tuple(progress)


def make_row(
    index: int,
    dt: float,
    state: State,
    control: Control,
    next_state: State,
    radii: Sequence[float],
) -> Tuple[object, ...]:
    return (
        index, index * dt,
        state.s, state.x, state.y, state.yaw, state.v, state.omega,
        state.eta_x, state.eta_x_dot, state.eta_y, state.eta_y_dot,
        control.a, control.alpha, control.v_s,
        next_state.v, next_state.omega,
        next_state.v, next_state.omega,
    ) + tuple(radii)


def generate_rows(
    points: Sequence[Point],
    dt: float,
    requested_speed: float,
    ramp_sec: float,
    lookahead: float,
    heading_gain: float,
    omega_max: float,
    alpha_max: float,
    zero_hold_sec: float,
    terminal_eta_norm_max: float,
    terminal_eta_dot_norm_max: float,
    model: LiquidModel,
    radii: Sequence[float],
) -> Tuple[Tuple[Tuple[object, ...], ...], int, float, float]:
    points = clean_points(points)
    path_s = cumulative_s(points)
    path_length = path_s[-1]
    velocities, progress = speed_schedule(
        path_length, dt, requested_speed, ramp_sec)

    state = State(
        points[0].x, points[0].y, path_heading(points, path_s, 0.0),
        velocities[0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    )
    rows: List[Tuple[object, ...]] = []
    max_path_deviation = 0.0
    for index in range(len(velocities) - 1):
        next_v = velocities[index + 1]
        next_s = progress[index + 1]
        target = interpolate_position(
            points, path_s, min(path_length, state.s + lookahead))
        target_heading = math.atan2(target.y - state.y, target.x - state.x)
        omega_target = clamp(
            heading_gain * wrap(target_heading - state.yaw),
            -omega_max, omega_max,
        )
        control = Control(
            (next_v - state.v) / dt,
            clamp((omega_target - state.omega) / dt, -alpha_max, alpha_max),
            (next_s - state.s) / dt,
        )
        next_state = rk4_step(state, control, model, dt)
        rows.append(make_row(index, dt, state, control, next_state, radii))
        nominal_position = interpolate_position(points, path_s, state.s)
        max_path_deviation = max(max_path_deviation, math.hypot(
            state.x - nominal_position.x, state.y - nominal_position.y))
        state = next_state

    # Stop angular motion before declaring the zero-command hold.  Linear
    # motion and virtual progress are already exactly zero/path_length here.
    while abs(state.omega) > 1.0e-10:
        control = Control(
            0.0,
            clamp(-state.omega / dt, -alpha_max, alpha_max),
            0.0,
        )
        next_state = rk4_step(state, control, model, dt)
        rows.append(make_row(len(rows), dt, state, control, next_state, radii))
        state = next_state
        if len(rows) > 1_000_000:
            raise GenerationError("angular settling failed to converge")

    zero_hold_begin = len(rows)
    minimum_hold_rows = max(5, int(math.ceil(zero_hold_sec / dt)) + 1)
    while True:
        control = Control(0.0, 0.0, 0.0)
        next_state = rk4_step(state, control, model, dt)
        rows.append(make_row(len(rows), dt, state, control, next_state, radii))
        state = next_state
        hold_rows_after_append = len(rows) - zero_hold_begin
        settled = (
            math.hypot(state.eta_x, state.eta_y) <= terminal_eta_norm_max and
            math.hypot(state.eta_x_dot, state.eta_y_dot) <=
                terminal_eta_dot_norm_max
        )
        # The final row itself also carries a zero control, hence +1.
        if hold_rows_after_append + 1 >= minimum_hold_rows and settled:
            break
        if hold_rows_after_append > int(math.ceil(20.0 / dt)):
            raise GenerationError("liquid did not settle within 20 seconds")

    final_control = Control(0.0, 0.0, 0.0)
    rows.append(make_row(
        len(rows), dt, state, final_control, state, radii))
    zero_hold_steps = len(rows) - zero_hold_begin
    terminal_eta_norm = math.hypot(state.eta_x, state.eta_y)
    terminal_eta_dot_norm = math.hypot(state.eta_x_dot, state.eta_y_dot)
    return tuple(rows), zero_hold_steps, max_path_deviation, path_length


def format_value(value: object) -> str:
    return str(value) if isinstance(value, int) else format(float(value), ".17g")


def write_artifact(
    output: Path,
    metadata: Sequence[Tuple[str, str]],
    rows: Sequence[Sequence[object]],
    overwrite: bool,
) -> None:
    if output.exists() and not overwrite:
        raise GenerationError(f"output exists (use --overwrite): {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = ""
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", newline="", delete=False,
            dir=str(output.parent), prefix=output.name + ".tmp.",
        ) as stream:
            temporary = stream.name
            for key, value in metadata:
                stream.write(f"# {key}={value}\n")
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(HEADER)
            writer.writerows(tuple(format_value(value) for value in row) for row in rows)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, output)
        temporary = ""
    finally:
        if temporary:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def load_path_from_bag(path: Path, topic: str) -> Tuple[Point, ...]:
    try:
        import rosbag
    except ImportError as exc:
        raise GenerationError("rosbag is required to read --bag") from exc
    with rosbag.Bag(str(path), "r") as bag:
        for _topic, message, _stamp in bag.read_messages(topics=[topic]):
            poses = list(getattr(message, "poses", ()))
            if len(poses) >= 3:
                return clean_points(
                    Point(item.pose.position.x, item.pose.position.y)
                    for item in poses
                )
    raise GenerationError(f"no nonempty nav_msgs/Path found on {topic}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument("--path-topic", default="/scout/global_path_fixed")
    parser.add_argument("--output", required=True)
    parser.add_argument("--contract-id", required=True)
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--dt", type=float, default=0.0333333333)
    parser.add_argument("--cruise-speed", type=float, default=0.34)
    parser.add_argument("--ramp-sec", type=float, default=2.5)
    parser.add_argument("--lookahead", type=float, default=0.30)
    parser.add_argument("--heading-gain", type=float, default=3.0)
    parser.add_argument("--omega-max", type=float, default=1.0)
    parser.add_argument("--alpha-max", type=float, default=1.0)
    parser.add_argument("--omega-n", type=float, required=True)
    parser.add_argument("--damping-ratio", type=float, default=0.05)
    parser.add_argument("--kappa-x", type=float, default=1.0)
    parser.add_argument("--kappa-y", type=float, default=1.0)
    parser.add_argument("--zero-hold-sec", type=float, default=2.0)
    parser.add_argument("--terminal-eta-norm-max", type=float, default=2.0e-6)
    parser.add_argument("--terminal-eta-dot-norm-max", type=float, default=1.0e-4)
    parser.add_argument(
        "--gate-radii", type=float, nargs=9, required=True,
        metavar=("RX", "RY", "RYAW", "RV", "ROMEGA", "REX", "REXD", "REY", "REYD"),
    )
    parser.add_argument("--overwrite", action="store_true")
    return parser


def main(argv: Sequence[str] = None) -> int:
    args = build_parser().parse_args(argv)
    bag = Path(args.bag).expanduser().resolve()
    output = Path(args.output).expanduser().resolve()
    if not bag.is_file():
        raise GenerationError(f"bag does not exist: {bag}")
    if not args.contract_id.strip() or "\n" in args.contract_id:
        raise GenerationError("invalid contract id")
    dt = positive(args.dt, "dt")
    omega_n = positive(args.omega_n, "omega_n")
    damping_ratio = finite(args.damping_ratio, "damping_ratio")
    if damping_ratio < 0.0 or damping_ratio > 1.0:
        raise GenerationError("damping_ratio must be in [0, 1]")
    radii = tuple(positive(value, "gate radius") for value in args.gate_radii)
    model = LiquidModel(
        2.0 * damping_ratio * omega_n,
        omega_n * omega_n,
        positive(args.kappa_x, "kappa_x"),
        positive(args.kappa_y, "kappa_y"),
    )
    eta_limit = positive(args.terminal_eta_norm_max, "terminal_eta_norm_max")
    eta_dot_limit = positive(
        args.terminal_eta_dot_norm_max, "terminal_eta_dot_norm_max")
    points = load_path_from_bag(bag, args.path_topic)
    rows, zero_hold_steps, max_deviation, path_length = generate_rows(
        points=points,
        dt=dt,
        requested_speed=positive(args.cruise_speed, "cruise_speed"),
        ramp_sec=positive(args.ramp_sec, "ramp_sec"),
        lookahead=positive(args.lookahead, "lookahead"),
        heading_gain=positive(args.heading_gain, "heading_gain"),
        omega_max=positive(args.omega_max, "omega_max"),
        alpha_max=positive(args.alpha_max, "alpha_max"),
        zero_hold_sec=positive(args.zero_hold_sec, "zero_hold_sec"),
        terminal_eta_norm_max=eta_limit,
        terminal_eta_dot_norm_max=eta_dot_limit,
        model=model,
        radii=radii,
    )
    if max_deviation > 0.20:
        raise GenerationError(
            f"generated nominal deviates {max_deviation:.3f} m from frozen path")
    metadata = (
        ("schema", SCHEMA),
        ("evidence_level", EVIDENCE_LEVEL),
        ("source", SOURCE),
        ("contract_id", args.contract_id.strip()),
        ("frame_id", args.frame_id.strip()),
        ("dt", format(dt, ".17g")),
        ("path_length", format(path_length, ".17g")),
        ("terminal_contract", TERMINAL_CONTRACT),
        ("recovery_contract", RECOVERY_CONTRACT),
        ("terminal_zero_hold_steps", str(zero_hold_steps)),
        ("terminal_eta_norm_max", format(eta_limit, ".17g")),
        ("terminal_eta_dot_norm_max", format(eta_dot_limit, ".17g")),
        ("two_zeta_omega_n", format(model.two_zeta_omega_n, ".17g")),
        ("omega_n_sq", format(model.omega_n_sq, ".17g")),
        ("kappa_x", format(model.kappa_x, ".17g")),
        ("kappa_y", format(model.kappa_y, ".17g")),
        ("dynamics_tolerance", "1e-8"),
        ("artifact_role", "interface_smoke_only"),
        ("nominal_sequence_kind", "uniform_dynamics_consistent_complete_tail"),
        ("offline_slosh_ocp", "false"),
        ("hardware_formal_release", "false"),
        ("paper_main_result_eligible", "false"),
        ("gate_evidence", "none_operator_supplied_development_radii"),
        ("recovery_policy_source", "nominal_next_command_development_fallback"),
        ("recovery_policy_evidence", "none_development_only"),
        ("source_bag_sha256", sha256(bag)),
        ("path_topic", args.path_topic),
        ("max_nominal_path_deviation_m", format(max_deviation, ".17g")),
    )
    write_artifact(output, metadata, rows, args.overwrite)
    duration = (len(rows) - 1) * dt
    print(
        f"[OK] development-only V2 artifact: {output}\n"
        f"rows={len(rows)} duration={duration:.3f}s path={path_length:.3f}m "
        f"zero_hold_steps={zero_hold_steps} max_path_deviation={max_deviation:.4f}m\n"
        "NOT OfflineSloshOCP; NOT hardware/formal/paper evidence"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
