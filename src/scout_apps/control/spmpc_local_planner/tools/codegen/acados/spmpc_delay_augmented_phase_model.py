"""Pure CasADi delay-augmented Phase-Rejoin transition model.

The model is intentionally independent of acados_template.  It mirrors the
C++ DelayAugmentedPhaseDynamics/ExecutionModel stage contract and is codegen'd
to a small C kernel before any optimizer capsule is admitted.
"""

import casadi as ca


STATE_BASE_WIDTH = 10
CONTROL_WIDTH = 3


def _mapped_target(command, channel):
    magnitude = ca.fabs(command)
    gain = ca.if_else(
        command >= 0.0,
        channel["positive_gain"],
        channel["negative_gain"],
    )
    mapped = ca.if_else(
        magnitude > channel["deadzone"],
        ca.sign(command) * gain * (magnitude - channel["deadzone"]),
        0.0,
    )
    return ca.fmin(
        channel["output_max"],
        ca.fmax(channel["output_min"], mapped),
    )


def _actuator_step(current, target, duration, time_constant):
    if time_constant <= 1e-12:
        return target
    decay = ca.exp(-duration / time_constant)
    return target + (current - target) * decay


def state_layout(contract):
    linear_count = contract["linear"]["integer_delay_steps"] + 1
    angular_count = contract["angular"]["integer_delay_steps"] + 1
    linear_offset = STATE_BASE_WIDTH
    angular_offset = linear_offset + linear_count
    return {
        "base_width": STATE_BASE_WIDTH,
        "linear_buffer_offset": linear_offset,
        "linear_buffer_count": linear_count,
        "angular_buffer_offset": angular_offset,
        "angular_buffer_count": angular_count,
        "state_width": angular_offset + angular_count,
        "control_width": CONTROL_WIDTH,
    }


def transition_expression(x, q, contract, layout):
    dt = contract["dt"]
    linear = contract["linear"]
    angular = contract["angular"]
    linear_buffer = [
        x[layout["linear_buffer_offset"] + index]
        for index in range(layout["linear_buffer_count"])
    ]
    angular_buffer = [
        x[layout["angular_buffer_offset"] + index]
        for index in range(layout["angular_buffer_count"])
    ]

    published_v = linear_buffer[-1] + q[0] * dt
    published_omega = angular_buffer[-1] + q[1] * dt
    linear_extended = linear_buffer + [published_v]
    angular_extended = angular_buffer + [published_omega]
    linear_older, linear_newer = linear_extended[0], linear_extended[1]
    angular_older, angular_newer = angular_extended[0], angular_extended[1]

    px, py, yaw = x[0], x[1], x[2]
    output_v, progress_s, output_omega = x[3], x[4], x[5]
    slosh = x[6:10]
    events = contract["events"]
    for segment_index in range(len(events) - 1):
        start = events[segment_index]
        duration = events[segment_index + 1] - start
        target_v_command = (
            linear_older
            if linear["fractional_delay_sec"] > 1e-12
            and start < linear["fractional_delay_sec"] - 1e-12
            else linear_newer
        )
        target_omega_command = (
            angular_older
            if angular["fractional_delay_sec"] > 1e-12
            and start < angular["fractional_delay_sec"] - 1e-12
            else angular_newer
        )
        target_v = _mapped_target(target_v_command, linear)
        target_omega = _mapped_target(target_omega_command, angular)
        previous_v = output_v
        output_v = _actuator_step(
            output_v, target_v, duration, linear["time_constant_sec"]
        )
        output_omega = _actuator_step(
            output_omega,
            target_omega,
            duration,
            angular["time_constant_sec"],
        )
        px = px + output_v * ca.cos(yaw) * duration
        py = py + output_v * ca.sin(yaw) * duration
        yaw = ca.atan2(
            ca.sin(yaw + output_omega * duration),
            ca.cos(yaw + output_omega * duration),
        )
        acceleration_x = (output_v - previous_v) / duration
        acceleration_y = output_v * output_omega
        matrices = contract["slosh_segment_matrices"][segment_index]
        slosh = (
            ca.DM(matrices["ad"]) @ slosh
            + ca.DM(matrices["bd"])
            @ ca.vertcat(acceleration_x, acceleration_y)
        )

    next_state = ca.vertcat(
        px,
        py,
        yaw,
        output_v,
        progress_s + q[2] * dt,
        output_omega,
        slosh,
        *linear_extended[1:],
        *angular_extended[1:],
    )
    return next_state, ca.vertcat(published_v, published_omega)


def export_transition_functions(contract):
    layout = state_layout(contract)
    x = ca.SX.sym("x", layout["state_width"])
    q = ca.SX.sym("q", CONTROL_WIDTH)
    x_next, published = transition_expression(x, q, contract, layout)
    step = ca.Function(
        "spmpc_delay_augmented_phase_transition",
        [x, q],
        [x_next, published],
        ["x", "q"],
        ["x_next", "u_pub"],
    )
    step_jacobian = ca.Function(
        "spmpc_delay_augmented_phase_step_jacobian",
        [x, q],
        [x_next, ca.densify(ca.jacobian(x_next, q))],
        ["x", "q"],
        ["x_next", "dx_next_dq"],
    )

    horizon_steps = contract["horizon_steps"]
    x0 = ca.SX.sym("x0", layout["state_width"])
    q0 = ca.SX.sym("q0", CONTROL_WIDTH)
    q_tail = ca.SX.sym("q_tail", CONTROL_WIDTH * (horizon_steps - 1))
    terminal = x0
    for stage in range(horizon_steps):
        stage_q = q0 if stage == 0 else q_tail[
            CONTROL_WIDTH * (stage - 1):CONTROL_WIDTH * stage
        ]
        terminal, _ = transition_expression(
            terminal, stage_q, contract, layout
        )
    terminal_jacobian = ca.Function(
        "spmpc_delay_augmented_phase_terminal_jacobian",
        [x0, q0, q_tail],
        [terminal, ca.densify(ca.jacobian(terminal, q0))],
        ["x0", "q0", "q_tail"],
        ["x_terminal", "dx_terminal_dq0"],
    )
    return {
        "layout": layout,
        "step": step,
        "step_jacobian": step_jacobian,
        "terminal_jacobian": terminal_jacobian,
    }
