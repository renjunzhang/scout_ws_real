"""Pure CasADi delay-augmented Phase-Rejoin transition model.

The model is intentionally independent of acados_template.  It mirrors the
C++ DelayAugmentedPhaseDynamics/ExecutionModel stage contract and is codegen'd
to a small C kernel before any optimizer capsule is admitted.
"""

import casadi as ca


STATE_BASE_WIDTH = 10
CONTROL_WIDTH = 3

WEIGHT_NAMES = (
    "w_position",
    "w_yaw",
    "w_progress",
    "w_v",
    "w_omega",
    "w_slosh_eta",
    "w_slosh_eta_dot",
    "w_linear_pending",
    "w_angular_pending",
    "w_a",
    "w_alpha",
    "w_v_s",
)
GATE_RADIUS_NAMES = (
    "gate_r_x",
    "gate_r_y",
    "gate_r_yaw",
    "gate_r_v",
    "gate_r_omega",
    "gate_r_eta_x",
    "gate_r_eta_x_dot",
    "gate_r_eta_y",
    "gate_r_eta_y_dot",
)


def _mapped_target(command, channel):
    # The frozen Scout contract has zero deadzone and unit directional gains.
    # Every pending/published command is already admitted inside the channel
    # envelope, where the C++ mapping is exactly the identity.  Returning that
    # equivalent smooth expression avoids the zero derivative introduced by
    # fabs/sign/if_else at a stopped command, which otherwise makes the RTI
    # linearization unable to predict the first acceleration from rest.
    if (
        channel["deadzone"] <= 1e-12
        and abs(channel["positive_gain"] - 1.0) <= 1e-12
        and abs(channel["negative_gain"] - 1.0) <= 1e-12
    ):
        return command
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
    state_names = [
        "x", "y", "yaw", "v", "progress_s", "omega",
        "eta_x", "eta_x_dot", "eta_y", "eta_y_dot",
    ]
    state_names.extend(
        f"linear_pending_{index}" for index in range(linear_count)
    )
    state_names.extend(
        f"angular_pending_{index}" for index in range(angular_count)
    )
    execution_indices = [3, 5]
    execution_indices.extend(
        linear_offset + index for index in range(linear_count)
    )
    execution_indices.extend(
        angular_offset + index for index in range(angular_count)
    )
    return {
        "base_width": STATE_BASE_WIDTH,
        "linear_buffer_offset": linear_offset,
        "linear_buffer_count": linear_count,
        "angular_buffer_offset": angular_offset,
        "angular_buffer_count": angular_count,
        "state_width": angular_offset + angular_count,
        "control_width": CONTROL_WIDTH,
        "state_names": state_names,
        "execution_indices": execution_indices,
    }


def parameter_layout(layout):
    names = [f"nom_{name}" for name in layout["state_names"]]
    nominal_state_offset = 0
    nominal_control_offset = len(names)
    names.extend(("nom_a", "nom_alpha", "nom_v_s"))
    nominal_publish_offset = len(names)
    names.extend(("nom_u_pub_v", "nom_u_pub_omega"))
    residual_bound_offset = len(names)
    names.extend(("max_residual_v", "max_residual_omega"))
    weight_offset = len(names)
    names.extend(WEIGHT_NAMES)
    gate_radius_offset = len(names)
    names.extend(GATE_RADIUS_NAMES)
    execution_bound_offset = len(names)
    names.extend(
        f"exec_beta_{layout['state_names'][index]}"
        for index in layout["execution_indices"]
    )
    return {
        "names": names,
        "index": {name: index for index, name in enumerate(names)},
        "nominal_state_offset": nominal_state_offset,
        "nominal_control_offset": nominal_control_offset,
        "nominal_publish_offset": nominal_publish_offset,
        "residual_bound_offset": residual_bound_offset,
        "weight_offset": weight_offset,
        "gate_radius_offset": gate_radius_offset,
        "execution_bound_offset": execution_bound_offset,
        "parameter_width": len(names),
    }


def published_command_constraints(published, p, layout, parameters,
                                  linear_min, linear_max,
                                  angular_min, angular_max):
    """Global command envelope plus nominal-relative residual authority.

    The residual limits live in the per-stage parameter image so every frozen
    nominal command is bound to the same stage that supplies its augmented
    state/control target.  Two one-sided inequalities per channel avoid a
    division by a possibly-zero authority and preserve the exact zero-residual
    case.
    """
    index = parameters["index"]
    nominal_v = p[index["nom_u_pub_v"]]
    nominal_omega = p[index["nom_u_pub_omega"]]
    residual_v = p[index["max_residual_v"]]
    residual_omega = p[index["max_residual_omega"]]
    constraints = ca.vertcat(
        published[0],
        published[1],
        published[0] - nominal_v - residual_v,
        nominal_v - published[0] - residual_v,
        published[1] - nominal_omega - residual_omega,
        nominal_omega - published[1] - residual_omega,
    )
    lower = (
        linear_min,
        angular_min,
        -1.0e15,
        -1.0e15,
        -1.0e15,
        -1.0e15,
    )
    upper = (
        linear_max, angular_max,
        0.0, 0.0, 0.0, 0.0,
    )
    return constraints, lower, upper


def execution_box_constraints(x, p, layout, parameters):
    """Phase-indexed B_exec path invariant in affine two-sided form.

    Constraining only the newly published queue tail is insufficient because
    that command subsequently shifts through queue positions whose frozen
    radii can tighten with phase.  Bind every execution component of every
    predicted state to the nominal/radius image of that same stage.
    """
    nominal = p[
        parameters["nominal_state_offset"]:
        parameters["nominal_state_offset"] + layout["state_width"]
    ]
    constraints = []
    lower = []
    upper = []
    for offset, state_index in enumerate(layout["execution_indices"]):
        beta = p[parameters["execution_bound_offset"] + offset]
        error = x[state_index] - nominal[state_index]
        constraints.extend((error - beta, -error - beta))
        lower.extend((-1.0e15, -1.0e15))
        upper.extend((0.0, 0.0))
    return ca.vertcat(*constraints), tuple(lower), tuple(upper)


def nominal_relative_cost(x, q, p, layout, parameters, scales,
                          terminal=False):
    index = parameters["index"]
    nominal = p[
        parameters["nominal_state_offset"]:
        parameters["nominal_state_offset"] + layout["state_width"]
    ]
    yaw_error = ca.atan2(
        ca.sin(x[2] - nominal[2]), ca.cos(x[2] - nominal[2])
    )
    cost = (
        p[index["w_position"]]
        * ((x[0] - nominal[0]) ** 2 + (x[1] - nominal[1]) ** 2)
        / (scales["position"] ** 2)
        + p[index["w_yaw"]] * (yaw_error / scales["yaw"]) ** 2
        + p[index["w_progress"]]
        * ((x[4] - nominal[4]) / scales["progress"]) ** 2
        + p[index["w_v"]] * ((x[3] - nominal[3]) / scales["v"]) ** 2
        + p[index["w_omega"]]
        * ((x[5] - nominal[5]) / scales["omega"]) ** 2
        + p[index["w_slosh_eta"]]
        * ((x[6] - nominal[6]) ** 2 + (x[8] - nominal[8]) ** 2)
        / (scales["eta"] ** 2)
        + p[index["w_slosh_eta_dot"]]
        * ((x[7] - nominal[7]) ** 2 + (x[9] - nominal[9]) ** 2)
        / (scales["eta_dot"] ** 2)
    )
    linear_slice = slice(
        layout["linear_buffer_offset"],
        layout["linear_buffer_offset"] + layout["linear_buffer_count"],
    )
    angular_slice = slice(
        layout["angular_buffer_offset"],
        layout["angular_buffer_offset"] + layout["angular_buffer_count"],
    )
    linear_error = x[linear_slice] - nominal[linear_slice]
    angular_error = x[angular_slice] - nominal[angular_slice]
    cost += (
        p[index["w_linear_pending"]]
        * ca.dot(linear_error, linear_error)
        / (scales["v"] ** 2)
        + p[index["w_angular_pending"]]
        * ca.dot(angular_error, angular_error)
        / (scales["omega"] ** 2)
    )
    if terminal:
        return cost
    nominal_q = p[
        parameters["nominal_control_offset"]:
        parameters["nominal_control_offset"] + CONTROL_WIDTH
    ]
    return cost + (
        p[index["w_a"]] * ((q[0] - nominal_q[0]) / scales["a"]) ** 2
        + p[index["w_alpha"]]
        * ((q[1] - nominal_q[1]) / scales["alpha"]) ** 2
        + p[index["w_v_s"]]
        * ((q[2] - nominal_q[2]) / scales["v_s"]) ** 2
    )


def nominal_relative_residual(x, q, p, layout, parameters, scales,
                              terminal=False):
    """Residual form of the same weighted quadratic tracking objective.

    Encoding the sum of squares as NONLINEAR_LS lets acados use a
    Gauss-Newton Hessian.  This avoids an ill-conditioned exact Lagrangian
    Hessian at the stopped, one-sided command boundary while preserving the
    objective exactly up to one global constant factor.
    """
    index = parameters["index"]
    nominal = p[
        parameters["nominal_state_offset"]:
        parameters["nominal_state_offset"] + layout["state_width"]
    ]

    def weighted(weight_name, value):
        return ca.sqrt(p[index[weight_name]]) * value

    yaw_error = ca.atan2(
        ca.sin(x[2] - nominal[2]), ca.cos(x[2] - nominal[2])
    )
    residuals = [
        weighted("w_position", (x[0] - nominal[0]) / scales["position"]),
        weighted("w_position", (x[1] - nominal[1]) / scales["position"]),
        weighted("w_yaw", yaw_error / scales["yaw"]),
        weighted("w_progress", (x[4] - nominal[4]) / scales["progress"]),
        weighted("w_v", (x[3] - nominal[3]) / scales["v"]),
        weighted("w_omega", (x[5] - nominal[5]) / scales["omega"]),
        weighted("w_slosh_eta", (x[6] - nominal[6]) / scales["eta"]),
        weighted("w_slosh_eta_dot",
                 (x[7] - nominal[7]) / scales["eta_dot"]),
        weighted("w_slosh_eta", (x[8] - nominal[8]) / scales["eta"]),
        weighted("w_slosh_eta_dot",
                 (x[9] - nominal[9]) / scales["eta_dot"]),
    ]
    for state_index in range(
            layout["linear_buffer_offset"],
            layout["linear_buffer_offset"] +
            layout["linear_buffer_count"]):
        residuals.append(weighted(
            "w_linear_pending",
            (x[state_index] - nominal[state_index]) / scales["v"]))
    for state_index in range(
            layout["angular_buffer_offset"],
            layout["angular_buffer_offset"] +
            layout["angular_buffer_count"]):
        residuals.append(weighted(
            "w_angular_pending",
            (x[state_index] - nominal[state_index]) / scales["omega"]))
    if not terminal:
        nominal_q = p[
            parameters["nominal_control_offset"]:
            parameters["nominal_control_offset"] + CONTROL_WIDTH
        ]
        residuals.extend((
            weighted("w_a", (q[0] - nominal_q[0]) / scales["a"]),
            weighted("w_alpha",
                     (q[1] - nominal_q[1]) / scales["alpha"]),
            weighted("w_v_s",
                     (q[2] - nominal_q[2]) / scales["v_s"]),
        ))
    return ca.vertcat(*residuals)


def terminal_recovery_constraints(x, p, layout, parameters):
    index = parameters["index"]
    nominal = p[
        parameters["nominal_state_offset"]:
        parameters["nominal_state_offset"] + layout["state_width"]
    ]
    gate_errors = (
        x[0] - nominal[0],
        x[1] - nominal[1],
        ca.atan2(ca.sin(x[2] - nominal[2]), ca.cos(x[2] - nominal[2])),
        x[3] - nominal[3],
        x[5] - nominal[5],
        x[6] - nominal[6],
        x[7] - nominal[7],
        x[8] - nominal[8],
        x[9] - nominal[9],
    )
    gate_metric = 0.0
    for error, radius_name in zip(gate_errors, GATE_RADIUS_NAMES):
        gate_metric += (error / p[index[radius_name]]) ** 2
    constraints = [gate_metric - 1.0]
    lower = [-1.0e15]
    upper = [0.0]
    for offset, state_index in enumerate(layout["execution_indices"]):
        beta = p[parameters["execution_bound_offset"] + offset]
        error = x[state_index] - nominal[state_index]
        # B_exec is a one-dimensional box for every execution component.
        # Keep beta in the affine expression rather than normalizing/squaring
        # by a potentially tiny terminal radius.  The feasible set is exactly
        # unchanged:
        #
        #   error - beta <= 0  and  -error - beta <= 0
        #       iff |error| <= beta.
        #
        # In the scaled OCP basis the two Jacobians are +/- state_scale,
        # independent of beta.  This prevents harmless interior-point dual
        # noise from being amplified by O(1 / beta) near the stopped tail.
        constraints.extend((error - beta, -error - beta))
        lower.extend((-1.0e15, -1.0e15))
        upper.extend((0.0, 0.0))
    return ca.vertcat(*constraints), tuple(lower), tuple(upper)


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
