// Numerical integration bridge. Tests compile the real production classes;
// only ROS logging in LiquidSloshModel is replaced in a temporary include dir.
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/dynamics/actual_motion_propagator.h"
#include "spmpc_local_planner/estimation/slosh_observer_bank.h"
#include "spmpc_local_planner/estimation/liquid_state_nowcaster.h"
#include <cmath>

using namespace spmpc_local_planner;

namespace {
SloshDynamics& dynamics() {
    static SloshDynamics result;
    static const bool configured = result.configure(SloshModelParams{});
    (void)configured;
    return result;
}
SloshState state(const double* x) { return {x[0], x[1], x[2], x[3]}; }
void copy(const SloshState& x, double* out) {
    out[0] = x.eta_x; out[1] = x.eta_x_dot;
    out[2] = x.eta_y; out[3] = x.eta_y_dot;
}
}

extern "C" {
void liquid_parameters(double* out) {
    out[0] = 2 * dynamics().params().damping_ratio * dynamics().omegaN();
    out[1] = dynamics().omegaN() * dynamics().omegaN();
    out[2] = 1; out[3] = 1; out[4] = dynamics().heightCoeff();
}
int liquid_step(const double* x, const double* excitation, double dt, double* out) {
    SloshState next;
    const bool ok = dynamics().stepWithDt(state(x), {excitation[0], excitation[1],
                                         excitation[2], excitation[3]}, dt, next);
    copy(next, out);
    return ok;
}
int actual_motion_step(const double* x, const double* cmd, const double* actuator,
                       double dt, double* out) {
    RobotState robot{x[0], x[1], x[2], x[3], x[4]};
    SloshState liquid = state(x + 5);
    ActuatorModelParams params;
    params.linear_tau_sec = actuator[0]; params.angular_tau_sec = actuator[1];
    params.linear_gain = actuator[2]; params.angular_gain = actuator[3];
    const bool ok = propagateActualMotion(robot, liquid, {cmd[0], cmd[1]}, params, dynamics(), dt);
    out[0] = robot.x; out[1] = robot.y; out[2] = robot.yaw;
    out[3] = robot.v; out[4] = robot.omega;
    copy(liquid, out + 5);
    return ok;
}
int observer_steps(const double* rows, int count, double* out) {
    SloshObserverBank bank;
    if (!bank.configure(SloshModelParams{}, .02)) return 0;
    std::int64_t stamp = 1000000000;
    for (int k = 0; k < count; ++k) {
        const double* row = rows + 5 * k;
        stamp += std::llround(row[4] * 1e9);
        MotionExcitation e;
        e.valid = true; e.ax = row[0]; e.ay = row[1];
        e.omega_z = row[2]; e.alpha_z = row[3]; e.sample_dt_sec = row[4];
        e.source_stamp_ns = e.measurement_stamp_ns = stamp;
        e.accel_effective_stamp_ns = e.gyro_effective_stamp_ns = e.alpha_effective_stamp_ns = stamp;
        e.source = MotionExcitationSource::Odom;
        if (!bank.stepOdom(e)) return 0;
        e.source = MotionExcitationSource::ProcessedImu;
        if (!bank.stepImu(e)) return 0;
    }
    copy(bank.odom().state, out); copy(bank.imu().state, out + 4);
    return 1;
}
int nowcast_step(const double* x, const double* excitation, double dt, double* out) {
    LiquidStateNowcaster nowcaster;
    LiquidStateNowcasterParams params;
    params.enable = true;
    if (!nowcaster.configure(SloshModelParams{}, params)) return 0;
    LiquidStateNowcastInput input;
    input.snapshot_valid = true; input.state = state(x); input.state_stamp_ns = 1000000000;
    auto& e = input.excitation;
    e.valid = true; e.source = MotionExcitationSource::ProcessedImu;
    e.ax = excitation[0]; e.ay = excitation[1]; e.omega_z = excitation[2]; e.alpha_z = excitation[3];
    e.source_stamp_ns = e.measurement_stamp_ns = e.accel_effective_stamp_ns = input.state_stamp_ns;
    e.gyro_effective_stamp_ns = e.alpha_effective_stamp_ns = input.state_stamp_ns;
    e.sample_dt_sec = .02;
    const auto result = nowcaster.predict(input, input.state_stamp_ns + std::llround(dt * 1e9));
    copy(result.predicted_state, out);
    return result.valid;
}
}
