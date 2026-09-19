#include "spmpc_local_planner/core/task_stop_manager.h"
#include "spmpc_local_planner/dynamics/actual_jerk.h"
#include "spmpc_local_planner/dynamics/actual_motion_propagator.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace spmpc_local_planner {
namespace {
bool positive(double x) { return std::isfinite(x) && x > 0; }

bool finiteRobotState(const RobotState& state) {
    return std::isfinite(state.x) && std::isfinite(state.y) &&
        std::isfinite(state.yaw) && std::isfinite(state.v) &&
        std::isfinite(state.omega);
}

bool finiteActuatorState(const ActuatorState& state) {
    if (!std::isfinite(state.v_cmd) || !std::isfinite(state.omega_cmd) ||
        !std::isfinite(state.a_cmd_memory) || !std::isfinite(state.delayed_v_cmd) ||
        !std::isfinite(state.delayed_omega_cmd) || !std::isfinite(state.a_actual) ||
        !std::isfinite(state.alpha_actual)) return false;
    return std::all_of(state.linear_delay_queue.begin(), state.linear_delay_queue.end(),
                       [](double value) { return std::isfinite(value); }) &&
        std::all_of(state.angular_delay_queue.begin(), state.angular_delay_queue.end(),
                    [](double value) { return std::isfinite(value); });
}

double regionClearance(const RegionStageData& stage, double x, double y) {
    double result = std::numeric_limits<double>::infinity();
    if (!stage.enabled) return result;
    for (const RegionHalfspace& face : stage.faces)
        result = std::min(result, face.offset - face.nx * x - face.ny * y);
    return result;
}

bool commandWithinBounds(double v, double omega, const StopMotionLimits& limits) {
    return std::isfinite(v) && v >= -1e-9 && v <= limits.v_max + 1e-9 &&
        std::isfinite(omega) && std::abs(omega) <= limits.omega_max + 1e-9;
}

bool actualWithinBounds(double v, double omega, const StopMotionLimits& limits) {
    return std::isfinite(v) && v >= limits.actual_v_min - 1e-9 &&
        v <= limits.v_max + 1e-9 && std::isfinite(omega) &&
        std::abs(omega) <= limits.omega_max + 1e-9;
}

bool stopCandidateConsistent(const StopCommand& command,
                             const ActuatorState& history, double dt,
                             double a_max, double alpha_max, double jerk_max,
                             const StopMotionLimits& limits) {
    if (!command.valid || !std::isfinite(command.v) || command.v < 0.0 ||
        !std::isfinite(command.omega) || !std::isfinite(command.a) ||
        !std::isfinite(history.v_cmd) || !std::isfinite(history.a_cmd_memory))
        return false;
    if (!commandWithinBounds(command.v, command.omega, limits)) return false;
    const double expected_a = (command.v - history.v_cmd) / dt;
    return std::isfinite(expected_a) && std::abs(expected_a - command.a) <= 1e-8 &&
        std::abs(command.a) <= a_max + 1e-9 &&
        std::abs(command.a - history.a_cmd_memory) <= jerk_max * dt + 1e-9 &&
        std::abs(command.omega - history.omega_cmd) <= alpha_max * dt + 1e-9;
}

bool commandsExactlyZero(const ActuatorState& state) {
    if (state.v_cmd != 0.0 || state.omega_cmd != 0.0 || state.a_cmd_memory != 0.0)
        return false;
    return std::all_of(state.linear_delay_queue.begin(), state.linear_delay_queue.end(),
                       [](double value) { return value == 0.0; }) &&
        std::all_of(state.angular_delay_queue.begin(), state.angular_delay_queue.end(),
                    [](double value) { return value == 0.0; });
}

// Velocity change when applying a now, then releasing negative acceleration
// towards zero at +jerk each subsequent command interval. Includes this step.
double releaseVelocityChange(double a, double dt, double jerk) {
    if (a >= 0) return a * dt;
    const double delta = jerk * dt;
    const double count = std::ceil(-a / delta - 1e-12);
    return dt * (count*a + delta*count*(count-1)*.5);
}
}

StopCommand makeJerkLimitedStopCommand(const ActuatorState& history, double dt,
                                       double a_max, double alpha_max, double jerk) {
    StopCommand out;
    if (!history.valid || (!positive(dt) || dt<1e-6 || dt>1.) || !positive(a_max) || !positive(alpha_max) ||
        !positive(jerk) || !std::isfinite(history.v_cmd) || history.v_cmd < 0 ||
        !std::isfinite(history.omega_cmd) || !std::isfinite(history.a_cmd_memory)) return out;
    const double delta = jerk*dt;
    double lo = std::max(-a_max, history.a_cmd_memory-delta);
    const double hi = std::min(a_max, history.a_cmd_memory+delta);
    if (lo > hi || history.v_cmd + releaseVelocityChange(hi,dt,jerk) < -1e-10) {
        out.status = "STOP_JERK_HISTORY_INFEASIBLE"; return out;
    }
    if (history.v_cmd <= 1e-12) {
        // Never restart a stopped command to repair an inconsistent memory.
        if (std::abs(history.a_cmd_memory) > delta+1e-10) {
            out.status = "STOP_JERK_HISTORY_INFEASIBLE"; return out;
        }
        lo = 0;
    } else if (history.v_cmd + releaseVelocityChange(lo,dt,jerk) < 0) {
        double upper = hi;
        for (int i=0;i<55;++i) {
            const double mid = .5*(lo+upper);
            if (history.v_cmd + releaseVelocityChange(mid,dt,jerk) < 0) lo=mid;
            else upper=mid;
        }
        lo=upper;
    }
    out.a=lo;
    out.v=std::max(0.0,history.v_cmd+out.a*dt);
    // Collapse the numerical last step to the same exact zero represented by
    // the command FIFO. Keep deriving acceleration from the emitted value so
    // the jerk contract remains tied to what is actually sent.
    if (out.v <= 1e-12) out.v = 0.0;
    // Derive the reported acceleration from the emitted command, including roundoff.
    out.a=(out.v-history.v_cmd)/dt;
    out.omega=std::copysign(std::max(0.,std::abs(history.omega_cmd)-alpha_max*dt),history.omega_cmd);
    out.valid=true;out.status="STOP_JERK_OK";
    return out;
}

bool TaskStopManager::configure(const TaskStopParams& params, const ActuatorModelParams& actuator,
                                const SloshModelParams& liquid, const StopMotionLimits& limits,
                                double a_max, double alpha_max, double jerk_max,
                                bool include_liquid, double actual_jerk_max) {
    params_=params;actuator_=actuator;limits_=limits;
    a_max_=a_max;alpha_max_=alpha_max;jerk_max_=command_jerk_max_=jerk_max;actual_jerk_max_=actual_jerk_max;
    include_liquid_=include_liquid;
    configured_=positive(params.residual_height_m) && positive(params.stable_hold_sec) &&
        positive(params.max_settle_sec) && positive(params.max_tail_prediction_sec) &&
        params.max_tail_prediction_sec<=60.0 &&
        positive(params.quiet_v) && positive(params.quiet_omega) &&
        positive(params.command_zero_tolerance) && positive(params.velocity_cost_weight) &&
        std::isfinite(actual_jerk_max) && actual_jerk_max >= 0 &&
        positive(a_max) && positive(alpha_max) && positive(jerk_max) &&
        std::isfinite(limits.actual_v_min) && limits.actual_v_min <= 0.0 &&
        std::isfinite(limits.v_max) && limits.v_max > 0.0 &&
        limits.actual_v_min >= -limits.v_max &&
        std::isfinite(limits.omega_max) && limits.omega_max > 0.0 &&
        validateActuatorModelParams(actuator) && liquid_.configure(liquid);
    if (configured_ && actual_jerk_max_ > 0) {
        // For the ZOH actuator, j_actual[k+1] = r*j_actual[k]
        // + gain*dt/tau*j_cmd[k-delay]. This conservative command bound
        // preserves |j_actual| <= J once the fixed FIFO prefix is qualified.
        // It applies to the fallback tail only; OCP command bounds stay intact.
        const auto row=actualAccelerationDeltaRow(actuator_,actuator_.dt);
        const double one_minus_decay=row[0]*actuator_.linear_tau_sec;
        jerk_max_=std::min(jerk_max_, actual_jerk_max_*actuator_.linear_tau_sec*
            one_minus_decay/(actuator_.linear_gain*actuator_.dt));
    }
    reset();return configured_;
}

void TaskStopManager::reset() {
    elapsed_=stable_duration_=0;vehicle_stop_time_=liquid_stable_time_=-1;
    start_epoch_ns_=last_epoch_ns_=0;
    timed_out_=false;
    previous_sample_stable_=false;
    previous_observation_settled_=false;
    settle_wait_start_=-1;
    last_raw_robot_epoch_ns_=last_raw_liquid_epoch_ns_=0;
}

bool actuatorCommandsClear(const ActuatorState& state, double zero_tolerance) {
    if (!std::isfinite(zero_tolerance) || zero_tolerance < 0) return false;
    const auto zero=[&](double x) { return std::isfinite(x) && std::abs(x)<=zero_tolerance; };
    return state.valid && zero(state.v_cmd) && zero(state.omega_cmd) && zero(state.a_cmd_memory) &&
        std::all_of(state.linear_delay_queue.begin(),state.linear_delay_queue.end(),zero) &&
        std::all_of(state.angular_delay_queue.begin(),state.angular_delay_queue.end(),zero);
}

bool TaskStopManager::queuesClear(const ActuatorState& state) const {
    return actuatorCommandsClear(state, params_.command_zero_tolerance);
}

bool TaskStopManager::excitationQuiet(const RobotState& robot,const ActuatorState& state) const {
    return queuesClear(state) && std::abs(robot.v)<=params_.quiet_v &&
        std::abs(robot.omega)<=params_.quiet_omega;
}

double TaskStopManager::residualHeight(const SloshState& q) const {
    const double position=std::hypot(q.eta_x,q.eta_y);
    const double velocity=std::hypot(q.eta_x_dot,q.eta_y_dot)/liquid_.omegaN();
    return liquid_.heightCoeff()*std::hypot(position,velocity);
}

StopReadiness TaskStopManager::observe(const SolverInput& input,bool position_reached,
                                      double stopped_v,double stopped_omega) {
    StopReadiness out;
    if (!configured_ || (!positive(input.dt) || std::abs(input.dt-actuator_.dt)>1e-6) ||
        !finiteRobotState(input.robot) || !finiteActuatorState(input.actuator) ||
        (include_liquid_ && !finiteSloshState(input.slosh)) || !input.actuator.valid) {
        stable_duration_=0;previous_sample_stable_=false;
        return out;
    }
    double period=input.dt;
    const auto epoch=input.cycle_timing.solver_input_epoch_ns;
    if (epoch>0) {
        if (last_epoch_ns_>0 && epoch<=last_epoch_ns_) {
            stable_duration_=0;previous_sample_stable_=false;
            return out;
        }
        if (start_epoch_ns_==0) {start_epoch_ns_=epoch;period=0.;}
        else period=static_cast<double>(epoch-last_epoch_ns_)*1e-9;
        elapsed_=static_cast<double>(epoch-start_epoch_ns_)*1e-9;
        last_epoch_ns_=epoch;
        // Do not count missing observations as a continuously stable interval.
        if (period>2.0*input.dt) {stable_duration_=0.;period=0.;previous_sample_stable_=false;}
    } else elapsed_+=period;  // Deterministic offline callers without ROS epochs.
    // Prefix prediction advances the solver epoch even when no sensor has
    // delivered a new observation. Only new robot AND liquid measurements can
    // establish continuous stability; repeats merely pause the hold counter.
    bool fresh_measurement=true;
    const auto robot_epoch=input.cycle_timing.raw_robot_state_stamp_ns;
    const auto liquid_epoch=input.cycle_timing.raw_liquid_state_stamp_ns;
    if (robot_epoch>0 || liquid_epoch>0 || last_raw_robot_epoch_ns_>0) {
        if (robot_epoch<=0 || liquid_epoch<=0) {
            stable_duration_=0;previous_sample_stable_=false;
            return out;
        }
        if (last_raw_robot_epoch_ns_>0) {
            fresh_measurement=robot_epoch>last_raw_robot_epoch_ns_ && liquid_epoch>last_raw_liquid_epoch_ns_;
            if (robot_epoch<last_raw_robot_epoch_ns_ || liquid_epoch<last_raw_liquid_epoch_ns_) {
                stable_duration_=0;previous_sample_stable_=false;
                return out;
            }
            const double robot_period=(robot_epoch-last_raw_robot_epoch_ns_)*1e-9;
            const double liquid_period=(liquid_epoch-last_raw_liquid_epoch_ns_)*1e-9;
            if (robot_period>2*input.dt || liquid_period>2*input.dt) {
                stable_duration_=0;previous_sample_stable_=false;
            }
            period=fresh_measurement ? std::min(robot_period,liquid_period) : 0.;
        } else period=0.;
        if (fresh_measurement) {
            last_raw_robot_epoch_ns_=robot_epoch;
            last_raw_liquid_epoch_ns_=liquid_epoch;
        }
    }
    out.valid=true;out.queues_clear=queuesClear(input.actuator);
    out.vehicle_stopped=position_reached && out.queues_clear &&
        std::abs(input.robot.v)<=stopped_v && std::abs(input.robot.omega)<=stopped_omega;
    out.excitation_quiet=excitationQuiet(input.robot,input.actuator);
    out.residual_height_m=include_liquid_ ? residualHeight(input.slosh) : 0.0;
    if (out.vehicle_stopped && vehicle_stop_time_<0) vehicle_stop_time_=elapsed_;
    if (!out.vehicle_stopped || !out.excitation_quiet || out.residual_height_m>params_.residual_height_m) {
        stable_duration_=0;previous_sample_stable_=false;
    } else if (fresh_measurement) {
        if (previous_sample_stable_) stable_duration_+=period;
        previous_sample_stable_=true;
    }
    out.liquid_stable=stable_duration_+1e-12>=params_.stable_hold_sec;
    if (out.vehicle_stopped && settle_wait_start_<0) settle_wait_start_=elapsed_;
    if (previous_observation_settled_ && !out.liquid_stable) settle_wait_start_=elapsed_;
    // A task that exhausted its settling budget stays failed until a new task.
    // A later quiet sample must not retroactively turn a timeout into success.
    if (settle_wait_start_>=0 && !previous_observation_settled_ &&
        elapsed_-settle_wait_start_>params_.max_settle_sec) timed_out_=true;
    if (out.liquid_stable && !timed_out_ && liquid_stable_time_<0) liquid_stable_time_=elapsed_;
    out.stable_duration_sec=stable_duration_;
    out.vehicle_stop_time_sec=vehicle_stop_time_;out.liquid_stable_time_sec=liquid_stable_time_;
    out.timed_out=timed_out_;
    previous_observation_settled_=out.liquid_stable;
    return out;
}

StopTailPrediction TaskStopManager::predict(const SolverInput& input,
                                            const MotionRegion* region,
                                            double progress) const {
    return predictTail(input, nullptr, region, progress);
}

StopTailPrediction TaskStopManager::predictAfterCommand(
    const SolverInput& input, const StopCommand& first_command,
    const MotionRegion* region, double progress) const {
    return predictTail(input, &first_command, region, progress);
}

StopTailPrediction TaskStopManager::predictTail(
    const SolverInput& input, const StopCommand* first_command,
    const MotionRegion* region, double progress) const {
    StopTailPrediction out;
    out.region_checked = region != nullptr;
    if (!configured_ || (!positive(input.dt) || std::abs(input.dt-actuator_.dt)>1e-6) ||
        !input.actuator.valid || !finiteRobotState(input.robot) ||
        !finiteActuatorState(input.actuator) ||
        (include_liquid_ && !finiteSloshState(input.slosh)) ||
        (region != nullptr && !std::isfinite(progress))) {
        out.status="STOP_MODEL_INVALID";return out;
    }
    const auto fifoWithinBounds = [&](const ActuatorState& state) {
        return std::all_of(state.linear_delay_queue.begin(), state.linear_delay_queue.end(),
                           [&](double value) { return std::isfinite(value) &&
                               value >= -1e-9 && value <= limits_.v_max + 1e-9; }) &&
            std::all_of(state.angular_delay_queue.begin(), state.angular_delay_queue.end(),
                        [&](double value) { return std::isfinite(value) &&
                            std::abs(value) <= limits_.omega_max + 1e-9; });
    };
    if (!commandWithinBounds(input.actuator.v_cmd, input.actuator.omega_cmd, limits_) ||
        !commandWithinBounds(input.actuator.delayed_v_cmd,
                             input.actuator.delayed_omega_cmd, limits_)) {
        out.status="STOP_COMMAND_BOUND_VIOLATION";
        return out;
    }
    if (!fifoWithinBounds(input.actuator)) {
        out.status="STOP_COMMAND_FIFO_BOUND_VIOLATION";
        return out;
    }
    if (!actualWithinBounds(input.robot.v, input.robot.omega, limits_)) {
        out.status="STOP_ACTUAL_MOTION_BOUND_VIOLATION";
        return out;
    }
    RegionStageData region_stage;
    if (region != nullptr) {
        try {
            // Freeze the progress/cell for the complete tail.  The terminal
            // command is conservative and is never allowed to cross to a
            // later cell while braking.
            region_stage = region->stage(progress, 0.0);
            const double initial_clearance = regionClearance(region_stage, input.robot.x, input.robot.y);
            out.minimum_region_clearance_m = initial_clearance;
            if (initial_clearance < -1e-9) out.status = "STOP_REGION_UNSAFE";
        } catch (const std::exception&) {
            out.status = "STOP_REGION_UNSAFE";
            return out;
        }
    }
    auto robot=input.robot;
    // A vehicle-only group has no truthful liquid state.  Its vehicle model
    // still uses the same propagator, with a neutral internal liquid state.
    auto liquid=include_liquid_ ? input.slosh : SloshState{};
    auto actuator=input.actuator;
    if (include_liquid_) {
        out.peak_height_m=liquid_.height(liquid);
    }
    bool region_unsafe = out.status == "STOP_REGION_UNSAFE";
    bool quiet_seen = false;
    const std::size_t fifo_prefix_steps = std::min(actuator.linear_delay_queue.size(),
                                                   actuator.angular_delay_queue.size());
    const int steps=static_cast<int>(std::ceil(params_.max_tail_prediction_sec/input.dt));
    for (int k=0;k<steps;++k) {
        const auto command = (k == 0 && first_command != nullptr)
            ? *first_command
            : makeJerkLimitedStopCommand(actuator,input.dt,a_max_,alpha_max_,jerk_max_);
        if (!stopCandidateConsistent(command, actuator, input.dt, a_max_, alpha_max_, command_jerk_max_, limits_)) {
            out.status = (k == 0 && first_command != nullptr)
                ? (commandWithinBounds(command.v, command.omega, limits_)
                    ? "STOP_CANDIDATE_INVALID" : "STOP_COMMAND_BOUND_VIOLATION")
                : (commandWithinBounds(command.v, command.omega, limits_)
                    ? command.status : "STOP_COMMAND_BOUND_VIOLATION");
            return out;
        }
        if (k==0) out.first_command=command;
        double start_clearance = std::numeric_limits<double>::infinity();
        const double sweep_distance = std::max(std::abs(robot.v),
            std::abs(actuator_.linear_gain * actuator.linear_delay_queue.front())) * input.dt;
        if (region != nullptr) {
            start_clearance = regionClearance(region_stage, robot.x, robot.y);
            out.minimum_region_clearance_m = std::min(out.minimum_region_clearance_m,
                                                       start_clearance - sweep_distance);
            // This is a reserve check from the beginning of the interval. It
            // covers the complete interval even when the integrator endpoint
            // happens to lie back inside the polygon.
            if (start_clearance < sweep_distance - 1e-9) region_unsafe = true;
        }
        if (actual_jerk_max_ > 0) {
            const double delta = actualAccelerationDelta(robot.v, actuator.linear_delay_queue[0],
                actuator.linear_delay_queue[1], actuator_, input.dt);
            if (!std::isfinite(delta) || std::abs(delta)>actual_jerk_max_*input.dt+1e-6) {
                out.status="STOP_ACTUAL_JERK_VIOLATION";
                out.fifo_prefix_violation=static_cast<size_t>(k)+1<fifo_prefix_steps;
                return out;
            }
        }
        const double x=robot.x,y=robot.y;
        ActualMotionDiagnostics motion_diagnostics;
        if (!propagateActualMotion(robot,liquid,{actuator.linear_delay_queue.front(),actuator.angular_delay_queue.front()},actuator_,liquid_,input.dt,
                                                 &motion_diagnostics)) {
            out.status="STOP_TAIL_PROPAGATION_FAILED";return out;
        }
        if (!motion_diagnostics.valid ||
            !actualWithinBounds(robot.v, robot.omega, limits_) ||
            !actualWithinBounds(motion_diagnostics.min_v, motion_diagnostics.min_omega, limits_) ||
            !actualWithinBounds(motion_diagnostics.max_v, motion_diagnostics.max_omega, limits_)) {
            out.status="STOP_ACTUAL_MOTION_BOUND_VIOLATION";
            return out;
        }
        if (include_liquid_) {
            out.peak_height_m=std::max(out.peak_height_m, motion_diagnostics.peak_height_m);
        }
        std::move(actuator.linear_delay_queue.begin()+1,actuator.linear_delay_queue.end(),actuator.linear_delay_queue.begin());
        std::move(actuator.angular_delay_queue.begin()+1,actuator.angular_delay_queue.end(),actuator.angular_delay_queue.begin());
        actuator.linear_delay_queue.back()=actuator.v_cmd=command.v;
        actuator.angular_delay_queue.back()=actuator.omega_cmd=command.omega;
        actuator.a_cmd_memory=command.a;
        if (!fifoWithinBounds(actuator)) {
            out.status="STOP_COMMAND_FIFO_BOUND_VIOLATION";
            return out;
        }
        out.distance_m+=std::hypot(robot.x-x,robot.y-y);
        out.final_robot=robot;
        out.duration_sec+=input.dt;
        if (include_liquid_) {
            out.peak_height_m=std::max(out.peak_height_m,liquid_.height(liquid));
        }
        if (region != nullptr) {
            const double endpoint_clearance = regionClearance(region_stage, robot.x, robot.y);
            out.minimum_region_clearance_m = std::min(out.minimum_region_clearance_m,
                                                       endpoint_clearance);
            if (endpoint_clearance < -1e-9) {
                region_unsafe = true;
                // Only a real propagated endpoint in the unavoidable command
                // prefix establishes FIFO-region violation. A negative swept
                // reserve alone is deliberately not labelled unavoidable.
                if (static_cast<std::size_t>(k) < fifo_prefix_steps)
                    out.fifo_prefix_violation = true;
            }
        }
        if (excitationQuiet(robot,actuator) && commandsExactlyZero(actuator)) {
            quiet_seen = true;
            out.residual_height_m=include_liquid_ ? residualHeight(liquid) : 0.0;
            // Do not miss the following modal peak when the surface currently
            // crosses zero. This is an energy proxy under the quiet-tail gate.
            if (include_liquid_) out.peak_height_m=std::max(out.peak_height_m,out.residual_height_m);
            if (region != nullptr) {
                const double residual_reserve = actuator_.linear_tau_sec * std::abs(robot.v);
                const double quiet_clearance = regionClearance(region_stage, robot.x, robot.y);
                out.minimum_region_clearance_m = std::min(out.minimum_region_clearance_m,
                                                           quiet_clearance - residual_reserve);
                if (quiet_clearance < residual_reserve - 1e-9) region_unsafe = true;
            }
            if (!region_unsafe && !out.fifo_prefix_violation) {
                out.valid=true;out.status="STOP_TAIL_QUIET";return out;
            }
            // A conservative reserve failure must not hide a later propagated
            // FIFO endpoint violation. Continue through the prefix before
            // returning the region result.
            if (static_cast<std::size_t>(k) + 1 >= fifo_prefix_steps) break;
        }
    }
    if (out.fifo_prefix_violation) {
        out.status="STOP_FIFO_REGION_VIOLATION";return out;
    }
    if (region_unsafe) {
        out.status="STOP_REGION_UNSAFE";return out;
    }
    if (quiet_seen) {
        out.valid=true;out.status="STOP_TAIL_QUIET";return out;
    }
    out.status="STOP_TAIL_TIMEOUT";return out;
}
}  // namespace spmpc_local_planner
