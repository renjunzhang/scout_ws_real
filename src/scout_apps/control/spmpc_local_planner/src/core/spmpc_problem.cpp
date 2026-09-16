#include "spmpc_local_planner/core/spmpc_problem.h"
#include "spmpc_local_planner/reference/progress_projector.h"
#include "spmpc_local_planner/solvers/solver_factory.h"
#include "spmpc_local_planner/planning/ocp_planning_adapter.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <cstddef>

namespace spmpc_local_planner {
namespace {

double sqr(double v) {
    return v * v;
}

double pointDist2(const TrajectoryPoint& a, const TrajectoryPoint& b) {
    return sqr(a.x - b.x) + sqr(a.y - b.y);
}

double angleDiff(double a, double b) {
    return std::atan2(std::sin(a - b), std::cos(a - b));
}

bool sameReferencePath(const ReferencePath& a, const ReferencePath& b) {
    if (a.empty() || b.empty()) {
        return false;
    }
    const auto& ap = a.points();
    const auto& bp = b.points();
    if (a.frameId() != b.frameId() || ap.size() != bp.size() ||
        std::abs(a.length() - b.length()) >= 1e-3) {
        return false;
    }
    for (size_t i = 0; i < ap.size(); ++i) {
        if (pointDist2(ap[i], bp[i]) >= 1e-4 || std::abs(angleDiff(ap[i].yaw, bp[i].yaw)) >= 1e-3) {
            return false;
        }
    }
    return true;
}

}  // namespace

SpmpcProblem::SpmpcProblem() = default;

void SpmpcProblem::configure(const SolverParams& solver_params, const VariantConfig& variant) {
    solver_params_ = solver_params;
    variant_=variant;
    task_clock_.reset();
    task_plan_.reset();
    task_reference_.reset();
    motion_region_.reset();
    configuration_error_.clear();
    try {
        const auto& planning=solver_params_.planning;
        if ((planning.region.enabled || planning.geometry.enabled || planning.task_deadline_sec>0 ||
             planning.trajectory.mode!=TrajectoryReferenceMode::Cruise) &&
            (solver_params_.solver_backend!="continuous_mpcc_acados" ||
             solver_params_.actuator.mode!=ExecutionModelMode::ExplicitActuator))
            throw std::invalid_argument("planning requires explicit-actuator continuous MPCC");
        if ((planning.task_deadline_sec>0 || planning.trajectory.mode!=TrajectoryReferenceMode::Cruise) &&
            (!solver_params_.terminal.enable || !solver_params_.terminal.mpc_stop_handoff_enable ||
             !solver_params_.jerk_limit_enable))
            throw std::invalid_argument("timed tasks require terminal handoff and the common jerk bound");
        if (solver_params_.rti_iterations < 1 || solver_params_.rti_iterations > 20)
            throw std::invalid_argument("acados/rti_iterations must be in [1,20]");
        if (!std::isfinite(solver_params_.terminal.goal_yaw_tolerance) ||
            solver_params_.terminal.goal_yaw_tolerance <= 0 || solver_params_.terminal.goal_yaw_tolerance > M_PI)
            throw std::invalid_argument("invalid terminal yaw tolerance");
        OcpPlanningAdapter validate(solver_params_);
        if (solver_params_.planning.region.enabled)
            motion_region_=std::make_shared<MotionRegion>(solver_params_.planning.region);
        if (solver_params_.planning.trajectory.mode != TrajectoryReferenceMode::Cruise) {
            task_plan_ = std::make_shared<TrajectoryPlan>(TrajectoryPlan::load(solver_params_.planning.trajectory.plan_file));
            task_reference_=std::make_shared<TrajectoryReference>(task_plan_);
            if (solver_params_.planning.task_deadline_sec > 0 &&
                std::abs(task_plan_->deadline-solver_params_.planning.task_deadline_sec)>1e-8)
                throw std::invalid_argument("configured deadline differs from plan");
        }
    } catch (const std::exception& e) { configuration_error_=e.what(); }
    liquid_state_required_ = variant.slosh_enable || solver_params_.task_stop.enable;
    liquid_limit_enabled_ = variant.slosh_enable && variant.slosh_constraint_enable;
    task_stop_configured_ = !solver_params_.task_stop.enable ||
        (solver_params_.terminal.enable && solver_params_.terminal.mpc_stop_handoff_enable &&
         solver_params_.jerk_limit_enable && !solver_params_.zero_liquid_initial_state &&
         task_stop_manager_.configure(solver_params_.task_stop, solver_params_.actuator,
             solver_params_.slosh, solver_params_.a_max, solver_params_.alpha_max, solver_params_.jerk_max));
    configured_v_ref_ = variant.v_ref;
    terminal_controller_.setParams(solver_params_.terminal);
    start_lock_recovery_.setParams(solver_params_.start_lock_recovery);
    solver_ = makeSolver(solver_params_.solver_backend);
    solver_->configure(solver_params_, variant);
}

void SpmpcProblem::setReferencePath(const ReferencePath& reference) {
    if (task_plan_) {
        std::vector<RegionVertex> points;
        for (const auto& p:reference.points()) points.push_back({p.x,p.y});
        if (!validateRoute(points,reference.frameId(),solver_params_.planning.trajectory.route_tolerance,*task_plan_)) {
            configuration_error_="trajectory route/frame identity mismatch";
            return;
        }
    }
    const bool same_path = sameReferencePath(reference_, reference);
    if (!reference_.empty() && !same_path && (task_plan_ || solver_params_.planning.task_deadline_sec>0)) {
        configuration_error_="route changed during a timed task; configure a new task explicitly";
        return;
    }
    const bool changed_task=!reference_.empty() && !same_path && !task_plan_;
    reference_ = reference;
    if (!same_path) {
        if (changed_task && solver_) solver_->configure(solver_params_,variant_);
        // Republishing/reassembling a route cannot buy more time for a plan.
        // A different task/plan requires configure(), which resets the clock.
        if (!task_plan_ && solver_params_.planning.task_deadline_sec<=0) task_clock_.reset();
        last_progress_s_ = 0.0;
        terminal_controller_.reset();
        task_stop_manager_.reset();
        start_lock_recovery_.reset();
    }
}

void SpmpcProblem::setCostmap(const CostmapGrid& costmap) {
    costmap_ = costmap;
    have_costmap_ = !costmap_.empty();
}

void SpmpcProblem::updateStartLockRecovery(const SolverInput& input, bool valid_output, SolverOutput& output) {
    StartLockRecoveryObservation obs;
    obs.valid = valid_output;
    obs.terminal_reached = output.terminal_diagnostics.reached || output.status == "GOAL_REACHED";
    obs.status = output.status;
    obs.progress_abs_s = output.progress_abs_s;
    obs.cmd_v = output.cmd_v;
    obs.robot_v = input.robot.v;

    const auto& projector = output.projector_debug;
    obs.raw_projection_valid = projector.raw_valid;
    obs.guarded_projection_valid = projector.guarded_valid;
    obs.projector_raw_s = projector.raw_s;
    obs.projector_guarded_s = projector.guarded_s;
    obs.projector_raw_distance = projector.raw_distance;
    obs.projector_guarded_distance = projector.guarded_distance;
    obs.monotonic_clip_applied = projector.monotonic_clip_applied;

    const auto& warm_start_head = output.warm_start_head_debug.points[0];
    obs.warm_start_v_s_valid = warm_start_head.valid;
    obs.warm_start_v_s0 = warm_start_head.control_v_s;

    obs.first_shot_valid = output.first_shot_debug.success;
    obs.first_shot_u0_v_s = output.first_shot_debug.u0_v_s;

    start_lock_recovery_.update(obs, input.dt);
    output.start_lock_recovery = start_lock_recovery_.diagnostics();
}

bool SpmpcProblem::solve(const SolverInput& observed_input, SolverOutput& output) {
    const bool result=solveCycle(observed_input,output);
    output.cycle_timing=observed_input.cycle_timing;
    output.pre_solve_snapshot.rti_iterations=solver_params_.rti_iterations;
    output.pre_solve_snapshot.max_prediction_defect=solver_params_.max_prediction_defect;
    output.predicted_horizon.rti_iterations=solver_params_.rti_iterations;
    for (auto* d:{&output.pre_solve_snapshot.planning,&output.predicted_horizon.planning}) {
        d->experiment_profile_id=solver_params_.planning.experiment_profile_id;
        d->region_id=solver_params_.planning.region.enabled ? solver_params_.planning.region.id : "";
        d->plan_id=task_plan_ ? task_plan_->plan_id : "";
        d->reference_mode=trajectoryReferenceModeName(solver_params_.planning.trajectory.mode);
        d->geometry_enabled=solver_params_.planning.geometry.enabled;
        d->task_elapsed_sec=task_clock_.elapsed();
        d->deadline_sec=task_plan_ ? task_plan_->deadline : solver_params_.planning.task_deadline_sec;
    }
    return result;
}

bool SpmpcProblem::solveCycle(const SolverInput& observed_input, SolverOutput& output) {
    SolverInput input = observed_input;
    if (!configuration_error_.empty()) {
        output = SolverOutput{};
        output.status = "INVALID_PLANNING_CONFIG: " + configuration_error_;
        return false;
    }
    if (reference_.empty()) {
        output = SolverOutput{};
        output.status = "NO_REFERENCE_PATH";
        updateStartLockRecovery(input, false, output);
        return false;
    }
    if (!solver_) {
        output = SolverOutput{};
        output.status = "NO_SOLVER";
        updateStartLockRecovery(input, false, output);
        return false;
    }

    const bool timed_task = task_plan_ || solver_params_.planning.task_deadline_sec > 0;
    if (timed_task) {
        if (!task_clock_.observe(input, input.task_elapsed_sec)) {
            output = SolverOutput{}; output.status = "INVALID_TASK_CLOCK"; return false;
        }
        input.has_task_elapsed = true;
    }

    ProgressProjector projector;
    const auto proj = task_reference_ ? task_reference_->project(input.robot.x,input.robot.y,last_progress_s_) :
        projector.project(reference_, input.robot.x, input.robot.y, last_progress_s_);
    if (!proj.valid) {
        output = SolverOutput{};
        output.status = "PROJECTION_FAILED";
        updateStartLockRecovery(input, false, output);
        return false;
    }

    const double len = reference_.length();
    if (motion_region_) {
        try {
            if (reference_.frameId()!=solver_params_.planning.region.frame_id ||
                motion_region_->clearance(input.robot.x,input.robot.y,proj.s)<-1e-6) {
                output=SolverOutput{};output.status="CURRENT_MOTION_REGION_VIOLATION";return false;
            }
        } catch (const std::exception& e) {
            output=SolverOutput{};output.status="INVALID_CURRENT_REGION: "+std::string(e.what());return false;
        }
    }
    const double remaining_s = std::max(0.0, len - proj.s);
    auto goal = reference_.sample(len);
    if (task_plan_) {
        goal.x=task_plan_->goal_pose[0]; goal.y=task_plan_->goal_pose[1]; goal.yaw=task_plan_->goal_pose[2];
    }
    const double dx = goal.x - input.robot.x;
    const double dy = goal.y - input.robot.y;
    const double distance_to_goal = std::hypot(dx, dy);
    TerminalGoalInfo goal_info;
    goal_info.valid = true;
    goal_info.remaining_s = remaining_s;
    goal_info.distance_to_goal = distance_to_goal;
    goal_info.dx_robot = std::cos(input.robot.yaw) * dx + std::sin(input.robot.yaw) * dy;
    goal_info.position_reached = distance_to_goal < terminal_controller_.params().goal_tolerance;
    const bool yaw_ready = !(task_plan_ || solver_params_.terminal.require_goal_yaw) ||
        std::abs(angleDiff(input.robot.yaw, goal.yaw)) <=
        (task_plan_ ? task_plan_->goal_yaw_tolerance : solver_params_.terminal.goal_yaw_tolerance);
    const bool goal_pose_ready = goal_info.position_reached && yaw_ready &&
        (!task_plan_ || distance_to_goal <= task_plan_->goal_position_tolerance);
    goal_info.position_reached = goal_info.position_reached && goal_pose_ready;
    const double deadline = task_plan_ ? task_plan_->deadline : solver_params_.planning.task_deadline_sec;
    if (timed_task && input.task_elapsed_sec > deadline+solver_params_.planning.trajectory.deadline_tolerance &&
        (!goal_info.position_reached || !goal_pose_ready ||
         std::abs(input.robot.v) > (task_plan_ ? task_plan_->stop_speed_tolerance : solver_params_.terminal.goal_reached_max_speed) ||
         !actuatorCommandsClear(input.actuator,solver_params_.task_stop.command_zero_tolerance) ||
         std::abs(input.robot.omega) > (task_plan_ ? task_plan_->stop_omega_tolerance : solver_params_.terminal.goal_reached_max_omega))) {
        output = SolverOutput{}; output.status = "TASK_DEADLINE_MISSED"; return false;
    }
    if (solver_params_.task_stop.enable) {
        goal_info.task_end_approach = remaining_s <= solver_params_.terminal.slowdown_distance;
        goal_info.position_reached = goal_info.position_reached && remaining_s <= solver_params_.terminal.goal_tolerance;
    }

    StopReadiness stop_readiness;
    StopTailPrediction stop_tail;
    std::string stop_failure;
    const bool complete_stop = solver_params_.task_stop.enable;
    if (complete_stop) {
        if (!task_stop_configured_) {
            output = SolverOutput{}; output.status = "INVALID_COMPLETE_STOP_CONFIG"; return false;
        }
        LiquidLimitPolicy limit;
        std::string error;
        if (!makeLiquidLimitPolicy(liquid_limit_enabled_, solver_params_.slosh.slosh_height_max,
                                   solver_params_.liquid_limit, limit, error)) {
            output = SolverOutput{}; output.status = error; return false;
        }
        stop_readiness = task_stop_manager_.observe(input, goal_info.position_reached,
            solver_params_.terminal.goal_reached_max_speed, solver_params_.terminal.goal_reached_max_omega);
        if (!stop_readiness.valid) {
            output = SolverOutput{}; output.status = "INVALID_COMPLETE_STOP_STATE"; return false;
        }
        if (goal_info.task_end_approach || terminal_controller_.diagnostics().command_owned ||
            terminal_controller_.reached()) {
            stop_tail = task_stop_manager_.predict(input);
            if (!stop_tail.valid) stop_failure = stop_tail.status;
            else if (limit.enabled && (stop_tail.peak_height_m > limit.cap_m + 1e-7 ||
                     (limit.physical_boundary_known && stop_tail.peak_height_m >= limit.physical_boundary_m)))
                stop_failure = "STOP_LIQUID_RECOVERY_CAP";
            else if (stop_tail.first_command.v > solver_params_.v_max + 1e-9 ||
                     std::abs(stop_tail.first_command.omega) > solver_params_.omega_max + 1e-9)
                stop_failure = "STOP_COMMAND_BOUND_VIOLATION";
        }
        if (stop_readiness.timed_out) stop_failure = "LIQUID_SETTLE_TIMEOUT";
    }
    const bool vehicle_queues_clear = actuatorCommandsClear(
        input.actuator, solver_params_.task_stop.command_zero_tolerance);
    const bool require_vehicle_queues = solver_params_.jerk_limit_enable &&
        solver_params_.terminal.mpc_stop_handoff_enable;
    const TerminalPlan terminal_plan = terminal_controller_.updateAndPlan(
        goal_info, input.robot.v, input.robot.omega, std::max(1e-6, solver_params_.a_max),
        goal_pose_ready && (!require_vehicle_queues || vehicle_queues_clear) &&
        (!complete_stop || (stop_failure.empty() && stop_tail.valid && stop_readiness.vehicle_stopped &&
                          stop_readiness.excitation_quiet && stop_readiness.liquid_stable)));
    const auto stamp_terminal = [&]() {
        output.cycle_timing = input.cycle_timing;
        output.terminal_diagnostics = terminal_controller_.diagnostics();
        auto& d = output.terminal_diagnostics;
        d.complete_stop_enabled = complete_stop;
        d.delay_queues_clear = vehicle_queues_clear;
        d.vehicle_stopped = goal_info.position_reached && vehicle_queues_clear &&
            std::abs(input.robot.v) <= solver_params_.terminal.goal_reached_max_speed &&
            std::abs(input.robot.omega) <= solver_params_.terminal.goal_reached_max_omega;
        if (!complete_stop) return;
        d.delay_queues_clear = stop_readiness.queues_clear;
        d.vehicle_stopped = stop_readiness.vehicle_stopped;
        d.excitation_quiet = stop_readiness.excitation_quiet;
        d.liquid_stable = stop_readiness.liquid_stable;
        d.settle_timed_out = stop_readiness.timed_out;
        d.residual_height_m = stop_readiness.residual_height_m;
        d.stable_duration_sec = stop_readiness.stable_duration_sec;
        d.vehicle_stop_time_sec = stop_readiness.vehicle_stop_time_sec;
        d.liquid_stable_time_sec = stop_readiness.liquid_stable_time_sec;
        d.predicted_stop_distance_m = stop_tail.distance_m;
        d.predicted_tail_valid = stop_tail.valid;
        d.predicted_tail_duration_sec = stop_tail.duration_sec;
        d.predicted_tail_peak_height_m = stop_tail.peak_height_m;
        d.predicted_tail_residual_height_m = stop_tail.residual_height_m;
        if (terminal_plan.owns_command && !d.reached) d.mode = output.status;
    };
    // Apply the same tail/cap/timeout checks before both STOP and GOAL_REACHED,
    // including calls after the legacy terminal controller latched completion.
    if (complete_stop && terminal_plan.owns_command && !stop_failure.empty()) {
        output = SolverOutput{};
        output.status = stop_failure;
        output.progress_s = len > 1e-6 ? proj.s / len : 0.0;
        output.progress_abs_s = proj.s;
        stamp_terminal();
        output.terminal_diagnostics.reached = false;
        output.terminal_diagnostics.mode = stop_failure;
        updateStartLockRecovery(input, false, output);
        return false;
    }
    if (!terminal_controller_.params().enable && goal_pose_ready &&
        std::abs(input.robot.v) <= solver_params_.terminal.goal_reached_max_speed &&
        std::abs(input.robot.omega) <= solver_params_.terminal.goal_reached_max_omega &&
        (!require_vehicle_queues || vehicle_queues_clear)) {
        output = SolverOutput{};
        output.success = true;
        output.status = "GOAL_REACHED";
        output.progress_s = len > 1e-6 ? proj.s / len : 0.0;
        output.progress_abs_s = proj.s;
        stamp_terminal();
        updateStartLockRecovery(input, true, output);
        return true;
    }
    if (terminal_controller_.reached()) {
        output = SolverOutput{};
        output.success = true;
        output.status = "GOAL_REACHED";
        output.progress_s = len > 1e-6 ? proj.s / len : 0.0;
        output.progress_abs_s = proj.s;
        output.cmd_v = 0.0;
        output.cmd_omega = 0.0;
        stamp_terminal();
        last_progress_s_ = std::max(last_progress_s_, output.progress_abs_s);
        updateStartLockRecovery(input, true, output);
        return true;
    }

    if (terminal_plan.owns_command) {
        output = SolverOutput{};
        bool valid_history = input.actuator.valid &&
            std::isfinite(input.actuator.v_cmd) && std::isfinite(input.actuator.omega_cmd) &&
            std::isfinite(input.dt) && input.dt > 0.0;
        output.success = valid_history;
        output.status = valid_history ? "TERMINAL_STOP" : "TERMINAL_STOP_HISTORY_FAILED";
        if (complete_stop) {
            valid_history = stop_tail.valid;
            output.status = stop_tail.valid ? (stop_readiness.vehicle_stopped ? "TERMINAL_SETTLING" : "TERMINAL_DRAINING") : stop_tail.status;
            if (stop_tail.valid) {
                output.cmd_v = stop_tail.first_command.v;
                output.cmd_omega = stop_tail.first_command.omega;
            }
            if (!valid_history) output.cmd_v = output.cmd_omega = 0.0;
            output.success = valid_history;
        } else if (valid_history && solver_params_.jerk_limit_enable) {
            const auto stop = makeJerkLimitedStopCommand(input.actuator, input.dt,
                solver_params_.a_max, solver_params_.alpha_max, solver_params_.jerk_max);
            valid_history = stop.valid;
            output.success = stop.valid;
            output.status = stop.valid ? "TERMINAL_DRAINING" : stop.status;
            output.cmd_v = stop.v;
            output.cmd_omega = stop.omega;
        } else if (valid_history && !(goal_info.dx_robot < solver_params_.terminal.goal_behind_x)) {
            const auto stop = terminal_controller_.stopCommand(
                input.actuator.v_cmd, input.actuator.omega_cmd, input.dt,
                solver_params_.a_max, solver_params_.alpha_max);
            output.cmd_v = stop.cmd_v_post;
            output.cmd_omega = stop.cmd_omega_post;
        }
        output.progress_s = len > 1e-6 ? proj.s / len : 0.0;
        output.progress_abs_s = proj.s;
        stamp_terminal();
        last_progress_s_ = std::max(last_progress_s_, proj.s);
        updateStartLockRecovery(input, valid_history, output);
        return valid_history;
    }

    SolverInput guarded_input = input;
    guarded_input.min_progress_s = last_progress_s_;
    guarded_input.costmap = have_costmap_ ? &costmap_ : nullptr;
    if (solver_params_.terminal.mpc_stop_handoff_enable && terminal_plan.envelope_active) {
        guarded_input.has_v_ref_current = true;
        guarded_input.v_ref_current = std::min(
            input.has_v_ref_current ? input.v_ref_current : configured_v_ref_,
            terminal_plan.v_envelope);
        guarded_input.v_ref_status = "TERMINAL_MPC_SLOWDOWN";
    }
    if (complete_stop && terminal_plan.terminal_phase) {
        guarded_input.task_stop_active = true;
        guarded_input.task_stop_goal_s = std::max(0.0, len - 0.5*solver_params_.terminal.goal_tolerance);
        // Stopping reference is evaluated at each predicted progress state in OCP.
        // Use nominal cruise here; the old current-only envelope would impose a
        // second, inconsistent speed reference over the entire horizon.
        guarded_input.has_v_ref_current = input.has_v_ref_current;
        guarded_input.v_ref_current = input.v_ref_current;
        guarded_input.v_ref_status = "TASK_GOAL_STOP_PROFILE";
    }
    const bool ok = solver_->solve(guarded_input, reference_, output);
    output.ocp_solve_attempted = true;
    if (ok && output.success) {
        if (!solver_params_.terminal.mpc_stop_handoff_enable) {
            const TerminalClampOutput clamp = terminal_controller_.clampCommand(
                output.cmd_v, output.cmd_omega, input.robot.v, input.dt,
                goal_info, terminal_plan, std::max(1e-6, solver_params_.a_max));
            output.cmd_v = clamp.cmd_v_post;
            output.cmd_omega = clamp.cmd_omega_post;
        }
        last_progress_s_ = std::max(last_progress_s_, output.progress_abs_s);
    }
    stamp_terminal();
    updateStartLockRecovery(input, ok && output.success, output);
    return ok;
}

}  // namespace spmpc_local_planner
