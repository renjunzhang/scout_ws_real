#include "spmpc_sim_local_planner/core/spmpc_problem.h"
#include "spmpc_sim_local_planner/reference/progress_projector.h"
#include "spmpc_sim_local_planner/solvers/solver_factory.h"
#include <algorithm>
#include <cmath>
#include <cstddef>

namespace spmpc_sim_local_planner {
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
    terminal_controller_.setParams(solver_params_.terminal);
    start_lock_recovery_.setParams(solver_params_.start_lock_recovery);
    solver_ = makeSolver(solver_params_.solver_backend);
    solver_->configure(solver_params_, variant);
}

void SpmpcProblem::setReferencePath(const ReferencePath& reference) {
    const bool same_path = sameReferencePath(reference_, reference);
    reference_ = reference;
    if (!same_path) {
        last_progress_s_ = 0.0;
        terminal_controller_.reset();
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

bool SpmpcProblem::solve(const SolverInput& input, SolverOutput& output) {
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

    ProgressProjector projector;
    const auto proj = projector.project(reference_, input.robot.x, input.robot.y, last_progress_s_);
    if (!proj.valid) {
        output = SolverOutput{};
        output.status = "PROJECTION_FAILED";
        updateStartLockRecovery(input, false, output);
        return false;
    }

    const double len = reference_.length();
    const double remaining_s = std::max(0.0, len - proj.s);
    const auto goal = reference_.sample(len);
    const double dx = goal.x - input.robot.x;
    const double dy = goal.y - input.robot.y;
    const double distance_to_goal = std::hypot(dx, dy);
    TerminalGoalInfo goal_info;
    goal_info.valid = true;
    goal_info.remaining_s = remaining_s;
    goal_info.distance_to_goal = distance_to_goal;
    goal_info.dx_robot = std::cos(input.robot.yaw) * dx + std::sin(input.robot.yaw) * dy;
    goal_info.position_reached = distance_to_goal < terminal_controller_.params().goal_tolerance;

    const TerminalPlan terminal_plan = terminal_controller_.updateAndPlan(
        goal_info, input.robot.v, input.robot.omega, std::max(1e-6, solver_params_.a_max));
    if (!terminal_controller_.params().enable && goal_info.position_reached) {
        output = SolverOutput{};
        output.success = true;
        output.status = "GOAL_REACHED";
        output.progress_s = len > 1e-6 ? proj.s / len : 0.0;
        output.progress_abs_s = proj.s;
        output.terminal_diagnostics = terminal_controller_.diagnostics();
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
        output.terminal_diagnostics = terminal_controller_.diagnostics();
        last_progress_s_ = std::max(last_progress_s_, output.progress_abs_s);
        updateStartLockRecovery(input, true, output);
        return true;
    }

    SolverInput guarded_input = input;
    guarded_input.min_progress_s = last_progress_s_;
    guarded_input.costmap = have_costmap_ ? &costmap_ : nullptr;
    const bool ok = solver_->solve(guarded_input, reference_, output);
    if (ok && output.success) {
        const TerminalClampOutput clamp = terminal_controller_.clampCommand(
            output.cmd_v,
            output.cmd_omega,
            input.robot.v,
            input.dt,
            goal_info,
            terminal_plan,
            std::max(1e-6, solver_params_.a_max));
        output.cmd_v = clamp.cmd_v_post;
        output.cmd_omega = clamp.cmd_omega_post;
        last_progress_s_ = std::max(last_progress_s_, output.progress_abs_s);
    }
    output.terminal_diagnostics = terminal_controller_.diagnostics();
    updateStartLockRecovery(input, ok && output.success, output);
    return ok;
}

}  // namespace spmpc_sim_local_planner
