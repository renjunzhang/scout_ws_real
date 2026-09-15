#include "spmpc_local_planner/planning/ocp_planning_adapter.h"
#include "spmpc_local_planner/core/spmpc_solver.h"
#include "spmpc_local_planner/dynamics/explicit_state_rollout.h"
#include "../core/generated/ocp_parameter_contract.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace spmpc_local_planner {
using namespace ocp_parameters;
namespace {
std::array<double, 4> fitCubic(const std::array<double, 4>& s, const std::array<double, 4>& values) {
    const double origin = s.front(), length = s.back()-s.front();
    if (length < 1e-6) throw std::invalid_argument("degenerate planned geometry domain");
    Eigen::Matrix4d A;
    Eigen::Vector4d b;
    for (int i=0; i<4; ++i) {
        const double q = (s[i]-origin)/length;
        A.row(i) << 1, q, q*q, q*q*q; b(i)=values[i];
    }
    const Eigen::Vector4d c = A.colPivHouseholderQr().solve(b);
    // Keep coefficients in q=(s-s0)/(s3-s0). Expanding a narrow window
    // into global s creates large cancelling terms and corrupts derivatives.
    return {{c(0), c(1), c(2), c(3)}};
}
void requireClose(double a, double b, const char* label) {
    if (!std::isfinite(a) || std::abs(a-b) > 1e-8*(1+std::abs(b)))
        throw std::invalid_argument(std::string("plan/runtime mismatch: ")+label);
}
}

OcpPlanningAdapter::OcpPlanningAdapter(const SolverParams& params)
    : config_(params.planning), terminal_(params.terminal), dt_(params.actuator.dt),
      max_speed_(std::max(params.v_max, std::abs(params.actual_v_min))), region_(config_.region) {
    motion_limits_={{params.actual_v_min,params.v_max,params.omega_max,params.a_max,params.alpha_max,params.jerk_max}};
    std::string reason;
    if (!validatePlanningConfig(config_, &reason)) throw std::invalid_argument(reason);
    if (!std::isfinite(params.actual_v_min) || params.actual_v_min > 0 ||
        !std::isfinite(params.v_max) || params.v_max <= 0 ||
        std::abs(params.actual_v_min)>params.v_max || !std::isfinite(dt_) || dt_ <= 0)
        throw std::invalid_argument("invalid planning motion bounds");
    for (size_t i=2;i<motion_limits_.size();++i)
        if (!std::isfinite(motion_limits_[i]) || motion_limits_[i]<=0)
            throw std::invalid_argument("invalid positive motion bound");
    for (size_t i = 0; i < config_.region.cells.size(); ++i) {
        if (!config_.region.enabled) break;
        // A circumscribed footprint disc plus v_max*dt protects the swept
        // interval, not just its endpoints; no orientation approximation.
        region_stages_.push_back(region_.stageForCell(i, max_speed_*dt_));
    }
    if (config_.trajectory.mode != TrajectoryReferenceMode::Cruise) {
        trajectory_ = std::make_shared<TrajectoryReference>(TrajectoryPlan::load(config_.trajectory.plan_file));
        const auto& plan=trajectory_->plan();
        requireClose(plan.dt, dt_, "dt");
        requireClose(plan.stop_window,config_.evaluation_window_sec,"evaluation window");
        requireClose(plan.goal_position_tolerance,terminal_.goal_tolerance,"goal position tolerance");
        requireClose(plan.goal_yaw_tolerance,terminal_.goal_yaw_tolerance,"goal yaw tolerance");
        requireClose(plan.stop_speed_tolerance,terminal_.goal_reached_max_speed,"stop speed tolerance");
        requireClose(plan.stop_omega_tolerance,terminal_.goal_reached_max_omega,"stop omega tolerance");
        const auto& expected_region=plan.region;
        if (!config_.region.enabled || config_.region.id!=expected_region.id ||
            config_.region.frame_id!=expected_region.frame_id || config_.region.cells.size()!=expected_region.cells.size())
            throw std::invalid_argument("plan/runtime region identity differs");
        requireClose(config_.region.footprint_radius,expected_region.footprint_radius,"footprint radius");
        requireClose(config_.region.margin,expected_region.margin,"region margin");
        for (size_t i=0;i<expected_region.cells.size();++i) {
            const auto& a=config_.region.cells[i]; const auto& b=expected_region.cells[i];
            if (a.id!=b.id || a.vertices.size()!=b.vertices.size()) throw std::invalid_argument("region cell differs");
            requireClose(a.s_begin,b.s_begin,"region begin"); requireClose(a.s_end,b.s_end,"region end");
            for (size_t j=0;j<a.vertices.size();++j) {
                requireClose(a.vertices[j].x,b.vertices[j].x,"region vertex x");
                requireClose(a.vertices[j].y,b.vertices[j].y,"region vertex y");
            }
        }
        const std::array<double,4> actuator{{params.actuator.linear_tau_sec, params.actuator.angular_tau_sec,
            params.actuator.linear_gain, params.actuator.angular_gain}};
        const std::array<double,6> limits{{params.actual_v_min, params.v_max, params.omega_max,
            params.a_max, params.alpha_max, params.jerk_max}};
        SloshDynamics liquid;
        if (!liquid.configure(params.slosh)) throw std::invalid_argument("invalid plan liquid model");
        for (size_t i=0; i<4; ++i) {
            requireClose(plan.actuator_parameters[i], actuator[i], "actuator");
            requireClose(plan.liquid_parameters[i], liquid.coefficients().values()[i], "liquid");
        }
        requireClose(plan.height_coeff, liquid.heightCoeff(), "height coefficient");
        for (size_t i=0; i<6; ++i) requireClose(plan.motion_limits[i], limits[i], "motion limits");
        for (size_t k=0; k+1<plan.samples.size(); ++k) {
            std::vector<double> next;
            if (!stepExplicitState(plan.samples[k].state, plan.samples[k].control, params.actuator, liquid, dt_, next))
                throw std::invalid_argument("plan dynamics validation failed");
            for (size_t j=0; j<28; ++j) if (std::abs(next[j]-plan.samples[k+1].state[j])>2e-6)
                throw std::invalid_argument("plan does not follow the shared full-state dynamics");
        }
    }
}

ReferenceSample OcpPlanningStage::sampleGeometry(double progress) const {
    if (!has_geometry_reference) throw std::logic_error("stage has no planned geometry");
    const double q=(progress-reference.s_knots.front())/(reference.s_knots.back()-reference.s_knots.front());
    const auto value=[&](const std::array<double,4>& c) {return ((c[3]*q+c[2])*q+c[1])*q+c[0];};
    const auto derivative=[&](const std::array<double,4>& c) {return (3*c[3]*q+2*c[2])*q+c[1];};
    const double dx=derivative(x_coeffs),dy=derivative(y_coeffs);
    ReferenceSample out;
    out.x=value(x_coeffs);out.y=value(y_coeffs);out.s=progress;out.psi=std::atan2(dy,dx);
    out.kappa=(dx*(6*y_coeffs[3]*q+2*y_coeffs[2])-dy*(6*x_coeffs[3]*q+2*x_coeffs[2]))/
        std::pow(dx*dx+dy*dy+1e-6,1.5);
    return out;
}

ProgressProjection OcpPlanningAdapter::project(const ReferencePath& route, double x, double y, double min_progress) const {
    return trajectory_ ? trajectory_->project(x,y,min_progress) : ProgressProjector{}.project(route,x,y,min_progress);
}

std::vector<TrajectoryPlanSample> OcpPlanningAdapter::nominalHorizon(double progress, double elapsed, int count) const {
    if (!trajectory_) return {};
    double origin=elapsed;
    if (config_.trajectory.mode==TrajectoryReferenceMode::Progress &&
        progress>trajectory_->plan().samples.front().state[4]+1e-9 && !trajectory_->atPlateau(progress))
        origin=trajectory_->sampleAtProgress(progress,elapsed).t;
    std::vector<TrajectoryPlanSample> rows;
    for (int k=0;k<=count;++k) rows.push_back(trajectory_->sampleAtTime(origin+k*dt_));
    return rows;
}

std::vector<OcpPlanningStage> OcpPlanningAdapter::prepare(const ReferencePath& route,
    const SolverInput& input, const std::vector<double>& progress, PlanningCycleDebug& debug) const {
    if (std::abs(input.dt-dt_) > 1e-8) throw std::invalid_argument("planning dt differs from actuator dt");
    if (config_.region.enabled && (route.frameId() != config_.region.frame_id ||
        config_.region.cells.front().s_begin > 0 || config_.region.cells.back().s_end < route.length()-1e-8))
        throw std::invalid_argument("motion region frame/progress does not cover the original route");
    debug = PlanningCycleDebug{};
    debug.experiment_profile_id = config_.experiment_profile_id;
    debug.region_id = config_.region.enabled ? config_.region.id : "";
    debug.reference_mode = trajectoryReferenceModeName(config_.trajectory.mode);
    debug.geometry_enabled = config_.geometry.enabled;
    debug.task_elapsed_sec = input.task_elapsed_sec;
    debug.reference_status = trajectory_ ? "PLAN" : "CRUISE";
    std::vector<StageTrajectoryReference> references(progress.size());
    if (trajectory_) {
        const auto& plan = trajectory_->plan();
        std::vector<RegionVertex> route_points;
        for (const auto& point : route.points()) route_points.push_back({point.x, point.y});
        if (!validateRoute(route_points, route.frameId(), config_.trajectory.route_tolerance, plan) ||
            plan.region_id != config_.region.id || !config_.region.enabled)
            throw std::invalid_argument("trajectory route/frame/region identity mismatch");
        if (!input.has_task_elapsed || !std::isfinite(input.task_elapsed_sec) || input.task_elapsed_sec < 0)
            throw std::invalid_argument("trajectory requires a persistent task clock");
        debug.plan_id = plan.plan_id;
        debug.deadline_sec = plan.deadline;
        references = HorizonReferenceBuilder::build(*trajectory_, config_.trajectory, progress,
                                                     input.task_elapsed_sec, dt_);
    }
    std::vector<OcpPlanningStage> result(progress.size());
    for (size_t k = 0; k < progress.size(); ++k) {
        if (!std::isfinite(progress[k])) throw std::invalid_argument("nonfinite stage progress");
        auto& stage = result[k];
        const auto endpoint=route.sample(route.length());
        stage.goal_pose=trajectory_ ? trajectory_->plan().goal_pose : std::array<double,3>{{endpoint.x,endpoint.y,endpoint.yaw}};
        const double deadline=trajectory_ ? trajectory_->plan().deadline : config_.task_deadline_sec;
        stage.task_goal_active=deadline>0 && input.task_elapsed_sec+k*dt_>=deadline-1e-9;
        if (config_.region.enabled) {
            bool found = false;
            for (const auto& cell : region_stages_) {
                if (progress[k] >= cell.s_begin-kProgressTolerance && progress[k] <= cell.s_end+kProgressTolerance) {
                    stage.region = cell; found = true; break;
                }
            }
            if (!found) throw std::out_of_range("stage progress outside region");
        }
        stage.reference = references[k];
        if (trajectory_) {
            stage.has_geometry_reference = true;
            stage.x_coeffs = fitCubic(stage.reference.s_knots, stage.reference.x_knots);
            stage.y_coeffs = fitCubic(stage.reference.s_knots, stage.reference.y_knots);
        }
        debug.stage_region_ids.push_back(stage.region.cell_id);
        debug.nominal_times.push_back(stage.reference.nominal_time);
        debug.nominal_phases.push_back(stage.reference.phase);
    }
    return result;
}

void OcpPlanningAdapter::write(const OcpPlanningStage& stage, double* p, int width) const {
    if (!p || (width != kB0ParameterCount && width != PARAM_MAX))
        throw std::invalid_argument("planning parameter schema mismatch");
    const auto& g = config_.geometry;
    p[W_CURVATURE] = g.enabled ? g.curvature_weight : 0.0;
    p[W_CURVATURE_RATE] = g.enabled ? g.curvature_rate_weight : 0.0;
    p[CURVATURE_SPEED_FLOOR] = g.speed_regularization;
    p[CURVATURE_REF] = g.curvature_scale;
    p[CURVATURE_RATE_REF] = g.curvature_rate_scale;
    p[TASK_GOAL_ACTIVE]=stage.task_goal_active ? 1 : 0;
    p[TASK_GOAL_X]=stage.goal_pose[0]; p[TASK_GOAL_Y]=stage.goal_pose[1]; p[TASK_GOAL_YAW]=stage.goal_pose[2];
    p[TASK_GOAL_REQUIRE_YAW]=(trajectory_ || terminal_.require_goal_yaw) ? 1 : 0;
    p[TASK_GOAL_POSITION_TOLERANCE]=trajectory_ ? trajectory_->plan().goal_position_tolerance : terminal_.goal_tolerance;
    p[TASK_GOAL_YAW_TOLERANCE]=trajectory_ ? trajectory_->plan().goal_yaw_tolerance : terminal_.goal_yaw_tolerance;
    p[TASK_GOAL_SPEED_TOLERANCE]=trajectory_ ? trajectory_->plan().stop_speed_tolerance : terminal_.goal_reached_max_speed;
    p[TASK_GOAL_OMEGA_TOLERANCE]=trajectory_ ? trajectory_->plan().stop_omega_tolerance : terminal_.goal_reached_max_omega;
    p[W_TASK_GOAL]=g.enabled ? g.goal_weight : 0.;
    p[TASK_GOAL_POSITION_SCALE]=g.goal_position_scale;
    p[TASK_GOAL_YAW_SCALE]=g.goal_yaw_scale;
    p[REFERENCE_CURVATURE_SPEED_ENABLE] = g.reference_curvature_speed_limit ? 1 : 0;
    const auto& ref = stage.reference;
    if (ref.mode < 0 || ref.mode > 2) throw std::invalid_argument("invalid stage reference mode");
    p[REFERENCE_MODE] = ref.mode;
    for (int i = 0; i < 4; ++i) {
        if (!std::isfinite(ref.s_knots[i]) || !std::isfinite(ref.v_knots[i]) ||
            !std::isfinite(ref.vs_knots[i]) || (i && ref.s_knots[i] <= ref.s_knots[i-1]))
            throw std::invalid_argument("invalid reference interpolation knots");
        p[REFERENCE_S0+i] = ref.s_knots[i];
        p[REFERENCE_V0+i] = ref.v_knots[i];
        p[REFERENCE_VS0+i] = ref.vs_knots[i];
    }
    p[REGION_ACTIVE] = stage.region.enabled ? 1 : 0;
    p[REGION_S_BEGIN] = stage.region.enabled ? stage.region.s_begin : 0;
    p[REGION_S_END] = stage.region.enabled ? stage.region.s_end : 1;
    for (int i = 0; i < static_cast<int>(kMaxRegionFaces); ++i) {
        const auto& f = stage.region.faces[i];
        p[REGION_NX0+3*i] = f.nx;
        p[REGION_NY0+3*i] = f.ny;
        p[REGION_OFFSET0+3*i] = std::hypot(f.nx, f.ny) > 0 ? f.offset : 1.0;
    }
    if (stage.has_geometry_reference) for (int i = 0; i < 4; ++i) {
        p[RX0+i] = stage.x_coeffs[i]; p[RY0+i] = stage.y_coeffs[i];
    }
}

bool OcpPlanningAdapter::check(const std::vector<OcpPlanningStage>& stages,
    const PredictedHorizonDebug& horizon, double& minimum_clearance, std::string& reason) const {
    if (stages.empty() || stages.size() != horizon.states.size() || horizon.controls.size()+1!=stages.size()) {
        reason = "PLANNING_HORIZON_SIZE"; return false;
    }
    for (const auto& u:horizon.controls) {
        if (!std::isfinite(u.a) || !std::isfinite(u.alpha_or_omega) || !std::isfinite(u.v_s) ||
            std::abs(u.a)>motion_limits_[3]+1e-6 || std::abs(u.alpha_or_omega)>motion_limits_[4]+1e-6 ||
            u.v_s<-1e-6 || u.v_s>motion_limits_[1]+1e-6) {
            reason="MOTION_CONTROL_BOUND_VIOLATION"; return false;
        }
    }
    minimum_clearance = config_.region.enabled ? std::numeric_limits<double>::infinity() : 0.0;
    for (size_t k = 0; k < stages.size(); ++k) {
        const auto& x = horizon.states[k]; const auto& stage = stages[k];
        if (!std::isfinite(x.x) || !std::isfinite(x.y) || !std::isfinite(x.s)) {
            reason = "NONFINITE_PLANNING_STATE"; return false;
        }
        const auto& state=x.model_state;
        if ((state.size()!=24 && state.size()!=28) ||
            !std::all_of(state.begin(),state.end(),[](double v){return std::isfinite(v);})) {
            reason="INVALID_PREDICTED_STATE"; return false;
        }
        // RTI status alone is not a feasibility certificate. Check every
        // bounded state, including the terminal node and queued commands.
        if (k>0) {
            const auto within=[](double v,double lo,double hi){return v>=lo-1e-6 && v<=hi+1e-6;};
            bool valid=within(x.v,motion_limits_[0],motion_limits_[1]) &&
                within(x.omega,-motion_limits_[2],motion_limits_[2]);
            for (size_t j=6;j<24;++j) {
                double lo=0,hi=motion_limits_[1];
                if (j==7 || (j>=13 && j<=22)) {lo=-motion_limits_[2];hi=motion_limits_[2];}
                if (j==23) {lo=-motion_limits_[3];hi=motion_limits_[3];}
                if (stage.task_goal_active) lo=hi=0.;
                valid=valid && within(state[j],lo,hi);
            }
            if (!valid) {reason="MOTION_STATE_BOUND_VIOLATION";return false;}
        }
        if (stage.task_goal_active) {
            const double pos_tol=trajectory_ ? trajectory_->plan().goal_position_tolerance : terminal_.goal_tolerance;
            const double yaw_tol=trajectory_ ? trajectory_->plan().goal_yaw_tolerance : terminal_.goal_yaw_tolerance;
            const double v_tol=trajectory_ ? trajectory_->plan().stop_speed_tolerance : terminal_.goal_reached_max_speed;
            const double w_tol=trajectory_ ? trajectory_->plan().stop_omega_tolerance : terminal_.goal_reached_max_omega;
            const double yaw_error=std::atan2(std::sin(x.yaw-stage.goal_pose[2]),std::cos(x.yaw-stage.goal_pose[2]));
            if (std::hypot(x.x-stage.goal_pose[0],x.y-stage.goal_pose[1])>pos_tol+1e-5 ||
                ((trajectory_ || terminal_.require_goal_yaw) && std::abs(yaw_error)>yaw_tol+1e-5) ||
                std::abs(x.v)>v_tol+1e-5 || std::abs(x.omega)>w_tol+1e-5) {
                reason="TASK_GOAL_CONSTRAINT_VIOLATION"; return false;
            }
        }
        if (stage.region.enabled) {
            for (const auto& face : stage.region.faces) if (std::hypot(face.nx, face.ny) > 0) {
                const double clearance = face.offset-face.nx*x.x-face.ny*x.y;
                minimum_clearance = std::min(minimum_clearance, clearance);
            }
            if (minimum_clearance < -1e-6 || x.s < stage.region.s_begin-1e-6 || x.s > stage.region.s_end+1e-6) {
                reason = "MOTION_REGION_CONSTRAINT_VIOLATION"; return false;
            }
        }
        if (stage.reference.mode != 0 && (x.s < stage.reference.s_knots.front()-1e-6 ||
                                         x.s > stage.reference.s_knots.back()+1e-6)) {
            reason = "REFERENCE_DOMAIN_VIOLATION"; return false;
        }
    }
    reason.clear(); return true;
}

}  // namespace spmpc_local_planner
