#pragma once

#include "spmpc_local_planner/planning/planning_config.h"
#include "spmpc_local_planner/reference/horizon_reference_builder.h"
#include "spmpc_local_planner/reference/reference_path.h"
#include "spmpc_local_planner/reference/reference_spline.h"
#include "spmpc_local_planner/core/terminal_controller.h"
#include <memory>

namespace spmpc_local_planner {

struct SolverParams;

struct OcpPlanningStage {
    RegionStageData region;
    StageTrajectoryReference reference;
    bool has_geometry_reference = false;
    std::array<double, 4> x_coeffs{}, y_coeffs{};
    bool task_goal_active = false;
    bool terminal_goal_tracking = false;
    // Approach heading for the pose cost only. Deadline constraints always use
    // goal_pose[2], independently of this intermediate heading reference.
    double goal_tracking_yaw = 0.0;
    std::array<double,3> goal_pose{};
    ReferenceSample sampleGeometry(double progress) const;
};

// No ROS or acados dependency. Owns validated region/plan data and assembles
// stage references; generated parameter indices stay in the implementation.
class OcpPlanningAdapter {
public:
    explicit OcpPlanningAdapter(const SolverParams& params);
    const TrajectoryPlan* plan() const;
    ProgressProjection project(const ReferencePath& route, double x, double y, double min_progress = 0) const;
    ProgressProjection project(const ReferencePath& route, double x, double y,
        ProgressProjectionState& state, double min_progress = 0) const;
    std::vector<TrajectoryPlanSample> nominalHorizon(double progress, double elapsed, int count) const;
    std::vector<OcpPlanningStage> prepare(const ReferencePath& route,
        const SolverInput& input, const std::vector<double>& stage_progress,
        PlanningCycleDebug& debug) const;
    void write(const OcpPlanningStage& stage, double* parameters, int width) const;
    bool check(const std::vector<OcpPlanningStage>& stages,
        const PredictedHorizonDebug& horizon, double& minimum_clearance,
        std::string& reason) const;

private:
    PlanningConfig config_;
    TerminalControllerParams terminal_;
    double dt_ = 0.0;
    double max_speed_ = 0.0;
    std::array<double,6> motion_limits_{};
    MotionRegion region_;
    std::vector<RegionStageData> region_stages_;
    std::shared_ptr<const TrajectoryReference> trajectory_;
};

}  // namespace spmpc_local_planner
