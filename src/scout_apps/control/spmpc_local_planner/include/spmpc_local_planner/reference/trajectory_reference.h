#pragma once
#include "spmpc_local_planner/reference/trajectory_plan.h"
#include "spmpc_local_planner/reference/progress_projector.h"
#include <memory>

namespace spmpc_local_planner {

class TrajectoryReference {
public:
    explicit TrajectoryReference(std::shared_ptr<const TrajectoryPlan> plan,
                                 const ProgressProjectionConfig& projection_config = ProgressProjectionConfig{});
    explicit TrajectoryReference(const TrajectoryPlan& plan,
                                 const ProgressProjectionConfig& projection_config = ProgressProjectionConfig{})
        : TrajectoryReference(std::make_shared<TrajectoryPlan>(plan), projection_config) {}
    const TrajectoryPlan& plan() const { return *plan_; }
    // Pure sampling: endpoint holds are explicit; no mutable query state.
    TrajectoryPlanSample sampleAtTime(double time) const;
    TrajectoryPlanSample sampleAtProgress(double progress, double task_elapsed) const;
    bool atPlateau(double progress) const;
    ProgressProjection project(double x, double y, double minimum_progress = 0) const;
    ProgressProjection project(double x, double y, ProgressProjectionState& state,
                               double minimum_progress = 0) const;
private:
    std::shared_ptr<const TrajectoryPlan> plan_;
    // A monotone lookup index removes accepted solver roundoff at plateaus;
    // the original full-state samples remain intact for dynamics validation.
    std::vector<double> progress_;
    std::vector<TrajectoryPoint> projection_points_;
    ProgressProjector projector_;
};

}  // namespace spmpc_local_planner
