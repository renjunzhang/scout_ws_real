#pragma once
#include "spmpc_local_planner/reference/trajectory_plan.h"
#include <memory>

namespace spmpc_local_planner {

class TrajectoryReference {
public:
    explicit TrajectoryReference(std::shared_ptr<const TrajectoryPlan> plan);
    explicit TrajectoryReference(const TrajectoryPlan& plan)
        : TrajectoryReference(std::make_shared<TrajectoryPlan>(plan)) {}
    const TrajectoryPlan& plan() const { return *plan_; }
    // Pure sampling: endpoint holds are explicit; no mutable query state.
    TrajectoryPlanSample sampleAtTime(double time) const;
    TrajectoryPlanSample sampleAtProgress(double progress, double task_elapsed) const;
    bool atPlateau(double progress) const;
private:
    std::shared_ptr<const TrajectoryPlan> plan_;
};

}  // namespace spmpc_local_planner
