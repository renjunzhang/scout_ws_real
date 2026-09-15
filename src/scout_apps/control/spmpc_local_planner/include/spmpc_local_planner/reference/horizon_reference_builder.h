#pragma once
#include "spmpc_local_planner/reference/trajectory_reference.h"
#include "spmpc_local_planner/planning/planning_config.h"

namespace spmpc_local_planner {
struct StageTrajectoryReference {
    int mode = 0;
    std::array<double, 4> s_knots{{0, 1, 2, 3}};
    std::array<double, 4> v_knots{}, vs_knots{}, x_knots{}, y_knots{};
    double nominal_time = 0;
    std::string phase;
};

class HorizonReferenceBuilder {
public:
    static std::vector<StageTrajectoryReference> build(const TrajectoryReference& reference,
        const TrajectoryReferenceConfig& config, const std::vector<double>& stage_progress,
        double task_elapsed, double dt);
};
}  // namespace spmpc_local_planner
