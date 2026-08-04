#pragma once

#include "spmpc_sim_local_planner/core/types.h"
#include <vector>

namespace spmpc_sim_local_planner {

struct ReferencePathPreprocessParams {
    bool enable = false;
    double resample_spacing = 0.08;
    int smoothing_window = 3;
    double min_segment_length = 1e-3;
};

class ReferencePathPreprocessor {
public:
    std::vector<TrajectoryPoint> preprocess(
        const std::vector<TrajectoryPoint>& points,
        const ReferencePathPreprocessParams& params) const;
};

}  // namespace spmpc_sim_local_planner
