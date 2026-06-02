#pragma once

#include "spmpc_local_planner/reference/reference_path.h"

namespace spmpc_local_planner {

struct ProgressProjection {
    bool valid = false;
    double s = 0.0;
    double distance = 0.0;
    double signed_distance = 0.0;
    TrajectoryPoint point;
};

class ProgressProjector {
public:
    ProgressProjection project(const ReferencePath& reference, double x, double y) const;
    ProgressProjection project(const ReferencePath& reference, double x, double y, double min_s) const;
};

}  // namespace spmpc_local_planner
