#pragma once

#include "spmpc_sim_local_planner/core/types.h"

namespace spmpc_sim_local_planner {

class ReferencePath {
public:
    void setPoints(const std::vector<TrajectoryPoint>& points, const std::string& frame_id);

    bool empty() const { return points_.empty(); }
    const std::string& frameId() const { return frame_id_; }
    double length() const { return total_length_; }
    const std::vector<TrajectoryPoint>& points() const { return points_; }
    TrajectoryPoint sample(double s) const;

private:
    std::vector<TrajectoryPoint> points_;
    std::string frame_id_ = "map";
    double total_length_ = 0.0;
};

}  // namespace spmpc_sim_local_planner
