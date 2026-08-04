#include "spmpc_sim_local_planner/core/costmap_grid.h"
#include <algorithm>
#include <cmath>
#include <cstddef>

namespace spmpc_sim_local_planner {

void CostmapGrid::setGrid(
    unsigned int width,
    unsigned int height,
    double resolution,
    double origin_x,
    double origin_y,
    double origin_yaw,
    const std::vector<int8_t>& data) {
    width_ = width;
    height_ = height;
    resolution_ = resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;
    origin_yaw_ = origin_yaw;

    if (width_ == 0 || height_ == 0 || resolution_ <= 0.0) {
        data_.clear();
        return;
    }

    const size_t expected_size = static_cast<size_t>(width_) * static_cast<size_t>(height_);
    data_.assign(expected_size, static_cast<int8_t>(-1));
    std::copy_n(data.begin(), std::min(data.size(), expected_size), data_.begin());
}

bool CostmapGrid::worldToMap(double x, double y, int& mx, int& my) const {
    if (empty()) {
        return false;
    }
    const double dx = x - origin_x_;
    const double dy = y - origin_y_;
    const double c = std::cos(origin_yaw_);
    const double s = std::sin(origin_yaw_);
    const double map_x = c * dx + s * dy;
    const double map_y = -s * dx + c * dy;
    mx = static_cast<int>(std::floor(map_x / resolution_));
    my = static_cast<int>(std::floor(map_y / resolution_));
    return mx >= 0 && my >= 0 && mx < static_cast<int>(width_) && my < static_cast<int>(height_);
}

int CostmapGrid::costAtCell(int mx, int my) const {
    if (mx < 0 || my < 0 || mx >= static_cast<int>(width_) || my >= static_cast<int>(height_)) {
        return 100;
    }
    const auto raw = data_[static_cast<size_t>(my) * width_ + static_cast<size_t>(mx)];
    if (raw < 0) {
        return 100;
    }
    return std::max(0, std::min(100, static_cast<int>(raw)));
}

int CostmapGrid::costAtWorld(double x, double y) const {
    int mx = 0;
    int my = 0;
    if (!worldToMap(x, y, mx, my)) {
        return 100;
    }
    return costAtCell(mx, my);
}

int CostmapGrid::maxCostInRadius(double x, double y, double radius) const {
    int cx = 0;
    int cy = 0;
    if (!worldToMap(x, y, cx, cy)) {
        return 100;
    }
    const int r_cells = std::max(0, static_cast<int>(std::ceil(radius / std::max(1e-6, resolution_))));
    int max_cost = 0;
    for (int dy = -r_cells; dy <= r_cells; ++dy) {
        for (int dx = -r_cells; dx <= r_cells; ++dx) {
            if (dx * dx + dy * dy > r_cells * r_cells) {
                continue;
            }
            max_cost = std::max(max_cost, costAtCell(cx + dx, cy + dy));
        }
    }
    return max_cost;
}

}  // namespace spmpc_sim_local_planner
