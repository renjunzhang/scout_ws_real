#pragma once

#include <cstdint>
#include <vector>

namespace spmpc_sim_local_planner {

class CostmapGrid {
public:
    void setGrid(
        unsigned int width,
        unsigned int height,
        double resolution,
        double origin_x,
        double origin_y,
        double origin_yaw,
        const std::vector<int8_t>& data);

    bool empty() const { return data_.empty() || width_ == 0 || height_ == 0 || resolution_ <= 0.0; }
    int costAtWorld(double x, double y) const;
    int maxCostInRadius(double x, double y, double radius) const;

private:
    bool worldToMap(double x, double y, int& mx, int& my) const;
    int costAtCell(int mx, int my) const;

    unsigned int width_ = 0;
    unsigned int height_ = 0;
    double resolution_ = 0.0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double origin_yaw_ = 0.0;
    std::vector<int8_t> data_;
};

}  // namespace spmpc_sim_local_planner
