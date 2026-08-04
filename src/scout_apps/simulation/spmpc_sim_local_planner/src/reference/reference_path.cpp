#include "spmpc_sim_local_planner/reference/reference_path.h"
#include <algorithm>
#include <cmath>

namespace spmpc_sim_local_planner {
namespace {

double dist2(double ax, double ay, double bx, double by) {
    const double dx = ax - bx;
    const double dy = ay - by;
    return dx * dx + dy * dy;
}

double lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

double normalizeAngle(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

double lerpAngle(double a, double b, double t) {
    return normalizeAngle(a + normalizeAngle(b - a) * t);
}

}  // namespace

void ReferencePath::setPoints(const std::vector<TrajectoryPoint>& points, const std::string& frame_id) {
    points_.clear();
    frame_id_ = frame_id.empty() ? "map" : frame_id;
    total_length_ = 0.0;

    points_.reserve(points.size());
    for (auto p : points) {
        if (!points_.empty()) {
            const auto& prev = points_.back();
            total_length_ += std::sqrt(dist2(prev.x, prev.y, p.x, p.y));
        }
        p.yaw = normalizeAngle(p.yaw);
        p.s = total_length_;
        points_.push_back(p);
    }
}

TrajectoryPoint ReferencePath::sample(double s) const {
    if (points_.empty()) {
        return TrajectoryPoint{};
    }
    if (s <= points_.front().s) {
        return points_.front();
    }
    if (s >= points_.back().s) {
        return points_.back();
    }

    auto it = std::lower_bound(points_.begin(), points_.end(), s,
        [](const TrajectoryPoint& p, double value) { return p.s < value; });
    const auto& b = *it;
    const auto& a = *(it - 1);
    const double ds = std::max(1e-6, b.s - a.s);
    const double t = (s - a.s) / ds;

    TrajectoryPoint out;
    out.x = lerp(a.x, b.x, t);
    out.y = lerp(a.y, b.y, t);
    out.yaw = lerpAngle(a.yaw, b.yaw, t);
    out.v = lerp(a.v, b.v, t);
    out.s = s;
    return out;
}

}  // namespace spmpc_sim_local_planner
