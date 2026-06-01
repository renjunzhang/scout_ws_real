#include "spmpc_local_planner/reference/progress_projector.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace spmpc_local_planner {
namespace {

double sqr(double v) {
    return v * v;
}

double clamp01(double v) {
    return std::max(0.0, std::min(1.0, v));
}

double segmentLength(const TrajectoryPoint& a, const TrajectoryPoint& b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

}  // namespace

ProgressProjection ProgressProjector::project(const ReferencePath& reference, double x, double y) const {
    ProgressProjection best;
    const auto& pts = reference.points();
    if (pts.empty()) {
        return best;
    }
    if (pts.size() == 1) {
        best.valid = true;
        best.point = pts.front();
        best.s = pts.front().s;
        best.distance = std::hypot(x - pts.front().x, y - pts.front().y);
        return best;
    }

    double best_d2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const auto& a = pts[i];
        const auto& b = pts[i + 1];
        const double vx = b.x - a.x;
        const double vy = b.y - a.y;
        const double seg_len2 = vx * vx + vy * vy;
        if (seg_len2 < 1e-12) {
            continue;
        }

        const double t = clamp01(((x - a.x) * vx + (y - a.y) * vy) / seg_len2);
        const double px = a.x + t * vx;
        const double py = a.y + t * vy;
        const double d2 = sqr(x - px) + sqr(y - py);
        if (d2 < best_d2) {
            best_d2 = d2;
            best.valid = true;
            best.point.x = px;
            best.point.y = py;
            best.point.yaw = std::atan2(vy, vx);
            best.point.v = a.v + t * (b.v - a.v);
            best.point.s = a.s + t * segmentLength(a, b);
            best.s = best.point.s;
            best.distance = std::sqrt(d2);
        }
    }

    return best;
}

}  // namespace spmpc_local_planner
