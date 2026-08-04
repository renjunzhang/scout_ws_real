#include "spmpc_sim_local_planner/reference/reference_path_preprocessor.h"
#include "spmpc_sim_local_planner/reference/reference_path.h"
#include <algorithm>
#include <cmath>

namespace spmpc_sim_local_planner {
namespace {

double dist2(const TrajectoryPoint& a, const TrajectoryPoint& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return dx * dx + dy * dy;
}

void recomputeYaw(std::vector<TrajectoryPoint>& points) {
    if (points.size() < 2) {
        return;
    }
    for (size_t i = 0; i < points.size(); ++i) {
        const size_t prev = (i == 0) ? i : i - 1;
        const size_t next = (i + 1 < points.size()) ? i + 1 : i;
        const double dx = points[next].x - points[prev].x;
        const double dy = points[next].y - points[prev].y;
        if (std::hypot(dx, dy) > 1e-9) {
            points[i].yaw = std::atan2(dy, dx);
        }
    }
}

std::vector<TrajectoryPoint> removeNearDuplicates(
    const std::vector<TrajectoryPoint>& points,
    double min_segment_length) {
    if (points.empty()) {
        return {};
    }

    const double min_dist2 = min_segment_length * min_segment_length;
    std::vector<TrajectoryPoint> out;
    out.reserve(points.size());
    out.push_back(points.front());
    for (size_t i = 1; i < points.size(); ++i) {
        if (dist2(out.back(), points[i]) >= min_dist2) {
            out.push_back(points[i]);
        }
    }
    if (out.size() == 1 && points.size() > 1) {
        out.push_back(points.back());
    }
    return out;
}

std::vector<TrajectoryPoint> resampleByArcLength(
    const std::vector<TrajectoryPoint>& points,
    double spacing) {
    if (points.size() < 2 || spacing <= 1e-6) {
        return points;
    }

    ReferencePath path;
    path.setPoints(points, "map");
    const double len = path.length();
    if (len <= spacing) {
        return points;
    }

    std::vector<TrajectoryPoint> out;
    out.reserve(static_cast<size_t>(std::ceil(len / spacing)) + 1);
    for (double s = 0.0; s < len; s += spacing) {
        out.push_back(path.sample(s));
    }
    out.push_back(path.sample(len));
    return out;
}

std::vector<TrajectoryPoint> smoothMovingAverage(
    const std::vector<TrajectoryPoint>& points,
    int window) {
    if (points.size() < 3 || window <= 1) {
        return points;
    }

    const int half = std::max(1, window / 2);
    std::vector<TrajectoryPoint> out = points;
    for (size_t i = 1; i + 1 < points.size(); ++i) {
        double sx = 0.0;
        double sy = 0.0;
        int count = 0;
        const int begin = std::max<int>(0, static_cast<int>(i) - half);
        const int end = std::min<int>(static_cast<int>(points.size()) - 1, static_cast<int>(i) + half);
        for (int j = begin; j <= end; ++j) {
            sx += points[static_cast<size_t>(j)].x;
            sy += points[static_cast<size_t>(j)].y;
            ++count;
        }
        out[i].x = sx / static_cast<double>(count);
        out[i].y = sy / static_cast<double>(count);
    }
    recomputeYaw(out);
    return out;
}

}  // namespace

std::vector<TrajectoryPoint> ReferencePathPreprocessor::preprocess(
    const std::vector<TrajectoryPoint>& points,
    const ReferencePathPreprocessParams& params) const {
    if (!params.enable || points.size() < 2) {
        return points;
    }

    auto out = removeNearDuplicates(points, std::max(0.0, params.min_segment_length));
    if (out.size() < 2) {
        return out;
    }
    out = resampleByArcLength(out, params.resample_spacing);
    out = smoothMovingAverage(out, params.smoothing_window);
    return out;
}

}  // namespace spmpc_sim_local_planner
