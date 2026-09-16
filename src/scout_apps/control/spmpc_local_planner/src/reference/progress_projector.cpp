#include "spmpc_local_planner/reference/progress_projector.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace spmpc_local_planner {
namespace {

constexpr double kProgressEpsilon = 1e-9;

double clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

double lerp(double a, double b, double q) {
    return a + q * (b - a);
}

double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

double lerpAngle(double a, double b, double q) {
    return normalizeAngle(a + q * normalizeAngle(b - a));
}

struct Candidate {
    ProgressProjection projection;
    double squared_distance = std::numeric_limits<double>::infinity();
};

bool validConfig(const ProgressProjectionConfig& config) {
    return std::isfinite(config.lookahead) && config.lookahead > 0.0 &&
           std::isfinite(config.local_minimum_tolerance) &&
           config.local_minimum_tolerance >= 0.0;
}

double comparisonTolerance(double a, double b, double relative_tolerance) {
    return relative_tolerance * std::max(1.0, std::max(std::abs(a), std::abs(b)));
}

// Find the first local minimum in path order.  Equal-distance candidates are
// one plateau: deciding from only one neighbour would accept the first
// waiting sample even when the path starts getting closer afterwards.
bool firstLocalMinimum(const std::vector<Candidate>& candidates,
                       double tolerance, std::size_t& selected) {
    for (std::size_t begin = 0; begin < candidates.size();) {
        std::size_t end = begin + 1;
        while (end < candidates.size() &&
               std::abs(candidates[end].squared_distance -
                        candidates[begin].squared_distance) <=
                   comparisonTolerance(candidates[end].squared_distance,
                                       candidates[begin].squared_distance, tolerance)) {
            ++end;
        }
        const double first = candidates[begin].squared_distance;
        const bool lower_before = begin > 0 &&
            candidates[begin - 1].squared_distance <
                first - comparisonTolerance(candidates[begin - 1].squared_distance, first, tolerance);
        const bool lower_after = end < candidates.size() &&
            candidates[end].squared_distance <
                candidates[end - 1].squared_distance -
                    comparisonTolerance(candidates[end - 1].squared_distance,
                                         candidates[end].squared_distance, tolerance);
        // A path boundary is a valid minimum when the distance first rises.
        // In the interior, distance must have descended into the plateau: the
        // preceding candidate is higher, rather than lower.
        if (!lower_after && (begin == 0 || !lower_before)) {
            selected = begin;
            return true;
        }
        begin = end;
    }
    return false;
}

ProgressProjection projectPoints(const std::vector<TrajectoryPoint>& points,
                                 double x, double y,
                                 ProgressProjectionState& state,
                                 double min_s,
                                 const ProgressProjectionConfig& config,
                                 bool derive_yaw_from_geometry) {
    ProgressProjection invalid;
    if (points.empty() || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(min_s) ||
        !validConfig(config)) return invalid;

    const double end = std::max(0.0, points.back().s);
    double lower = std::max(0.0, std::min(end, min_s));
    double upper = end;
    if (state.initialized) {
        if (!std::isfinite(state.progress)) {
            state.reset();
        } else {
            const double previous = std::max(0.0, std::min(end, state.progress));
            lower = std::max(lower, previous);
            upper = std::min(end, previous + config.lookahead);
            if (upper < lower) upper = lower;
        }
    }

    if (points.size() == 1) {
        if (points.front().s + kProgressEpsilon < lower ||
            points.front().s - kProgressEpsilon > upper) return invalid;
        invalid.valid = true;
        invalid.point = points.front();
        invalid.s = points.front().s;
        invalid.distance = std::hypot(x - invalid.point.x, y - invalid.point.y);
        state.initialized = true;
        state.progress = invalid.s;
        return invalid;
    }

    std::vector<Candidate> candidates;
    candidates.reserve(points.size() - 1);
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
        const auto& a = points[i];
        const auto& b = points[i + 1];
        if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.s) ||
            !std::isfinite(b.x) || !std::isfinite(b.y) || !std::isfinite(b.s) ||
            b.s + kProgressEpsilon < a.s || b.s + kProgressEpsilon < lower ||
            a.s - kProgressEpsilon > upper) continue;

        const double ds = b.s - a.s;
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double norm2 = dx * dx + dy * dy;
        const double q_lower = ds > kProgressEpsilon ? clamp01((lower - a.s) / ds) : 0.0;
        const double q_upper = ds > kProgressEpsilon ? clamp01((upper - a.s) / ds) : 1.0;
        if (q_lower > q_upper + kProgressEpsilon) continue;

        double q = q_lower;
        if (norm2 > 1e-16) {
            q = clamp01(((x - a.x) * dx + (y - a.y) * dy) / norm2);
            q = std::max(q_lower, std::min(q_upper, q));
        }

        const double px = lerp(a.x, b.x, q);
        const double py = lerp(a.y, b.y, q);
        const double ex = x - px;
        const double ey = y - py;
        Candidate candidate;
        candidate.squared_distance = ex * ex + ey * ey;
        candidate.projection.valid = true;
        candidate.projection.point = a;
        candidate.projection.point.x = px;
        candidate.projection.point.y = py;
        candidate.projection.point.v = lerp(a.v, b.v, q);
        candidate.projection.point.s = lerp(a.s, b.s, q);
        if (derive_yaw_from_geometry && norm2 > 1e-16) {
            candidate.projection.point.yaw = std::atan2(dy, dx);
        } else {
            candidate.projection.point.yaw = lerpAngle(a.yaw, b.yaw, q);
        }
        candidate.projection.s = candidate.projection.point.s;
        candidate.projection.distance = std::sqrt(candidate.squared_distance);
        if (norm2 > 1e-16) {
            const double inverse_norm = 1.0 / std::sqrt(norm2);
            candidate.projection.signed_distance = ex * (-dy * inverse_norm) +
                                                    ey * (dx * inverse_norm);
        }
        candidates.push_back(candidate);
    }

    if (candidates.empty()) return invalid;

    std::size_t selected = 0;
    if (state.initialized) {
        const bool found_local = firstLocalMinimum(
            candidates, config.local_minimum_tolerance, selected);
        if (!found_local) {
            for (std::size_t i = 1; i < candidates.size(); ++i) {
                if (candidates[i].squared_distance <
                    candidates[selected].squared_distance) selected = i;
            }
        }
    } else {
        for (std::size_t i = 1; i < candidates.size(); ++i) {
            if (candidates[i].squared_distance < candidates[selected].squared_distance)
                selected = i;
        }
    }

    const Candidate& chosen = candidates[selected];
    state.initialized = true;
    state.progress = chosen.projection.s;
    return chosen.projection;
}

}  // namespace

ProgressProjection ProgressProjector::project(const ReferencePath& reference,
                                              double x, double y) const {
    return project(reference, x, y, 0.0);
}

ProgressProjection ProgressProjector::project(const ReferencePath& reference,
                                              double x, double y, double min_s) const {
    ProgressProjectionState state;
    return project(reference, x, y, state, min_s);
}

ProgressProjection ProgressProjector::project(const ReferencePath& reference,
                                              double x, double y,
                                              ProgressProjectionState& state,
                                              double min_s) const {
    return project(reference.points(), x, y, state, min_s, true);
}

ProgressProjection ProgressProjector::project(const std::vector<TrajectoryPoint>& points,
                                              double x, double y,
                                              ProgressProjectionState& state,
                                              double min_s,
                                              bool derive_yaw_from_geometry) const {
    return projectPoints(points, x, y, state, min_s, config_, derive_yaw_from_geometry);
}

}  // namespace spmpc_local_planner
