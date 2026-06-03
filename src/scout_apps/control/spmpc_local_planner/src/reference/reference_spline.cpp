#include "spmpc_local_planner/reference/reference_spline.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {
namespace {

double normalizeAngle(double a) {
    while (a > M_PI) {
        a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
        a += 2.0 * M_PI;
    }
    return a;
}

}  // namespace

void ReferenceSpline::build(const ReferencePath& path) {
    path_ = path;
}

ReferenceSample ReferenceSpline::sample(double s) const {
    ReferenceSample out;
    if (path_.empty()) {
        return out;
    }

    const double len = path_.length();
    const double sc = std::min(std::max(s, 0.0), len);
    const TrajectoryPoint p = path_.sample(sc);
    out.x = p.x;
    out.y = p.y;
    out.s = sc;

    // psi / kappa：弧长有限差分估计（骨架版本）。
    const double h = std::min(0.1, std::max(1e-3, 0.25 * len));
    const double s_lo = std::max(0.0, sc - h);
    const double s_hi = std::min(len, sc + h);
    const TrajectoryPoint p_lo = path_.sample(s_lo);
    const TrajectoryPoint p_hi = path_.sample(s_hi);

    out.psi = std::atan2(p_hi.y - p_lo.y, p_hi.x - p_lo.x);

    const double psi_back = std::atan2(p.y - p_lo.y, p.x - p_lo.x);
    const double psi_fwd = std::atan2(p_hi.y - p.y, p_hi.x - p.x);
    const double dpsi = normalizeAngle(psi_fwd - psi_back);
    out.kappa = dpsi / std::max(1e-6, h);

    return out;
}

}  // namespace spmpc_local_planner
