#include "spmpc_local_planner/reference/reference_spline.h"

#include <Eigen/Dense>
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

struct Direction {
    double x = 0.0;
    double y = 0.0;
    bool valid = false;
};

Direction segmentDirection(const TrajectoryPoint& a, const TrajectoryPoint& b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    if (std::hypot(dx, dy) <= 1e-9) return {};
    return {dx, dy, true};
}

Direction endpointDirection(const ReferencePath& path, bool from_end) {
    const auto& points = path.points();
    if (from_end) {
        for (size_t i = points.size(); i-- > 1;) {
            const Direction d = segmentDirection(points[i - 1], points[i]);
            if (d.valid) return d;
        }
    } else {
        for (size_t i = 1; i < points.size(); ++i) {
            const Direction d = segmentDirection(points[i - 1], points[i]);
            if (d.valid) return d;
        }
    }
    return {};
}

Direction directionAt(const ReferencePath& path, double s) {
    const auto& points = path.points();
    constexpr double kEps = 1e-9;
    for (size_t i = 1; i < points.size(); ++i) {
        if (points[i].s + kEps < s || points[i - 1].s - kEps > s) continue;
        const Direction d = segmentDirection(points[i - 1], points[i]);
        if (d.valid) return d;
    }
    return endpointDirection(path, s >= path.length() - kEps);
}

double localSegmentLength(const ReferencePath& path, bool from_end) {
    const auto& points = path.points();
    if (from_end) {
        for (size_t i = points.size(); i-- > 1;) {
            const double ds = points[i].s - points[i - 1].s;
            if (ds > 1e-9 && segmentDirection(points[i - 1], points[i]).valid) return ds;
        }
    } else {
        for (size_t i = 1; i < points.size(); ++i) {
            const double ds = points[i].s - points[i - 1].s;
            if (ds > 1e-9 && segmentDirection(points[i - 1], points[i]).valid) return ds;
        }
    }
    return 0.0;
}

void chooseFitWindow(const ReferencePath& path, double s0, double s_end,
                     double& left, double& right) {
    const double len = path.length();
    left = std::min(std::max(s0, 0.0), len);
    right = std::min(std::max(s_end, 0.0), len);
    if (right - left >= std::min(1e-3, len)) return;
    if (right < left) right = left;

    const double fallback = std::max(1e-3, 0.25 * len);
    if (left <= 1e-9) {
        right = std::min(len, left + std::min(fallback, localSegmentLength(path, false)));
    } else if (right >= len - 1e-9) {
        left = std::max(0.0, right - std::min(fallback, localSegmentLength(path, true)));
    } else {
        const double half = 0.5 * fallback;
        left = std::max(0.0, left - half);
        right = std::min(len, right + half);
    }
}

Eigen::Vector4d absoluteCoefficients(const Eigen::Vector4d& normalized,
                                     double center, double scale) {
    // q=(s-center)/scale; expand c0+c1*q+c2*q²+c3*q³ in absolute s.
    const double inv = 1.0 / scale;
    const double inv2 = inv * inv;
    const double inv3 = inv2 * inv;
    Eigen::Vector4d out;
    out(3) = normalized(3) * inv3;
    out(2) = normalized(2) * inv2 - 3.0 * center * normalized(3) * inv3;
    out(1) = normalized(1) * inv - 2.0 * center * normalized(2) * inv2 +
             3.0 * center * center * normalized(3) * inv3;
    out(0) = normalized(0) - center * normalized(1) * inv +
             center * center * normalized(2) * inv2 -
             center * center * center * normalized(3) * inv3;
    return out;
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

    // At an endpoint use the one-sided geometric segment. In particular,
    // never interpret the absent side as a zero vector: atan2(0, 0) would
    // inject a pi/2 heading jump into the curvature estimate.
    const bool at_start = sc <= 1e-9;
    const bool at_end = sc >= len - 1e-9;
    if (at_start || at_end) {
        const Direction d = endpointDirection(path_, at_end);
        if (d.valid) out.psi = std::atan2(d.y, d.x);
        else if (std::isfinite(p.yaw)) out.psi = normalizeAngle(p.yaw);
        out.kappa = 0.0;
        return out;
    }

    // psi / kappa：弧长有限差分估计。
    const double h = std::min(0.1, std::max(1e-3, 0.25 * len));
    const double s_lo = std::max(0.0, sc - h);
    const double s_hi = std::min(len, sc + h);
    const TrajectoryPoint p_lo = path_.sample(s_lo);
    const TrajectoryPoint p_hi = path_.sample(s_hi);

    const Direction chord = segmentDirection(p_lo, p_hi);
    const Direction tangent = directionAt(path_, sc);
    if (chord.valid) out.psi = std::atan2(chord.y, chord.x);
    else if (tangent.valid) out.psi = std::atan2(tangent.y, tangent.x);

    const Direction back = segmentDirection(p_lo, p);
    const Direction fwd = segmentDirection(p, p_hi);
    const double psi_back = back.valid ? std::atan2(back.y, back.x) : out.psi;
    const double psi_fwd = fwd.valid ? std::atan2(fwd.y, fwd.x) : out.psi;
    const double dpsi = normalizeAngle(psi_fwd - psi_back);
    // The two chord directions live at the half-interval midpoints.
    out.kappa = dpsi / std::max(1e-6, 0.5 * (s_hi - s_lo));

    return out;
}

void fitReferencePolynomials(const ReferenceSpline& spline, double s0, double s_end,
                             Eigen::Vector4d& cx, Eigen::Vector4d& cy) {
    cx.setZero();
    cy.setZero();
    if (spline.empty() || !std::isfinite(s0) || !std::isfinite(s_end)) return;

    double left = 0.0;
    double right = 0.0;
    chooseFitWindow(spline.path_, s0, s_end, left, right);
    const double span = right - left;
    if (span <= 1e-9) {
        const auto point = spline.sample(left);
        cx(0) = point.x;
        cy(0) = point.y;
        return;
    }

    constexpr int kSamples = 12;
    Eigen::MatrixXd A(kSamples, 4);
    Eigen::VectorXd bx(kSamples), by(kSamples);
    const double center = 0.5 * (left + right);
    const double scale = 0.5 * span;
    const auto origin = spline.sample(center);
    for (int i = 0; i < kSamples; ++i) {
        const double s = left + span * static_cast<double>(i) / (kSamples - 1);
        const double q = (s - center) / scale;
        const auto sample = spline.sample(s);
        A(i, 0) = 1.0;
        A(i, 1) = q;
        A(i, 2) = q * q;
        A(i, 3) = q * q * q;
        bx(i) = sample.x - origin.x;
        by(i) = sample.y - origin.y;
    }
    Eigen::Vector4d nx = A.colPivHouseholderQr().solve(bx);
    Eigen::Vector4d ny = A.colPivHouseholderQr().solve(by);
    nx(0) += origin.x;
    ny(0) += origin.y;
    cx = absoluteCoefficients(nx, center, scale);
    cy = absoluteCoefficients(ny, center, scale);
}

}  // namespace spmpc_local_planner
