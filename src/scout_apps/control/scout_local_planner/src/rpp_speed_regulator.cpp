/**
 * @file rpp_speed_regulator.cpp
 * @brief RPP-inspired reference speed regulator implementation
 */

#include "scout_local_planner/rpp_speed_regulator.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace scout_local_planner {

RppSpeedRegulatorOutput RppSpeedRegulator::regulate(
        double v_in,
        double kappa,
        double dist_to_goal,
        const RppSpeedRegulatorParams& params) const {
    RppSpeedRegulatorOutput out;
    const double v_nonneg = std::max(0.0, std::isfinite(v_in) ? v_in : 0.0);
    out.v_curvature_cap = v_nonneg;
    out.v_approach_cap = v_nonneg;
    out.v_out = v_nonneg;

    if (!params.enable) {
        return out;
    }

    // Based on Nav2 Regulated Pure Pursuit regulation_functions.hpp
    // navigation2 commit c7e9b6f: curvatureConstraint(raw_linear_vel, curvature, min_radius).
    // This is a velocity-reference regulator only; it is not a full RPP controller port.
    if (params.regulated_min_radius > 1e-6 && std::abs(kappa) > 1e-6) {
        const double radius = std::abs(1.0 / kappa);
        if (radius < params.regulated_min_radius) {
            const double scale =
                1.0 - (std::abs(radius - params.regulated_min_radius) /
                       params.regulated_min_radius);
            out.v_curvature_cap = v_nonneg * std::max(0.0, scale);
            out.curvature_active = out.v_curvature_cap < v_nonneg - 1e-6;
        }
    }

    // Horizon version of Nav2 approachVelocityConstraint: use the remaining
    // path distance for each reference step so v_ref tapers along the horizon.
    if (params.approach_dist > 1e-6 && std::isfinite(dist_to_goal) &&
        dist_to_goal < params.approach_dist) {
        const double scale = std::max(0.0, dist_to_goal / params.approach_dist);
        const double approach_vel = out.v_curvature_cap * scale;
        out.v_approach_cap = std::max(0.0, params.min_approach_v);
        if (approach_vel > out.v_approach_cap) {
            out.v_approach_cap = approach_vel;
        }
        out.v_approach_cap = std::min(out.v_curvature_cap, out.v_approach_cap);
        out.approach_active = out.v_approach_cap < out.v_curvature_cap - 1e-6;
    } else {
        out.v_approach_cap = out.v_curvature_cap;
    }

    out.v_out = std::min(out.v_curvature_cap, out.v_approach_cap);
    return out;
}

}  // namespace scout_local_planner
