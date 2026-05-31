/**
 * @file rpp_speed_regulator.h
 * @brief RPP-inspired reference speed regulator for fixed-path baselines
 */

#pragma once

namespace scout_local_planner {

struct RppSpeedRegulatorParams {
    bool enable = false;
    double regulated_min_radius = 0.5;
    double approach_dist = 0.7;
    double min_approach_v = 0.05;
    bool replace_base_curvature_cap = true;
};

struct RppSpeedRegulatorOutput {
    double v_out = 0.0;
    double v_curvature_cap = 0.0;
    double v_approach_cap = 0.0;
    bool curvature_active = false;
    bool approach_active = false;
};

class RppSpeedRegulator {
public:
    RppSpeedRegulatorOutput regulate(double v_in,
                                     double kappa,
                                     double dist_to_goal,
                                     const RppSpeedRegulatorParams& params) const;
};

}  // namespace scout_local_planner
