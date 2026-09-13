#pragma once

#include <array>
#include <cmath>

namespace spmpc_local_planner {

struct SloshState {
    // Modal displacement and relative derivative in rotating container axes.
    // Units: m, m/s. Surface height is c_h * hypot(eta_x, eta_y).
    double eta_x = 0.0;
    double eta_x_dot = 0.0;
    double eta_y = 0.0;
    double eta_y_dot = 0.0;
};

struct ContainerExcitation {
    // Acceleration already at the container centre, in container axes.
    // omega/alpha describe actual angular motion, never command derivatives.
    double ax = 0.0;
    double ay = 0.0;
    double omega = 0.0;
    double alpha = 0.0;
};

struct SloshCoefficients {
    double damping = 0.0;       // 2*zeta*omega_n
    double frequency_sq = 0.0;  // omega_n^2
    double kappa_x = 1.0;
    double kappa_y = 1.0;

    std::array<double, 4> values() const {
        return {{damping, frequency_sq, kappa_x, kappa_y}};
    }
};

inline bool finiteSloshState(const SloshState& state) {
    return std::isfinite(state.eta_x) && std::isfinite(state.eta_x_dot) &&
           std::isfinite(state.eta_y) && std::isfinite(state.eta_y_dot);
}

}  // namespace spmpc_local_planner
