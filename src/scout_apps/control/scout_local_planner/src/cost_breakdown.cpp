/**
 * @file cost_breakdown.cpp
 * @brief MPC cost contribution 统计实现
 */

#include "scout_local_planner/cost_breakdown.h"

#include <algorithm>

namespace scout_local_planner {

DiagnosticsCostBreakdown computeMpcCostBreakdown(
    const MPCSolution& solution,
    const std::vector<ReferencePoint>& refs,
    const MPCParams& params,
    const ControlVector& u_prev) {

    DiagnosticsCostBreakdown out;
    if (solution.x_predicted.empty()) {
        return out;
    }

    const int N = static_cast<int>(solution.u_optimal.size());
    const int n_states = static_cast<int>(solution.x_predicted.size());
    const int ramp_steps = std::max(1, params.terminal_ramp_steps);
    const int ramp_start = N - ramp_steps;

    auto terminal_factor = [&](int k, double configured_factor) {
        if (configured_factor <= 0.0 || k < ramp_start || k > N) {
            return 1.0;
        }
        const double alpha = static_cast<double>(k - ramp_start + 1)
                           / static_cast<double>(ramp_steps + 1);
        return 1.0 + alpha * (configured_factor - 1.0);
    };

    for (int k = 0; k < n_states; ++k) {
        const StateVector& x = solution.x_predicted[static_cast<size_t>(k)];
        const double e_l = x(StateIndex::E_L);
        const double e_c = x(StateIndex::E_C);
        const double e_theta = x(StateIndex::E_THETA);
        const double v = x(StateIndex::V);
        const double eta_x = x(StateIndex::ETA_X);
        const double eta_x_dot = x(StateIndex::ETA_X_DOT);
        const double eta_y = x(StateIndex::ETA_Y);
        const double eta_y_dot = x(StateIndex::ETA_Y_DOT);

        out.J_lag += (params.use_contour_lag ? params.Q_lag : params.Q_el) * e_l * e_l;
        out.J_contour += terminal_factor(k, params.terminal_factor_ec) *
                         (params.use_contour_lag ? params.Q_contour : params.Q_ec) * e_c * e_c;
        out.J_etheta += terminal_factor(k, params.terminal_factor_etheta) *
                        params.Q_etheta * e_theta * e_theta;

        const double v_factor = terminal_factor(k, params.terminal_factor_v);
        if (k < static_cast<int>(refs.size())) {
            const double dv = v - refs[static_cast<size_t>(k)].v_ref;
            out.J_v += v_factor * params.Q_v * dv * dv;
        } else {
            // buildQPCost() has no v_ref linear term at x_N; match the QP term.
            out.J_v += v_factor * params.Q_v * v * v;
        }

        if (params.Q_slosh_eta > 0.0) {
            const double preview_factor = (k > 0) ? std::max(0.0, params.slosh_preview_factor) : 0.0;
            out.J_slosh_eta +=
                (terminal_factor(k, params.terminal_factor_slosh_eta) + preview_factor) *
                params.Q_slosh_eta * (eta_x * eta_x + eta_y * eta_y);
        }
        if (params.Q_slosh_eta_dot > 0.0) {
            const double preview_factor = (k > 0) ? std::max(0.0, params.slosh_preview_factor) : 0.0;
            out.J_slosh_eta_dot +=
                (terminal_factor(k, params.terminal_factor_slosh_eta_dot) + preview_factor) *
                params.Q_slosh_eta_dot *
                (eta_x_dot * eta_x_dot + eta_y_dot * eta_y_dot);
        }
    }

    for (int k = 0; k < N; ++k) {
        const ControlVector& u = solution.u_optimal[static_cast<size_t>(k)];
        const double a = u(ControlIndex::A);
        const double omega = u(ControlIndex::OMEGA);

        out.J_control += params.R_a * a * a + params.R_omega * omega * omega;

        if (params.enable_omega_ff && k < static_cast<int>(refs.size())) {
            const double omega_ref = refs[static_cast<size_t>(k)].v_ref *
                                     refs[static_cast<size_t>(k)].kappa;
            const double domega_ref = omega - omega_ref;
            out.J_omega_ff += params.Q_omega_ff * domega_ref * domega_ref;
        }

        const ControlVector& up = (k == 0) ? u_prev : solution.u_optimal[static_cast<size_t>(k - 1)];
        const double da = a - up(ControlIndex::A);
        const double domega = omega - up(ControlIndex::OMEGA);
        out.J_smooth += params.R_da * da * da + params.R_domega * domega * domega;
    }

    out.J_total =
        out.J_lag + out.J_contour + out.J_etheta + out.J_v + out.J_omega_ff +
        out.J_control + out.J_smooth + out.J_slosh_eta + out.J_slosh_eta_dot;
    return out;
}

}  // namespace scout_local_planner
