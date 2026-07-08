#include "spmpc_local_planner/core/slosh_risk_governor.h"
#include <algorithm>
#include <chrono>
#include <cmath>

namespace spmpc_local_planner {
namespace {

using Clock = std::chrono::steady_clock;

double clampValue(double value, double lo, double hi) {
    return std::max(lo, std::min(hi, value));
}

bool finitePositive(double value) {
    return std::isfinite(value) && value > 0.0;
}

bool riskAdmissible(double risk, double threshold) {
    return std::isfinite(risk) && risk <= threshold + 1e-12;
}

double elapsedMs(const Clock::time_point& start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

}  // namespace

bool SloshRiskGovernor::configure(const SloshModelParams& slosh_params,
                                  const SloshRiskGovernorParams& params) {
    params_ = params;
    params_.horizon_steps = std::max(1, params_.horizon_steps);
    params_.beta_grid_count = std::max(1, params_.beta_grid_count);
    params_.beta_min = clampValue(params_.beta_min, 0.0, 1.0);
    params_.risk_threshold = std::max(0.0, params_.risk_threshold);
    params_.release_threshold = std::max(0.0, params_.release_threshold);
    params_.min_v_ref = std::max(0.0, params_.min_v_ref);
    params_.accel_limit = std::max(0.0, params_.accel_limit);
    params_.beta_rate_up_per_sec = std::max(0.0, params_.beta_rate_up_per_sec);
    params_.beta_rate_down_per_sec = std::max(0.0, params_.beta_rate_down_per_sec);

    SloshModelParams governed_slosh_params = slosh_params;
    governed_slosh_params.use_parabola_term = params_.include_parabola_height;
    slosh_params_ = governed_slosh_params;
    configured_ = slosh_dyn_.configure(governed_slosh_params);
    reset();
    return configured_;
}

void SloshRiskGovernor::reset() {
    have_beta_filtered_ = false;
    beta_filtered_ = 1.0;
}

SloshRiskGovernorOutput SloshRiskGovernor::passThrough(
    const SloshRiskGovernorInput& input,
    const std::string& status) const {
    SloshRiskGovernorOutput out;
    out.enabled = params_.enable;
    out.active = false;
    out.nominal_v_ref = std::isfinite(input.nominal_v_ref) ? input.nominal_v_ref : 0.0;
    out.governed_v_ref = out.nominal_v_ref;
    out.beta_raw = 1.0;
    out.beta_filtered = have_beta_filtered_ ? beta_filtered_ : 1.0;
    if (configured_ && finitePositive(params_.height_limit_m)) {
        out.h_now_m = height(input.slosh, input.robot_omega);
        out.h_peak_m = out.h_now_m;
        out.risk_now = out.h_now_m / params_.height_limit_m;
        out.risk_peak = out.risk_now;
        out.predicted_risk_admissible = riskAdmissible(out.risk_peak, params_.risk_threshold);
    }
    out.risk_margin = params_.risk_threshold - out.risk_peak;
    out.selected_candidate_index = 0;
    out.status = status;
    return out;
}

double SloshRiskGovernor::height(const SloshState& state, double omega_z) const {
    return slosh_dyn_.height(state, omega_z);
}

SloshRiskGovernor::RolloutResult SloshRiskGovernor::rollout(
    const SloshRiskGovernorInput& input,
    double beta) const {
    RolloutResult out;
    SloshState state = input.slosh;
    double v_sim = clampValue(std::isfinite(input.robot_v) ? input.robot_v : 0.0,
                              0.0,
                              std::max(0.0, input.nominal_v_ref));
    const double dt = std::max(1e-4, input.dt);
    const double target_v = clampValue(beta * input.nominal_v_ref, 0.0, input.nominal_v_ref);

    out.h_peak_m = height(state, input.robot_omega);
    out.risk_peak = finitePositive(params_.height_limit_m) ? out.h_peak_m / params_.height_limit_m : 0.0;

    for (int k = 0; k < params_.horizon_steps; ++k) {
        double omega_sim = input.robot_omega;
        if (std::isfinite(params_.omega_decay_tau) && params_.omega_decay_tau > 0.0) {
            omega_sim *= std::exp(-static_cast<double>(k) * dt / params_.omega_decay_tau);
        }

        const double accel_bound = std::max(0.0, params_.accel_limit);
        const double ax_unclamped = (target_v - v_sim) / dt;
        const double ax = clampValue(ax_unclamped, -accel_bound, accel_bound);
        const double ay = v_sim * omega_sim;
        state = slosh_dyn_.step(state, ax, ay, omega_sim);

        const double h = height(state, omega_sim);
        out.h_peak_m = std::max(out.h_peak_m, h);
        if (finitePositive(params_.height_limit_m)) {
            out.risk_peak = std::max(out.risk_peak, h / params_.height_limit_m);
        }

        v_sim = clampValue(v_sim + ax * dt, 0.0, input.nominal_v_ref);
    }
    return out;
}

SloshRiskGovernorOutput SloshRiskGovernor::update(const SloshRiskGovernorInput& input) {
    const Clock::time_point start = Clock::now();
    auto finish = [&](SloshRiskGovernorOutput out) {
        out.computation_time_ms = elapsedMs(start);
        return out;
    };

    if (!params_.enable) {
        return finish(passThrough(input, "DISABLED"));
    }
    if (params_.require_slosh_variant && !input.slosh_variant_enabled) {
        return finish(passThrough(input, "NOT_SLOSH_VARIANT"));
    }
    if (finitePositive(input.dt) && configured_ &&
        std::abs(slosh_dyn_.params().dt - input.dt) > 1e-6) {
        slosh_params_.dt = input.dt;
        configured_ = slosh_dyn_.configure(slosh_params_);
    }
    if (!configured_ || !slosh_dyn_.configured() || !finitePositive(params_.height_limit_m) ||
        !finitePositive(input.nominal_v_ref) || !finitePositive(input.dt)) {
        return finish(passThrough(input, "INVALID_CONFIG"));
    }

    SloshRiskGovernorOutput out;
    out.enabled = true;
    out.nominal_v_ref = input.nominal_v_ref;
    out.h_now_m = height(input.slosh, input.robot_omega);
    out.risk_now = out.h_now_m / params_.height_limit_m;

    const int grid = std::max(1, params_.beta_grid_count);
    double selected_beta = params_.beta_min;
    int selected_index = grid - 1;
    bool found_feasible = false;

    for (int i = 0; i < grid; ++i) {
        const double ratio = grid == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(grid - 1);
        const double beta = 1.0 - ratio * (1.0 - params_.beta_min);
        const RolloutResult candidate = rollout(input, beta);
        if (riskAdmissible(candidate.risk_peak, params_.risk_threshold)) {
            selected_beta = beta;
            selected_index = i;
            found_feasible = true;
            break;
        }
    }
    if (!found_feasible) {
        selected_beta = params_.beta_min;
        selected_index = grid - 1;
    }

    out.beta_raw = selected_beta;
    out.feasible_found = found_feasible;
    out.saturated = !found_feasible;
    out.selected_candidate_index = selected_index;

    const double previous_beta = have_beta_filtered_ ? beta_filtered_ : 1.0;
    double filtered_beta = selected_beta;
    const double dt = std::max(1e-4, input.dt);
    if (selected_beta > previous_beta) {
        filtered_beta = std::min(selected_beta, previous_beta + params_.beta_rate_up_per_sec * dt);
    } else if (selected_beta < previous_beta) {
        filtered_beta = std::max(selected_beta, previous_beta - params_.beta_rate_down_per_sec * dt);
    }
    filtered_beta = clampValue(filtered_beta, params_.beta_min, 1.0);
    beta_filtered_ = filtered_beta;
    have_beta_filtered_ = true;
    out.beta_filtered = filtered_beta;

    const double min_v_ref = std::min(std::max(0.0, params_.min_v_ref), input.nominal_v_ref);
    out.governed_v_ref = clampValue(filtered_beta * input.nominal_v_ref, min_v_ref, input.nominal_v_ref);

    const RolloutResult filtered_rollout = rollout(input, filtered_beta);
    out.risk_peak = filtered_rollout.risk_peak;
    out.h_peak_m = filtered_rollout.h_peak_m;
    out.risk_margin = params_.risk_threshold - out.risk_peak;
    out.predicted_risk_admissible = riskAdmissible(out.risk_peak, params_.risk_threshold);
    out.active = filtered_beta < 0.999 || out.risk_peak > params_.release_threshold;

    if (out.saturated) {
        out.status = "SATURATED";
    } else if (!out.predicted_risk_admissible) {
        out.status = "TRANSIENT_RATE_LIMITED";
    } else if (out.active) {
        out.status = "ACTIVE";
    } else {
        out.status = "PASS_THROUGH";
    }
    return finish(out);
}

}  // namespace spmpc_local_planner
