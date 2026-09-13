#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "integration_policy.h"
#include <cmath>
#include <slosh_models/liquid_slosh_model.h>
#include "generated/slosh_kernel_generated.h"
#include "generated/slosh_kernel_contract.h"
#include <array>

namespace spmpc_local_planner {

bool SloshDynamics::configure(const SloshModelParams& params) {
    configured_ = false;
    if (!std::isfinite(params.dt) || params.dt <= 1e-4 ||
        !std::isfinite(params.container_radius) || !std::isfinite(params.liquid_height) ||
        !std::isfinite(params.liquid_density) || !std::isfinite(params.damping_ratio) ||
        params.damping_ratio < 0.0) {
        return false;
    }
    params_ = params;

    slosh_models::LiquidSloshModel model;
    slosh_models::LiquidSloshModel::Params p;
    p.R = params.container_radius;
    p.h = params.liquid_height;
    p.rho = params.liquid_density;
    p.dt = params.dt;
    p.mode_index = params.mode_index;
    p.zeta = params.damping_ratio;
    p.use_linear_model = params.use_linear_model;
    p.use_parabola_term = params.use_parabola_term;

    if (!model.configure(p)) {
        configured_ = false;
        return false;
    }

    omega_n_ = model.getModalParams().omega_n;
    height_coeff_ = model.getModalParams().height_coeff;
    configured_ = true;
    return true;
}

static_assert(SPMPC_LIQUID_KERNEL_VERSION == SloshDynamics::modelVersion(),
              "Regenerate the shared rotating-container kernel");

bool SloshDynamics::stepWithDt(
    const SloshState& state, const ContainerExcitation& input,
    double dt_sec, SloshState& next_state) const {
    next_state = state;
    if (!configured_ || !integration_policy::validInterval(dt_sec) || !finiteSloshState(state) ||
        !std::isfinite(input.ax) || !std::isfinite(input.ay) ||
        !std::isfinite(input.omega) || !std::isfinite(input.alpha)) {
        return false;
    }
    double x[4] = {state.eta_x, state.eta_x_dot, state.eta_y, state.eta_y_dot};
    const double excitation[4] = {input.ax, input.ay, input.omega, input.alpha};
    const auto physical = coefficients().values();
    const int count = integration_policy::substeps(dt_sec);
    const double dt = dt_sec / count;
    double next[4];
    const double* arg[] = {x, excitation, physical.data(), &dt};
    double* res[] = {next};
    std::array<casadi_int, spmpc_slosh_rk4_SZ_IW + 1> iw{};
    std::array<double, spmpc_slosh_rk4_SZ_W + 1> work{};
    for (int k = 0; k < count; ++k) {
        if (spmpc_slosh_rk4(arg, res, iw.data(), work.data(), 0) != 0) return false;
        for (int i = 0; i < 4; ++i) {
            if (!std::isfinite(next[i])) return false;
            x[i] = next[i];
        }
    }
    next_state = {x[0], x[1], x[2], x[3]};
    return true;
}

double SloshDynamics::height(const SloshState& state, double omega_z) const {
    if (!configured_) {
        return 0.0;
    }
    const double modal = height_coeff_ * etaNorm(state);
    double parabola = 0.0;
    if (params_.use_parabola_term) {
        const double r = params_.container_radius;
        parabola = r * r * omega_z * omega_z / (4.0 * 9.81);
    }
    return modal + parabola;
}

double SloshDynamics::etaNorm(const SloshState& state) const {
    return std::hypot(state.eta_x, state.eta_y);
}

double SloshDynamics::etaDotNorm(const SloshState& state) const {
    return std::hypot(state.eta_x_dot, state.eta_y_dot);
}

}  // namespace spmpc_local_planner
