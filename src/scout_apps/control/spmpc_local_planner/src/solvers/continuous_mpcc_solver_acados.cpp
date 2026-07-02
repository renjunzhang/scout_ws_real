#include "spmpc_local_planner/solvers/continuous_mpcc_solver_acados.h"

#ifdef SPMPC_WITH_ACADOS

#include "spmpc_local_planner/reference/progress_projector.h"
#include "spmpc_local_planner/reference/reference_spline.h"
#include "spmpc_local_planner/warm_start/warm_start_factory.h"

#include "acados_solver_spmpc_b0.h"
#ifdef SPMPC_WITH_ACADOS_SLOSH
#include "acados_solver_spmpc_slosh.h"
#endif
#include "acados_c/ocp_nlp_interface.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace spmpc_local_planner {
namespace {

// 参数索引：必须与 scripts/acados/spmpc_acados_model.py 的 PARAM_NAMES / PARAM_NAMES_SLOSH 一致。
enum Param {
    RX0 = 0, RX1, RX2, RX3,
    RY0, RY1, RY2, RY3,
    W_CONTOUR, W_LAG, W_PROGRESS,
    W_A, W_OMEGA, W_V, W_VS, W_ALPHA,
    W_DU_A, W_DU_VS,
    A_PREV, VS_PREV,
    E_C_REF, E_L_REF,
    V_REF,
    // 以下仅 slosh 模型（接在 B0 的 23 个之后）
    TWO_ZETA_OMEGA_N, OMEGA_N_SQ, KAPPA_X, KAPPA_Y,
    ETA_REF, ETA_DOT_REF, W_SLOSH_ETA, W_SLOSH_ETA_DOT,
    ETA_MAX_SQ,
    PARAM_MAX,
};

// 参数布局契约：与 scripts/acados/spmpc_acados_model.py（→生成器 NP 宏）绑死，漂移即编译失败。
static_assert(V_REF + 1 == SPMPC_B0_NP, "B0 参数布局与生成的 spmpc_b0 求解器不一致");
#ifdef SPMPC_WITH_ACADOS_SLOSH
static_assert(ETA_MAX_SQ + 1 == SPMPC_SLOSH_NP, "slosh 参数布局与生成的 spmpc_slosh 求解器不一致");
static_assert(SPMPC_SLOSH_NH > 0, "spmpc_slosh 求解器缺少 slosh hard constraint，请重新生成 acados artifacts");
#endif

constexpr double kDisabledEtaMaxSq = 1e12;

// 统一封装两个生成求解器（b0 6维 / slosh 10维），把前缀相关调用收敛到一处。
struct GenSolver {
    enum Kind { B0, SLOSH } kind = B0;
    void* capsule = nullptr;
    int nx = 0, nu = 0, np = 0, n_horizon = 0;

    bool create(Kind k) {
        kind = k;
        if (k == B0) {
            auto* c = spmpc_b0_acados_create_capsule();
            if (c == nullptr || spmpc_b0_acados_create(c) != 0) {
                if (c) spmpc_b0_acados_free_capsule(c);
                return false;
            }
            capsule = c; nx = SPMPC_B0_NX; nu = SPMPC_B0_NU; np = SPMPC_B0_NP; n_horizon = SPMPC_B0_N;
        } else {
#ifdef SPMPC_WITH_ACADOS_SLOSH
            auto* c = spmpc_slosh_acados_create_capsule();
            if (c == nullptr || spmpc_slosh_acados_create(c) != 0) {
                if (c) spmpc_slosh_acados_free_capsule(c);
                return false;
            }
            capsule = c; nx = SPMPC_SLOSH_NX; nu = SPMPC_SLOSH_NU; np = SPMPC_SLOSH_NP; n_horizon = SPMPC_SLOSH_N;
#else
            return false;
#endif
        }
        return true;
    }
    void destroy() {
        if (capsule == nullptr) return;
        if (kind == B0) {
            spmpc_b0_acados_free(static_cast<spmpc_b0_solver_capsule*>(capsule));
            spmpc_b0_acados_free_capsule(static_cast<spmpc_b0_solver_capsule*>(capsule));
        } else {
#ifdef SPMPC_WITH_ACADOS_SLOSH
            spmpc_slosh_acados_free(static_cast<spmpc_slosh_solver_capsule*>(capsule));
            spmpc_slosh_acados_free_capsule(static_cast<spmpc_slosh_solver_capsule*>(capsule));
#endif
        }
        capsule = nullptr;
    }
    void update_params(int stage, double* p) {
        if (kind == B0) {
            spmpc_b0_acados_update_params(static_cast<spmpc_b0_solver_capsule*>(capsule), stage, p, np);
        } else {
#ifdef SPMPC_WITH_ACADOS_SLOSH
            spmpc_slosh_acados_update_params(static_cast<spmpc_slosh_solver_capsule*>(capsule), stage, p, np);
#endif
        }
    }
    int solve() {
        if (kind == B0) {
            return spmpc_b0_acados_solve(static_cast<spmpc_b0_solver_capsule*>(capsule));
        }
#ifdef SPMPC_WITH_ACADOS_SLOSH
        return spmpc_slosh_acados_solve(static_cast<spmpc_slosh_solver_capsule*>(capsule));
#else
        return -1;
#endif
    }
    ocp_nlp_config* config() {
        if (kind == B0) return spmpc_b0_acados_get_nlp_config(static_cast<spmpc_b0_solver_capsule*>(capsule));
#ifdef SPMPC_WITH_ACADOS_SLOSH
        return spmpc_slosh_acados_get_nlp_config(static_cast<spmpc_slosh_solver_capsule*>(capsule));
#else
        return nullptr;
#endif
    }
    ocp_nlp_dims* dims() {
        if (kind == B0) return spmpc_b0_acados_get_nlp_dims(static_cast<spmpc_b0_solver_capsule*>(capsule));
#ifdef SPMPC_WITH_ACADOS_SLOSH
        return spmpc_slosh_acados_get_nlp_dims(static_cast<spmpc_slosh_solver_capsule*>(capsule));
#else
        return nullptr;
#endif
    }
    ocp_nlp_in* in() {
        if (kind == B0) return spmpc_b0_acados_get_nlp_in(static_cast<spmpc_b0_solver_capsule*>(capsule));
#ifdef SPMPC_WITH_ACADOS_SLOSH
        return spmpc_slosh_acados_get_nlp_in(static_cast<spmpc_slosh_solver_capsule*>(capsule));
#else
        return nullptr;
#endif
    }
    ocp_nlp_out* out() {
        if (kind == B0) return spmpc_b0_acados_get_nlp_out(static_cast<spmpc_b0_solver_capsule*>(capsule));
#ifdef SPMPC_WITH_ACADOS_SLOSH
        return spmpc_slosh_acados_get_nlp_out(static_cast<spmpc_slosh_solver_capsule*>(capsule));
#else
        return nullptr;
#endif
    }
    ocp_nlp_solver* solver() {
        if (kind == B0) return spmpc_b0_acados_get_nlp_solver(static_cast<spmpc_b0_solver_capsule*>(capsule));
#ifdef SPMPC_WITH_ACADOS_SLOSH
        return spmpc_slosh_acados_get_nlp_solver(static_cast<spmpc_slosh_solver_capsule*>(capsule));
#else
        return nullptr;
#endif
    }
};

double clampValue(double value, double lo, double hi) {
    return std::max(lo, std::min(hi, value));
}

double wrapAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

SolverBoundSummary makeRuntimeBounds(const SolverParams& params) {
    SolverBoundSummary bounds;
    bounds.a_min = -std::max(0.0, params.a_max);
    bounds.a_max = std::max(0.0, params.a_max);
    bounds.alpha_min = -std::max(0.0, params.alpha_max);
    bounds.alpha_max = std::max(0.0, params.alpha_max);
    bounds.v_s_min = 0.0;
    bounds.v_s_max = std::max(0.0, params.v_max);
    bounds.v_min = 0.0;
    bounds.v_max = std::max(0.0, params.v_max);
    bounds.omega_min = -std::max(0.0, params.omega_max);
    bounds.omega_max = std::max(0.0, params.omega_max);
    return bounds;
}

SolverBoundSummary makeGeneratedBounds() {
    SolverBoundSummary bounds;
    bounds.a_min = -0.6;
    bounds.a_max = 0.6;
    bounds.alpha_min = -1.2;
    bounds.alpha_max = 1.2;
    bounds.v_s_min = 0.0;
    bounds.v_s_max = 0.8;
    bounds.v_min = 0.0;
    bounds.v_max = 0.8;
    bounds.omega_min = -1.2;
    bounds.omega_max = 1.2;
    return bounds;
}

void applyRuntimeBounds(GenSolver& gen, const SolverBoundSummary& bounds, double* x0) {
    ocp_nlp_config* cfg = gen.config();
    ocp_nlp_dims* dims = gen.dims();
    ocp_nlp_in* nlp_in = gen.in();
    ocp_nlp_out* nlp_out = gen.out();

    ocp_nlp_constraints_model_set(cfg, dims, nlp_in, nlp_out, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(cfg, dims, nlp_in, nlp_out, 0, "ubx", x0);

    double lbu[3] = {bounds.a_min, bounds.alpha_min, bounds.v_s_min};
    double ubu[3] = {bounds.a_max, bounds.alpha_max, bounds.v_s_max};
    for (int stage = 0; stage < gen.n_horizon; ++stage) {
        ocp_nlp_constraints_model_set(cfg, dims, nlp_in, nlp_out, stage, "lbu", lbu);
        ocp_nlp_constraints_model_set(cfg, dims, nlp_in, nlp_out, stage, "ubu", ubu);
    }

    double lbx[2] = {bounds.v_min, bounds.omega_min};
    double ubx[2] = {bounds.v_max, bounds.omega_max};
    for (int stage = 1; stage <= gen.n_horizon; ++stage) {
        ocp_nlp_constraints_model_set(cfg, dims, nlp_in, nlp_out, stage, "lbx", lbx);
        ocp_nlp_constraints_model_set(cfg, dims, nlp_in, nlp_out, stage, "ubx", ubx);
    }
}

double polyEval(const Eigen::Vector4d& c, double s) {
    return c(0) + c(1) * s + c(2) * s * s + c(3) * s * s * s;
}
double polyDeriv(const Eigen::Vector4d& c, double s) {
    return c(1) + 2.0 * c(2) * s + 3.0 * c(3) * s * s;
}

void fitReferencePolynomials(const ReferenceSpline& spline, double s0, double s_end,
                             Eigen::Vector4d& cx, Eigen::Vector4d& cy) {
    const int m = 12;
    Eigen::MatrixXd A(m, 4);
    Eigen::VectorXd bx(m), by(m);
    const double span = std::max(1e-3, s_end - s0);
    for (int i = 0; i < m; ++i) {
        const double s = s0 + span * static_cast<double>(i) / static_cast<double>(m - 1);
        const ReferenceSample r = spline.sample(s);
        A(i, 0) = 1.0; A(i, 1) = s; A(i, 2) = s * s; A(i, 3) = s * s * s;
        bx(i) = r.x; by(i) = r.y;
    }
    cx = A.colPivHouseholderQr().solve(bx);
    cy = A.colPivHouseholderQr().solve(by);
}

WarmStartState makeWarmStartState(const double* x, bool slosh) {
    WarmStartState state;
    state.px = x[0]; state.py = x[1]; state.theta = x[2]; state.v = x[3]; state.s = x[4];
    state.omega = x[5];
    if (slosh) {
        state.eta_x = x[6]; state.eta_x_dot = x[7]; state.eta_y = x[8]; state.eta_y_dot = x[9];
    }
    return state;
}

WarmStartControl makeWarmStartControl(const double* u) {
    WarmStartControl control;
    control.a = u[0]; control.alpha = u[1]; control.v_s = u[2];
    return control;
}

void fillAcadosState(const WarmStartState& state, bool slosh, double* x) {
    x[0] = state.px; x[1] = state.py; x[2] = state.theta; x[3] = state.v; x[4] = state.s;
    x[5] = state.omega;
    x[6] = slosh ? state.eta_x : 0.0;
    x[7] = slosh ? state.eta_x_dot : 0.0;
    x[8] = slosh ? state.eta_y : 0.0;
    x[9] = slosh ? state.eta_y_dot : 0.0;
}

void fillAcadosControl(const WarmStartControl& control, double* u) {
    u[0] = control.a;
    u[1] = control.alpha;
    u[2] = control.v_s;
}

void setAcadosWarmStart(GenSolver& gen, const WarmStartOutput& warm_start, bool slosh) {
    if (!warm_start.valid || warm_start.states.size() < static_cast<size_t>(gen.n_horizon + 1) ||
        warm_start.controls.size() < static_cast<size_t>(gen.n_horizon)) {
        return;
    }
    ocp_nlp_config* cfg = gen.config();
    ocp_nlp_dims* dims = gen.dims();
    ocp_nlp_in* nlp_in = gen.in();
    ocp_nlp_out* nlp_out = gen.out();
    double x_guess[10];
    double u_guess[3];
    for (int k = 0; k <= gen.n_horizon; ++k) {
        fillAcadosState(warm_start.states[k], slosh, x_guess);
        ocp_nlp_out_set(cfg, dims, nlp_out, nlp_in, k, "x", x_guess);
        if (k < gen.n_horizon) {
            fillAcadosControl(warm_start.controls[k], u_guess);
            ocp_nlp_out_set(cfg, dims, nlp_out, nlp_in, k, "u", u_guess);
        }
    }
}

WarmStartInput makeWarmStartInput(const SolverInput& input,
                                  const ReferencePath& reference,
                                  const ReferenceSpline& spline,
                                  double s0,
                                  double len,
                                  int n,
                                  const SolverParams& params,
                                  const SloshDynamics& slosh_dyn,
                                  bool have_u_prev,
                                  const double* u_prev) {
    WarmStartInput warm_input;
    warm_input.robot = input.robot;
    warm_input.slosh = input.slosh;
    warm_input.reference = &reference;
    warm_input.spline = &spline;
    warm_input.horizon_steps = n;
    warm_input.dt = input.dt;
    warm_input.s0 = s0;
    warm_input.reference_length = len;
    warm_input.platform = params.platform;
    warm_input.slosh_params = params.slosh;
    warm_input.slosh_dynamics = &slosh_dyn;
    warm_input.bounds.v_max = params.v_max;
    warm_input.bounds.omega_max = params.omega_max;
    warm_input.bounds.a_max = params.a_max;
    warm_input.bounds.omega_rate_max = params.alpha_max;
    warm_input.bounds.v_s_max = params.v_max;
    warm_input.config = params.warm_start;
    warm_input.have_previous_control = have_u_prev;
    if (have_u_prev && u_prev != nullptr) {
        warm_input.previous_a = u_prev[0];
        // Legacy field; alpha-state u_prev[1] is alpha, while the flatness generator does not consume previous_omega.
        warm_input.previous_omega = input.robot.omega;
        warm_input.previous_v_s = u_prev[2];
    }
    return warm_input;
}

bool isWarmStartFinite(const WarmStartOutput& warm_start) {
    for (const auto& state : warm_start.states) {
        if (!std::isfinite(state.px) || !std::isfinite(state.py) || !std::isfinite(state.theta) ||
            !std::isfinite(state.v) || !std::isfinite(state.s) || !std::isfinite(state.omega) ||
            !std::isfinite(state.eta_x) || !std::isfinite(state.eta_x_dot) ||
            !std::isfinite(state.eta_y) || !std::isfinite(state.eta_y_dot)) {
            return false;
        }
    }
    for (const auto& control : warm_start.controls) {
        if (!std::isfinite(control.a) || !std::isfinite(control.alpha) || !std::isfinite(control.v_s)) {
            return false;
        }
    }
    return true;
}

void stampWarmStartMetrics(WarmStartOutput& warm_start,
                           const SolverParams& params,
                           const SloshDynamics& slosh_dyn,
                           bool slosh) {
    for (const auto& state : warm_start.states) {
        warm_start.diagnostics.max_v = std::max(warm_start.diagnostics.max_v, std::abs(state.v));
        warm_start.diagnostics.max_omega = std::max(warm_start.diagnostics.max_omega, std::abs(state.omega));
        warm_start.diagnostics.max_lateral_acc = std::max(
            warm_start.diagnostics.max_lateral_acc, std::abs(state.v * state.omega));
        if (slosh && slosh_dyn.configured()) {
            SloshState ss;
            ss.eta_x = state.eta_x; ss.eta_x_dot = state.eta_x_dot;
            ss.eta_y = state.eta_y; ss.eta_y_dot = state.eta_y_dot;
            warm_start.diagnostics.max_slosh_height_pred = std::max(
                warm_start.diagnostics.max_slosh_height_pred, slosh_dyn.height(ss));
        }
        if (state.v < -1e-9 || state.v > params.v_max + 1e-9 ||
            std::abs(state.omega) > params.omega_max + 1e-9) {
            ++warm_start.diagnostics.bound_violation_count;
        }
    }
    for (const auto& control : warm_start.controls) {
        warm_start.diagnostics.max_a = std::max(warm_start.diagnostics.max_a, std::abs(control.a));
        if (std::abs(control.a) > params.a_max + 1e-9 ||
            std::abs(control.alpha) > params.alpha_max + 1e-9 ||
            control.v_s < -1e-9 || control.v_s > params.v_max + 1e-9) {
            ++warm_start.diagnostics.bound_violation_count;
        }
    }
}

WarmStartOutput makeShiftedPreviousWarmStart(const WarmStartOutput& previous,
                                             const SolverInput& input,
                                             double s0,
                                             int n,
                                             bool slosh,
                                             const SolverParams& params,
                                             const SloshDynamics& slosh_dyn) {
    WarmStartOutput out;
    out.diagnostics.used_previous_solution = true;
    if (!previous.valid || previous.states.size() < static_cast<size_t>(n + 1) ||
        previous.controls.size() < static_cast<size_t>(n)) {
        out.fallback_reason = "NO_PREVIOUS_WARM_START";
        out.diagnostics.failure_reason = out.fallback_reason;
        return out;
    }
    if (previous.states.size() > 1 && std::abs(previous.states[1].s - s0) > std::max(0.5, 5.0 * params.v_max * input.dt)) {
        out.fallback_reason = "PREVIOUS_WARM_START_PROGRESS_JUMP";
        out.diagnostics.failure_reason = out.fallback_reason;
        return out;
    }

    out.states.resize(n + 1);
    out.controls.resize(n);
    for (int k = 0; k <= n; ++k) {
        out.states[k] = previous.states[std::min(k + 1, n)];
        out.states[k].v = clampValue(out.states[k].v, 0.0, params.v_max);
    }
    for (int k = 0; k < n; ++k) {
        out.controls[k] = previous.controls[std::min(k + 1, n - 1)];
        out.controls[k].a = clampValue(out.controls[k].a, -params.a_max, params.a_max);
        out.controls[k].alpha = clampValue(out.controls[k].alpha, -params.alpha_max, params.alpha_max);
        out.controls[k].v_s = clampValue(out.controls[k].v_s, 0.0, params.v_max);
    }

    out.states[0].px = input.robot.x;
    out.states[0].py = input.robot.y;
    out.states[0].theta = input.robot.yaw;
    out.states[0].v = clampValue(input.robot.v, 0.0, params.v_max);
    out.states[0].s = s0;
    out.states[0].omega = input.robot.omega;
    if (slosh) {
        out.states[0].eta_x = input.slosh.eta_x;
        out.states[0].eta_x_dot = input.slosh.eta_x_dot;
        out.states[0].eta_y = input.slosh.eta_y;
        out.states[0].eta_y_dot = input.slosh.eta_y_dot;
    }

    out.valid = isWarmStartFinite(out);
    out.diagnostics.warm_start_valid = out.valid;
    if (!out.valid) {
        out.fallback_reason = "PREVIOUS_WARM_START_NONFINITE";
        out.diagnostics.failure_reason = out.fallback_reason;
    }
    stampWarmStartMetrics(out, params, slosh_dyn, slosh);
    return out;
}

WarmStartOutput makeConservativeWarmStart(const WarmStartInput& warm_input,
                                          const SolverParams& params,
                                          const SloshDynamics& slosh_dyn,
                                          bool slosh) {
    WarmStartOutput out;
    out.diagnostics.used_fallback = true;
    if (warm_input.spline == nullptr || warm_input.spline->empty() || warm_input.horizon_steps <= 0) {
        out.fallback_reason = "CONSERVATIVE_FALLBACK_NO_REFERENCE";
        out.diagnostics.failure_reason = out.fallback_reason;
        return out;
    }
    const int n = warm_input.horizon_steps;
    out.states.resize(n + 1);
    out.controls.resize(n);
    const double v_seed = clampValue(0.25 * params.v_max, 0.0, params.v_max);
    const double dt = std::max(1e-3, warm_input.dt);
    for (int k = 0; k <= n; ++k) {
        const double s = clampValue(warm_input.s0 + v_seed * warm_input.dt * k, warm_input.s0, warm_input.reference_length);
        const ReferenceSample ref = warm_input.spline->sample(s);
        out.states[k].px = (k == 0) ? warm_input.robot.x : ref.x;
        out.states[k].py = (k == 0) ? warm_input.robot.y : ref.y;
        out.states[k].theta = (k == 0) ? warm_input.robot.yaw : ref.psi;
        out.states[k].v = (k == 0) ? clampValue(warm_input.robot.v, 0.0, params.v_max) : v_seed;
        out.states[k].s = (k == 0) ? warm_input.s0 : s;
        out.states[k].omega = (k == 0) ? warm_input.robot.omega
                                       : clampValue(ref.kappa * out.states[k].v, -params.omega_max, params.omega_max);
        if (k < n) {
            out.controls[k].a = clampValue((v_seed - out.states[k].v) / dt, -params.a_max, params.a_max);
            out.controls[k].alpha = 0.0;  // 保守初值：alpha 由优化器细化
        }
    }
    for (int k = 0; k < n; ++k) {
        const double ds = out.states[k + 1].s - out.states[k].s;
        out.controls[k].v_s = clampValue(ds / dt, 0.0, params.v_max);
    }
    if (slosh) {
        SloshState slosh_state = warm_input.slosh;
        for (int k = 0; k <= n; ++k) {
            out.states[k].eta_x = slosh_state.eta_x;
            out.states[k].eta_x_dot = slosh_state.eta_x_dot;
            out.states[k].eta_y = slosh_state.eta_y;
            out.states[k].eta_y_dot = slosh_state.eta_y_dot;
            if (k < n && slosh_dyn.configured()) {
                slosh_state = slosh_dyn.step(slosh_state, out.controls[k].a, out.states[k].v * out.states[k].omega, out.states[k].omega);
            }
        }
    }
    out.valid = isWarmStartFinite(out);
    out.diagnostics.warm_start_valid = out.valid;
    if (!out.valid) {
        out.fallback_reason = "CONSERVATIVE_FALLBACK_NONFINITE";
        out.diagnostics.failure_reason = out.fallback_reason;
    }
    stampWarmStartMetrics(out, params, slosh_dyn, slosh);
    return out;
}

}  // namespace

ContinuousMpccSolverAcados::ContinuousMpccSolverAcados() = default;

ContinuousMpccSolverAcados::~ContinuousMpccSolverAcados() {
    if (capsule_ != nullptr) {
        auto* gen = static_cast<GenSolver*>(capsule_);
        gen->destroy();
        delete gen;
        capsule_ = nullptr;
    }
}

void ContinuousMpccSolverAcados::configure(const SolverParams& params, const VariantConfig& variant) {
    params_ = params;
    variant_ = variant;
    use_slosh_model_ = variant.slosh_enable;
    if (params_.warm_start_flatness_enable) {
        params_.warm_start.enable = true;
    }
    have_u_prev_ = false;
    have_previous_solution_ = false;
    previous_warm_start_solution_ = WarmStartOutput{};
    slosh_dyn_.configure(params.slosh);
    warm_start_generator_ = makeWarmStartGenerator(params_.warm_start, params_.platform);

    if (capsule_ != nullptr) {
        auto* old = static_cast<GenSolver*>(capsule_);
        old->destroy();
        delete old;
        capsule_ = nullptr;
    }
    auto* gen = new GenSolver();
    if (!gen->create(use_slosh_model_ ? GenSolver::SLOSH : GenSolver::B0)) {
        delete gen;
        capsule_ = nullptr;
        return;
    }
    capsule_ = gen;
}

bool ContinuousMpccSolverAcados::solve(
    const SolverInput& input,
    const ReferencePath& reference,
    SolverOutput& output) const {
    output = SolverOutput{};
    if (capsule_ == nullptr) {
        output.status = "ACADOS_NOT_CREATED";
        return false;
    }
    if (reference.empty()) {
        output.status = "NO_REFERENCE_PATH";
        return false;
    }

    auto* gen = static_cast<GenSolver*>(capsule_);
    const bool slosh = use_slosh_model_;

    ProgressProjector projector;
    const auto raw_proj = projector.project(reference, input.robot.x, input.robot.y);
    const auto proj = projector.project(reference, input.robot.x, input.robot.y, input.min_progress_s);
    output.projector_debug.min_progress_s = input.min_progress_s;
    if (raw_proj.valid) {
        output.projector_debug.raw_valid = true;
        output.projector_debug.raw_s = raw_proj.s;
        output.projector_debug.raw_distance = raw_proj.distance;
        output.projector_debug.raw_signed_distance = raw_proj.signed_distance;
        output.projector_debug.raw_x = raw_proj.point.x;
        output.projector_debug.raw_y = raw_proj.point.y;
        output.projector_debug.raw_yaw = raw_proj.point.yaw;
    }
    if (proj.valid) {
        output.projector_debug.guarded_valid = true;
        output.projector_debug.guarded_s = proj.s;
        output.projector_debug.guarded_distance = proj.distance;
        output.projector_debug.guarded_signed_distance = proj.signed_distance;
        output.projector_debug.guarded_x = proj.point.x;
        output.projector_debug.guarded_y = proj.point.y;
        output.projector_debug.guarded_yaw = proj.point.yaw;
        output.projector_debug.monotonic_clip_applied = raw_proj.valid && proj.s > raw_proj.s + 1e-9;
    }
    if (!proj.valid) {
        output.status = "PROJECTION_FAILED";
        return false;
    }

    const double len = reference.length();
    const double s0 = proj.s;
    output.progress_s = len > 1e-6 ? s0 / len : 0.0;
    output.progress_abs_s = s0;

    // 用求解器固化的 N（codegen 时确定），而非 input.horizon_steps，避免二者不一致导致越界。
    const int n = gen->n_horizon;
    const double Tf = input.dt * n;

    ReferenceSpline spline;
    spline.build(reference);
    const double s_end = std::min(len, s0 + params_.v_max * Tf);
    Eigen::Vector4d cx, cy;
    fitReferencePolynomials(spline, s0, s_end, cx, cy);

    const ReferenceSample ref0 = spline.sample(s0);
    const double ref0_x = polyEval(cx, s0);
    const double ref0_y = polyEval(cy, s0);
    const double ref0_yaw = std::atan2(polyDeriv(cy, s0), polyDeriv(cx, s0));
    const double dx0 = input.robot.x - ref0_x;
    const double dy0 = input.robot.y - ref0_y;
    output.stage0_reference_debug.s0 = s0;
    output.stage0_reference_debug.ref_x = ref0_x;
    output.stage0_reference_debug.ref_y = ref0_y;
    output.stage0_reference_debug.ref_yaw = ref0_yaw;
    output.stage0_reference_debug.ref_kappa = ref0.kappa;
    output.stage0_reference_debug.robot_x = input.robot.x;
    output.stage0_reference_debug.robot_y = input.robot.y;
    output.stage0_reference_debug.robot_yaw = input.robot.yaw;
    output.stage0_reference_debug.yaw_error = wrapAngle(input.robot.yaw - ref0_yaw);
    output.stage0_reference_debug.contour_error = std::sin(ref0_yaw) * dx0 - std::cos(ref0_yaw) * dy0;
    output.stage0_reference_debug.lag_error = -std::cos(ref0_yaw) * dx0 - std::sin(ref0_yaw) * dy0;

    const double e_c_ref = std::max(1e-3, 0.5 * params_.corridor_width);
    const double e_l_ref = std::max(0.1, params_.v_max * input.dt);
    const double requested_v_ref = input.has_v_ref_current ? input.v_ref_current : variant_.v_ref;
    const double v_ref = clampValue(requested_v_ref, 0.0, params_.v_max);
    output.v_ref_debug.configured = variant_.v_ref;
    output.v_ref_debug.requested = requested_v_ref;
    output.v_ref_debug.effective = v_ref;
    output.v_ref_debug.runtime_override = input.has_v_ref_current;
    output.v_ref_debug.status = input.v_ref_status;

    // slosh 物理：取自同一套 slosh_dynamics 核（§4.3），κ=1（与 slosh_models 的单位输入增益一致）。
    double c_h = 1.0, eta_ref = 1.0, eta_dot_ref = 1.0;
    double eta_max = 0.0;
    double eta_max_sq = kDisabledEtaMaxSq;
    double h_limit = 0.0;
    double omega_n = 0.0;
    double two_zeta_omega_n = 0.0, omega_n_sq = 0.0;
    if (slosh && slosh_dyn_.configured()) {
        omega_n = slosh_dyn_.omegaN();
        const double zeta = params_.slosh.damping_ratio;
        const double h_ref = std::max(1e-4, params_.slosh.slosh_height_ref);
        c_h = std::max(1e-6, slosh_dyn_.heightCoeff());
        two_zeta_omega_n = 2.0 * zeta * omega_n;
        omega_n_sq = omega_n * omega_n;
        eta_ref = std::max(1e-6, h_ref / c_h);      // 使 ||eta||/eta_ref == (c_h||eta||)/h_ref，与 primitive 一致
        eta_dot_ref = std::max(1e-6, omega_n * h_ref);
        if (variant_.slosh_constraint_enable) {
            h_limit = std::max(1e-6, params_.slosh.slosh_height_max);
            eta_max = std::max(1e-6, h_limit / c_h);
            eta_max_sq = eta_max * eta_max;
        }
    }
    output.slosh_summary.hard_constraint_enable = (slosh && variant_.slosh_constraint_enable &&
                                                   eta_max_sq < kDisabledEtaMaxSq);
    output.slosh_summary.h_limit = h_limit;
    output.slosh_summary.h_limit_margin = h_limit;
    output.slosh_hard_constraint.enabled = output.slosh_summary.hard_constraint_enable;
    output.slosh_hard_constraint.h_limit = h_limit;
    output.slosh_hard_constraint.height_coeff = c_h;
    output.slosh_hard_constraint.eta_max = output.slosh_summary.hard_constraint_enable ? eta_max : 0.0;
    output.slosh_hard_constraint.eta_max_sq = output.slosh_summary.hard_constraint_enable ? eta_max_sq : 0.0;
    output.slosh_hard_constraint.h_limit_margin = h_limit;
    output.slosh_hard_constraint.modal_only = true;
    output.slosh_hard_constraint.observer_uses_parabola = params_.slosh.use_parabola_term;
    output.slosh_cost_monitor.eta_ref = eta_ref;
    output.slosh_cost_monitor.eta_dot_ref = eta_dot_ref;
    output.slosh_cost_monitor.omega_n = omega_n;
    output.slosh_cost_monitor.height_coeff = c_h;
    output.slosh_cost_monitor.slosh_eta_dot_ratio = params_.slosh.slosh_eta_dot_ratio;

    double p[PARAM_MAX];
    for (int i = 0; i < PARAM_MAX; ++i) p[i] = 0.0;
    p[RX0] = cx(0); p[RX1] = cx(1); p[RX2] = cx(2); p[RX3] = cx(3);
    p[RY0] = cy(0); p[RY1] = cy(1); p[RY2] = cy(2); p[RY3] = cy(3);
    p[W_CONTOUR] = variant_.w_contour;
    p[W_LAG] = variant_.w_lag;
    p[W_PROGRESS] = variant_.w_progress;
    p[W_A] = variant_.w_control + variant_.w_accel;
    p[W_OMEGA] = variant_.w_control;          // omega 现在是状态(转向幅值)
    p[W_V] = variant_.w_v;                    // 物理速度 tracking 权重：防止 cmd_v 塌到 0
    p[W_VS] = variant_.w_vs;                  // v_s tracking 权重：防止弯处 creep
    p[W_ALPHA] = variant_.w_alpha;            // 转向角加速度权重(抗 chatter，所有 stage)
    p[E_C_REF] = e_c_ref;
    p[E_L_REF] = e_l_ref;
    p[V_REF] = v_ref;
    if (slosh) {
        p[TWO_ZETA_OMEGA_N] = two_zeta_omega_n;
        p[OMEGA_N_SQ] = omega_n_sq;
        p[KAPPA_X] = 1.0;
        p[KAPPA_Y] = 1.0;
        p[ETA_REF] = eta_ref;
        p[ETA_DOT_REF] = eta_dot_ref;
        p[W_SLOSH_ETA] = variant_.w_slosh;
        p[W_SLOSH_ETA_DOT] = variant_.w_slosh * params_.slosh.slosh_eta_dot_ratio;
        p[ETA_MAX_SQ] = eta_max_sq;
    }

    for (int stage = 0; stage <= n; ++stage) {
        // omega 已是状态(初值=实测 omega)，跨周期连续性由状态保证；仅 a/v_s 做第一帧连续性。
        if (stage == 0 && have_u_prev_) {
            p[W_DU_A] = variant_.w_du_a;
            p[W_DU_VS] = variant_.w_du_vs;
            p[A_PREV] = u_prev_[0];
            p[VS_PREV] = u_prev_[2];
        } else {
            p[W_DU_A] = 0.0; p[W_DU_VS] = 0.0;
            p[A_PREV] = 0.0; p[VS_PREV] = 0.0;
        }
        gen->update_params(stage, p);
    }

    double x0[10] = {input.robot.x, input.robot.y, input.robot.yaw, input.robot.v, s0,
                     input.robot.omega, 0, 0, 0, 0};
    if (slosh) {
        x0[6] = input.slosh.eta_x;
        x0[7] = input.slosh.eta_x_dot;
        x0[8] = input.slosh.eta_y;
        x0[9] = input.slosh.eta_y_dot;
    }
    output.runtime_bounds = makeRuntimeBounds(params_);
    output.generated_bounds = makeGeneratedBounds();
    output.first_shot_debug.progress_s = output.progress_s;
    output.first_shot_debug.progress_abs_s = output.progress_abs_s;
    output.first_shot_debug.x0_v = input.robot.v;
    output.first_shot_debug.x0_omega = input.robot.omega;
    output.first_shot_debug.x0_s = s0;

    applyRuntimeBounds(*gen, output.runtime_bounds, x0);

    ocp_nlp_config* cfg = gen->config();
    ocp_nlp_dims* dims = gen->dims();
    ocp_nlp_in* nlp_in = gen->in();
    ocp_nlp_out* nlp_out = gen->out();
    WarmStartOutput warm_start;
    bool warm_start_applied = false;
    const bool warm_start_requested = params_.warm_start.enable || params_.warm_start_flatness_enable;
    const WarmStartInput warm_input = makeWarmStartInput(
        input, reference, spline, s0, len, n, params_, slosh_dyn_, have_u_prev_, u_prev_);
    if (warm_start_requested && warm_start_generator_) {
        WarmStartDiagnostics diagnostics;
        warm_start_generator_->generate(warm_input, warm_start, diagnostics);
        warm_start.diagnostics = diagnostics;
        if (warm_start.valid) {
            setAcadosWarmStart(*gen, warm_start, slosh);
            warm_start_applied = true;
        }
    }
    if (warm_start_requested && !warm_start_applied && params_.warm_start.fallback_to_previous_solution && have_previous_solution_) {
        warm_start = makeShiftedPreviousWarmStart(
            previous_warm_start_solution_, input, s0, n, slosh, params_, slosh_dyn_);
        if (warm_start.valid) {
            setAcadosWarmStart(*gen, warm_start, slosh);
            warm_start_applied = true;
        }
    }
    if (warm_start_requested && !warm_start_applied && params_.warm_start.fallback_to_primitive) {
        warm_start = makeConservativeWarmStart(warm_input, params_, slosh_dyn_, slosh);
        if (warm_start.valid) {
            setAcadosWarmStart(*gen, warm_start, slosh);
            warm_start_applied = true;
        }
    }
    output.warm_start_diagnostics = warm_start.diagnostics;
    for (int k = 0; k < 3; ++k) {
        if (warm_start.valid && k < static_cast<int>(warm_start.states.size()) &&
            k < static_cast<int>(warm_start.controls.size())) {
            auto& head = output.warm_start_head_debug.points[k];
            head.valid = true;
            head.state_s = warm_start.states[k].s;
            head.state_omega = warm_start.states[k].omega;
            head.control_alpha = warm_start.controls[k].alpha;
            head.control_v_s = warm_start.controls[k].v_s;
        }
    }

    const int status = gen->solve();

    double time_tot = 0.0;
    ocp_nlp_get(gen->solver(), "time_tot", &time_tot);
    output.solver_time_ms = time_tot * 1000.0;
    output.first_shot_debug.status_code = static_cast<double>(status);
    if (status != 0) {
        output.success = false;
        output.status = "ACADOS_SOLVE_FAILED_" + std::to_string(status);
        output.cmd_v = 0.0;
        output.cmd_omega = 0.0;
        return false;
    }

    // 读轨迹 + 诊断量（contour/lag/slosh/控制），按 §11.5 对齐 primitive。
    const double inv_n = 1.0 / static_cast<double>(std::max(1, n));
    output.trajectory.reserve(n + 1);
    std::vector<WarmStartState> solved_states;
    solved_states.reserve(n + 1);
    std::vector<double> heights;
    heights.reserve(n + 1);
    double xk[10];
    for (int k = 0; k <= n; ++k) {
        ocp_nlp_out_get(cfg, dims, nlp_out, k, "x", xk);
        TrajectoryPoint pt;
        pt.x = xk[0]; pt.y = xk[1]; pt.yaw = xk[2]; pt.v = xk[3]; pt.s = xk[4];
        output.trajectory.push_back(pt);
        solved_states.push_back(makeWarmStartState(xk, slosh));

        const double xref = polyEval(cx, pt.s);
        const double yref = polyEval(cy, pt.s);
        const double phi = std::atan2(polyDeriv(cy, pt.s), polyDeriv(cx, pt.s));
        const double e_c = std::sin(phi) * (pt.x - xref) - std::cos(phi) * (pt.y - yref);
        const double e_l = -std::cos(phi) * (pt.x - xref) - std::sin(phi) * (pt.y - yref);
        output.cost.J_contour += variant_.w_contour * (e_c / e_c_ref) * (e_c / e_c_ref) * inv_n;
        output.cost.J_lag += variant_.w_lag * (e_l / e_l_ref) * (e_l / e_l_ref) * inv_n;

        if (slosh) {
            const double ex = xk[6], exd = xk[7], ey = xk[8], eyd = xk[9];
            const double eta_norm = std::hypot(ex, ey);
            const double eta_dot_norm = std::hypot(exd, eyd);
            const double h = c_h * eta_norm;
            heights.push_back(h);
            if (h > output.slosh_summary.h_peak_pred) {
                output.slosh_summary.h_peak_pred = h;
                output.slosh_summary.peak_k = k;
            }
            output.slosh_summary.eta_x_peak = std::max(output.slosh_summary.eta_x_peak, std::abs(ex));
            output.slosh_summary.eta_y_peak = std::max(output.slosh_summary.eta_y_peak, std::abs(ey));
            output.slosh_summary.eta_dot_norm_peak = std::max(output.slosh_summary.eta_dot_norm_peak, eta_dot_norm);
            output.slosh_cost_monitor.eta_norm_peak = std::max(output.slosh_cost_monitor.eta_norm_peak, eta_norm);
            output.slosh_cost_monitor.eta_dot_norm_peak = std::max(output.slosh_cost_monitor.eta_dot_norm_peak, eta_dot_norm);
            output.cost.J_slosh_eta += variant_.w_slosh * (eta_norm / eta_ref) * (eta_norm / eta_ref) * inv_n;
            output.cost.J_slosh_eta_dot += variant_.w_slosh * params_.slosh.slosh_eta_dot_ratio *
                (eta_dot_norm / eta_dot_ref) * (eta_dot_norm / eta_dot_ref) * inv_n;
        }
    }

    for (int k = 0; k < 3 && k < static_cast<int>(solved_states.size()); ++k) {
        const auto& state = solved_states[k];
        auto& head = output.local_traj_head_debug.points[k];
        head.valid = true;
        head.x = state.px;
        head.y = state.py;
        head.yaw = state.theta;
        head.v = state.v;
        head.omega = state.omega;
        head.s = state.s;
        const auto head_proj = projector.project(reference, state.px, state.py);
        if (head_proj.valid) {
            head.proj_s = head_proj.s;
            head.proj_distance = head_proj.distance;
            head.proj_signed_distance = head_proj.signed_distance;
        }
        const double xref = polyEval(cx, state.s);
        const double yref = polyEval(cy, state.s);
        const double phi = std::atan2(polyDeriv(cy, state.s), polyDeriv(cx, state.s));
        const double dx = state.px - xref;
        const double dy = state.py - yref;
        head.contour_error = std::sin(phi) * dx - std::cos(phi) * dy;
        head.lag_error = -std::cos(phi) * dx - std::sin(phi) * dy;
        head.yaw_error = wrapAngle(state.theta - phi);
    }

    const double a_ref = std::max(0.1, params_.a_max);
    const double omega_ref = std::max(1e-3, params_.omega_max);
    const double alpha_ref = std::max(1e-3, params_.alpha_max);
    const double vs_ref = std::max(0.1, params_.v_max);
    std::vector<WarmStartControl> solved_controls;
    solved_controls.reserve(n);
    double uk[3], u0[3] = {0, 0, 0};
    for (int k = 0; k < n; ++k) {
        ocp_nlp_out_get(cfg, dims, nlp_out, k, "u", uk);
        solved_controls.push_back(makeWarmStartControl(uk));
        if (k == 0) { u0[0] = uk[0]; u0[1] = uk[1]; u0[2] = uk[2]; }
        const double an = uk[0] / a_ref;                          // a (控制)
        const double aln = uk[1] / alpha_ref;                     // alpha = omega-rate (控制)
        const double wn = solved_states[k].omega / omega_ref;     // omega 现在是状态
        output.cost.J_control += ((variant_.w_control + variant_.w_accel) * an * an +
                                  variant_.w_control * wn * wn +
                                  variant_.w_alpha * aln * aln) * inv_n;
        output.cost.J_progress += -variant_.w_progress * (uk[2] / vs_ref) * inv_n;
        const double vn = (solved_states[k].v - v_ref) / vs_ref;
        const double vsn = (uk[2] - v_ref) / vs_ref;
        output.cost.J_v += (variant_.w_v * vn * vn + variant_.w_vs * vsn * vsn) * inv_n;

        // a/v_s 跨周期第一帧连续性（stage 0）；omega 连续性由状态保证，Δomega 平滑由 w_alpha(全 stage)负责。
        if (k == 0 && have_u_prev_) {
            const double da = (uk[0] - u_prev_[0]) / a_ref;
            const double dvs = (uk[2] - u_prev_[2]) / vs_ref;
            output.cost.J_smooth += (variant_.w_du_a * da * da + variant_.w_du_vs * dvs * dvs) * inv_n;
        }
    }

    if (!heights.empty()) {
        std::vector<double> sorted = heights;
        std::sort(sorted.begin(), sorted.end());
        const size_t idx = std::min(sorted.size() - 1,
            static_cast<size_t>(std::floor(0.95 * (sorted.size() - 1))));
        output.slosh_summary.h_p95_pred = sorted[idx];
    }
    if (output.slosh_summary.hard_constraint_enable) {
        output.slosh_summary.h_limit_margin = output.slosh_summary.h_limit - output.slosh_summary.h_peak_pred;
    }
    output.slosh_hard_constraint.h_peak_pred = output.slosh_summary.h_peak_pred;
    output.slosh_hard_constraint.h_limit_margin = output.slosh_summary.h_limit_margin;
    output.slosh_hard_constraint.peak_k = output.slosh_summary.peak_k;

    const double abs_sum =
        std::abs(output.cost.J_contour) + std::abs(output.cost.J_lag) + std::abs(output.cost.J_progress) +
        std::abs(output.cost.J_v) + std::abs(output.cost.J_control) + std::abs(output.cost.J_smooth) +
        std::abs(output.cost.J_terminal) + std::abs(output.cost.J_corridor) + std::abs(output.cost.J_obstacle) +
        std::abs(output.cost.J_slosh_eta) + std::abs(output.cost.J_slosh_eta_dot);
    const double slosh_abs = std::abs(output.cost.J_slosh_eta) + std::abs(output.cost.J_slosh_eta_dot);
    output.slosh_cost_monitor.J_slosh_eta = output.cost.J_slosh_eta;
    output.slosh_cost_monitor.J_slosh_eta_dot = output.cost.J_slosh_eta_dot;
    output.slosh_cost_monitor.J_slosh_total = output.cost.J_slosh_eta + output.cost.J_slosh_eta_dot;
    output.slosh_cost_monitor.abs_cost_sum = abs_sum;
    output.slosh_cost_monitor.pct_slosh_total_abs_sum = abs_sum > 1e-9 ? 100.0 * slosh_abs / abs_sum : 0.0;
    output.slosh_cost_monitor.pct_eta_in_slosh = slosh_abs > 1e-9 ? 100.0 * std::abs(output.cost.J_slosh_eta) / slosh_abs : 0.0;
    output.slosh_cost_monitor.pct_eta_dot_in_slosh = slosh_abs > 1e-9 ? 100.0 * std::abs(output.cost.J_slosh_eta_dot) / slosh_abs : 0.0;

    // u = [a, alpha, v_s]; v_s 是虚拟路径进度速度，不直接作为 /cmd_vel.linear.x。
    // omega 是状态：下发角速度 = 实测 omega + alpha_0*dt（与 cmd_v 同口径单步积分）。
    const double cmd_v_pre = input.robot.v + u0[0] * input.dt;
    const double cmd_omega_pre = input.robot.omega + u0[1] * input.dt;
    output.cmd_v = clampValue(cmd_v_pre, 0.0, params_.v_max);
    output.cmd_omega = clampValue(cmd_omega_pre, -params_.omega_max, params_.omega_max);

    output.first_shot_debug.success = true;
    output.first_shot_debug.u0_a = u0[0];
    output.first_shot_debug.u0_alpha = u0[1];
    output.first_shot_debug.u0_v_s = u0[2];
    output.first_shot_debug.cmd_v_pre_clamp = cmd_v_pre;
    output.first_shot_debug.cmd_v_post_clamp = output.cmd_v;
    output.first_shot_debug.cmd_omega_pre_clamp = cmd_omega_pre;
    output.first_shot_debug.cmd_omega_post_clamp = output.cmd_omega;
    if (solved_states.size() > 1) {
        output.first_shot_debug.x1_v = solved_states[1].v;
        output.first_shot_debug.x1_omega = solved_states[1].omega;
        output.first_shot_debug.x1_s = solved_states[1].s;
    }
    if (solved_states.size() > 2) {
        output.first_shot_debug.x2_v = solved_states[2].v;
        output.first_shot_debug.x2_omega = solved_states[2].omega;
        output.first_shot_debug.x2_s = solved_states[2].s;
    }
    if (solved_states.size() > 3) {
        output.first_shot_debug.x3_v = solved_states[3].v;
        output.first_shot_debug.x3_omega = solved_states[3].omega;
        output.first_shot_debug.x3_s = solved_states[3].s;
    }

    u_prev_[0] = u0[0];
    u_prev_[1] = u0[1];
    u_prev_[2] = u0[2];
    have_u_prev_ = true;
    previous_warm_start_solution_.states = solved_states;
    previous_warm_start_solution_.controls = solved_controls;
    previous_warm_start_solution_.valid = !solved_states.empty() && solved_controls.size() == static_cast<size_t>(n);
    previous_warm_start_solution_.diagnostics = output.warm_start_diagnostics;
    have_previous_solution_ = previous_warm_start_solution_.valid;

    output.success = true;
    output.status = variant_.name + "_ACADOS_OK";
    return true;
}

}  // namespace spmpc_local_planner

#else  // SPMPC_WITH_ACADOS

namespace spmpc_local_planner {

ContinuousMpccSolverAcados::ContinuousMpccSolverAcados() = default;
ContinuousMpccSolverAcados::~ContinuousMpccSolverAcados() = default;

void ContinuousMpccSolverAcados::configure(const SolverParams& params, const VariantConfig& variant) {
    params_ = params;
    variant_ = variant;
}

bool ContinuousMpccSolverAcados::solve(
    const SolverInput& input,
    const ReferencePath& reference,
    SolverOutput& output) const {
    (void)input;
    (void)reference;
    output = SolverOutput{};
    output.success = false;
    output.status = "ACADOS_NOT_IMPLEMENTED";
    return false;
}

}  // namespace spmpc_local_planner

#endif  // SPMPC_WITH_ACADOS
