#include "spmpc_local_planner/solvers/continuous_mpcc_solver_acados.h"

#ifdef SPMPC_WITH_ACADOS

#include "spmpc_local_planner/reference/progress_projector.h"
#include "spmpc_local_planner/core/ocp_cost_evaluator.h"
#include "../core/generated/ocp_cost_contract.h"
#include "../core/generated/ocp_parameter_contract.h"
#include "spmpc_local_planner/reference/reference_spline.h"
#include "spmpc_local_planner/warm_start/warm_start_factory.h"
#include "spmpc_local_planner/warm_start/explicit_actuator_warm_start.h"
#include "spmpc_local_planner/dynamics/explicit_state_rollout.h"

#include "acados_solver_spmpc_b0.h"
#include "spmpc_b0_model_contract.h"
#include "../dynamics/generated/slosh_kernel_contract.h"
#ifdef SPMPC_WITH_ACADOS_SLOSH
#include "acados_solver_spmpc_slosh.h"
#include "spmpc_slosh_model_contract.h"
#endif
#include "acados_c/ocp_nlp_interface.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace spmpc_local_planner {
namespace {

using namespace ocp_parameters;

// 参数布局契约：与 scripts/acados/spmpc_acados_model.py（→生成器 NP 宏）绑死，漂移即编译失败。
static_assert(SPMPC_B0_LIQUID_MODEL_VERSION == SPMPC_LIQUID_KERNEL_VERSION &&
              SPMPC_B0_RK4_SUBSTEPS == SPMPC_LIQUID_RK4_SUBSTEPS,
              "Regenerate B0 for the shared RK4 motion model");
static_assert(kB0ParameterCount == SPMPC_B0_NP,
              "B0 参数布局与生成的 spmpc_b0 求解器不一致");
static_assert(SPMPC_B0_NBX==20 && SPMPC_B0_NBXN==20 && SPMPC_B0_NH==16,
              "Regenerate B0 vehicle/terminal/region/task bounds");
static_assert(SPMPC_B0_NX == kExplicitActuatorB0StateSize,
              "B0 状态布局与生成的 spmpc_b0 求解器不一致");
static_assert(SPMPC_B0_NG == 1,
              "Regenerate B0 acados artifacts for the full-horizon jerk switch");
#ifdef SPMPC_WITH_ACADOS_SLOSH
static_assert(SPMPC_SLOSH_NBX==20 && SPMPC_SLOSH_NBXN==20 && SPMPC_SLOSH_NH==17,
              "Regenerate slosh vehicle/terminal/region/task bounds");
static_assert(SPMPC_SLOSH_LIQUID_MODEL_VERSION == SPMPC_LIQUID_KERNEL_VERSION &&
              SPMPC_SLOSH_RK4_SUBSTEPS == SPMPC_B0_RK4_SUBSTEPS,
              "Regenerate slosh for rotating-container model version 1");
static_assert(ETA_MAX_SQ + 1 == SPMPC_SLOSH_NP, "slosh 参数布局与生成的 spmpc_slosh 求解器不一致");
static_assert(SPMPC_SLOSH_NX == kExplicitActuatorSloshStateSize,
              "slosh 状态布局与生成的 spmpc_slosh 求解器不一致");
static_assert(SPMPC_SLOSH_NH > 0, "spmpc_slosh 求解器缺少 slosh hard constraint，请重新生成 acados artifacts");
static_assert(SPMPC_SLOSH_NG == 1,
              "Regenerate slosh acados artifacts for the full-horizon jerk switch");
#endif

static_assert(SPMPC_B0_COST_VERSION == SPMPC_COST_VERSION, "Regenerate B0 cost v3");
#ifdef SPMPC_WITH_ACADOS_SLOSH
static_assert(SPMPC_SLOSH_COST_VERSION == SPMPC_COST_VERSION, "Regenerate slosh cost v3");
#endif

constexpr double kDisabledEtaMaxSq = 1e12;

// 统一封装两个生成求解器（B0 24 维 / slosh 28 维），把前缀相关调用收敛到一处。
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
    bounds.v_min = params.actual_v_min;
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

void applyRuntimeBounds(GenSolver& gen, const SolverBoundSummary& bounds,
                        double* x0, double delta_a_max, const std::vector<OcpPlanningStage>& planning_stages) {
    ocp_nlp_config* cfg = gen.config();
    ocp_nlp_dims* dims = gen.dims();
    ocp_nlp_in* nlp_in = gen.in();
    ocp_nlp_out* nlp_out = gen.out();

    ocp_nlp_constraints_model_set(cfg, dims, nlp_in, nlp_out, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(cfg, dims, nlp_in, nlp_out, 0, "ubx", x0);

    double lbu[3] = {bounds.a_min, bounds.alpha_min, bounds.v_s_min};
    double ubu[3] = {bounds.a_max, bounds.alpha_max, bounds.v_s_max};
    double lg[1] = {-delta_a_max};
    double ug[1] = {delta_a_max};
    for (int stage = 0; stage < gen.n_horizon; ++stage) {
        ocp_nlp_constraints_model_set(cfg, dims, nlp_in, nlp_out, stage, "lbu", lbu);
        ocp_nlp_constraints_model_set(cfg, dims, nlp_in, nlp_out, stage, "ubu", ubu);
        ocp_nlp_constraints_model_set(cfg, dims, nlp_in, nlp_out, stage, "lg", lg);
        ocp_nlp_constraints_model_set(cfg, dims, nlp_in, nlp_out, stage, "ug", ug);
    }

    // Generated explicit-actuator models constrain actual v/omega and command
    // v/omega independently.  The latter are the values published to /cmd_vel.
    for (int stage = 1; stage <= gen.n_horizon; ++stage) {
        double lbx[20] = {bounds.v_min,bounds.omega_min,0.,bounds.omega_min};
        double ubx[20] = {bounds.v_max,bounds.omega_max,bounds.v_max,bounds.omega_max};
        for (int i=4;i<9;++i) {lbx[i]=0.;ubx[i]=bounds.v_max;}
        for (int i=9;i<19;++i) {lbx[i]=bounds.omega_min;ubx[i]=bounds.omega_max;}
        lbx[19]=bounds.a_min;ubx[19]=bounds.a_max;
        if (planning_stages[static_cast<size_t>(stage)].task_goal_active)
            for (int i=2;i<20;++i) lbx[i]=ubx[i]=0.;
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

WarmStartState makeWarmStartState(const double* x, bool slosh) {
    WarmStartState state;
    state.px = x[0]; state.py = x[1]; state.theta = x[2]; state.v = x[3]; state.s = x[4];
    state.omega = x[5];
    state.v_cmd = x[6];
    state.omega_cmd = x[7];
    for (int i = 0; i < kExplicitLinearDelaySteps; ++i) {
        state.linear_delay_queue[static_cast<size_t>(i)] = x[8 + i];
    }
    for (int i = 0; i < kExplicitAngularDelaySteps; ++i) {
        state.angular_delay_queue[static_cast<size_t>(i)] =
            x[8 + kExplicitLinearDelaySteps + i];
    }
    state.a_cmd_memory = x[kExplicitActuatorAccelMemoryIndex];
    if (slosh) {
        state.eta_x = x[kExplicitActuatorSloshStateOffset];
        state.eta_x_dot = x[kExplicitActuatorSloshStateOffset + 1];
        state.eta_y = x[kExplicitActuatorSloshStateOffset + 2];
        state.eta_y_dot = x[kExplicitActuatorSloshStateOffset + 3];
    }
    return state;
}

WarmStartControl makeWarmStartControl(const double* u) {
    WarmStartControl control;
    control.a = u[0]; control.alpha = u[1]; control.v_s = u[2];
    return control;
}

HorizonStateDebug makeHorizonState(const WarmStartState& state,
                                   const ActuatorModelParams& actuator,
                                   bool slosh,
                                   double h_modal = 0.0) {
    HorizonStateDebug out;
    out.x = state.px;
    out.y = state.py;
    out.yaw = state.theta;
    out.v = state.v;
    out.s = state.s;
    out.omega = state.omega;
    out.eta_x = state.eta_x;
    out.eta_x_dot = state.eta_x_dot;
    out.eta_y = state.eta_y;
    out.eta_y_dot = state.eta_y_dot;
    out.h_modal = h_modal;
    out.v_cmd = state.v_cmd;
    out.omega_cmd = state.omega_cmd;
    out.a_cmd_memory = state.a_cmd_memory;
    out.delayed_v_cmd = state.linear_delay_queue.front();
    out.delayed_omega_cmd = state.angular_delay_queue.front();
    out.a_actual =
        (actuator.linear_gain * out.delayed_v_cmd - state.v) /
        actuator.linear_tau_sec;
    out.alpha_actual =
        (actuator.angular_gain * out.delayed_omega_cmd - state.omega) /
        actuator.angular_tau_sec;
    out.model_state.reserve(static_cast<size_t>(
        slosh ? kExplicitActuatorSloshStateSize
              : kExplicitActuatorB0StateSize));
    out.model_state = {
        state.px, state.py, state.theta, state.v, state.s, state.omega,
        state.v_cmd, state.omega_cmd};
    out.model_state.insert(out.model_state.end(),
                           state.linear_delay_queue.begin(),
                           state.linear_delay_queue.end());
    out.model_state.insert(out.model_state.end(),
                           state.angular_delay_queue.begin(),
                           state.angular_delay_queue.end());
    out.model_state.push_back(state.a_cmd_memory);
    if (slosh) {
        out.model_state.push_back(state.eta_x);
        out.model_state.push_back(state.eta_x_dot);
        out.model_state.push_back(state.eta_y);
        out.model_state.push_back(state.eta_y_dot);
    }
    return out;
}

HorizonControlDebug makeHorizonControl(const WarmStartControl& control) {
    HorizonControlDebug out;
    out.a = control.a;
    out.alpha_or_omega = control.alpha;
    out.v_s = control.v_s;
    return out;
}

void copyWarmStartForSnapshot(const WarmStartOutput& warm_start,
                              const ActuatorModelParams& actuator,
                              bool slosh,
                              double height_coeff,
                              std::vector<HorizonStateDebug>& states,
                              std::vector<HorizonControlDebug>& controls) {
    states.clear();
    controls.clear();
    states.reserve(warm_start.states.size());
    controls.reserve(warm_start.controls.size());
    for (const auto& state : warm_start.states) {
        const double h_modal = height_coeff * std::hypot(state.eta_x, state.eta_y);
        states.push_back(makeHorizonState(state, actuator, slosh, h_modal));
    }
    for (const auto& control : warm_start.controls) {
        controls.push_back(makeHorizonControl(control));
    }
}

void capturePrimalGuess(GenSolver& gen,
                        bool slosh,
                        const ActuatorModelParams& actuator,
                        double height_coeff,
                        std::vector<HorizonStateDebug>& states,
                        std::vector<HorizonControlDebug>& controls) {
    states.clear();
    controls.clear();
    states.reserve(static_cast<size_t>(gen.n_horizon + 1));
    controls.reserve(static_cast<size_t>(gen.n_horizon));
    ocp_nlp_config* cfg = gen.config();
    ocp_nlp_dims* dims = gen.dims();
    ocp_nlp_out* nlp_out = gen.out();
    double x[kExplicitActuatorSloshStateSize] = {0.0};
    double u[3] = {0.0};
    for (int k = 0; k <= gen.n_horizon; ++k) {
        std::fill(x, x + kExplicitActuatorSloshStateSize, 0.0);
        ocp_nlp_out_get(cfg, dims, nlp_out, k, "x", x);
        const WarmStartState state = makeWarmStartState(x, slosh);
        const double h_modal = height_coeff * std::hypot(state.eta_x, state.eta_y);
        states.push_back(makeHorizonState(state, actuator, slosh, h_modal));
        if (k < gen.n_horizon) {
            ocp_nlp_out_get(cfg, dims, nlp_out, k, "u", u);
            controls.push_back(makeHorizonControl(makeWarmStartControl(u)));
        }
    }
}

void fillAcadosState(const WarmStartState& state, bool slosh, double* x) {
    x[0] = state.px; x[1] = state.py; x[2] = state.theta; x[3] = state.v; x[4] = state.s;
    x[5] = state.omega;
    x[6] = state.v_cmd;
    x[7] = state.omega_cmd;
    for (int i = 0; i < kExplicitLinearDelaySteps; ++i) {
        x[8 + i] = state.linear_delay_queue[static_cast<size_t>(i)];
    }
    for (int i = 0; i < kExplicitAngularDelaySteps; ++i) {
        x[8 + kExplicitLinearDelaySteps + i] =
            state.angular_delay_queue[static_cast<size_t>(i)];
    }
    x[kExplicitActuatorAccelMemoryIndex] = state.a_cmd_memory;
    if (slosh) {
        x[kExplicitActuatorSloshStateOffset] = state.eta_x;
        x[kExplicitActuatorSloshStateOffset + 1] = state.eta_x_dot;
        x[kExplicitActuatorSloshStateOffset + 2] = state.eta_y;
        x[kExplicitActuatorSloshStateOffset + 3] = state.eta_y_dot;
    }
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
    double x_guess[kExplicitActuatorSloshStateSize] = {0.0};
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
    warm_input.bounds.v_max = params.v_max;
    warm_input.bounds.omega_max = params.omega_max;
    warm_input.bounds.a_max = params.a_max;
    warm_input.bounds.omega_rate_max = params.alpha_max;
    warm_input.bounds.v_s_max = params.v_max;
    warm_input.config = params.warm_start;
    // Geometry/control seed only; the actual actuator rollout owns liquid propagation.
    warm_input.config.use_slosh_rollout = false;
    warm_input.have_previous_control = have_u_prev;
    if (have_u_prev && u_prev != nullptr) {
        warm_input.previous_a = u_prev[0];
        // Legacy field; alpha-state u_prev[1] is alpha, while the flatness generator does not consume previous_omega.
        warm_input.previous_omega = input.robot.omega;
        warm_input.previous_v_s = u_prev[2];
    }
    return warm_input;
}

void copyActuatorState(const ActuatorState& actuator, WarmStartState& state) {
    state.v_cmd = actuator.v_cmd;
    state.omega_cmd = actuator.omega_cmd;
    state.a_cmd_memory = actuator.a_cmd_memory;
    state.linear_delay_queue = actuator.linear_delay_queue;
    state.angular_delay_queue = actuator.angular_delay_queue;
}

WarmStartOutput makeShiftedPreviousWarmStart(const WarmStartOutput& previous,
                                             const SolverInput& input,
                                             double s0,
                                             int n,
                                             bool slosh,
                                             const SolverParams& params) {
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
    copyActuatorState(input.actuator, out.states[0]);
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
    return out;
}

WarmStartOutput makeConservativeWarmStart(const WarmStartInput& warm_input,
                                          const SolverParams& params) {
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
    out.valid = isWarmStartFinite(out);
    out.diagnostics.warm_start_valid = out.valid;
    if (!out.valid) {
        out.fallback_reason = "CONSERVATIVE_FALLBACK_NONFINITE";
        out.diagnostics.failure_reason = out.fallback_reason;
    }
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
    previous_iteration_wall_sec_ = 0.0;
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
    configuration_error_.clear();
    planning_adapter_.reset();
    try {
        if (params_.rti_iterations<1 || params_.rti_iterations>20 ||
            !std::isfinite(params_.max_prediction_defect) || params_.max_prediction_defect<0)
            throw std::invalid_argument("invalid RTI iteration/defect configuration");
        if (params_.planning.liquid_free_baseline && (variant_.slosh_enable ||
            variant_.slosh_constraint_enable || variant_.w_slosh != 0 || params_.task_stop.enable ||
            params_.liquid_limit.recovery_enable || params_.zero_liquid_initial_state))
            throw std::invalid_argument("raw MPCC baseline contains a liquid decision mechanism");
        planning_adapter_ = std::make_unique<OcpPlanningAdapter>(params_);
    } catch (const std::exception& e) {
        configuration_error_ = e.what();
        return;
    }
    std::string actuator_error;
    if (params_.actuator.mode != ExecutionModelMode::ExplicitActuator ||
        !validateActuatorModelParams(params_.actuator, &actuator_error)) {
        return;
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
    const SolverInput& observed_input,
    const ReferencePath& reference,
    SolverOutput& output) const {
    // Copy after common-epoch alignment and execution prediction. Observers,
    // command history and the caller's state remain untouched in every mode.
    SolverInput input = observed_input;
    if (use_slosh_model_ && params_.zero_liquid_initial_state) {
        input.slosh = SloshState{};
    }
    output = SolverOutput{};
    output.pre_solve_snapshot.max_prediction_defect = params_.max_prediction_defect;
    output.cycle_timing = input.cycle_timing;
    if ((params_.zero_liquid_initial_state && !use_slosh_model_) ||
        !std::isfinite(params_.jerk_max) || params_.jerk_max <= 0.0 ||
        !std::isfinite(input.dt) || input.dt <= 0.0 ||
        !std::isfinite(params_.jerk_max * input.dt)) {
        output.status = "INVALID_ABLATION_CONFIG";
        return false;
    }
    if (!std::isfinite(params_.anticreep_gain) || params_.anticreep_gain < 0.0) {
        output.status = "INVALID_COST_CONFIG";
        return false;
    }
    if (!configuration_error_.empty()) {
        output.status = "INVALID_PLANNING_CONFIG: " + configuration_error_;
        return false;
    }
    if (capsule_ == nullptr) {
        output.status = "ACADOS_NOT_CREATED";
        return false;
    }
    if (reference.empty()) {
        output.status = "NO_REFERENCE_PATH";
        return false;
    }
    if (params_.actuator.mode != ExecutionModelMode::ExplicitActuator ||
        !input.actuator.valid ||
        !std::isfinite(input.actuator.a_cmd_memory)) {
        output.status = "EXPLICIT_ACTUATOR_STATE_INVALID";
        return false;
    }

    auto* gen = static_cast<GenSolver*>(capsule_);
    const bool slosh = use_slosh_model_;

    const auto raw_proj = planning_adapter_->project(reference, input.robot.x, input.robot.y);
    ProgressProjectionState projection_state{true,input.min_progress_s};
    const auto proj = planning_adapter_->project(reference, input.robot.x, input.robot.y,
        projection_state, input.min_progress_s);
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

    auto& snapshot = output.pre_solve_snapshot;
    snapshot.valid = true;
    snapshot.backend = "continuous_mpcc_acados_explicit_actuator";
    snapshot.variant = variant_.name;
    snapshot.slosh_enabled = slosh;
    snapshot.zero_liquid_initial_state = params_.zero_liquid_initial_state;
    snapshot.jerk_limit_enable = params_.jerk_limit_enable;
    snapshot.jerk_max = params_.jerk_max;
    snapshot.delta_a_max = params_.jerk_limit_enable
        ? params_.jerk_max * input.dt : 1e15;
    snapshot.observed_slosh = observed_input.slosh;
    snapshot.primal_guess_only = true;
    snapshot.control_semantics = "a_cmd_alpha_cmd";
    snapshot.dt = input.dt;
    snapshot.horizon_steps = n;
    snapshot.state_width = gen->nx;
    snapshot.control_width = 3;
    snapshot.parameter_width = gen->np;
    snapshot.slosh_cost_horizon_steps = variant_.slosh_cost_horizon_steps;
    snapshot.slosh_cost_tail_discount = variant_.slosh_cost_tail_discount;
    snapshot.robot = input.robot;
    snapshot.slosh = input.slosh;
    snapshot.actuator = input.actuator;
    snapshot.min_progress_s = input.min_progress_s;
    snapshot.reference_length = len;
    snapshot.s0 = s0;
    snapshot.s_end = s_end;
    for (int i = 0; i < 4; ++i) {
        snapshot.reference_x_coeffs[i] = cx(i);
        snapshot.reference_y_coeffs[i] = cy(i);
    }
    snapshot.has_v_ref_current = input.has_v_ref_current;
    snapshot.configured_v_ref = variant_.v_ref;
    snapshot.requested_v_ref = requested_v_ref;
    snapshot.effective_v_ref = v_ref;
    snapshot.v_ref_status = input.v_ref_status;
    snapshot.have_previous_control = have_u_prev_;
    if (have_u_prev_) {
        snapshot.previous_a = u_prev_[0];
        snapshot.previous_alpha_or_omega = u_prev_[1];
        snapshot.previous_v_s = u_prev_[2];
    }
    snapshot.have_previous_solution = have_previous_solution_;
    snapshot.parameter_names = ocp_parameters::names(gen->np);

    // slosh 物理：取自同一套 slosh_dynamics 核（§4.3），κ=1（与 slosh_models 的单位输入增益一致）。
    double c_h = 1.0, eta_ref = 1.0, eta_dot_ref = 1.0;
    double eta_max = 0.0;
    double eta_max_sq = kDisabledEtaMaxSq;
    double h_limit = 0.0;
    LiquidLimitPolicy liquid_limit;
    std::string liquid_limit_error;
    if (!makeLiquidLimitPolicy(slosh && variant_.slosh_constraint_enable,
            params_.slosh.slosh_height_max, params_.liquid_limit, liquid_limit, liquid_limit_error)) {
        output.status = liquid_limit_error;
        snapshot.solver_status = output.status;
        return false;
    }
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
        // eta_dot_ref 与 eta_ref 同口径：omega_n × eta_ref = omega_n × h_ref / c_h
        // 原曾误写为 omega_n × h_ref（比设计值大 c_h 倍），导致 eta_dot 惩罚被人为压小
        eta_dot_ref = std::max(1e-6, omega_n * eta_ref);
        if (liquid_limit.enabled) {
            h_limit = liquid_limit.target_m;
            eta_max = liquid_limit.cap_m / c_h;
            eta_max_sq = eta_max * eta_max;
        }
    }
    output.slosh_summary.hard_constraint_enable = liquid_limit.enabled && !liquid_limit.recovery_enabled;
    output.slosh_summary.h_limit = h_limit;
    output.slosh_summary.h_limit_margin = h_limit;
    output.slosh_hard_constraint.enabled = output.slosh_summary.hard_constraint_enable;
    output.slosh_hard_constraint.recovery_enabled = liquid_limit.recovery_enabled;
    output.slosh_hard_constraint.recovery_budget_m = liquid_limit.recovery_budget_m;
    output.slosh_hard_constraint.cap_m = liquid_limit.cap_m;
    output.slosh_hard_constraint.physical_boundary_known = liquid_limit.physical_boundary_known;
    output.slosh_hard_constraint.physical_boundary_m = liquid_limit.physical_boundary_m;
    output.slosh_hard_constraint.initial_height_m = slosh_dyn_.height(observed_input.slosh);
    if (liquid_limit.enabled && params_.zero_liquid_initial_state) {
        output.status = "NOSTATE_INCOMPATIBLE_WITH_LIQUID_LIMIT";
        snapshot.solver_status = output.status;
        return false;
    }
    if (liquid_limit.enabled &&
        (output.slosh_hard_constraint.initial_height_m > liquid_limit.cap_m + 1e-9 ||
         (liquid_limit.physical_boundary_known && output.slosh_hard_constraint.initial_height_m >= liquid_limit.physical_boundary_m))) {
        output.status = liquid_limit.physical_boundary_known &&
            output.slosh_hard_constraint.initial_height_m >= liquid_limit.physical_boundary_m
            ? "LIQUID_PHYSICAL_BOUNDARY" : "LIQUID_RECOVERY_BUDGET_EXCEEDED";
        snapshot.solver_status = output.status;
        return false;
    }
    output.slosh_hard_constraint.h_limit = h_limit;
    output.slosh_hard_constraint.height_coeff = c_h;
    output.slosh_hard_constraint.eta_max = liquid_limit.enabled ? eta_max : 0.0;
    output.slosh_hard_constraint.eta_max_sq = liquid_limit.enabled ? eta_max_sq : 0.0;
    output.slosh_hard_constraint.h_limit_margin = h_limit;
    // Solver 硬约束/代价诊断统一采用 modal-only 高度 c_h·||eta||。
    // slosh/use_parabola_term 只属于 observer/可视化 total-height proxy；在当前 R=18.5mm、常用角速度下
    // r^2*omega^2/(4g) 为 0.01mm 量级，故不进入 solver 诊断，避免把转弯准静态项误当作模态晃动。
    output.slosh_hard_constraint.modal_only = true;
    output.slosh_hard_constraint.solver_uses_parabola = false;
    output.slosh_cost_monitor.eta_ref = eta_ref;
    output.slosh_cost_monitor.eta_dot_ref = eta_dot_ref;
    output.slosh_cost_monitor.omega_n = omega_n;
    output.slosh_cost_monitor.height_coeff = c_h;
    output.slosh_cost_monitor.slosh_eta_dot_ratio = params_.slosh.slosh_eta_dot_ratio;

    const auto nominal_horizon=planning_adapter_->nominalHorizon(s0,input.task_elapsed_sec,n);
    std::vector<double> stage_progress(static_cast<size_t>(n+1));
    for (int k = 0; k <= n; ++k) {
        double guess = nominal_horizon.empty() ? s0 + k*input.dt*v_ref : nominal_horizon[static_cast<size_t>(k)].state[4];
        if (have_previous_solution_ && previous_warm_start_solution_.states.size() == static_cast<size_t>(n+1))
            guess = previous_warm_start_solution_.states[static_cast<size_t>(std::min(k+1,n))].s;
        stage_progress[static_cast<size_t>(k)] = clampValue(guess, s0, len);
    }
    stage_progress.front() = s0;
    std::vector<OcpPlanningStage> planning_stages;
    try {
        planning_stages = planning_adapter_->prepare(reference, input, stage_progress, snapshot.planning);
    } catch (const std::exception& e) {
        output.status = "PLANNING_INPUT_INVALID: " + std::string(e.what());
        snapshot.solver_status = output.status;
        return false;
    }
    output.predicted_horizon.planning = snapshot.planning;

    if (planning_stages.front().has_geometry_reference) {
        const auto planned=planning_stages.front().sampleGeometry(s0);
        auto& d=output.stage0_reference_debug;
        d.ref_x=planned.x; d.ref_y=planned.y; d.ref_yaw=planned.psi; d.ref_kappa=planned.kappa;
        d.yaw_error=wrapAngle(input.robot.yaw-planned.psi);
        const double dx=input.robot.x-planned.x,dy=input.robot.y-planned.y;
        d.contour_error=std::sin(planned.psi)*dx-std::cos(planned.psi)*dy;
        d.lag_error=-std::cos(planned.psi)*dx-std::sin(planned.psi)*dy;
    }

    auto parameter_values = ocp_parameters::defaults();
    double* p = parameter_values.data();
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
    p[ACTUATOR_DT] = input.dt;
    p[ACTUATOR_TAU_V] = params_.actuator.linear_tau_sec;
    p[ACTUATOR_TAU_OMEGA] = params_.actuator.angular_tau_sec;
    p[ACTUATOR_GAIN_V] = params_.actuator.linear_gain;
    p[ACTUATOR_GAIN_OMEGA] = params_.actuator.angular_gain;
    p[ANTICREEP_GAIN] = params_.anticreep_gain;
    p[STOP_ACTIVE] = input.task_stop_active ? 1.0 : 0.0;
    p[STOP_GOAL_S] = input.task_stop_goal_s;
    p[STOP_BRAKE_ACCEL] = params_.a_max;
    p[STOP_DELAY_MARGIN] = params_.actuator.linear_delay_sec + params_.actuator.linear_tau_sec +
        params_.a_max / params_.jerk_max;
    p[STOP_VELOCITY_WEIGHT] = params_.task_stop.velocity_cost_weight;
    if (slosh) {
        p[TWO_ZETA_OMEGA_N] = two_zeta_omega_n;
        p[OMEGA_N_SQ] = omega_n_sq;
        p[KAPPA_X] = 1.0;
        p[KAPPA_Y] = 1.0;
        p[ETA_REF] = eta_ref;
        p[ETA_DOT_REF] = eta_dot_ref;
        p[ETA_MAX_SQ] = eta_max_sq;
        p[ETA_TARGET_SQ] = liquid_limit.enabled ? (h_limit/c_h)*(h_limit/c_h) : kDisabledEtaMaxSq;
        p[SLACK_LINEAR_WEIGHT] = liquid_limit.linear_weight;
        p[SLACK_QUADRATIC_WEIGHT] = liquid_limit.quadratic_weight;
    }

    for (int stage = 0; stage <= n; ++stage) {
        if (slosh) {
            const double stage_scale = sloshCostStageScale(variant_, stage, n);
            p[W_SLOSH_ETA] = variant_.w_slosh * stage_scale;
            p[W_SLOSH_ETA_DOT] = variant_.w_slosh *
                params_.slosh.slosh_eta_dot_ratio * stage_scale;
        }
        // a_cmd 通过 a_cmd_memory 在所有控制 stage 做连续性代价。
        // a_prev 仅保留在参数 ABI/快照中；显式代价不再消费它。
        p[W_DU_A] = variant_.w_du_a;
        p[A_PREV] = input.actuator.a_cmd_memory;
        // v_s 没有记忆状态，仍只在 stage 0 对上一次 solver 控制做跨周期连续性。
        if (stage == 0 && have_u_prev_) {
            p[W_DU_VS] = variant_.w_du_vs;
            p[VS_PREV] = u_prev_[2];
        } else {
            p[W_DU_VS] = 0.0;
            p[VS_PREV] = 0.0;
        }
        try {
            planning_adapter_->write(planning_stages[static_cast<size_t>(stage)], p, gen->np);
        } catch (const std::exception& e) {
            output.status = "PLANNING_ASSEMBLY_INVALID: " + std::string(e.what());
            snapshot.solver_status = output.status;
            return false;
        }
        snapshot.stage_parameters.insert(
            snapshot.stage_parameters.end(), p, p + gen->np);
        gen->update_params(stage, p);
    }

    WarmStartState initial_state;
    initial_state.px = input.robot.x;
    initial_state.py = input.robot.y;
    initial_state.theta = input.robot.yaw;
    initial_state.v = input.robot.v;
    initial_state.s = s0;
    initial_state.omega = input.robot.omega;
    initial_state.eta_x = input.slosh.eta_x;
    initial_state.eta_x_dot = input.slosh.eta_x_dot;
    initial_state.eta_y = input.slosh.eta_y;
    initial_state.eta_y_dot = input.slosh.eta_y_dot;
    copyActuatorState(input.actuator, initial_state);
    double x0[kExplicitActuatorSloshStateSize] = {0.0};
    fillAcadosState(initial_state, slosh, x0);
    output.runtime_bounds = makeRuntimeBounds(params_);
    snapshot.runtime_bounds = output.runtime_bounds;
    output.generated_bounds = makeGeneratedBounds();
    output.first_shot_debug.progress_s = output.progress_s;
    output.first_shot_debug.progress_abs_s = output.progress_abs_s;
    output.first_shot_debug.x0_v = input.robot.v;
    output.first_shot_debug.x0_omega = input.robot.omega;
    output.first_shot_debug.x0_s = s0;

    applyRuntimeBounds(*gen, output.runtime_bounds, x0, snapshot.delta_a_max, planning_stages);

    ocp_nlp_config* cfg = gen->config();
    ocp_nlp_dims* dims = gen->dims();
    ocp_nlp_in* nlp_in = gen->in();
    ocp_nlp_out* nlp_out = gen->out();
    WarmStartOutput warm_start;
    bool warm_start_applied = false;
    const bool warm_start_requested = params_.warm_start.enable || params_.warm_start_flatness_enable;
    snapshot.warm_start_requested = warm_start_requested;
    snapshot.warm_start_source = "CAPSULE_REUSE";
    if (have_previous_solution_) {
        copyWarmStartForSnapshot(
            previous_warm_start_solution_, params_.actuator, slosh, c_h,
            snapshot.previous_solution_states,
            snapshot.previous_solution_controls);
    }
    const WarmStartInput warm_input = makeWarmStartInput(
        input, reference, spline, s0, len, n, params_, have_u_prev_, u_prev_);
    if (warm_start_requested && params_.warm_start.use_previous_solution && have_previous_solution_) {
        warm_start=makeShiftedPreviousWarmStart(previous_warm_start_solution_,input,s0,n,slosh,params_);
        if (warm_start.valid) rolloutExplicitActuatorWarmStart(
            warm_start,warm_input,input.actuator,params_.actuator,slosh_dyn_,slosh);
        if (warm_start.valid) {
            setAcadosWarmStart(*gen,warm_start,slosh); warm_start_applied=true;
            snapshot.warm_start_source="SHIFTED_PREVIOUS_SOLUTION";
        }
    }
    if (warm_start_requested && !warm_start_applied && !nominal_horizon.empty()) {
        warm_start.valid=true;
        for (const auto& row:nominal_horizon) warm_start.states.push_back(makeWarmStartState(row.state.data(),slosh));
        for (int k=0;k<n;++k) warm_start.controls.push_back(makeWarmStartControl(nominal_horizon[static_cast<size_t>(k)].control.data()));
        if (warm_start.valid) rolloutExplicitActuatorWarmStart(
            warm_start,warm_input,input.actuator,params_.actuator,slosh_dyn_,slosh);
        if (warm_start.valid) {
            setAcadosWarmStart(*gen,warm_start,slosh); warm_start_applied=true;
            snapshot.warm_start_source="TRAJECTORY_PLAN";
        }
    }
    if (warm_start_requested && !warm_start_applied && warm_start_generator_) {
        WarmStartDiagnostics diagnostics;
        warm_start_generator_->generate(warm_input, warm_start, diagnostics);
        warm_start.diagnostics = diagnostics;
        if (warm_start.valid) {
            rolloutExplicitActuatorWarmStart(
                warm_start, warm_input, input.actuator, params_.actuator, slosh_dyn_, slosh);
        }
        if (warm_start.valid) {
            setAcadosWarmStart(*gen, warm_start, slosh);
            warm_start_applied = true;
            snapshot.warm_start_source = warm_start.diagnostics.used_flatness ?
                "FLATNESS_GENERATOR" : "WARM_START_GENERATOR";
        }
    }
    if (warm_start_requested && !warm_start_applied && params_.warm_start.fallback_to_previous_solution && have_previous_solution_) {
        warm_start = makeShiftedPreviousWarmStart(
            previous_warm_start_solution_, input, s0, n, slosh, params_);
        if (warm_start.valid) {
            rolloutExplicitActuatorWarmStart(
                warm_start, warm_input, input.actuator, params_.actuator, slosh_dyn_, slosh);
        }
        if (warm_start.valid) {
            setAcadosWarmStart(*gen, warm_start, slosh);
            warm_start_applied = true;
            snapshot.warm_start_source = "SHIFTED_PREVIOUS_SOLUTION";
        }
    }
    if (warm_start_requested && !warm_start_applied && params_.warm_start.fallback_to_primitive) {
        warm_start = makeConservativeWarmStart(warm_input, params_);
        if (warm_start.valid) {
            rolloutExplicitActuatorWarmStart(
                warm_start, warm_input, input.actuator, params_.actuator, slosh_dyn_, slosh);
        }
        if (warm_start.valid) {
            setAcadosWarmStart(*gen, warm_start, slosh);
            warm_start_applied = true;
            snapshot.warm_start_source = "CONSERVATIVE_FALLBACK";
        }
    }
    snapshot.warm_start_applied = warm_start_applied;
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

    // Capture the exact primal x/u guess present in the capsule immediately before solve().
    // Dual variables and internal SQP memory are intentionally not claimed by this schema;
    // actual replay must still pass the frozen numerical reproduction gate.
    capturePrimalGuess(
        *gen, slosh, params_.actuator, c_h,
        snapshot.initial_guess_states,
        snapshot.initial_guess_controls);

    int status = 0;
    double time_tot = 0.0;
    int iterations_executed = 0;
    double iteration_estimate = previous_iteration_wall_sec_;
    for (int iteration=0;iteration<params_.rti_iterations;++iteration) {
        if (!input.solve_budget.permits(iteration_estimate)) break;
        const auto iteration_start = SolveBudget::Clock::now();
        status=gen->solve();
        previous_iteration_wall_sec_ = std::chrono::duration<double>(
            SolveBudget::Clock::now() - iteration_start).count();
        iteration_estimate = std::max(iteration_estimate, previous_iteration_wall_sec_);
        ++iterations_executed;
        double iteration_time=0;
        ocp_nlp_get(gen->solver(), "time_tot", &iteration_time);
        time_tot+=iteration_time;
        if (status!=0) break;
    }
    output.solver_time_ms = time_tot * 1000.0;
    // The replay needs the number actually executed, not the configured cap.
    snapshot.rti_iterations = iterations_executed;
    output.predicted_horizon.rti_iterations = iterations_executed;
    if (iterations_executed == 0) {
        output.recoverable_solver_failure = true;
        output.status = snapshot.solver_status = "SOLVE_BUDGET_EXHAUSTED";
        return false;
    }
    output.first_shot_debug.status_code = static_cast<double>(status);
    if (status != 0) {
        output.recoverable_solver_failure = true;
        snapshot.solver_status = "ACADOS_SOLVE_FAILED_" + std::to_string(status);
        output.success = false;
        output.status = "ACADOS_SOLVE_FAILED_" + std::to_string(status);
        output.cmd_v = 0.0;
        output.cmd_omega = 0.0;
        return false;
    }

    // 读轨迹 + 诊断量（contour/lag/slosh/控制），按 §11.5 对齐 primitive。
    output.trajectory.reserve(n + 1);
    output.predicted_horizon.backend = "continuous_mpcc_acados_explicit_actuator";
    output.predicted_horizon.variant = variant_.name;
    output.predicted_horizon.slosh_enabled = slosh;
    output.predicted_horizon.zero_liquid_initial_state = params_.zero_liquid_initial_state;
    output.predicted_horizon.jerk_limit_enable = params_.jerk_limit_enable;
    output.predicted_horizon.jerk_max = params_.jerk_max;
    output.predicted_horizon.delta_a_max = snapshot.delta_a_max;
    output.predicted_horizon.control_semantics = "a_cmd_alpha_cmd";
    output.predicted_horizon.dt = input.dt;
    output.predicted_horizon.slosh_cost_horizon_steps =
        variant_.slosh_cost_horizon_steps;
    output.predicted_horizon.slosh_cost_tail_discount =
        variant_.slosh_cost_tail_discount;
    output.predicted_horizon.states.reserve(static_cast<size_t>(n + 1));
    output.predicted_horizon.controls.reserve(static_cast<size_t>(n));
    std::vector<WarmStartState> solved_states;
    solved_states.reserve(n + 1);
    std::vector<double> heights;
    heights.reserve(n + 1);
    double xk[kExplicitActuatorSloshStateSize] = {0.0};
    for (int k = 0; k <= n; ++k) {
        ocp_nlp_out_get(cfg, dims, nlp_out, k, "x", xk);
        TrajectoryPoint pt;
        pt.x = xk[0]; pt.y = xk[1]; pt.yaw = xk[2]; pt.v = xk[3]; pt.s = xk[4];
        output.trajectory.push_back(pt);
        const WarmStartState solved_state = makeWarmStartState(xk, slosh);
        solved_states.push_back(solved_state);
        const double solved_h_modal = slosh ? c_h * std::hypot(solved_state.eta_x, solved_state.eta_y) : 0.0;
        output.predicted_horizon.states.push_back(
            makeHorizonState(solved_state, params_.actuator, slosh,
                             solved_h_modal));

        if (slosh) {
            const double ex = xk[kExplicitActuatorSloshStateOffset];
            const double exd = xk[kExplicitActuatorSloshStateOffset + 1];
            const double ey = xk[kExplicitActuatorSloshStateOffset + 2];
            const double eyd = xk[kExplicitActuatorSloshStateOffset + 3];
            const double eta_norm = std::hypot(ex, ey);
            const double eta_dot_norm = std::hypot(exd, eyd);
            // Solver 预测高度保持 modal-only: h_modal = c_h·||eta||。
            // yaw-induced parabola 项刻意不进入 solver hard-constraint/cost 诊断（见上方 solver_uses_parabola=false）。
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

        }
    }

    ProgressProjectionState head_projection_state{true,s0};
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
        const auto head_proj = planning_adapter_->project(reference, state.px, state.py,
            head_projection_state, s0);
        if (head_proj.valid) {
            head.proj_s = head_proj.s;
            head.proj_distance = head_proj.distance;
            head.proj_signed_distance = head_proj.signed_distance;
        }
        auto sampled=ReferenceSample{};
        const auto& stage=planning_stages[static_cast<size_t>(k)];
        if (stage.has_geometry_reference) sampled=stage.sampleGeometry(state.s);
        else {
            sampled.x=polyEval(cx,state.s);sampled.y=polyEval(cy,state.s);
            sampled.psi=std::atan2(polyDeriv(cy,state.s),polyDeriv(cx,state.s));
        }
        const double xref = sampled.x, yref = sampled.y, phi = sampled.psi;
        const double dx = state.px - xref;
        const double dy = state.py - yref;
        head.contour_error = std::sin(phi) * dx - std::cos(phi) * dy;
        head.lag_error = -std::cos(phi) * dx - std::sin(phi) * dy;
        head.yaw_error = wrapAngle(state.theta - phi);
    }

    std::vector<WarmStartControl> solved_controls;
    solved_controls.reserve(n);
    double uk[3], u0[3] = {0, 0, 0};
    for (int k = 0; k < n; ++k) {
        ocp_nlp_out_get(cfg, dims, nlp_out, k, "u", uk);
        solved_controls.push_back(makeWarmStartControl(uk));
        output.predicted_horizon.controls.push_back(
            makeHorizonControl(solved_controls.back()));
        if (k == 0) { u0[0] = uk[0]; u0[1] = uk[1]; u0[2] = uk[2]; }
    }

    for (int k=0;k<n;++k) {
        const auto& current=output.predicted_horizon.states[static_cast<size_t>(k)].model_state;
        const auto& next=output.predicted_horizon.states[static_cast<size_t>(k+1)].model_state;
        const auto& u=solved_controls[static_cast<size_t>(k)];
        std::vector<double> replay;
        if (!stepExplicitState(current,{{u.a,u.alpha,u.v_s}},params_.actuator,slosh_dyn_,input.dt,replay) || replay.size()!=next.size()) {
            output.status=snapshot.solver_status="PREDICTION_DYNAMICS_INVALID"; return false;
        }
        for (size_t j=0;j<next.size();++j) {
            const double error=std::abs(next[j]-replay[j]);
            if (!std::isfinite(error)) {
                output.status=snapshot.solver_status="PREDICTION_DYNAMICS_INVALID"; return false;
            }
            output.predicted_horizon.dynamics_max_defect=std::max(output.predicted_horizon.dynamics_max_defect,error);
        }
    }
    if (params_.max_prediction_defect>0 && output.predicted_horizon.dynamics_max_defect>params_.max_prediction_defect) {
        output.status=snapshot.solver_status=output.predicted_horizon.solver_status="PREDICTION_DYNAMICS_VIOLATION";
        return false;
    }

    std::string planning_error;
    if (!planning_adapter_->check(planning_stages, output.predicted_horizon,
            output.predicted_horizon.planning.minimum_region_clearance, planning_error)) {
        output.status = planning_error;
        snapshot.solver_status = output.predicted_horizon.solver_status = output.status;
        return false;
    }
    snapshot.planning.minimum_region_clearance = output.predicted_horizon.planning.minimum_region_clearance;

    if (!evaluateOcpCost(output.predicted_horizon, snapshot.stage_parameters, gen->np, output.cost)) {
        output.status = "COST_RECONSTRUCTION_FAILED";
        snapshot.solver_status = output.status;
        return false;
    }
    ocp_nlp_eval_cost(gen->solver(), nlp_in, nlp_out);
    ocp_nlp_get(gen->solver(), "cost_value", &output.cost.solver_total);
    output.cost.reconstruction_error = output.cost.total() - output.cost.solver_total;
    output.cost.reconstruction_valid = std::isfinite(output.cost.solver_total) &&
        std::abs(output.cost.reconstruction_error) <= 1e-8 * (1.0 + std::abs(output.cost.solver_total));
    if (!output.cost.reconstruction_valid) {
        output.status = "COST_RECONSTRUCTION_MISMATCH";
        snapshot.solver_status = output.status;
        return false;
    }

    if (params_.jerk_limit_enable) {
        // Do not silently publish an RTI result that violates the hard row.
        // Check consecutive controls, including the truthful published-history
        // anchor, rather than trusting only the optimized memory coordinates.
        double previous_a = input.actuator.a_cmd_memory;
        for (const auto& control : solved_controls) {
            if (!std::isfinite(control.a) ||
                std::abs(control.a - previous_a) > snapshot.delta_a_max + 1e-6) {
                output.status = "JERK_CONSTRAINT_VIOLATION";
                snapshot.solver_status = output.status;
                output.predicted_horizon.solver_status = output.status;
                return false;
            }
            previous_a = control.a;
        }
    }

    if (!heights.empty()) {
        std::vector<double> sorted = heights;
        std::sort(sorted.begin(), sorted.end());
        const size_t idx = std::min(sorted.size() - 1,
            static_cast<size_t>(std::floor(0.95 * (sorted.size() - 1))));
        output.slosh_summary.h_p95_pred = sorted[idx];
    }
    if (liquid_limit.enabled) {
        auto& limit = output.slosh_hard_constraint;
        limit.maximum_excess_m = std::max(0.0, output.slosh_summary.h_peak_pred - h_limit);
        limit.exceedance_nodes = static_cast<int>(std::count_if(heights.begin(), heights.end(),
            [&](double h) { return h > h_limit + 1e-9; }));
        limit.recovery_used = limit.exceedance_nodes > 0;
        limit.strict_target_satisfied = !limit.recovery_used;
        if (output.slosh_summary.h_peak_pred > liquid_limit.cap_m + 1e-7 ||
            (liquid_limit.physical_boundary_known && output.slosh_summary.h_peak_pred >= liquid_limit.physical_boundary_m)) {
            output.status = "LIQUID_RECOVERY_CAP_VIOLATION";
            snapshot.solver_status = output.status;
            output.predicted_horizon.solver_status = output.status;
            return false;
        }
        output.slosh_summary.h_limit_margin = output.slosh_summary.h_limit - output.slosh_summary.h_peak_pred;
    }
    output.slosh_hard_constraint.h_peak_pred = output.slosh_summary.h_peak_pred;
    output.slosh_hard_constraint.h_limit_margin = output.slosh_summary.h_limit_margin;
    output.slosh_hard_constraint.peak_k = output.slosh_summary.peak_k;

    const double abs_sum =
        std::abs(output.cost.J_contour) + std::abs(output.cost.J_lag) + std::abs(output.cost.J_progress) +
        std::abs(output.cost.J_v) + std::abs(output.cost.J_control) + std::abs(output.cost.J_smooth) +
        std::abs(output.cost.J_anti_creep) + std::abs(output.cost.J_slack) + std::abs(output.cost.J_stop) +
        std::abs(output.cost.J_terminal) + std::abs(output.cost.J_corridor) + std::abs(output.cost.J_obstacle) +
        std::abs(output.cost.J_slosh_eta) + std::abs(output.cost.J_slosh_eta_dot) +
        std::abs(output.cost.J_curvature) + std::abs(output.cost.J_curvature_change);
    const double slosh_abs = std::abs(output.cost.J_slosh_eta) + std::abs(output.cost.J_slosh_eta_dot);
    output.slosh_cost_monitor.J_slosh_eta = output.cost.J_slosh_eta;
    output.slosh_cost_monitor.J_slosh_eta_dot = output.cost.J_slosh_eta_dot;
    output.slosh_cost_monitor.J_slosh_total = output.cost.J_slosh_eta + output.cost.J_slosh_eta_dot;
    output.slosh_cost_monitor.abs_cost_sum = abs_sum;
    output.slosh_cost_monitor.pct_slosh_total_abs_sum = abs_sum > 1e-9 ? 100.0 * slosh_abs / abs_sum : 0.0;
    output.slosh_cost_monitor.pct_eta_in_slosh = slosh_abs > 1e-9 ? 100.0 * std::abs(output.cost.J_slosh_eta) / slosh_abs : 0.0;
    output.slosh_cost_monitor.pct_eta_dot_in_slosh = slosh_abs > 1e-9 ? 100.0 * std::abs(output.cost.J_slosh_eta_dot) / slosh_abs : 0.0;

    // Publish the next command state.  Never rebuild a command from measured
    // actual velocity: actual and command are deliberately separate states.
    const double cmd_v_pre = solved_states.size() > 1
        ? solved_states[1].v_cmd
        : input.actuator.v_cmd + u0[0] * input.dt;
    const double cmd_omega_pre = solved_states.size() > 1
        ? solved_states[1].omega_cmd
        : input.actuator.omega_cmd + u0[1] * input.dt;
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
    output.predicted_horizon.valid = true;
    output.predicted_horizon.solver_status = output.status;
    snapshot.solver_status = output.status;
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
    (void)reference;
    output = SolverOutput{};
    output.cycle_timing = input.cycle_timing;
    output.success = false;
    output.status = "ACADOS_NOT_IMPLEMENTED";
    return false;
}

}  // namespace spmpc_local_planner

#endif  // SPMPC_WITH_ACADOS
