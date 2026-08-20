#include "spmpc_local_planner/solvers/continuous_mpcc_solver_acados.h"

#ifdef SPMPC_WITH_ACADOS

#include "spmpc_local_planner/reference/progress_projector.h"
#include "spmpc_local_planner/reference/reference_spline.h"
#include "spmpc_local_planner/solver/acados/stage_parameter_builder.h"
#include "spmpc_local_planner/warm_start/warm_start_factory.h"
#include "spmpc_parameter_manifest.h"

#include "acados_solver_spmpc_b0.h"
#ifdef SPMPC_WITH_ACADOS_SLOSH
#include "acados_solver_spmpc_slosh.h"
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
#include "acados_solver_spmpc_phase_rejoin.h"
#endif
#include "acados_c/ocp_nlp_interface.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace spmpc_local_planner {
namespace {

using namespace acados_manifest::mainline;

// The generated C++ manifest and generated acados headers must agree.  The
// Python model is now the only handwritten parameter-layout source.
static_assert(kB0ParameterCount == SPMPC_B0_NP,
              "B0 parameter manifest differs from generated solver");
#ifdef SPMPC_WITH_ACADOS_SLOSH
static_assert(kSloshParameterCount == SPMPC_SLOSH_NP,
              "slosh/phase-rejoin 参数布局与生成的 spmpc_slosh 求解器不一致");
static_assert(SPMPC_SLOSH_NH == kSloshNonlinearConstraintCount,
              "spmpc_slosh 求解器必须同时包含 slosh cap 和 empirical recovery gate");
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
static_assert(kSloshParameterCount == SPMPC_PHASE_REJOIN_NP,
              "phase-rejoin 参数布局与生成的短时求解器不一致");
static_assert(SPMPC_PHASE_REJOIN_NH == kSloshNonlinearConstraintCount,
              "phase-rejoin 求解器必须包含 liquid cap 与 terminal gate");
#endif

struct B0CapsuleDeleter {
    void operator()(spmpc_b0_solver_capsule* capsule) const {
        if (capsule == nullptr) return;
        spmpc_b0_acados_free(capsule);
        spmpc_b0_acados_free_capsule(capsule);
    }
};

#ifdef SPMPC_WITH_ACADOS_SLOSH
struct SloshCapsuleDeleter {
    void operator()(spmpc_slosh_solver_capsule* capsule) const {
        if (capsule == nullptr) return;
        spmpc_slosh_acados_free(capsule);
        spmpc_slosh_acados_free_capsule(capsule);
    }
};
#endif

#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
struct PhaseRejoinCapsuleDeleter {
    void operator()(spmpc_phase_rejoin_solver_capsule* capsule) const {
        if (capsule == nullptr) return;
        spmpc_phase_rejoin_acados_free(capsule);
        spmpc_phase_rejoin_acados_free_capsule(capsule);
    }
};
#endif

// 统一封装三个生成求解器，把前缀相关调用和各自的typed capsule
// 生命周期收敛到唯一的acados ABI边界。
struct GenSolver {
    enum Kind { B0, SLOSH, PHASE_REJOIN } kind = B0;
    std::unique_ptr<spmpc_b0_solver_capsule, B0CapsuleDeleter> b0_capsule;
#ifdef SPMPC_WITH_ACADOS_SLOSH
    std::unique_ptr<spmpc_slosh_solver_capsule, SloshCapsuleDeleter>
        slosh_capsule;
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
    std::unique_ptr<
        spmpc_phase_rejoin_solver_capsule,
        PhaseRejoinCapsuleDeleter> phase_rejoin_capsule;
#endif
    int nx = 0, nu = 0, np = 0, n_horizon = 0;

    GenSolver() = default;
    GenSolver(const GenSolver&) = delete;
    GenSolver& operator=(const GenSolver&) = delete;

    void reset() {
        b0_capsule.reset();
#ifdef SPMPC_WITH_ACADOS_SLOSH
        slosh_capsule.reset();
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
        phase_rejoin_capsule.reset();
#endif
        nx = 0;
        nu = 0;
        np = 0;
        n_horizon = 0;
    }

    bool create(Kind requested_kind) {
        reset();
        kind = requested_kind;
        if (kind == B0) {
            spmpc_b0_solver_capsule* capsule =
                spmpc_b0_acados_create_capsule();
            if (capsule == nullptr) return false;
            if (spmpc_b0_acados_create(capsule) != 0) {
                spmpc_b0_acados_free_capsule(capsule);
                return false;
            }
            b0_capsule.reset(capsule);
            nx = SPMPC_B0_NX;
            nu = SPMPC_B0_NU;
            np = SPMPC_B0_NP;
            n_horizon = SPMPC_B0_N;
            return true;
        }
        if (kind == SLOSH) {
#ifdef SPMPC_WITH_ACADOS_SLOSH
            spmpc_slosh_solver_capsule* capsule =
                spmpc_slosh_acados_create_capsule();
            if (capsule == nullptr) return false;
            if (spmpc_slosh_acados_create(capsule) != 0) {
                spmpc_slosh_acados_free_capsule(capsule);
                return false;
            }
            slosh_capsule.reset(capsule);
            nx = SPMPC_SLOSH_NX;
            nu = SPMPC_SLOSH_NU;
            np = SPMPC_SLOSH_NP;
            n_horizon = SPMPC_SLOSH_N;
            return true;
#else
            return false;
#endif
        }
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
        spmpc_phase_rejoin_solver_capsule* capsule =
            spmpc_phase_rejoin_acados_create_capsule();
        if (capsule == nullptr) return false;
        if (spmpc_phase_rejoin_acados_create(capsule) != 0) {
            spmpc_phase_rejoin_acados_free_capsule(capsule);
            return false;
        }
        phase_rejoin_capsule.reset(capsule);
        nx = SPMPC_PHASE_REJOIN_NX;
        nu = SPMPC_PHASE_REJOIN_NU;
        np = SPMPC_PHASE_REJOIN_NP;
        n_horizon = SPMPC_PHASE_REJOIN_N;
        return true;
#else
        return false;
#endif
    }

    void update_params(int stage, double* p) {
        if (kind == B0) {
            spmpc_b0_acados_update_params(
                b0_capsule.get(), stage, p, np);
        } else if (kind == SLOSH) {
#ifdef SPMPC_WITH_ACADOS_SLOSH
            spmpc_slosh_acados_update_params(
                slosh_capsule.get(), stage, p, np);
#endif
        } else {
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
            spmpc_phase_rejoin_acados_update_params(
                phase_rejoin_capsule.get(), stage, p, np);
#endif
        }
    }
    int solve() {
        if (kind == B0) {
            return spmpc_b0_acados_solve(b0_capsule.get());
        }
#ifdef SPMPC_WITH_ACADOS_SLOSH
        if (kind == SLOSH) {
            return spmpc_slosh_acados_solve(slosh_capsule.get());
        }
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
        if (kind == PHASE_REJOIN) {
            return spmpc_phase_rejoin_acados_solve(
                phase_rejoin_capsule.get());
        }
#endif
        return -1;
    }
    ocp_nlp_config* config() {
        if (kind == B0) {
            return spmpc_b0_acados_get_nlp_config(b0_capsule.get());
        }
#ifdef SPMPC_WITH_ACADOS_SLOSH
        if (kind == SLOSH) {
            return spmpc_slosh_acados_get_nlp_config(slosh_capsule.get());
        }
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
        if (kind == PHASE_REJOIN) {
            return spmpc_phase_rejoin_acados_get_nlp_config(
                phase_rejoin_capsule.get());
        }
#endif
        return nullptr;
    }
    ocp_nlp_dims* dims() {
        if (kind == B0) {
            return spmpc_b0_acados_get_nlp_dims(b0_capsule.get());
        }
#ifdef SPMPC_WITH_ACADOS_SLOSH
        if (kind == SLOSH) {
            return spmpc_slosh_acados_get_nlp_dims(slosh_capsule.get());
        }
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
        if (kind == PHASE_REJOIN) {
            return spmpc_phase_rejoin_acados_get_nlp_dims(
                phase_rejoin_capsule.get());
        }
#endif
        return nullptr;
    }
    ocp_nlp_in* in() {
        if (kind == B0) {
            return spmpc_b0_acados_get_nlp_in(b0_capsule.get());
        }
#ifdef SPMPC_WITH_ACADOS_SLOSH
        if (kind == SLOSH) {
            return spmpc_slosh_acados_get_nlp_in(slosh_capsule.get());
        }
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
        if (kind == PHASE_REJOIN) {
            return spmpc_phase_rejoin_acados_get_nlp_in(
                phase_rejoin_capsule.get());
        }
#endif
        return nullptr;
    }
    ocp_nlp_out* out() {
        if (kind == B0) {
            return spmpc_b0_acados_get_nlp_out(b0_capsule.get());
        }
#ifdef SPMPC_WITH_ACADOS_SLOSH
        if (kind == SLOSH) {
            return spmpc_slosh_acados_get_nlp_out(slosh_capsule.get());
        }
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
        if (kind == PHASE_REJOIN) {
            return spmpc_phase_rejoin_acados_get_nlp_out(
                phase_rejoin_capsule.get());
        }
#endif
        return nullptr;
    }
    ocp_nlp_solver* solver() {
        if (kind == B0) {
            return spmpc_b0_acados_get_nlp_solver(b0_capsule.get());
        }
#ifdef SPMPC_WITH_ACADOS_SLOSH
        if (kind == SLOSH) {
            return spmpc_slosh_acados_get_nlp_solver(slosh_capsule.get());
        }
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
        if (kind == PHASE_REJOIN) {
            return spmpc_phase_rejoin_acados_get_nlp_solver(
                phase_rejoin_capsule.get());
        }
#endif
        return nullptr;
    }
};

double clampValue(double value, double lo, double hi) {
    return std::max(lo, std::min(hi, value));
}

double wrapAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

bool finitePositive(double value) {
    return std::isfinite(value) && value > 0.0;
}

bool validEmpiricalRadii(const EmpiricalRecoveryRadii& radii) {
    const double values[] = {
        radii.x, radii.y, radii.yaw, radii.v, radii.omega,
        radii.eta_x, radii.eta_x_dot, radii.eta_y, radii.eta_y_dot,
    };
    for (double value : values) {
        if (!finitePositive(value)) return false;
    }
    return true;
}

bool finiteNominalStage(const PhaseNominalStage& stage) {
    const double values[] = {
        stage.x, stage.y, stage.yaw, stage.s, stage.v, stage.omega,
        stage.eta_x, stage.eta_x_dot, stage.eta_y, stage.eta_y_dot,
        stage.a, stage.alpha, stage.v_s,
    };
    for (double value : values) {
        if (!std::isfinite(value)) return false;
    }
    return true;
}

// Empty string means the enforce context is safe to inject.  The wrapper uses
// stable status suffixes so ROS diagnostics and modification reports can
// distinguish contract failures without parsing free-form text.
std::string validatePhaseRejoinContext(const PhaseRejoinSolverContext& context,
                                       int solver_horizon_steps) {
    if (!context.active || !context.enforce) return std::string{};
    if (!context.empirical_gate) return "EMPIRICAL_GATE_REQUIRED";
    if (!context.owns_terminal_maneuver) return "TERMINAL_TAIL_REQUIRED";
    if (context.front_steps < 0) return "NEGATIVE_FRONT_STEPS";
    if (context.liquid_steps <= 0 ||
        context.liquid_steps > solver_horizon_steps) {
        return "LIQUID_STEPS_OUT_OF_RANGE";
    }
    if (!std::isfinite(context.nominal_publish_v) ||
        !std::isfinite(context.nominal_publish_omega) ||
        !std::isfinite(context.max_residual_v) ||
        !std::isfinite(context.max_residual_omega) ||
        context.max_residual_v < 0.0 ||
        context.max_residual_omega < 0.0) {
        return "RESIDUAL_AUTHORITY";
    }
    const std::size_t expected_count =
        static_cast<std::size_t>(context.liquid_steps + 1);
    if (context.stages.size() != expected_count) return "STAGE_COUNT";

    const std::size_t front_offset =
        static_cast<std::size_t>(context.front_steps);
    const std::size_t liquid_offset =
        static_cast<std::size_t>(context.liquid_steps);
    if (context.current_index >
        std::numeric_limits<std::size_t>::max() - front_offset ||
        context.current_index + front_offset != context.front_index) {
        return "FRONT_INDEX";
    }
    if (context.front_index >
        std::numeric_limits<std::size_t>::max() - liquid_offset ||
        context.front_index + liquid_offset != context.terminal_index) {
        return "TERMINAL_INDEX";
    }

    for (std::size_t k = 0; k < context.stages.size(); ++k) {
        const PhaseNominalStage& stage = context.stages[k];
        const bool terminal = k == liquid_offset;
        if (!stage.valid) return "STAGE_INVALID";
        if (stage.gate_active != terminal) return "GATE_STAGE";
        if (context.front_index >
            std::numeric_limits<std::size_t>::max() - k ||
            stage.artifact_index != context.front_index + k) {
            return "ARTIFACT_INDEX";
        }
        if (!finiteNominalStage(stage)) return "NONFINITE_NOMINAL";
        // Artifact v1 requires valid radii on every row; only the terminal row
        // enters the NLP constraint, but validating all rows prevents a
        // malformed monitor artifact from becoming an enforce artifact later.
        if (!validEmpiricalRadii(stage.radii)) return "INVALID_RADIUS";
    }
    return std::string{};
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

bool applyPhaseResidualBounds(GenSolver& gen,
                              const SolverBoundSummary& bounds,
                              const SolverInput& input) {
    const auto& context = input.phase_rejoin;
    if (!context.active || !context.enforce || input.dt <= 1e-9) {
        return true;
    }

    // output.cmd = measured execution-front velocity + u0 * dt.  Intersecting
    // the stage-0 acceleration bounds with the nominal publish interval makes
    // the solver trajectory, certified first command, and published command
    // the same object; no post-solve residual clamp is permitted.
    const double residual_a_min =
        (context.nominal_publish_v - context.max_residual_v -
         input.robot.v) / input.dt;
    const double residual_a_max =
        (context.nominal_publish_v + context.max_residual_v -
         input.robot.v) / input.dt;
    const double residual_alpha_min =
        (context.nominal_publish_omega - context.max_residual_omega -
         input.robot.omega) / input.dt;
    const double residual_alpha_max =
        (context.nominal_publish_omega + context.max_residual_omega -
         input.robot.omega) / input.dt;

    double lbu[3] = {
        std::max(bounds.a_min, residual_a_min),
        std::max(bounds.alpha_min, residual_alpha_min),
        bounds.v_s_min,
    };
    double ubu[3] = {
        std::min(bounds.a_max, residual_a_max),
        std::min(bounds.alpha_max, residual_alpha_max),
        bounds.v_s_max,
    };
    if (lbu[0] > ubu[0] + 1e-10 || lbu[1] > ubu[1] + 1e-10) {
        return false;
    }
    ocp_nlp_constraints_model_set(
        gen.config(), gen.dims(), gen.in(), gen.out(), 0, "lbu", lbu);
    ocp_nlp_constraints_model_set(
        gen.config(), gen.dims(), gen.in(), gen.out(), 0, "ubu", ubu);
    return true;
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

HorizonStateDebug makeHorizonState(const WarmStartState& state, double h_modal = 0.0) {
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
                              double height_coeff,
                              std::vector<HorizonStateDebug>& states,
                              std::vector<HorizonControlDebug>& controls) {
    states.clear();
    controls.clear();
    states.reserve(warm_start.states.size());
    controls.reserve(warm_start.controls.size());
    for (const auto& state : warm_start.states) {
        const double h_modal = height_coeff * std::hypot(state.eta_x, state.eta_y);
        states.push_back(makeHorizonState(state, h_modal));
    }
    for (const auto& control : warm_start.controls) {
        controls.push_back(makeHorizonControl(control));
    }
}

void capturePrimalGuess(GenSolver& gen,
                        bool slosh,
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
    double x[10] = {0.0};
    double u[3] = {0.0};
    for (int k = 0; k <= gen.n_horizon; ++k) {
        std::fill(x, x + 10, 0.0);
        ocp_nlp_out_get(cfg, dims, nlp_out, k, "x", x);
        const WarmStartState state = makeWarmStartState(x, slosh);
        const double h_modal = height_coeff * std::hypot(state.eta_x, state.eta_y);
        states.push_back(makeHorizonState(state, h_modal));
        if (k < gen.n_horizon) {
            ocp_nlp_out_get(cfg, dims, nlp_out, k, "u", u);
            controls.push_back(makeHorizonControl(makeWarmStartControl(u)));
        }
    }
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

struct ContinuousMpccSolverAcados::Impl {
    std::unique_ptr<GenSolver> primary;
    // Enforce uses a separately generated N=N_l solver.  Keeping a distinct
    // capsule makes it impossible for the trusted liquid window to inherit the
    // baseline geometry tail.
    std::unique_ptr<GenSolver> phase_rejoin;
};

bool continuousMpccPhaseRejoinAvailable() {
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
    return true;
#else
    return false;
#endif
}

int continuousMpccPhaseRejoinHorizonSteps() {
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
    return SPMPC_PHASE_REJOIN_N;
#else
    return 0;
#endif
}

ContinuousMpccSolverAcados::ContinuousMpccSolverAcados()
    : impl_(new Impl()) {}

ContinuousMpccSolverAcados::~ContinuousMpccSolverAcados() = default;

SolverConfigureResult ContinuousMpccSolverAcados::configure(
    const SolverParams& params,
    const VariantConfig& variant) {
    params_ = params;
    variant_ = variant;
    use_slosh_model_ = variant.slosh_enable;
    if (params_.warm_start_flatness_enable) {
        params_.warm_start.enable = true;
    }
    have_u_prev_ = false;
    have_previous_solution_ = false;
    previous_warm_start_solution_ = WarmStartOutput{};
    const bool slosh_configured = slosh_dyn_.configure(params.slosh);
    if (use_slosh_model_ && !slosh_configured) {
        SolverConfigureResult result;
        result.status = "ACADOS_SLOSH_DYNAMICS_CONFIG_FAILED";
        return result;
    }
    warm_start_generator_ = makeWarmStartGenerator(params_.warm_start, params_.platform);

    impl_.reset(new Impl());
    std::unique_ptr<GenSolver> gen(new GenSolver());
    if (!gen->create(
            use_slosh_model_ ? GenSolver::SLOSH : GenSolver::B0)) {
        SolverConfigureResult result;
        result.status = use_slosh_model_
            ? "ACADOS_SLOSH_CAPSULE_CREATE_FAILED"
            : "ACADOS_B0_CAPSULE_CREATE_FAILED";
        return result;
    }
    impl_->primary = std::move(gen);
    if (use_slosh_model_) {
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
        std::unique_ptr<GenSolver> phase_gen(new GenSolver());
        if (!phase_gen->create(GenSolver::PHASE_REJOIN)) {
            impl_.reset(new Impl());
            SolverConfigureResult result;
            result.status = "ACADOS_PHASE_REJOIN_CAPSULE_CREATE_FAILED";
            return result;
        }
        impl_->phase_rejoin = std::move(phase_gen);
#endif
    }
    SolverConfigureResult result;
    result.success = true;
    result.status = "ACADOS_CONFIGURED";
    if (use_slosh_model_ && !impl_->phase_rejoin) {
        result.status = "ACADOS_CONFIGURED_PHASE_REJOIN_UNAVAILABLE";
        result.detail =
            "main slosh capsule is ready; short phase-rejoin capsule is unavailable";
    }
    return result;
}

bool ContinuousMpccSolverAcados::solve(
    const SolverInput& input,
    const ReferencePath& reference,
    SolverOutput& output) {
    output = SolverOutput{};
    output.cycle_timing = input.cycle_timing;
    if (!impl_ || !impl_->primary) {
        output.status = "ACADOS_NOT_CREATED";
        return false;
    }
    if (reference.empty()) {
        output.status = "NO_REFERENCE_PATH";
        return false;
    }

    const bool phase_rejoin_requested =
        input.phase_rejoin.active && input.phase_rejoin.enforce;
    GenSolver* gen = phase_rejoin_requested
        ? impl_->phase_rejoin.get()
        : impl_->primary.get();
    if (gen == nullptr) {
        output.status = "PHASE_REJOIN_SHORT_SOLVER_NOT_CREATED";
        return false;
    }
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

    // `enforce` is the solver-side authorization bit.  An active monitor
    // context is intentionally ignored so off/monitor retain the baseline OCP
    // even if the ROS adapter forwards diagnostics by mistake.
    if (input.phase_rejoin.enforce && !input.phase_rejoin.active) {
        output.status = "PHASE_REJOIN_CONTEXT_INVALID_INACTIVE";
        return false;
    }
    const bool phase_rejoin_enforce =
        input.phase_rejoin.active && input.phase_rejoin.enforce;
    if (phase_rejoin_enforce && !slosh) {
        output.status = "PHASE_REJOIN_CONTEXT_INVALID_SLOSH_REQUIRED";
        return false;
    }
    if (phase_rejoin_enforce) {
        const std::string phase_error =
            validatePhaseRejoinContext(input.phase_rejoin, n);
        if (!phase_error.empty()) {
            output.status = "PHASE_REJOIN_CONTEXT_INVALID_" + phase_error;
            return false;
        }
    }

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
    snapshot.backend = "continuous_mpcc_acados";
    snapshot.variant = variant_.name;
    snapshot.slosh_enabled = slosh;
    snapshot.primal_guess_only = true;
    snapshot.control_semantics = "alpha";
    snapshot.dt = input.dt;
    snapshot.horizon_steps = n;
    snapshot.state_width = 10;
    snapshot.control_width = 3;
    snapshot.parameter_width = gen->np;
    snapshot.slosh_cost_horizon_steps = variant_.slosh_cost_horizon_steps;
    snapshot.slosh_cost_tail_discount = variant_.slosh_cost_tail_discount;
    snapshot.robot = input.robot;
    snapshot.slosh = input.slosh;
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
    // slosh 物理：取自同一套 slosh_dynamics 核（§4.3），κ=1（与 slosh_models 的单位输入增益一致）。
    double c_h = 1.0, eta_ref = 1.0, eta_dot_ref = 1.0;
    double eta_max = 0.0;
    double eta_max_sq = kAcadosDisabledEtaMaxSq;
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
        // eta_dot_ref 与 eta_ref 同口径：omega_n × eta_ref = omega_n × h_ref / c_h
        // 原曾误写为 omega_n × h_ref（比设计值大 c_h 倍），导致 eta_dot 惩罚被人为压小
        eta_dot_ref = std::max(1e-6, omega_n * eta_ref);
        if (variant_.slosh_constraint_enable) {
            h_limit = std::max(1e-6, params_.slosh.slosh_height_max);
            eta_max = std::max(1e-6, h_limit / c_h);
            eta_max_sq = eta_max * eta_max;
        }
    }
    output.slosh_summary.hard_constraint_enable = (slosh && variant_.slosh_constraint_enable &&
                                                   eta_max_sq < kAcadosDisabledEtaMaxSq);
    output.slosh_summary.h_limit = h_limit;
    output.slosh_summary.h_limit_margin = h_limit;
    output.slosh_hard_constraint.enabled = output.slosh_summary.hard_constraint_enable;
    output.slosh_hard_constraint.h_limit = h_limit;
    output.slosh_hard_constraint.height_coeff = c_h;
    output.slosh_hard_constraint.eta_max = output.slosh_summary.hard_constraint_enable ? eta_max : 0.0;
    output.slosh_hard_constraint.eta_max_sq = output.slosh_summary.hard_constraint_enable ? eta_max_sq : 0.0;
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

    AcadosStageParameterInput parameter_input;
    parameter_input.horizon_steps = n;
    parameter_input.slosh_enabled = slosh;
    for (int index = 0; index < 4; ++index) {
        parameter_input.reference_x_coeffs[static_cast<std::size_t>(index)] =
            cx(index);
        parameter_input.reference_y_coeffs[static_cast<std::size_t>(index)] =
            cy(index);
    }
    parameter_input.variant = variant_;
    parameter_input.contour_error_ref = e_c_ref;
    parameter_input.lag_error_ref = e_l_ref;
    parameter_input.effective_v_ref = v_ref;
    parameter_input.slosh.two_zeta_omega_n = two_zeta_omega_n;
    parameter_input.slosh.omega_n_sq = omega_n_sq;
    parameter_input.slosh.eta_ref = eta_ref;
    parameter_input.slosh.eta_dot_ref = eta_dot_ref;
    parameter_input.slosh.eta_max_sq = eta_max_sq;
    parameter_input.slosh.eta_dot_weight_ratio =
        params_.slosh.slosh_eta_dot_ratio;
    parameter_input.have_previous_control = have_u_prev_;
    parameter_input.previous_control = {{
        u_prev_[0], u_prev_[1], u_prev_[2]}};
    parameter_input.phase_rejoin = input.phase_rejoin;

    AcadosStageParameterMatrix stage_parameters =
        AcadosStageParameterBuilder::build(parameter_input);
    if (!stage_parameters.valid) {
        output.status = "ACADOS_PARAMETER_BUILD_FAILED_" +
            stage_parameters.status;
        return false;
    }
    if (stage_parameters.parameter_width != gen->np) {
        output.status = "ACADOS_PARAMETER_WIDTH_MISMATCH";
        return false;
    }
    snapshot.parameter_names = stage_parameters.parameter_names;
    snapshot.stage_parameters = stage_parameters.values;
    for (int stage = 0; stage <= n; ++stage) {
        gen->update_params(stage, stage_parameters.stageData(stage));
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
    snapshot.runtime_bounds = output.runtime_bounds;
    output.generated_bounds = makeGeneratedBounds();
    output.first_shot_debug.progress_s = output.progress_s;
    output.first_shot_debug.progress_abs_s = output.progress_abs_s;
    output.first_shot_debug.x0_v = input.robot.v;
    output.first_shot_debug.x0_omega = input.robot.omega;
    output.first_shot_debug.x0_s = s0;

    applyRuntimeBounds(*gen, output.runtime_bounds, x0);
    if (!applyPhaseResidualBounds(*gen, output.runtime_bounds, input)) {
        output.status = "PHASE_REJOIN_RESIDUAL_AUTHORITY_INFEASIBLE";
        return false;
    }

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
            previous_warm_start_solution_, c_h,
            snapshot.previous_solution_states,
            snapshot.previous_solution_controls);
    }
    const WarmStartInput warm_input = makeWarmStartInput(
        input, reference, spline, s0, len, n, params_, slosh_dyn_, have_u_prev_, u_prev_);
    if (warm_start_requested && warm_start_generator_) {
        WarmStartDiagnostics diagnostics;
        warm_start_generator_->generate(warm_input, warm_start, diagnostics);
        warm_start.diagnostics = diagnostics;
        if (warm_start.valid) {
            setAcadosWarmStart(*gen, warm_start, slosh);
            warm_start_applied = true;
            snapshot.warm_start_source = warm_start.diagnostics.used_flatness ?
                "FLATNESS_GENERATOR" : "WARM_START_GENERATOR";
        }
    }
    if (warm_start_requested && !warm_start_applied && params_.warm_start.fallback_to_previous_solution && have_previous_solution_) {
        warm_start = makeShiftedPreviousWarmStart(
            previous_warm_start_solution_, input, s0, n, slosh, params_, slosh_dyn_);
        if (warm_start.valid) {
            setAcadosWarmStart(*gen, warm_start, slosh);
            warm_start_applied = true;
            snapshot.warm_start_source = "SHIFTED_PREVIOUS_SOLUTION";
        }
    }
    if (warm_start_requested && !warm_start_applied && params_.warm_start.fallback_to_primitive) {
        warm_start = makeConservativeWarmStart(warm_input, params_, slosh_dyn_, slosh);
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
    // Dual variables and internal SQP memory are intentionally not claimed by schema v1;
    // actual replay must still pass the frozen numerical reproduction gate.
    capturePrimalGuess(
        *gen, slosh, c_h,
        snapshot.initial_guess_states,
        snapshot.initial_guess_controls);

    const int status = gen->solve();

    double time_tot = 0.0;
    ocp_nlp_get(gen->solver(), "time_tot", &time_tot);
    output.solver_time_ms = time_tot * 1000.0;
    output.first_shot_debug.status_code = static_cast<double>(status);
    if (status != 0) {
        snapshot.solver_status = "ACADOS_SOLVE_FAILED_" + std::to_string(status);
        output.success = false;
        output.status = "ACADOS_SOLVE_FAILED_" + std::to_string(status);
        output.cmd_v = 0.0;
        output.cmd_omega = 0.0;
        return false;
    }

    // 读轨迹 + 诊断量（contour/lag/slosh/控制），按 §11.5 对齐 primitive。
    const double inv_n = 1.0 / static_cast<double>(std::max(1, n));
    output.trajectory.reserve(n + 1);
    output.predicted_horizon.backend = "continuous_mpcc_acados";
    output.predicted_horizon.variant = variant_.name;
    output.predicted_horizon.slosh_enabled = slosh;
    output.predicted_horizon.control_semantics = "alpha";
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
    const double a_ref = std::max(0.1, params_.a_max);
    const double omega_ref = std::max(1e-3, params_.omega_max);
    const double alpha_ref = std::max(1e-3, params_.alpha_max);
    const double vs_ref = std::max(0.1, params_.v_max);
    double xk[10];
    for (int k = 0; k <= n; ++k) {
        ocp_nlp_out_get(cfg, dims, nlp_out, k, "x", xk);
        TrajectoryPoint pt;
        pt.x = xk[0]; pt.y = xk[1]; pt.yaw = xk[2]; pt.v = xk[3]; pt.s = xk[4];
        output.trajectory.push_back(pt);
        const WarmStartState solved_state = makeWarmStartState(xk, slosh);
        solved_states.push_back(solved_state);
        const double solved_h_modal = slosh ? c_h * std::hypot(solved_state.eta_x, solved_state.eta_y) : 0.0;
        output.predicted_horizon.states.push_back(makeHorizonState(solved_state, solved_h_modal));

        const double xref = polyEval(cx, pt.s);
        const double yref = polyEval(cy, pt.s);
        const double phi = std::atan2(polyDeriv(cy, pt.s), polyDeriv(cx, pt.s));
        const double e_c = std::sin(phi) * (pt.x - xref) - std::cos(phi) * (pt.y - yref);
        const double e_l = -std::cos(phi) * (pt.x - xref) - std::sin(phi) * (pt.y - yref);
        // Generated stage costs are divided by N, while the terminal cost is
        // not.  Mirror that convention so diagnostics describe the OCP that
        // actually produced the command.
        const double cost_scale = k < n ? inv_n : 1.0;
        output.cost.J_contour += variant_.w_contour *
            (e_c / e_c_ref) * (e_c / e_c_ref) * cost_scale;
        output.cost.J_lag += variant_.w_lag *
            (e_l / e_l_ref) * (e_l / e_l_ref) * cost_scale;

        if (slosh) {
            const double ex = xk[6], exd = xk[7], ey = xk[8], eyd = xk[9];
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
            double cost_ex = ex;
            double cost_exd = exd;
            double cost_ey = ey;
            double cost_eyd = eyd;
            if (phase_rejoin_enforce &&
                k <= input.phase_rejoin.liquid_steps) {
                const PhaseNominalStage& nominal =
                    input.phase_rejoin.stages[static_cast<std::size_t>(k)];
                cost_ex -= nominal.eta_x;
                cost_exd -= nominal.eta_x_dot;
                cost_ey -= nominal.eta_y;
                cost_eyd -= nominal.eta_y_dot;
            }
            const double eta_cost_norm = std::hypot(cost_ex, cost_ey);
            const double eta_dot_cost_norm = std::hypot(cost_exd, cost_eyd);
            const double stage_scale = phase_rejoin_enforce
                ? (k <= input.phase_rejoin.liquid_steps ? 1.0 : 0.0)
                : sloshCostStageScale(variant_, k, n);
            output.cost.J_slosh_eta += variant_.w_slosh * stage_scale *
                (eta_cost_norm / eta_ref) * (eta_cost_norm / eta_ref) *
                cost_scale;
            output.cost.J_slosh_eta_dot += variant_.w_slosh * stage_scale *
                params_.slosh.slosh_eta_dot_ratio *
                (eta_dot_cost_norm / eta_dot_ref) *
                (eta_dot_cost_norm / eta_dot_ref) * cost_scale;
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

    if (phase_rejoin_enforce && n <= input.phase_rejoin.liquid_steps &&
        static_cast<std::size_t>(n) < input.phase_rejoin.stages.size()) {
        const PhaseNominalStage& terminal_nominal =
            input.phase_rejoin.stages[static_cast<std::size_t>(n)];
        const double dv_nominal =
            (solved_states[static_cast<std::size_t>(n)].v -
             terminal_nominal.v) / vs_ref;
        const double domega_nominal =
            (solved_states[static_cast<std::size_t>(n)].omega -
             terminal_nominal.omega) / omega_ref;
        // terminal_cost_expr is not divided by N.
        output.cost.J_v += variant_.w_v * dv_nominal * dv_nominal;
        output.cost.J_control +=
            variant_.w_control * domega_nominal * domega_nominal;
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
        const bool phase_stage = phase_rejoin_enforce &&
            k <= input.phase_rejoin.liquid_steps;
        if (phase_stage) {
            const PhaseNominalStage& nominal =
                input.phase_rejoin.stages[static_cast<std::size_t>(k)];
            const double dv_nominal =
                (solved_states[k].v - nominal.v) / vs_ref;
            const double domega_nominal =
                (solved_states[k].omega - nominal.omega) / omega_ref;
            const double da_nominal = (uk[0] - nominal.a) / a_ref;
            const double dalpha_nominal =
                (uk[1] - nominal.alpha) / alpha_ref;
            const double dvs_nominal = (uk[2] - nominal.v_s) / vs_ref;
            output.cost.J_v +=
                (variant_.w_v * dv_nominal * dv_nominal +
                 variant_.w_vs * dvs_nominal * dvs_nominal) * inv_n;
            output.cost.J_control +=
                ((variant_.w_control + variant_.w_accel) *
                     da_nominal * da_nominal +
                 variant_.w_control * domega_nominal * domega_nominal +
                 variant_.w_alpha * dalpha_nominal * dalpha_nominal) * inv_n;
        } else {
            const double an = uk[0] / a_ref;                      // a (控制)
            const double aln = uk[1] / alpha_ref;                 // alpha = omega-rate
            const double wn = solved_states[k].omega / omega_ref;
            output.cost.J_control +=
                ((variant_.w_control + variant_.w_accel) * an * an +
                 variant_.w_control * wn * wn +
                 variant_.w_alpha * aln * aln) * inv_n;
            output.cost.J_progress +=
                -variant_.w_progress * (uk[2] / vs_ref) * inv_n;
            const double vn = (solved_states[k].v - v_ref) / vs_ref;
            const double vsn = (uk[2] - v_ref) / vs_ref;
            output.cost.J_v +=
                (variant_.w_v * vn * vn +
                 variant_.w_vs * vsn * vsn) * inv_n;

            // a/v_s 跨周期第一帧连续性（stage 0）；phase mode
            // replaces this baseline prior with the nominal-relative cost.
            if (k == 0 && have_u_prev_) {
                const double da = (uk[0] - u_prev_[0]) / a_ref;
                const double dvs = (uk[2] - u_prev_[2]) / vs_ref;
                output.cost.J_smooth +=
                    (variant_.w_du_a * da * da +
                     variant_.w_du_vs * dvs * dvs) * inv_n;
            }
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
    output.predicted_horizon.valid = true;
    output.predicted_horizon.solver_status = output.status;
    snapshot.solver_status = output.status;
    return true;
}

}  // namespace spmpc_local_planner

#else  // SPMPC_WITH_ACADOS

namespace spmpc_local_planner {

struct ContinuousMpccSolverAcados::Impl {};

bool continuousMpccPhaseRejoinAvailable() {
    return false;
}

int continuousMpccPhaseRejoinHorizonSteps() {
    return 0;
}

ContinuousMpccSolverAcados::ContinuousMpccSolverAcados()
    : impl_(new Impl()) {}
ContinuousMpccSolverAcados::~ContinuousMpccSolverAcados() = default;

SolverConfigureResult ContinuousMpccSolverAcados::configure(
    const SolverParams& params,
    const VariantConfig& variant) {
    params_ = params;
    variant_ = variant;
    SolverConfigureResult result;
    result.status = "ACADOS_NOT_IMPLEMENTED";
    result.detail = "package was built without the generated mainline acados solver";
    return result;
}

bool ContinuousMpccSolverAcados::solve(
    const SolverInput& input,
    const ReferencePath& reference,
    SolverOutput& output) {
    (void)reference;
    output = SolverOutput{};
    output.cycle_timing = input.cycle_timing;
    output.success = false;
    output.status = "ACADOS_NOT_IMPLEMENTED";
    return false;
}

}  // namespace spmpc_local_planner

#endif  // SPMPC_WITH_ACADOS
