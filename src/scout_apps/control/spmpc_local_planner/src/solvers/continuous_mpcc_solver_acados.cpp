#include "spmpc_local_planner/solvers/continuous_mpcc_solver_acados.h"

#ifdef SPMPC_WITH_ACADOS

#include "spmpc_local_planner/reference/progress_projector.h"
#include "spmpc_local_planner/reference/reference_spline.h"
#include "spmpc_local_planner/solver/acados/solution_decoder.h"
#include "spmpc_local_planner/solver/acados/generated_solver.h"
#include "spmpc_local_planner/solver/acados/stage_parameter_builder.h"
#include "spmpc_local_planner/warm_start/warm_start_factory.h"
#include "spmpc_local_planner/warm_start/warm_start_policy.h"
#include "spmpc_parameter_manifest.h"

#include "acados_solver_spmpc_b0.h"
#ifdef SPMPC_WITH_ACADOS_SLOSH
#include "acados_solver_spmpc_slosh.h"
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
#include "acados_solver_spmpc_phase_rejoin.h"
#endif
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
static_assert(kB0StateCount == SPMPC_B0_NX,
              "B0 state manifest differs from generated solver");
static_assert(kControlCount == SPMPC_B0_NU,
              "B0 control manifest differs from generated solver");
static_assert(acados_manifest::generated_bounds::kMainHorizonSteps ==
                  SPMPC_B0_N,
              "B0 horizon manifest differs from generated solver");
#ifdef SPMPC_WITH_ACADOS_SLOSH
static_assert(kSloshParameterCount == SPMPC_SLOSH_NP,
              "slosh/phase-rejoin 参数布局与生成的 spmpc_slosh 求解器不一致");
static_assert(kSloshStateCount == SPMPC_SLOSH_NX,
              "slosh state manifest differs from generated solver");
static_assert(kControlCount == SPMPC_SLOSH_NU,
              "slosh control manifest differs from generated solver");
static_assert(acados_manifest::generated_bounds::kMainHorizonSteps ==
                  SPMPC_SLOSH_N,
              "slosh horizon manifest differs from generated solver");
static_assert(SPMPC_SLOSH_NH == kSloshNonlinearConstraintCount,
              "spmpc_slosh 求解器必须同时包含 slosh cap 和 empirical recovery gate");
#endif
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
static_assert(kSloshParameterCount == SPMPC_PHASE_REJOIN_NP,
              "phase-rejoin 参数布局与生成的短时求解器不一致");
static_assert(kSloshStateCount == SPMPC_PHASE_REJOIN_NX,
              "phase-rejoin state manifest differs from generated solver");
static_assert(kControlCount == SPMPC_PHASE_REJOIN_NU,
              "phase-rejoin control manifest differs from generated solver");
static_assert(acados_manifest::generated_bounds::kPhaseRejoinHorizonSteps ==
                  SPMPC_PHASE_REJOIN_N,
              "phase-rejoin horizon manifest differs from generated solver");
static_assert(SPMPC_PHASE_REJOIN_NH == kSloshNonlinearConstraintCount,
              "phase-rejoin 求解器必须包含 liquid cap 与 terminal gate");
#endif


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
    namespace generated = acados_manifest::generated_bounds;
    SolverBoundSummary bounds;
    bounds.a_min = -generated::kAMax;
    bounds.a_max = generated::kAMax;
    bounds.alpha_min = -generated::kAlphaMax;
    bounds.alpha_max = generated::kAlphaMax;
    bounds.v_s_min = 0.0;
    bounds.v_s_max = generated::kVsMax;
    bounds.v_min = 0.0;
    bounds.v_max = generated::kVMax;
    bounds.omega_min = -generated::kOmegaMax;
    bounds.omega_max = generated::kOmegaMax;
    return bounds;
}

void applyRuntimeBounds(GeneratedAcadosSolver& gen,
                        const SolverBoundSummary& bounds,
                        double* x0) {
    gen.setStateBounds(0, x0, x0);

    double lbu[3] = {bounds.a_min, bounds.alpha_min, bounds.v_s_min};
    double ubu[3] = {bounds.a_max, bounds.alpha_max, bounds.v_s_max};
    for (int stage = 0; stage < gen.horizonSteps(); ++stage) {
        gen.setControlBounds(stage, lbu, ubu);
    }

    double lbx[2] = {bounds.v_min, bounds.omega_min};
    double ubx[2] = {bounds.v_max, bounds.omega_max};
    for (int stage = 1; stage <= gen.horizonSteps(); ++stage) {
        gen.setStateBounds(stage, lbx, ubx);
    }
}

bool applyPhaseResidualBounds(GeneratedAcadosSolver& gen,
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
    return gen.setControlBounds(0, lbu, ubu);
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

void capturePrimalGuess(GeneratedAcadosSolver& gen,
                        bool slosh,
                        double height_coeff,
                        std::vector<HorizonStateDebug>& states,
                        std::vector<HorizonControlDebug>& controls) {
    states.clear();
    controls.clear();
    states.reserve(static_cast<size_t>(gen.horizonSteps() + 1));
    controls.reserve(static_cast<size_t>(gen.horizonSteps()));
    double x[10] = {0.0};
    double u[3] = {0.0};
    for (int k = 0; k <= gen.horizonSteps(); ++k) {
        std::fill(x, x + 10, 0.0);
        gen.getState(k, x);
        const WarmStartState state = makeWarmStartState(x, slosh);
        const double h_modal = height_coeff * std::hypot(state.eta_x, state.eta_y);
        states.push_back(makeHorizonState(state, h_modal));
        if (k < gen.horizonSteps()) {
            gen.getControl(k, u);
            controls.push_back(makeHorizonControl(makeWarmStartControl(u)));
        }
    }
}

AcadosRawSolution captureRawSolution(GeneratedAcadosSolver& gen) {
    AcadosRawSolution raw;
    raw.horizon_steps = gen.horizonSteps();
    raw.states.reserve(static_cast<std::size_t>(
        (gen.horizonSteps() + 1) * raw.state_width));
    raw.controls.reserve(static_cast<std::size_t>(
        gen.horizonSteps() * raw.control_width));
    double state[10] = {0.0};
    double control[3] = {0.0};
    for (int stage = 0; stage <= gen.horizonSteps(); ++stage) {
        std::fill(state, state + 10, 0.0);
        gen.getState(stage, state);
        raw.states.insert(raw.states.end(), state, state + 10);
        if (stage < gen.horizonSteps()) {
            std::fill(control, control + 3, 0.0);
            gen.getControl(stage, control);
            raw.controls.insert(raw.controls.end(), control, control + 3);
        }
    }
    return raw;
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

void setAcadosWarmStart(GeneratedAcadosSolver& gen,
                        const WarmStartOutput& warm_start,
                        bool slosh) {
    if (!warm_start.valid ||
        warm_start.states.size() < static_cast<size_t>(gen.horizonSteps() + 1) ||
        warm_start.controls.size() < static_cast<size_t>(gen.horizonSteps())) {
        return;
    }
    double x_guess[10];
    double u_guess[3];
    for (int k = 0; k <= gen.horizonSteps(); ++k) {
        fillAcadosState(warm_start.states[k], slosh, x_guess);
        gen.setState(k, x_guess);
        if (k < gen.horizonSteps()) {
            fillAcadosControl(warm_start.controls[k], u_guess);
            gen.setControl(k, u_guess);
        }
    }
}

}  // namespace

struct ContinuousMpccSolverAcados::Impl {
    std::unique_ptr<GeneratedAcadosSolver> primary;
    // Enforce uses a separately generated N=N_l solver.  Keeping a distinct
    // capsule makes it impossible for the trusted liquid window to inherit the
    // baseline geometry tail.
    std::unique_ptr<GeneratedAcadosSolver> phase_rejoin;
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
    std::unique_ptr<GeneratedAcadosSolver> gen(
        new GeneratedAcadosSolver());
    if (!gen->create(
            use_slosh_model_
                ? GeneratedAcadosSolver::Kind::SLOSH
                : GeneratedAcadosSolver::Kind::B0)) {
        SolverConfigureResult result;
        result.status = use_slosh_model_
            ? "ACADOS_SLOSH_CAPSULE_CREATE_FAILED"
            : "ACADOS_B0_CAPSULE_CREATE_FAILED";
        return result;
    }
    impl_->primary = std::move(gen);
    if (use_slosh_model_) {
#ifdef SPMPC_WITH_ACADOS_PHASE_REJOIN
        std::unique_ptr<GeneratedAcadosSolver> phase_gen(
            new GeneratedAcadosSolver());
        if (!phase_gen->create(
                GeneratedAcadosSolver::Kind::PHASE_REJOIN)) {
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
    GeneratedAcadosSolver* gen = phase_rejoin_requested
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
    const int n = gen->horizonSteps();
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
    snapshot.parameter_width = gen->parameterWidth();
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
    if (stage_parameters.parameter_width != gen->parameterWidth()) {
        output.status = "ACADOS_PARAMETER_WIDTH_MISMATCH";
        return false;
    }
    snapshot.parameter_names = stage_parameters.parameter_names;
    snapshot.stage_parameters = stage_parameters.values;
    for (int stage = 0; stage <= n; ++stage) {
        if (!gen->updateParameters(stage, stage_parameters.stageData(stage))) {
            output.status = "ACADOS_PARAMETER_UPDATE_FAILED";
            return false;
        }
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

    if (have_previous_solution_) {
        copyWarmStartForSnapshot(
            previous_warm_start_solution_, c_h,
            snapshot.previous_solution_states,
            snapshot.previous_solution_controls);
    }

    WarmStartPolicyInput warm_start_input;
    warm_start_input.solver_input = &input;
    warm_start_input.reference = &reference;
    warm_start_input.spline = &spline;
    warm_start_input.params = &params_;
    warm_start_input.slosh_dynamics = &slosh_dyn_;
    warm_start_input.generator = warm_start_generator_.get();
    warm_start_input.previous_solution = have_previous_solution_
        ? &previous_warm_start_solution_ : nullptr;
    warm_start_input.progress_s = s0;
    warm_start_input.reference_length = len;
    warm_start_input.horizon_steps = n;
    warm_start_input.slosh_enabled = slosh;
    warm_start_input.have_previous_control = have_u_prev_;
    warm_start_input.previous_control = {{
        u_prev_[0], u_prev_[1], u_prev_[2]}};
    WarmStartPolicyDecision warm_start_decision =
        WarmStartPolicy::select(warm_start_input);
    WarmStartOutput& warm_start = warm_start_decision.warm_start;
    if (warm_start_decision.applied) {
        setAcadosWarmStart(*gen, warm_start, slosh);
    }
    snapshot.warm_start_requested = warm_start_decision.requested;
    snapshot.warm_start_applied = warm_start_decision.applied;
    snapshot.warm_start_source = warm_start_decision.source;
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

    output.solver_time_ms = gen->solveTimeSec() * 1000.0;
    output.first_shot_debug.status_code = static_cast<double>(status);
    if (status != 0) {
        snapshot.solver_status = "ACADOS_SOLVE_FAILED_" + std::to_string(status);
        output.success = false;
        output.status = "ACADOS_SOLVE_FAILED_" + std::to_string(status);
        output.cmd_v = 0.0;
        output.cmd_omega = 0.0;
        return false;
    }

    AcadosRawSolution raw_solution = captureRawSolution(*gen);
    AcadosSolutionDecoderInput decoder_input;
    decoder_input.raw_solution = &raw_solution;
    decoder_input.solver_input = &input;
    decoder_input.reference = &reference;
    decoder_input.params = &params_;
    decoder_input.variant = &variant_;
    for (int index = 0; index < 4; ++index) {
        decoder_input.reference_x_coeffs[static_cast<std::size_t>(index)] =
            cx(index);
        decoder_input.reference_y_coeffs[static_cast<std::size_t>(index)] =
            cy(index);
    }
    decoder_input.contour_error_ref = e_c_ref;
    decoder_input.lag_error_ref = e_l_ref;
    decoder_input.effective_v_ref = v_ref;
    decoder_input.height_coeff = c_h;
    decoder_input.eta_ref = eta_ref;
    decoder_input.eta_dot_ref = eta_dot_ref;
    decoder_input.slosh_enabled = slosh;
    decoder_input.have_previous_control = have_u_prev_;
    decoder_input.previous_control = {{
        u_prev_[0], u_prev_[1], u_prev_[2]}};

    AcadosSolutionDecodeResult decoded =
        AcadosSolutionDecoder::decode(decoder_input, output);
    if (!decoded.valid) {
        output.success = false;
        output.status = "ACADOS_SOLUTION_DECODE_FAILED_" + decoded.status;
        snapshot.solver_status = output.status;
        output.cmd_v = 0.0;
        output.cmd_omega = 0.0;
        return false;
    }

    u_prev_[0] = decoded.first_control[0];
    u_prev_[1] = decoded.first_control[1];
    u_prev_[2] = decoded.first_control[2];
    have_u_prev_ = true;
    previous_warm_start_solution_ = std::move(decoded.solved_warm_start);
    have_previous_solution_ = previous_warm_start_solution_.valid;
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
