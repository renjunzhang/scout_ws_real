#include "spmpc_local_planner/solvers/continuous_mpcc_solver_acados.h"

#ifdef SPMPC_WITH_ACADOS

#include "spmpc_local_planner/reference/progress_projector.h"
#include "spmpc_local_planner/reference/reference_spline.h"

#include "acados_solver_spmpc_b0.h"
#include "acados_solver_spmpc_slosh.h"
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
    W_A, W_OMEGA, W_VS,
    W_DU_A, W_DU_OMEGA, W_DU_VS,
    A_PREV, OMEGA_PREV, VS_PREV,
    E_C_REF, E_L_REF,
    // 以下仅 slosh 模型（接在 B0 的 22 个之后）
    TWO_ZETA_OMEGA_N, OMEGA_N_SQ, KAPPA_X, KAPPA_Y,
    ETA_REF, ETA_DOT_REF, W_SLOSH_ETA, W_SLOSH_ETA_DOT,
    PARAM_MAX,
};

// 参数布局契约：与 scripts/acados/spmpc_acados_model.py（→生成器 NP 宏）绑死，漂移即编译失败。
static_assert(E_L_REF + 1 == SPMPC_B0_NP, "B0 参数布局与生成的 spmpc_b0 求解器不一致");
static_assert(W_SLOSH_ETA_DOT + 1 == SPMPC_SLOSH_NP, "slosh 参数布局与生成的 spmpc_slosh 求解器不一致");

// 统一封装两个生成求解器（b0 5维 / slosh 9维），把前缀相关调用收敛到一处。
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
            auto* c = spmpc_slosh_acados_create_capsule();
            if (c == nullptr || spmpc_slosh_acados_create(c) != 0) {
                if (c) spmpc_slosh_acados_free_capsule(c);
                return false;
            }
            capsule = c; nx = SPMPC_SLOSH_NX; nu = SPMPC_SLOSH_NU; np = SPMPC_SLOSH_NP; n_horizon = SPMPC_SLOSH_N;
        }
        return true;
    }
    void destroy() {
        if (capsule == nullptr) return;
        if (kind == B0) {
            spmpc_b0_acados_free(static_cast<spmpc_b0_solver_capsule*>(capsule));
            spmpc_b0_acados_free_capsule(static_cast<spmpc_b0_solver_capsule*>(capsule));
        } else {
            spmpc_slosh_acados_free(static_cast<spmpc_slosh_solver_capsule*>(capsule));
            spmpc_slosh_acados_free_capsule(static_cast<spmpc_slosh_solver_capsule*>(capsule));
        }
        capsule = nullptr;
    }
    void update_params(int stage, double* p) {
        if (kind == B0) spmpc_b0_acados_update_params(static_cast<spmpc_b0_solver_capsule*>(capsule), stage, p, np);
        else spmpc_slosh_acados_update_params(static_cast<spmpc_slosh_solver_capsule*>(capsule), stage, p, np);
    }
    int solve() {
        return kind == B0 ? spmpc_b0_acados_solve(static_cast<spmpc_b0_solver_capsule*>(capsule))
                          : spmpc_slosh_acados_solve(static_cast<spmpc_slosh_solver_capsule*>(capsule));
    }
    ocp_nlp_config* config() {
        return kind == B0 ? spmpc_b0_acados_get_nlp_config(static_cast<spmpc_b0_solver_capsule*>(capsule))
                          : spmpc_slosh_acados_get_nlp_config(static_cast<spmpc_slosh_solver_capsule*>(capsule));
    }
    ocp_nlp_dims* dims() {
        return kind == B0 ? spmpc_b0_acados_get_nlp_dims(static_cast<spmpc_b0_solver_capsule*>(capsule))
                          : spmpc_slosh_acados_get_nlp_dims(static_cast<spmpc_slosh_solver_capsule*>(capsule));
    }
    ocp_nlp_in* in() {
        return kind == B0 ? spmpc_b0_acados_get_nlp_in(static_cast<spmpc_b0_solver_capsule*>(capsule))
                          : spmpc_slosh_acados_get_nlp_in(static_cast<spmpc_slosh_solver_capsule*>(capsule));
    }
    ocp_nlp_out* out() {
        return kind == B0 ? spmpc_b0_acados_get_nlp_out(static_cast<spmpc_b0_solver_capsule*>(capsule))
                          : spmpc_slosh_acados_get_nlp_out(static_cast<spmpc_slosh_solver_capsule*>(capsule));
    }
    ocp_nlp_solver* solver() {
        return kind == B0 ? spmpc_b0_acados_get_nlp_solver(static_cast<spmpc_b0_solver_capsule*>(capsule))
                          : spmpc_slosh_acados_get_nlp_solver(static_cast<spmpc_slosh_solver_capsule*>(capsule));
    }
};

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
    have_u_prev_ = false;
    slosh_dyn_.configure(params.slosh);

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
    const auto proj = projector.project(reference, input.robot.x, input.robot.y, input.min_progress_s);
    if (!proj.valid) {
        output.status = "PROJECTION_FAILED";
        return false;
    }

    const double len = reference.length();
    const double s0 = proj.s;
    output.progress_s = len > 1e-6 ? s0 / len : 0.0;
    output.progress_abs_s = s0;

    if (std::max(0.0, len - s0) < params_.goal_tolerance) {
        output.success = true;
        output.status = "GOAL_REACHED";
        return true;
    }

    // 用求解器固化的 N（codegen 时确定），而非 input.horizon_steps，避免二者不一致导致越界。
    const int n = gen->n_horizon;
    const double Tf = input.dt * n;

    ReferenceSpline spline;
    spline.build(reference);
    const double s_end = std::min(len, s0 + params_.v_max * Tf);
    Eigen::Vector4d cx, cy;
    fitReferencePolynomials(spline, s0, s_end, cx, cy);

    const double e_c_ref = std::max(1e-3, 0.5 * params_.corridor_width);
    const double e_l_ref = std::max(0.1, params_.v_max * input.dt);

    // slosh 物理：取自同一套 slosh_dynamics 核（§4.3），κ=1（与 slosh_models 的单位输入增益一致）。
    double c_h = 1.0, eta_ref = 1.0, eta_dot_ref = 1.0;
    double two_zeta_omega_n = 0.0, omega_n_sq = 0.0;
    if (slosh && slosh_dyn_.configured()) {
        const double omega_n = slosh_dyn_.omegaN();
        const double zeta = params_.slosh.damping_ratio;
        const double h_ref = std::max(1e-4, params_.slosh.slosh_height_ref);
        c_h = std::max(1e-6, slosh_dyn_.heightCoeff());
        two_zeta_omega_n = 2.0 * zeta * omega_n;
        omega_n_sq = omega_n * omega_n;
        eta_ref = std::max(1e-6, h_ref / c_h);      // 使 ||eta||/eta_ref == (c_h||eta||)/h_ref，与 primitive 一致
        eta_dot_ref = std::max(1e-6, omega_n * h_ref);
    }

    double p[PARAM_MAX];
    for (int i = 0; i < PARAM_MAX; ++i) p[i] = 0.0;
    p[RX0] = cx(0); p[RX1] = cx(1); p[RX2] = cx(2); p[RX3] = cx(3);
    p[RY0] = cy(0); p[RY1] = cy(1); p[RY2] = cy(2); p[RY3] = cy(3);
    p[W_CONTOUR] = variant_.w_contour;
    p[W_LAG] = variant_.w_lag;
    p[W_PROGRESS] = variant_.w_progress;
    p[W_A] = variant_.w_control;
    p[W_OMEGA] = variant_.w_control;
    p[W_VS] = 0.0;
    p[E_C_REF] = e_c_ref;
    p[E_L_REF] = e_l_ref;
    if (slosh) {
        p[TWO_ZETA_OMEGA_N] = two_zeta_omega_n;
        p[OMEGA_N_SQ] = omega_n_sq;
        p[KAPPA_X] = 1.0;
        p[KAPPA_Y] = 1.0;
        p[ETA_REF] = eta_ref;
        p[ETA_DOT_REF] = eta_dot_ref;
        p[W_SLOSH_ETA] = variant_.w_slosh;
        p[W_SLOSH_ETA_DOT] = variant_.w_slosh * params_.slosh.slosh_eta_dot_ratio;
    }

    for (int stage = 0; stage <= n; ++stage) {
        if (stage == 0 && have_u_prev_) {
            p[W_DU_A] = variant_.w_smooth;
            p[W_DU_OMEGA] = variant_.w_smooth;
            p[W_DU_VS] = variant_.w_smooth;
            p[A_PREV] = u_prev_[0];
            p[OMEGA_PREV] = u_prev_[1];
            p[VS_PREV] = u_prev_[2];
        } else {
            p[W_DU_A] = 0.0; p[W_DU_OMEGA] = 0.0; p[W_DU_VS] = 0.0;
            p[A_PREV] = 0.0; p[OMEGA_PREV] = 0.0; p[VS_PREV] = 0.0;
        }
        gen->update_params(stage, p);
    }

    double x0[9] = {input.robot.x, input.robot.y, input.robot.yaw, input.robot.v, s0, 0, 0, 0, 0};
    if (slosh) {
        x0[5] = input.slosh.eta_x;
        x0[6] = input.slosh.eta_x_dot;
        x0[7] = input.slosh.eta_y;
        x0[8] = input.slosh.eta_y_dot;
    }
    ocp_nlp_config* cfg = gen->config();
    ocp_nlp_dims* dims = gen->dims();
    ocp_nlp_in* nlp_in = gen->in();
    ocp_nlp_out* nlp_out = gen->out();
    ocp_nlp_constraints_model_set(cfg, dims, nlp_in, nlp_out, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(cfg, dims, nlp_in, nlp_out, 0, "ubx", x0);

    const int status = gen->solve();

    double time_tot = 0.0;
    ocp_nlp_get(gen->solver(), "time_tot", &time_tot);
    output.solver_time_ms = time_tot * 1000.0;

    // 读轨迹 + 诊断量（contour/lag/slosh/控制），按 §11.5 对齐 primitive。
    const double inv_n = 1.0 / static_cast<double>(std::max(1, n));
    output.trajectory.reserve(n + 1);
    std::vector<double> heights;
    heights.reserve(n + 1);
    double xk[9];
    for (int k = 0; k <= n; ++k) {
        ocp_nlp_out_get(cfg, dims, nlp_out, k, "x", xk);
        TrajectoryPoint pt;
        pt.x = xk[0]; pt.y = xk[1]; pt.yaw = xk[2]; pt.v = xk[3]; pt.s = xk[4];
        output.trajectory.push_back(pt);

        const double xref = polyEval(cx, pt.s);
        const double yref = polyEval(cy, pt.s);
        const double phi = std::atan2(polyDeriv(cy, pt.s), polyDeriv(cx, pt.s));
        const double e_c = std::sin(phi) * (pt.x - xref) - std::cos(phi) * (pt.y - yref);
        const double e_l = -std::cos(phi) * (pt.x - xref) - std::sin(phi) * (pt.y - yref);
        output.cost.J_contour += variant_.w_contour * (e_c / e_c_ref) * (e_c / e_c_ref) * inv_n;
        output.cost.J_lag += variant_.w_lag * (e_l / e_l_ref) * (e_l / e_l_ref) * inv_n;

        if (slosh) {
            const double ex = xk[5], exd = xk[6], ey = xk[7], eyd = xk[8];
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
            output.cost.J_slosh_eta += variant_.w_slosh * (eta_norm / eta_ref) * (eta_norm / eta_ref) * inv_n;
            output.cost.J_slosh_eta_dot += variant_.w_slosh * params_.slosh.slosh_eta_dot_ratio *
                (eta_dot_norm / eta_dot_ref) * (eta_dot_norm / eta_dot_ref) * inv_n;
        }
    }

    const double a_ref = std::max(0.1, params_.a_max);
    const double omega_ref = std::max(1e-3, params_.omega_max);
    const double vs_ref = std::max(0.1, params_.v_max);
    double uk[3], u0[3] = {0, 0, 0};
    for (int k = 0; k < n; ++k) {
        ocp_nlp_out_get(cfg, dims, nlp_out, k, "u", uk);
        if (k == 0) { u0[0] = uk[0]; u0[1] = uk[1]; u0[2] = uk[2]; }
        const double vn = uk[0] / a_ref;
        const double wn = uk[1] / omega_ref;
        output.cost.J_control += variant_.w_control * (vn * vn + wn * wn) * inv_n;

        // 与当前 acados external cost 保持一致：w_du_* 只在 stage 0 置非零，
        // 表示跨控制周期第一帧连续性；horizon 内 Δu_k 尚未进入 OCP 代价。
        if (k == 0 && have_u_prev_) {
            const double da = (uk[0] - u_prev_[0]) / a_ref;
            const double dw = (uk[1] - u_prev_[1]) / omega_ref;
            const double dvs = (uk[2] - u_prev_[2]) / vs_ref;
            output.cost.J_smooth += variant_.w_smooth * (da * da + dw * dw + dvs * dvs) * inv_n;
        }
    }
    output.cost.J_progress = -variant_.w_progress *
        std::max(0.0, output.trajectory.back().s - output.trajectory.front().s);

    if (!heights.empty()) {
        std::vector<double> sorted = heights;
        std::sort(sorted.begin(), sorted.end());
        const size_t idx = std::min(sorted.size() - 1,
            static_cast<size_t>(std::floor(0.95 * (sorted.size() - 1))));
        output.slosh_summary.h_p95_pred = sorted[idx];
    }

    if (status != 0) {
        output.success = false;
        output.status = "ACADOS_SOLVE_FAILED_" + std::to_string(status);
        output.cmd_v = 0.0;
        output.cmd_omega = 0.0;
        return false;
    }

    output.cmd_v = output.trajectory.size() > 1 ? output.trajectory[1].v : input.robot.v;
    output.cmd_omega = u0[1];
    u_prev_[0] = u0[0];
    u_prev_[1] = u0[1];
    u_prev_[2] = u0[2];
    have_u_prev_ = true;

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
