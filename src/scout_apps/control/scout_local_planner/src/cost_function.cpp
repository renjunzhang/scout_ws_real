/**
 * @file cost_function.cpp
 * @brief MPC 代价函数实现
 */

#include "scout_local_planner/cost_function.h"
#include <algorithm>

namespace scout_local_planner {

//==============================================================================
// StateTrackingCost
//==============================================================================

StateTrackingCost::StateTrackingCost(const MPCParams& params)
    : params_(params) {}

double StateTrackingCost::evaluate(
    const StateVector& x,
    const ControlVector& u,
    const ReferencePoint& ref,
    int k) const {
    
    double e_l = x(StateIndex::E_L);
    double e_c = x(StateIndex::E_C);
    double e_theta = x(StateIndex::E_THETA);
    double v = x(StateIndex::V);
    double omega = u(ControlIndex::OMEGA);  // ω 现在是控制量！
    
    double cost = 0.0;
    if (params_.use_contour_lag) {
        cost += params_.Q_lag * e_l * e_l;
        cost += params_.Q_contour * e_c * e_c;
    } else {
        cost += params_.Q_el * e_l * e_l;
        cost += params_.Q_ec * e_c * e_c;
    }
    cost += params_.Q_etheta * e_theta * e_theta;
    cost += params_.Q_v * (v - ref.v_ref) * (v - ref.v_ref);
    if (params_.enable_omega_ff) {
        double omega_ref = ref.v_ref * ref.kappa;
        cost += params_.Q_omega_ff * (omega - omega_ref) * (omega - omega_ref);
    }
    
    return cost;
}

void StateTrackingCost::getQuadraticCost(
    int k, int N,
    Eigen::MatrixXd& Q_contrib,
    Eigen::MatrixXd& R_contrib,
    Eigen::VectorXd& q_contrib,
    Eigen::VectorXd& r_contrib) const {
    
    const int nx = StateIndex::TOTAL_DIM;
    const int nu = ControlIndex::DIM;
    
    Q_contrib = Eigen::MatrixXd::Zero(nx, nx);
    R_contrib = Eigen::MatrixXd::Zero(nu, nu);
    q_contrib = Eigen::VectorXd::Zero(nx);
    r_contrib = Eigen::VectorXd::Zero(nu);
    
    // 状态权重矩阵
    if (params_.use_contour_lag) {
        Q_contrib(StateIndex::E_L, StateIndex::E_L) = params_.Q_lag;
        Q_contrib(StateIndex::E_C, StateIndex::E_C) = params_.Q_contour;
    } else {
        Q_contrib(StateIndex::E_L, StateIndex::E_L) = params_.Q_el;
        Q_contrib(StateIndex::E_C, StateIndex::E_C) = params_.Q_ec;
    }
    Q_contrib(StateIndex::E_THETA, StateIndex::E_THETA) = params_.Q_etheta;
    Q_contrib(StateIndex::V, StateIndex::V) = params_.Q_v;
    // 注意：omega_ff 现在应用到控制量，在 ControlCost 中处理
    
    // 注意：v_ref 的线性项会在 buildQPCost 中根据 refs 添加
    // q_contrib(StateIndex::V) = -2 * params_.Q_v * v_ref;
}

//==============================================================================
// ControlCost
//==============================================================================

ControlCost::ControlCost(const MPCParams& params)
    : params_(params) {}

double ControlCost::evaluate(
    const StateVector& x,
    const ControlVector& u,
    const ReferencePoint& ref,
    int k) const {
    
    double a = u(ControlIndex::A);
    double omega = u(ControlIndex::OMEGA);
    
    return params_.R_a * a * a + params_.R_omega * omega * omega;
}

void ControlCost::getQuadraticCost(
    int k, int N,
    Eigen::MatrixXd& Q_contrib,
    Eigen::MatrixXd& R_contrib,
    Eigen::VectorXd& q_contrib,
    Eigen::VectorXd& r_contrib) const {
    
    const int nx = StateIndex::TOTAL_DIM;
    const int nu = ControlIndex::DIM;
    
    Q_contrib = Eigen::MatrixXd::Zero(nx, nx);
    R_contrib = Eigen::MatrixXd::Zero(nu, nu);
    q_contrib = Eigen::VectorXd::Zero(nx);
    r_contrib = Eigen::VectorXd::Zero(nu);
    
    R_contrib(ControlIndex::A, ControlIndex::A) = params_.R_a;
    R_contrib(ControlIndex::OMEGA, ControlIndex::OMEGA) = params_.R_omega;
}

//==============================================================================
// ControlRateCost
//==============================================================================

ControlRateCost::ControlRateCost(const MPCParams& params)
    : params_(params) {}

double ControlRateCost::evaluate(
    const StateVector& x,
    const ControlVector& u,
    const ReferencePoint& ref,
    int k) const {
    
    double da = u(ControlIndex::A) - u_prev_(ControlIndex::A);
    double domega = u(ControlIndex::OMEGA) - u_prev_(ControlIndex::OMEGA);
    
    return params_.R_da * da * da + params_.R_domega * domega * domega;
}

void ControlRateCost::getQuadraticCost(
    int k, int N,
    Eigen::MatrixXd& Q_contrib,
    Eigen::MatrixXd& R_contrib,
    Eigen::VectorXd& q_contrib,
    Eigen::VectorXd& r_contrib) const {
    
    const int nx = StateIndex::TOTAL_DIM;
    const int nu = ControlIndex::DIM;
    
    Q_contrib = Eigen::MatrixXd::Zero(nx, nx);
    R_contrib = Eigen::MatrixXd::Zero(nu, nu);
    q_contrib = Eigen::VectorXd::Zero(nx);
    r_contrib = Eigen::VectorXd::Zero(nu);
    
    // 控制变化率代价需要跨步耦合，在 buildQPCost 中统一处理
}

//==============================================================================
// CostFunction
//==============================================================================

void CostFunction::initialize(const MPCParams& params) {
    params_ = params;
    cost_terms_.clear();
    
    // 添加默认代价项
    cost_terms_.push_back(std::make_shared<StateTrackingCost>(params));
    cost_terms_.push_back(std::make_shared<ControlCost>(params));
    cost_terms_.push_back(std::make_shared<ControlRateCost>(params));
}

void CostFunction::addCostTerm(CostTermPtr term) {
    cost_terms_.push_back(term);
}

void CostFunction::removeCostTerm(const std::string& name) {
    cost_terms_.erase(
        std::remove_if(cost_terms_.begin(), cost_terms_.end(),
            [&name](const CostTermPtr& term) {
                return term->name() == name;
            }),
        cost_terms_.end());
}

double CostFunction::computeTotalCost(
    const std::vector<StateVector>& x_traj,
    const std::vector<ControlVector>& u_traj,
    const std::vector<ReferencePoint>& refs) const {
    
    double total = 0.0;
    int N = static_cast<int>(u_traj.size());
    
    for (int k = 0; k < N; ++k) {
        for (const auto& term : cost_terms_) {
            total += term->evaluate(x_traj[k], u_traj[k], refs[k], k);
        }
    }
    
    return total;
}

void CostFunction::buildQPCost(
    int N,
    const std::vector<ReferencePoint>& refs,
    Eigen::SparseMatrix<double>& H,
    Eigen::VectorXd& g) const {
    
    const int nx = StateIndex::TOTAL_DIM;
    const int nu = ControlIndex::DIM;
    
    // 决策变量：z = [x_0, u_0, x_1, u_1, ..., x_{N-1}, u_{N-1}, x_N]
    // 维度：N * (nx + nu) + nx
    const int nz = N * (nx + nu) + nx;
    
    // 构建稀疏 H 矩阵
    std::vector<Eigen::Triplet<double>> triplets;
    g = Eigen::VectorXd::Zero(nz);
    
    auto add_upper_triplet = [&triplets](int row, int col, double value) {
        if (row <= col) {
            triplets.emplace_back(row, col, value);
        } else {
            triplets.emplace_back(col, row, value);
        }
    };

    // 对每个时间步
    for (int k = 0; k <= N; ++k) {
        int x_idx = k * (nx + nu);  // x_k 在 z 中的起始索引
        int u_idx = x_idx + nx;      // u_k 在 z 中的起始索引
        
        Eigen::MatrixXd Q_total = Eigen::MatrixXd::Zero(nx, nx);
        Eigen::MatrixXd R_total = Eigen::MatrixXd::Zero(nu, nu);
        Eigen::VectorXd q_total = Eigen::VectorXd::Zero(nx);
        Eigen::VectorXd r_total = Eigen::VectorXd::Zero(nu);
        
        // 汇总所有代价项
        for (const auto& term : cost_terms_) {
            Eigen::MatrixXd Q, R;
            Eigen::VectorXd q, r;
            term->getQuadraticCost(k, N, Q, R, q, r);
            Q_total += Q;
            R_total += R;
            q_total += q;
            r_total += r;
        }
        
        // 添加 v_ref 的线性项
        if (k < static_cast<int>(refs.size())) {
            q_total(StateIndex::V) -= 2.0 * params_.Q_v * refs[k].v_ref;
            // omega_ff: Q_omega_ff*(ω - ω_ref)^2 展开 → 二次项 + 线性项
            if (params_.enable_omega_ff && k < N) {
                double omega_ref = refs[k].v_ref * refs[k].kappa;
                R_total(ControlIndex::OMEGA, ControlIndex::OMEGA) += params_.Q_omega_ff;
                r_total(ControlIndex::OMEGA) -= 2.0 * params_.Q_omega_ff * omega_ref;
            }
        }

        // 渐进式终端权重：最后 ramp_steps 步线性递增到 terminal_factor
        {
            const int ramp_steps = std::max(1, params_.terminal_ramp_steps);
            const int ramp_start = N - ramp_steps;  // 开始递增的时间步
            if (k >= ramp_start && k <= N) {
                // alpha ∈ (0, 1]，k == ramp_start 时 alpha = 1/ramp_steps，k == N 时 alpha = 1
                double alpha = static_cast<double>(k - ramp_start + 1)
                             / static_cast<double>(ramp_steps + 1);
                if (params_.terminal_factor_ec > 0.0) {
                    double factor = 1.0 + alpha * (params_.terminal_factor_ec - 1.0);
                    Q_total(StateIndex::E_C, StateIndex::E_C) *= factor;
                }
                if (params_.terminal_factor_etheta > 0.0) {
                    double factor = 1.0 + alpha * (params_.terminal_factor_etheta - 1.0);
                    Q_total(StateIndex::E_THETA, StateIndex::E_THETA) *= factor;
                }
                if (params_.terminal_factor_v > 0.0) {
                    double factor = 1.0 + alpha * (params_.terminal_factor_v - 1.0);
                    Q_total(StateIndex::V, StateIndex::V) *= factor;
                    q_total(StateIndex::V) *= factor;
                }
            }
        }
        
        // 填充 H 矩阵（状态部分）
        // OSQP 目标函数为 min 1/2 z'Pz + q'z
        // 代价 Q*e^2 展开为 1/2*(2Q)*e^2 + 0*e → P 对角需要 2*Q
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nx; ++j) {
                add_upper_triplet(x_idx + i, x_idx + j, 2.0 * Q_total(i, j));
            }
        }
        
        // 填充 g 向量（状态部分）
        g.segment(x_idx, nx) += q_total;
        
        // 控制部分（最后一步没有控制）
        if (k < N) {
            for (int i = 0; i < nu; ++i) {
                for (int j = 0; j < nu; ++j) {
                    add_upper_triplet(u_idx + i, u_idx + j, 2.0 * R_total(i, j));
                }
            }
            g.segment(u_idx, nu) += r_total;
        }
    }

    // 控制变化率代价：跨步耦合项 (u_k - u_{k-1})^2
    if (N > 0 && (params_.R_da > 0.0 || params_.R_domega > 0.0)) {
        // k = 0：与上一时刻控制 u_prev 的差分
        {
            int u_idx = nx;  // k=0 的控制起始索引
            add_upper_triplet(u_idx + ControlIndex::A,
                              u_idx + ControlIndex::A,
                              2.0 * params_.R_da);
            add_upper_triplet(u_idx + ControlIndex::OMEGA,
                              u_idx + ControlIndex::OMEGA,
                              2.0 * params_.R_domega);

            g(u_idx + ControlIndex::A) += -2.0 * params_.R_da * u_prev_(ControlIndex::A);
            g(u_idx + ControlIndex::OMEGA) += -2.0 * params_.R_domega * u_prev_(ControlIndex::OMEGA);
        }

        // k = 1..N-1：相邻控制差分
        for (int k = 1; k < N; ++k) {
            int u_idx = k * (nx + nu) + nx;
            int u_prev_idx = (k - 1) * (nx + nu) + nx;

            add_upper_triplet(u_idx + ControlIndex::A,
                              u_idx + ControlIndex::A,
                              2.0 * params_.R_da);
            add_upper_triplet(u_prev_idx + ControlIndex::A,
                              u_prev_idx + ControlIndex::A,
                              2.0 * params_.R_da);
            add_upper_triplet(u_idx + ControlIndex::A,
                              u_prev_idx + ControlIndex::A,
                              -2.0 * params_.R_da);

            add_upper_triplet(u_idx + ControlIndex::OMEGA,
                              u_idx + ControlIndex::OMEGA,
                              2.0 * params_.R_domega);
            add_upper_triplet(u_prev_idx + ControlIndex::OMEGA,
                              u_prev_idx + ControlIndex::OMEGA,
                              2.0 * params_.R_domega);
            add_upper_triplet(u_idx + ControlIndex::OMEGA,
                              u_prev_idx + ControlIndex::OMEGA,
                              -2.0 * params_.R_domega);
        }
    }
    
    H.resize(nz, nz);
    H.setFromTriplets(triplets.begin(), triplets.end());
}

void CostFunction::setPreviousControl(const ControlVector& u_prev) {
    u_prev_ = u_prev;
    for (auto& term : cost_terms_) {
        auto rate_cost = std::dynamic_pointer_cast<ControlRateCost>(term);
        if (rate_cost) {
            rate_cost->setPreviousControl(u_prev);
        }
    }
}

}  // namespace scout_local_planner
