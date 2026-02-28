/**
 * @file cost_function.cpp
 * @brief 全向轮 MPC 代价函数实现
 * 
 * 与差速版本的关键区别：
 * - 状态跟踪：Q_vx*(v_x - v_ref)² + Q_vy*(v_y - vy_ref)²
 * - 控制代价：R_ax*a_x² + R_ay*a_y² + R_omega*ω²
 * - 变化率代价：R_dax*Δa_x² + R_day*Δa_y² + R_domega*Δω²
 */

#include "scout_omni_local_planner/cost_function.h"
#include <algorithm>

namespace scout_omni_local_planner {

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
    double vx = x(StateIndex::V_X);
    double vy = x(StateIndex::V_Y);
    double omega = u(ControlIndex::OMEGA);
    
    double cost = 0.0;
    if (params_.use_contour_lag) {
        cost += params_.Q_lag * e_l * e_l;
        cost += params_.Q_contour * e_c * e_c;
    } else {
        cost += params_.Q_el * e_l * e_l;
        cost += params_.Q_ec * e_c * e_c;
    }
    cost += params_.Q_etheta * e_theta * e_theta;
    cost += params_.Q_vx * (vx - ref.v_ref) * (vx - ref.v_ref);
    cost += params_.Q_vy * (vy - ref.vy_ref) * (vy - ref.vy_ref);
    
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
    
    if (params_.use_contour_lag) {
        Q_contrib(StateIndex::E_L, StateIndex::E_L) = params_.Q_lag;
        Q_contrib(StateIndex::E_C, StateIndex::E_C) = params_.Q_contour;
    } else {
        Q_contrib(StateIndex::E_L, StateIndex::E_L) = params_.Q_el;
        Q_contrib(StateIndex::E_C, StateIndex::E_C) = params_.Q_ec;
    }
    Q_contrib(StateIndex::E_THETA, StateIndex::E_THETA) = params_.Q_etheta;
    Q_contrib(StateIndex::V_X, StateIndex::V_X) = params_.Q_vx;
    Q_contrib(StateIndex::V_Y, StateIndex::V_Y) = params_.Q_vy;
    
    // v_ref / vy_ref 的线性项在 buildQPCost 中根据 refs 添加
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
    
    double ax = u(ControlIndex::A_X);
    double ay = u(ControlIndex::A_Y);
    double omega = u(ControlIndex::OMEGA);
    
    return params_.R_ax * ax * ax + params_.R_ay * ay * ay + params_.R_omega * omega * omega;
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
    
    R_contrib(ControlIndex::A_X, ControlIndex::A_X) = params_.R_ax;
    R_contrib(ControlIndex::A_Y, ControlIndex::A_Y) = params_.R_ay;
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
    
    double dax = u(ControlIndex::A_X) - u_prev_(ControlIndex::A_X);
    double day = u(ControlIndex::A_Y) - u_prev_(ControlIndex::A_Y);
    double domega = u(ControlIndex::OMEGA) - u_prev_(ControlIndex::OMEGA);
    
    return params_.R_dax * dax * dax + params_.R_day * day * day + params_.R_domega * domega * domega;
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
    const int nz = N * (nx + nu) + nx;
    
    std::vector<Eigen::Triplet<double>> triplets;
    g = Eigen::VectorXd::Zero(nz);
    
    auto add_upper_triplet = [&triplets](int row, int col, double value) {
        if (row <= col) {
            triplets.emplace_back(row, col, value);
        } else {
            triplets.emplace_back(col, row, value);
        }
    };

    for (int k = 0; k <= N; ++k) {
        int x_idx = k * (nx + nu);
        int u_idx = x_idx + nx;
        
        Eigen::MatrixXd Q_total = Eigen::MatrixXd::Zero(nx, nx);
        Eigen::MatrixXd R_total = Eigen::MatrixXd::Zero(nu, nu);
        Eigen::VectorXd q_total = Eigen::VectorXd::Zero(nx);
        Eigen::VectorXd r_total = Eigen::VectorXd::Zero(nu);
        
        for (const auto& term : cost_terms_) {
            Eigen::MatrixXd Q, R;
            Eigen::VectorXd q, r;
            term->getQuadraticCost(k, N, Q, R, q, r);
            Q_total += Q;
            R_total += R;
            q_total += q;
            r_total += r;
        }
        
        // 添加 v_ref / vy_ref 线性项
        if (k < static_cast<int>(refs.size())) {
            q_total(StateIndex::V_X) -= 2.0 * params_.Q_vx * refs[k].v_ref;
            q_total(StateIndex::V_Y) -= 2.0 * params_.Q_vy * refs[k].vy_ref;
            
            // omega_ff
            if (params_.enable_omega_ff && k < N) {
                double omega_ref = refs[k].v_ref * refs[k].kappa;
                r_total(ControlIndex::OMEGA) -= 2.0 * params_.Q_omega_ff * omega_ref;
            }
        }

        // 终端权重放大
        if (k == N) {
            if (params_.terminal_factor_ec > 0.0) {
                Q_total(StateIndex::E_C, StateIndex::E_C) *= params_.terminal_factor_ec;
            }
            if (params_.terminal_factor_etheta > 0.0) {
                Q_total(StateIndex::E_THETA, StateIndex::E_THETA) *= params_.terminal_factor_etheta;
            }
            if (params_.terminal_factor_vx > 0.0) {
                Q_total(StateIndex::V_X, StateIndex::V_X) *= params_.terminal_factor_vx;
                q_total(StateIndex::V_X) *= params_.terminal_factor_vx;
            }
        }
        
        // 填充 H（状态）
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nx; ++j) {
                add_upper_triplet(x_idx + i, x_idx + j, Q_total(i, j));
            }
        }
        g.segment(x_idx, nx) += q_total;
        
        // 填充 H（控制）
        if (k < N) {
            for (int i = 0; i < nu; ++i) {
                for (int j = 0; j < nu; ++j) {
                    add_upper_triplet(u_idx + i, u_idx + j, R_total(i, j));
                }
            }
            g.segment(u_idx, nu) += r_total;
        }
    }

    // 控制变化率代价：(u_k - u_{k-1})² 跨步耦合
    // 对所有 3 个控制维度独立处理
    if (N > 0) {
        double R_rates[3] = {params_.R_dax, params_.R_day, params_.R_domega};
        
        // k = 0: 与 u_prev 的差分
        {
            int u_idx = nx;
            for (int d = 0; d < nu; ++d) {
                if (R_rates[d] <= 0.0) continue;
                add_upper_triplet(u_idx + d, u_idx + d, 2.0 * R_rates[d]);
                g(u_idx + d) += -2.0 * R_rates[d] * u_prev_(d);
            }
        }

        // k = 1..N-1: 相邻控制差分
        for (int k = 1; k < N; ++k) {
            int u_idx = k * (nx + nu) + nx;
            int u_prev_idx = (k - 1) * (nx + nu) + nx;
            
            for (int d = 0; d < nu; ++d) {
                if (R_rates[d] <= 0.0) continue;
                add_upper_triplet(u_idx + d, u_idx + d, 2.0 * R_rates[d]);
                add_upper_triplet(u_prev_idx + d, u_prev_idx + d, 2.0 * R_rates[d]);
                add_upper_triplet(u_idx + d, u_prev_idx + d, -2.0 * R_rates[d]);
            }
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

}  // namespace scout_omni_local_planner
