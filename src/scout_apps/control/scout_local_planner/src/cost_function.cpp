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
    
    double cost = 0.0;
    cost += params_.Q_el * e_l * e_l;
    cost += params_.Q_ec * e_c * e_c;
    cost += params_.Q_etheta * e_theta * e_theta;
    cost += params_.Q_v * (v - ref.v_ref) * (v - ref.v_ref);
    
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
    Q_contrib(StateIndex::E_L, StateIndex::E_L) = params_.Q_el;
    Q_contrib(StateIndex::E_C, StateIndex::E_C) = params_.Q_ec;
    Q_contrib(StateIndex::E_THETA, StateIndex::E_THETA) = params_.Q_etheta;
    Q_contrib(StateIndex::V, StateIndex::V) = params_.Q_v;
    
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
    double alpha = u(ControlIndex::ANG_ACC);
    
    return params_.R_a * a * a + params_.R_alpha * alpha * alpha;
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
    R_contrib(ControlIndex::ANG_ACC, ControlIndex::ANG_ACC) = params_.R_alpha;
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
    double dalpha = u(ControlIndex::ANG_ACC) - u_prev_(ControlIndex::ANG_ACC);
    
    return params_.R_da * da * da + params_.R_dalpha * dalpha * dalpha;
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
    
    // 控制变化率代价需要在 QP 中特殊处理
    // 这里只返回对角元素，实际的耦合项在 buildQPCost 中处理
    R_contrib(ControlIndex::A, ControlIndex::A) = params_.R_da;
    R_contrib(ControlIndex::ANG_ACC, ControlIndex::ANG_ACC) = params_.R_dalpha;
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
        }
        
        // 填充 H 矩阵（状态部分）
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nx; ++j) {
                if (std::abs(Q_total(i, j)) > 1e-10) {
                    triplets.emplace_back(x_idx + i, x_idx + j, Q_total(i, j));
                }
            }
        }
        
        // 填充 g 向量（状态部分）
        g.segment(x_idx, nx) += q_total;
        
        // 控制部分（最后一步没有控制）
        if (k < N) {
            for (int i = 0; i < nu; ++i) {
                for (int j = 0; j < nu; ++j) {
                    if (std::abs(R_total(i, j)) > 1e-10) {
                        triplets.emplace_back(u_idx + i, u_idx + j, R_total(i, j));
                    }
                }
            }
            g.segment(u_idx, nu) += r_total;
        }
    }
    
    H.resize(nz, nz);
    H.setFromTriplets(triplets.begin(), triplets.end());
}

void CostFunction::setPreviousControl(const ControlVector& u_prev) {
    for (auto& term : cost_terms_) {
        auto rate_cost = std::dynamic_pointer_cast<ControlRateCost>(term);
        if (rate_cost) {
            rate_cost->setPreviousControl(u_prev);
        }
    }
}

}  // namespace scout_local_planner
