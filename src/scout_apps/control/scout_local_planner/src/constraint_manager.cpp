/**
 * @file constraint_manager.cpp
 * @brief 约束管理器实现
 */

#include "scout_local_planner/constraint_manager.h"
#include <algorithm>
#include <cmath>

namespace scout_local_planner {

//==============================================================================
// StateBoundsConstraint
//==============================================================================

StateBoundsConstraint::StateBoundsConstraint(const VehicleParams& params)
    : params_(params) {}

Eigen::VectorXd StateBoundsConstraint::evaluate(
    const StateVector& x,
    const ControlVector& u) const {
    
    // 只约束 v（ω 现在是控制量，在 ControlBoundsConstraint 中约束）
    Eigen::VectorXd c(1);
    c(0) = x(StateIndex::V);
    return c;
}

Eigen::VectorXd StateBoundsConstraint::lowerBound() const {
    Eigen::VectorXd l(1);
    l(0) = params_.v_min;
    return l;
}

Eigen::VectorXd StateBoundsConstraint::upperBound() const {
    Eigen::VectorXd u(1);
    u(0) = params_.v_max;
    return u;
}

//==============================================================================
// ControlBoundsConstraint
//==============================================================================

ControlBoundsConstraint::ControlBoundsConstraint(const VehicleParams& params)
    : params_(params) {}

Eigen::VectorXd ControlBoundsConstraint::evaluate(
    const StateVector& x,
    const ControlVector& u) const {
    
    Eigen::VectorXd c(2);
    c(0) = u(ControlIndex::A);
    c(1) = u(ControlIndex::OMEGA);  // ω 现在是控制量
    return c;
}

Eigen::VectorXd ControlBoundsConstraint::lowerBound() const {
    Eigen::VectorXd l(2);
    l(0) = -params_.a_max;
    l(1) = -params_.omega_max;  // 角速度约束
    return l;
}

Eigen::VectorXd ControlBoundsConstraint::upperBound() const {
    Eigen::VectorXd u(2);
    u(0) = params_.a_max;
    u(1) = params_.omega_max;  // 角速度约束
    return u;
}

//==============================================================================
// ConstraintManager
//==============================================================================

void ConstraintManager::initialize(const VehicleParams& params) {
    params_ = params;
    constraints_.clear();
    
    // 添加默认约束
    constraints_.push_back(std::make_shared<StateBoundsConstraint>(params));
    constraints_.push_back(std::make_shared<ControlBoundsConstraint>(params));
}

void ConstraintManager::addConstraint(ConstraintPtr constraint) {
    constraints_.push_back(constraint);
}

void ConstraintManager::removeConstraint(const std::string& name) {
    constraints_.erase(
        std::remove_if(constraints_.begin(), constraints_.end(),
            [&name](const ConstraintPtr& c) {
                return c->name() == name;
            }),
        constraints_.end());
}

bool ConstraintManager::checkConstraints(
    const StateVector& x,
    const ControlVector& u,
    double tolerance) const {
    
    for (const auto& constraint : constraints_) {
        Eigen::VectorXd c = constraint->evaluate(x, u);
        Eigen::VectorXd l = constraint->lowerBound();
        Eigen::VectorXd ub = constraint->upperBound();
        
        for (int i = 0; i < c.size(); ++i) {
            if (c(i) < l(i) - tolerance || c(i) > ub(i) + tolerance) {
                return false;
            }
        }
    }
    return true;
}

int ConstraintManager::totalConstraints() const {
    int total = 0;
    for (const auto& constraint : constraints_) {
        total += constraint->numConstraints();
    }
    return total;
}

void ConstraintManager::buildQPConstraints(
    int N,
    Eigen::SparseMatrix<double>& A,
    Eigen::VectorXd& l,
    Eigen::VectorXd& u) const {
    
    const int nx = StateIndex::TOTAL_DIM;
    const int nu = ControlIndex::DIM;
    
    // 决策变量：z = [x_0, u_0, x_1, u_1, ..., x_{N-1}, u_{N-1}, x_N]
    const int nz = N * (nx + nu) + nx;
    
    // 计算总约束数
    int num_state_constraints = 0;
    int num_control_constraints = 0;
    
    for (const auto& constraint : constraints_) {
        if (constraint->name() == "StateBoundsConstraint") {
            num_state_constraints += constraint->numConstraints();
        } else if (constraint->name() == "ControlBoundsConstraint") {
            num_control_constraints += constraint->numConstraints();
        }
    }
    
    // 状态约束：每个时间步都有
    // 控制约束：只有 N 步（最后一步没有控制）
    int total_constraints = (N + 1) * num_state_constraints + N * num_control_constraints;
    
    std::vector<Eigen::Triplet<double>> triplets;
    l = Eigen::VectorXd::Zero(total_constraints);
    u = Eigen::VectorXd::Zero(total_constraints);
    
    int constraint_idx = 0;
    
    // 对每个时间步构建约束
    for (int k = 0; k <= N; ++k) {
        int x_idx = k * (nx + nu);  // x_k 在 z 中的起始索引
        int u_idx = x_idx + nx;      // u_k 在 z 中的起始索引
        
        for (const auto& constraint : constraints_) {
            if (constraint->name() == "StateBoundsConstraint") {
                // 状态约束：仅约束 v（ω 现在是控制量，不在状态中）
                // c = [v]' = [0,0,0,1] * x (4D state)
                triplets.emplace_back(constraint_idx, x_idx + StateIndex::V, 1.0);
                
                Eigen::VectorXd lb = constraint->lowerBound();
                Eigen::VectorXd ub = constraint->upperBound();
                l(constraint_idx) = lb(0);
                u(constraint_idx) = ub(0);
                
                constraint_idx += 1;
            }
            else if (constraint->name() == "ControlBoundsConstraint" && k < N) {
                // 控制约束：提取 a 和 omega
                triplets.emplace_back(constraint_idx, u_idx + ControlIndex::A, 1.0);
                triplets.emplace_back(constraint_idx + 1, u_idx + ControlIndex::OMEGA, 1.0);
                
                Eigen::VectorXd lb = constraint->lowerBound();
                Eigen::VectorXd ub = constraint->upperBound();
                l(constraint_idx) = lb(0);
                l(constraint_idx + 1) = lb(1);
                u(constraint_idx) = ub(0);
                u(constraint_idx + 1) = ub(1);
                
                constraint_idx += 2;
            }
        }
    }
    
    A.resize(total_constraints, nz);
    A.setFromTriplets(triplets.begin(), triplets.end());
}

}  // namespace scout_local_planner
