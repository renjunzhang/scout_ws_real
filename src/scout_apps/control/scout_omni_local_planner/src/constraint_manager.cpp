/**
 * @file constraint_manager.cpp
 * @brief 全向轮约束管理器实现
 * 
 * 与差速版本的关键区别：
 * - 状态约束：v_x 和 v_y 两个维度
 * - 控制约束：a_x, a_y, ω 三个维度
 */

#include "scout_omni_local_planner/constraint_manager.h"
#include <algorithm>
#include <cmath>

namespace scout_omni_local_planner {

//==============================================================================
// StateBoundsConstraint (约束 v_x 和 v_y)
//==============================================================================

StateBoundsConstraint::StateBoundsConstraint(const VehicleParams& params)
    : params_(params) {}

Eigen::VectorXd StateBoundsConstraint::evaluate(
    const StateVector& x,
    const ControlVector& u) const {
    
    Eigen::VectorXd c(2);
    c(0) = x(StateIndex::V_X);
    c(1) = x(StateIndex::V_Y);
    return c;
}

Eigen::VectorXd StateBoundsConstraint::lowerBound() const {
    Eigen::VectorXd l(2);
    l(0) = params_.vx_min;
    l(1) = -params_.vy_max;
    return l;
}

Eigen::VectorXd StateBoundsConstraint::upperBound() const {
    Eigen::VectorXd u(2);
    u(0) = params_.vx_max;
    u(1) = params_.vy_max;
    return u;
}

//==============================================================================
// ControlBoundsConstraint (约束 a_x, a_y, ω)
//==============================================================================

ControlBoundsConstraint::ControlBoundsConstraint(const VehicleParams& params)
    : params_(params) {}

Eigen::VectorXd ControlBoundsConstraint::evaluate(
    const StateVector& x,
    const ControlVector& u) const {
    
    Eigen::VectorXd c(3);
    c(0) = u(ControlIndex::A_X);
    c(1) = u(ControlIndex::A_Y);
    c(2) = u(ControlIndex::OMEGA);
    return c;
}

Eigen::VectorXd ControlBoundsConstraint::lowerBound() const {
    Eigen::VectorXd l(3);
    l(0) = -params_.ax_max;
    l(1) = -params_.ay_max;
    l(2) = -params_.omega_max;
    return l;
}

Eigen::VectorXd ControlBoundsConstraint::upperBound() const {
    Eigen::VectorXd u(3);
    u(0) = params_.ax_max;
    u(1) = params_.ay_max;
    u(2) = params_.omega_max;
    return u;
}

//==============================================================================
// ConstraintManager
//==============================================================================

void ConstraintManager::initialize(const VehicleParams& params) {
    params_ = params;
    constraints_.clear();
    
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

int ConstraintManager::totalConstraints(int N) const {
    int num_state_constraints = 0;
    int num_control_constraints = 0;
    for (const auto& constraint : constraints_) {
        if (constraint->name() == "StateBoundsConstraint") {
            num_state_constraints += constraint->numConstraints();
        } else if (constraint->name() == "ControlBoundsConstraint") {
            num_control_constraints += constraint->numConstraints();
        }
    }
    int total = (N + 1) * num_state_constraints + N * num_control_constraints;
    if (enable_omega_rate_) total += N;
    if (enable_accel_rate_) total += N;
    return total;
}

void ConstraintManager::setControlRateConstraints(bool enable_omega,
                                                  bool enable_accel,
                                                  double dt,
                                                  const ControlVector& u_prev) {
    enable_omega_rate_ = enable_omega;
    enable_accel_rate_ = enable_accel;
    dt_ = dt;
    u_prev_ = u_prev;
}

void ConstraintManager::buildQPConstraints(
    int N,
    Eigen::SparseMatrix<double>& A,
    Eigen::VectorXd& l,
    Eigen::VectorXd& u) const {
    
    const int nx = StateIndex::TOTAL_DIM;
    const int nu = ControlIndex::DIM;
    const int nz = N * (nx + nu) + nx;
    
    int num_state_constraints = 0;
    int num_control_constraints = 0;
    
    for (const auto& constraint : constraints_) {
        if (constraint->name() == "StateBoundsConstraint") {
            num_state_constraints += constraint->numConstraints();  // 2: v_x, v_y
        } else if (constraint->name() == "ControlBoundsConstraint") {
            num_control_constraints += constraint->numConstraints();  // 3: a_x, a_y, omega
        }
    }
    
    int total_constraints = (N + 1) * num_state_constraints + N * num_control_constraints;
    if (enable_omega_rate_) total_constraints += N;
    if (enable_accel_rate_) total_constraints += N;
    
    std::vector<Eigen::Triplet<double>> triplets;
    l = Eigen::VectorXd::Zero(total_constraints);
    u = Eigen::VectorXd::Zero(total_constraints);
    
    int constraint_idx = 0;
    
    for (int k = 0; k <= N; ++k) {
        int x_idx = k * (nx + nu);
        int u_idx = x_idx + nx;
        
        for (const auto& constraint : constraints_) {
            if (constraint->name() == "StateBoundsConstraint") {
                // v_x 约束
                triplets.emplace_back(constraint_idx, x_idx + StateIndex::V_X, 1.0);
                // v_y 约束
                triplets.emplace_back(constraint_idx + 1, x_idx + StateIndex::V_Y, 1.0);
                
                Eigen::VectorXd lb = constraint->lowerBound();
                Eigen::VectorXd ub = constraint->upperBound();
                l(constraint_idx) = lb(0);      // vx_min
                l(constraint_idx + 1) = lb(1);  // -vy_max
                u(constraint_idx) = ub(0);       // vx_max
                u(constraint_idx + 1) = ub(1);   // vy_max
                
                constraint_idx += 2;
            }
            else if (constraint->name() == "ControlBoundsConstraint" && k < N) {
                // a_x 约束
                triplets.emplace_back(constraint_idx, u_idx + ControlIndex::A_X, 1.0);
                // a_y 约束
                triplets.emplace_back(constraint_idx + 1, u_idx + ControlIndex::A_Y, 1.0);
                // omega 约束
                triplets.emplace_back(constraint_idx + 2, u_idx + ControlIndex::OMEGA, 1.0);
                
                Eigen::VectorXd lb = constraint->lowerBound();
                Eigen::VectorXd ub = constraint->upperBound();
                l(constraint_idx) = lb(0);
                l(constraint_idx + 1) = lb(1);
                l(constraint_idx + 2) = lb(2);
                u(constraint_idx) = ub(0);
                u(constraint_idx + 1) = ub(1);
                u(constraint_idx + 2) = ub(2);
                
                constraint_idx += 3;
            }
        }
    }

    // 角速度变化率约束
    if (enable_omega_rate_) {
        const double domega_max = params_.alpha_max * dt_;
        for (int k = 0; k < N; ++k) {
            int u_k_idx = k * (nx + nu) + nx;
            if (k == 0) {
                triplets.emplace_back(constraint_idx, u_k_idx + ControlIndex::OMEGA, 1.0);
                l(constraint_idx) = u_prev_(ControlIndex::OMEGA) - domega_max;
                u(constraint_idx) = u_prev_(ControlIndex::OMEGA) + domega_max;
            } else {
                int u_prev_idx = (k - 1) * (nx + nu) + nx;
                triplets.emplace_back(constraint_idx, u_k_idx + ControlIndex::OMEGA, 1.0);
                triplets.emplace_back(constraint_idx, u_prev_idx + ControlIndex::OMEGA, -1.0);
                l(constraint_idx) = -domega_max;
                u(constraint_idx) = domega_max;
            }
            constraint_idx += 1;
        }
    }

    // 纵向加速度变化率约束（Δa_x）
    if (enable_accel_rate_) {
        const double jx = params_.jx_max > 0.0 ? params_.jx_max : params_.ax_max;
        const double dax_max = jx * dt_;
        for (int k = 0; k < N; ++k) {
            int u_k_idx = k * (nx + nu) + nx;
            if (k == 0) {
                triplets.emplace_back(constraint_idx, u_k_idx + ControlIndex::A_X, 1.0);
                l(constraint_idx) = u_prev_(ControlIndex::A_X) - dax_max;
                u(constraint_idx) = u_prev_(ControlIndex::A_X) + dax_max;
            } else {
                int u_prev_idx = (k - 1) * (nx + nu) + nx;
                triplets.emplace_back(constraint_idx, u_k_idx + ControlIndex::A_X, 1.0);
                triplets.emplace_back(constraint_idx, u_prev_idx + ControlIndex::A_X, -1.0);
                l(constraint_idx) = -dax_max;
                u(constraint_idx) = dax_max;
            }
            constraint_idx += 1;
        }
    }
    
    A.resize(total_constraints, nz);
    A.setFromTriplets(triplets.begin(), triplets.end());
}

}  // namespace scout_omni_local_planner
