/**
 * @file constraint_manager.h
 * @brief 全向轮约束管理器
 * 
 * 管理 MPC 的各种约束：
 * - 状态约束（v_x, v_y 限制）
 * - 控制约束（a_x, a_y, ω 限制）
 * - 控制变化率约束
 * - 液体晃动约束（可选）
 */

#pragma once

#include "scout_omni_local_planner/types.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>
#include <vector>
#include <string>

namespace scout_omni_local_planner {

/**
 * @brief 约束基类
 */
class ConstraintBase {
public:
    virtual ~ConstraintBase() = default;
    virtual std::string name() const = 0;
    virtual int numConstraints() const = 0;
    
    virtual Eigen::VectorXd evaluate(
        const StateVector& x,
        const ControlVector& u) const = 0;
    
    virtual Eigen::VectorXd lowerBound() const = 0;
    virtual Eigen::VectorXd upperBound() const = 0;
    
    virtual bool isSoft() const { return false; }
    virtual double softWeight() const { return 0.0; }
};

using ConstraintPtr = std::shared_ptr<ConstraintBase>;

/**
 * @brief 状态边界约束
 * 
 * v_x_min ≤ v_x ≤ v_x_max
 * -v_y_max ≤ v_y ≤ v_y_max
 */
class StateBoundsConstraint : public ConstraintBase {
public:
    StateBoundsConstraint(const VehicleParams& params);
    
    std::string name() const override { return "StateBoundsConstraint"; }
    int numConstraints() const override { return 2; }  // v_x, v_y
    
    Eigen::VectorXd evaluate(
        const StateVector& x,
        const ControlVector& u) const override;
    
    Eigen::VectorXd lowerBound() const override;
    Eigen::VectorXd upperBound() const override;

private:
    VehicleParams params_;
};

/**
 * @brief 控制边界约束
 * 
 * -a_x_max ≤ a_x ≤ a_x_max
 * -a_y_max ≤ a_y ≤ a_y_max
 * -omega_max ≤ ω ≤ omega_max
 */
class ControlBoundsConstraint : public ConstraintBase {
public:
    ControlBoundsConstraint(const VehicleParams& params);
    
    std::string name() const override { return "ControlBoundsConstraint"; }
    int numConstraints() const override { return 3; }  // a_x, a_y, omega
    
    Eigen::VectorXd evaluate(
        const StateVector& x,
        const ControlVector& u) const override;
    
    Eigen::VectorXd lowerBound() const override;
    Eigen::VectorXd upperBound() const override;

private:
    VehicleParams params_;
};

/**
 * @brief 约束管理器
 */
class ConstraintManager {
public:
    ConstraintManager() = default;
    
    void initialize(const VehicleParams& params);
    void addConstraint(ConstraintPtr constraint);
    void removeConstraint(const std::string& name);
    
    bool checkConstraints(
        const StateVector& x,
        const ControlVector& u,
        double tolerance = 1e-6) const;
    
    void buildQPConstraints(
        int N,
        Eigen::SparseMatrix<double>& A,
        Eigen::VectorXd& l,
        Eigen::VectorXd& u) const;
    
    int totalConstraints(int N) const;

    void setControlRateConstraints(bool enable_omega,
                                   bool enable_accel,
                                   double dt,
                                   const ControlVector& u_prev);

private:
    std::vector<ConstraintPtr> constraints_;
    VehicleParams params_;
    bool enable_omega_rate_ = false;
    bool enable_accel_rate_ = false;
    double dt_ = 0.0;
    ControlVector u_prev_ = ControlVector::Zero();
};

}  // namespace scout_omni_local_planner
