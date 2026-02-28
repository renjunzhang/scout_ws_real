/**
 * @file cost_function.h
 * @brief 全向轮 MPC 代价函数
 * 
 * 代价函数包括：
 * - 路径跟踪误差（Frenet）
 * - 纵向/横向速度跟踪误差
 * - 控制平滑（a_x, a_y, ω）
 * - 控制变化率（Δa_x, Δa_y, Δω）
 * - 液体晃动抑制（可选）
 */

#pragma once

#include "scout_omni_local_planner/types.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>
#include <vector>

namespace scout_omni_local_planner {

/**
 * @brief 代价项基类
 */
class CostTermBase {
public:
    virtual ~CostTermBase() = default;
    virtual std::string name() const = 0;
    
    virtual double evaluate(
        const StateVector& x,
        const ControlVector& u,
        const ReferencePoint& ref,
        int k) const = 0;
    
    virtual void getQuadraticCost(
        int k, int N,
        Eigen::MatrixXd& Q_contrib,
        Eigen::MatrixXd& R_contrib,
        Eigen::VectorXd& q_contrib,
        Eigen::VectorXd& r_contrib) const = 0;
};

using CostTermPtr = std::shared_ptr<CostTermBase>;

/**
 * @brief 状态跟踪代价
 * 
 * J = Q_el * e_l² + Q_ec * e_c² + Q_etheta * e_theta²
 *   + Q_vx * (v_x - v_ref)² + Q_vy * (v_y - vy_ref)²
 */
class StateTrackingCost : public CostTermBase {
public:
    StateTrackingCost(const MPCParams& params);
    
    std::string name() const override { return "StateTrackingCost"; }
    
    double evaluate(
        const StateVector& x,
        const ControlVector& u,
        const ReferencePoint& ref,
        int k) const override;
    
    void getQuadraticCost(
        int k, int N,
        Eigen::MatrixXd& Q_contrib,
        Eigen::MatrixXd& R_contrib,
        Eigen::VectorXd& q_contrib,
        Eigen::VectorXd& r_contrib) const override;
    
    void setParams(const MPCParams& params) { params_ = params; }

private:
    MPCParams params_;
};

/**
 * @brief 控制代价
 * 
 * J = R_ax * a_x² + R_ay * a_y² + R_omega * ω²
 */
class ControlCost : public CostTermBase {
public:
    ControlCost(const MPCParams& params);
    
    std::string name() const override { return "ControlCost"; }
    
    double evaluate(
        const StateVector& x,
        const ControlVector& u,
        const ReferencePoint& ref,
        int k) const override;
    
    void getQuadraticCost(
        int k, int N,
        Eigen::MatrixXd& Q_contrib,
        Eigen::MatrixXd& R_contrib,
        Eigen::VectorXd& q_contrib,
        Eigen::VectorXd& r_contrib) const override;

private:
    MPCParams params_;
};

/**
 * @brief 控制变化率代价
 * 
 * J = R_dax * (a_x[k] - a_x[k-1])² + R_day * (a_y[k] - a_y[k-1])²
 *   + R_domega * (ω[k] - ω[k-1])²
 */
class ControlRateCost : public CostTermBase {
public:
    ControlRateCost(const MPCParams& params);
    
    std::string name() const override { return "ControlRateCost"; }
    
    double evaluate(
        const StateVector& x,
        const ControlVector& u,
        const ReferencePoint& ref,
        int k) const override;
    
    void getQuadraticCost(
        int k, int N,
        Eigen::MatrixXd& Q_contrib,
        Eigen::MatrixXd& R_contrib,
        Eigen::VectorXd& q_contrib,
        Eigen::VectorXd& r_contrib) const override;
    
    void setPreviousControl(const ControlVector& u_prev) { u_prev_ = u_prev; }

private:
    MPCParams params_;
    ControlVector u_prev_ = ControlVector::Zero();
};

/**
 * @brief 代价函数管理器
 */
class CostFunction {
public:
    CostFunction() = default;
    
    void initialize(const MPCParams& params);
    void addCostTerm(CostTermPtr term);
    void removeCostTerm(const std::string& name);
    
    double computeTotalCost(
        const std::vector<StateVector>& x_traj,
        const std::vector<ControlVector>& u_traj,
        const std::vector<ReferencePoint>& refs) const;
    
    void buildQPCost(
        int N,
        const std::vector<ReferencePoint>& refs,
        Eigen::SparseMatrix<double>& H,
        Eigen::VectorXd& g) const;
    
    void setPreviousControl(const ControlVector& u_prev);

private:
    std::vector<CostTermPtr> cost_terms_;
    MPCParams params_;
    ControlVector u_prev_ = ControlVector::Zero();
};

}  // namespace scout_omni_local_planner
