/**
 * @file cost_function.h
 * @brief MPC 代价函数
 * 
 * 代价函数包括：
 * - 路径跟踪误差（Frenet）
 * - 速度跟踪误差（v_ref 只在这里！）
 * - 控制平滑
 * - 控制变化率
 * - 液体晃动抑制（第 2 步）
 */

#pragma once

#include "scout_local_planner/types.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>
#include <vector>

namespace scout_local_planner {

/**
 * @brief 代价项基类
 */
class CostTermBase {
public:
    virtual ~CostTermBase() = default;
    
    /**
     * @brief 获取代价项名称
     */
    virtual std::string name() const = 0;
    
    /**
     * @brief 计算代价值
     * @param x 状态
     * @param u 控制
     * @param ref 参考点
     * @param k 时间步
     * @return 代价值
     */
    virtual double evaluate(
        const StateVector& x,
        const ControlVector& u,
        const ReferencePoint& ref,
        int k) const = 0;
    
    /**
     * @brief 获取二次代价的 Q, R 矩阵贡献
     * 用于构建 QP 问题
     */
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
 * J = Q_el * e_l² + Q_ec * e_c² + Q_etheta * e_theta² + Q_v * (v - v_ref)²
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
 * J = R_a * a² + R_alpha * alpha²
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
 * J = R_da * (a[k] - a[k-1])² + R_dalpha * (alpha[k] - alpha[k-1])²
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
 * 
 * 汇总所有代价项，构建 QP 问题的代价矩阵
 */
class CostFunction {
public:
    CostFunction() = default;
    
    /**
     * @brief 使用默认代价项初始化
     */
    void initialize(const MPCParams& params);
    
    /**
     * @brief 添加代价项
     */
    void addCostTerm(CostTermPtr term);
    
    /**
     * @brief 移除代价项
     */
    void removeCostTerm(const std::string& name);
    
    /**
     * @brief 计算总代价
     */
    double computeTotalCost(
        const std::vector<StateVector>& x_traj,
        const std::vector<ControlVector>& u_traj,
        const std::vector<ReferencePoint>& refs) const;
    
    /**
     * @brief 构建 QP 问题的代价矩阵
     * 
     * min 0.5 * z'Hz + g'z
     * 其中 z = [x_0, u_0, x_1, u_1, ..., x_N]
     * 
     * @param N 预测步长
     * @param refs 参考点序列
     * @param H 输出：Hessian 矩阵（稀疏）
     * @param g 输出：梯度向量
     */
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

}  // namespace scout_local_planner
