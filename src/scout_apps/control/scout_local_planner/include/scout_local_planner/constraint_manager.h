/**
 * @file constraint_manager.h
 * @brief 约束管理器
 * 
 * 管理 MPC 的各种约束：
 * - 状态约束（速度限制）
 * - 控制约束（加速度限制）
 * - 液体晃动约束（第 2 步）
 */

#pragma once

#include "scout_local_planner/types.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>
#include <vector>
#include <string>

namespace scout_local_planner {

/**
 * @brief 约束基类
 */
class ConstraintBase {
public:
    virtual ~ConstraintBase() = default;
    
    /**
     * @brief 获取约束名称
     */
    virtual std::string name() const = 0;
    
    /**
     * @brief 获取约束数量
     */
    virtual int numConstraints() const = 0;
    
    /**
     * @brief 评估约束值
     * @param x 状态
     * @param u 控制
     * @return 约束值向量 c(x, u)
     */
    virtual Eigen::VectorXd evaluate(
        const StateVector& x,
        const ControlVector& u) const = 0;
    
    /**
     * @brief 获取约束下界
     */
    virtual Eigen::VectorXd lowerBound() const = 0;
    
    /**
     * @brief 获取约束上界
     */
    virtual Eigen::VectorXd upperBound() const = 0;
    
    /**
     * @brief 是否为软约束
     */
    virtual bool isSoft() const { return false; }
    
    /**
     * @brief 获取软约束惩罚权重
     */
    virtual double softWeight() const { return 0.0; }
};

using ConstraintPtr = std::shared_ptr<ConstraintBase>;

/**
 * @brief 状态边界约束
 * 
 * v_min ≤ v ≤ v_max
 */
class StateBoundsConstraint : public ConstraintBase {
public:
    StateBoundsConstraint(const VehicleParams& params);
    
    std::string name() const override { return "StateBoundsConstraint"; }
    int numConstraints() const override { return 1; }  // v
    
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
 * -a_max ≤ a ≤ a_max
 * -alpha_max ≤ alpha ≤ alpha_max
 */
class ControlBoundsConstraint : public ConstraintBase {
public:
    ControlBoundsConstraint(const VehicleParams& params);
    
    std::string name() const override { return "ControlBoundsConstraint"; }
    int numConstraints() const override { return 2; }  // a, alpha
    
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
 * 
 * 汇总所有约束，构建 QP 问题的约束矩阵
 */
class ConstraintManager {
public:
    ConstraintManager() = default;
    
    /**
     * @brief 使用默认约束初始化
     */
    void initialize(const VehicleParams& params);
    
    /**
     * @brief 添加约束
     */
    void addConstraint(ConstraintPtr constraint);
    
    /**
     * @brief 移除约束
     */
    void removeConstraint(const std::string& name);
    
    /**
     * @brief 检查约束是否满足
     */
    bool checkConstraints(
        const StateVector& x,
        const ControlVector& u,
        double tolerance = 1e-6) const;
    
    /**
     * @brief 构建 QP 问题的约束矩阵
     * 
     * l ≤ Az ≤ u
     * 其中 z = [x_0, u_0, x_1, u_1, ..., x_N]
     * 
     * @param N 预测步长
     * @param A 输出：约束矩阵（稀疏）
     * @param l 输出：下界
     * @param u 输出：上界
     */
    void buildQPConstraints(
        int N,
        Eigen::SparseMatrix<double>& A,
        Eigen::VectorXd& l,
        Eigen::VectorXd& u) const;
    
    /**
     * @brief 获取总约束数
     */
    int totalConstraints(int N) const;

    /**
     * @brief 设置控制变化率约束参数
     * @param enable_omega 是否启用角速度变化率约束
     * @param enable_accel 是否启用加速度变化率约束
     * @param dt 离散时间步长
     * @param u_prev 上一时刻控制量（用于 k=0）
     */
    void setControlRateConstraints(bool enable_omega,
                                   bool enable_accel,
                                   double dt,
                                   const ControlVector& u_prev);

    /**
     * @brief 设置液体晃动盒约束代理
     * @param enable 是否启用
     * @param eta_bar 模态位移盒约束阈值
     */
    void setSloshBoxConstraint(bool enable, double eta_bar);

private:
    std::vector<ConstraintPtr> constraints_;
    VehicleParams params_;
    bool enable_omega_rate_ = false;
    bool enable_accel_rate_ = false;
    bool enable_slosh_box_constraint_ = false;
    double slosh_eta_bar_ = 0.0;
    double dt_ = 0.0;
    ControlVector u_prev_ = ControlVector::Zero();
};

}  // namespace scout_local_planner
