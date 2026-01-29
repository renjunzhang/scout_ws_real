/**
 * @file dynamics_model.h
 * @brief 动力学模型基类
 * 
 * 定义 MPC 使用的动力学模型接口
 * 支持多种底盘类型（差速、全向等）
 */

#pragma once

#include "scout_local_planner/types.h"
#include <Eigen/Dense>
#include <memory>

namespace scout_local_planner {

/**
 * @brief 动力学模型基类
 * 
 * 提供状态预测和线性化接口
 */
class DynamicsModelBase {
public:
    virtual ~DynamicsModelBase() = default;
    
    /**
     * @brief 获取状态维度
     */
    virtual int stateDim() const = 0;
    
    /**
     * @brief 获取控制维度
     */
    virtual int controlDim() const = 0;
    
    /**
     * @brief 状态预测：x[k+1] = f(x[k], u[k], ref[k])
     * 
     * @param x 当前状态
     * @param u 控制输入
     * @param ref 参考点（提供 κ, v_path 等）
     * @param dt 时间步长
     * @return 下一步状态
     */
    virtual StateVector predict(
        const StateVector& x,
        const ControlVector& u,
        const ReferencePoint& ref,
        double dt) const = 0;
    
    /**
     * @brief 获取线性化矩阵（用于 QP 求解）
     * 
     * 线性化后：x[k+1] ≈ A * x[k] + B * u[k] + c
     * 
     * @param x 线性化点的状态
     * @param u 线性化点的控制
     * @param ref 参考点
     * @param dt 时间步长
     * @param A 输出：状态转移矩阵
     * @param B 输出：控制输入矩阵
     * @param c 输出：常数项（可选）
     */
    virtual void linearize(
        const StateVector& x,
        const ControlVector& u,
        const ReferencePoint& ref,
        double dt,
        Eigen::MatrixXd& A,
        Eigen::MatrixXd& B,
        Eigen::VectorXd& c) const = 0;
    
    /**
     * @brief 获取模型名称
     */
    virtual std::string name() const = 0;
};

using DynamicsModelPtr = std::shared_ptr<DynamicsModelBase>;

}  // namespace scout_local_planner
