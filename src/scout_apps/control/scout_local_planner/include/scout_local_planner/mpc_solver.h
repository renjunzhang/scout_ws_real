/**
 * @file mpc_solver.h
 * @brief MPC 求解器
 * 
 * 使用 OSQP 求解二次规划问题
 */

#pragma once

#include "scout_local_planner/types.h"
#include "scout_local_planner/dynamics_model.h"
#include "scout_local_planner/cost_function.h"
#include "scout_local_planner/constraint_manager.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <osqp/osqp.h>
#include <memory>
#include <vector>

namespace scout_local_planner {

/**
 * @brief MPC 求解器
 * 
 * 将 MPC 问题转化为 QP 并使用 OSQP 求解
 */
class MPCSolver {
public:
    MPCSolver();
    ~MPCSolver();
    
    /**
     * @brief 初始化求解器
     */
    bool initialize(const MPCParams& mpc_params, const VehicleParams& vehicle_params);
    
    /**
     * @brief 设置动力学模型
     */
    void setDynamicsModel(DynamicsModelPtr model);
    
    /**
     * @brief 添加约束
     */
    void addConstraint(ConstraintPtr constraint);
    
    /**
     * @brief 移除约束
     */
    void removeConstraint(const std::string& name);
    
    /**
     * @brief 添加代价项
     */
    void addCostTerm(CostTermPtr term);
    
    /**
     * @brief 求解 MPC 问题
     * 
     * @param current_state 当前完整状态 [e_l, e_c, e_θ, v, ω]
     * @param reference_path 参考点序列
     * @return 求解结果
     */
    MPCSolution solve(
        const StateVector& current_state,
        const std::vector<ReferencePoint>& reference_path);
    
    /**
     * @brief 设置上一步控制量（用于控制变化率代价）
     */
    void setPreviousControl(const ControlVector& u_prev);
    
    /**
     * @brief 获取 MPC 参数
     */
    const MPCParams& getMPCParams() const { return mpc_params_; }
    
    /**
     * @brief 更新 MPC 参数
     */
    void setMPCParams(const MPCParams& params);
    void resetWarmStart(bool keep_u_prev = true);

private:
    /**
     * @brief 构建 QP 问题
     * 
     * min  0.5 * z'Hz + g'z
     * s.t. l ≤ Az ≤ u
     *      A_eq * z = b_eq  (动力学约束)
     */
    bool buildQP(
        const StateVector& x0,
        const std::vector<ReferencePoint>& refs);
    
    /**
     * @brief 更新 OSQP 工作空间
     */
    bool updateOSQP();
    
    /**
     * @brief 从 QP 解中提取 MPC 解
     */
    void extractSolution(MPCSolution& solution);
    
    /**
     * @brief 使用上一次解进行热启动
     */
    void warmStart();
    
    /**
     * @brief 清理 OSQP 资源
     */
    void cleanupOSQP();

private:
    MPCParams mpc_params_;
    VehicleParams vehicle_params_;
    
    DynamicsModelPtr dynamics_model_;
    CostFunction cost_function_;
    ConstraintManager constraint_manager_;
    
    // 上一步控制量
    ControlVector u_prev_ = ControlVector::Zero();
    
    // QP 问题数据
    Eigen::SparseMatrix<double> P_;  // Hessian
    Eigen::VectorXd q_;              // 线性项
    Eigen::SparseMatrix<double> A_;  // 约束矩阵
    Eigen::VectorXd l_, u_;          // 约束边界
    
    // OSQP 数据（旧版 API）
    OSQPWorkspace* osqp_work_ = nullptr;
    OSQPSettings* osqp_settings_ = nullptr;
    OSQPData* osqp_data_ = nullptr;
    bool osqp_initialized_ = false;
    
    // CSC 格式数据（需要持久保存）
    std::vector<c_int> P_p_, P_i_;
    std::vector<c_float> P_x_;
    std::vector<c_int> A_p_, A_i_;
    std::vector<c_float> A_x_;
    std::vector<c_float> q_data_, l_data_, u_data_;
    
    // 决策变量维度
    int nz_ = 0;  // 总变量数
    int nc_ = 0;  // 总约束数
    
    // 上一次解（用于热启动）
    Eigen::VectorXd z_prev_;
    
    bool initialized_ = false;
};

}  // namespace scout_local_planner
