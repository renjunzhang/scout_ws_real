/**
 * @file mpc_solver.h
 * @brief 全向轮 MPC 求解器
 * 
 * 使用 OSQP 求解二次规划问题
 * 决策变量 z = [x₀, u₀, x₁, u₁, ..., x_N]
 *   其中 x ∈ R⁹, u ∈ R³
 */

#pragma once

#include "scout_omni_local_planner/types.h"
#include "scout_omni_local_planner/dynamics_model.h"
#include "scout_omni_local_planner/cost_function.h"
#include "scout_omni_local_planner/constraint_manager.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <osqp/osqp.h>
#include <memory>
#include <vector>

namespace scout_omni_local_planner {

class MPCSolver {
public:
    MPCSolver();
    ~MPCSolver();
    
    bool initialize(const MPCParams& mpc_params, const VehicleParams& vehicle_params);
    void setDynamicsModel(DynamicsModelPtr model);
    void addConstraint(ConstraintPtr constraint);
    void removeConstraint(const std::string& name);
    void addCostTerm(CostTermPtr term);
    
    MPCSolution solve(
        const StateVector& current_state,
        const std::vector<ReferencePoint>& reference_path);
    
    void setPreviousControl(const ControlVector& u_prev);
    const MPCParams& getMPCParams() const { return mpc_params_; }
    void setMPCParams(const MPCParams& params);
    void resetWarmStart(bool keep_u_prev = true);

private:
    bool buildQP(
        const StateVector& x0,
        const std::vector<ReferencePoint>& refs);
    
    bool updateOSQP();
    void extractSolution(MPCSolution& solution);
    void warmStart();
    void cleanupOSQP();

private:
    MPCParams mpc_params_;
    VehicleParams vehicle_params_;
    
    DynamicsModelPtr dynamics_model_;
    CostFunction cost_function_;
    ConstraintManager constraint_manager_;
    
    ControlVector u_prev_ = ControlVector::Zero();
    
    // QP 问题数据
    Eigen::SparseMatrix<double> P_;
    Eigen::VectorXd q_;
    Eigen::SparseMatrix<double> A_;
    Eigen::VectorXd l_, u_;
    
    // OSQP（旧版 API）
    OSQPWorkspace* osqp_work_ = nullptr;
    OSQPSettings* osqp_settings_ = nullptr;
    OSQPData* osqp_data_ = nullptr;
    bool osqp_initialized_ = false;
    
    std::vector<c_int> P_p_, P_i_;
    std::vector<c_float> P_x_;
    std::vector<c_int> A_p_, A_i_;
    std::vector<c_float> A_x_;
    std::vector<c_float> q_data_, l_data_, u_data_;
    
    int nz_ = 0;
    int nc_ = 0;
    
    Eigen::VectorXd z_prev_;
    bool initialized_ = false;
};

}  // namespace scout_omni_local_planner
