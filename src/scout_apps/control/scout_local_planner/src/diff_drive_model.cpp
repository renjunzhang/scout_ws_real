/**
 * @file diff_drive_model.cpp
 * @brief 差速底盘 Frenet 动力学模型实现
 */

#include "scout_local_planner/diff_drive_model.h"
#include <cmath>

namespace scout_local_planner {

DiffDriveModel::DiffDriveModel(const VehicleParams& params)
    : params_(params) {}

StateVector DiffDriveModel::predict(
    const StateVector& x,
    const ControlVector& u,
    const ReferencePoint& ref,
    double dt) const {
    
    // 提取当前状态
    double e_l = x(StateIndex::E_L);
    double e_c = x(StateIndex::E_C);
    double e_theta = x(StateIndex::E_THETA);
    double v = x(StateIndex::V);
    double omega = x(StateIndex::OMEGA);
    
    // 提取控制量
    double a = u(ControlIndex::A);
    double alpha = u(ControlIndex::ANG_ACC);
    
    // 提取参考信息
    double kappa = ref.kappa;
    double v_path = ref.v_path;
    
    // Frenet 动力学（小角度简化，欧拉法离散化）
    // e_l_dot ≈ v - v_path
    // e_c_dot ≈ v * e_theta
    // e_theta_dot ≈ omega - kappa * v
    // v_dot = a
    // omega_dot = alpha
    
    StateVector x_next;
    x_next(StateIndex::E_L) = e_l + dt * (v - v_path);
    x_next(StateIndex::E_C) = e_c + dt * v * e_theta;
    x_next(StateIndex::E_THETA) = e_theta + dt * (omega - kappa * v);
    x_next(StateIndex::V) = v + dt * a;
    x_next(StateIndex::OMEGA) = omega + dt * alpha;
    
    // 速度约束（硬裁剪）
    x_next(StateIndex::V) = std::max(params_.v_min, 
                             std::min(params_.v_max, x_next(StateIndex::V)));
    x_next(StateIndex::OMEGA) = std::max(-params_.omega_max,
                                 std::min(params_.omega_max, x_next(StateIndex::OMEGA)));
    
    return x_next;
}

void DiffDriveModel::linearize(
    const StateVector& x,
    const ControlVector& u,
    const ReferencePoint& ref,
    double dt,
    Eigen::MatrixXd& A,
    Eigen::MatrixXd& B,
    Eigen::VectorXd& c) const {
    
    const int nx = stateDim();
    const int nu = controlDim();
    
    // 提取状态
    double e_theta = x(StateIndex::E_THETA);
    double v = x(StateIndex::V);
    
    // 提取参考
    double kappa = ref.kappa;
    double v_path = ref.v_path;
    
    // 初始化矩阵
    A = Eigen::MatrixXd::Identity(nx, nx);
    B = Eigen::MatrixXd::Zero(nx, nu);
    c = Eigen::VectorXd::Zero(nx);
    
    // ====== A 矩阵（∂f/∂x）======
    // 对于离散化后的系统 x[k+1] = x[k] + dt * f(x[k], u[k])
    // A = I + dt * (∂f_continuous/∂x)
    
    // ∂(e_l_dot)/∂v = 1
    A(StateIndex::E_L, StateIndex::V) = dt * 1.0;
    
    // ∂(e_c_dot)/∂e_theta = v
    A(StateIndex::E_C, StateIndex::E_THETA) = dt * v;
    // ∂(e_c_dot)/∂v = e_theta
    A(StateIndex::E_C, StateIndex::V) = dt * e_theta;
    
    // ∂(e_theta_dot)/∂v = -kappa
    A(StateIndex::E_THETA, StateIndex::V) = dt * (-kappa);
    // ∂(e_theta_dot)/∂omega = 1
    A(StateIndex::E_THETA, StateIndex::OMEGA) = dt * 1.0;
    
    // v_dot = a, omega_dot = alpha（线性，已在 Identity 中）
    
    // ====== B 矩阵（∂f/∂u）======
    // ∂(v_dot)/∂a = 1
    B(StateIndex::V, ControlIndex::A) = dt;
    // ∂(omega_dot)/∂alpha = 1
    B(StateIndex::OMEGA, ControlIndex::ANG_ACC) = dt;
    
    // ====== c 常数项（仿射项）======
    // c = f(x_ref, u_ref) - A * x_ref - B * u_ref
    // 对于线性系统，c 主要来自 v_path
    c(StateIndex::E_L) = -dt * v_path;
    
    // 注意：对于非线性项 v * e_theta，线性化后有仿射项
    // 这里简化处理，假设 e_theta 较小
}

}  // namespace scout_local_planner
