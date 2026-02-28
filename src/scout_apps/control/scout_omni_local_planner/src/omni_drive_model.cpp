/**
 * @file omni_drive_model.cpp
 * @brief 全向轮底盘 Frenet 动力学模型实现
 * 
 * 状态：x = [e_l, e_c, e_θ, v_x, v_y]ᵀ (5维)
 * 控制：u = [a_x, a_y, ω]ᵀ
 * 
 * 动力学：
 *   e_l_dot   = v_x - v_path
 *   e_c_dot   = v_x * e_θ + v_y  (小角度近似)
 *   e_θ_dot   = ω - κ(s) * v_x
 *   v_x_dot   = a_x
 *   v_y_dot   = a_y
 */

#include "scout_omni_local_planner/omni_drive_model.h"
#include "scout_omni_local_planner/slosh_integration.h"
#include <cmath>

namespace scout_omni_local_planner {

OmniDriveModel::OmniDriveModel(const VehicleParams& params)
    : params_(params) {}

StateVector OmniDriveModel::predict(
    const StateVector& x,
    const ControlVector& u,
    const ReferencePoint& ref,
    double dt) const {
    
    // ========== 基础状态动力学 (5维) ==========
    double e_l = x(StateIndex::E_L);
    double e_c = x(StateIndex::E_C);
    double e_theta = x(StateIndex::E_THETA);
    double vx = x(StateIndex::V_X);
    double vy = x(StateIndex::V_Y);
    
    double ax = u(ControlIndex::A_X);
    double ay = u(ControlIndex::A_Y);
    double omega = u(ControlIndex::OMEGA);
    
    double kappa = ref.kappa;
    double v_path = ref.v_path;
    
    // 全向轮 Frenet 动力学（欧拉法离散化）
    StateVector x_next;
    x_next(StateIndex::E_L)     = e_l + dt * (vx - v_path);
    x_next(StateIndex::E_C)     = e_c + dt * (vx * e_theta + vy);  // 关键：v_y 可直接消除 e_c
    x_next(StateIndex::E_THETA) = e_theta + dt * (omega - kappa * vx);
    x_next(StateIndex::V_X)     = vx + dt * ax;
    x_next(StateIndex::V_Y)     = vy + dt * ay;
    
    // ========== 晃动状态动力学 (4维) ==========
    if (StateIndex::SLOSH_DIM > 0) {
        if (slosh_integration_ != nullptr && slosh_integration_->isConfigured()) {
            Eigen::Vector4d x_slosh_curr;
            x_slosh_curr << x(StateIndex::ETA_X), 
                            x(StateIndex::ETA_X_DOT),
                            x(StateIndex::ETA_Y), 
                            x(StateIndex::ETA_Y_DOT);
            
            // 全向轮加速度映射：
            //   a_lon = a_x（纵向）
            //   a_lat = a_y + v_x * omega（横向加速度 + 离心力）
            double a_lon = ax;
            double a_lat = ay + vx * omega;
            
            Eigen::Vector4d x_slosh_next = slosh_integration_->predictSlosh(x_slosh_curr, a_lon, a_lat);
            
            x_next(StateIndex::ETA_X)     = x_slosh_next(0);
            x_next(StateIndex::ETA_X_DOT) = x_slosh_next(1);
            x_next(StateIndex::ETA_Y)     = x_slosh_next(2);
            x_next(StateIndex::ETA_Y_DOT) = x_slosh_next(3);
        } else {
            x_next(StateIndex::ETA_X)     = x(StateIndex::ETA_X);
            x_next(StateIndex::ETA_X_DOT) = x(StateIndex::ETA_X_DOT);
            x_next(StateIndex::ETA_Y)     = x(StateIndex::ETA_Y);
            x_next(StateIndex::ETA_Y_DOT) = x(StateIndex::ETA_Y_DOT);
        }
    }
    
    return x_next;
}

void OmniDriveModel::linearize(
    const StateVector& x,
    const ControlVector& u,
    const ReferencePoint& ref,
    double dt,
    Eigen::MatrixXd& A,
    Eigen::MatrixXd& B,
    Eigen::VectorXd& c) const {
    
    const int nx = stateDim();
    const int nu = controlDim();
    
    double e_theta = x(StateIndex::E_THETA);
    double vx = x(StateIndex::V_X);
    double omega = u(ControlIndex::OMEGA);
    
    double kappa = ref.kappa;
    double v_path = ref.v_path;
    
    // 初始化增广矩阵
    A = Eigen::MatrixXd::Identity(nx, nx);
    B = Eigen::MatrixXd::Zero(nx, nu);
    c = Eigen::VectorXd::Zero(nx);
    
    // ====== 基础状态 A 矩阵 ======
    // ∂(e_l_dot)/∂v_x = 1
    A(StateIndex::E_L, StateIndex::V_X) = dt;
    
    // ∂(e_c_dot)/∂e_theta = v_x
    A(StateIndex::E_C, StateIndex::E_THETA) = dt * vx;
    // ∂(e_c_dot)/∂v_x = e_theta
    A(StateIndex::E_C, StateIndex::V_X) = dt * e_theta;
    // ∂(e_c_dot)/∂v_y = 1  ← 全向轮关键项！
    A(StateIndex::E_C, StateIndex::V_Y) = dt;
    
    // ∂(e_theta_dot)/∂v_x = -kappa
    A(StateIndex::E_THETA, StateIndex::V_X) = dt * (-kappa);
    
    // ====== 基础状态 B 矩阵 ======
    // ∂(v_x_dot)/∂a_x = 1
    B(StateIndex::V_X, ControlIndex::A_X) = dt;
    // ∂(v_y_dot)/∂a_y = 1
    B(StateIndex::V_Y, ControlIndex::A_Y) = dt;
    // ∂(e_theta_dot)/∂omega = 1
    B(StateIndex::E_THETA, ControlIndex::OMEGA) = dt;
    
    // ====== 常数项 ======
    c(StateIndex::E_L) = -dt * v_path;
    
    // ====== 晃动状态部分 ======
    if (StateIndex::SLOSH_DIM > 0) {
        if (slosh_integration_ != nullptr && slosh_integration_->isConfigured()) {
            Eigen::Matrix4d A_slosh;
            Eigen::Matrix<double, 4, 2> B_slosh;
            slosh_integration_->getDiscreteMatrices(A_slosh, B_slosh);
            
            // 填充增广 A 矩阵的晃动子块
            A.block<4, 4>(StateIndex::ETA_X, StateIndex::ETA_X) = A_slosh;
            
            // a_lon = a_x → ∂晃动/∂a_x = B_slosh[:, 0]
            B(StateIndex::ETA_X,     ControlIndex::A_X) = B_slosh(0, 0);
            B(StateIndex::ETA_X_DOT, ControlIndex::A_X) = B_slosh(1, 0);
            B(StateIndex::ETA_Y,     ControlIndex::A_X) = B_slosh(2, 0);
            B(StateIndex::ETA_Y_DOT, ControlIndex::A_X) = B_slosh(3, 0);
            
            // a_lat = a_y + v_x * omega
            // ∂晃动/∂a_y = B_slosh[:, 1]
            B(StateIndex::ETA_X,     ControlIndex::A_Y) = B_slosh(0, 1);
            B(StateIndex::ETA_X_DOT, ControlIndex::A_Y) = B_slosh(1, 1);
            B(StateIndex::ETA_Y,     ControlIndex::A_Y) = B_slosh(2, 1);
            B(StateIndex::ETA_Y_DOT, ControlIndex::A_Y) = B_slosh(3, 1);
            
            // ∂晃动/∂omega = B_slosh[:, 1] * v_x（离心力贡献）
            B(StateIndex::ETA_X,     ControlIndex::OMEGA) += B_slosh(0, 1) * vx;
            B(StateIndex::ETA_X_DOT, ControlIndex::OMEGA) += B_slosh(1, 1) * vx;
            B(StateIndex::ETA_Y,     ControlIndex::OMEGA) += B_slosh(2, 1) * vx;
            B(StateIndex::ETA_Y_DOT, ControlIndex::OMEGA) += B_slosh(3, 1) * vx;
            
            // ∂晃动/∂v_x（通过 a_lat = ... + v_x * omega 的 v_x 依赖）
            A(StateIndex::ETA_X,     StateIndex::V_X) += B_slosh(0, 1) * omega;
            A(StateIndex::ETA_X_DOT, StateIndex::V_X) += B_slosh(1, 1) * omega;
            A(StateIndex::ETA_Y,     StateIndex::V_X) += B_slosh(2, 1) * omega;
            A(StateIndex::ETA_Y_DOT, StateIndex::V_X) += B_slosh(3, 1) * omega;
        }
    }

    // 仿射修正项
    StateVector x_next_nom = predict(x, u, ref, dt);
    c = x_next_nom - A * x - B * u;
}

}  // namespace scout_omni_local_planner
