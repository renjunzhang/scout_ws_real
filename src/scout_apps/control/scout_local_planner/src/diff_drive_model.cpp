/**
 * @file diff_drive_model.cpp
 * @brief 差速底盘 Frenet 动力学模型实现 (直接 ω 控制模式)
 * 
 * 状态：x = [e_l, e_c, e_θ, v]ᵀ (4维)
 * 控制：u = [a, ω]ᵀ (直接控制角速度，更平滑！)
 * 
 * 动力学：
 *   e_l_dot = v - v_path
 *   e_c_dot = v * e_θ
 *   e_θ_dot = ω - κ(s) * v
 *   v_dot = a
 */

#include "scout_local_planner/diff_drive_model.h"
#include "scout_local_planner/slosh_integration.h"
#include <cmath>

namespace scout_local_planner {

DiffDriveModel::DiffDriveModel(const VehicleParams& params)
    : params_(params) {}

StateVector DiffDriveModel::predict(
    const StateVector& x,
    const ControlVector& u,
    const ReferencePoint& ref,
    double dt) const {
    
    // ========== 基础状态动力学 (4维) ==========
    // 提取当前状态
    double e_l = x(StateIndex::E_L);
    double e_c = x(StateIndex::E_C);
    double e_theta = x(StateIndex::E_THETA);
    double v = x(StateIndex::V);
    
    // 提取控制量（直接 ω 控制！）
    double a = u(ControlIndex::A);
    double omega = u(ControlIndex::OMEGA);  // 直接控制角速度
    
    // 提取参考信息
    double kappa = ref.kappa;
    double v_path = ref.v_path;
    
    // Frenet 动力学（小角度简化，欧拉法离散化）
    // 注意：ω 是控制量，直接使用，无需积分！
    StateVector x_next;
    x_next(StateIndex::E_L) = e_l + dt * (v - v_path);
    x_next(StateIndex::E_C) = e_c + dt * v * e_theta;
    x_next(StateIndex::E_THETA) = e_theta + dt * (omega - kappa * v);
    x_next(StateIndex::V) = v + dt * a;
    
    // 速度约束由 QP 约束处理，避免非光滑裁剪影响线性化一致性
    
    // ========== 晃动状态动力学 (4维) ==========
    if (StateIndex::SLOSH_DIM > 0) {
        if (slosh_integration_ != nullptr && slosh_integration_->isConfigured()) {
            // 从当前增广状态提取晃动状态
            Eigen::Vector4d x_slosh_curr;
            x_slosh_curr << x(StateIndex::ETA_X), 
                            x(StateIndex::ETA_X_DOT),
                            x(StateIndex::ETA_Y), 
                            x(StateIndex::ETA_Y_DOT);
            
            // 加速度映射：
            // ax = a (纵向加速度)
            // ay = v * omega (离心加速度，横向)
            // 注意(MPC_INTEGRATION_NOTES §4.3)：此处 predictSlosh() 不做旋转修正，
            // 与 LiquidSloshModel::update() 在 offset_x/y=0 时一致。
            // 若未来开启偏心项，必须在此处也加入旋转修正项！
            double ax = a;
            double ay = v * omega;
            
            // 预测下一步晃动状态
            Eigen::Vector4d x_slosh_next = slosh_integration_->predictSlosh(x_slosh_curr, ax, ay);
            
            x_next(StateIndex::ETA_X) = x_slosh_next(0);
            x_next(StateIndex::ETA_X_DOT) = x_slosh_next(1);
            x_next(StateIndex::ETA_Y) = x_slosh_next(2);
            x_next(StateIndex::ETA_Y_DOT) = x_slosh_next(3);
        } else {
            // 无晃动模型时保持状态不变
            x_next(StateIndex::ETA_X) = x(StateIndex::ETA_X);
            x_next(StateIndex::ETA_X_DOT) = x(StateIndex::ETA_X_DOT);
            x_next(StateIndex::ETA_Y) = x(StateIndex::ETA_Y);
            x_next(StateIndex::ETA_Y_DOT) = x(StateIndex::ETA_Y_DOT);
        }
    }
    
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
    
    // 初始化增广矩阵
    A = Eigen::MatrixXd::Identity(nx, nx);
    B = Eigen::MatrixXd::Zero(nx, nu);
    c = Eigen::VectorXd::Zero(nx);
    
    // ====== 基础状态部分 A 矩阵（∂f/∂x）======
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
    // 注意：ω 是控制量，不在状态中！
    
    // ====== 基础状态部分 B 矩阵（∂f/∂u）======
    // ∂(v_dot)/∂a = 1
    B(StateIndex::V, ControlIndex::A) = dt;
    // ∂(e_theta_dot)/∂omega = 1 （直接控制！）
    B(StateIndex::E_THETA, ControlIndex::OMEGA) = dt;
    
    // ====== 基础状态部分 c 常数项 ======
    c(StateIndex::E_L) = -dt * v_path;
    
    // ====== 晃动状态部分（如果启用）======
    if (StateIndex::SLOSH_DIM > 0) {
        if (slosh_integration_ != nullptr && slosh_integration_->isConfigured()) {
            // 获取晃动模型的离散矩阵
            Eigen::Matrix4d A_slosh;
            Eigen::Matrix<double, 4, 2> B_slosh;
            slosh_integration_->getDiscreteMatrices(A_slosh, B_slosh);
            
            // 填充增广 A 矩阵的晃动部分
            A.block<4, 4>(StateIndex::ETA_X, StateIndex::ETA_X) = A_slosh;
            
            // 填充增广 B 矩阵和 A 矩阵的晃动耦合部分
            // ay = v * omega → 输入 u_slosh = [ax, ay] = [a, v*omega]
            //
            // ∂(slosh)/∂a = B_slosh[:, 0]  (纵向加速度)
            B(StateIndex::ETA_X, ControlIndex::A) = B_slosh(0, 0);
            B(StateIndex::ETA_X_DOT, ControlIndex::A) = B_slosh(1, 0);
            B(StateIndex::ETA_Y, ControlIndex::A) = B_slosh(2, 0);
            B(StateIndex::ETA_Y_DOT, ControlIndex::A) = B_slosh(3, 0);
            
            // ∂(slosh)/∂omega = B_slosh[:, 1] * v  (∂ay/∂omega = v)
            B(StateIndex::ETA_X, ControlIndex::OMEGA) = B_slosh(0, 1) * v;
            B(StateIndex::ETA_X_DOT, ControlIndex::OMEGA) = B_slosh(1, 1) * v;
            B(StateIndex::ETA_Y, ControlIndex::OMEGA) = B_slosh(2, 1) * v;
            B(StateIndex::ETA_Y_DOT, ControlIndex::OMEGA) = B_slosh(3, 1) * v;
            
            // ∂(slosh)/∂v = B_slosh[:, 1] * omega  (∂ay/∂v = omega)
            // 关键耦合项：速度变化对横向晃动激励的灵敏度
            double omega_lin = u(ControlIndex::OMEGA);
            A(StateIndex::ETA_X, StateIndex::V) += B_slosh(0, 1) * omega_lin;
            A(StateIndex::ETA_X_DOT, StateIndex::V) += B_slosh(1, 1) * omega_lin;
            A(StateIndex::ETA_Y, StateIndex::V) += B_slosh(2, 1) * omega_lin;
            A(StateIndex::ETA_Y_DOT, StateIndex::V) += B_slosh(3, 1) * omega_lin;
        }
        // 如果无晃动模型，A 已初始化为单位阵（状态保持）
    }

    // 仿射项：保证线性化与名义轨迹一致
    StateVector x_next_nom = predict(x, u, ref, dt);
    c = x_next_nom - A * x - B * u;
}

}  // namespace scout_local_planner
