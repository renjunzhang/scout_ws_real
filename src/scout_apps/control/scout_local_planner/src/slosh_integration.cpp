/**
 * @file slosh_integration.cpp
 * @brief 液体晃动模型与 MPC 的集成实现
 */

#include "scout_local_planner/slosh_integration.h"
#include <ros/ros.h>

namespace scout_local_planner {

SloshIntegration::SloshIntegration() {
    A_discrete_.setIdentity();
    B_discrete_.setZero();
}

bool SloshIntegration::configure(const SloshParams& params) {
    params_ = params;
    
    // 构建 LiquidSloshModel 参数
    slosh_models::LiquidSloshModel::Params slosh_params;
    slosh_params.R = params.container_radius;
    slosh_params.h = params.liquid_height;
    slosh_params.rho = params.liquid_density;
    slosh_params.dt = params.dt;
    slosh_params.mode_index = params.mode_index;
    slosh_params.zeta = params.damping_ratio;
    slosh_params.r_x = params.offset_x;
    slosh_params.r_y = params.offset_y;
    slosh_params.use_linear_model = params.use_linear_model;
    slosh_params.use_parabola_term = params.use_parabola_term;
    
    // 配置晃动模型
    if (!slosh_model_.configure(slosh_params)) {
        ROS_ERROR("[SloshIntegration] Failed to configure liquid slosh model");
        configured_ = false;
        return false;
    }
    
    // 缓存离散矩阵
    slosh_model_.getDiscreteMatrices(A_discrete_, B_discrete_);
    
    ROS_INFO("[SloshIntegration] Configured successfully:");
    ROS_INFO("  Container: R=%.3f m, h=%.3f m", params.container_radius, params.liquid_height);
    ROS_INFO("  Mode %d: omega_n=%.2f rad/s, zeta=%.3f", 
             params.mode_index, slosh_model_.getModalParams().omega_n, params.damping_ratio);
    
    configured_ = true;
    return true;
}

void SloshIntegration::reset() {
    slosh_model_.reset();
}

void SloshIntegration::getDiscreteMatrices(Eigen::Matrix4d& A_slosh, 
                                           Eigen::Matrix<double, 4, 2>& B_slosh) const {
    A_slosh = A_discrete_;
    B_slosh = B_discrete_;
}

void SloshIntegration::update(double ax, double ay, double omega_z, double alpha_z) {
    Eigen::Vector2d accel(ax, ay);
    slosh_model_.update(accel, omega_z, alpha_z);
}

Eigen::Vector4d SloshIntegration::getSloshState() const {
    return slosh_model_.getState();
}

double SloshIntegration::getSloshHeight() const {
    return slosh_model_.getSloshHeight();
}

Eigen::Matrix4d SloshIntegration::getSloshCostMatrix(double Q_slosh) const {
    if (!configured_ || Q_slosh <= 0.0) {
        return Eigen::Matrix4d::Zero();
    }
    
    // 液面高度 η = h_coeff * sqrt(xn² + yn²)
    // 简化代价: J = Q_slosh * (xn² + yn²)
    // 即只惩罚位移，不惩罚速度
    // H_slosh = Q_slosh * diag([1, 0, 1, 0])
    Eigen::Matrix4d H;
    H.setZero();
    H(0, 0) = Q_slosh;  // xn²
    H(2, 2) = Q_slosh;  // yn²
    
    // 更精确的方法是使用 height_coeff 加权
    // double h_coeff = slosh_model_.getModalParams().height_coeff;
    // H *= h_coeff * h_coeff;
    
    return H;
}

const slosh_models::LiquidSloshModel::ModalParams& SloshIntegration::getModalParams() const {
    return slosh_model_.getModalParams();
}

void SloshIntegration::writeToAugmentedState(StateVector& x_augmented) const {
    Eigen::Vector4d slosh_state = getSloshState();
    x_augmented(StateIndex::ETA_X) = slosh_state(0);
    x_augmented(StateIndex::ETA_X_DOT) = slosh_state(1);
    x_augmented(StateIndex::ETA_Y) = slosh_state(2);
    x_augmented(StateIndex::ETA_Y_DOT) = slosh_state(3);
}

void SloshIntegration::readFromAugmentedState(const StateVector& x_augmented) {
    // 注意：这只是读取状态用于外部逻辑，不直接设置内部 slosh_model_ 状态
    // 如需设置，需要在 LiquidSloshModel 添加 setState() 方法
}

Eigen::Vector4d SloshIntegration::predictSlosh(const Eigen::Vector4d& x_slosh_curr,
                                                double ax, double ay) const {
    Eigen::Vector2d u(ax, ay);
    return A_discrete_ * x_slosh_curr + B_discrete_ * u;
}

}  // namespace scout_local_planner
