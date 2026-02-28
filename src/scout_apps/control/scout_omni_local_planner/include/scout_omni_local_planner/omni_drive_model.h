/**
 * @file omni_drive_model.h
 * @brief 全向轮底盘 Frenet 动力学模型
 * 
 * 基础状态：x_base = [e_l, e_c, e_θ, v_x, v_y]ᵀ (5维)
 * 晃动状态：x_slosh = [η_x, η̇_x, η_y, η̇_y]ᵀ (4维)
 * 增广状态：x = [x_base; x_slosh]ᵀ (9维)
 * 控制：u = [a_x, a_y, ω]ᵀ
 * 
 * 全向轮 Frenet 动力学：
 *   e_l_dot   = v_x - v_path
 *   e_c_dot   = v_x * sin(e_θ) + v_y * cos(e_θ) ≈ v_x * e_θ + v_y
 *   e_θ_dot   = ω - κ(s) * v_x
 *   v_x_dot   = a_x
 *   v_y_dot   = a_y
 * 
 * 与差速模型的关键区别：
 *   - e_c_dot 包含 v_y 项：全向轮可直接横移消除横向误差
 *   - v_y 是独立状态，a_y 是独立控制量
 *   - cmd_vel 使用 linear.x, linear.y, angular.z
 */

#pragma once

#include "scout_omni_local_planner/dynamics_model.h"
#include <Eigen/Dense>

namespace scout_omni_local_planner {

class SloshIntegration;

class OmniDriveModel : public DynamicsModelBase {
public:
    OmniDriveModel() = default;
    explicit OmniDriveModel(const VehicleParams& params);
    
    int stateDim() const override { return StateIndex::TOTAL_DIM; }
    int controlDim() const override { return ControlIndex::DIM; }
    
    StateVector predict(
        const StateVector& x,
        const ControlVector& u,
        const ReferencePoint& ref,
        double dt) const override;
    
    void linearize(
        const StateVector& x,
        const ControlVector& u,
        const ReferencePoint& ref,
        double dt,
        Eigen::MatrixXd& A,
        Eigen::MatrixXd& B,
        Eigen::VectorXd& c) const override;
    
    std::string name() const override { return "OmniDriveModel"; }
    
    void setParams(const VehicleParams& params) { params_ = params; }
    void setSloshIntegration(SloshIntegration* slosh) { slosh_integration_ = slosh; }

private:
    VehicleParams params_;
    SloshIntegration* slosh_integration_ = nullptr;
};

}  // namespace scout_omni_local_planner
