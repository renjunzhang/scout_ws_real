/**
 * @file diff_drive_model.h
 * @brief 差速底盘 Frenet 动力学模型 (直接 ω 控制模式)
 * 
 * 基础状态：x_base = [e_l, e_c, e_θ, v]ᵀ (4维)
 * 晃动状态：x_slosh = [η_x, η̇_x, η_y, η̇_y]ᵀ (4维)
 * 增广状态：x = [x_base; x_slosh]ᵀ (8维)
 * 控制：u = [a, ω]ᵀ (直接控制角速度，更平滑！)
 * 
 * 基础动力学（小角度简化）：
 *   e_l_dot = v - v_path
 *   e_c_dot = v * e_θ
 *   e_θ_dot = ω - κ(s) * v  (ω 直接控制！)
 *   v_dot = a
 * 
 * 晃动动力学（独立子系统）：
 *   x_slosh[k+1] = A_slosh * x_slosh[k] + B_slosh * [ax, ay]ᵀ
 *   其中 ay = v * ω (离心加速度)
 */

#pragma once

#include "scout_local_planner/dynamics_model.h"
#include <Eigen/Dense>

namespace scout_local_planner {

// 前向声明
class SloshIntegration;

class DiffDriveModel : public DynamicsModelBase {
public:
    DiffDriveModel() = default;
    explicit DiffDriveModel(const VehicleParams& params);
    
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
    
    std::string name() const override { return "DiffDriveModel"; }
    
    void setParams(const VehicleParams& params) { params_ = params; }
    
    /**
     * @brief 设置晃动集成接口 (可选，为 nullptr 时忽略晃动动力学)
     */
    void setSloshIntegration(SloshIntegration* slosh) { slosh_integration_ = slosh; }

private:
    VehicleParams params_;
    SloshIntegration* slosh_integration_ = nullptr;  ///< 晃动集成接口 (不持有所有权)
};

}  // namespace scout_local_planner
