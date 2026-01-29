/**
 * @file diff_drive_model.h
 * @brief 差速底盘 Frenet 动力学模型
 * 
 * 状态：x = [e_l, e_c, e_θ, v, ω]ᵀ
 * 控制：u = [a, α]ᵀ
 * 
 * 动力学（小角度简化）：
 *   e_l_dot ≈ v - v_path
 *   e_c_dot ≈ v * e_θ
 *   e_θ_dot ≈ ω - κ(s) * v
 *   v_dot = a
 *   ω_dot = α
 */

#pragma once

#include "scout_local_planner/dynamics_model.h"

namespace scout_local_planner {

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

private:
    VehicleParams params_;
};

}  // namespace scout_local_planner
