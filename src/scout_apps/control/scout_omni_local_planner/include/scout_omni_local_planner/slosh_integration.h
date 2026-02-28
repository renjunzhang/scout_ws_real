/**
 * @file slosh_integration.h
 * @brief 液体晃动模型与 MPC 的集成接口
 * 
 * 增广状态空间 (9维，全向轮):
 *   x = [e_l, e_c, e_θ, v_x, v_y, η_x, η̇_x, η_y, η̇_y]ᵀ
 *   u = [a_x, a_y, ω]ᵀ
 * 
 * 晃动动力学 (独立子系统):
 *   x_slosh[k+1] = A_slosh * x_slosh[k] + B_slosh * [a_lon, a_lat]ᵀ
 *   其中 a_lon = a_x, a_lat = a_y + v_x * ω
 */

#pragma once

#include "scout_omni_local_planner/types.h"
#include <slosh_models/liquid_slosh_model.h>
#include <Eigen/Dense>
#include <memory>

namespace scout_omni_local_planner {

/**
 * @brief 晃动参数结构
 */
struct SloshParams {
    double container_radius = 0.15;
    double liquid_height = 0.20;
    double liquid_density = 1000.0;
    int mode_index = 1;
    double damping_ratio = 0.05;
    double offset_x = 0.0;
    double offset_y = 0.0;
    bool use_linear_model = true;
    bool use_parabola_term = true;
    double dt = 0.05;
};

/**
 * @brief 晃动集成类
 */
class SloshIntegration {
public:
    SloshIntegration();
    ~SloshIntegration() = default;
    
    bool configure(const SloshParams& params);
    bool isConfigured() const { return configured_; }
    void reset();
    
    static constexpr int sloshDim() { return StateIndex::SLOSH_DIM; }
    
    void getDiscreteMatrices(Eigen::Matrix4d& A_slosh, 
                             Eigen::Matrix<double, 4, 2>& B_slosh) const;
    
    void update(double ax, double ay, double omega_z = 0.0, double alpha_z = 0.0);
    Eigen::Vector4d getSloshState() const;
    double getSloshHeight() const;
    Eigen::Matrix4d getSloshCostMatrix(double Q_slosh) const;
    const slosh_models::LiquidSloshModel::ModalParams& getModalParams() const;
    
    void writeToAugmentedState(StateVector& x_augmented) const;
    void readFromAugmentedState(const StateVector& x_augmented);
    
    Eigen::Vector4d predictSlosh(const Eigen::Vector4d& x_slosh_curr,
                                  double ax, double ay) const;

private:
    slosh_models::LiquidSloshModel slosh_model_;
    SloshParams params_;
    bool configured_ = false;
    
    Eigen::Matrix4d A_discrete_;
    Eigen::Matrix<double, 4, 2> B_discrete_;
};

}  // namespace scout_omni_local_planner
