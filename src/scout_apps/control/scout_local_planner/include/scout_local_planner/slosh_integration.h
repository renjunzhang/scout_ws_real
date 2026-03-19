/**
 * @file slosh_integration.h
 * @brief 液体晃动模型与 MPC 的集成接口
 * 
 * 提供晃动模型配置、状态更新、离散矩阵获取等功能，
 * 使 MPC 能够在增广状态空间中考虑液体晃动动力学。
 * 
 * 增广状态空间 (8维，直接 ω 控制模式):
 *   x = [e_l, e_c, e_θ, v, η_x, η̇_x, η_y, η̇_y]ᵀ
 *   u = [a, ω]ᵀ
 * 
 * 晃动动力学 (独立子系统):
 *   x_slosh[k+1] = A_slosh * x_slosh[k] + B_slosh * [ax, ay]ᵀ
 *   其中 ay = v * ω (离心加速度)
 * 
 * 其中 A_slosh (4x4), B_slosh (4x2) 来自 LiquidSloshModel::getDiscreteMatrices()
 */

#pragma once

#include "scout_local_planner/types.h"
#include <slosh_models/liquid_slosh_model.h>
#include <Eigen/Dense>
#include <memory>

namespace scout_local_planner {

/**
 * @brief 晃动参数结构 (从 YAML 加载)
 */
struct SloshParams {
    // 容器参数
    double container_radius = 0.014;  ///< 28 mm 试管内半径 [m]
    double liquid_height = 0.055;     ///< 当前试管液体静液高度 [m]
    double liquid_density = 1000.0;   ///< 液体密度 [kg/m³]
    
    // 模态参数
    int mode_index = 1;               ///< 使用的模态阶数 (1=第一阶)
    double damping_ratio = 0.12;      ///< 28 mm 水试管初始阻尼比
    
    // 偏心参数 (容器相对底盘旋转中心)
    double offset_x = 0.0;            ///< 容器偏心距 x [m]
    double offset_y = 0.0;            ///< 容器偏心距 y [m]
    
    // 模型选项
    bool use_linear_model = true;     ///< true=线性模型(L), false=非线性模型(NL)
    bool use_parabola_term = true;    ///< 是否启用旋转抛物面高度修正 (Lp 模型)
    
    // 采样周期 (应与 MPC dt 一致)
    double dt = 0.05;                 ///< 离散化采样周期 [s]
};

/**
 * @brief 晃动集成类
 * 
 * 封装 slosh_models::LiquidSloshModel，提供 MPC 所需的接口：
 * - 配置模型参数
 * - 获取离散状态空间矩阵 (用于增广 MPC 动力学)
 * - 更新晃动状态 (用于状态估计)
 * - 计算液面高度 (用于约束检查)
 */
class SloshIntegration {
public:
    SloshIntegration();
    ~SloshIntegration() = default;
    
    /**
     * @brief 配置晃动模型
     * @param params 晃动参数
     * @return true 成功, false 失败
     */
    bool configure(const SloshParams& params);
    
    /**
     * @brief 检查模型是否已配置
     */
    bool isConfigured() const { return configured_; }
    
    /**
     * @brief 重置晃动状态为零
     */
    void reset();
    
    /**
     * @brief 获取晃动状态维度
     */
    static constexpr int sloshDim() { return StateIndex::SLOSH_DIM; }
    
    /**
     * @brief 获取离散状态空间矩阵
     * @param[out] A_slosh 状态转移矩阵 (4x4)
     * @param[out] B_slosh 输入矩阵 (4x2)
     * 
     * 晃动动力学: x_slosh[k+1] = A_slosh * x_slosh[k] + B_slosh * [ax, ay]ᵀ
     */
    void getDiscreteMatrices(Eigen::Matrix4d& A_slosh, 
                             Eigen::Matrix<double, 4, 2>& B_slosh) const;
    
    /**
     * @brief 更新晃动状态
     * @param ax 底盘 x 方向加速度 [m/s²]
     * @param ay 底盘 y 方向加速度 [m/s²]
     * @param omega_z 底盘角速度 [rad/s] (用于旋转修正)
     * @param alpha_z 底盘角加速度 [rad/s²] (用于旋转修正)
     */
    void update(double ax, double ay, double omega_z = 0.0, double alpha_z = 0.0);
    
    /**
     * @brief 获取当前晃动状态
     * @return [η_x, η̇_x, η_y, η̇_y]
     */
    Eigen::Vector4d getSloshState() const;
    
    /**
     * @brief 计算液面晃动高度
     * @return η [m] (液面相对静止位置的最大抬升高度)
     */
    double getSloshHeight() const;
    
    /**
     * @brief 计算液面高度二次代价系数
     * 
     * 晃动代价: J_slosh = Q_slosh * η² = Q_slosh * (h_coeff * ||x_slosh||)²
     *         = x_slosh' * H_slosh * x_slosh
     * 
     * @param Q_slosh 晃动惩罚权重
     * @return H_slosh (4x4) 晃动代价二次系数矩阵
     */
    Eigen::Matrix4d getSloshCostMatrix(double Q_slosh) const;
    
    /**
     * @brief 获取模态参数
     */
    const slosh_models::LiquidSloshModel::ModalParams& getModalParams() const;
    
    /**
     * @brief 将晃动状态写入增广状态向量
     */
    void writeToAugmentedState(StateVector& x_augmented) const;
    
    /**
     * @brief 从增广状态向量读取晃动状态
     */
    void readFromAugmentedState(const StateVector& x_augmented);
    
    /**
     * @brief 预测一步晃动状态 (用于 MPC 名义轨迹)
     * @param x_slosh_curr 当前晃动状态 [η_x, η̇_x, η_y, η̇_y]
     * @param ax 预测的 x 加速度
     * @param ay 预测的 y 加速度
     * @return 下一步晃动状态
     */
    Eigen::Vector4d predictSlosh(const Eigen::Vector4d& x_slosh_curr,
                                  double ax, double ay) const;

private:
    slosh_models::LiquidSloshModel slosh_model_;
    SloshParams params_;
    bool configured_ = false;
    
    // 缓存的离散矩阵
    Eigen::Matrix4d A_discrete_;
    Eigen::Matrix<double, 4, 2> B_discrete_;
};

// ============================================================================
// 辅助函数：构建增广动力学矩阵
// ============================================================================

/**
 * @brief 构建增广状态转移矩阵
 * 
 * 增广系统:
 * | x_base[k+1]  |   | A_base    0      | | x_base[k]  |   | B_base | | u[k] |   | c_base |
 * |              | = |                  | |            | + |        | |      | + |        |
 * | x_slosh[k+1] |   | 0      A_slosh   | | x_slosh[k] |   | B_accel| | u[k] |   | 0      |
 * 
 * @param A_base 基础状态转移矩阵 (5x5)
 * @param A_slosh 晃动状态转移矩阵 (4x4)
 * @return 增广状态转移矩阵 (9x9)
 */
inline Eigen::Matrix<double, StateIndex::TOTAL_DIM, StateIndex::TOTAL_DIM>
buildAugmentedA(const Eigen::MatrixXd& A_base, const Eigen::Matrix4d& A_slosh) {
    Eigen::Matrix<double, StateIndex::TOTAL_DIM, StateIndex::TOTAL_DIM> A_aug;
    A_aug.setZero();
    
    // 基础状态部分
    A_aug.topLeftCorner(StateIndex::BASE_DIM, StateIndex::BASE_DIM) = A_base;
    
    // 晃动状态部分
    A_aug.bottomRightCorner(StateIndex::SLOSH_DIM, StateIndex::SLOSH_DIM) = A_slosh;
    
    return A_aug;
}

/**
 * @brief 构建增广输入矩阵
 * 
 * 控制输入 u = [a, α] 影响：
 * - 基础状态：通过 B_base
 * - 晃动状态：加速度 a 产生 [a, 0] 作为晃动输入 (假设纯纵向加速)
 *             若考虑旋转，需额外计算横向加速度分量
 * 
 * @param B_base 基础输入矩阵 (5x2)
 * @param B_slosh 晃动输入矩阵 (4x2)
 * @return 增广输入矩阵 (9x2)
 */
inline Eigen::Matrix<double, StateIndex::TOTAL_DIM, ControlIndex::DIM>
buildAugmentedB(const Eigen::MatrixXd& B_base, 
                const Eigen::Matrix<double, 4, 2>& B_slosh) {
    Eigen::Matrix<double, StateIndex::TOTAL_DIM, ControlIndex::DIM> B_aug;
    B_aug.setZero();
    
    // 基础状态对控制的响应
    B_aug.topRows(StateIndex::BASE_DIM) = B_base;
    
    // 晃动状态对加速度的响应
    // 这里简化假设：线加速度 a 主要产生 x 方向加速度
    // B_slosh 的输入是 [ax, ay]，我们将 a 映射到 ax
    // 完整实现需要考虑机器人朝向和角速度产生的离心力
    B_aug(StateIndex::ETA_X, ControlIndex::A) = B_slosh(0, 0);      // ∂η_x/∂ax * ∂ax/∂a
    B_aug(StateIndex::ETA_X_DOT, ControlIndex::A) = B_slosh(1, 0);  // ∂η̇_x/∂ax * ∂ax/∂a
    
    // 角加速度产生的横向加速度贡献（二阶效应，可选）
    // ay ≈ r * alpha + ... (离心力等，暂时忽略)
    
    return B_aug;
}

}  // namespace scout_local_planner
