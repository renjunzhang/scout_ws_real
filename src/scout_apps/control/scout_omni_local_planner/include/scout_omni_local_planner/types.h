/**
 * @file types.h
 * @brief 全向轮 MPC 局部规划器的类型定义
 * 
 * 全向轮底盘（Mecanum / 全向轮）：
 *   - 可在任意方向平移 + 旋转，三自由度
 *   - 状态增加 v_y（横向速度），控制增加 a_y（横向加速度）
 *   
 * 基础状态 (5维): [e_l, e_c, e_θ, v_x, v_y]
 * 晃动状态 (4维): [η_x, η̇_x, η_y, η̇_y]
 * 增广状态 (9维)
 * 控制 (3维): [a_x, a_y, ω]
 */

#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace scout_omni_local_planner {

//==============================================================================
// 状态索引定义
//==============================================================================

struct StateIndex {
    // 基础状态（5 维）- 全向轮
    static constexpr int E_L = 0;       // 纵向误差 (lag error)
    static constexpr int E_C = 1;       // 横向误差 (contour error)
    static constexpr int E_THETA = 2;   // 航向误差
    static constexpr int V_X = 3;       // 纵向速度（body frame X）
    static constexpr int V_Y = 4;       // 横向速度（body frame Y）
    
    // 晃动状态（4 维）
    static constexpr int ETA_X = 5;      // X方向模态位移 [m]
    static constexpr int ETA_X_DOT = 6;  // X方向模态速度 [m/s]
    static constexpr int ETA_Y = 7;      // Y方向模态位移 [m]
    static constexpr int ETA_Y_DOT = 8;  // Y方向模态速度 [m/s]
    
    static constexpr int BASE_DIM = 5;      // 基础状态维度
    static constexpr int SLOSH_DIM = 4;     // 晃动状态维度
    static constexpr int TOTAL_DIM = BASE_DIM + SLOSH_DIM;  // 9维增广状态
};

struct ControlIndex {
    static constexpr int A_X = 0;       // 纵向加速度 (body frame)
    static constexpr int A_Y = 1;       // 横向加速度 (body frame)
    static constexpr int OMEGA = 2;     // 角速度（直接控制）
    
    static constexpr int DIM = 3;       // 控制维度
};

//==============================================================================
// 向量类型定义
//==============================================================================

using StateVector = Eigen::Matrix<double, StateIndex::TOTAL_DIM, 1>;
using ControlVector = Eigen::Matrix<double, ControlIndex::DIM, 1>;

// 动态大小版本（用于 QP 求解）
using StateVectorX = Eigen::VectorXd;
using ControlVectorX = Eigen::VectorXd;

//==============================================================================
// 参考点结构
//==============================================================================

struct ReferencePoint {
    // 路径点位置（base_link 坐标系）
    double x = 0.0;
    double y = 0.0;
    
    // 路径切向角
    double theta_path = 0.0;
    
    // 路径曲率
    double kappa = 0.0;
    
    // 路径推进速度
    double v_path = 0.0;
    
    // 弧长参数
    double s = 0.0;
    
    // 参考纵向速度
    double v_ref = 0.0;
    
    // 参考横向速度（全向轮：通常为 0，但可用于主动消除横向误差）
    double vy_ref = 0.0;
};

//==============================================================================
// Frenet 误差状态
//==============================================================================

struct FrenetState {
    double e_l = 0.0;       // 纵向误差
    double e_c = 0.0;       // 横向误差
    double e_theta = 0.0;   // 航向误差
    
    Eigen::Vector3d toVector() const {
        return Eigen::Vector3d(e_l, e_c, e_theta);
    }
};

//==============================================================================
// MPC 参数结构
//==============================================================================

struct MPCParams {
    // 预测参数
    int N = 20;
    double dt = 0.05;
    
    // 状态权重
    double Q_el = 1.0;       // 纵向误差权重
    double Q_ec = 10.0;      // 横向误差权重
    double Q_etheta = 5.0;   // 航向误差权重
    double Q_vx = 1.0;       // 纵向速度误差权重
    double Q_vy = 5.0;       // 横向速度权重（惩罚不必要的横移）

    // Contour + Lag 误差结构（可选）
    bool use_contour_lag = false;
    double Q_contour = 10.0;
    double Q_lag = 1.0;

    // 角速度前馈（基于曲率）
    bool enable_omega_ff = false;
    double Q_omega_ff = 0.0;

    // 终端权重放大
    double terminal_factor_ec = 1.0;
    double terminal_factor_etheta = 1.0;
    double terminal_factor_vx = 1.0;
    
    // 控制权重
    double R_ax = 1.0;      // 纵向加速度权重
    double R_ay = 1.0;      // 横向加速度权重
    double R_omega = 0.1;   // 角速度权重
    
    // 控制变化率权重
    double R_dax = 0.1;     // 纵向加速度变化权重
    double R_day = 0.1;     // 横向加速度变化权重
    double R_domega = 0.1;  // 角速度变化权重

    // 控制变化率约束（硬约束）
    bool constrain_omega_rate = true;
    bool constrain_accel_rate = false;
    
    // 晃动权重
    double Q_slosh = 0.0;
    double slosh_height_max = 0.05;
};

//==============================================================================
// 车辆参数结构
//==============================================================================

struct VehicleParams {
    // 速度约束
    double vx_max = 1.0;     // 最大纵向速度 (m/s)
    double vx_min = -0.3;    // 最小纵向速度 (m/s)
    double vy_max = 0.5;     // 最大横向速度 (m/s)
    double omega_max = 1.0;  // 最大角速度 (rad/s)
    
    // 加速度约束
    double ax_max = 0.5;     // 最大纵向加速度 (m/s²)
    double ay_max = 0.5;     // 最大横向加速度 (m/s²)
    double alpha_max = 1.0;  // 最大角加速度 (rad/s²)
    double jx_max = 0.0;     // 最大纵向加加速度 (m/s^3)
    
    // 车辆几何
    double track_width = 0.456;   // 轮距 (m)
    double wheelbase = 0.451;     // 轴距 (m)（全向轮需要）
    double wheel_radius = 0.09;   // 轮半径 (m)
};

//==============================================================================
// 路径处理参数
//==============================================================================

struct PathHandlerParams {
    double lookahead_distance = 1.0;
    double goal_tolerance = 0.1;
    double yaw_tolerance = 0.1;
    double path_timeout = 5.0;
    int min_path_points = 2;
    std::string base_frame = "base_link";
    bool publish_smoothed_path = false;
    std::string smoothed_path_topic = "global_path_smooth";
    int smoothed_path_points = 80;
    int window_back = 2;
    int window_forward = 2;
    double s_jump_threshold = 0.5;
    double resample_spacing = 0.0;
    double max_lat_accel = 0.0;
    double min_ref_speed = 0.0;
    bool time_parameterize = false;
    double speed_profile_ds = 0.05;
    double max_tan_accel = 0.0;
    double max_tan_decel = 0.0;
    double goal_speed = 0.0;
    bool use_bspline_smoothing = false;
    int bspline_samples_per_segment = 8;
};

//==============================================================================
// MPC 求解结果
//==============================================================================

struct MPCSolution {
    bool success = false;
    std::string status_msg;
    
    // 最优控制序列
    std::vector<ControlVector> u_optimal;
    
    // 预测状态轨迹
    std::vector<StateVector> x_predicted;
    
    // 第一个控制量（实际执行）
    ControlVector u_first = ControlVector::Zero();
    
    // 预测的下一步速度（用于 cmd_vel）
    double vx_cmd = 0.0;
    double vy_cmd = 0.0;
    double omega_cmd = 0.0;
    
    // 求解时间 (ms)
    double solve_time_ms = 0.0;
};

//==============================================================================
// 节点状态枚举
//==============================================================================

enum class PlannerState {
    IDLE,
    TRACKING,
    REACHED,
    ERROR
};

inline std::string plannerStateToString(PlannerState state) {
    switch (state) {
        case PlannerState::IDLE:     return "IDLE";
        case PlannerState::TRACKING: return "TRACKING";
        case PlannerState::REACHED:  return "REACHED";
        case PlannerState::ERROR:    return "ERROR";
        default:                     return "UNKNOWN";
    }
}

}  // namespace scout_omni_local_planner
