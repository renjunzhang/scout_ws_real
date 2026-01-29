/**
 * @file types.h
 * @brief MPC 局部规划器的类型定义
 * 
 * 包含状态向量、控制向量、参考点等核心类型定义
 */

#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace scout_local_planner {

//==============================================================================
// 状态索引定义
//==============================================================================

struct StateIndex {
    // 基础状态（第 1 步：5 维）
    static constexpr int E_L = 0;       // 纵向误差 (lag error)
    static constexpr int E_C = 1;       // 横向误差 (contour error)
    static constexpr int E_THETA = 2;   // 航向误差
    static constexpr int V = 3;         // 线速度
    static constexpr int OMEGA = 4;     // 角速度
    
    // 晃动状态（第 2 步添加：4 维）
    // static constexpr int ETA_X = 5;      // X方向模态位移
    // static constexpr int ETA_X_DOT = 6;  // X方向模态速度
    // static constexpr int ETA_Y = 7;      // Y方向模态位移
    // static constexpr int ETA_Y_DOT = 8;  // Y方向模态速度
    
    static constexpr int BASE_DIM = 5;      // 基础状态维度
    static constexpr int SLOSH_DIM = 0;     // 晃动状态维度（第 2 步改为 4）
    static constexpr int TOTAL_DIM = BASE_DIM + SLOSH_DIM;
};

struct ControlIndex {
    static constexpr int A = 0;         // 线加速度
    static constexpr int ANG_ACC = 1;   // 角加速度 (避免与 OSQP 的 ALPHA 宏冲突)
    
    static constexpr int DIM = 2;       // 控制维度
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
    
    // 路径切向角（用于计算 e_θ = θ_robot - θ_path）
    double theta_path = 0.0;
    
    // 路径曲率（用于 Frenet 动力学）
    double kappa = 0.0;
    
    // 路径推进速度（可设为 v_des 或根据曲率调整）
    double v_path = 0.0;
    
    // 弧长参数
    double s = 0.0;
    
    // 参考速度（只用于代价函数，不用于动力学！）
    double v_ref = 0.0;
};

//==============================================================================
// Frenet 误差状态
//==============================================================================

struct FrenetState {
    double e_l = 0.0;       // 纵向误差
    double e_c = 0.0;       // 横向误差
    double e_theta = 0.0;   // 航向误差
    
    // 转换为状态向量（前 3 维）
    Eigen::Vector3d toVector() const {
        return Eigen::Vector3d(e_l, e_c, e_theta);
    }
};

//==============================================================================
// MPC 参数结构
//==============================================================================

struct MPCParams {
    // 预测参数
    int N = 20;             // 预测步长
    double dt = 0.05;       // 时间步长 (s)
    
    // 状态权重
    double Q_el = 1.0;      // 纵向误差权重
    double Q_ec = 10.0;     // 横向误差权重（主要关注）
    double Q_etheta = 5.0;  // 航向误差权重
    double Q_v = 1.0;       // 速度误差权重
    
    // 控制权重
    double R_a = 1.0;       // 加速度权重
    double R_alpha = 1.0;   // 角加速度权重
    
    // 控制变化率权重
    double R_da = 0.1;      // 加速度变化权重
    double R_dalpha = 0.1;  // 角加速度变化权重
    
    // 晃动权重（第 2 步启用）
    double Q_slosh = 0.0;   // 设为 0 表示不启用
    double slosh_height_max = 0.05;  // 液面高度约束 (m)
};

//==============================================================================
// 车辆参数结构
//==============================================================================

struct VehicleParams {
    // 速度约束
    double v_max = 1.0;     // 最大线速度 (m/s)
    double v_min = -0.3;    // 最小线速度 (m/s)，负值表示倒车
    double omega_max = 1.0; // 最大角速度 (rad/s)
    
    // 加速度约束
    double a_max = 0.5;     // 最大线加速度 (m/s²)
    double alpha_max = 1.0; // 最大角加速度 (rad/s²)
    
    // 车辆几何
    double track_width = 0.456;  // Scout Mini 轮距 (m)
    double wheelbase = 0.0;      // 差速轮设为 0
};

//==============================================================================
// 路径处理参数
//==============================================================================

struct PathHandlerParams {
    double lookahead_distance = 1.0;  // 前视距离 (m)
    double goal_tolerance = 0.1;      // 到达目标容差 (m)
    double yaw_tolerance = 0.1;       // 航向容差 (rad)
    double path_timeout = 5.0;        // 路径超时时间 (s)
    int min_path_points = 2;          // 最少路径点数
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
    double v_cmd = 0.0;
    double omega_cmd = 0.0;
    
    // 求解时间 (ms)
    double solve_time_ms = 0.0;
};

//==============================================================================
// 节点状态枚举
//==============================================================================

enum class PlannerState {
    IDLE,       // 等待全局路径
    TRACKING,   // 正在跟踪路径
    REACHED,    // 到达目标点
    ERROR       // 异常状态
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

}  // namespace scout_local_planner
