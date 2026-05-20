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
    // 基础状态（4 维）- 直接 ω 控制模式
    static constexpr int E_L = 0;       // 纵向误差 (lag error)
    static constexpr int E_C = 1;       // 横向误差 (contour error)
    static constexpr int E_THETA = 2;   // 航向误差
    static constexpr int V = 3;         // 线速度
    // 注意：ω 不再是状态，而是控制量！
    
    // 晃动状态（4 维）- 来自 slosh_models::LiquidSloshModel
    // 状态向量: [xn, vxn, yn, vyn] (模态位移和速度)
    static constexpr int ETA_X = 4;      // X方向模态位移 [m]
    static constexpr int ETA_X_DOT = 5;  // X方向模态速度 [m/s]
    static constexpr int ETA_Y = 6;      // Y方向模态位移 [m]
    static constexpr int ETA_Y_DOT = 7;  // Y方向模态速度 [m/s]
    
    static constexpr int BASE_DIM = 4;      // 基础状态维度
    static constexpr int SLOSH_DIM = 4;     // 晃动状态维度
    static constexpr int TOTAL_DIM = BASE_DIM + SLOSH_DIM;  // 8维增广状态
};

struct ControlIndex {
    static constexpr int A = 0;         // 线加速度
    static constexpr int OMEGA = 1;     // 角速度（直接控制，更平滑！）
    
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
// 终点几何信息
//==============================================================================

struct GoalInfo {
    bool valid = false;

    // goal 在 base_link 坐标系下的位置
    double dx = 0.0;
    double dy = 0.0;
    double dist = 0.0;
    double bearing = 0.0;  // atan2(dy, dx)

    // 终点姿态信息（若可用）
    bool has_goal_yaw = false;
    double goal_yaw_in_base = 0.0;
    double goal_yaw_err = 0.0;  // 机器人在 base_link 下 yaw=0，因此与 goal_yaw_in_base 同号

    // 门限判断
    bool position_reached = false;
    bool pose_reached = false;
};

//==============================================================================
// MPC 参数结构
//==============================================================================

struct MPCParams {
    // 预测参数
    int N = 20;             // 预测步长
    double dt = 0.05;       // 时间步长 (s)
    double cmd_vel_lead_time = -1.0;  // 输出速度前瞻时间；<0 时沿用 0.5*dt
    
    // 状态权重
    double Q_el = 1.0;      // 纵向误差权重
    double Q_ec = 10.0;     // 横向误差权重（主要关注）
    double Q_etheta = 5.0;  // 航向误差权重
    double Q_v = 1.0;       // 速度误差权重

    // Contour + Lag 误差结构（可选）
    bool use_contour_lag = false;
    double Q_contour = 10.0; // 横向误差（contour）权重
    double Q_lag = 1.0;      // 纵向误差（lag）权重

    // 角速度前馈（基于曲率）
    bool enable_omega_ff = false;
    double Q_omega_ff = 0.0;

    // 终端权重放大（k==N）
    double terminal_factor_ec = 1.0;
    double terminal_factor_etheta = 1.0;
    double terminal_factor_v = 1.0;
    int terminal_ramp_steps = 1;  // 渐进式终端权重的步数（>1 时线性递增）
    
    // 控制权重
    double R_a = 1.0;       // 加速度权重
    double R_omega = 0.1;   // 角速度权重（直接控制）
    
    // 控制变化率权重
    double R_da = 0.1;      // 加速度变化权重
    double R_domega = 0.1;  // 角速度变化权重

    // 控制变化率约束（硬约束）
    bool constrain_omega_rate = true;   // 是否约束 Δω
    bool constrain_accel_rate = false;  // 是否约束 Δa（可选）
    
    // 晃动权重
    double Q_slosh = 0.0;   // 归一化晃动风险权重 (设为 0 表示不启用)
    double slosh_height_ref = 0.005;  // 参考晃动高度 (m)，用于归一化 Q_slosh
    double slosh_eta_dot_ratio = 0.3; // eta_dot 等效位移项相对 eta 项的比例；<=0 时使用手动 Q_slosh_eta_dot
    double slosh_preview_factor = 0.0; // 对 x_1..x_N 额外加权的 one-step preview 系数；0=关闭
    double slosh_height_max = 0.05;  // 液面高度约束 (m)
    bool enable_slosh_box_constraint = false;  // 是否启用一阶盒约束代理
    // 运行时由 local_planner_ros 计算: Q_slosh_eta = Q_slosh * height_coeff² / slosh_height_ref²
    // cost_function 直接用此值乘 ETA_X² / ETA_Y²
    double Q_slosh_eta = 0.0;
    double slosh_eta_bar = 0.0;  // 运行时换算得到的模态位移盒约束阈值
    // P1: eta_dot 速度代价（直接 QP 权重，不经 height_coeff 缩放）
    double Q_slosh_eta_dot = 0.0;
    // P2: terminal slosh 能量代价放大系数（在 terminal ramp 步叠加；需对应 base cost > 0 才生效）
    double terminal_factor_slosh_eta = 0.0;
    double terminal_factor_slosh_eta_dot = 0.0;
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
    double j_max = 0.0;     // 最大线加加速度 (m/s^3)，用于 Δa 约束
    
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
    double goal_reached_max_speed = 0.08;  // 判定 REACHED 的最大线速度 (m/s)
    double goal_reached_max_omega = 0.15;  // 判定 REACHED 的最大角速度 (rad/s)
    double goal_capture_distance = 0.4;   // 近终点捕获区半径 (m)
    double goal_capture_min_speed = 0.08; // 捕获区内最低参考速度 (m/s)
    double path_timeout = 5.0;        // 路径超时时间 (s)
    int min_path_points = 2;          // 最少路径点数
    std::string base_frame = "base_link";  // 机器人坐标系
    bool publish_smoothed_path = false;     // 是否发布平滑路径
    std::string smoothed_path_topic = "global_path_smooth";  // 平滑路径话题名
    int smoothed_path_points = 80;          // 平滑路径采样点数
    int window_back = 2;               // 样条拟合窗口：向后点数
    int window_forward = 2;            // 样条拟合窗口：向前额外点数
    double s_jump_threshold = 0.5;     // 弧长跳变阈值 (m)
    double resample_spacing = 0.0;     // 路径重采样间隔 (m)，<=0 表示不重采样
    double max_lat_accel = 0.0;        // 最大横向加速度 (m/s^2)，<=0 表示不限制
    double min_ref_speed = 0.0;        // 参考速度下限 (m/s)
    bool time_parameterize = false;   // 是否启用时间化速度规划 v(s)
    double speed_profile_ds = 0.05;   // 速度曲线采样间隔 (m)
    double max_tan_accel = 0.0;       // 最大切向加速度 (m/s^2)，<=0 表示不限制
    double max_tan_decel = 0.0;       // 最大切向减速度 (m/s^2)，<=0 表示不限制
    double goal_speed = 0.0;          // 末端期望速度 (m/s)
    bool use_bspline_smoothing = false; // 是否对局部窗口做 B-spline 平滑
    int bspline_samples_per_segment = 8; // B-spline 每段采样点数
    double path_change_threshold = 0.3;  // 路径相似性阈值 (m)，低于此值视为相同路径
    double speed_profile_omega_max = 0.0; // v(s) 规划用 ω_max (rad/s)，<=0 不限制
    double speed_profile_alpha_max = 0.0; // v(s) 规划用 α_max (rad/s²)，<=0 不限制
    bool energy_profile_enable = false;   // Step1: 几何低激励 profile 消融入口
    double energy_profile_lat_accel = 0.0; // PROFILE_ENERGY 横向加速度预算
    double energy_profile_omega_max = 0.0; // PROFILE_ENERGY 角速度预算
    double energy_profile_alpha_max = 0.0; // PROFILE_ENERGY 角加速度预算
    double energy_profile_ax_max = 0.0;    // PROFILE_ENERGY 切向加速度预算
    double energy_profile_decel_max = 0.0; // PROFILE_ENERGY 切向减速度预算
    double energy_profile_min_v = 0.0;     // PROFILE_ENERGY 最低参考速度
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
    SETTLING,   // 终点附近残余晃动收敛
    REACHED,    // 到达目标点
    ERROR       // 异常状态
};

inline std::string plannerStateToString(PlannerState state) {
    switch (state) {
        case PlannerState::IDLE:     return "IDLE";
        case PlannerState::TRACKING: return "TRACKING";
        case PlannerState::SETTLING: return "SETTLING";
        case PlannerState::REACHED:  return "REACHED";
        case PlannerState::ERROR:    return "ERROR";
        default:                     return "UNKNOWN";
    }
}

}  // namespace scout_local_planner
