/**
 * @file risk_scheduler.h
 * @brief ρ_k 风险自适应调度器
 *
 * 统一调度 Q_eta、η̄、v_ref_eff 三个 QP 参数，
 * 外层 20 Hz 循环调用，内层 MPC 求解器每步使用当前参数。
 *
 * 风险度量：
 *   r_k = w_h * h_risk + w_e * e_risk + w_t * t_risk   (physical risk)
 *   u_k = |a_y_imu - v*ω| / a_uncert_max               (excitation uncertainty)
 *   ρ_k = clip(w_r * r_k + w_u * u_k, 0, 1)
 *   s_ρ  = sigmoid(γ*(ρ_k - ρ_0))
 *
 * fallback 条件（任一触发）：
 *   - IMU 超时（最近 IMU 时间戳距今 > imu_timeout_s）
 *   - bias 未完成估计
 *   - u_k 持续高于阈值（连续 u_high_count_trigger 步）
 */

#pragma once

#include <ros/time.h>
#include <cmath>
#include <algorithm>

namespace scout_local_planner {

struct RiskSchedulerParams {
    // sigmoid 参数
    double gamma = 5.0;
    double rho_0 = 0.3;

    // 变化率限制（每周期最大变化量占满量程比例）
    double rate_limit_per_step = 0.05;

    // 调度范围
    double Q_eta_min = 0.0;
    double Q_eta_max = 10.0;
    double eta_bar_max = 0.05;     // 应与 slosh_height_max 一致
    double delta_eta_bar = 0.02;   // 高风险时 eta_bar 收紧量
    double beta = 0.3;             // 速度折扣上限

    // 风险权重
    double w_h = 0.4;   // 液面预测风险
    double w_e = 0.3;   // 模态能量风险
    double w_t = 0.3;   // 终点接近度风险
    double w_r = 0.7;   // r_k 权重
    double w_u = 0.3;   // u_k 权重

    // fallback 触发阈值
    double u_threshold_high = 0.8;
    int    u_high_count_trigger = 10;
    double imu_timeout_s = 0.1;
    double a_uncert_max = 0.5;     // u_k 归一化分母 (m/s²)

    // fallback 固定参数（回退到保守固定值）
    double Q_eta_fix = 5.0;
    double eta_bar_fix = 0.04;

    // 终点接近距离阈值
    double d_goal_thresh = 1.0;
};

struct RiskSchedulerOutput {
    double Q_eta_k = 0.0;
    double eta_bar_k = 0.05;
    double v_ref_eff_k = 0.0;
    double rho_k = 0.0;
    double r_k = 0.0;
    double u_k = 0.0;
    double s_rho = 0.0;
    bool   fallback_active = false;
};

class RiskScheduler {
public:
    /**
     * @brief 初始化调度器参数
     */
    void init(const RiskSchedulerParams& params);

    /**
     * @brief outer loop 每周期调用一次
     *
     * @param h_pred_max_prev  上一周期 MPC 预测的液面高度最大值 [m]
     * @param E_slosh_prev     上一周期模态能量 η_x² + η_y² [m²]
     * @param d_goal           当前到目标点距离 [m]
     * @param a_y_imu          IMU 横向加速度（已去零偏）[m/s²]
     * @param v_omega          运动学离心估计 v*ω [m/s²]
     * @param imu_stamp        最新 IMU 时间戳（用于超时检测）
     * @param imu_bias_ready   bias 是否已完成估计
     * @param v_ref            当前基础参考速度（未折扣）[m/s]
     */
    RiskSchedulerOutput update(
        double h_pred_max_prev,
        double E_slosh_prev,
        double d_goal,
        double a_y_imu,
        double v_omega,
        ros::Time imu_stamp,
        bool imu_bias_ready,
        double v_ref
    );

private:
    double sigmoid(double x);
    double clip(double val, double lo, double hi);

    RiskSchedulerParams params_;
    RiskSchedulerOutput prev_output_;
    int  u_high_count_ = 0;
    bool initialized_ = false;
};

}  // namespace scout_local_planner
