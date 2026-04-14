/**
 * @file risk_scheduler.cpp
 * @brief ρ_k 风险自适应调度器实现
 */

#include "scout_local_planner/risk_scheduler.h"
#include <ros/ros.h>

namespace scout_local_planner {

void RiskScheduler::init(const RiskSchedulerParams& params) {
    params_ = params;

    // 初始化前一周期输出为低风险默认值
    prev_output_ = RiskSchedulerOutput();
    prev_output_.Q_eta_k     = params_.Q_eta_min;
    prev_output_.eta_bar_k   = params_.eta_bar_max;
    prev_output_.v_ref_eff_k = 0.0;  // 将在第一次 update() 中用 v_ref 填充
    prev_output_.fallback_active = false;

    u_high_count_ = 0;
    initialized_  = true;
}

double RiskScheduler::sigmoid(double x) {
    return 1.0 / (1.0 + std::exp(-x));
}

double RiskScheduler::clip(double val, double lo, double hi) {
    return std::max(lo, std::min(hi, val));
}

RiskSchedulerOutput RiskScheduler::update(
    double h_pred_max_prev,
    double E_slosh_prev,
    double d_goal,
    double a_y_imu,
    double v_omega,
    ros::Time imu_stamp,
    bool imu_bias_ready,
    double v_ref)
{
    if (!initialized_) {
        ROS_ERROR_ONCE("[RiskScheduler] update() called before init()");
        return prev_output_;
    }

    RiskSchedulerOutput out;

    // ── Fallback 检测 ──────────────────────────────────────────
    const bool imu_timeout   = (ros::Time::now() - imu_stamp).toSec() > params_.imu_timeout_s;
    const bool bias_not_ready = !imu_bias_ready;

    const double u_k_raw = clip(std::fabs(a_y_imu - v_omega) / params_.a_uncert_max, 0.0, 1.0);
    if (u_k_raw > params_.u_threshold_high) {
        ++u_high_count_;
    } else {
        u_high_count_ = 0;
    }
    const bool u_persistent = (u_high_count_ >= params_.u_high_count_trigger);

    out.fallback_active = imu_timeout || bias_not_ready || u_persistent;

    if (out.fallback_active) {
        out.Q_eta_k     = params_.Q_eta_fix;
        out.eta_bar_k   = params_.eta_bar_fix;
        out.v_ref_eff_k = v_ref;   // 速度不折扣，保持调用方传入值
        out.rho_k = out.r_k = out.u_k = out.s_rho = 0.0;
        prev_output_ = out;
        return out;
    }

    // ── r_k：physical risk（来自冻结 monitor rollout）─────────
    const double h_max  = params_.eta_bar_max;
    const double E_max  = h_max * h_max * 2.0;  // 两轴满量程估计

    const double h_risk = clip(h_pred_max_prev / (h_max + 1e-9), 0.0, 1.0);
    const double e_risk = clip(E_slosh_prev    / (E_max + 1e-9), 0.0, 1.0);
    const double t_risk = clip((params_.d_goal_thresh - d_goal) / (params_.d_goal_thresh + 1e-9),
                               0.0, 1.0);

    out.r_k = clip(params_.w_h * h_risk + params_.w_e * e_risk + params_.w_t * t_risk,
                   0.0, 1.0);

    // ── u_k：excitation uncertainty（独立 IMU 观测）───────────
    out.u_k = u_k_raw;

    // ── ρ_k ─────────────────────────────────────────────────
    out.rho_k = clip(params_.w_r * out.r_k + params_.w_u * out.u_k, 0.0, 1.0);
    out.s_rho  = sigmoid(params_.gamma * (out.rho_k - params_.rho_0));

    // ── 期望值 ───────────────────────────────────────────────
    const double Q_des    = params_.Q_eta_min + (params_.Q_eta_max - params_.Q_eta_min) * out.s_rho;
    const double bar_des  = params_.eta_bar_max - params_.delta_eta_bar * out.s_rho;
    const double vref_des = v_ref * (1.0 - params_.beta * out.s_rho);

    // ── 变化率限制（防止 QP 参数突变引起求解器抖动）────────
    const double dQ    = (params_.Q_eta_max - params_.Q_eta_min) * params_.rate_limit_per_step;
    const double dbar  = params_.delta_eta_bar * params_.rate_limit_per_step;
    const double dvref = v_ref * params_.beta * params_.rate_limit_per_step;

    // 初始化时 prev_output_.v_ref_eff_k = 0，用 v_ref 替换以避免第一步剧烈变化
    const double prev_vref = (prev_output_.v_ref_eff_k < 1e-9) ? v_ref : prev_output_.v_ref_eff_k;

    out.Q_eta_k   = clip(Q_des,
                         prev_output_.Q_eta_k   - dQ,
                         prev_output_.Q_eta_k   + dQ);
    out.eta_bar_k = clip(bar_des,
                         prev_output_.eta_bar_k - dbar,
                         prev_output_.eta_bar_k + dbar);
    out.v_ref_eff_k = clip(vref_des,
                           prev_vref - dvref,
                           prev_vref + dvref);

    prev_output_ = out;
    return out;
}

}  // namespace scout_local_planner
