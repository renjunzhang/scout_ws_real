/**
 * @file slosh_feedback.h
 * @brief Odom/IMU acceleration feedback used by the slosh model.
 */

#pragma once

#include <ros/ros.h>

#include <string>
#include <vector>

namespace scout_local_planner {

struct SloshFeedbackParams {
    double accel_filter_alpha = 0.3;
    bool use_imu_lateral_accel = false;
    bool use_imu_yaw_rate = true;
    bool use_imu_alpha_z = false;
    std::string imu_topic = "/imu/data";
    double imu_filter_alpha = 0.3;
    bool imu_ay_bias_compensation_enable = true;
    double imu_ay_bias_init_duration = 3.0;
    double imu_ay_bias_static_v_max = 0.03;
    double imu_ay_bias_static_omega_max = 0.03;
    int imu_ay_bias_min_samples = 100;
    double imu_ay_bias_estimator_alpha = 0.15;
    double imu_ay_bias_trim_ratio = 0.10;
    double imu_ay_scale = 1.0;
};

struct SloshFeedbackOutput {
    double ax = 0.0;
    double ay = 0.0;
    double alpha = 0.0;
    double omega = 0.0;
    double odom_ay = 0.0;
    double odom_alpha = 0.0;
    bool has_imu = false;
    bool has_prev_imu = false;
    double imu_ay_filtered = 0.0;
    double imu_ay_bias = 0.0;
    double imu_omega_z_filtered = 0.0;
    bool imu_ay_bias_ready = false;
};

class SloshFeedback {
public:
    void setParams(const SloshFeedbackParams& params);
    const SloshFeedbackParams& params() const { return params_; }
    bool imuRequired() const;
    const std::string& imuTopic() const { return params_.imu_topic; }

    void onOdom(double v, double omega, const ros::Time& stamp);
    void onImu(double ay_raw,
               double omega_z_raw,
               const ros::Time& stamp,
               double current_v,
               double current_omega,
               bool has_odom);
    SloshFeedbackOutput update(double current_v,
                               double current_omega,
                               const ros::Time& current_odom_time);
    void resetOdomFilters();
    const SloshFeedbackOutput& output() const { return output_; }

private:
    static double trimmedMean(std::vector<double> samples, double trim_ratio);

    SloshFeedbackParams params_;
    SloshFeedbackOutput output_;

    double prev_v_ = 0.0;
    double prev_omega_ = 0.0;
    ros::Time prev_odom_time_;
    bool has_prev_odom_ = false;
    double ax_filtered_ = 0.0;
    double ay_filtered_ = 0.0;
    double alpha_filtered_ = 0.0;

    bool has_imu_ = false;
    bool has_prev_imu_ = false;
    ros::Time prev_imu_time_;
    double imu_ay_unbiased_ = 0.0;
    bool imu_ay_bias_window_started_ = false;
    bool imu_ay_bias_window_closed_ = false;
    bool imu_ay_bias_window_ema_initialized_ = false;
    ros::Time imu_ay_bias_window_start_;
    double imu_ay_bias_window_ema_ = 0.0;
    std::vector<double> imu_ay_bias_samples_;
    double imu_alpha_filtered_ = 0.0;
    double prev_imu_omega_z_ = 0.0;
};

}  // namespace scout_local_planner
