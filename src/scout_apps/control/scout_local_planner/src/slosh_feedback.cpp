#include "scout_local_planner/slosh_feedback.h"

#include <algorithm>
#include <cmath>

namespace scout_local_planner {

void SloshFeedback::setParams(const SloshFeedbackParams& params) {
    params_ = params;
}

bool SloshFeedback::imuRequired() const {
    return params_.use_imu_lateral_accel ||
           params_.use_imu_yaw_rate ||
           params_.use_imu_alpha_z;
}

void SloshFeedback::onOdom(double v, double omega, const ros::Time& stamp) {
    output_.odom_ay = v * omega;
    prev_v_ = has_prev_odom_ ? prev_v_ : v;
    prev_omega_ = has_prev_odom_ ? prev_omega_ : omega;
    if (!stamp.isZero() && prev_odom_time_.isZero()) {
        prev_odom_time_ = stamp;
    }
}

void SloshFeedback::onImu(double ay_raw,
                          double omega_z_raw,
                          const ros::Time& stamp,
                          double current_v,
                          double current_omega,
                          bool has_odom) {
    bool ay_bias_just_initialized = false;

    if (params_.imu_ay_bias_compensation_enable &&
        !output_.imu_ay_bias_ready &&
        !imu_ay_bias_window_closed_) {
        const bool static_for_bias =
            has_odom &&
            std::abs(current_v) < params_.imu_ay_bias_static_v_max &&
            std::abs(current_omega) < params_.imu_ay_bias_static_omega_max;

        if (static_for_bias) {
            if (!imu_ay_bias_window_started_) {
                imu_ay_bias_window_started_ = true;
                imu_ay_bias_window_start_ = stamp;
                imu_ay_bias_window_ema_initialized_ = false;
                imu_ay_bias_window_ema_ = 0.0;
                imu_ay_bias_samples_.clear();
            }

            if (!imu_ay_bias_window_ema_initialized_) {
                imu_ay_bias_window_ema_ = ay_raw;
                imu_ay_bias_window_ema_initialized_ = true;
            } else {
                imu_ay_bias_window_ema_ =
                    params_.imu_ay_bias_estimator_alpha * ay_raw +
                    (1.0 - params_.imu_ay_bias_estimator_alpha) * imu_ay_bias_window_ema_;
            }
            imu_ay_bias_samples_.push_back(imu_ay_bias_window_ema_);
        } else if (imu_ay_bias_window_started_) {
            const double elapsed = (stamp - imu_ay_bias_window_start_).toSec();
            const int min_samples = std::max(1, params_.imu_ay_bias_min_samples);
            const int sample_count = static_cast<int>(imu_ay_bias_samples_.size());

            if (elapsed >= params_.imu_ay_bias_init_duration && sample_count >= min_samples) {
                output_.imu_ay_bias =
                    trimmedMean(imu_ay_bias_samples_, params_.imu_ay_bias_trim_ratio);
                output_.imu_ay_bias_ready = true;
                ay_bias_just_initialized = true;
                ROS_INFO("[SloshFeedback] IMU ay bias initialized: bias=%.5f, samples=%d, static_window=%.3fs",
                         output_.imu_ay_bias, sample_count, elapsed);
            } else {
                ROS_WARN("[SloshFeedback] IMU ay bias not initialized: static window too short (elapsed=%.3fs, samples=%d)",
                         elapsed, sample_count);
            }

            imu_ay_bias_window_closed_ = true;
            imu_ay_bias_window_started_ = false;
            imu_ay_bias_window_ema_initialized_ = false;
            imu_ay_bias_samples_.clear();
        } else if (has_odom) {
            imu_ay_bias_window_closed_ = true;
            ROS_WARN("[SloshFeedback] IMU ay bias not initialized: robot moved before the first static window.");
        }
    }

    const double ay_bias =
        (params_.imu_ay_bias_compensation_enable && output_.imu_ay_bias_ready)
            ? output_.imu_ay_bias
            : 0.0;
    imu_ay_unbiased_ = (ay_raw - ay_bias) * params_.imu_ay_scale;

    if (!has_imu_) {
        output_.imu_ay_filtered = imu_ay_unbiased_;
        output_.imu_omega_z_filtered = omega_z_raw;
        imu_alpha_filtered_ = 0.0;
        prev_imu_omega_z_ = omega_z_raw;
        prev_imu_time_ = stamp;
        has_imu_ = true;
        has_prev_imu_ = true;
        output_.has_imu = true;
        output_.has_prev_imu = true;
        return;
    }

    if (ay_bias_just_initialized) {
        output_.imu_ay_filtered = imu_ay_unbiased_;
    } else {
        output_.imu_ay_filtered =
            params_.imu_filter_alpha * imu_ay_unbiased_ +
            (1.0 - params_.imu_filter_alpha) * output_.imu_ay_filtered;
    }
    output_.imu_omega_z_filtered =
        params_.imu_filter_alpha * omega_z_raw +
        (1.0 - params_.imu_filter_alpha) * output_.imu_omega_z_filtered;

    if (has_prev_imu_) {
        const double dt_imu = (stamp - prev_imu_time_).toSec();
        if (dt_imu > 1e-4 && dt_imu < 1.0) {
            const double alpha_raw = (omega_z_raw - prev_imu_omega_z_) / dt_imu;
            imu_alpha_filtered_ =
                params_.imu_filter_alpha * alpha_raw +
                (1.0 - params_.imu_filter_alpha) * imu_alpha_filtered_;
        }
    }

    prev_imu_omega_z_ = omega_z_raw;
    prev_imu_time_ = stamp;
    has_imu_ = true;
    has_prev_imu_ = true;
    output_.has_imu = true;
    output_.has_prev_imu = true;
}

SloshFeedbackOutput SloshFeedback::update(double current_v,
                                          double current_omega,
                                          const ros::Time& current_odom_time) {
    if (has_prev_odom_ && !prev_odom_time_.isZero() && !current_odom_time.isZero()) {
        const double dt_odom = (current_odom_time - prev_odom_time_).toSec();
        if (dt_odom > 1e-4 && dt_odom < 1.0) {
            const double ax_raw = (current_v - prev_v_) / dt_odom;
            const double alpha_raw = (current_omega - prev_omega_) / dt_odom;
            const double ay_raw = current_v * current_omega;

            ax_filtered_ =
                params_.accel_filter_alpha * ax_raw +
                (1.0 - params_.accel_filter_alpha) * ax_filtered_;
            ay_filtered_ =
                params_.accel_filter_alpha * ay_raw +
                (1.0 - params_.accel_filter_alpha) * ay_filtered_;
            alpha_filtered_ =
                params_.accel_filter_alpha * alpha_raw +
                (1.0 - params_.accel_filter_alpha) * alpha_filtered_;
        }
    }

    if (imuRequired() && !has_imu_) {
        ROS_WARN_THROTTLE(
            2.0,
            "[SloshFeedback] IMU input requested but no IMU message received on %s, fallback to odom-based slosh estimate",
            params_.imu_topic.c_str());
    }

    const bool use_imu_ay = params_.use_imu_lateral_accel && has_imu_;
    const bool use_imu_omega = params_.use_imu_yaw_rate && has_imu_;
    const bool use_imu_alpha = params_.use_imu_alpha_z && has_imu_ && has_prev_imu_;

    output_.ax = ax_filtered_;
    output_.odom_ay = ay_filtered_;
    output_.odom_alpha = alpha_filtered_;
    output_.ay = use_imu_ay ? output_.imu_ay_filtered : ay_filtered_;
    output_.omega = use_imu_omega ? output_.imu_omega_z_filtered : current_omega;
    output_.alpha = use_imu_alpha ? imu_alpha_filtered_ : alpha_filtered_;
    output_.has_imu = has_imu_;
    output_.has_prev_imu = has_prev_imu_;

    prev_v_ = current_v;
    prev_omega_ = current_omega;
    prev_odom_time_ = current_odom_time;
    has_prev_odom_ = true;
    return output_;
}

void SloshFeedback::resetOdomFilters() {
    ax_filtered_ = 0.0;
    ay_filtered_ = 0.0;
    alpha_filtered_ = 0.0;
    has_prev_odom_ = false;
    prev_odom_time_ = ros::Time(0);
    output_.ax = 0.0;
    output_.ay = 0.0;
    output_.alpha = 0.0;
    output_.odom_ay = 0.0;
    output_.odom_alpha = 0.0;
}

double SloshFeedback::trimmedMean(std::vector<double> samples, double trim_ratio) {
    if (samples.empty()) {
        return 0.0;
    }

    std::sort(samples.begin(), samples.end());
    const double clamped_trim = std::max(0.0, std::min(0.49, trim_ratio));
    std::size_t trim_count = static_cast<std::size_t>(
        std::floor(static_cast<double>(samples.size()) * clamped_trim));
    if (trim_count * 2 >= samples.size()) {
        trim_count = 0;
    }

    const std::size_t begin = trim_count;
    const std::size_t end = samples.size() - trim_count;
    if (begin >= end) {
        return samples[samples.size() / 2];
    }

    double sum = 0.0;
    for (std::size_t i = begin; i < end; ++i) {
        sum += samples[i];
    }
    return sum / static_cast<double>(end - begin);
}

}  // namespace scout_local_planner
