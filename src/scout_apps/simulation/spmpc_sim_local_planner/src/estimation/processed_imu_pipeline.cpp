#include "spmpc_sim_local_planner/estimation/processed_imu_pipeline.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace spmpc_sim_local_planner {

namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr double kNanosecondsPerSecond = 1.0e9;

bool finite(double value) {
    return std::isfinite(value);
}

}  // namespace

const char* imuPipelineStatusName(ImuPipelineStatusCode status) {
    switch (status) {
        case ImuPipelineStatusCode::Unconfigured: return "UNCONFIGURED";
        case ImuPipelineStatusCode::WaitingForBiasWindow: return "WAITING_FOR_BIAS_WINDOW";
        case ImuPipelineStatusCode::CollectingBias: return "COLLECTING_BIAS";
        case ImuPipelineStatusCode::BiasInsufficient: return "BIAS_INSUFFICIENT";
        case ImuPipelineStatusCode::BiasMotionDetected: return "BIAS_MOTION_DETECTED";
        case ImuPipelineStatusCode::FilterWarmup: return "FILTER_WARMUP";
        case ImuPipelineStatusCode::Ready: return "READY";
        case ImuPipelineStatusCode::InvalidSample: return "INVALID_SAMPLE";
        case ImuPipelineStatusCode::OrientationUnavailable: return "ORIENTATION_UNAVAILABLE";
        case ImuPipelineStatusCode::InvalidOrientation: return "INVALID_ORIENTATION";
        case ImuPipelineStatusCode::FrameMismatch: return "FRAME_MISMATCH";
        case ImuPipelineStatusCode::DuplicateTimestamp: return "DUPLICATE_TIMESTAMP";
        case ImuPipelineStatusCode::OutOfOrderDrop: return "OUT_OF_ORDER_DROP";
        case ImuPipelineStatusCode::ClockReset: return "CLOCK_RESET";
        case ImuPipelineStatusCode::SampleGap: return "SAMPLE_GAP";
        case ImuPipelineStatusCode::StaleSample: return "STALE_SAMPLE";
    }
    return "UNKNOWN";
}

bool ProcessedImuPipeline::configure(const ProcessedImuParams& params) {
    if (!validateParams(params)) {
        configured_ = false;
        output_ = ProcessedImuOutput();
        return false;
    }
    params_ = params;
    configured_ = true;
    clearState(false);
    return true;
}

void ProcessedImuPipeline::reset() {
    clearState(true);
}

ProcessedImuOutput ProcessedImuPipeline::process(const ImuSample& sample) {
    if (!configured_) {
        return reject(sample, ImuPipelineStatusCode::Unconfigured);
    }
    if (!validateFiniteSample(sample) || sample.source_stamp_ns <= 0 || sample.receive_stamp_ns <= 0) {
        return reject(sample, ImuPipelineStatusCode::InvalidSample);
    }
    if (!sample.orientation_available) {
        return reject(sample, ImuPipelineStatusCode::OrientationUnavailable);
    }

    // Validate sensor content before interpreting a timestamp discontinuity.
    // A bad quaternion must not reset epochs, consume the first sample after a
    // gap, or mutate any accepted timing/filter state.
    std::array<double, 3> linear_accel_imu{{0.0, 0.0, 0.0}};
    double quaternion_norm = 0.0;
    if (!removeGravity(sample, linear_accel_imu, quaternion_norm)) {
        return reject(sample, ImuPipelineStatusCode::InvalidOrientation);
    }

    const double transport_age_sec =
        static_cast<double>(sample.receive_stamp_ns - sample.source_stamp_ns) / kNanosecondsPerSecond;
    if (transport_age_sec < -params_.max_future_skew_sec ||
        (params_.max_receive_age_sec > 0.0 && transport_age_sec > params_.max_receive_age_sec)) {
        return reject(sample, ImuPipelineStatusCode::StaleSample);
    }

    if (last_receive_stamp_ns_ > 0 && sample.receive_stamp_ns < last_receive_stamp_ns_) {
        clearState(true);
        output_ = makeOutput(sample, ImuPipelineStatusCode::ClockReset, quaternion_norm, 0.0);
        output_.linear_accel_imu_mps2 = linear_accel_imu;
        return output_;
    }
    if (last_accepted_source_stamp_ns_ > 0) {
        if (sample.source_stamp_ns == last_accepted_source_stamp_ns_) {
            return reject(sample, ImuPipelineStatusCode::DuplicateTimestamp);
        }
        if (sample.source_stamp_ns < last_accepted_source_stamp_ns_) {
            const double regression_sec = static_cast<double>(
                last_accepted_source_stamp_ns_ - sample.source_stamp_ns) / kNanosecondsPerSecond;
            if (regression_sec > params_.clock_reset_threshold_sec) {
                clearState(true);
                output_ = makeOutput(
                    sample, ImuPipelineStatusCode::ClockReset, quaternion_norm, 0.0);
                output_.linear_accel_imu_mps2 = linear_accel_imu;
                return output_;
            }
            return reject(sample, ImuPipelineStatusCode::OutOfOrderDrop);
        }
    }

    double dt_sec = 0.0;
    if (last_accepted_source_stamp_ns_ > 0) {
        dt_sec = static_cast<double>(sample.source_stamp_ns - last_accepted_source_stamp_ns_) /
                 kNanosecondsPerSecond;
        if (dt_sec > params_.max_sample_gap_sec) {
            if (bias_failed_) {
                // Calibration failures are fail-closed within a clock domain.
                // A packet gap may start a new timing/observer epoch, but it
                // must not silently reopen calibration while the robot could
                // already be moving.  Explicit reset/reconfigure or a clock
                // reset is required before another bias attempt.
                const ImuPipelineStatusCode failure_status = output_.status;
                clearTransientState(true);
                beginEpochAt(sample.source_stamp_ns);
                last_accepted_source_stamp_ns_ = sample.source_stamp_ns;
                last_receive_stamp_ns_ = sample.receive_stamp_ns;
                ++accepted_sample_count_;
                output_ = makeOutput(sample, failure_status, quaternion_norm, 0.0);
                output_.linear_accel_imu_mps2 = linear_accel_imu;
                return output_;
            }
            if (bias_ready_) {
                clearTransientState(true);
            } else {
                clearState(true);
            }
            beginEpochAt(sample.source_stamp_ns);
            last_accepted_source_stamp_ns_ = sample.source_stamp_ns;
            last_receive_stamp_ns_ = sample.receive_stamp_ns;
            ++accepted_sample_count_;
            output_ = makeOutput(
                sample, ImuPipelineStatusCode::SampleGap, quaternion_norm, 0.0);
            output_.linear_accel_imu_mps2 = linear_accel_imu;
            return output_;
        }
    }

    if (epoch_start_stamp_ns_ <= 0) {
        beginEpochAt(sample.source_stamp_ns);
    }
    const double epoch_elapsed_sec = static_cast<double>(
        sample.source_stamp_ns - epoch_start_stamp_ns_) / kNanosecondsPerSecond;
    const double omega_calibrated =
        params_.gyro_scale * sample.angular_velocity_z + params_.gyro_offset_radps;

    ImuPipelineStatusCode status = ImuPipelineStatusCode::WaitingForBiasWindow;
    if (!bias_ready_ && !bias_failed_) {
        if (epoch_elapsed_sec >= params_.bias_window_start_sec &&
            epoch_elapsed_sec < params_.bias_window_end_sec) {
            bias_accel_samples_.push_back(linear_accel_imu);
            bias_gyro_samples_.push_back(omega_calibrated);
            status = ImuPipelineStatusCode::CollectingBias;
        } else if (epoch_elapsed_sec >= params_.bias_window_end_sec) {
            if (!finalizeBias()) {
                status = output_.status;
            }
        }
    }

    last_accepted_source_stamp_ns_ = sample.source_stamp_ns;
    last_receive_stamp_ns_ = sample.receive_stamp_ns;
    ++accepted_sample_count_;

    if (!bias_ready_) {
        if (bias_failed_) {
            status = output_.status;
        }
        output_ = makeOutput(sample, status, quaternion_norm, dt_sec);
        output_.linear_accel_imu_mps2 = linear_accel_imu;
        return output_;
    }

    const double accel_imu_x = linear_accel_imu[0] - bias_mps2_[0];
    const double accel_imu_y = linear_accel_imu[1] - bias_mps2_[1];
    const double cy = std::cos(params_.imu_to_base_yaw_rad);
    const double sy = std::sin(params_.imu_to_base_yaw_rad);
    const std::array<double, 2> accel_base{{
        cy * accel_imu_x - sy * accel_imu_y,
        sy * accel_imu_x + cy * accel_imu_y,
    }};

    if (!filters_initialized_) {
        initializeFilters(accel_base, omega_calibrated, sample.source_stamp_ns);
        output_ = makeOutput(sample, ImuPipelineStatusCode::FilterWarmup, quaternion_norm, dt_sec);
        output_.linear_accel_imu_mps2 = linear_accel_imu;
        output_.accel_filtered_base_mps2 = {{accel_x_filter_.value, accel_y_filter_.value}};
        output_.gyro_filtered_radps = gyro_filter_.value;
        return output_;
    }

    const double filter_dt_sec = static_cast<double>(
        sample.source_stamp_ns - last_filter_stamp_ns_) / kNanosecondsPerSecond;
    if (!(filter_dt_sec > 0.0) || filter_dt_sec > params_.max_sample_gap_sec) {
        clearTransientState(true);
        beginEpochAt(sample.source_stamp_ns);
        last_accepted_source_stamp_ns_ = sample.source_stamp_ns;
        last_receive_stamp_ns_ = sample.receive_stamp_ns;
        initializeFilters(accel_base, omega_calibrated, sample.source_stamp_ns);
        ++accepted_sample_count_;
        output_ = makeOutput(sample, ImuPipelineStatusCode::SampleGap, quaternion_norm, 0.0);
        output_.linear_accel_imu_mps2 = linear_accel_imu;
        return output_;
    }

    updateOnePole(accel_x_filter_, accel_base[0], params_.accel_cutoff_hz, filter_dt_sec);
    updateOnePole(accel_y_filter_, accel_base[1], params_.accel_cutoff_hz, filter_dt_sec);
    const double old_gyro_filtered = gyro_filter_.value;
    updateOnePole(gyro_filter_, omega_calibrated, params_.gyro_cutoff_hz, filter_dt_sec);
    previous_gyro_filtered_radps_ = old_gyro_filtered;
    latest_alpha_radps2_ = (gyro_filter_.value - old_gyro_filtered) / filter_dt_sec;
    last_filter_stamp_ns_ = sample.source_stamp_ns;

    const double omega_sq = gyro_filter_.value * gyro_filter_.value;
    const double rx = params_.lever_arm_imu_to_target_x_m;
    const double ry = params_.lever_arm_imu_to_target_y_m;
    const double target_ax = accel_x_filter_.value - latest_alpha_radps2_ * ry - omega_sq * rx;
    const double target_ay = accel_y_filter_.value + latest_alpha_radps2_ * rx - omega_sq * ry;

    const double warmup_elapsed_sec = static_cast<double>(
        sample.source_stamp_ns - filter_start_stamp_ns_) / kNanosecondsPerSecond;
    const bool filter_ready = warmup_elapsed_sec >= params_.filter_warmup_sec;
    status = filter_ready ? ImuPipelineStatusCode::Ready : ImuPipelineStatusCode::FilterWarmup;
    output_ = makeOutput(sample, status, quaternion_norm, filter_dt_sec);
    output_.linear_accel_imu_mps2 = linear_accel_imu;
    output_.accel_filtered_base_mps2 = {{accel_x_filter_.value, accel_y_filter_.value}};
    output_.gyro_filtered_radps = gyro_filter_.value;
    output_.alpha_radps2 = latest_alpha_radps2_;
    output_.filter_ready = filter_ready;
    output_.excitation.valid = filter_ready;
    output_.excitation.ax = target_ax;
    output_.excitation.ay = target_ay;
    output_.excitation.omega_z = gyro_filter_.value;
    output_.excitation.alpha_z = latest_alpha_radps2_;
    output_.excitation.sample_dt_sec = filter_dt_sec;
    return output_;
}

ProcessedImuOutput ProcessedImuPipeline::reject(
    const ImuSample& sample,
    ImuPipelineStatusCode status) const {
    ProcessedImuOutput rejected = makeOutput(sample, status, 0.0, 0.0);
    rejected.excitation.valid = false;
    return rejected;
}

void ProcessedImuPipeline::clearState(bool increment_epoch) {
    if (increment_epoch) {
        ++reset_epoch_;
    }
    bias_ready_ = false;
    bias_failed_ = false;
    bias_mps2_ = {{0.0, 0.0, 0.0}};
    bias_accel_samples_.clear();
    bias_gyro_samples_.clear();
    epoch_start_stamp_ns_ = 0;
    last_accepted_source_stamp_ns_ = 0;
    last_receive_stamp_ns_ = 0;
    accepted_sample_count_ = 0;
    clearTransientState(false);
    output_ = ProcessedImuOutput();
    output_.reset_epoch = reset_epoch_;
}

void ProcessedImuPipeline::clearTransientState(bool increment_epoch) {
    if (increment_epoch) {
        ++reset_epoch_;
        // accepted_sample_count is epoch-local, matching reset_epoch and the
        // shadow observer update counter.  Bias may remain frozen across a
        // post-calibration gap, but sample accounting starts at the new epoch.
        accepted_sample_count_ = 0;
    }
    filters_initialized_ = false;
    filter_start_stamp_ns_ = 0;
    last_filter_stamp_ns_ = 0;
    accel_x_filter_ = OnePoleState();
    accel_y_filter_ = OnePoleState();
    gyro_filter_ = OnePoleState();
    previous_gyro_filtered_radps_ = 0.0;
    latest_alpha_radps2_ = 0.0;
}

bool ProcessedImuPipeline::validateParams(const ProcessedImuParams& p) const {
    return finite(p.gravity_mps2) && p.gravity_mps2 > 0.0 &&
           finite(p.sensor_delay_sec) && p.sensor_delay_sec >= 0.0 &&
           finite(p.accel_cutoff_hz) && p.accel_cutoff_hz > 0.0 &&
           finite(p.gyro_cutoff_hz) && p.gyro_cutoff_hz > 0.0 &&
           finite(p.accel_phase_delay_sec) && p.accel_phase_delay_sec >= 0.0 &&
           finite(p.gyro_phase_delay_sec) && p.gyro_phase_delay_sec >= 0.0 &&
           finite(p.alpha_phase_delay_sec) && p.alpha_phase_delay_sec >= 0.0 &&
           finite(p.gyro_scale) && finite(p.gyro_offset_radps) &&
           finite(p.imu_to_base_yaw_rad) &&
           finite(p.lever_arm_imu_to_target_x_m) && finite(p.lever_arm_imu_to_target_y_m) &&
           finite(p.bias_window_start_sec) && p.bias_window_start_sec >= 0.0 &&
           finite(p.bias_window_end_sec) && p.bias_window_end_sec > p.bias_window_start_sec &&
           p.bias_min_samples > 0 &&
           finite(p.bias_max_accel_mad_mps2) && p.bias_max_accel_mad_mps2 >= 0.0 &&
           finite(p.bias_max_gyro_p95_radps) && p.bias_max_gyro_p95_radps >= 0.0 &&
           finite(p.filter_warmup_sec) && p.filter_warmup_sec >= 0.0 &&
           finite(p.max_sample_gap_sec) && p.max_sample_gap_sec > 0.0 &&
           finite(p.clock_reset_threshold_sec) && p.clock_reset_threshold_sec > 0.0 &&
           finite(p.max_receive_age_sec) && p.max_receive_age_sec >= 0.0 &&
           finite(p.max_future_skew_sec) && p.max_future_skew_sec >= 0.0 &&
           finite(p.quaternion_norm_min) && p.quaternion_norm_min > 0.0 &&
           finite(p.quaternion_norm_max) && p.quaternion_norm_max >= p.quaternion_norm_min;
}

bool ProcessedImuPipeline::validateFiniteSample(const ImuSample& s) const {
    return finite(s.orientation_x) && finite(s.orientation_y) &&
           finite(s.orientation_z) && finite(s.orientation_w) &&
           finite(s.angular_velocity_x) && finite(s.angular_velocity_y) &&
           finite(s.angular_velocity_z) && finite(s.linear_acceleration_x) &&
           finite(s.linear_acceleration_y) && finite(s.linear_acceleration_z);
}

bool ProcessedImuPipeline::removeGravity(
    const ImuSample& s,
    std::array<double, 3>& linear_accel_imu,
    double& quaternion_norm) const {
    const double qx = s.orientation_x;
    const double qy = s.orientation_y;
    const double qz = s.orientation_z;
    const double qw = s.orientation_w;
    const double norm_sq = qx * qx + qy * qy + qz * qz + qw * qw;
    if (!finite(norm_sq) || norm_sq <= std::numeric_limits<double>::epsilon()) {
        return false;
    }
    quaternion_norm = std::sqrt(norm_sq);
    if (quaternion_norm < params_.quaternion_norm_min ||
        quaternion_norm > params_.quaternion_norm_max) {
        return false;
    }

    const double inv_norm = 1.0 / quaternion_norm;
    const double x = qx * inv_norm;
    const double y = qy * inv_norm;
    const double z = qz * inv_norm;
    const double w = qw * inv_norm;

    // Quaternion maps IMU/body coordinates into world coordinates. Therefore
    // gravity in IMU axes is R_world_imu^T * [0, 0, g].
    const double gx = params_.gravity_mps2 * 2.0 * (x * z - w * y);
    const double gy = params_.gravity_mps2 * 2.0 * (y * z + w * x);
    const double gz = params_.gravity_mps2 * (1.0 - 2.0 * (x * x + y * y));

    linear_accel_imu[0] = s.linear_acceleration_x - gx;
    linear_accel_imu[1] = s.linear_acceleration_y - gy;
    linear_accel_imu[2] = s.linear_acceleration_z - gz;
    return finite(linear_accel_imu[0]) && finite(linear_accel_imu[1]) &&
           finite(linear_accel_imu[2]);
}

bool ProcessedImuPipeline::finalizeBias() {
    if (static_cast<int>(bias_accel_samples_.size()) < params_.bias_min_samples ||
        bias_accel_samples_.size() != bias_gyro_samples_.size()) {
        bias_failed_ = true;
        output_.status = ImuPipelineStatusCode::BiasInsufficient;
        return false;
    }

    std::array<std::vector<double>, 3> axes;
    for (const auto& sample : bias_accel_samples_) {
        axes[0].push_back(sample[0]);
        axes[1].push_back(sample[1]);
        axes[2].push_back(sample[2]);
    }
    std::array<double, 3> mad{{0.0, 0.0, 0.0}};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        bias_mps2_[axis] = median(axes[axis]);
        std::vector<double> deviations;
        deviations.reserve(axes[axis].size());
        for (double value : axes[axis]) {
            deviations.push_back(std::abs(value - bias_mps2_[axis]));
        }
        mad[axis] = median(deviations);
    }
    std::vector<double> gyro_abs;
    gyro_abs.reserve(bias_gyro_samples_.size());
    for (double value : bias_gyro_samples_) {
        gyro_abs.push_back(std::abs(value));
    }
    const double gyro_p95 = percentile(gyro_abs, 0.95);
    const bool accel_static = mad[0] <= params_.bias_max_accel_mad_mps2 &&
                              mad[1] <= params_.bias_max_accel_mad_mps2 &&
                              mad[2] <= params_.bias_max_accel_mad_mps2;
    if (!accel_static || gyro_p95 > params_.bias_max_gyro_p95_radps) {
        bias_failed_ = true;
        output_.status = ImuPipelineStatusCode::BiasMotionDetected;
        return false;
    }

    bias_ready_ = true;
    bias_failed_ = false;
    return true;
}

void ProcessedImuPipeline::beginEpochAt(std::int64_t source_stamp_ns) {
    epoch_start_stamp_ns_ = source_stamp_ns;
}

void ProcessedImuPipeline::initializeFilters(
    const std::array<double, 2>& accel_base,
    double omega_calibrated,
    std::int64_t source_stamp_ns) {
    accel_x_filter_.initialized = true;
    accel_x_filter_.value = accel_base[0];
    accel_y_filter_.initialized = true;
    accel_y_filter_.value = accel_base[1];
    gyro_filter_.initialized = true;
    gyro_filter_.value = omega_calibrated;
    previous_gyro_filtered_radps_ = omega_calibrated;
    latest_alpha_radps2_ = 0.0;
    filter_start_stamp_ns_ = source_stamp_ns;
    last_filter_stamp_ns_ = source_stamp_ns;
    filters_initialized_ = true;
}

double ProcessedImuPipeline::updateOnePole(
    OnePoleState& state,
    double input,
    double cutoff_hz,
    double dt_sec) const {
    if (!state.initialized) {
        state.initialized = true;
        state.value = input;
        return state.value;
    }
    const double beta = 1.0 - std::exp(-kTwoPi * cutoff_hz * dt_sec);
    state.value += beta * (input - state.value);
    return state.value;
}

ProcessedImuOutput ProcessedImuPipeline::makeOutput(
    const ImuSample& sample,
    ImuPipelineStatusCode status,
    double quaternion_norm,
    double dt_sec) const {
    ProcessedImuOutput out;
    out.status = status;
    out.bias_ready = bias_ready_;
    out.filter_ready = status == ImuPipelineStatusCode::Ready;
    out.reset_epoch = reset_epoch_;
    out.accepted_sample_count = accepted_sample_count_;
    out.bias_sample_count = bias_accel_samples_.size();
    out.bias_mps2 = bias_mps2_;
    out.quaternion_norm = quaternion_norm;
    out.transport_age_sec = sample.source_stamp_ns > 0 && sample.receive_stamp_ns > 0
        ? static_cast<double>(sample.receive_stamp_ns - sample.source_stamp_ns) /
              kNanosecondsPerSecond
        : 0.0;

    MotionExcitation excitation;
    excitation.source = MotionExcitationSource::ProcessedImu;
    excitation.valid = status == ImuPipelineStatusCode::Ready;
    excitation.sample_dt_sec = dt_sec;
    excitation.source_stamp_ns = sample.source_stamp_ns;
    excitation.measurement_stamp_ns =
        sample.source_stamp_ns - secondsToNanoseconds(params_.sensor_delay_sec);
    excitation.accel_effective_stamp_ns = excitation.measurement_stamp_ns -
        secondsToNanoseconds(params_.accel_phase_delay_sec);
    excitation.gyro_effective_stamp_ns = excitation.measurement_stamp_ns -
        secondsToNanoseconds(params_.gyro_phase_delay_sec);
    excitation.alpha_effective_stamp_ns = excitation.measurement_stamp_ns -
        secondsToNanoseconds(params_.alpha_phase_delay_sec);
    excitation.receive_stamp_ns = sample.receive_stamp_ns;
    excitation.reset_epoch = reset_epoch_;
    out.excitation = excitation;
    return out;
}

double ProcessedImuPipeline::median(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    const double upper = values[middle];
    if (values.size() % 2 != 0) {
        return upper;
    }
    std::nth_element(values.begin(), values.begin() + middle - 1, values.begin() + middle);
    return 0.5 * (values[middle - 1] + upper);
}

double ProcessedImuPipeline::percentile(std::vector<double> values, double probability) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const double p = std::max(0.0, std::min(1.0, probability));
    const double index = p * static_cast<double>(values.size() - 1);
    const std::size_t lower = static_cast<std::size_t>(std::floor(index));
    const std::size_t upper = static_cast<std::size_t>(std::ceil(index));
    const double weight = index - static_cast<double>(lower);
    return values[lower] + weight * (values[upper] - values[lower]);
}

std::int64_t ProcessedImuPipeline::secondsToNanoseconds(double seconds) {
    return static_cast<std::int64_t>(std::llround(seconds * kNanosecondsPerSecond));
}

}  // namespace spmpc_sim_local_planner
