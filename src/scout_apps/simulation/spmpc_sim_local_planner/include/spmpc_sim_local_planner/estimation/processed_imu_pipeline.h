#pragma once

#include "spmpc_sim_local_planner/estimation/motion_excitation.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace spmpc_sim_local_planner {

enum class ImuPipelineStatusCode : std::uint8_t {
    Unconfigured = 0,
    WaitingForBiasWindow = 1,
    CollectingBias = 2,
    BiasInsufficient = 3,
    BiasMotionDetected = 4,
    FilterWarmup = 5,
    Ready = 6,
    InvalidSample = 7,
    OrientationUnavailable = 8,
    InvalidOrientation = 9,
    FrameMismatch = 10,
    DuplicateTimestamp = 11,
    OutOfOrderDrop = 12,
    ClockReset = 13,
    SampleGap = 14,
    StaleSample = 15,
};

const char* imuPipelineStatusName(ImuPipelineStatusCode status);

struct ImuSample {
    std::int64_t source_stamp_ns = 0;
    std::int64_t receive_stamp_ns = 0;
    bool orientation_available = true;

    double orientation_x = 0.0;
    double orientation_y = 0.0;
    double orientation_z = 0.0;
    double orientation_w = 1.0;

    double angular_velocity_x = 0.0;
    double angular_velocity_y = 0.0;
    double angular_velocity_z = 0.0;

    double linear_acceleration_x = 0.0;
    double linear_acceleration_y = 0.0;
    double linear_acceleration_z = 0.0;
};

struct ProcessedImuParams {
    double gravity_mps2 = 9.8;
    double sensor_delay_sec = 0.015;

    double accel_cutoff_hz = 10.0;
    double gyro_cutoff_hz = 12.0;
    double accel_phase_delay_sec = 0.006834;
    double gyro_phase_delay_sec = 0.005020;
    double alpha_phase_delay_sec = 0.015001;

    double gyro_scale = 1.001773;
    double gyro_offset_radps = -0.000297;
    double imu_to_base_yaw_rad = 0.0;
    // Configured nominal planar vector from the IMU origin to the development
    // liquid-observer/ICR proxy, expressed in base axes.  This does not claim a
    // numerically identified physical base_link origin or full extrinsic.
    double lever_arm_imu_to_target_x_m = -0.100;
    double lever_arm_imu_to_target_y_m = 0.045;

    double bias_window_start_sec = 2.0;
    double bias_window_end_sec = 10.0;
    int bias_min_samples = 100;
    double bias_max_accel_mad_mps2 = 0.02;
    double bias_max_gyro_p95_radps = 0.03;

    double filter_warmup_sec = 0.20;
    double max_sample_gap_sec = 0.035;
    double clock_reset_threshold_sec = 0.50;
    double max_receive_age_sec = 0.10;
    double max_future_skew_sec = 0.005;

    double quaternion_norm_min = 0.95;
    double quaternion_norm_max = 1.05;
};

struct ProcessedImuOutput {
    ImuPipelineStatusCode status = ImuPipelineStatusCode::Unconfigured;
    MotionExcitation excitation;

    bool bias_ready = false;
    bool filter_ready = false;
    std::uint32_t reset_epoch = 0;
    std::uint64_t accepted_sample_count = 0;
    std::size_t bias_sample_count = 0;

    std::array<double, 3> bias_mps2{{0.0, 0.0, 0.0}};
    std::array<double, 3> linear_accel_imu_mps2{{0.0, 0.0, 0.0}};
    std::array<double, 2> accel_filtered_base_mps2{{0.0, 0.0}};
    double gyro_filtered_radps = 0.0;
    double alpha_radps2 = 0.0;
    double quaternion_norm = 0.0;
    double transport_age_sec = 0.0;
};

class ProcessedImuPipeline {
public:
    bool configure(const ProcessedImuParams& params);
    void reset();

    ProcessedImuOutput process(const ImuSample& sample);
    ProcessedImuOutput reject(const ImuSample& sample, ImuPipelineStatusCode status) const;

    bool configured() const { return configured_; }
    bool biasReady() const { return bias_ready_; }
    const ProcessedImuParams& params() const { return params_; }
    const ProcessedImuOutput& output() const { return output_; }

private:
    struct OnePoleState {
        bool initialized = false;
        double value = 0.0;
    };

    void clearState(bool increment_epoch);
    void clearTransientState(bool increment_epoch);
    bool validateParams(const ProcessedImuParams& params) const;
    bool validateFiniteSample(const ImuSample& sample) const;
    bool removeGravity(const ImuSample& sample,
                       std::array<double, 3>& linear_accel_imu,
                       double& quaternion_norm) const;
    bool finalizeBias();
    void beginEpochAt(std::int64_t source_stamp_ns);
    void initializeFilters(const std::array<double, 2>& accel_base,
                           double omega_calibrated,
                           std::int64_t source_stamp_ns);
    double updateOnePole(OnePoleState& state, double input, double cutoff_hz, double dt_sec) const;
    ProcessedImuOutput makeOutput(const ImuSample& sample,
                                  ImuPipelineStatusCode status,
                                  double quaternion_norm,
                                  double dt_sec) const;
    static double median(std::vector<double> values);
    static double percentile(std::vector<double> values, double probability);
    static std::int64_t secondsToNanoseconds(double seconds);

    ProcessedImuParams params_;
    bool configured_ = false;
    bool bias_ready_ = false;
    bool bias_failed_ = false;
    bool filters_initialized_ = false;

    std::uint32_t reset_epoch_ = 0;
    std::uint64_t accepted_sample_count_ = 0;
    std::int64_t epoch_start_stamp_ns_ = 0;
    std::int64_t last_accepted_source_stamp_ns_ = 0;
    std::int64_t last_receive_stamp_ns_ = 0;
    std::int64_t filter_start_stamp_ns_ = 0;
    std::int64_t last_filter_stamp_ns_ = 0;

    std::array<double, 3> bias_mps2_{{0.0, 0.0, 0.0}};
    std::vector<std::array<double, 3>> bias_accel_samples_;
    std::vector<double> bias_gyro_samples_;

    OnePoleState accel_x_filter_;
    OnePoleState accel_y_filter_;
    OnePoleState gyro_filter_;
    double previous_gyro_filtered_radps_ = 0.0;
    double latest_alpha_radps2_ = 0.0;
    ProcessedImuOutput output_;
};

}  // namespace spmpc_sim_local_planner
