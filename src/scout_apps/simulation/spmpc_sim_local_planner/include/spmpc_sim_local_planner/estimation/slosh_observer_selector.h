#pragma once

#include "spmpc_sim_local_planner/estimation/slosh_observer_bank.h"

#include <cstdint>
#include <string>

namespace spmpc_sim_local_planner {

enum class SloshObserverSource : std::uint8_t {
    Unknown = 0,
    Odom = 1,
    ProcessedImu = 2,
};

enum class SloshObserverFallbackPolicy : std::uint8_t {
    Odom = 1,
    FailClosed = 2,
};

enum class SloshObserverSelectionStatus : std::uint8_t {
    Unconfigured = 0,
    WaitingForNominalImu = 1,
    NominalOdom = 2,
    NominalProcessedImu = 3,
    FallbackToOdom = 4,
    FailClosed = 5,
};

enum class SloshObserverSelectionReason : std::uint8_t {
    None = 0,
    ImuNotReady = 1,
    ImuInvalid = 2,
    ImuStale = 3,
    OdomInvalid = 4,
    OdomStale = 5,
};

const char* sloshObserverSourceName(SloshObserverSource source);
const char* sloshObserverFallbackPolicyName(SloshObserverFallbackPolicy policy);
const char* sloshObserverSelectionStatusName(SloshObserverSelectionStatus status);
const char* sloshObserverSelectionReasonName(SloshObserverSelectionReason reason);
bool parseSloshObserverSource(const std::string& text, SloshObserverSource& source);
bool parseSloshObserverFallbackPolicy(
    const std::string& text,
    SloshObserverFallbackPolicy& policy);

struct SloshObserverSelectorParams {
    SloshObserverSource nominal_source = SloshObserverSource::Odom;
    SloshObserverFallbackPolicy fallback_policy = SloshObserverFallbackPolicy::Odom;
    bool latch_fallback = true;
    double max_imu_state_age_sec = 0.10;
    double max_odom_state_age_sec = 0.50;
    double max_future_skew_sec = 0.005;
};

struct SloshObserverHealth {
    SloshObserverSnapshot snapshot;
    // For processed IMU this is stricter than snapshot.valid: the upstream
    // pipeline must report READY with completed bias and filter warmup.
    bool input_ready = false;
    std::uint32_t input_reset_epoch = 0;
};

struct SloshObserverSelection {
    bool configured = false;
    bool valid = false;
    bool fallback_active = false;
    bool fallback_latched = false;
    bool nominal_ready_seen = false;
    bool odom_snapshot_valid = false;
    bool imu_snapshot_valid = false;
    bool odom_fresh = false;
    bool imu_fresh = false;
    bool imu_pipeline_ready = false;
    SloshObserverSource nominal_source = SloshObserverSource::Unknown;
    SloshObserverSource effective_source = SloshObserverSource::Unknown;
    SloshObserverFallbackPolicy fallback_policy = SloshObserverFallbackPolicy::FailClosed;
    SloshObserverSelectionStatus status = SloshObserverSelectionStatus::Unconfigured;
    SloshObserverSelectionReason reason = SloshObserverSelectionReason::None;
    SloshState state;
    std::int64_t selected_state_stamp_ns = 0;
    std::int64_t odom_state_stamp_ns = 0;
    std::int64_t imu_state_stamp_ns = 0;
    double odom_state_age_sec = -1.0;
    double imu_state_age_sec = -1.0;
    std::uint32_t imu_reset_epoch = 0;
    std::uint64_t selection_epoch = 0;
};

// Selects the current liquid observer state at the control-cycle boundary.
// It deliberately does not propagate a horizon: future liquid state remains
// the responsibility of the robot+liquid dynamics in the solver/predictor.
class SloshObserverSelector {
public:
    bool configure(const SloshObserverSelectorParams& params);
    void reset();

    SloshObserverSelection select(
        const SloshObserverHealth& odom,
        const SloshObserverHealth& imu,
        std::int64_t now_ns);

    bool configured() const { return configured_; }
    const SloshObserverSelectorParams& params() const { return params_; }

private:
    static bool finiteState(const SloshState& state);
    static bool snapshotValid(const SloshObserverSnapshot& snapshot);
    static double stateAgeSec(const SloshObserverSnapshot& snapshot, std::int64_t now_ns);
    bool stateFresh(double age_sec, double max_age_sec) const;
    void updateSelectionEpoch(SloshObserverSelection& selection);

    SloshObserverSelectorParams params_;
    bool configured_ = false;
    bool nominal_imu_ready_seen_ = false;
    bool fallback_latched_ = false;
    SloshObserverSelectionReason latched_fallback_reason_ =
        SloshObserverSelectionReason::None;
    bool have_previous_selection_ = false;
    bool previous_valid_ = false;
    bool previous_fallback_latched_ = false;
    SloshObserverSource previous_effective_source_ = SloshObserverSource::Unknown;
    SloshObserverSelectionStatus previous_status_ =
        SloshObserverSelectionStatus::Unconfigured;
    SloshObserverSelectionReason previous_reason_ =
        SloshObserverSelectionReason::None;
    std::uint32_t previous_imu_reset_epoch_ = 0;
    std::uint64_t selection_epoch_ = 0;
};

}  // namespace spmpc_sim_local_planner
