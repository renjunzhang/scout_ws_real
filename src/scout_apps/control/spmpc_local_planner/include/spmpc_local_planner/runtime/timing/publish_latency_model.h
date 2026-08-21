#pragma once

#include "spmpc_local_planner/domain/time.h"
#include "spmpc_local_planner/runtime/control_cycle_timing.h"

#include <cstdint>
#include <string>

namespace spmpc_local_planner {

struct CycleTimingContract {
    std::uint64_t cycle_id = 0;
    StampNs cycle_start_stamp_ns = 0;
    double control_period_sec = 0.0;
};

struct PublishLatencyModelConfig {
    bool enabled = false;
    double estimated_dc_sec = 0.0;
};

struct PublishEpochEstimate {
    bool valid = false;
    CycleTimingContract cycle;
    StampNs expected_publish_stamp_ns = 0;
    StampNs publish_deadline_stamp_ns = 0;
    double estimated_dc_sec = 0.0;
    bool expected_deadline_missed = false;
    std::string status = "NOT_EVALUATED";
};

struct PublishLatencyObservation {
    PublishEpochEstimate estimate;
    bool actual_valid = false;
    StampNs actual_publish_stamp_ns = 0;
    double actual_dc_sec = 0.0;
    double dc_error_sec = 0.0;
    bool publish_deadline_missed = false;
    std::string status = "NOT_EVALUATED";
};

// A supplied estimate is accepted only when its complete typed image matches
// the cycle and can be reproduced from that cycle.  This prevents prediction,
// solver and publication from silently consuming different publish epochs.
bool publishEpochEstimateMatchesCycle(
    const PublishEpochEstimate& estimate,
    const CycleTimingContract& cycle);

inline void applyPublishEpochEstimate(
    const PublishEpochEstimate& estimate,
    ControlCycleTimingDebug& timing) {
    timing.expected_publish_stamp_ns =
        estimate.expected_publish_stamp_ns;
    timing.publish_deadline_stamp_ns =
        estimate.publish_deadline_stamp_ns;
    timing.estimated_dc_sec = estimate.estimated_dc_sec;
    timing.publish_epoch_estimate_valid = estimate.valid;
    timing.expected_publish_deadline_missed =
        estimate.expected_deadline_missed;
    timing.publish_timing_status = estimate.status;
}

inline void applyPublishLatencyObservation(
    const PublishLatencyObservation& observation,
    ControlCycleTimingDebug& timing) {
    applyPublishEpochEstimate(observation.estimate, timing);
    timing.actual_dc_sec = observation.actual_dc_sec;
    timing.dc_error_sec = observation.dc_error_sec;
    timing.publish_latency_observation_valid = observation.actual_valid;
    timing.publish_deadline_missed =
        observation.publish_deadline_missed;
    timing.publish_timing_status = observation.status;
}

// Frozen estimate of solve/finalization/ROS-publication latency.  This model
// does not adapt online: formal sessions will bind estimated_dc_sec to an
// execution calibration artifact.  Runtime observations are evidence only.
class PublishLatencyModel {
public:
    bool configure(const PublishLatencyModelConfig& config,
                   std::string& error);

    PublishEpochEstimate estimate(
        const CycleTimingContract& cycle) const;
    PublishLatencyObservation observe(
        const PublishEpochEstimate& estimate,
        StampNs actual_publish_stamp_ns) const;

    const PublishLatencyModelConfig& config() const { return config_; }

private:
    PublishLatencyModelConfig config_;
};

}  // namespace spmpc_local_planner
