#include "spmpc_local_planner/runtime/timing/publish_latency_model.h"

#include <cmath>

namespace spmpc_local_planner {

bool PublishLatencyModel::configure(
    const PublishLatencyModelConfig& config,
    std::string& error) {
    error.clear();
    if (!std::isfinite(config.estimated_dc_sec) ||
        config.estimated_dc_sec < 0.0) {
        error = "estimated_dc_sec must be finite and non-negative";
        return false;
    }
    config_ = config;
    return true;
}

PublishEpochEstimate PublishLatencyModel::estimate(
    const CycleTimingContract& cycle) const {
    PublishEpochEstimate result;
    result.cycle = cycle;
    result.estimated_dc_sec = config_.estimated_dc_sec;

    if (!validStamp(cycle.cycle_start_stamp_ns) ||
        !std::isfinite(cycle.control_period_sec) ||
        cycle.control_period_sec <= 0.0) {
        result.status = "INVALID_CYCLE_TIMING";
        return result;
    }

    result.publish_deadline_stamp_ns = addSeconds(
        cycle.cycle_start_stamp_ns, cycle.control_period_sec);
    if (!validStamp(result.publish_deadline_stamp_ns)) {
        result.status = "PUBLISH_DEADLINE_OVERFLOW";
        return result;
    }

    if (!config_.enabled) {
        result.status = "ESTIMATE_OFF";
        return result;
    }

    result.expected_publish_stamp_ns = addSeconds(
        cycle.cycle_start_stamp_ns, config_.estimated_dc_sec);
    if (!validStamp(result.expected_publish_stamp_ns)) {
        result.status = "EXPECTED_PUBLISH_OVERFLOW";
        return result;
    }

    result.valid = true;
    result.expected_deadline_missed =
        result.expected_publish_stamp_ns > result.publish_deadline_stamp_ns;
    result.status = result.expected_deadline_missed
        ? "EXPECTED_DEADLINE_MISS"
        : "ESTIMATED";
    return result;
}

PublishLatencyObservation PublishLatencyModel::observe(
    const PublishEpochEstimate& estimate,
    StampNs actual_publish_stamp_ns) const {
    PublishLatencyObservation result;
    result.estimate = estimate;
    result.actual_publish_stamp_ns = actual_publish_stamp_ns;

    if (!validStamp(estimate.cycle.cycle_start_stamp_ns) ||
        !validStamp(estimate.publish_deadline_stamp_ns)) {
        result.status = "INVALID_CYCLE_TIMING";
        return result;
    }
    if (!validStamp(actual_publish_stamp_ns)) {
        result.status = "PUBLISH_NOT_DELIVERED";
        return result;
    }
    if (actual_publish_stamp_ns < estimate.cycle.cycle_start_stamp_ns) {
        result.status = "ACTUAL_PUBLISH_BEFORE_CYCLE";
        return result;
    }

    result.actual_valid = true;
    result.actual_dc_sec = secondsBetween(
        actual_publish_stamp_ns, estimate.cycle.cycle_start_stamp_ns);
    result.publish_deadline_missed =
        actual_publish_stamp_ns > estimate.publish_deadline_stamp_ns;
    if (estimate.valid) {
        result.dc_error_sec = secondsBetween(
            actual_publish_stamp_ns,
            estimate.expected_publish_stamp_ns);
    }

    if (result.publish_deadline_missed) {
        result.status = "PUBLISH_DEADLINE_MISSED";
    } else if (estimate.status == "ESTIMATE_OFF") {
        result.status = "MEASURED_ESTIMATE_OFF";
    } else if (!estimate.valid) {
        result.status = "MEASURED_INVALID_ESTIMATE";
    } else {
        result.status = "OK";
    }
    return result;
}

}  // namespace spmpc_local_planner
