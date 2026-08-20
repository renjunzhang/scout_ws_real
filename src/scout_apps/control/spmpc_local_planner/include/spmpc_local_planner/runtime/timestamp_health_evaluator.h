#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace spmpc_local_planner {

struct TimestampRecord {
    double receive_time_sec = 0.0;
    double source_time_sec = 0.0;
};

struct TimestampHealthThresholds {
    double max_future_skew_sec = 0.05;
    double max_p95_lag_sec = 0.20;
    double min_clock_rate_ratio = 0.98;
    double max_clock_rate_ratio = 1.02;
    double max_gap_sec = 0.20;
};

struct TimestampHealthResult {
    std::string status = "FAIL";
    std::vector<std::string> failures;
    std::size_t samples = 0;
    std::size_t nonfinite_samples = 0;
    bool has_zero_stamp_samples = false;
    std::size_t zero_stamp_samples = 0;
    bool has_full_metrics = false;
    std::size_t positive_stamp_samples = 0;
    double receive_span_sec = std::numeric_limits<double>::quiet_NaN();
    double source_span_sec = std::numeric_limits<double>::quiet_NaN();
    double clock_rate_ratio = std::numeric_limits<double>::quiet_NaN();
    double min_receive_minus_source_sec =
        std::numeric_limits<double>::quiet_NaN();
    double p95_receive_minus_source_sec =
        std::numeric_limits<double>::quiet_NaN();
    double max_receive_minus_source_sec =
        std::numeric_limits<double>::quiet_NaN();
    double max_source_gap_sec = std::numeric_limits<double>::quiet_NaN();
    std::size_t source_regressions = 0;
    std::size_t receive_regressions = 0;
    TimestampHealthThresholds thresholds;
};

// Evaluates a receive/source timestamp window without ROS, wall clocks or I/O.
// Failure strings and percentile semantics intentionally match the historical
// Python pre-motion gate so archived reports remain comparable.
class TimestampHealthEvaluator {
public:
    static TimestampHealthResult evaluate(
        const std::vector<TimestampRecord>& records,
        const TimestampHealthThresholds& thresholds =
            TimestampHealthThresholds{});
};

}  // namespace spmpc_local_planner
