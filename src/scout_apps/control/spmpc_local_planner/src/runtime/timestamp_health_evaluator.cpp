#include "spmpc_local_planner/runtime/timestamp_health_evaluator.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace spmpc_local_planner {
namespace {

double percentile(std::vector<double> values, double quantile) {
    values.erase(
        std::remove_if(values.begin(), values.end(),
                       [](double value) { return !std::isfinite(value); }),
        values.end());
    if (values.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::sort(values.begin(), values.end());
    if (values.size() == 1) {
        return values.front();
    }
    const double position =
        static_cast<double>(values.size() - 1) * quantile;
    const std::size_t lower =
        static_cast<std::size_t>(std::floor(position));
    const std::size_t upper =
        static_cast<std::size_t>(std::ceil(position));
    const double weight = position - static_cast<double>(lower);
    return values[lower] * (1.0 - weight) + values[upper] * weight;
}

std::string fixed6(double value) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6) << value;
    return stream.str();
}

}  // namespace

TimestampHealthResult TimestampHealthEvaluator::evaluate(
    const std::vector<TimestampRecord>& records,
    const TimestampHealthThresholds& thresholds) {
    TimestampHealthResult result;
    result.thresholds = thresholds;

    std::vector<TimestampRecord> clean;
    clean.reserve(records.size());
    for (const TimestampRecord& record : records) {
        if (!std::isfinite(record.receive_time_sec) ||
            !std::isfinite(record.source_time_sec)) {
            ++result.nonfinite_samples;
            continue;
        }
        clean.push_back(record);
    }
    result.samples = clean.size();
    if (result.nonfinite_samples > 0) {
        result.failures.push_back(
            std::to_string(result.nonfinite_samples) +
            " non-finite timestamp samples");
    }
    if (clean.size() < 2) {
        result.failures.push_back("fewer than two finite stamped samples");
        return result;
    }

    result.has_zero_stamp_samples = true;
    std::vector<TimestampRecord> positive;
    positive.reserve(clean.size());
    for (const TimestampRecord& record : clean) {
        if (record.source_time_sec <= 0.0) {
            ++result.zero_stamp_samples;
        } else {
            positive.push_back(record);
        }
    }
    if (result.zero_stamp_samples > 0) {
        result.failures.push_back(
            std::to_string(result.zero_stamp_samples) +
            " zero/nonpositive source stamps");
    }
    if (positive.size() < 2) {
        result.failures.push_back("fewer than two positive source stamps");
        return result;
    }

    result.has_full_metrics = true;
    result.positive_stamp_samples = positive.size();
    std::vector<double> lags;
    std::vector<double> source_deltas;
    lags.reserve(positive.size());
    source_deltas.reserve(positive.size() - 1);
    for (std::size_t index = 0; index < positive.size(); ++index) {
        lags.push_back(positive[index].receive_time_sec -
                       positive[index].source_time_sec);
        if (index == 0) {
            continue;
        }
        const double receive_delta =
            positive[index].receive_time_sec -
            positive[index - 1].receive_time_sec;
        const double source_delta =
            positive[index].source_time_sec -
            positive[index - 1].source_time_sec;
        source_deltas.push_back(source_delta);
        if (receive_delta <= 0.0) {
            ++result.receive_regressions;
        }
        if (source_delta <= 0.0) {
            ++result.source_regressions;
        }
    }

    result.receive_span_sec =
        positive.back().receive_time_sec - positive.front().receive_time_sec;
    result.source_span_sec =
        positive.back().source_time_sec - positive.front().source_time_sec;
    result.clock_rate_ratio = result.receive_span_sec > 0.0
        ? result.source_span_sec / result.receive_span_sec
        : std::numeric_limits<double>::quiet_NaN();
    result.min_receive_minus_source_sec =
        *std::min_element(lags.begin(), lags.end());
    result.p95_receive_minus_source_sec = percentile(lags, 0.95);
    result.max_receive_minus_source_sec =
        *std::max_element(lags.begin(), lags.end());
    result.max_source_gap_sec =
        *std::max_element(source_deltas.begin(), source_deltas.end());

    if (result.min_receive_minus_source_sec <
        -thresholds.max_future_skew_sec) {
        result.failures.push_back(
            "source stamp future skew " +
            fixed6(-result.min_receive_minus_source_sec) + "s exceeds " +
            fixed6(thresholds.max_future_skew_sec) + "s");
    }
    if (!std::isfinite(result.p95_receive_minus_source_sec) ||
        result.p95_receive_minus_source_sec > thresholds.max_p95_lag_sec) {
        result.failures.push_back(
            "receive lag P95 " +
            fixed6(result.p95_receive_minus_source_sec) + "s exceeds " +
            fixed6(thresholds.max_p95_lag_sec) + "s");
    }
    if (!std::isfinite(result.clock_rate_ratio) ||
        result.clock_rate_ratio < thresholds.min_clock_rate_ratio ||
        result.clock_rate_ratio > thresholds.max_clock_rate_ratio) {
        result.failures.push_back(
            "source/receive clock-rate ratio " +
            fixed6(result.clock_rate_ratio) + " outside [" +
            fixed6(thresholds.min_clock_rate_ratio) + ", " +
            fixed6(thresholds.max_clock_rate_ratio) + "]");
    }
    if (!std::isfinite(result.max_source_gap_sec) ||
        result.max_source_gap_sec > thresholds.max_gap_sec) {
        result.failures.push_back(
            "source max gap " + fixed6(result.max_source_gap_sec) +
            "s exceeds " + fixed6(thresholds.max_gap_sec) + "s");
    }
    if (result.source_regressions > 0) {
        result.failures.push_back(
            std::to_string(result.source_regressions) +
            " source timestamp regressions/duplicates");
    }
    if (result.receive_regressions > 0) {
        result.failures.push_back(
            std::to_string(result.receive_regressions) +
            " receive-time regressions/duplicates");
    }

    result.status = result.failures.empty() ? "PASS" : "FAIL";
    return result;
}

}  // namespace spmpc_local_planner
