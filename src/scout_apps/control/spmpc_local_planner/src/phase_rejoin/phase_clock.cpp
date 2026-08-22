#include "spmpc_local_planner/phase_rejoin/phase_clock.h"

#include <algorithm>
#include <cmath>

namespace spmpc_local_planner {

namespace {

// Runtime timestamps are quantized to integer nanoseconds while nominal
// artifact timestamps are stored as decimal seconds.  At an exact phase
// boundary the two representations can therefore differ by less than one
// nanosecond.  Apply this tolerance only to the lookup key: the reported
// elapsed/artifact times remain the unmodified audit values.
constexpr double kArtifactLookupToleranceSec = 1e-9;

}  // namespace

void PhaseClock::reset() {
    initialized_ = false;
    runtime_origin_sec_ = 0.0;
    artifact_origin_sec_ = 0.0;
    last_runtime_sec_ = 0.0;
}

PhaseClockResult PhaseClock::update(
    const NominalSequenceArtifact& artifact,
    double runtime_time_sec,
    std::size_t max_index) {
    PhaseClockResult result;
    if (!artifact.valid() || artifact.empty()) {
        result.status = "ARTIFACT_UNAVAILABLE";
        return result;
    }
    if (!std::isfinite(runtime_time_sec)) {
        result.status = "CLOCK_NONFINITE";
        return result;
    }
    max_index = std::min(max_index, artifact.size() - 1);
    if (!initialized_) {
        initialized_ = true;
        runtime_origin_sec_ = runtime_time_sec;
        artifact_origin_sec_ = artifact.samples().front().t;
        last_runtime_sec_ = runtime_time_sec;
    } else if (runtime_time_sec + 1e-9 < last_runtime_sec_) {
        result.status = "CLOCK_REGRESSION";
        return result;
    }
    last_runtime_sec_ = runtime_time_sec;

    result.elapsed_sec = std::max(0.0, runtime_time_sec - runtime_origin_sec_);
    result.artifact_time_sec = artifact_origin_sec_ + result.elapsed_sec;

    const auto& samples = artifact.samples();
    const double lookup_time_sec =
        result.artifact_time_sec + kArtifactLookupToleranceSec;
    const auto upper = std::upper_bound(
        samples.begin(), samples.begin() + static_cast<std::ptrdiff_t>(max_index + 1),
        lookup_time_sec,
        [](double value, const PhaseNominalSample& sample) {
            return value < sample.t;
        });
    if (upper == samples.begin()) {
        result.index = 0;
    } else {
        result.index = static_cast<std::size_t>(
            std::distance(samples.begin(), upper - 1));
    }
    result.index = std::min(result.index, max_index);
    result.valid = true;
    result.status = result.index == max_index &&
                            result.artifact_time_sec > samples[max_index].t
        ? "CLOCK_AT_TAIL"
        : "OK";
    return result;
}

}  // namespace spmpc_local_planner
