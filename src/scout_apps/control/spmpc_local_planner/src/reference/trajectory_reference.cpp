#include "spmpc_local_planner/reference/trajectory_reference.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace spmpc_local_planner {
namespace {
TrajectoryPlanSample interpolate(const TrajectoryPlanSample& a, const TrajectoryPlanSample& b, double q) {
    TrajectoryPlanSample result = a;
    result.t = a.t + q*(b.t-a.t);
    for (size_t i = 0; i < a.state.size(); ++i) result.state[i] = a.state[i]+q*(b.state[i]-a.state[i]);
    for (size_t i = 0; i < 3; ++i) result.control[i] = a.control[i]+q*(b.control[i]-a.control[i]);
    // Phase transitions happen at the stored timestamp, not halfway through it.
    result.phase = q >= 1-1e-12 ? b.phase : a.phase;
    return result;
}
}

TrajectoryReference::TrajectoryReference(std::shared_ptr<const TrajectoryPlan> plan) : plan_(std::move(plan)) {
    if (!plan_) throw std::invalid_argument("null trajectory plan");
    plan_->validate();
}

TrajectoryPlanSample TrajectoryReference::sampleAtTime(double time) const {
    if (!std::isfinite(time) || time < 0) throw std::invalid_argument("invalid sample time");
    const auto& samples = plan_->samples;
    if (time >= samples.back().t) return samples.back();
    auto upper = std::upper_bound(samples.begin(), samples.end(), time,
        [](double t, const TrajectoryPlanSample& row) { return t < row.t; });
    if (upper == samples.begin()) return *upper;
    const auto& a = *(upper-1);
    return interpolate(a, *upper, (time-a.t)/(upper->t-a.t));
}

bool TrajectoryReference::atPlateau(double progress) const {
    const auto& samples = plan_->samples;
    for (size_t i = 1; i < samples.size(); ++i)
        if (std::abs(samples[i].state[4]-progress) <= 1e-9 &&
            std::abs(samples[i-1].state[4]-progress) <= 1e-9) return true;
    return false;
}

TrajectoryPlanSample TrajectoryReference::sampleAtProgress(double progress, double elapsed) const {
    if (!std::isfinite(progress) || !std::isfinite(elapsed) || elapsed < 0)
        throw std::invalid_argument("invalid progress/time");
    const auto& samples = plan_->samples;
    if (progress < samples.front().state[4]-1e-8 || progress > samples.back().state[4]+1e-8)
        throw std::out_of_range("progress outside trajectory");
    progress = std::max(samples.front().state[4], std::min(samples.back().state[4], progress));
    auto lower = std::lower_bound(samples.begin(), samples.end(), progress,
        [](const TrajectoryPlanSample& row, double s) { return row.state[4] < s; });
    if (lower == samples.end()) return samples.back();
    if (std::abs(lower->state[4]-progress) <= 1e-10) {
        auto last = lower;
        while (last+1 != samples.end() && std::abs((last+1)->state[4]-progress) <= 1e-10) ++last;
        if (last != lower) return sampleAtTime(std::max(lower->t, std::min(last->t, elapsed)));
        return *lower;
    }
    if (lower == samples.begin()) return *lower;
    const auto& a = *(lower-1);
    return interpolate(a, *lower, (progress-a.state[4])/(lower->state[4]-a.state[4]));
}
}  // namespace spmpc_local_planner
