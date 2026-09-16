#include "spmpc_local_planner/reference/trajectory_reference.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <limits>

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

TrajectoryReference::TrajectoryReference(std::shared_ptr<const TrajectoryPlan> plan,
                                         const ProgressProjectionConfig& projection_config)
    : plan_(std::move(plan)), projector_(projection_config) {
    if (!plan_) throw std::invalid_argument("null trajectory plan");
    plan_->validate();
    double length=0;
    for (size_t i=1;i<plan_->route.size();++i)
        length+=std::hypot(plan_->route[i].x-plan_->route[i-1].x,plan_->route[i].y-plan_->route[i-1].y);
    for (const auto& row:plan_->samples) {
        const double s=std::max(progress_.empty()?0.:progress_.back(),std::min(length,row.state[4]));
        progress_.push_back(s);
        TrajectoryPoint point;
        point.x = row.state[0];
        point.y = row.state[1];
        point.yaw = row.state[2];
        point.v = row.state[3];
        point.s = s;
        projection_points_.push_back(point);
    }
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
    for (size_t i = 1; i < progress_.size(); ++i)
        if (std::abs(progress_[i]-progress) <= 1e-9 &&
            std::abs(progress_[i-1]-progress) <= 1e-9) return true;
    return false;
}

ProgressProjection TrajectoryReference::project(double x, double y, double minimum_progress) const {
    ProgressProjectionState state;
    return project(x, y, state, minimum_progress);
}

ProgressProjection TrajectoryReference::project(double x, double y,
                                                 ProgressProjectionState& state,
                                                 double minimum_progress) const {
    // state[4] is the plan's progress coordinate.  Passing the cached points
    // through the shared projector preserves that coordinate even when its
    // scale differs from the route's geometric arc length.
    return projector_.project(projection_points_, x, y, state, minimum_progress, false);
}

TrajectoryPlanSample TrajectoryReference::sampleAtProgress(double progress, double elapsed) const {
    if (!std::isfinite(progress) || !std::isfinite(elapsed) || elapsed < 0)
        throw std::invalid_argument("invalid progress/time");
    const auto& samples = plan_->samples;
    if (progress < progress_.front()-kProgressTolerance || progress > progress_.back()+kProgressTolerance)
        throw std::out_of_range("progress outside trajectory");
    progress = std::max(progress_.front(), std::min(progress_.back(), progress));
    size_t index=std::lower_bound(progress_.begin(),progress_.end(),progress)-progress_.begin();
    if (index==samples.size()) return samples.back();
    if (std::abs(progress_[index]-progress) <= 1e-10) {
        size_t last=index;
        while (last+1<samples.size() && std::abs(progress_[last+1]-progress)<=1e-10) ++last;
        if (last!=index) return sampleAtTime(std::max(samples[index].t,std::min(samples[last].t,elapsed)));
        return samples[index];
    }
    if (index==0) return samples.front();
    return interpolate(samples[index-1],samples[index],
        (progress-progress_[index-1])/(progress_[index]-progress_[index-1]));
}
}  // namespace spmpc_local_planner
