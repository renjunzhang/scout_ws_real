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

TrajectoryReference::TrajectoryReference(std::shared_ptr<const TrajectoryPlan> plan) : plan_(std::move(plan)) {
    if (!plan_) throw std::invalid_argument("null trajectory plan");
    plan_->validate();
    double length=0;
    for (size_t i=1;i<plan_->route.size();++i)
        length+=std::hypot(plan_->route[i].x-plan_->route[i-1].x,plan_->route[i].y-plan_->route[i-1].y);
    for (const auto& row:plan_->samples) {
        const double s=std::max(progress_.empty()?0.:progress_.back(),std::min(length,row.state[4]));
        progress_.push_back(s);
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
    ProgressProjection best;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(minimum_progress)) return best;
    const auto& rows=plan_->samples;
    const double lower=std::max(progress_.front(), std::min(progress_.back(),minimum_progress));
    double best_squared=std::numeric_limits<double>::infinity();
    for (size_t k=0;k+1<rows.size();++k) {
        const auto& a=rows[k].state; const auto& b=rows[k+1].state;
        const double sa=progress_[k],sb=progress_[k+1];
        if (sb<lower) continue;
        const double dx=b[0]-a[0],dy=b[1]-a[1], norm=dx*dx+dy*dy;
        const double min_q=sb>sa+1e-12 ? std::max(0.,std::min(1.,(lower-sa)/(sb-sa))) : 0.;
        const double q=norm>1e-16 ? std::max(min_q,std::min(1.,((x-a[0])*dx+(y-a[1])*dy)/norm)) : min_q;
        const double px=a[0]+q*dx,py=a[1]+q*dy;
        const double squared=(x-px)*(x-px)+(y-py)*(y-py);
        // Prefer the first progress on coincident waiting/launch samples.
        if (squared >= best_squared-1e-14) continue;
        best_squared=squared; best.valid=true;
        best.s=std::max(lower,std::min(progress_.back(),sa+q*(sb-sa))); // Not geometric arc length.
        best.point={px,py,a[2]+q*(b[2]-a[2]),a[3]+q*(b[3]-a[3]),best.s};
        best.distance=std::sqrt(squared);
        best.signed_distance=-std::sin(best.point.yaw)*(x-px)+std::cos(best.point.yaw)*(y-py);
    }
    return best;
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
