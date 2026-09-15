#include "spmpc_local_planner/reference/horizon_reference_builder.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace spmpc_local_planner {
namespace {
double linear(const StageTrajectoryReference& ref, const std::array<double, 4>& values, double s) {
    int segment = s < ref.s_knots[1] ? 0 : (s < ref.s_knots[2] ? 1 : 2);
    const double q = (s-ref.s_knots[segment])/(ref.s_knots[segment+1]-ref.s_knots[segment]);
    return values[segment]+q*(values[segment+1]-values[segment]);
}
}

std::vector<StageTrajectoryReference> HorizonReferenceBuilder::build(const TrajectoryReference& ref,
    const TrajectoryReferenceConfig& cfg, const std::vector<double>& progress, double elapsed, double dt) {
    if (!std::isfinite(elapsed) || elapsed < 0 || !std::isfinite(dt) || dt <= 0 ||
        !std::isfinite(cfg.progress_window) || cfg.progress_window <= 0 ||
        !std::isfinite(cfg.max_speed_error) || cfg.max_speed_error <= 0)
        throw std::invalid_argument("invalid horizon timing/interpolation config");
    if (cfg.mode != TrajectoryReferenceMode::Cruise && cfg.mode != TrajectoryReferenceMode::Progress &&
        cfg.mode != TrajectoryReferenceMode::FixedTime) throw std::invalid_argument("invalid reference mode");
    std::vector<StageTrajectoryReference> result(progress.size());
    if (cfg.mode == TrajectoryReferenceMode::Cruise) return result;
    const auto& samples = ref.plan().samples;
    const double lo = samples.front().state[4], hi = samples.back().state[4];
    if (hi-lo < 1e-6) throw std::invalid_argument("plan has no moving progress domain");
    for (size_t i = 0; i < progress.size(); ++i) {
        const double s = progress[i], time = elapsed+i*dt;
        if (!std::isfinite(s) || s < lo-1e-8 || s > hi+1e-8) throw std::out_of_range("stage progress outside plan");
        auto& out = result[i];
        const auto spatial = ref.sampleAtProgress(s, time);
        // Startup needs a time-domain launch reference even at v_s == 0.
        // Waiting/tail plateaus have no invertible s -> time map.
        const bool time_mode = cfg.mode == TrajectoryReferenceMode::FixedTime ||
            s <= lo+1e-9 || s >= hi-1e-9 || ref.atPlateau(s);
        out.mode = time_mode ? 2 : 1;
        auto selected = time_mode ? ref.sampleAtTime(time) : spatial;
        if (time_mode && cfg.mode == TrajectoryReferenceMode::Progress && s >= hi-1e-9)
            selected = ref.sampleAtTime(std::max(time, spatial.t));
        out.nominal_time = selected.t;
        out.phase = selected.phase;
        double window = cfg.progress_window;
        bool accepted = false;
        for (int attempt = 0; attempt < 25; ++attempt, window *= 0.5) {
            const double left = std::max(lo, s-window), right = std::min(hi, s+window);
            if (right-left < 1e-6) break;
            for (int k = 0; k < 4; ++k) {
                out.s_knots[k] = left+(right-left)*k/3.0;
                const auto point = ref.sampleAtProgress(out.s_knots[k], time);
                out.x_knots[k] = point.state[0]; out.y_knots[k] = point.state[1];
                out.v_knots[k] = time_mode ? selected.state[3] : point.state[3];
                out.vs_knots[k] = time_mode ? selected.control[2] : point.control[2];
            }
            accepted = true;
            if (!time_mode) {
                // Difference of two piecewise linear profiles is linear on
                // their common subdivision. Check every original breakpoint;
                // approximation knots/endpoints are exact by construction.
                for (const auto& row : samples) if (row.state[4] >= left && row.state[4] <= right) {
                    const auto exact = ref.sampleAtProgress(row.state[4], time);
                    if (std::abs(linear(out, out.v_knots, row.state[4])-exact.state[3]) > cfg.max_speed_error ||
                        std::abs(linear(out, out.vs_knots, row.state[4])-exact.control[2]) > cfg.max_speed_error) {
                        accepted = false; break;
                    }
                }
            }
            if (accepted) break;
        }
        if (!accepted) throw std::invalid_argument("cannot bound spatial reference interpolation error");
    }
    return result;
}
}  // namespace spmpc_local_planner
