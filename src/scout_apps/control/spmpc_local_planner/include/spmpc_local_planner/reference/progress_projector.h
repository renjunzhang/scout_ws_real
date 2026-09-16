#pragma once

#include "spmpc_local_planner/reference/reference_path.h"
#include <cstddef>
#include <limits>
#include <vector>

namespace spmpc_local_planner {

struct ProgressProjection {
    bool valid = false;
    double s = 0.0;
    double distance = 0.0;
    double signed_distance = 0.0;
    TrajectoryPoint point;
};

// Projection continuity is deliberately caller-owned.  A task reset should
// call reset(), after which the next projection is allowed to relocalize on
// the complete path.  While initialized, the projector only searches a
// bounded progress window around the last accepted branch.
struct ProgressProjectionState {
    bool initialized = false;
    double progress = 0.0;

    void reset() {
        initialized = false;
        progress = 0.0;
    }
};

struct ProgressProjectionConfig {
    // These are in the path's progress units.  For ReferencePath that is
    // geometric metres; a planned trajectory may use a different progress
    // coordinate and must not be silently converted to arc length.
    double lookahead = 2.0;
    // Distances within this tolerance are considered the same local minimum;
    // scanning order then retains the earlier branch at a foldback/crossing.
    double local_minimum_tolerance = 1e-9;
};

class ProgressProjector {
public:
    explicit ProgressProjector(const ProgressProjectionConfig& config = ProgressProjectionConfig{})
        : config_(config) {}

    const ProgressProjectionConfig& config() const { return config_; }

    // Compatibility entry points perform an uninitialized (global) lookup,
    // subject only to the supplied lower progress bound.  Callers that run
    // over multiple cycles should retain a ProgressProjectionState and use
    // the state overload below.
    ProgressProjection project(const ReferencePath& reference, double x, double y) const;
    ProgressProjection project(const ReferencePath& reference, double x, double y, double min_s) const;

    ProgressProjection project(const ReferencePath& reference, double x, double y,
                               ProgressProjectionState& state, double min_s = 0.0) const;

    // Generic form used by planned references whose point.s values are a
    // non-geometric progress coordinate.  The points must be ordered by
    // nondecreasing s, as produced by ReferencePath or TrajectoryReference.
    ProgressProjection project(const std::vector<TrajectoryPoint>& points, double x, double y,
                               ProgressProjectionState& state, double min_s = 0.0,
                               bool derive_yaw_from_geometry = true) const;

private:
    ProgressProjectionConfig config_;
};

}  // namespace spmpc_local_planner
