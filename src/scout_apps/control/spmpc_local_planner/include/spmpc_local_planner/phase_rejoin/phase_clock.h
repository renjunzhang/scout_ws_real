#pragma once

#include "spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h"

#include <cstddef>
#include <string>

namespace spmpc_local_planner {

struct PhaseClockResult {
    bool valid = false;
    std::size_t index = 0;
    double elapsed_sec = 0.0;
    double artifact_time_sec = 0.0;
    std::string status = "NOT_RUN";
};

// Maps an absolute runtime clock to the frozen artifact time axis.  The clock
// never advances relative to a previously selected candidate, so a temporary
// future rejoin cannot accumulate into an implicit free-time scaling.
class PhaseClock {
public:
    void reset();

    PhaseClockResult update(const NominalSequenceArtifact& artifact,
                            double runtime_time_sec,
                            std::size_t max_index);

    bool initialized() const { return initialized_; }
    std::size_t index() const { return last_index_; }

private:
    bool initialized_ = false;
    double runtime_origin_sec_ = 0.0;
    double artifact_origin_sec_ = 0.0;
    double last_runtime_sec_ = 0.0;
    std::size_t last_index_ = 0;
};

}  // namespace spmpc_local_planner
