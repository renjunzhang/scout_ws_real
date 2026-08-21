#pragma once

#include "spmpc_local_planner/domain/state.h"
#include "spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h"
#include "spmpc_local_planner/phase_rejoin/types.h"

#include <cstddef>

namespace spmpc_local_planner {

class PhaseCandidateSelector {
public:
    bool configure(const PhaseCandidateSelectorParams& params);

    PhaseCandidateResult select(const NominalSequenceArtifact& artifact,
                                const RobotState& execution_front_robot,
                                const SloshState& execution_front_slosh,
                                int front_steps,
                                int liquid_steps,
                                std::size_t clock_index,
                                bool have_last_accepted,
                                std::size_t last_accepted_index,
                                bool observation_at_execution_front = true) const;

    const PhaseCandidateSelectorParams& params() const { return params_; }

private:
    double score(const PhaseNominalSample& nominal,
                 const RobotState& robot,
                 const SloshState& slosh) const;

    PhaseCandidateSelectorParams params_;
    bool configured_ = false;
};

}  // namespace spmpc_local_planner
