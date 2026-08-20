#pragma once

#include "spmpc_local_planner/domain/state.h"
#include "spmpc_local_planner/phase_rejoin/types.h"

namespace spmpc_local_planner {

class EmpiricalRecoveryGate {
public:
    EmpiricalRecoveryGateResult evaluate(const PhaseNominalSample& nominal,
                                         const RobotState& robot,
                                         const SloshState& slosh) const;

    static bool validRadii(const EmpiricalRecoveryRadii& radii);
};

}  // namespace spmpc_local_planner
