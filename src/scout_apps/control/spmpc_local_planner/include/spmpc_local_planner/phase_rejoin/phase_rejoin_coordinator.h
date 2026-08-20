#pragma once

#include "spmpc_local_planner/solver/api/solver_io.h"
#include "spmpc_local_planner/phase_rejoin/empirical_recovery_gate.h"
#include "spmpc_local_planner/phase_rejoin/nominal_sequence_artifact.h"
#include "spmpc_local_planner/phase_rejoin/phase_candidate_selector.h"
#include "spmpc_local_planner/phase_rejoin/phase_clock.h"
#include "spmpc_local_planner/phase_rejoin/types.h"
#include "spmpc_local_planner/reference/reference_path.h"

#include <cstddef>
#include <string>

namespace spmpc_local_planner {

class PhaseRejoinCoordinator {
public:
    bool configure(const PhaseRejoinParams& params, std::string& error);
    NominalArtifactLoadResult loadArtifact(const std::string& path);
    bool setArtifact(const NominalSequenceArtifact& artifact,
                     std::string& error);

    bool validateRuntimeContract(const PhaseRejoinRuntimeContract& runtime,
                                 const ReferencePath& reference,
                                 std::string& error);
    void resetProgress();

    PhaseRejoinPreparation prepare(const RobotState& execution_front_robot,
                                   const SloshState& execution_front_slosh,
                                   int front_steps,
                                   int solver_horizon_steps,
                                   double phase_time_sec,
                                   bool solver_origin_at_execution_front = true);

    PhaseRejoinDecision decide(const PhaseRejoinPreparation& preparation,
                               const RobotState& execution_front_robot,
                               const SloshState& execution_front_slosh,
                               bool solver_success,
                               const SolverOutput& solver_output) const;

    void commit(const PhaseRejoinPreparation& preparation,
                const PhaseRejoinDecision& decision);

    PhaseRejoinDebugData makeDebug(
        const PhaseRejoinPreparation* preparation,
        const PhaseRejoinDecision* decision) const;

    const PhaseRejoinParams& params() const { return params_; }
    const NominalSequenceArtifact& artifact() const { return artifact_; }
    bool configured() const { return configured_; }
    bool contractValid() const { return contract_valid_; }
    bool haveAcceptedIndex() const { return have_accepted_index_; }
    std::size_t acceptedIndex() const { return accepted_index_; }
    bool terminalReleaseAuthorized() const {
        return terminal_release_authorized_;
    }

private:
    static PhaseNominalStage makeStage(const PhaseNominalSample& sample,
                                       bool gate_active);
    static RobotState robotFromHorizon(const HorizonStateDebug& state);
    static SloshState sloshFromHorizon(const HorizonStateDebug& state);

    PhaseRejoinParams params_;
    NominalSequenceArtifact artifact_;
    PhaseCandidateSelector selector_;
    PhaseClock phase_clock_;
    EmpiricalRecoveryGate gate_;
    bool configured_ = false;
    bool contract_valid_ = false;
    bool have_accepted_index_ = false;
    std::size_t accepted_index_ = 0;
    bool terminal_release_authorized_ = false;
    std::string contract_status_ = "NOT_VALIDATED";
};

}  // namespace spmpc_local_planner
