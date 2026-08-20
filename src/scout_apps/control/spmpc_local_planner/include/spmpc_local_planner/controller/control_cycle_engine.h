#pragma once

#include "spmpc_local_planner/controller/command/command_pipeline.h"
#include "spmpc_local_planner/core/types.h"
#include "spmpc_local_planner/phase_rejoin/phase_rejoin_coordinator.h"
#include "spmpc_local_planner/safety/safety_supervisor.h"
#include "spmpc_local_planner/solver/api/solver_session.h"

#include <cstdint>
#include <string>

namespace spmpc_local_planner {

struct ControlCycleRequest {
    std::uint64_t cycle_id = 0;
    StampNs cycle_start_ns = 0;
    SolverInput solver_input;

    // Execution-front snapshot used exclusively by Phase-Rejoin.  The actual
    // solver origin is carried by solver_input and audited separately.
    bool prediction_valid = false;
    std::string prediction_status;
    RobotState execution_front_robot;
    SloshState execution_front_slosh;
    bool solver_origin_at_execution_front = false;
    int execution_front_steps = 0;
    double phase_time_sec = 0.0;
    double period_sec = 0.0;
};

struct ControlCycleResult {
    SolverInput solver_input;
    SolverOutput solver_output;
    SolverOutput output;

    bool solve_returned = false;
    bool solver_success = false;
    VelocityCommand solver_command;
    VelocityCommand terminal_command;
    VelocityCommand post_phase_command;
    CommandDecision decision;

    PhaseRejoinPreparation phase_preparation;
    PhaseRejoinDecision phase_decision;
    bool have_phase_decision = false;
    bool terminal_priority = false;
    bool terminal_controller_intervened = false;
    SafetySupervisorResult safety;
};

class ControlCycleEngine {
public:
    explicit ControlCycleEngine(SolverSession& solver_session);

    bool configurePhaseRejoin(const PhaseRejoinParams& params,
                              std::string& error);
    NominalArtifactLoadResult loadPhaseRejoinArtifact(
        const std::string& path);
    bool validatePhaseRejoinRuntimeContract(
        const PhaseRejoinRuntimeContract& runtime,
        const ReferencePath& reference,
        std::string& error);
    bool configureSafety(const SafetySupervisorConfig& config,
                         std::string& error);

    ControlCycleResult step(const ControlCycleRequest& request);

    void resetSafety();
    void resetForReference();
    bool phaseRejoinContractValid() const;
    PhaseRejoinDebugData makePhaseRejoinDebug(
        const PhaseRejoinPreparation* preparation,
        const PhaseRejoinDecision* decision) const;
    const PhaseRejoinCoordinator& phaseRejoinCoordinator() const {
        return phase_rejoin_;
    }

private:
    PhaseRejoinPreparation preparePhase(
        const ControlCycleRequest& request);
    static VelocityCommand rawSolverCommand(const SolverOutput& output);
    static TrackingProjectionView projectionView(
        const ProjectorDebugSummary& projector);

    SolverSession& solver_session_;
    PhaseRejoinCoordinator phase_rejoin_;
    SafetySupervisor safety_;
    PhaseRejoinParams phase_params_;
    bool goal_reached_latched_ = false;
};

}  // namespace spmpc_local_planner
