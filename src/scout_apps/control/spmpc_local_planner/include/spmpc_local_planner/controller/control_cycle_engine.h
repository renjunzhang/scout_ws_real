#pragma once

#include "spmpc_local_planner/controller/command/command_pipeline.h"
#include "spmpc_local_planner/controller/speed_reference_controller.h"
#include "spmpc_local_planner/controller/control_cycle_telemetry.h"
#include "spmpc_local_planner/solver/api/solver_io.h"
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
    PhaseRejoinDebugData phase_debug;
    bool have_phase_decision = false;
    bool terminal_priority = false;
    bool terminal_controller_intervened = false;
    SafetySupervisorResult safety;
    ControlCycleTelemetrySnapshot telemetry;
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
    bool configureCommandPipeline(const CommandPipelineConfig& config,
                                  std::string& error);
    SpeedReferenceConfigureResult configureSpeedReference(
        const SpeedReferenceControllerConfig& config);
    SpeedReferenceEvaluation prepareSpeedReference(SolverInput& input);
    CommandPipelineResult finalizeCommand(const CommandDecision& decision,
                                          StampNs stamp_ns,
                                          bool publish_enabled);
    CommandPipelineResult finalizeFailClosedZero(
        StampNs stamp_ns,
        bool publish_enabled,
        const std::string& reason);

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
    CommandPipeline command_pipeline_;
    SpeedReferenceController speed_reference_;
    PhaseRejoinParams phase_params_;
    bool goal_reached_latched_ = false;
    bool have_previous_shifted_plan_ = false;
    std::uint64_t previous_plan_cycle_id_ = 0;
    double previous_shifted_plan_a_ = 0.0;
    double previous_shifted_plan_alpha_ = 0.0;
};

}  // namespace spmpc_local_planner
