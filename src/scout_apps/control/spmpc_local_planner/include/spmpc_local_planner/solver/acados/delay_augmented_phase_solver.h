#pragma once

#include "spmpc_local_planner/solver/api/execution_horizon_context.h"
#include "spmpc_local_planner/solver/acados/delay_augmented_phase_parameter_builder.h"
#include "spmpc_local_planner/solver/delay_augmented/phase_rejoin_dynamics.h"
#include "spmpc_local_planner/dynamics/slosh_dynamics.h"

#include <cstdint>
#include <memory>
#include <string>

namespace spmpc_local_planner {

enum DelayAugmentedPhaseSolverCapability : std::uint32_t {
    DELAY_AUGMENTED_DISCRETE_DYNAMICS = 1u << 0,
    DELAY_AUGMENTED_INITIAL_STATE = 1u << 1,
    DELAY_AUGMENTED_PUBLISHED_COMMAND_BOUNDS = 1u << 2,
    DELAY_AUGMENTED_ROBOT_SPEED_BOUNDS = 1u << 3,
    DELAY_AUGMENTED_PUBLISHED_RATE_BOUNDS = 1u << 4,
    DELAY_AUGMENTED_TERMINAL_EMPIRICAL_GATE = 1u << 5,
    DELAY_AUGMENTED_EXECUTION_COMPATIBILITY_SET = 1u << 6,
    DELAY_AUGMENTED_PUBLISHED_RESIDUAL_BOUNDS = 1u << 7,
};

constexpr std::uint32_t kDelayAugmentedPhaseWp3cCapabilities =
    DELAY_AUGMENTED_DISCRETE_DYNAMICS |
    DELAY_AUGMENTED_INITIAL_STATE |
    DELAY_AUGMENTED_PUBLISHED_COMMAND_BOUNDS |
    DELAY_AUGMENTED_ROBOT_SPEED_BOUNDS |
    DELAY_AUGMENTED_PUBLISHED_RATE_BOUNDS;

constexpr std::uint32_t kDelayAugmentedPhaseFormalCapabilities =
    kDelayAugmentedPhaseWp3cCapabilities |
    DELAY_AUGMENTED_TERMINAL_EMPIRICAL_GATE |
    DELAY_AUGMENTED_EXECUTION_COMPATIBILITY_SET |
    DELAY_AUGMENTED_PUBLISHED_RESIDUAL_BOUNDS;

struct DelayAugmentedPhaseCompiledContract {
    ExecutionModelContract execution;
    SloshModelParams slosh;
    int state_width = 0;
    int control_width = 0;
    int horizon_steps = 0;
    int execution_front_steps = 0;
    int liquid_horizon_steps = 0;
    int parameter_schema_version = 0;
    std::string parameter_schema_id;
    std::string parameter_schema_hash;
    std::string execution_compatibility_contract;
    std::uint32_t capabilities = 0;
    double acceleration_max = 0.0;
    double angular_acceleration_max = 0.0;
    double progress_rate_max = 0.0;
};

struct DelayAugmentedPhaseSolveDiagnostics {
    bool optimizer_invoked = false;
    bool evaluated = false;
    bool residual_admitted = false;
    int nlp_status = -1;
    int qp_status = -1;
    double stationarity_residual = 0.0;
    double equality_residual = 0.0;
    double inequality_residual = 0.0;
    double complementarity_residual = 0.0;
    std::string status = "NOT_EVALUATED";
};

// Independent owner for the development nx=22, nu=3, N=10 DISCRETE capsule.
// The explicit opt-in online backend owns this class. create() validates the
// complete generated contract and requested capability mask before allocating
// an acados capsule.
class DelayAugmentedPhaseAcadosSolver {
public:
    DelayAugmentedPhaseAcadosSolver();
    ~DelayAugmentedPhaseAcadosSolver();
    DelayAugmentedPhaseAcadosSolver(
        const DelayAugmentedPhaseAcadosSolver&) = delete;
    DelayAugmentedPhaseAcadosSolver& operator=(
        const DelayAugmentedPhaseAcadosSolver&) = delete;

    static std::uint32_t compiledCapabilities();
    static DelayAugmentedPhaseCompiledContract compiledContract();
    static bool validateContextContract(
        const ExecutionHorizonContext& context,
        std::uint32_t required_capabilities,
        std::string& error);
    static bool compiled();

    bool create(
        const ExecutionHorizonContext& context,
        std::uint32_t required_capabilities,
        std::string& error);
    void reset();
    bool ready() const;
    int stateWidth() const;
    int controlWidth() const;
    int horizonSteps() const;
    bool setControlGuess(int stage, const double* control);
    bool setCausalWarmStart(
        const ExecutionHorizonContext& context,
        const std::vector<DelayAugmentedPhaseControl>& controls,
        std::string& error);
    bool setParameterImage(
        const DelayAugmentedPhaseParameterMatrix& parameters,
        std::string& error);
    bool getState(int stage, double* state) const;
    bool getControl(int stage, double* control) const;
    int solve();
    double solveTimeSec() const;
    const DelayAugmentedPhaseSolveDiagnostics& lastSolveDiagnostics() const;
    static bool residualsAdmissible(
        const DelayAugmentedPhaseSolveDiagnostics& diagnostics);
    bool causalRollout(
        const ExecutionHorizonContext& context,
        DelayAugmentedPhaseRolloutResult& rollout,
        std::string& error) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace spmpc_local_planner
