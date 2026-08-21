#pragma once

#include "spmpc_local_planner/solver/api/execution_horizon_context.h"

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
    DELAY_AUGMENTED_EXECUTION_COMPATIBILITY_SET;

// Independent owner for the candidate nx=22, nu=3, N=10 DISCRETE capsule.
// No runtime factory selects this class yet; the online development solver
// therefore remains unchanged.  create() validates the complete generated
// contract and requested capability mask before allocating an acados capsule.
class DelayAugmentedPhaseAcadosSolver {
public:
    DelayAugmentedPhaseAcadosSolver();
    ~DelayAugmentedPhaseAcadosSolver();
    DelayAugmentedPhaseAcadosSolver(
        const DelayAugmentedPhaseAcadosSolver&) = delete;
    DelayAugmentedPhaseAcadosSolver& operator=(
        const DelayAugmentedPhaseAcadosSolver&) = delete;

    static std::uint32_t compiledCapabilities();
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
    bool getState(int stage, double* state) const;
    bool getControl(int stage, double* control) const;
    int solve();
    double solveTimeSec() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace spmpc_local_planner
