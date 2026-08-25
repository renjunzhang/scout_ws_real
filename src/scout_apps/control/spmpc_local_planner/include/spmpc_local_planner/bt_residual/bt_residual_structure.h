#pragma once

#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/phase_rejoin/bounded_tracking_recovery_policy.h"
#include "spmpc_local_planner/phase_rejoin/types.h"
#include "spmpc_local_planner/runtime/execution_prediction/execution_model.h"

#include <Eigen/Dense>

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace spmpc_local_planner {
namespace bt_residual {

constexpr int kStateWidth = 15;
constexpr int kResidualWidth = 2;

// Build-time source identity for the frozen evidence core.  The campaign
// executable rejects a clean working tree whose freshly invoked binary was
// compiled for a different Git HEAD.
const char* compiledSourceHead();

using StateVector = Eigen::Matrix<double, kStateWidth, 1>;
using ResidualVector = Eigen::Matrix<double, kResidualWidth, 1>;
using StateMatrix = Eigen::Matrix<double, kStateWidth, kStateWidth>;
using ResidualMatrix =
    Eigen::Matrix<double, kStateWidth, kResidualWidth>;

// CR0 method contract.  It deliberately contains no optimizer phase state,
// empirical recovery gate, or Tail-Commit switch.  The artifact index is the
// only prediction phase and progress_s is diagnostic physical progress.
struct StructuralContract {
    std::string schema = "spmpc_bt_residual_structural_contract_v1";
    std::string implementation_id =
        "bt_centered_residual_terminal_mpc_v1";
    std::string claim_level =
        "linearized_deterministic_development_only";
    std::string expected_artifact_sha256;
    std::string expected_artifact_contract_id;
    std::string expected_execution_contract_hash;
    std::string expected_bt_policy_contract_id =
        "bounded_tracking_recovery_policy_v1";

    int residual_prefix_steps = 0;
    int recovery_suffix_steps = 0;
    std::size_t authority_taper_begin_index = 0;
    std::size_t authority_zero_index = 0;

    double maximum_published_acceleration = 0.0;
    double maximum_published_angular_acceleration = 0.0;
    double maximum_residual_v = 0.0;
    double maximum_residual_omega = 0.0;
    double maximum_residual_slew_v = 0.0;
    double maximum_residual_slew_omega = 0.0;
    double cumulative_progress_budget_m = 0.0;
    double cumulative_yaw_budget_rad = 0.0;
    double finite_difference_relative_step = 0.0;
    double maximum_finite_difference_reconstruction_error = 0.0;
    double identity_tolerance = 0.0;

    StateVector finite_difference_scales = StateVector::Zero();
    StateVector candidate_path_deviation_bounds = StateVector::Zero();
    StateVector recovery_path_deviation_bounds = StateVector::Zero();
    StateVector terminal_deviation_bounds = StateVector::Zero();
    StateVector terminal_absolute_bounds = StateVector::Zero();

    double terminal_liquid_increment_eta = 0.0;
    double terminal_liquid_increment_eta_dot = 0.0;
    double minimum_relative_tracking_improvement = 0.0;
    double minimum_absolute_tracking_improvement = 0.0;
    double minimum_nonzero_residual = 0.0;
    double model_dominance_margin = 0.0;
    double maximum_absolute_eta = 0.0;
    double maximum_absolute_eta_dot = 0.0;
};

bool validateStructuralContract(const StructuralContract& contract,
                                const ExecutionModelContract& execution,
                                std::string& error);

double residualAuthority(const StructuralContract& contract,
                         std::size_t phase_index);

struct AugmentedState15 {
    ExecutionAugmentedState execution;
    double progress_s = 0.0;
};

struct StagePublicationConstraint {
    bool linear_cap_active = false;
    double maximum_linear = 0.0;
};

struct ClosedLoopStepResult {
    bool valid = false;
    std::string status = "NOT_RUN";
    AugmentedState15 state;
    VelocityCommand bt_command;
    VelocityCommand published_command;
    ResidualVector applied_residual = ResidualVector::Zero();
    double authority = 0.0;
    bool bt_rate_limited = false;
    bool linear_cap_active = false;
    bool linear_cap_modified = false;
    double integrated_progress_m = 0.0;
};

struct ClosedLoopRolloutResult {
    bool valid = false;
    std::string status = "NOT_RUN";
    std::size_t initial_phase_index = 0;
    std::vector<AugmentedState15> states;
    std::vector<VelocityCommand> bt_commands;
    std::vector<VelocityCommand> published_commands;
    std::vector<ResidualVector> residuals;
};

// Independent zero-residual oracle used only by the frozen CR0--CR2 audit.
// It deliberately does not call BtClosedLoopModel::step/rollout: the
// implementation composes the frozen BT policy, production publication
// transaction and a separately configured ExecutionModel.
struct IndependentBtOracleRolloutResult {
    bool valid = false;
    std::string status = "NOT_RUN";
    std::size_t initial_phase_index = 0;
    std::vector<AugmentedState15> states;
    std::vector<VelocityCommand> published_commands;
};

IndependentBtOracleRolloutResult rolloutIndependentBtOracle(
    const ExecutionModelContract& execution,
    const SloshModelParams& slosh,
    const StructuralContract& structure,
    const std::vector<PhaseNominalSample>& samples,
    const AugmentedState15& initial_state,
    std::size_t initial_phase_index,
    std::size_t step_count,
    const std::vector<StagePublicationConstraint>& publications = {});

class BtClosedLoopModel {
public:
    bool configure(const ExecutionModelContract& execution,
                   const SloshModelParams& slosh,
                   const StructuralContract& structure,
                   const std::vector<PhaseNominalSample>* samples,
                   std::string& error);

    AugmentedState15 artifactState(std::size_t phase_index) const;

    ClosedLoopStepResult step(
        const AugmentedState15& state,
        std::size_t phase_index,
        const ResidualVector& residual = ResidualVector::Zero(),
        const StagePublicationConstraint& publication =
            StagePublicationConstraint{}) const;

    ClosedLoopRolloutResult rollout(
        const AugmentedState15& initial_state,
        std::size_t initial_phase_index,
        const std::vector<ResidualVector>& residuals,
        const std::vector<StagePublicationConstraint>& publications = {})
        const;

    StateVector pack(const AugmentedState15& state) const;
    bool unpack(const StateVector& packed,
                std::uint64_t stage_index,
                AugmentedState15& state,
                std::string& error) const;
    StateVector difference(const AugmentedState15& lhs,
                           const AugmentedState15& rhs) const;

    const StructuralContract& structure() const { return structure_; }
    const ExecutionModelContract& executionContract() const {
        return execution_model_.contract();
    }
    bool configured() const { return configured_; }

private:
    ExecutionModel execution_model_;
    BoundedTrackingRecoveryPolicy bt_policy_;
    StructuralContract structure_;
    const std::vector<PhaseNominalSample>* samples_ = nullptr;
    bool configured_ = false;
};

enum class DifferenceScheme {
    Invalid = 0,
    Central = 1,
    Forward = 2,
    Backward = 3,
    AuthorityZero = 4,
};

struct ClosedLoopLinearization {
    bool valid = false;
    std::string status = "NOT_RUN";
    std::size_t phase_index = 0;
    AugmentedState15 center;
    ClosedLoopStepResult nominal_step;
    StateMatrix a = StateMatrix::Zero();
    // Elementwise bound over the forward/backward directional Jacobians.
    // The backward box predecessor uses this matrix at limiter kinks instead
    // of pretending the central Jacobian is the only local branch.
    StateMatrix a_absolute_bound = StateMatrix::Zero();
    ResidualMatrix b = ResidualMatrix::Zero();
    ResidualMatrix b_absolute_bound = ResidualMatrix::Zero();
    Eigen::Matrix<double, kResidualWidth, kStateWidth>
        bt_command_state_jacobian =
            Eigen::Matrix<double, kResidualWidth, kStateWidth>::Zero();
    std::array<DifferenceScheme, kStateWidth> a_schemes{};
    std::array<DifferenceScheme, kResidualWidth> b_schemes{};
    double maximum_reconstruction_error = 0.0;
    double maximum_directional_asymmetry = 0.0;
};

ClosedLoopLinearization linearizeClosedLoop(
    const BtClosedLoopModel& model,
    const AugmentedState15& center,
    std::size_t phase_index,
    const StagePublicationConstraint& publication =
        StagePublicationConstraint{});

// A development-only phase-indexed tube represented as the backward linear
// map from a local 15D deviation to the final 15D deviation.  Membership is a
// necessary linear check; callers must additionally run the nonlinear
// BT-only suffix before treating a state as recoverable.
struct RecoverableTubeStage {
    bool valid = false;
    std::size_t phase_index = 0;
    AugmentedState15 center;
    StateMatrix terminal_map = StateMatrix::Identity();
    StateVector half_width = StateVector::Zero();
};

struct RecoverableTube {
    bool valid = false;
    std::string status = "NOT_RUN";
    std::string claim_level;
    std::vector<ClosedLoopLinearization> linearizations;
    std::vector<RecoverableTubeStage> stages;
};

RecoverableTube buildLinearizedRecoverableTube(
    const BtClosedLoopModel& model,
    const AugmentedState15& initial_center,
    std::size_t initial_phase_index,
    std::size_t terminal_phase_index);

struct TubeMembershipResult {
    bool valid = false;
    bool inside = false;
    std::string status = "NOT_RUN";
    double minimum_margin = 0.0;
    StateVector predicted_terminal_deviation = StateVector::Zero();
};

TubeMembershipResult evaluateTubeMembership(
    const BtClosedLoopModel& model,
    const RecoverableTube& tube,
    const AugmentedState15& state,
    std::size_t phase_index);

struct TerminalRecoveryResult {
    bool valid = false;
    bool recovered = false;
    bool nonlinear_recovered = false;
    bool nonlinear_rollout_completed = false;
    bool nonlinear_path_passed = false;
    bool tube_path_passed = false;
    bool terminal_contract_passed = false;
    bool liquid_path_passed = false;
    std::string status = "NOT_RUN";
    AugmentedState15 terminal_state;
    StateVector terminal_error = StateVector::Zero();
    double maximum_eta = 0.0;
    double maximum_eta_dot = 0.0;
};

TerminalRecoveryResult auditNonlinearBtRecovery(
    const BtClosedLoopModel& model,
    const AugmentedState15& state,
    std::size_t phase_index,
    const RecoverableTube& tube);

}  // namespace bt_residual
}  // namespace spmpc_local_planner
