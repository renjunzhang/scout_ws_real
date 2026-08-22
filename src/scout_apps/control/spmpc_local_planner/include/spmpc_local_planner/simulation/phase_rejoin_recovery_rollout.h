#pragma once

#include "spmpc_local_planner/dynamics/slosh_dynamics.h"
#include "spmpc_local_planner/phase_rejoin/bounded_tracking_recovery_policy.h"
#include "spmpc_local_planner/phase_rejoin/types.h"
#include "spmpc_local_planner/simulation/independent_scout_liquid_plant.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace spmpc_local_planner {
namespace simulation {

// One open-loop, pre-terminal residual pulse.  The pulse is selected from a
// frozen table using only split/seed/phase/profile identity and the nominal
// published command.  It must never depend on IndependentPlantState liquid
// fields or on the eventual recovery label.
struct RecoveryExcitationProfile {
    std::string profile_id;
    int pulse_steps = 0;
    double residual_v = 0.0;
    double residual_omega = 0.0;
};

struct RecoveryRolloutLabelContract {
    double maximum_path_position_error_m = 0.0;
    double maximum_path_yaw_error_rad = 0.0;
    double maximum_external_height_m = 0.0;
    double terminal_position_error_m = 0.0;
    double terminal_yaw_error_rad = 0.0;
    double terminal_v_abs_mps = 0.0;
    double terminal_omega_abs_radps = 0.0;
    double terminal_external_height_m = 0.0;
    double fixed_tail_sec = 0.0;
};

struct RecoveryRolloutSamplingConfig {
    std::string schema;
    bool simulation_only = false;
    bool external_liquid_truth_visible_to_candidate_policy = true;
    bool external_liquid_truth_used_for_features = true;
    bool external_liquid_truth_used_for_label = false;
    bool controller_liquid_observer_uses_motion_only = false;
    int warmup_steps = 0;
    double published_linear_min = 0.0;
    double published_linear_max = 0.0;
    double published_angular_min = 0.0;
    double published_angular_max = 0.0;
    double maximum_candidate_residual_v = 0.0;
    double maximum_candidate_residual_omega = 0.0;
    double maximum_published_acceleration = 0.0;
    double maximum_published_angular_acceleration = 0.0;
    BoundedTrackingRecoveryPolicyParams recovery_policy;
    RecoveryRolloutLabelContract label;
    std::vector<RecoveryExcitationProfile> profiles;
};

// Exact row consumed by fit_phase_rejoin_recovery.py.  state_errors follow
// [x,y,yaw,v,omega,eta_x,eta_x_dot,eta_y,eta_y_dot].  execution_errors follow
// [linear_output,angular_output,5 linear pending,7 angular pending].
struct RecoveryDatasetRow {
    std::string split;
    std::string rollout_id;
    std::uint32_t seed = 0;
    std::size_t phase_index = 0;
    bool recovered = false;
    std::array<double, 9> state_errors{{}};
    std::array<double, 14> execution_errors{{}};
};

// Extra evidence is deliberately kept outside the fitter CSV.  It makes the
// physical label auditable without allowing any of these truth quantities to
// become an online feature.
struct RecoveryRolloutAudit {
    std::string split;
    std::string rollout_id;
    std::string profile_id;
    std::uint32_t seed = 0;
    std::size_t phase_index = 0;
    bool recovered = false;
    bool path_position_passed = false;
    bool path_yaw_passed = false;
    bool external_height_passed = false;
    bool terminal_position_passed = false;
    bool terminal_yaw_passed = false;
    bool terminal_velocity_passed = false;
    bool terminal_external_height_passed = false;
    double snapshot_time_sec = 0.0;
    double final_time_sec = 0.0;
    double maximum_path_position_error_m = 0.0;
    double maximum_path_yaw_error_rad = 0.0;
    double maximum_external_height_m = 0.0;
    double terminal_position_error_m = 0.0;
    double terminal_yaw_error_rad = 0.0;
    double terminal_v_abs_mps = 0.0;
    double terminal_omega_abs_radps = 0.0;
    double terminal_external_height_m = 0.0;
};

struct RecoverySeedSampleResult {
    bool valid = false;
    std::string status = "NOT_RUN";
    std::vector<RecoveryDatasetRow> rows;
    std::vector<RecoveryRolloutAudit> audits;
};

bool loadRecoveryRolloutSamplingConfig(
    const std::string& path,
    RecoveryRolloutSamplingConfig& config,
    std::string& error);

bool validateRecoveryRolloutSamplingConfig(
    const RecoveryRolloutSamplingConfig& config,
    std::string& error);

// Simulation-only generator for empirical recovery data.  The nominal input
// is a gate-free projection reconstructed directly from the OfflineSloshOCP
// published-command plan with the compiled 22D execution transition.  No
// provisional recovery radii/bounds are accepted as input, avoiding a
// circular "gate generates its own labels" pipeline.
class PhaseRejoinRecoveryRolloutSampler {
public:
    bool configure(
        const IndependentPlantConfig& plant_config,
        const RecoveryRolloutSamplingConfig& sampling_config,
        const SloshModelParams& controller_slosh_params,
        double nominal_dt_sec,
        const std::vector<PhaseNominalSample>& nominal_samples,
        std::string& error);

    RecoverySeedSampleResult sampleSeed(
        const std::string& split,
        std::uint32_t seed,
        std::size_t phase_begin,
        std::size_t phase_end_inclusive) const;

private:
    IndependentPlantConfig plant_config_;
    RecoveryRolloutSamplingConfig sampling_config_;
    SloshModelParams controller_slosh_params_;
    double nominal_dt_sec_ = 0.0;
    std::vector<PhaseNominalSample> nominal_samples_;
    bool configured_ = false;
};

}  // namespace simulation
}  // namespace spmpc_local_planner
