#pragma once

#include "spmpc_local_planner/core/slosh_risk_governor.h"
#include "spmpc_local_planner/config/variant_config.h"
#include "spmpc_local_planner/estimation/processed_imu_pipeline.h"
#include "spmpc_local_planner/estimation/slosh_observer_selector.h"
#include "spmpc_local_planner/phase_rejoin/types.h"
#include "spmpc_local_planner/reference/reference_path_preprocessor.h"
#include "spmpc_local_planner/runtime/execution_prediction/types.h"
#include "spmpc_local_planner/safety/safety_supervisor.h"
#include "spmpc_local_planner/solver/api/solver_config.h"

#include <string>
#include <vector>

namespace spmpc_local_planner {

enum class ValidationSeverity {
    Warning,
    Fatal,
};

struct ValidationIssue {
    ValidationSeverity severity = ValidationSeverity::Warning;
    std::string key;
    std::string message;
};

class ValidationReport {
public:
    void warning(const std::string& key, const std::string& message);
    void fatal(const std::string& key, const std::string& message);

    bool ok() const;
    const std::vector<ValidationIssue>& issues() const { return issues_; }

private:
    std::vector<ValidationIssue> issues_;
};

struct RuntimeVRefConfig {
    bool runtime_override_enable = false;
    double runtime_override_mps = -1.0;
    bool profile_enable = false;
    std::string profile_path;
    double profile_lookahead_m = 0.0;
};

struct RosInterfaceConfig {
    std::string experiment_mode = "fixed_path";
    std::string odom_topic = "/odom";
    std::string imu_topic = "/imu/data";
    std::string reference_path_topic = "/scout/global_path_fixed";
    std::string costmap_topic = "/map";
    std::string cmd_vel_topic = "/cmd_vel";
    std::string robot_base_frame = "base_link";
    std::string reference_target_frame;
    bool use_tf_pose = true;
    double tf_timeout_sec = 0.05;
    bool publish_cmd_vel = true;
};

struct ImuShadowConfig {
    bool enable = false;
    bool publish_diagnostics = true;
    std::string expected_frame = "imu_link";
    int subscriber_queue_size = 10;
    double observer_dt_sec = 0.02;
    ProcessedImuParams processed;
};

struct ControlRuntimeConfig {
    double frequency_hz = 30.0;
    double dt = 1.0 / 30.0;
    int horizon_steps = 60;
    DelayPhaseParams delay_phase;
    StateTimingParams state_timing;
    CommandExecutionContractParams execution_contract;
};

struct PhaseRejoinConfig {
    PhaseRejoinParams params;
    bool publish_diagnostics = true;
    std::string artifact_path;
};

struct SharedCommandLimitsConfig {
    bool linear_accel_limit_enable = true;
    double linear_accel_max = 0.6;
    double linear_accel_max_dt = 0.2;
    bool angular_limit_enable = false;
    double angular_rate_max = 1.2;
    double angular_accel_max = 1.2;
    double angular_accel_max_dt = 0.2;
};

struct ConfigCompatibility {
    bool variant_weight_table_present = false;
    bool legacy_container_height_used = false;
};

// Complete typed root configuration.  ROS parameter keys are mapped once by
// RosConfigLoader; initialization and control-cycle code consume this value
// without returning to the parameter server.
struct AppConfig {
    std::string requested_variant = "B0";
    RosInterfaceConfig ros_interface;
    ImuShadowConfig imu_shadow;
    SloshObserverSelectorParams slosh_observer;
    ControlRuntimeConfig control;
    PhaseRejoinConfig phase_rejoin;
    ReferencePathPreprocessParams reference_preprocess;
    SolverParams solver;
    VariantConfig variant;
    SharedCommandLimitsConfig shared_command_limits;
    SafetySupervisorConfig safety;
    SloshRiskGovernorParams slosh_risk_governor;
    RuntimeVRefConfig map_vref;
    ConfigCompatibility compatibility;
};

ValidationReport validateAndNormalize(AppConfig& config);

}  // namespace spmpc_local_planner
