#pragma once

#include "spmpc_local_planner/core/slosh_risk_governor.h"
#include "spmpc_local_planner/core/types.h"
#include "spmpc_local_planner/core/variant_config.h"
#include "spmpc_local_planner/ros/delay_phase_types.h"
#include "spmpc_local_planner/ControlCycleAudit.h"
#include "spmpc_local_planner/PreSolveSnapshot.h"
#include "spmpc_local_planner/PredictedHorizon.h"
#include "spmpc_local_planner/SloshEstimatorComparison.h"
#include "spmpc_local_planner/SloshObserverDebug.h"
#include "spmpc_local_planner/SloshObserverSelectionDebug.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <cstdint>

namespace spmpc_local_planner {

class DiagnosticsPublisher {
public:
    void initialize(ros::NodeHandle& nh);
    void publishVariant(const VariantConfig& variant, const std::string& experiment_mode);
    void publishSolverBackend(const std::string& solver_backend);
    void publishEffectiveConfig(const EffectiveConfigDebug& config);
    void publishOutput(const SolverOutput& output, const std::string& frame_id);
    void publishRawState(const RobotState& robot, const SloshState& slosh, double height_coeff);
    void publishPredictedState(const ExecutionStatePrediction& prediction, double height_coeff);
    void publishSolverInputState(const SolverInput& input,
                                 std::uint8_t source_code,
                                 bool robot_delay_compensation_applied,
                                 bool liquid_delay_compensation_applied,
                                 double height_coeff);
    void publishCommandIntervention(const CommandInterventionDebug& intervention);
    void publishControlCycleAudit(const ControlCycleAuditDebug& audit,
                                  const std::string& frame_id);
    void publishCommandOutput(const geometry_msgs::Twist& desired,
                              const geometry_msgs::Twist& limited,
                              const geometry_msgs::Twist& previous,
                              double dt,
                              bool linear_limited,
                              bool angular_rate_limited,
                              bool angular_accel_limited);
    void publishSloshState(const SloshState& state);
    void publishSloshHeight(double height_m);
    void publishOdomSloshObserver(const SloshObserverDebug& msg);
    void publishImuSloshObserver(const SloshObserverDebug& msg);
    void publishSloshObserverSelection(const SloshObserverSelectionDebug& msg);
    void publishSloshEstimatorComparison(const SloshEstimatorComparison& msg);
    void publishSloshGovernor(const SloshRiskGovernorOutput& output);
    void publishDelayPhase(const DelayPhaseDebugSummary& summary);
    void publishOdomTiming(const OdomTimingDebug& timing);
    void publishExecutionState(const ExecutionStatePrediction& prediction);
    void publishExecutionAlignmentStatus(const std::string& status);
    void publishDelayCompensation(const DelayPhaseDebugSummary& summary);
    void publishCmdOdomAlignment(const CmdOdomAlignmentDebug& alignment);
    void publishStatus(const std::string& status);

private:
    nav_msgs::Path makePathMsg(const SolverOutput& output, const std::string& frame_id) const;
    PredictedHorizon makePredictedHorizonMsg(
        const SolverOutput& output, const std::string& frame_id) const;
    PreSolveSnapshot makePreSolveSnapshotMsg(
        const SolverOutput& output, const std::string& frame_id) const;

    ros::Publisher status_pub_;
    ros::Publisher variant_pub_;
    ros::Publisher experiment_mode_pub_;
    ros::Publisher solver_backend_pub_;
    ros::Publisher effective_config_pub_;
    ros::Publisher trajectory_pub_;
    ros::Publisher predicted_horizon_pub_;
    ros::Publisher pre_solve_snapshot_pub_;
    ros::Publisher progress_pub_;
    ros::Publisher v_ref_current_pub_;
    ros::Publisher map_vref_status_pub_;
    ros::Publisher solver_time_pub_;
    ros::Publisher cost_breakdown_pub_;
    ros::Publisher corridor_pub_;
    ros::Publisher guidance_pub_;
    ros::Publisher primitive_pub_;
    ros::Publisher slosh_state_pub_;
    ros::Publisher slosh_height_pub_;
    ros::Publisher odom_slosh_observer_pub_;
    ros::Publisher imu_slosh_observer_pub_;
    ros::Publisher slosh_observer_selection_pub_;
    ros::Publisher slosh_estimator_comparison_pub_;
    ros::Publisher slosh_horizon_summary_pub_;
    ros::Publisher slosh_hard_constraint_pub_;
    ros::Publisher slosh_hard_constraint_effective_pub_;
    ros::Publisher slosh_cost_monitor_pub_;
    ros::Publisher slosh_governor_pub_;
    ros::Publisher slosh_governor_status_pub_;
    ros::Publisher warm_start_pub_;
    ros::Publisher warm_start_status_pub_;
    ros::Publisher runtime_bounds_pub_;
    ros::Publisher generated_bounds_pub_;
    ros::Publisher first_shot_pub_;
    ros::Publisher projector_pub_;
    ros::Publisher stage0_reference_pub_;
    ros::Publisher local_traj_head_pub_;
    ros::Publisher warm_start_head_pub_;
    ros::Publisher raw_state_pub_;
    ros::Publisher predicted_state_pub_;
    ros::Publisher solver_input_state_pub_;
    ros::Publisher command_intervention_pub_;
    ros::Publisher control_cycle_audit_pub_;
    ros::Publisher cmd_output_pub_;
    ros::Publisher cmd_output_status_pub_;
    ros::Publisher delay_phase_pub_;
    ros::Publisher odom_timing_pub_;
    ros::Publisher execution_state_pub_;
    ros::Publisher execution_alignment_status_pub_;
    ros::Publisher delay_compensation_pub_;
    ros::Publisher cmd_odom_alignment_pub_;
    ros::Publisher terminal_pub_;
    ros::Publisher terminal_mode_pub_;
    ros::Publisher start_lock_active_pub_;
    ros::Publisher start_lock_mode_pub_;
    ros::Publisher start_lock_debug_pub_;
};

}  // namespace spmpc_local_planner
