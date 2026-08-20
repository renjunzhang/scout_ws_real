#pragma once

#include "spmpc_local_planner/core/slosh_risk_governor.h"
#include "spmpc_local_planner/core/spmpc_problem.h"
#include "spmpc_local_planner/estimation/processed_imu_pipeline.h"
#include "spmpc_local_planner/estimation/slosh_observer_bank.h"
#include "spmpc_local_planner/estimation/slosh_observer_selector.h"
#include "spmpc_local_planner/phase_rejoin/phase_rejoin_coordinator.h"
#include "spmpc_local_planner/reference/reference_path_preprocessor.h"
#include "spmpc_local_planner/ros/command_history_buffer.h"
#include "spmpc_local_planner/ros/control_cycle_contract.h"
#include "spmpc_local_planner/ros/diagnostics_publisher.h"
#include "spmpc_local_planner/ros/execution_state_predictor.h"
#include "spmpc_local_planner/ros/imu_shadow_ros_adapter.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace spmpc_local_planner {

struct MapVRefProfileSample {
    double s_m = 0.0;
    double v_ref_mps = 0.0;
};

class SpmpcLocalPlannerROS {
public:
    SpmpcLocalPlannerROS();
    ~SpmpcLocalPlannerROS();
    bool initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    void spin();

private:
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);
    void pathCallback(const nav_msgs::PathConstPtr& msg);
    void costmapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
    void controlTimerCallback(const ros::TimerEvent&);
    void publishZeroCommand(const CommandInterventionDebug& intervention = CommandInterventionDebug(),
                            ControlCycleAuditDebug* audit = nullptr);
    void publishCommand(const geometry_msgs::Twist& desired,
                        const CommandInterventionDebug& intervention = CommandInterventionDebug(),
                        ControlCycleAuditDebug* audit = nullptr);
    void recordPublishedCommand(const geometry_msgs::Twist& cmd, const ros::Time& stamp, const CommandPublishMeta& meta);
    bool delayPhaseActive() const;
    bool delayPhasePredictionEnabled() const;
    bool delayPhaseClosedLoopEnabled() const;
    void publishDelayPhaseDiagnostics(const ros::Time& now,
                                      DelayPhaseStatusCode status_code,
                                      const ExecutionStatePrediction* prediction,
                                      double solver_time_ms,
                                      bool closed_loop_enabled = false);
    void publishDelayPhaseEarlyStatus(DelayPhaseStatusCode status_code);
    geometry_msgs::Twist applySharedCommandLimits(const geometry_msgs::Twist& desired,
                                                  const ros::Time& stamp,
                                                  geometry_msgs::Twist& previous,
                                                  double& dt,
                                                  bool& linear_limited,
                                                  bool& angular_rate_limited,
                                                  bool& angular_accel_limited);
    bool updateTerminalSpinFailGate(const SolverInput& input, const SolverOutput& output, double period_sec);
    void resetTerminalSpinFailGate();
    bool updateTrackingSafetyGate(const SolverInput& input,
                                  const SolverOutput& output,
                                  double period_sec,
                                  std::string& failure_status);
    void resetTrackingSafetyGate();
    RobotState robotStateFromOdom(const nav_msgs::Odometry& odom) const;
    bool robotStateFromLatest(RobotState& state);
    bool robotStateAtEpoch(const ros::Time& target_stamp,
                           RobotState& state,
                           bool& interpolated,
                           bool& extrapolated,
                           std::string& status);
    void appendOdomStateHistory(const nav_msgs::Odometry& odom);
    bool processOdomInput(const nav_msgs::Odometry& odom,
                          const ros::Time& receive_stamp);
    void publishOdomSloshObserverDebug(const nav_msgs::Odometry& odom,
                                       const MotionExcitation& excitation,
                                       const std::string& status);
    void publishImuSloshObserverDebug(const sensor_msgs::Imu& imu,
                                      const ProcessedImuOutput& output);
    void publishSloshObserverSelectionDebug(
        const ros::Time& now,
        const SloshObserverSelection& selection,
        bool solver_consumes_selected_state,
        const ControlCycleTimingDebug& cycle_timing);
    bool updateReferenceSignature(const nav_msgs::Path& path);
    ReferencePath referencePathFromMsg(const nav_msgs::Path& path) const;
    CostmapGrid costmapFromMsg(const nav_msgs::OccupancyGrid& map) const;
    bool loadMapVRefProfile(const std::string& path);
    bool ensureMapVRefProfileLoaded(const std::string& path);
    bool lookupMapVRef(double s_m, double& v_ref_mps) const;
    void applyRuntimeVRef(SolverInput& input);
    void applySloshRiskGovernor(SolverInput& input);
    void resetMapVRefProgress();
    void loadVariantOverrides(const std::string& variant_name);
    SloshModelParams loadSloshParams() const;
    ProcessedImuParams loadProcessedImuParams() const;
    SloshRiskGovernorParams loadSloshRiskGovernorParams() const;
    void validatePhaseRejoinReference(const ReferencePath& reference);
    bool phaseRejoinNeedsPrediction() const;

    // The processed-IMU shadow runs on a private ROS1 callback queue so its
    // filtering/matrix exponential/diagnostic publication cannot queue ahead
    // of the formal odom/path/control callbacks on the global queue.
    ros::CallbackQueue imu_callback_queue_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::NodeHandle imu_nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber costmap_sub_;
    ros::Publisher cmd_pub_;
    ros::Timer control_timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    SpmpcProblem problem_;
    DiagnosticsPublisher diagnostics_;
    VariantConfig variant_;
    ReferencePathPreprocessor reference_preprocessor_;
    ReferencePathPreprocessParams reference_preprocess_params_;
    SloshObserverBank slosh_observers_;
    SloshObserverSelector slosh_observer_selector_;
    SloshObserverSelectorParams slosh_observer_selector_params_;
    ImuShadowRosAdapter imu_shadow_adapter_;
    SloshRiskGovernor slosh_risk_governor_;
    SloshRiskGovernorParams slosh_risk_governor_params_;
    SloshRiskGovernorOutput last_slosh_governor_output_;
    CommandHistoryBuffer command_history_;
    ExecutionStatePredictor execution_predictor_;
    PhaseRejoinCoordinator phase_rejoin_coordinator_;
    PhaseRejoinParams phase_rejoin_params_;
    DelayPhaseParams delay_phase_params_;
    StateTimingParams state_timing_params_;
    CommandExecutionContractParams command_contract_params_;
    EffectiveConfigDebug effective_config_;
    OdomTimingDebug last_odom_timing_;
    ros::Time last_odom_receive_stamp_;
    std::mutex slosh_observers_mutex_;
    bool imu_input_ready_ = false;
    std::uint32_t imu_input_reset_epoch_ = 0;

    nav_msgs::Odometry last_odom_;
    nav_msgs::Odometry prev_odom_;
    std::deque<StampedRobotState> odom_state_history_;
    bool have_odom_ = false;
    bool have_prev_odom_ = false;
    bool have_reference_signature_ = false;
    std::string map_vref_profile_path_;
    std::vector<MapVRefProfileSample> map_vref_profile_;
    bool map_vref_profile_loaded_ = false;
    double map_vref_last_progress_abs_s_ = 0.0;
    bool have_map_vref_progress_ = false;
    std::string reference_signature_frame_;
    std::size_t reference_signature_size_ = 0;
    double reference_signature_start_x_ = 0.0;
    double reference_signature_start_y_ = 0.0;
    double reference_signature_end_x_ = 0.0;
    double reference_signature_end_y_ = 0.0;

    std::string odom_topic_ = "/odom";
    std::string imu_topic_ = "/imu/data";
    std::string path_topic_ = "/scout/global_path_fixed";
    std::string costmap_topic_ = "/map";
    std::string cmd_topic_ = "/cmd_vel";
    std::string robot_base_frame_ = "base_link";
    std::string imu_expected_frame_ = "imu_link";
    std::string reference_target_frame_;
    std::string experiment_mode_ = "fixed_path";
    std::string phase_rejoin_artifact_path_;
    bool phase_rejoin_publish_diagnostics_ = true;
    bool publish_cmd_vel_ = true;
    bool imu_shadow_enable_ = false;
    bool imu_shadow_publish_diagnostics_ = true;
    int imu_subscriber_queue_size_ = 10;
    double imu_observer_dt_sec_ = 0.02;
    bool use_tf_pose_ = true;
    bool obstacle_enable_ = false;
    bool shared_cmd_linear_accel_limit_enable_ = true;
    double shared_cmd_linear_accel_max_ = 0.6;
    double shared_cmd_linear_accel_max_dt_ = 0.2;
    bool shared_cmd_angular_limit_enable_ = false;
    double shared_cmd_angular_rate_max_ = 1.2;
    double shared_cmd_angular_accel_max_ = 1.2;
    double shared_cmd_angular_accel_max_dt_ = 0.2;
    bool terminal_spin_fail_enable_ = true;
    double terminal_spin_fail_omega_threshold_ = 0.20;
    double terminal_spin_fail_max_duration_sec_ = 2.0;
    double terminal_spin_fail_duration_sec_ = 0.0;
    bool terminal_spin_fail_latched_ = false;
    bool tracking_safety_enable_ = true;
    bool tracking_safety_projection_enable_ = true;
    double tracking_safety_max_projection_distance_m_ = 0.50;
    double tracking_safety_max_projection_duration_sec_ = 0.20;
    double tracking_safety_projection_duration_sec_ = 0.0;
    bool tracking_safety_projection_latched_ = false;
    bool tracking_safety_spin_enable_ = true;
    double tracking_safety_spin_omega_threshold_ = 0.50;
    double tracking_safety_spin_max_duration_sec_ = 2.0;
    double tracking_safety_spin_duration_sec_ = 0.0;
    bool tracking_safety_spin_latched_ = false;
    geometry_msgs::Twist last_published_cmd_;
    ros::Time last_cmd_stamp_;
    bool have_last_published_cmd_ = false;
    double tf_timeout_sec_ = 0.05;
    double control_frequency_ = 30.0;
    double dt_ = 1.0 / 30.0;
    int horizon_steps_ = 60;
    std::uint64_t next_cycle_id_ = 0;
    bool have_previous_shifted_plan_ = false;
    std::uint64_t previous_plan_cycle_id_ = 0;
    double previous_shifted_plan_a_ = 0.0;
    double previous_shifted_plan_alpha_ = 0.0;
    // Declared last so the worker stops before any callback-owned state is
    // destroyed.  The explicit destructor also stops it before member teardown.
    std::unique_ptr<ros::AsyncSpinner> imu_spinner_;
};

}  // namespace spmpc_local_planner
