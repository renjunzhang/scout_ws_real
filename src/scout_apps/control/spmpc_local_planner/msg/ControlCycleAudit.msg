# One joinable record for solver input, first action, command interventions,
# and the latest timestamped realized odom/IMU excitation available at solve.
# Realized samples generally precede this cycle's new command; consumers must
# align them by the explicit effective timestamps, never by bag arrival order.

std_msgs/Header header
uint32 schema_version
uint64 cycle_id
string variant
string status
string solver_status

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_ODOM=1
uint8 SOURCE_PROCESSED_IMU=2
uint8 observer_source

bool solve_attempted
bool solve_success
bool command_accepted
bool publish_cmd_vel
bool command_was_published
bool publication_receipt_consistent
bool command_history_committed
bool phase_rejoin_committed
bool command_contract_violation
bool terminal_phase
bool terminal_controller_intervened
bool safety_gate_intervened
bool linear_limited
bool angular_rate_limited
bool angular_accel_limited

time cycle_start_stamp
time raw_robot_state_stamp
time raw_liquid_state_stamp
time robot_state_stamp
time liquid_state_stamp
time solver_input_epoch
time solve_start_stamp
time solve_end_stamp
time horizon_available_stamp
time command_publish_stamp
float64 raw_state_skew_sec
float64 aligned_state_skew_sec
bool state_alignment_required
bool state_time_aligned
bool robot_state_interpolated
bool robot_state_extrapolated
string state_alignment_status

float64 solver_u0_a
float64 solver_u0_alpha
float64 planned_ax
float64 planned_ay
float64 solver_cmd_v
float64 solver_cmd_omega
float64 terminal_cmd_v
float64 terminal_cmd_omega
float64 post_gate_cmd_v
float64 post_gate_cmd_omega
float64 finalized_cmd_v
float64 finalized_cmd_omega
float64 published_cmd_v
float64 published_cmd_omega

# Replanning adherence: previous horizon control k=1 should become the current
# first action after one 30 Hz shift.  The deltas quantify plan replacement.
bool previous_shifted_plan_available
uint64 previous_plan_cycle_id
float64 previous_shifted_plan_a
float64 previous_shifted_plan_alpha
float64 replanned_minus_shifted_a
float64 replanned_minus_shifted_alpha

bool odom_excitation_valid
time odom_measurement_stamp
time odom_accel_effective_stamp
time odom_receive_stamp
float64 odom_ax
float64 odom_ay
float64 odom_omega
float64 odom_alpha
float64 odom_sample_dt_sec

bool imu_excitation_valid
time imu_measurement_stamp
time imu_accel_effective_stamp
time imu_receive_stamp
float64 imu_ax
float64 imu_ay
float64 imu_omega
float64 imu_alpha
float64 imu_sample_dt_sec
