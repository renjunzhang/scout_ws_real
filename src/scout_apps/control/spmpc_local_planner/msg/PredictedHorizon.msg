# Complete solver horizon for offline trajectory, speed, excitation, and replay analysis.
# State arrays contain N+1 samples. Control arrays contain N samples.

std_msgs/Header header
uint32 schema_version
# Schema 9 adds actuator-output jerk bound; schema 8: planning identity and per-stage reference audit; 1 identifies the shared rotating-container RK4 model.
uint32 cost_model_version
uint32 liquid_model_version
uint64 cycle_id
time cycle_start_stamp
time raw_robot_state_stamp
time raw_liquid_state_stamp
time robot_state_stamp
time liquid_state_stamp
time solver_input_epoch
time solve_start_stamp
time solve_end_stamp
time horizon_available_stamp
float64 raw_state_skew_sec
float64 aligned_state_skew_sec
bool state_alignment_required
bool state_time_aligned
bool robot_state_interpolated
bool robot_state_extrapolated
string state_alignment_status
bool valid
string backend
string variant
string solver_status
bool slosh_enabled
string control_semantics
float64 dt
uint32 horizon_steps
int32 slosh_cost_horizon_steps
float64 slosh_cost_horizon_sec
float64 slosh_cost_tail_discount

float64[] t
float64[] x
float64[] y
float64[] yaw
float64[] v
float64[] omega
float64[] s
float64[] eta_x
float64[] eta_x_dot
float64[] eta_y
float64[] eta_y_dot
float64[] h_modal
float64[] v_cmd
float64[] omega_cmd
float64[] a_cmd_memory
float64[] delayed_v_cmd
float64[] delayed_omega_cmd
float64[] a_actual
float64[] alpha_actual

float64[] a
float64[] alpha_or_omega
float64[] v_s

# Schema 5: do not infer an ablation condition from the variant name alone.
bool zero_liquid_initial_state
bool jerk_limit_enable
float64 jerk_max
# Actual iterations executed; configuration gives only the upper bound.
int32 rti_iterations
float64 dynamics_max_defect
float64 delta_a_max

# Schema 9 adds actuator-output jerk bound; schema 8: planning identity and per-stage reference audit.
string experiment_profile_id
string plan_id
string region_id
string reference_mode
float64 task_elapsed_sec
float64 deadline_sec
string reference_status
bool geometry_enabled
string[] stage_region_ids
float64[] nominal_times
string[] nominal_phases
float64 minimum_region_clearance
# Command lower bound (not the minimum observed command).
float64 v_cmd_min
# Exact row-major generated states, including every FIFO entry. Width is 24/28
# for explicit B0/slosh; 0 means this backend/early failure supplied no rows.
uint32 model_state_width
float64[] model_states

# Actuator-output longitudinal jerk on the control grid; zero disables.
float64 actual_jerk_max
