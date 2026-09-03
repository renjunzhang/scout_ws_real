# Replay-oriented snapshot captured after OCP parameters/bounds/warm-start are set
# and immediately before acados solve(). Explicit-actuator state rows are:
# B0: [x,y,yaw,v_actual,s,omega_actual,v_cmd,omega_cmd,q_v(5),q_omega(10)]
# slosh: the B0 row followed by [eta_x,eta_x_dot,eta_y,eta_y_dot].
# Control rows are [a_cmd,alpha_cmd,v_s].
# stage_parameters contains (N+1) rows with parameter_width entries per row.

std_msgs/Header header
uint32 schema_version
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
bool primal_guess_only
string control_semantics

float64 dt
uint32 horizon_steps
uint32 state_width
uint32 control_width
uint32 parameter_width
int32 slosh_cost_horizon_steps
float64 slosh_cost_horizon_sec
float64 slosh_cost_tail_discount

float64 robot_x
float64 robot_y
float64 robot_yaw
float64 robot_v
float64 robot_omega
float64 eta_x
float64 eta_x_dot
float64 eta_y
float64 eta_y_dot

bool actuator_state_valid
float64 actuator_v_cmd
float64 actuator_omega_cmd
float64 actuator_delayed_v_cmd
float64 actuator_delayed_omega_cmd
float64 actuator_a_actual
float64 actuator_alpha_actual
float64[] actuator_linear_delay_queue
float64[] actuator_angular_delay_queue

float64 min_progress_s
float64 reference_length
float64 s0
float64 s_end
float64[4] reference_x_coeffs
float64[4] reference_y_coeffs

bool has_v_ref_current
float64 configured_v_ref
float64 requested_v_ref
float64 effective_v_ref
string v_ref_status

bool have_previous_control
float64 previous_a
float64 previous_alpha_or_omega
float64 previous_v_s
bool have_previous_solution
bool warm_start_requested
bool warm_start_applied
string warm_start_source

float64 a_min
float64 a_max
float64 alpha_or_omega_min
float64 alpha_or_omega_max
float64 v_s_min
float64 v_s_max
float64 v_min
float64 v_max
float64 omega_min
float64 omega_max

string[] parameter_names
float64[] stage_parameters
float64[] initial_guess_states
float64[] initial_guess_controls
float64[] previous_solution_states
float64[] previous_solution_controls
