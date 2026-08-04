# Replay-oriented snapshot captured after OCP parameters/bounds/warm-start are set
# and immediately before acados solve(). State rows are flattened as:
# [x,y,yaw,v,s,omega,eta_x,eta_x_dot,eta_y,eta_y_dot].
# Control rows are flattened as [a,alpha_or_omega,v_s].
# stage_parameters contains (N+1) rows with parameter_width entries per row.

std_msgs/Header header
uint32 schema_version
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

float64 robot_x
float64 robot_y
float64 robot_yaw
float64 robot_v
float64 robot_omega
float64 eta_x
float64 eta_x_dot
float64 eta_y
float64 eta_y_dot

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
