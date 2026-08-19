# Complete solver horizon for offline trajectory, speed, excitation, and replay analysis.
# State arrays contain N+1 samples. Control arrays contain N samples.

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

float64[] a
float64[] alpha_or_omega
float64[] v_s
