# Complete solver horizon for offline trajectory, speed, excitation, and replay analysis.
# State arrays contain N+1 samples. Control arrays contain N samples.

std_msgs/Header header
uint32 schema_version
bool valid
string backend
string variant
string solver_status
bool slosh_enabled
string control_semantics
float64 dt
uint32 horizon_steps

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
