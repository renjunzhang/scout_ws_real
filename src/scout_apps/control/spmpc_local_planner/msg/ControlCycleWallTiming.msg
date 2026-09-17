# Join to ControlCycleAudit by cycle_id. Kept separate to preserve the ROS1
# wire format of the recorded command history and existing replay bags.
std_msgs/Header header
uint32 schema_version
uint64 cycle_id
string status
bool solve_attempted
bool solve_success
bool command_accepted
string state_alignment_status

# Steady-clock durations, independent of ROS / simulated time.
# -1 means that a cycle stage was not reached. dispatch_wall_ms ends immediately
# before command transport (or its dry-run equivalent), excluding diagnostics.
float64 state_alignment_wall_ms
# Zero: exact TF / odom pose. Positive: bounded odom propagation from an older TF.
float64 pose_propagation_sec
float64 pre_solve_wall_ms
float64 problem_solve_wall_ms
float64 dispatch_wall_ms
# Solver fields are populated only after setup reaches the RTI budget check.
# rti_wall_ms includes residual_wall_ms; remaining budget may be negative.
bool solver_wall_timing_valid
float64 solver_setup_wall_ms
float64 rti_wall_ms
float64 residual_wall_ms
float64 iteration_estimate_wall_ms
float64 remaining_budget_wall_ms
uint32 rti_iterations
