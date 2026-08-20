# Typed, joinable audit record for the phase-rejoining coordinator.  This is
# an empirical/development gate diagnostic unless state_complete_for_certificate
# is explicitly true; the current ROS wiring deliberately keeps it false.

std_msgs/Header header
uint32 schema_version
uint64 cycle_id

uint8 MODE_OFF=0
uint8 MODE_MONITOR=1
uint8 MODE_ENFORCE=2
uint8 mode
string mode_name

uint8 EVIDENCE_UNKNOWN=0
uint8 EVIDENCE_DEVELOPMENT_ONLY=1
uint8 EVIDENCE_EMPIRICAL_HELD_OUT=2
uint8 evidence_level
string evidence_level_name

bool artifact_loaded
bool contract_valid
bool ready
bool empirical_gate
bool state_complete_for_certificate
uint64 artifact_size
string artifact_path
string contract_id

uint64 current_index
uint64 front_index
uint64 terminal_index
uint64 candidate_count
int32 front_steps
int32 liquid_steps
int32 solver_terminal_step
bool solver_origin_at_execution_front
float64 candidate_score

float64 terminal_gate_metric
float64 current_gate_metric
bool terminal_gate_accepted
bool current_gate_accepted

bool command_intervened
bool recovery_command_used
bool controlled_stop_used
float64 nominal_cmd_v
float64 nominal_cmd_omega
float64 solver_cmd_v
float64 solver_cmd_omega
float64 output_cmd_v
float64 output_cmd_omega
float64 residual_v
float64 residual_omega

bool prediction_valid
time prediction_origin_epoch
time prediction_epoch
float64 prediction_integrated_duration_sec
string prediction_status
time solver_input_epoch
string status
