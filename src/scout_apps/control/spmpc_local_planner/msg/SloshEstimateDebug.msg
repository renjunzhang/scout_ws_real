uint8 METHOD_O0=0
uint8 METHOD_I0=1
uint8 METHOD_I1=2
uint8 METHOD_L22=3
uint8 method
string method_name

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_ODOM=1
uint8 SOURCE_PROCESSED_IMU=2
uint8 SOURCE_COMMAND_HISTORY=3
uint8 source
string source_name

uint8 STATUS_NOT_AVAILABLE=0
uint8 STATUS_RAW_OBSERVER=1
uint8 STATUS_NOWCAST_PASS_THROUGH=2
uint8 STATUS_NOWCAST_PREDICTED=3
uint8 STATUS_LEGACY_ROLLOUT=4
uint8 STATUS_ERROR=255
uint8 status_code
int32 detail_status_code
string status

bool configured
bool valid
bool applied_to_solver

time input_state_stamp
time output_state_stamp
time excitation_effective_stamp
float64 input_state_age_sec
float64 output_state_age_sec
float64 propagation_sec
float64 excitation_age_sec

float64 eta_x
float64 eta_x_dot
float64 eta_y
float64 eta_y_dot
float64 modal_height_m
float64 total_height_m
