std_msgs/Header header

uint32 schema_version

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_ODOM=1
uint8 SOURCE_PROCESSED_IMU=2
uint8 source

# For this diagnostic message, header.frame_id and excitation_axes_frame name
# the axes used by ax/ay/omega/alpha.  The point at which the acceleration acts
# is named separately so an ICR proxy is never mistaken for base_link.
string excitation_axes_frame
string excitation_reference_point

bool configured
bool valid
uint8 input_status_code
string input_status
uint32 reset_epoch
uint64 accepted_sample_count
uint64 observer_update_count

time source_stamp
time measurement_stamp
time accel_effective_stamp
time gyro_effective_stamp
time alpha_effective_stamp
time receive_stamp
time state_stamp

float64 transport_age_sec
float64 measurement_age_sec
float64 state_age_sec
float64 sample_dt_sec

float64 ax_mps2
float64 ay_mps2
float64 omega_z_radps
float64 alpha_z_radps2

bool bias_ready
bool filter_ready
uint32 bias_sample_count
float64 bias_x_mps2
float64 bias_y_mps2
float64 bias_z_mps2
float64 accel_filtered_base_x_mps2
float64 accel_filtered_base_y_mps2
float64 quaternion_norm

float64 eta_x
float64 eta_x_dot
float64 eta_y
float64 eta_y_dot
float64 modal_height_m
float64 total_height_m
