std_msgs/Header header

uint32 schema_version

uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_ODOM=1
uint8 SOURCE_PROCESSED_IMU=2

uint8 FALLBACK_POLICY_ODOM=1
uint8 FALLBACK_POLICY_FAIL_CLOSED=2

uint8 STATUS_UNCONFIGURED=0
uint8 STATUS_WAITING_FOR_NOMINAL_IMU=1
uint8 STATUS_NOMINAL_ODOM=2
uint8 STATUS_NOMINAL_PROCESSED_IMU=3
uint8 STATUS_FALLBACK_TO_ODOM=4
uint8 STATUS_FAIL_CLOSED=5

uint8 REASON_NONE=0
uint8 REASON_IMU_NOT_READY=1
uint8 REASON_IMU_INVALID=2
uint8 REASON_IMU_STALE=3
uint8 REASON_ODOM_INVALID=4
uint8 REASON_ODOM_STALE=5

bool solver_consumes_selected_state
bool configured
bool valid
bool fallback_active
bool fallback_latched
bool nominal_ready_seen

uint8 nominal_source
string nominal_source_name
uint8 effective_source
string effective_source_name
uint8 fallback_policy
string fallback_policy_name
uint8 status_code
string status
uint8 reason_code
string reason

bool odom_snapshot_valid
bool imu_snapshot_valid
bool odom_fresh
bool imu_fresh
bool imu_pipeline_ready

time selected_state_stamp
time odom_state_stamp
time imu_state_stamp
float64 odom_state_age_sec
float64 imu_state_age_sec

uint32 imu_reset_epoch
uint64 selection_epoch
