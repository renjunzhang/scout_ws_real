# A compact, auditable replacement for recording the source RGB stream.
# header is copied from the source sensor_msgs/Image, not stamped at publish time.

uint8 STATUS_OK=0
uint8 STATUS_ZERO_UNLOCKED=1
uint8 STATUS_NO_DETECTION=2
uint8 STATUS_CLIPPED=3
uint8 STATUS_IMAGE_CONVERSION_ERROR=4
uint8 STATUS_DETECTOR_ERROR=5

std_msgs/Header header
uint32 frame_index
uint32 process_every
uint32 image_width
uint32 image_height
string image_encoding
float32 processing_latency_ms

uint8 status_code
string status
bool valid
bool zero_locked
uint32 zero_valid_samples
float32 zero_max_lcr_mm
time zero_window_start_stamp
time zero_window_end_stamp
float32[] zero_samples_max_lcr_raw_mm
float32 height_bias_mm

float32 height_max_lcr_raw_mm
float32 height_max_lcr_mm
float32 height_median_raw_mm
float32 height_median_mm
float32[3] height_lcr_raw_mm
float32[3] height_lcr_mm

float32[3] confidence
bool[3] ruler_valid
bool[3] clipped
uint8 valid_ruler_count
bool any_clipped
float32 confidence_mean
