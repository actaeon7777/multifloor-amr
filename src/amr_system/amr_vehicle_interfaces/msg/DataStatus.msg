# Source sample validity. Values must only be used when received && valid && fresh.
# source_stamp is ROS time when stamp_valid; age_sec includes ROS and steady age.
bool received
bool valid
bool fresh
bool stamp_valid
builtin_interfaces/Time source_stamp
float64 age_sec
float64 timeout_sec
bool rate_valid
float64 rate_hz
