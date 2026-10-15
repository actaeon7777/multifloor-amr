# Compact, read-only vehicle state. No commands, maps, scan rays or UI goal handles.
# header.stamp: ROS time; published_wall_stamp: system UTC for cross-host expiry.
# Detect restarts with publisher_session, then require increasing sequence.
# Consumers must also enforce a local monotonic reception timeout (default 0.5 s).
uint8 SCHEMA_VERSION=1
std_msgs/Header header
builtin_interfaces/Time published_wall_stamp
string publisher_session
uint64 sequence
uint32 epoch

amr_vehicle_interfaces/MotorStatus left_motor
amr_vehicle_interfaces/MotorStatus right_motor
amr_vehicle_interfaces/DataStatus scan
amr_vehicle_interfaces/DataStatus wheel
amr_vehicle_interfaces/DataStatus imu
amr_vehicle_interfaces/DataStatus ekf
amr_vehicle_interfaces/DataStatus joints
amr_vehicle_interfaces/DataStatus command
amr_vehicle_interfaces/DataStatus diagnostics
geometry_msgs/Twist command_velocity
geometry_msgs/Twist wheel_velocity
geometry_msgs/Twist ekf_velocity

# Odom path distance since continuity epoch. A gap/reset advances the epoch.
float64 wheel_distance_m
uint64 wheel_distance_epoch
bool wheel_distance_valid

amr_vehicle_interfaces/DataStatus pose_status
geometry_msgs/Pose map_pose
# These source topics are retained change events, not producer heartbeats.
bool control_mode_received
string control_mode
bool navigation_state_received
string navigation_state

amr_vehicle_interfaces/DataStatus localization_status
string localization_state
bool localization_ready
string localization_detail
string tracking_warning
amr_vehicle_interfaces/DataStatus time_status
bool time_healthy
bool time_fault_latched
string time_detail

# Counter values are valid only with a fresh diagnostics sample.
uint8 diagnostic_level
string diagnostic_detail
uint64 received_frames
uint64 invalid_frames
uint64 stale_frames
uint64 wheel_rebases
uint64 clock_resets

# Battery voltage is the mean of fresh LEFT/RIGHT driver input voltages.
# SOC/current/temperature/charge-state remain unavailable without a BMS source.
amr_vehicle_interfaces/DataStatus battery_status
bool battery_soc_available
float64 battery_soc_percent
bool battery_voltage_available
float64 battery_voltage_v
bool battery_current_available
float64 battery_current_a
bool battery_temperature_available
float64 battery_temperature_c
uint8 CHARGE_UNKNOWN=0
uint8 CHARGE_DISCHARGING=1
uint8 CHARGE_CHARGING=2
uint8 CHARGE_FULL=3
uint8 battery_charge_state
