# Elevator door state sensed by the AMR LiDAR (shared by real vehicle and simulation).
uint8 UNKNOWN=0
uint8 CLOSED=1
uint8 OPENING=2
uint8 OPEN=3
uint8 CLOSING=4

std_msgs/Header header
string elevator_id
uint8 state
float32 open_ratio            # share of door-line beams that pass the door plane
uint16 valid_beams
