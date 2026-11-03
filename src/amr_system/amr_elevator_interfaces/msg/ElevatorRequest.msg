# What the AMR asks of the elevator. Nothing presses buttons on its own: in demos a person
# presses what the robot announces (the kiosk shows it), in simulation the simulated elevator
# acts on it, and later a SPS-B-KOLA-0001-7700 elevator-server bridge can forward it
# (KIND_CALL = 로봇출발층 + 목적층호출, KIND_CANCEL = 목적층호출취소). No acknowledgement exists.
uint8 KIND_CALL=1             # hall call at start_floor towards target_floor
uint8 KIND_CAR_FLOOR=2        # inside the car: target_floor
uint8 KIND_CANCEL=3           # the mission ended or gave up; withdraw any request

uint8 DIR_NONE=0
uint8 DIR_UP=1
uint8 DIR_DOWN=2

std_msgs/Header header
uint8 kind
string start_floor            # floors.yaml name where the robot waits (KIND_CALL), else ""
string target_floor           # floors.yaml name (KIND_CALL, KIND_CAR_FLOOR), else ""
uint8 direction               # KIND_CALL: DIR_UP/DIR_DOWN, else DIR_NONE
uint32 sequence               # increments per published request (retries get a new number)
