# AMR -> elevator RF device: physically press a button.
uint8 CMD_HALL_CALL=1         # hall device on floor_code presses the direction button
uint8 CMD_CAR_FLOOR=2         # car device presses the floor_code button
uint8 CMD_DOOR_OPEN=3         # car device presses the door-open button (planned hardware)

uint8 DIR_NONE=0
uint8 DIR_UP=1
uint8 DIR_DOWN=2

std_msgs/Header header
uint8 command
int8 floor_code
uint8 direction
uint8 sequence                # chosen by the sender; echoed by EVENT_BUTTON_ACK, wraps at 256
