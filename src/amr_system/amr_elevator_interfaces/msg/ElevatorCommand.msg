# AMR -> elevator RF device: physically press a button.
uint8 CMD_HALL_CALL=1         # hall device on floor_code presses the direction button
uint8 CMD_CAR_FLOOR=2         # car device presses the floor_code button
uint8 CMD_DOOR_OPEN=3         # car device presses the door-open button

uint8 DIR_NONE=0
uint8 DIR_UP=1
uint8 DIR_DOWN=2

uint8 DOOR_OPEN_REPEAT_MAX=15

std_msgs/Header header
uint8 command
int8 floor_code
uint8 direction
uint8 sequence                # chosen by the sender; echoed by EVENT_BUTTON_ACK, wraps at 256
uint8 repeat                  # CMD_DOOR_OPEN only: 0/1 press once, 2-15 presses about 1 s apart
                              # (the car device repeats on its own); 0 for other commands
