# Elevator RF device -> AMR. Real: RF -> STM32 -> CAN -> elevator_can_link. Simulation: elevator_sim.
# A hall beacon announces its floor every second (EVENT_PRESENCE). Any device reports a button
# press it performed for an AMR command (EVENT_BUTTON_ACK). Door state is NOT reported; the AMR
# senses doors with its LiDAR.
uint8 EVENT_PRESENCE=1
uint8 EVENT_BUTTON_ACK=2

uint8 LOCATION_HALL=0
uint8 LOCATION_CAR=1

std_msgs/Header header        # reception time on the AMR clock
uint8 event
uint8 location
int8 floor_code               # hall device floor (floors.yaml beacon_floor_code); 0 for the car device
uint8 sequence                # device frame sequence, wraps at 256
uint8 ack_sequence            # ElevatorCommand.sequence acknowledged by EVENT_BUTTON_ACK
int8 rssi_dbm                 # 0 when unavailable
string source                 # "can" | "sim"
