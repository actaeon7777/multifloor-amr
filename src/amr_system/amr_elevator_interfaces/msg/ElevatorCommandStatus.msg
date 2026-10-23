# STM32 RF relay -> AMR: processing stage of an ElevatorCommand (CAN 0x311 kind 1).
# Real: STM32 -> CAN -> elevator_can_link. Simulation: elevator_sim.
uint8 STATUS_DELIVERED=1        # the beacon hardware-acknowledged the RF COMMAND
uint8 STATUS_COMPLETED=2        # success RESULT (PRESSED/DUPLICATE); EVENT_BUTTON_ACK follows
uint8 STATUS_DEVICE_REJECTED=3  # failure RESULT, see device_result
uint8 STATUS_RF_TIMEOUT=4       # not delivered within 2.5 s of the command
uint8 STATUS_NO_RESULT=5        # delivered, but no RESULT within 1.5 s
uint8 STATUS_NO_ROUTE=6         # no beacon for this command
uint8 STATUS_BAD_FRAME=7        # the 0x210 frame failed validation
uint8 STATUS_REPLACED=8         # a newer command for the same beacon replaced it

uint8 RESULT_PRESSED=0
uint8 RESULT_DUPLICATE=1
uint8 RESULT_WRONG_DEVICE=129
uint8 RESULT_UNSUPPORTED=130    # e.g. UP on the top floor, DOWN on the bottom floor
uint8 RESULT_BUTTON_FAULT=131
uint8 RESULT_BAD_REQUEST=132
uint8 RESULT_NONE=255

std_msgs/Header header          # reception time on the AMR clock
uint8 cmd_seq                   # ElevatorCommand.sequence
uint8 status
uint8 device_result             # beacon RESULT code, RESULT_NONE when there is none
uint8 target                    # beacon RF node_id (hall 0x40 + floor, car 0x21), 0 if unknown
uint8 attempts                  # RF COMMAND attempts (beacon wake-ups); 0 for a late RESULT
string source                   # "can" | "sim"
