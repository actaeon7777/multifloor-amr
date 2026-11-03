# Answer to an arrival candidate (FloorState.awaiting_confirm) inside the car.
# The operator answers on the kiosk/web; a vision floor recognizer or the simulator may answer too.
uint8 SOURCE_OPERATOR=1
uint8 SOURCE_VISION=2
uint8 SOURCE_SIM=3

std_msgs/Header header
uint8 source
string floor                  # must equal FloorState.floor of the pending candidate, else ignored
uint32 candidate_id           # must equal FloorState.candidate_id being answered, else ignored
bool accepted                 # true: the car is at `floor`
string actual_floor           # only with accepted=false: the floor the answerer says it is
                              # (floors.yaml floor or pass floor), or "" for "don't know" (keep riding)
float32 score                 # recognizer confidence 0..1; 1.0 for operator and simulator
