# Current floor and whether the AMR is inside the elevator car.
uint8 ZONE_UNKNOWN=0
uint8 ZONE_OUTSIDE=1
uint8 ZONE_IN_ELEVATOR=2

std_msgs/Header header
uint8 zone
string floor                  # floors.yaml name, e.g. "4F"; empty while travelling or unknown
string transit_from
string transit_to
string confidence             # CONFIRMED | VERIFIED | TRANSIT | ASSUMED | UNKNOWN
string last_event
bool awaiting_confirm         # in the car, door open after travel: `floor` is the asked floor (camera
                              # guess, or the ride target when the camera is unsure) and needs a FloorArrivalConfirm
uint32 candidate_id           # increments with every new arrival candidate; 0 before the first
string candidate_source       # while awaiting_confirm: "VISION" (camera guess) or "TARGET" (ride target fallback)
