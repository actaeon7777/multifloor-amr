# Current floor and whether the AMR is inside the elevator car.
uint8 ZONE_UNKNOWN=0
uint8 ZONE_OUTSIDE=1
uint8 ZONE_IN_ELEVATOR=2

std_msgs/Header header
uint8 zone
string floor                  # floors.yaml name, e.g. "4F"; empty while travelling or unknown
string transit_from
string transit_to
string confidence             # CONFIRMED | BEACON | TRANSIT | ASSUMED | UNKNOWN
string last_event
bool floor_conflict           # a non-current hall beacon is heard while outside
uint32 duplicate_count
uint32 out_of_order_count
uint32 stale_count
