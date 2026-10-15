# Floor/zone transition or rule violation, e.g. ENTERED_ELEVATOR, EXITED_ELEVATOR, DEPARTED,
# ARRIVED, FLOOR_CONFIRMED, RULE_VIOLATION.
std_msgs/Header header
string event
string from_floor
string to_floor
string detail
