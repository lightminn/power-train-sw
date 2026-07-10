uint8 CHECKING=0
uint8 VALID=1
uint8 INVALID_READING=2
uint8 NO_RESPONSE=3

std_msgs/Header header
uint8 status
float32 distance_mm
bool estop_required
uint32 consecutive_failures
string detail
