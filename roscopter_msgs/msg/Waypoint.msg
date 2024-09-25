# Message defining a single waypoint

# Type of waypoint.
uint8 TYPE_HOLD = 0
uint8 TYPE_GOTO = 1

std_msgs/Header header

uint8 type

float32[3] w    # NED or LLA
float32 speed   # Desired speed
float32 psi     # Desired heading at waypoint

# Used by HOLD type waypoints
float32 hold_seconds
bool hold_indefinitely

# Other flags
bool use_lla