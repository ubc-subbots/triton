# Waypoint message

# Waypoint types

uint8 STABILIZE=0
uint8 PASSTHROUGH=1

# pose : destination pose
# success : whether a pose has been achieved
# type : waypoint type, must be one of the above waypoint types

geometry_msgs/Pose pose 
bool success
uint8 type