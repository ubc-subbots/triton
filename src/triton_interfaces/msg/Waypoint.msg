# Waypoint message

# Waypoint types

uint8 STABILIZE=0
uint8 PASSTHROUGH=1

# pose      : destination pose
# success   : whether a pose has been achieved
# type      : waypoint type, must be one of the above waypoint types
# distance  : tolerable distance from destination 
# duration  : minimum duration at destination required

geometry_msgs/Pose pose 
bool success
uint8 type
geometry_msgs/Pose distance 
float32 duration