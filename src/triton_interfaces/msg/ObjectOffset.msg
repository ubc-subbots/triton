# Object Offset message

# class_id  : Class of the object
#               0: Gate. Assume it always faces away from us. 
#               1: Pole
# pose      : pose of object in the robot frame

int32 class_id
geometry_msgs/Pose pose 
