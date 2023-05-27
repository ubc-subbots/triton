# Object Offset message

# class_id  : Class of the object
#               1: Gate. Assume it always faces away from us. 
#               2: Pole
#               Make sure they match trajectory type
# pose      : pose of object in the robot frame

int32 class_id
geometry_msgs/Pose pose 
