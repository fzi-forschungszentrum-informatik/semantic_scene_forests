std_msgs/Header header

# Timestamp of the latest accepted 3D observation; not advanced by prediction or publication
builtin_interfaces/Time last_seen

int64 id
string class_name

vision_msgs/Detection3D detection
sensor_msgs/Image[] image_crops
