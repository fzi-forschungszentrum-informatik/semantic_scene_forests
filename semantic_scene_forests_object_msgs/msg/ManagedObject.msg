# Unique identifier for this object
int64 id

# Semantic class name (e.g., "cup", "chair", "table")
string class_name

# 3D detection with pose and bounding box
vision_msgs/Detection3D detection

# Header for timestamping and frame reference
std_msgs/Header header

# Indicates if object is manually blocked (not published when true)
bool is_blocked

# Source of object: 0=tracker, 1=manual_edit
uint8 source

# Timestamp of last modification
builtin_interfaces/Time last_updated
