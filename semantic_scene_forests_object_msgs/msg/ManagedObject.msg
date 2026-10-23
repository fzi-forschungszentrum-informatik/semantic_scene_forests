# Unique identifier for this object
int64 id

# Semantic class name (e.g., "cup", "chair", "table")
string class_name

# 3D detection with pose and bounding box
vision_msgs/Detection3D detection

# Header for timestamping and frame reference
std_msgs/Header header

# Manual blocking deletes the payload and keeps only a process-local suppression
# ID; blocked objects are not published. Unblocking waits for a fresh candidate
# rather than restoring old state.
bool is_blocked

# Source of object: 0=tracker, 1=manual_edit
uint8 source

# Timestamp of last modification
builtin_interfaces/Time last_updated

# Timestamp of the latest accepted 3D observation; not advanced by manual edits or publication
builtin_interfaces/Time last_seen
