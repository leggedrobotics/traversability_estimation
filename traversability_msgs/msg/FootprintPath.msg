# Path of the footprint defined as array of connected poses.
# Poses are connected piece-wise linear in the order of the array.
geometry_msgs/PoseArray poses

# Either: Define footprint radius.
float64 radius 

# Or: Define footprint as polygon in the robot base frame.
# Polygon is used if it is defined, otherwise radius is used.
geometry_msgs/PolygonStamped footprint

# Use conservative footprint. Only available if polygon is used.
bool conservative

# Compute untraversable polygon in the checked area for traversability. If true, computation demand is higher.
bool compute_untraversable_polygon
