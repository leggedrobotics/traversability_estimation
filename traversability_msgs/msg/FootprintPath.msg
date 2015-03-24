# Piece-wise linear path of the footprint poses.
geometry_msgs/PoseArray poses

# Either: Define footprint radius.
float64 radius 

# Or: Define footprint as polygon in the robot base frame.
# Polygon is used if it is defined, otherwise radius is used.
geometry_msgs/PolygonStamped footprint