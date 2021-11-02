# Represent semantics as a flat 3D polygon
std_msgs/Header header

# Class of semantics
planner_semantic_msgs/SemanticClass type

# Further info if needed
std_msgs/String description

# Coordinate of 4 corners of the polygon listed in clockwise direction
geometry_msgs/Polygon polygon