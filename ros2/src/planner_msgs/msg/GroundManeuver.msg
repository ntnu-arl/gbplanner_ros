# Reference q = base_link + control_offset * heading; offset in metres.
# This reference model is not a claim of actual wheel/contact dynamics.
float64 control_offset
float64 max_speed
float64 max_yaw_rate
float64 max_acceleration
float64 max_yaw_acceleration
# Total execution budget in seconds of the motion (ROS simulation) clock.
float64 max_duration
# All arrays correspond one-to-one with PathFollower.path. Times start at zero
# and increase strictly; seconds, body-forward m/s, and yaw rad/s respectively.
# Time is a reference, not permission to skip an unobserved primitive boundary.
float64[] reference_times
float64[] forward_velocities
float64[] yaw_rates
planner_msgs/GroundManeuverSegment[] segments
