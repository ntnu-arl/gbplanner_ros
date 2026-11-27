cmake_minimum_required(VERSION 3.16)
project(planner_gazebo_sim)

find_package(ament_cmake REQUIRED)

# This package owns data and launch glue only. The actual Gazebo executable and
# ROS plugins remain runtime dependencies, so the package is buildable on the
# development image even when Gazebo is not installed there.
install(DIRECTORY models worlds worlds_ros2 launch collision
  DESTINATION share/${PROJECT_NAME})
install(FILES SOURCE.json UPSTREAM_LICENSE UPSTREAM_package.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
