cmake_minimum_required(VERSION 3.0.2)
project(planner_semantic_msgs)
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
  actionlib_msgs
)


add_message_files(
  DIRECTORY msg
  FILES
  SemanticClass.msg
  SemanticPoint.msg
  SemanticPolygon.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs
)
