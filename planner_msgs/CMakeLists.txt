cmake_minimum_required(VERSION 3.0.2)
project(planner_msgs)
find_package(catkin REQUIRED COMPONENTS
std_msgs
geometry_msgs
message_generation
actionlib_msgs)

add_service_files(
  DIRECTORY srv
  FILES
  planner_srv.srv
  planner_global.srv
  planner_search.srv
  planner_homing.srv
  planner_set_homing_pos.srv
  planner_geofence.srv
  planner_request_path.srv
  pci_trigger.srv
  pci_homing_trigger.srv
  pci_set_homing_pos.srv
  pci_initialization.srv
  pci_search.srv
  pci_global.srv
  pci_stop.srv
  pci_geofence.srv
  pci_to_waypoint.srv
  planner_set_global_bound.srv
  planner_set_vel.srv
  planner_set_exp_mode.srv
  planner_string_trigger.srv
  planner_set_search_mode.srv
  planner_go_to_waypoint.srv
  planner_dynamic_global_bound.srv
  planner_set_planning_mode.srv
)

add_message_files(
  DIRECTORY msg
  FILES
  RobotStatus.msg
  RectangleShape.msg
  BoundMode.msg
  PlanningMode.msg
  ExecutionPathMode.msg
  TriggerMode.msg
  PlannerStatus.msg
  PlanningBound.msg
  Vertex.msg
  Edge.msg
  Graph.msg
  CoveragePlannerLogger.msg
  BehaviourPlannerLogger.msg
)

add_action_files(
 FILES
 pathFollowerAction.action
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs actionlib_msgs
)
