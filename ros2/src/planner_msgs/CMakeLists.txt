cmake_minimum_required(VERSION 3.16)
project(planner_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(planner_msgs
  "msg/BehaviourPlannerLogger.msg"
  "msg/BoundMode.msg"
  "msg/CoveragePlannerLogger.msg"
  "msg/Edge.msg"
  "msg/ExecutionPathMode.msg"
  "msg/Graph.msg"
  "msg/GroundManeuver.msg"
  "msg/GroundManeuverSegment.msg"
  "msg/PlannerStatus.msg"
  "msg/PlanningBound.msg"
  "msg/PlanningMode.msg"
  "msg/RectangleShape.msg"
  "msg/RobotStatus.msg"
  "msg/TriggerMode.msg"
  "msg/Vertex.msg"
  "msg/VoxelMapUpdate.msg"
  "srv/AugmentFreeBox.srv"
  "srv/PciGeofence.srv"
  "srv/PciGlobal.srv"
  "srv/PciHomingTrigger.srv"
  "srv/PciInitialization.srv"
  "srv/PciSearch.srv"
  "srv/PciSetHomingPos.srv"
  "srv/PciStop.srv"
  "srv/PciToWaypoint.srv"
  "srv/PciTrigger.srv"
  "srv/PlannerDynamicGlobalBound.srv"
  "srv/PlannerGeofence.srv"
  "srv/PlannerGlobal.srv"
  "srv/PlannerGoToWaypoint.srv"
  "srv/PlannerHoming.srv"
  "srv/PlannerRequestPath.srv"
  "srv/PlannerSearch.srv"
  "srv/PlannerSetExpMode.srv"
  "srv/PlannerSetGlobalBound.srv"
  "srv/PlannerSetHomingPos.srv"
  "srv/PlannerSetPlanningMode.srv"
  "srv/PlannerSetSearchMode.srv"
  "srv/PlannerSetVel.srv"
  "srv/PlannerSrv.srv"
  "srv/PlannerStringTrigger.srv"
  "srv/ValidatePath.srv"
  "srv/ContinuePath.srv"
  "srv/PlanAhead.srv"
  "action/PathFollower.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
