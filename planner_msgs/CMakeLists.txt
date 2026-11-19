cmake_minimum_required(VERSION 3.8)
project(planner_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/BehaviourPlannerLogger.msg" "msg/BoundMode.msg" "msg/CoveragePlannerLogger.msg" "msg/Edge.msg" "msg/ExecutionPathMode.msg" "msg/Graph.msg" "msg/MultipleOpeningDetections.msg" "msg/OpeningDetection.msg" "msg/PlannerStatus.msg" "msg/PlanningBound.msg" "msg/PlanningMode.msg" "msg/RectangleShape.msg" "msg/RobotStatus.msg" "msg/TriggerMode.msg" "msg/Vertex.msg" "srv/PciGeofence.srv" "srv/PciGlobal.srv" "srv/PciHomingTrigger.srv" "srv/PciInitialization.srv" "srv/PciSearch.srv" "srv/PciSetHomingPos.srv" "srv/PciStop.srv" "srv/PciToWaypoint.srv" "srv/PciTrigger.srv" "srv/PlannerDynamicGlobalBound.srv" "srv/PlannerGeofence.srv" "srv/PlannerGlobal.srv" "srv/PlannerGoToWaypoint.srv" "srv/PlannerHoming.srv" "srv/PlannerOpeningApproval.srv" "srv/PlannerRequestPath.srv" "srv/PlannerSearch.srv" "srv/PlannerSetExpMode.srv" "srv/PlannerSetGlobalBound.srv" "srv/PlannerSetHomingPos.srv" "srv/PlannerSetPlanningMode.srv" "srv/PlannerSetSearchMode.srv" "srv/PlannerSetVel.srv" "srv/PlannerSrv.srv" "srv/PlannerStringTrigger.srv" "action/PathFollowerAction.action" DEPENDENCIES action_msgs geometry_msgs std_msgs)
find_package(ros1_compat REQUIRED)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})  # rosidl exports include/<pkg>

ament_export_dependencies(rosidl_default_runtime ros1_compat)
ament_package()
