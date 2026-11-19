// Lets ros::NodeHandle::advertiseService deduce planner_msgs service types (ROS 2 port).
#pragma once
#include <ros1_compat/node_handle.h>
#include <planner_msgs/srv/pci_geofence.hpp>
#include <planner_msgs/srv/pci_global.hpp>
#include <planner_msgs/srv/pci_homing_trigger.hpp>
#include <planner_msgs/srv/pci_initialization.hpp>
#include <planner_msgs/srv/pci_search.hpp>
#include <planner_msgs/srv/pci_set_homing_pos.hpp>
#include <planner_msgs/srv/pci_stop.hpp>
#include <planner_msgs/srv/pci_to_waypoint.hpp>
#include <planner_msgs/srv/pci_trigger.hpp>
#include <planner_msgs/srv/planner_dynamic_global_bound.hpp>
#include <planner_msgs/srv/planner_geofence.hpp>
#include <planner_msgs/srv/planner_global.hpp>
#include <planner_msgs/srv/planner_go_to_waypoint.hpp>
#include <planner_msgs/srv/planner_homing.hpp>
#include <planner_msgs/srv/planner_opening_approval.hpp>
#include <planner_msgs/srv/planner_request_path.hpp>
#include <planner_msgs/srv/planner_search.hpp>
#include <planner_msgs/srv/planner_set_exp_mode.hpp>
#include <planner_msgs/srv/planner_set_global_bound.hpp>
#include <planner_msgs/srv/planner_set_homing_pos.hpp>
#include <planner_msgs/srv/planner_set_planning_mode.hpp>
#include <planner_msgs/srv/planner_set_search_mode.hpp>
#include <planner_msgs/srv/planner_set_vel.hpp>
#include <planner_msgs/srv/planner_srv.hpp>
#include <planner_msgs/srv/planner_string_trigger.hpp>
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PciGeofence)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PciGlobal)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PciHomingTrigger)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PciInitialization)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PciSearch)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PciSetHomingPos)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PciStop)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PciToWaypoint)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PciTrigger)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PlannerDynamicGlobalBound)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PlannerGeofence)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PlannerGlobal)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PlannerGoToWaypoint)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PlannerHoming)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PlannerOpeningApproval)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PlannerRequestPath)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PlannerSearch)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PlannerSetExpMode)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PlannerSetGlobalBound)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PlannerSetHomingPos)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PlannerSetPlanningMode)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PlannerSetSearchMode)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PlannerSetVel)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PlannerSrv)
ROS1_COMPAT_REGISTER_SERVICE(planner_msgs::srv::PlannerStringTrigger)
