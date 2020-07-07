#ifndef GBPLANNER_H_
#define GBPLANNER_H_
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include "gbplanner/gbplanner_rviz.h"
#include "gbplanner/geofence_manager.h"
#include "gbplanner/graph.h"
#include "gbplanner/params.h"
#include "gbplanner/rrg.h"
#include "planner_msgs/RobotStatus.h"
#include "planner_msgs/planner_geofence.h"
#include "planner_msgs/planner_global.h"
#include "planner_msgs/planner_homing.h"
#include "planner_msgs/planner_request_path.h"
#include "planner_msgs/planner_search.h"
#include "planner_msgs/planner_set_exp_mode.h"
#include "planner_msgs/planner_set_global_bound.h"
#include "planner_msgs/planner_set_homing_pos.h"
#include "planner_msgs/planner_srv.h"
#include "planner_msgs/planner_string_trigger.h"

namespace explorer {

class Gbplanner {
 public:
  enum PlannerStatus { NOT_READY = 0, READY };
  enum PlanningState { kNull = 0, kStart, kBoxClearance, kExploration, kStop };
  Gbplanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::ServiceServer planner_service_;
  ros::ServiceServer global_planner_service_;
  ros::ServiceServer planner_homing_service_;
  ros::ServiceServer planner_set_homing_pos_service_;
  ros::ServiceServer planner_search_service_;
  ros::ServiceServer planner_geofence_service_;
  ros::ServiceServer planner_set_global_bound_service_;
  ros::ServiceServer planner_set_exp_mode_service_;
  ros::ServiceServer planner_load_graph_service_;
  ros::ServiceServer planner_save_graph_service_;

  ros::Subscriber pose_subscriber_;
  ros::Subscriber pose_stamped_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber robot_status_subcriber_;
  ros::ServiceClient map_save_service_;

  gbplanner::Rrg* rrg_;

  PlannerStatus planner_status_;
  PlanningState planning_state_;

  bool plannerServiceCallback(planner_msgs::planner_srv::Request& req,
                              planner_msgs::planner_srv::Response& res);
  bool homingServiceCallback(planner_msgs::planner_homing::Request& req,
                             planner_msgs::planner_homing::Response& res);
  bool setHomingPosServiceCallback(
      planner_msgs::planner_set_homing_pos::Request& req,
      planner_msgs::planner_set_homing_pos::Response& res);
  bool plannerSearchServiceCallback(
      planner_msgs::planner_search::Request& req,
      planner_msgs::planner_search::Response& res);
  bool globalPlannerServiceCallback(
      planner_msgs::planner_global::Request& req,
      planner_msgs::planner_global::Response& res);
  bool geofenceServiceCallback(planner_msgs::planner_geofence::Request& req,
                               planner_msgs::planner_geofence::Response& res);
  bool setGlobalBound(planner_msgs::planner_set_global_bound::Request& req,
                      planner_msgs::planner_set_global_bound::Response& res);
  bool setExpMode(planner_msgs::planner_set_exp_mode::Request& req,
                  planner_msgs::planner_set_exp_mode::Response& res);

  bool plannerLoadGraphCallback(
      planner_msgs::planner_string_trigger::Request& req,
      planner_msgs::planner_string_trigger::Response& res);

  bool plannerSaveGraphCallback(
      planner_msgs::planner_string_trigger::Request& req,
      planner_msgs::planner_string_trigger::Response& res);

  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void poseStampedCallback(const geometry_msgs::PoseStamped& pose);
  void processPose(const geometry_msgs::Pose& pose);
  void odometryCallback(const nav_msgs::Odometry& odo);
  void robotStatusCallback(const planner_msgs::RobotStatus& status);

  Gbplanner::PlannerStatus getPlannerStatus();
};

}  // namespace explorer
#endif
