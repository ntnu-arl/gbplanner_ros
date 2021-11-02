#ifndef GBPLANNER_H_
#define GBPLANNER_H_
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include "gbplanner/gbplanner_rviz.h"
#include "gbplanner/rrg.h"
#include "planner_common/geofence_manager.h"
#include "planner_common/graph.h"
#include "planner_common/graph_base.h"
#include "planner_common/graph_manager.h"
#include "planner_common/params.h"
#include "planner_msgs/RobotStatus.h"
#include "planner_msgs/planner_geofence.h"
#include "planner_msgs/planner_global.h"
#include "planner_msgs/planner_go_to_waypoint.h"
#include "planner_msgs/planner_homing.h"
#include "planner_msgs/planner_request_path.h"
#include "planner_msgs/planner_search.h"
#include "planner_msgs/planner_set_exp_mode.h"
#include "planner_msgs/planner_set_global_bound.h"
#include "planner_msgs/planner_set_homing_pos.h"
#include "planner_msgs/planner_set_planning_mode.h"
#include "planner_msgs/planner_set_search_mode.h"
#include "planner_msgs/planner_srv.h"
#include "planner_msgs/planner_string_trigger.h"

namespace explorer {

class Gbplanner {
 public:
  enum PlannerStatus { NOT_READY = 0, READY };

  Gbplanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  Gbplanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
            MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>*
                map_manager);

  void initializeAttributes();

  bool plannerServiceCallback(planner_msgs::planner_srv::Request& req,
                              planner_msgs::planner_srv::Response& res);

  void setGeofenceManager(std::shared_ptr<GeofenceManager> geofence_manager);
  void setUntraversablePolygon(
      const geometry_msgs::PolygonStamped& polygon_msgs);
  void setSharedParams(const RobotParams& robot_params,
                       const BoundedSpaceParams& global_space_params);
  void setSharedParams(const RobotParams& robot_params,
                       const BoundedSpaceParams& global_space_params,
                       const BoundedSpaceParams& local_space_params);

  Rrg* rrg_;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::ServiceServer planner_service_;
  ros::ServiceServer global_planner_service_;
  ros::ServiceServer planner_homing_service_;
  ros::ServiceServer planner_set_homing_pos_service_;
  ros::ServiceServer planner_search_service_;
  ros::ServiceServer planner_geofence_service_;
  ros::ServiceServer planner_passing_gate_service_;
  ros::ServiceServer planner_set_global_bound_service_;
  ros::ServiceServer planner_set_dynamic_global_bound_service_;
  ros::ServiceServer planner_clear_untraversable_zones_service_;
  ros::ServiceServer planner_load_graph_service_;
  ros::ServiceServer planner_save_graph_service_;
  ros::ServiceServer planner_goto_wp_service_;
  ros::ServiceServer planner_enable_untraversable_polygon_subscriber_service_;
  ros::ServiceServer planner_set_planning_trigger_mode_service_;

  ros::Subscriber pose_subscriber_;
  ros::Subscriber pose_stamped_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber untraversable_polygon_subscriber_;
  ros::Subscriber robot_status_subcriber_;
  ros::ServiceClient map_save_service_;

  PlannerStatus planner_status_;

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
  bool passingGateCallback(planner_msgs::planner_request_path::Request& req,
                           planner_msgs::planner_request_path::Response& res);
  bool setGlobalBound(planner_msgs::planner_set_global_bound::Request& req,
                      planner_msgs::planner_set_global_bound::Response& res);
  bool setDynamicGlobalBound(
      planner_msgs::planner_dynamic_global_bound::Request& req,
      planner_msgs::planner_dynamic_global_bound::Response& res);
  bool clearUntraversableZones(std_srvs::Trigger::Request& req,
                               std_srvs::Trigger::Response& res);

  bool plannerLoadGraphCallback(
      planner_msgs::planner_string_trigger::Request& req,
      planner_msgs::planner_string_trigger::Response& res);

  bool plannerSaveGraphCallback(
      planner_msgs::planner_string_trigger::Request& req,
      planner_msgs::planner_string_trigger::Response& res);

  // Goes to a point in the global graph that is closest to the given waypoint
  bool plannerGotoWaypointCallback(
      planner_msgs::planner_go_to_waypoint::Request& req,
      planner_msgs::planner_go_to_waypoint::Response& res);

  bool plannerEnableUntraversablePolygonSubscriberCallback(
      std_srvs::SetBool::Request& request,
      std_srvs::SetBool::Response& response);

  bool plannerSetPlanningTriggerModeCallback(
      planner_msgs::planner_set_planning_mode::Request& request,
      planner_msgs::planner_set_planning_mode::Response& response);

  void untraversablePolygonCallback(
      const geometry_msgs::PolygonStamped& polygon_msgs);
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void poseStampedCallback(const geometry_msgs::PoseStamped& pose);
  void processPose(const geometry_msgs::Pose& pose);
  void odometryCallback(const nav_msgs::Odometry& odo);
  void robotStatusCallback(const planner_msgs::RobotStatus& status);

  Gbplanner::PlannerStatus getPlannerStatus();
};

}  // namespace explorer
#endif
