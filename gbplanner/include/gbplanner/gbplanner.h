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

// namespace explorer {

class Gbplanner {
 public:
  enum PlannerStatus { NOT_READY = 0, READY };

  enum PlannerMode {
    kExploration = 0,
    kExplorationComplete,
    kInspection,
    kCompartmentChange
  };

  struct PlannerBTStates
  {
    bool local_exp_exhausted = false;
    bool local_navigation_complete = false;
    bool local_navigation_stuck = false;
    bool homing_triggered = false;
    bool homing_required = false;
    bool global_exp_exhausted = false;
    bool opening_phase1_failed = false;
    int operation_mode = 0; // 0: exploration/inspection, 1: local_navigation
  };
   

  Gbplanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  Gbplanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
            MapManager* map_manager);

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

  void setPlannerSrvReq(const planner_msgs::planner_srv::Request& req)
  {
    in_srv_req_ = req;
  }
  
  void getPlannerSrvRes(planner_msgs::planner_srv::Response& res)
  {
    res = out_srv_res_;
  }

  void clearResPath()
  {
    out_srv_res_.path.clear();
    out_srv_res_.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
    geometry_msgs::Pose current_pose;
    tf::Quaternion quat;
    quat.setEuler(0.0, 0.0, current_state_[3]);
    tf::Vector3 origin(current_state_[0], current_state_[1], current_state_[2]);
    tf::Pose poseTF(quat, origin);
    tf::poseTFToMsg(poseTF, current_pose);
    out_srv_res_.path.push_back(current_pose);
  }

  Rrg::LocalPlannerStatus getExplorationPath();
  Rrg::LocalPlannerStatus getLocalNavigationPath();
  Rrg::GlobalPlannerStatus getGlobalExplorationPath();
  bool checkGlobalExplorationStatus();
  bool getInspectionPath();
  
  bool getHomingPath();
  bool homingRequired();
  bool calculateHomingPath(); // For active homing
  bool updateHomingGoal();
  bool calculateGlobalPath(); // For active global planner
  bool updateGlobalGoal();

  void getOpeningTraversalPath(OpeningTraversalMode mode, OpeningTraversalStatus &status);
  bool transitionCompartment();
  bool allCompartmentsInspected();
	bool getCompartmentTransitionPath();
  
  Rrg* rrg_;
  PlannerBTStates bt_states_;
  planner_msgs::planner_srv::Request in_srv_req_;
  planner_msgs::planner_srv::Response out_srv_res_;

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
  ros::ServiceServer planner_stop_service_;
  ros::ServiceServer inspection_path_service_;
  ros::ServiceServer force_compartment_transition_service_;
  ros::ServiceServer switch_operation_mode_service_;

  ros::Publisher global_planner_local_goal_pub_;

  ros::Subscriber pose_subscriber_;
  ros::Subscriber pose_stamped_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber untraversable_polygon_subscriber_;
  ros::Subscriber robot_status_subcriber_;
  ros::Subscriber local_nav_goal_subscriber_;
  ros::Subscriber stop_srv_subscriber_;  

  ros::ServiceClient map_save_service_;

  std::vector<geometry_msgs::Pose> active_homing_path_;
  std::vector<geometry_msgs::Pose> active_global_path_;

  StateVec current_state_;

  PlannerStatus planner_status_;

  PlanningParams planning_params_;
  PlannerMode planner_mode_;
  int compartment_counter_ = 0;
  int exploration_counter_ = 0;
  int compartment_change_tries_ = 0;
  int max_compartment_change_tries_ = 3;
  bool decidePlanningAction();
  bool getExplorationPath(planner_msgs::planner_srv::Request& req,
      planner_msgs::planner_srv::Response& res);
  bool getInspectionPath(planner_msgs::planner_srv::Request& req,
      planner_msgs::planner_srv::Response& res);
  bool getCompartmentTransitionPath(planner_msgs::planner_srv::Request& req,
      planner_msgs::planner_srv::Response& res);

  bool opening_traversal_ongoing_ = false;
  bool opening_traversal_requested_ = false;
  bool inspection_requested_ = false;  // Temp

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

  bool forceCompartmentChangeServiceCallback(
              std_srvs::Trigger::Request& req,
              std_srvs::Trigger::Response& res);

  bool inspectionServiceCallback(
    planner_msgs::planner_srv::Request& req,
    planner_msgs::planner_srv::Response& res);
  
  bool switchOperationModeServiceCallback(
    std_srvs::SetBool::Request& req,
    std_srvs::SetBool::Response& res);

  void untraversablePolygonCallback(
      const geometry_msgs::PolygonStamped& polygon_msgs);
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void poseStampedCallback(const geometry_msgs::PoseStamped& pose);
  void processPose(const geometry_msgs::Pose& pose);
  void odometryCallback(const nav_msgs::Odometry& odo);
  void robotStatusCallback(const planner_msgs::RobotStatus& status);
  void localNavGoalCallback(const geometry_msgs::PoseStamped& goal);
  void stopMsgCallback(const std_msgs::Bool& msg);

  Gbplanner::PlannerStatus getPlannerStatus();
};

// }  // namespace explorer
#endif
