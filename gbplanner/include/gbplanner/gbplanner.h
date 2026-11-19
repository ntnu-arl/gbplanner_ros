#ifndef GBPLANNER_H_
#define GBPLANNER_H_
#include <ros1_compat/node_handle.h>
#include <planner_msgs/ros2_service_traits.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "gbplanner/gbplanner_rviz.h"
#include "gbplanner/rrg.h"
#include "planner_common/geofence_manager.h"
#include "planner_common/graph.h"
#include "planner_common/graph_base.h"
#include "planner_common/graph_manager.h"
#include "planner_common/params.h"
#include <planner_msgs/msg/robot_status.hpp>
#include <planner_msgs/srv/planner_geofence.hpp>
#include <planner_msgs/srv/planner_global.hpp>
#include <planner_msgs/srv/planner_go_to_waypoint.hpp>
#include <planner_msgs/srv/planner_homing.hpp>
#include <planner_msgs/srv/planner_request_path.hpp>
#include <planner_msgs/srv/planner_search.hpp>
#include <planner_msgs/srv/planner_set_exp_mode.hpp>
#include <planner_msgs/srv/planner_set_global_bound.hpp>
#include <planner_msgs/srv/planner_set_homing_pos.hpp>
#include <planner_msgs/srv/planner_set_planning_mode.hpp>
#include <planner_msgs/srv/planner_set_search_mode.hpp>
#include <planner_msgs/srv/planner_srv.hpp>
#include <planner_msgs/srv/planner_string_trigger.hpp>

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

  bool plannerServiceCallback(planner_msgs::srv::PlannerSrv::Request& req,
                              planner_msgs::srv::PlannerSrv::Response& res);

  void setGeofenceManager(std::shared_ptr<GeofenceManager> geofence_manager);
  void setUntraversablePolygon(
      const geometry_msgs::msg::PolygonStamped& polygon_msgs);
  void setSharedParams(const RobotParams& robot_params,
                       const BoundedSpaceParams& global_space_params);
  void setSharedParams(const RobotParams& robot_params,
                       const BoundedSpaceParams& global_space_params,
                       const BoundedSpaceParams& local_space_params);

  void setPlannerSrvReq(const planner_msgs::srv::PlannerSrv::Request& req)
  {
    in_srv_req_ = req;
  }
  
  void getPlannerSrvRes(planner_msgs::srv::PlannerSrv::Response& res)
  {
    res = out_srv_res_;
  }

  void clearResPath()
  {
    out_srv_res_.path.clear();
    out_srv_res_.status = planner_msgs::srv::PlannerSrv::Response::K_AUTO_CUSTOM_PATH;
    geometry_msgs::msg::Pose current_pose;
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
  planner_msgs::srv::PlannerSrv::Request in_srv_req_;
  planner_msgs::srv::PlannerSrv::Response out_srv_res_;

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

  std::vector<geometry_msgs::msg::Pose> active_homing_path_;
  std::vector<geometry_msgs::msg::Pose> active_global_path_;

  StateVec current_state_;

  PlannerStatus planner_status_;

  PlanningParams planning_params_;
  PlannerMode planner_mode_;
  int compartment_counter_ = 0;
  int exploration_counter_ = 0;
  int compartment_change_tries_ = 0;
  int max_compartment_change_tries_ = 3;
  bool decidePlanningAction();
  bool getExplorationPath(planner_msgs::srv::PlannerSrv::Request& req,
      planner_msgs::srv::PlannerSrv::Response& res);
  bool getInspectionPath(planner_msgs::srv::PlannerSrv::Request& req,
      planner_msgs::srv::PlannerSrv::Response& res);
  bool getCompartmentTransitionPath(planner_msgs::srv::PlannerSrv::Request& req,
      planner_msgs::srv::PlannerSrv::Response& res);

  bool opening_traversal_ongoing_ = false;
  bool opening_traversal_requested_ = false;
  bool inspection_requested_ = false;  // Temp

  bool homingServiceCallback(planner_msgs::srv::PlannerHoming::Request& req,
                             planner_msgs::srv::PlannerHoming::Response& res);
  bool setHomingPosServiceCallback(
      planner_msgs::srv::PlannerSetHomingPos::Request& req,
      planner_msgs::srv::PlannerSetHomingPos::Response& res);
  bool plannerSearchServiceCallback(
      planner_msgs::srv::PlannerSearch::Request& req,
      planner_msgs::srv::PlannerSearch::Response& res);
  bool globalPlannerServiceCallback(
      planner_msgs::srv::PlannerGlobal::Request& req,
      planner_msgs::srv::PlannerGlobal::Response& res);
  bool geofenceServiceCallback(planner_msgs::srv::PlannerGeofence::Request& req,
                               planner_msgs::srv::PlannerGeofence::Response& res);
  bool passingGateCallback(planner_msgs::srv::PlannerRequestPath::Request& req,
                           planner_msgs::srv::PlannerRequestPath::Response& res);
  bool setGlobalBound(planner_msgs::srv::PlannerSetGlobalBound::Request& req,
                      planner_msgs::srv::PlannerSetGlobalBound::Response& res);
  bool setDynamicGlobalBound(
      planner_msgs::srv::PlannerDynamicGlobalBound::Request& req,
      planner_msgs::srv::PlannerDynamicGlobalBound::Response& res);
  bool clearUntraversableZones(std_srvs::srv::Trigger::Request& req,
                               std_srvs::srv::Trigger::Response& res);

  bool plannerLoadGraphCallback(
      planner_msgs::srv::PlannerStringTrigger::Request& req,
      planner_msgs::srv::PlannerStringTrigger::Response& res);

  bool plannerSaveGraphCallback(
      planner_msgs::srv::PlannerStringTrigger::Request& req,
      planner_msgs::srv::PlannerStringTrigger::Response& res);

  // Goes to a point in the global graph that is closest to the given waypoint
  bool plannerGotoWaypointCallback(
      planner_msgs::srv::PlannerGoToWaypoint::Request& req,
      planner_msgs::srv::PlannerGoToWaypoint::Response& res);

  bool plannerEnableUntraversablePolygonSubscriberCallback(
      std_srvs::srv::SetBool::Request& request,
      std_srvs::srv::SetBool::Response& response);

  bool plannerSetPlanningTriggerModeCallback(
      planner_msgs::srv::PlannerSetPlanningMode::Request& request,
      planner_msgs::srv::PlannerSetPlanningMode::Response& response);

  bool forceCompartmentChangeServiceCallback(
              std_srvs::srv::Trigger::Request& req,
              std_srvs::srv::Trigger::Response& res);

  bool inspectionServiceCallback(
    planner_msgs::srv::PlannerSrv::Request& req,
    planner_msgs::srv::PlannerSrv::Response& res);
  
  bool switchOperationModeServiceCallback(
    std_srvs::srv::SetBool::Request& req,
    std_srvs::srv::SetBool::Response& res);

  void untraversablePolygonCallback(
      const geometry_msgs::msg::PolygonStamped& polygon_msgs);
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& pose);
  void poseStampedCallback(const geometry_msgs::msg::PoseStamped& pose);
  void processPose(const geometry_msgs::msg::Pose& pose);
  void odometryCallback(const nav_msgs::msg::Odometry& odo);
  void robotStatusCallback(const planner_msgs::msg::RobotStatus& status);
  void localNavGoalCallback(const geometry_msgs::msg::PoseStamped& goal);
  void stopMsgCallback(const std_msgs::msg::Bool& msg);

  Gbplanner::PlannerStatus getPlannerStatus();
};

// }  // namespace explorer
#endif
