#ifndef GBPLANNER_H_
#define GBPLANNER_H_
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include "gbplanner/gbplanner_rviz.h"
#include "gbplanner/rrg.h"
#include "planner_common/geofence_manager.h"
#include "graph/graph.hpp"
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

#include "common/communicator.hpp"
#include "bwt_ssg_builder/bwt_ssg_manager.hpp"


class Gbplanner {
 public:
  enum PlannerStatus { NOT_READY = 0, READY };

  enum PlannerMode {
    kExploration = 0,
    kExplorationComplete,
    kInspection,
    kCompartmentChange
  };

  Gbplanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, std::shared_ptr<Communicator> comm);
  Gbplanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
            MapManagerVoxblox<MapManagerVoxbloxServer, MapManagerVoxbloxVoxel>*
                map_manager, std::shared_ptr<Communicator> comm);

  void initializeAttributes();

  void setBoundMode(BoundModeType bmode);
  void setRobotBoundingBox(Eigen::Vector3d robot_box)
  {
    rrg_->setRobotBoundingBox(robot_box);
  }
  void setRootState(geometry_msgs::Pose root_pose);

  bool plannerServiceCallback(planner_msgs::planner_srv::Request& req,
                              planner_msgs::planner_srv::Response& res);
  void convertLongs(const geometry_msgs::PoseArray &longs_array, std::vector<Longitudinal> &longs_vec);

  void setGeofenceManager(std::shared_ptr<GeofenceManager> geofence_manager);
  void setUntraversablePolygon(
      const geometry_msgs::PolygonStamped& polygon_msgs);
  void setSharedParams(const RobotParams& robot_params,
                       const BoundedSpaceParams& global_space_params);
  void setSharedParams(const RobotParams& robot_params,
                       const BoundedSpaceParams& global_space_params,
                       const BoundedSpaceParams& local_space_params);

  bool getLocalExplorationPath(planner_msgs::planner_srv::Request& req,
      planner_msgs::planner_srv::Response& res);
  bool getAssistedExplorationPath(planner_msgs::planner_srv::Request& req,
      planner_msgs::planner_srv::Response& res);
//   bool getInspectionPath(planner_msgs::planner_srv::Request& req,
//       planner_msgs::planner_srv::Response& res);
  std::vector<StateVec> getBlindTSPOrder(std::vector<StateVec> viewpoints) { return rrg_->getBlindTSPOrder(viewpoints); }
  std::vector<StateVec> getLongsInspectionViewpointsOnly(std::vector<Longitudinal> longs, bool all) { return rrg_->getLongsInspectionViewpointsOnly(longs, all); }
  bool getInspectionPath(std::vector<geometry_msgs::Pose> &inspection_path, InspectionStatus &status);
  std::vector<geometry_msgs::Pose> getInspectionPath(std::vector<Longitudinal> longs, InspectionStatus &status);
  bool getCompartmentTransitionPath(planner_msgs::planner_srv::Request& req,
      planner_msgs::planner_srv::Response& res);
  std::vector<geometry_msgs::Pose> getManholeTraversalPath(ManholeTraversalMode mode, ManholeTraversalStatus &status);
  std::vector<geometry_msgs::Pose> getManholeTraversalPath(ManholeTraversalMode mode, ManholeTraversalStatus &status, int mh_id);
  std::vector<int> getTraversedManholesInOrder();
  bool updateCompartmentCounter();
  bool updateCompartmentBoundingBox();
  bool lastCompartment();
  std::vector<StateVec> getVerificationViewpoints(std::vector<Longitudinal> longs);
  int getCompartmentCounter() { return compartment_counter_; }
  void annotateSemanticPredictions(std::vector<Eigen::Vector3d> points) { rrg_->annotateSemanticPredictions(points); }
  void annotateSemanticPredictions(std::shared_ptr<SSGManager> predicted_graph) { rrg_->annotateSemanticPredictions(predicted_graph); }
  bool planTo(geometry_msgs::Pose source_pose,
              geometry_msgs::Pose target_pose, bool use_current_state,
              std::vector<geometry_msgs::Pose>& path_ret);
  bool planTo(StateVec& source, StateVec& target, RandomSamplingParams& params, std::vector<geometry_msgs::Pose>& path_ret);
  bool isSeen(StateVec viewpoint, Longitudinal l) { return rrg_->isSeen(viewpoint, l); }
  void setConfig(std::shared_ptr<Config> config)
  { 
    config_ = config; 
    rrg_->setConfig(config);
  }

  void setRobotBoxSize(Eigen::Vector3d box) { rrg_->setRobotBoxSize(box);}

  void doAnnotation(bool trig);

  StateVec robotState() {return current_state_;}

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
  ros::ServiceServer planner_stop_service_;
  ros::ServiceServer inspection_path_service_;
  ros::ServiceServer force_compartment_transition_service_;

  ros::Subscriber pose_subscriber_;
  ros::Subscriber pose_stamped_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber untraversable_polygon_subscriber_;
  ros::Subscriber robot_status_subcriber_;
  ros::ServiceClient map_save_service_;

  ros::Publisher current_compartment_center_pub_;

  std::shared_ptr<Communicator> comm_;
  std::shared_ptr<Config> config_;

  StateVec current_state_;

  PlannerStatus planner_status_;

  PlanningParams planning_params_;
  PlannerMode planner_mode_;
  // std::vector<Eigen::Vector3d> compartment_centers_;
  // BoundedSpaceParams compartment_dimensions_;
  int compartment_counter_ = 0;
  int exploration_counter_ = 0;
  int compartment_change_tries_ = 0;
  int max_compartment_change_tries_ = 3;
  // int max_exploration_iterations_ = 2;
  // bool exploration_only_ = false;
  bool decidePlanningAction();
  bool getExplorationPath(planner_msgs::planner_srv::Request& req,
      planner_msgs::planner_srv::Response& res);
  

  bool manhole_traversal_ongoing_ = false;
  bool manhole_traversal_requested_ = false;
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

  bool stopServiceCallback(
              std_srvs::Trigger::Request& req,
              std_srvs::Trigger::Response& res);
  bool forceCompartmentChangeServiceCallback(
              std_srvs::Trigger::Request& req,
              std_srvs::Trigger::Response& res);

  bool inspectionServiceCallback(
    planner_msgs::planner_srv::Request& req,
    planner_msgs::planner_srv::Response& res);

  void untraversablePolygonCallback(
      const geometry_msgs::PolygonStamped& polygon_msgs);
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void poseStampedCallback(const geometry_msgs::PoseStamped& pose);
  void processPose(const geometry_msgs::Pose& pose);
  void odometryCallback(const nav_msgs::Odometry& odo);
  void robotStatusCallback(const planner_msgs::RobotStatus& status);

  Gbplanner::PlannerStatus getPlannerStatus();
};
#endif
