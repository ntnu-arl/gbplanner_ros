#ifndef PLANNER_CONTROL_INTERFACE_H_
#define PLANNER_CONTROL_INTERFACE_H_

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "planner_control_interface/pci_manager.h"
#include "planner_msgs/BoundMode.h"
#include "planner_msgs/ExecutionPathMode.h"
#include "planner_msgs/PlannerStatus.h"
#include "planner_msgs/PlanningMode.h"
#include "planner_msgs/TriggerMode.h"
#include "planner_msgs/pci_geofence.h"
#include "planner_msgs/pci_global.h"
#include "planner_msgs/pci_homing_trigger.h"
#include "planner_msgs/pci_initialization.h"
#include "planner_msgs/pci_search.h"
#include "planner_msgs/pci_set_homing_pos.h"
#include "planner_msgs/pci_stop.h"
#include "planner_msgs/pci_to_waypoint.h"
#include "planner_msgs/pci_trigger.h"
#include "planner_msgs/planner_geofence.h"
#include "planner_msgs/planner_global.h"
#include "planner_msgs/planner_go_to_waypoint.h"
#include "planner_msgs/planner_homing.h"
#include "planner_msgs/planner_request_path.h"
#include "planner_msgs/planner_search.h"
#include "planner_msgs/planner_set_exp_mode.h"
#include "planner_msgs/planner_set_homing_pos.h"
#include "planner_msgs/planner_set_planning_mode.h"
#include "planner_msgs/planner_srv.h"
#include "planner_semantic_msgs/SemanticPoint.h"

namespace explorer {

class PlannerControlInterface {
 public:
  enum struct RobotModeType { kAerialRobot = 0, kLeggedRobot };
  enum struct RunModeType {
    kSim = 0,  // Run in simulation.
    kReal,     // Run with real robot.
  };
  enum struct PlannerTriggerModeType {
    kManual = 0,  // Manually trigger the control interface each time.
    kAuto = 1     // Automatic exploration.
  };

  PlannerControlInterface(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                          std::shared_ptr<PCIManager> pci_manager);

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

 private:
  ros::Publisher reference_pub_;
  ros::Publisher planner_status_pub_;
  ros::Publisher stop_request_pub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber pose_stamped_sub_;
  ros::Subscriber nav_goal_sub_;
  ros::Subscriber pose_goal_sub_;
  ros::ServiceClient planner_client_;
  ros::ServiceClient planner_homing_client_;
  ros::ServiceClient planner_set_homing_pos_client_;
  ros::ServiceClient planner_search_client_;
  ros::ServiceClient planner_global_client_;
  ros::ServiceClient planner_geofence_client_;
  ros::ServiceClient planner_passing_gate_client_;
  ros::ServiceClient planner_set_exp_mode_client_;
  ros::ServiceClient nav_goal_client_;
  ros::ServiceClient planner_set_trigger_mode_client_;

  ros::ServiceServer pci_server_;
  ros::ServiceServer pci_std_automatic_planning_server_;
  ros::ServiceServer pci_std_single_planning_server_;
  ros::ServiceServer pci_homing_server_;
  ros::ServiceServer pci_std_set_homing_pos_server_;
  ros::ServiceServer pci_std_homing_server_;
  ros::ServiceServer pci_set_homing_pos_server_;
  ros::ServiceServer pci_initialization_server_;
  ros::ServiceServer pci_search_server_;
  ros::ServiceServer pci_global_server_;
  ros::ServiceServer pci_stop_server_;
  ros::ServiceServer pci_std_stop_server_;
  ros::ServiceServer pci_std_go_to_waypoint_server_;
  ros::ServiceServer pci_geofence_server_;
  ros::ServiceServer pci_to_waypoint_server_;
  ros::ServiceServer pci_passing_gate_server_;
  ros::ServiceServer rotate_180_deg_server_;
  ros::ServiceServer pci_std_global_last_specified_frontier_server_;

  tf::TransformListener tf_listener_;

  std::shared_ptr<PCIManager> pci_manager_;
  uint8_t bound_mode_;
  PlannerTriggerModeType trigger_mode_;
  double v_current_;
  bool run_en_;
  bool exe_path_en_;
  bool force_forward_;
  bool homing_request_;
  bool pose_is_ready_;
  bool init_request_;
  bool global_request_;
  bool stop_planner_request_;

  bool passing_gate_success_;
  bool passing_gate_request_;

  geometry_msgs::Pose set_waypoint_;
  geometry_msgs::PoseStamped set_waypoint_stamped_;
  // Visualization Publisher
  ros::Publisher go_to_waypoint_visualization_pub_;

  bool go_to_waypoint_request_;
  bool go_to_waypoint_with_checking_;
  bool received_first_waypoint_to_go_ = false;

  int reference_pub_id_;
  planner_msgs::pci_global::Request pci_global_request_params_;
  int frontier_id_;

  std::string world_frame_name = "world";

  // Semantics marker
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer>
      semantic_server;

  interactive_markers::MenuHandler menu_handler;
  interactive_markers::MenuHandler::EntryHandle accept_entry_handle;
  interactive_markers::MenuHandler::EntryHandle class_entry_handle;
  interactive_markers::MenuHandler::EntryHandle sub_class_entry_handle;

  const std::string kStaircaseStr = "Stairs";
  const std::string kDoorStr = "Door";
  planner_semantic_msgs::SemanticPoint semantic_location;
  planner_semantic_msgs::SemanticClass current_semantic_class_;
  geometry_msgs::Point32 semantic_position;

  ros::Publisher semantic_pub;

  double control_size = 1.0;

  bool menu_initialized = false;

  // Current following path.
  std::vector<geometry_msgs::Pose> current_path_;

  int planner_iteration_;
  geometry_msgs::Pose current_pose_;
  geometry_msgs::Pose previous_pose_;
  std::string world_frame_id_;

  void odometryCallback(const nav_msgs::Odometry& odo);
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void navGoalCallback(const geometry_msgs::PoseStamped& nav_msg);
  void poseGoalCallback(const geometry_msgs::PoseStamped& pose_msg);
  void setGoal(const geometry_msgs::PoseStamped& pose);
  void poseStampedCallback(const geometry_msgs::PoseStamped& pose);
  void processPose(const geometry_msgs::Pose& pose);

  bool setHomingPosCallback(planner_msgs::pci_set_homing_pos::Request& req,
                            planner_msgs::pci_set_homing_pos::Response& res);
  bool stdSrvSetHomingPositionHereCallback(std_srvs::Trigger::Request& req,
                                           std_srvs::Trigger::Response& res);

  bool homingCallback(planner_msgs::pci_homing_trigger::Request& req,
                      planner_msgs::pci_homing_trigger::Response& res);
  bool stdSrvHomingCallback(std_srvs::Trigger::Request& req,
                            std_srvs::Trigger::Response& res);
  bool triggerCallback(planner_msgs::pci_trigger::Request& req,
                       planner_msgs::pci_trigger::Response& res);
  bool stdSrvsAutomaticPlanningCallback(std_srvs::Trigger::Request& req,
                                        std_srvs::Trigger::Response& res);

  bool stdSrvGoToWaypointCallback(std_srvs::Trigger::Request& req,
                                  std_srvs::Trigger::Response& res);

  bool stdSrvsSinglePlanningCallback(std_srvs::Trigger::Request& req,
                                     std_srvs::Trigger::Response& res);

  bool initializationCallback(planner_msgs::pci_initialization::Request& req,
                              planner_msgs::pci_initialization::Response& res);

  bool searchCallback(planner_msgs::pci_search::Request& req,
                      planner_msgs::pci_search::Response& res);

  bool globalPlannerCallback(planner_msgs::pci_global::Request& req,
                             planner_msgs::pci_global::Response& res);

  bool stopPlannerCallback(planner_msgs::pci_stop::Request& req,
                           planner_msgs::pci_stop::Response& res);
  bool stdSrvStopPlannerCallback(std_srvs::Trigger::Request& req,
                                 std_srvs::Trigger::Response& res);

  bool addGeofenceCallback(planner_msgs::pci_geofence::Request& req,
                           planner_msgs::pci_geofence::Response& res);
  bool goToWaypointCallback(planner_msgs::pci_to_waypoint::Request& req,
                            planner_msgs::pci_to_waypoint::Response& res);
  bool passingGateCallback(std_srvs::Trigger::Request& req,
                           std_srvs::Trigger::Response& res);

  bool rotate180DegCallback(std_srvs::Trigger::Request& req,
                            std_srvs::Trigger::Response& res);

  bool stdSrvReplanLastSpecifiedFrontierCallback(
      std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  void resetPlanner();

  bool loadParams();
  bool init();
  void run();
  void runPlanner(bool exe_path);
  void runGlobalPlanner(bool exe_path);
  void runHoming(bool exe_path);
  void runInitialization();
  void runSearch(bool exe_path);
  void runPassingGate();
  void runGlobalRepositioning();
  geometry_msgs::Pose getPoseToStart();

  bool search_request_;
  bool use_current_state_;
  const std::string source_marker_name = "wp_source";
  const std::string target_marker_name = "wp_target";
  geometry_msgs::Pose source_setpoint_;
  geometry_msgs::Pose target_setpoint_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> imarker_server_;
  // Waypoints i-markers
  void initIMarker();
  void processFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void publishPlannerStatus(const planner_msgs::planner_srv::Response& res,
                            bool success);
  void publishGoToWaypointVisualization(
      const geometry_msgs::PoseStamped& poseStamped);
  // Semantics i-marker
  void initSemanticIMarker();
  void semanticMarkerFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void acceptButtonFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void selectSemanticsFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
};
}  // namespace explorer

#endif
