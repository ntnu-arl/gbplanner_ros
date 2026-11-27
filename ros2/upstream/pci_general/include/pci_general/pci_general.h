#ifndef GBPLANNER_PCI_ANYMAL_H_
#define GBPLANNER_PCI_ANYMAL_H_

#include <actionlib/client/simple_action_client.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Status.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "planner_control_interface/pci_manager.h"
#include "planner_msgs/pathFollowerActionAction.h"

namespace explorer {

class PCIGeneral : public PCIManager {
 public:
  enum struct OutputType { kTopic = 0, kAction };
  PCIGeneral(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  bool loadParams(const std::string ns);
  bool initialize();
  bool initMotion();
  // To change
  bool executePath(const std::vector<geometry_msgs::Pose>& path,
                   std::vector<geometry_msgs::Pose>& modified_path,
                   ExecutionPathType path_type = ExecutionPathType::kLocalPath);
  void setState(const geometry_msgs::Pose& pose);
  void setVelocity(double v);
  bool planAhead() { return ((planner_trigger_lead_time_ > 0) ? true : false); }

  bool reconnectPath(const std::vector<geometry_msgs::Pose>& path,
                     std::vector<geometry_msgs::Pose>& path_new);
  void allocateYawAlongPath(std::vector<geometry_msgs::Pose>& path) const;
  void allocateYawAlongFistSegment(
      std::vector<geometry_msgs::Pose>& path) const;

  double getVelocity(ExecutionPathType path_type) { return v_max_; }
  bool goToWaypoint(geometry_msgs::Pose& pose);

  void processActionFeedback(double estimated_time_left);
  void triggerPlanner();

 private:
  visualization_msgs::MarkerArray::Ptr generateTrajectoryMarkerArray(
      const std::vector<geometry_msgs::Pose>& traj) const;
  visualization_msgs::MarkerArray::Ptr generateTrajectoryMarkerArray(
      const trajectory_msgs::MultiDOFJointTrajectory& traj) const;

  RunModeType run_mode_;
  bool init_motion_enable_;

  ros::Publisher trajectory_pub_;
  ros::Publisher path_pub_;

  std::string world_frame_id_;
  double planner_trigger_lead_time_;
  bool smooth_heading_enable_;
  bool homing_yaw_allocation_enable_;

  trajectory_msgs::MultiDOFJointTrajectory samples_array_;
  mav_msgs::EigenTrajectoryPoint trajectory_point_;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg_;
  std::vector<geometry_msgs::Pose> executing_path_;

  actionlib::SimpleActionClient<planner_msgs::pathFollowerActionAction> ac_;
  const double kServerWatingTimeout = 1.0;

  bool actionDoneCallback(
      const actionlib::SimpleClientGoalState& state,
      const planner_msgs::pathFollowerActionResultConstPtr& result);
  void actionActiveCallback();
  void actionFeedbackCallback(
      const planner_msgs::pathFollowerActionFeedbackConstPtr& feedback);

  void executionTimerCallback(const ros::TimerEvent& event);
  ros::Timer execution_timer_;

  int n_seq_;

  bool use_action_client_path_manager_ = false;

  double v_max_;
  double v_init_max_;
  double v_homing_max_;
  double v_narrow_env_max_;
  double yaw_rate_max_;
  double dt_;
  const double kVelMax = 1.0;
  const double kVelMin = 0.2;
  bool finish_goal_;
  RobotType robot_type_;
  OutputType output_type_;
  bool concatenate_path_enable_ = true;

  double init_z_takeoff_;
  double init_z_drop_;
  double init_x_forward_;

  void interpolatePath(const std::vector<geometry_msgs::Pose>& path,
                       std::vector<geometry_msgs::Pose>& path_res);

  double getEndPointDistanceAlongPath(
      const std::vector<geometry_msgs::Pose>& path);
  double calculateDistance(const geometry_msgs::Pose& p1,
                           const geometry_msgs::Pose& p2);
};

}  // namespace explorer

#endif
