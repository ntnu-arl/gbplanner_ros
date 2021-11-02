#ifndef PCI_MANAGER_H_
#define PCI_MANAGER_H_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace explorer {

enum Verbosity { SILENT = 0, PLANNER_STATUS = 1, ERROR = 2, WARN = 3, INFO = 4, DEBUG = 5 };

#define global_verbosity Verbosity::ERROR
#define param_verbosity Verbosity::SILENT
class PCIManager {
 public:
  PCIManager(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : nh_(nh),
        nh_private_(nh_private),
        pci_status_(PCIStatus::kReady),
        force_stop_(false) {
    trajectory_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "pci_command_trajectory_vis", 10);
  }

  enum struct PCIStatus { kReady = 0, kRunning = 1, kError = 2 };

  enum struct RunModeType {
    kSim = 0,  // Run in simulation.
    kReal,     // Run with real robot.
  };

  enum struct ExecutionPathType {
    kLocalPath = 0,
    kHomingPath = 1,
    kGlobalPath = 2,
    kNarrowEnvPath =
        3,           // For special case, to slow down the copter in narrow env.
    kManualPath = 4  // Manually set path.
  };

  enum struct RobotType { kAerial = 0, kGround };

  virtual bool loadParams(const std::string ns) = 0;
  virtual bool initialize() = 0;
  virtual bool initMotion() = 0;

  virtual bool goToWaypoint(geometry_msgs::Pose& pose) = 0;

  // Send a path to be executed by the robot, return a new path if the
  // PCI modified the original path based on robot's dynamics.
  // is_global_path: indicate this is local or global path.
  virtual bool executePath(
      const std::vector<geometry_msgs::Pose>& path,
      std::vector<geometry_msgs::Pose>& modified_path,
      ExecutionPathType path_type = ExecutionPathType::kLocalPath) = 0;

  // Set the current state of the robot based on odometry or pose.
  virtual void setState(const geometry_msgs::Pose& pose) = 0;

  // Set max linear velocity allowed.
  virtual void setVelocity(double v) = 0;

  // Check if we should trigger planner in advance.
  virtual bool planAhead() = 0;

  // Smoothly allocate yaw angle along the whole path to prevent sudden change
  // in yaw. Fix the heading from the root node, relax them over time.
  virtual void allocateYawAlongPath(
      std::vector<geometry_msgs::Pose>& path) const = 0;
  // Fix the commanded heading to match the first segment.
  virtual void allocateYawAlongFistSegment(
      std::vector<geometry_msgs::Pose>& path) const = 0;

  virtual double getVelocity(ExecutionPathType path_type) = 0;

  // Check if the PCI is ready to use.
  bool isReady() { return (pci_status_ == PCIStatus::kReady); }

  const PCIStatus& getStatus() const { return pci_status_; }
  void setStatus(const PCIStatus& status) { pci_status_ = status; }
  const geometry_msgs::Pose& getState() { return current_pose_; }

  void stopPCI() { force_stop_ = true; }

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher trajectory_vis_pub_;

  PCIStatus pci_status_;
  geometry_msgs::Pose current_pose_;

  bool force_stop_;
};

}  // namespace explorer

#endif
