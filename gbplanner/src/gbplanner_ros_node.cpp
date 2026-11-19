#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros1_compat/node_handle.h>
#include <planner_msgs/ros2_service_traits.hpp>

#include "gbplanner/gbplanner_ros.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  // gflags would reject the ROS 2 --ros-args arguments; no gflags are used.
  // google::ParseCommandLineFlags(&argc, &argv, false);

  ros1_compat::init(argc, argv, "gbplanner_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  GbplannerRos planner(nh, nh_private);

  ros::spin();

  return 0;
}
