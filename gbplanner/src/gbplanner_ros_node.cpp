#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include "gbplanner/gbplanner_ros.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  ros::init(argc, argv, "gbplanner_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  GbplannerRos planner(nh, nh_private);

  ros::spin();

  return 0;
}
