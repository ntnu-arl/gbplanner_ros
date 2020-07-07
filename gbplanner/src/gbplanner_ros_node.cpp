#include <ros/ros.h>

#include "gbplanner/gbplanner.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "gbplanner_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  explorer::Gbplanner planner(nh, nh_private);

  ros::spin();
  return 0;
}
