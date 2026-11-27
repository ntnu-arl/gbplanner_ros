#include <ros/ros.h>
#include "smb_path_tracker/pure_pursuit.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "smb_path_tracker_node");

	ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

	PurePursuitController ppc(nh, nh_private);

	ros::spin();
}