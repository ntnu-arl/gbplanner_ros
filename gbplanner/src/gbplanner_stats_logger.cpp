#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <time.h>
#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

#include "std_msgs/Int32MultiArray.h"

ros::Publisher cloud_sizes_pub_;
int occ_cloud_size, com_cloud_size;
bool occ_cloud_set = false, com_cloud_set = false;

void occ_pcl_cb(const sensor_msgs::PointCloud2ConstPtr & input)
{
  // std::cout << "Occ cloud received" << std::endl;
  // Convert ROS msg to pcl data structure and store it in global variable
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud);
  if (cloud->points.empty()) return;

  occ_cloud_size = cloud->points.size();
  occ_cloud_set = true;
}

void com_pcl_cb(const sensor_msgs::PointCloud2ConstPtr & input)
{
  // std::cout << "complete cloud received" << std::endl;
  // Convert ROS msg to pcl data structure and store it in global variable
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud);
  if (cloud->points.empty()) return;

  com_cloud_size = cloud->points.size();
  com_cloud_set = true;
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "sparsify");  // Initialise the node. Node name = talker
  ros::NodeHandle nh;

  cloud_sizes_pub_ = nh.advertise<std_msgs::Int32MultiArray>("cloud_sizes", 100);
  ros::Subscriber occ_pcl_sub = nh.subscribe("occ_cloud", 1, occ_pcl_cb);
  ros::Subscriber com_pcl_sub = nh.subscribe("com_cloud", 1, com_pcl_cb);

  ros::Timer timer = nh.createTimer(ros::Duration(0.1), [&](ros::TimerEvent) {
    if (occ_cloud_set && com_cloud_set) {
      std_msgs::Int32MultiArray msg;
      msg.data.push_back(occ_cloud_size);
      msg.data.push_back(com_cloud_size);
      cloud_sizes_pub_.publish(msg);
      occ_cloud_set = false;
      com_cloud_set = false;
    }
  });

  ros::spin();
  return 0;
}