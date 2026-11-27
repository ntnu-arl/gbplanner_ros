#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include "planner_msgs/pathFollowerActionAction.h"

#include <vector>
#include <iostream>
#include <memory>
#include <algorithm>
#include <eigen3/Eigen/Dense>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_path_tracker");

	ros::NodeHandle nh;

  actionlib::SimpleActionClient<planner_msgs::pathFollowerActionAction> ac("smb_path_tracker", true);

	ros::Publisher path_pub = nh.advertise<geometry_msgs::PoseArray>("global_path", 100);

  ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	std::vector<geometry_msgs::PoseStamped> goal_path;
	geometry_msgs::PoseArray pa;
	pa.header.frame_id = "world";
	for(int i=0;i<15;++i)
	{
		geometry_msgs::PoseStamped p;
		// p.pose.position.x = 14*0.1 - 0.1*i;
		// p.pose.position.y = 196*0.1/5.0 - 0.1*i*i/5.0;
		p.pose.position.x = 0.1*i;
		p.pose.position.y = 0.1*i*i/5.0;
		p.pose.orientation.w = 1.0;
		goal_path.push_back(p);
		pa.poses.push_back(p.pose);
	}
	for(int i=0;i<15;++i)
	{
		geometry_msgs::PoseStamped p;
		p.pose.position.x = 14*0.1 - 0.1*i*i/5.0;
		p.pose.position.y = 196*0.1/5.0 - 0.1*i;
		// p.pose.position.x = 0.1*i;
		// p.pose.position.y = 0.1*i*i/5.0;
		p.pose.orientation.w = 1.0;
		goal_path.push_back(p);
		pa.poses.push_back(p.pose);
	}
	// std::vector<Eigen::Vector3d> waypoints;
	// Eigen::Vector3d p1(0.0,0.0,0.0);
	// waypoints.push_back(p1);
	// Eigen::Vector3d p2(2.0,5.0,0.0);
	// waypoints.push_back(p2);
	// Eigen::Vector3d p3(4.0,0.0,0.0);
	// waypoints.push_back(p3);
	// Eigen::Vector3d p4(4.0,2.0,0.0);
	// waypoints.push_back(p4);
	// Eigen::Vector3d p5(5.0,-5.0,0.0);
	// waypoints.push_back(p5);
	// for(int i=1;i<waypoints.size();++i)
	// {
	// 	Eigen::Vector3d segment = waypoints[i] - waypoints[i-1];
	// 	double seg_len = segment.norm();
	// 	Eigen::Vector3d seg_normed = segment.normalized();
	// 	for(double d=0.0;d<seg_len;d+=0.1)
	// 	{
	// 		Eigen::Vector3d pt = waypoints[i-1] + seg_normed * d;
	// 		geometry_msgs::PoseStamped ps;
	// 		ps.pose.position.x = pt(0);
	// 		ps.pose.position.y = pt(1);
	// 		ps.pose.orientation.z = 0.0;
	// 		goal_path.push_back(ps);
	// 		pa.poses.push_back(ps.pose);
	// 	}
	// }

	planner_msgs::pathFollowerActionGoal goal;

	path_pub.publish(pa);

	goal.path = goal_path;
	ac.sendGoal(goal);
}