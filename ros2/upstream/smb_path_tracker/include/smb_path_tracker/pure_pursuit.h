#pragma once

#include <vector>
#include <iostream>
#include <memory>
#include <algorithm>

#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include "planner_msgs/pathFollowerActionAction.h"

class PurePursuitController
{
private:
  /* Parameters: */
	double critical_angle_;
	double curvature_scaling_angle_;
	double dt_;
	double desired_vel_;
	double max_lin_vel_;
	double min_lin_vel_;
	double max_ang_vel_;
	double min_ang_vel_;
	double max_lin_acc_;
	double max_lin_dec_;
	double look_ahead_distance_;
	double look_ahead_error_margin_;
	double goal_reaching_threshold_;
	std::string fixed_frame_;
	std::string robot_frame_;

	std::vector<geometry_msgs::PoseStamped> path_to_exec_;
	geometry_msgs::Twist cmd_vel_;

	geometry_msgs::Pose current_pose_;
	geometry_msgs::Twist current_vel_;

	// State variables
	bool start_execution_ = false;

	ros::NodeHandle nh_, nh_private_;
	ros::Timer control_timer_;
	ros::Publisher cmd_vel_pub_;
	ros::Publisher carrot_point_pub_;
	ros::Subscriber odom_sub_;
	std::shared_ptr<tf::TransformListener> listener_;

	actionlib::SimpleActionServer<planner_msgs::pathFollowerActionAction> as_;
	planner_msgs::pathFollowerActionFeedback feedback_;
	planner_msgs::pathFollowerActionResult result_;

public:
  PurePursuitController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
	
	void actionGoalCallback(const planner_msgs::pathFollowerActionGoalConstPtr &goal);
	void odomCallback(const nav_msgs::Odometry &odom);

	bool computeCommandVelocity();
	
	void loadParams();

	double calculateDistance(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2);
	geometry_msgs::Pose convertEigenVecToPose(const Eigen::Vector3d &vec);
	double calculatePathLength(const std::vector<geometry_msgs::PoseStamped> &path);
};