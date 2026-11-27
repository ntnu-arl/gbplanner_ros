#include "smb_path_tracker/pure_pursuit.h"


PurePursuitController::PurePursuitController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
	:nh_(nh), nh_private_(nh_private), 
	as_(nh_, "pci_output_path", boost::bind(&PurePursuitController::actionGoalCallback, this, _1), false)
{
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	carrot_point_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("carrot_point", 10);
	odom_sub_ = nh_.subscribe("odometry", 100, &PurePursuitController::odomCallback, this);
	loadParams();
	path_to_exec_.clear();

	listener_ = std::make_shared<tf::TransformListener>();

	as_.start();

	// control_timer_ = nh_.createTimer(ros::Duration(dt_), &PurePursuitController::controlLoop, this);
}

double PurePursuitController::calculateDistance(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2)
{
	Eigen::Vector3d v1(p1.position.x, p1.position.y, p1.position.z);
	Eigen::Vector3d v2(p2.position.x, p2.position.y, p2.position.z);

	return (v1 - v2).head(2).norm();
}

geometry_msgs::Pose PurePursuitController::convertEigenVecToPose(const Eigen::Vector3d &vec)
{
	geometry_msgs::Pose p;
	p.position.x = vec(0);
	p.position.y = vec(1);
	p.position.z = vec(2);
	p.orientation.w = 1.0;

	return p;
}

double PurePursuitController::calculatePathLength(const std::vector<geometry_msgs::PoseStamped> &path)
{
	double dist = 0.0;
	for(int i=1;i<path.size();++i)
	{
		dist += calculateDistance(path[i].pose, path[i-1].pose);
	}
	return dist;
}

void PurePursuitController::odomCallback(const nav_msgs::Odometry &odom)
{
	current_pose_ = odom.pose.pose;
	current_vel_ = odom.twist.twist;
}

void PurePursuitController::actionGoalCallback(const planner_msgs::pathFollowerActionGoalConstPtr &goal)
{
	path_to_exec_.clear();
	path_to_exec_ = goal->path;
	start_execution_ = true;

	ros::Rate rate(1.0/dt_);

	while(ros::ok())
	{
		// Check if the action is preempted
		if(as_.isPreemptRequested())
		{
			ROS_WARN("SMB Path Tracker: Path tracking preempted, stopping");
			// Publish 0 velocity
			geometry_msgs::Twist vel;
			vel.linear.x = 0.0;
			vel.angular.z = 0.0;
			cmd_vel_ = vel;
			cmd_vel_pub_.publish(cmd_vel_);

			path_to_exec_.clear();

			as_.setPreempted();

			return;
		}

		// Check if the path to execute is empty
		// std::cout << path_to_exec_.size() << std::endl;
		if(path_to_exec_.size() <= 0)
		{
			ROS_WARN("SMB Path Tracker: Commanded path is empty, stopping");
			// Publish 0 velocity
			geometry_msgs::Twist vel;
			vel.linear.x = 0.0;
			vel.angular.z = 0.0;
			cmd_vel_ = vel;
			cmd_vel_pub_.publish(cmd_vel_);

			result_.success = false;
			as_.setSucceeded(result_);

			return;
		}

		if(calculateDistance(path_to_exec_.back().pose, current_pose_) < goal_reaching_threshold_)
		{
			ROS_INFO("SMB Path Tracker: Reached end of path");
			result_.success = true;
			as_.setSucceeded(result_);

			start_execution_ = false;

			return;
		}

		if(computeCommandVelocity())
		{
			// std::cout << "cmd vel: " << cmd_vel_.linear.x << ", " << cmd_vel_.angular.z << std::endl;
			cmd_vel_pub_.publish(cmd_vel_);
		}
		else
		{
			ROS_WARN("SMB Path Tracker: Can't track given path");
		}
		feedback_.dist_to_goal = calculateDistance(path_to_exec_.back().pose, current_pose_);
		feedback_.remaining_waypoints = (int)path_to_exec_.size();
		feedback_.estimated_time_remaining = calculatePathLength(path_to_exec_) / desired_vel_;  // Very crude estimate
		as_.publishFeedback(feedback_);
		rate.sleep();
		ros::spinOnce();
	}
}

bool PurePursuitController::computeCommandVelocity()
{
	// Find closest point on the path
	std::vector<double> dists;
	for(int i=0;i<path_to_exec_.size();++i)
	{
		dists.push_back(calculateDistance(current_pose_, path_to_exec_[i].pose));
	}
	int closest_waypoint_ind = std::min_element(dists.begin(),dists.end()) - dists.begin();
	// std::cout << "closest waypoint ind: " << closest_waypoint_ind << std::endl;

	// Prune path to the closest waypoint
	std::vector<geometry_msgs::PoseStamped> tmp_path = path_to_exec_;
	path_to_exec_.clear();
	for(int i=closest_waypoint_ind;i<tmp_path.size();++i)
	{
		path_to_exec_.push_back(tmp_path[i]);
	}

	// Find carrot waypoint
	int carrot_point_ind = path_to_exec_.size()-1;
	for(int i=0;i<path_to_exec_.size();++i)
	{
		if(calculateDistance(current_pose_, path_to_exec_[i].pose) >= look_ahead_distance_)
		{
			carrot_point_ind = i;
			break;
		}
	}

	// Carrot waypoint to follow
	geometry_msgs::Pose carrot_waypoint = path_to_exec_[carrot_point_ind].pose;

	if(calculateDistance(current_pose_, path_to_exec_[carrot_point_ind].pose) >= (look_ahead_distance_ + look_ahead_error_margin_) 
		 && carrot_point_ind != 0)
	{
		// Carrot point is further away than the look ahead distance
		// and it is not the beginning of the path
		Eigen::Vector3d v2(path_to_exec_[carrot_point_ind].pose.position.x,
											path_to_exec_[carrot_point_ind].pose.position.y,
											path_to_exec_[carrot_point_ind].pose.position.z);
		Eigen::Vector3d v1(path_to_exec_[carrot_point_ind-1].pose.position.x,
											path_to_exec_[carrot_point_ind-1].pose.position.y,
											path_to_exec_[carrot_point_ind-1].pose.position.z);
		Eigen::Vector3d carrot_point_vec;
		Eigen::Vector3d dir_vec = v2 - v1;
		for(double d=0.0;d<dir_vec.norm();d+=look_ahead_error_margin_)
		{
			Eigen::Vector3d v = v1 + (v2 - v1).normalized() * d;
			Eigen::Vector3d cv(current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
			if((v - cv).norm() >= look_ahead_distance_)
			{
				carrot_point_vec = v;
				break;
			}
		}
		// carrot_point_vec = look_ahead_distance_ * (v2 - v1).normalized() + v1;
		tf::Quaternion quat;
		quat.setEuler(0.0, 0.0, std::atan2(dir_vec(1), dir_vec(0)));
		tf::Vector3 origin(carrot_point_vec(0), carrot_point_vec(1), carrot_point_vec(2));
		tf::Pose poseTF(quat, origin);
		tf::poseTFToMsg(poseTF, carrot_waypoint);
	}

	tmp_path = path_to_exec_;
	path_to_exec_.clear();
	geometry_msgs::PoseStamped carrot_waypoint_stamped;
	carrot_waypoint_stamped.pose = carrot_waypoint;
	path_to_exec_.push_back(carrot_waypoint_stamped);
	for(int ind=carrot_point_ind;ind<tmp_path.size();++ind)
	{
		path_to_exec_.push_back(tmp_path[ind]);
	}

	// Transform carrot point into robot frame for easier calculations:
	tf::StampedTransform fixed_to_robot_transform;
	try {
    listener_->lookupTransform(robot_frame_, fixed_frame_, ros::Time(0), fixed_to_robot_transform);
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    return false;
  }
	geometry_msgs::Pose carrot_waypoint_robot;
	tf::Vector3 carrot_point_tf_fixed;
	tf::pointMsgToTF(carrot_waypoint.position, carrot_point_tf_fixed);
	tf::Vector3 carrot_point_tf_robot = fixed_to_robot_transform * carrot_point_tf_fixed;
	tf::pointTFToMsg(carrot_point_tf_robot, carrot_waypoint_robot.position);

	geometry_msgs::PoseStamped ps;
	ps.header.frame_id = robot_frame_;
	ps.pose = carrot_waypoint_robot;

	carrot_point_pub_.publish(ps);

	Eigen::Vector3d carrot_vec_robot;
	carrot_vec_robot << carrot_waypoint_robot.position.x, carrot_waypoint_robot.position.y, carrot_waypoint_robot.position.z;

	int lin_vel_sign = (carrot_waypoint_robot.position.x >= 0) ? 1 : -1;
	double delta_theta = std::atan2(carrot_waypoint_robot.position.y, carrot_waypoint_robot.position.x);
	if(lin_vel_sign < 0)
	{
		if(delta_theta > 0)
		{
			delta_theta -= M_PI;
		}
		else
		{
			delta_theta += M_PI;
		}
	}
	int ang_vel_sign = (delta_theta >= 0) ? 1 : -1;

	double radius = std::pow(carrot_vec_robot.head(2).norm(), 2) / (2.0 * (std::abs(carrot_waypoint_robot.position.y) + 0.01));
	double radius_min = carrot_vec_robot.head(2).norm() / (2.0 * std::sin(curvature_scaling_angle_));
	double radius_max = carrot_vec_robot.head(2).norm() / (2.0 * std::sin(10.0 * M_PI / 180.0));

	if(delta_theta >= critical_angle_)
	{
		// std::cout << "Case 1: 0 lin vel" << std::endl;
		cmd_vel_.linear.x = 0.0;
		cmd_vel_.angular.z = max_ang_vel_ * ang_vel_sign;
		return true;
	}
	else if(delta_theta >= curvature_scaling_angle_)
	{
		// std::cout << "Case 2: curvature scaled lin vel" << std::endl;
		

		// double max_feasible_speed = std::min(current_vel_.linear.x + max_lin_acc_ * dt_, max_lin_vel_);
		// double min_feasible_speed = std::max(current_vel_.linear.x - max_lin_dec_ * dt_, -max_lin_vel_);

		cmd_vel_.linear.x = lin_vel_sign * desired_vel_ * (radius / radius_min);
		cmd_vel_.angular.z = ang_vel_sign * std::abs(cmd_vel_.linear.x) / radius;
		if(std::abs(cmd_vel_.angular.z) > max_ang_vel_)
		{
			cmd_vel_.angular.z = ang_vel_sign * max_ang_vel_;
		}
		return true;
	}
	else
	{
		// std::cout << "Case 3: full lin vel" << std::endl;
		// std::cout << "radius: " << radius << " min radius: " << radius_min << std::endl;
		// double max_feasible_speed = std::min(current_vel_.linear.x + max_lin_acc_ * dt_, max_lin_vel_);
		// double min_feasible_speed = std::max(current_vel_.linear.x - max_lin_dec_ * dt_, -max_lin_vel_);
		double r = std::min(radius_max, radius);
		cmd_vel_.linear.x = lin_vel_sign * desired_vel_;
		cmd_vel_.angular.z = ang_vel_sign * std::abs(cmd_vel_.linear.x) / radius;
		if(std::abs(cmd_vel_.angular.z) > max_ang_vel_)
		{
			cmd_vel_.angular.z = ang_vel_sign * max_ang_vel_;
		}
		cmd_vel_.linear.x *= ((r - radius_min)/(radius_max - radius_min));
		return true;
	}

}

void PurePursuitController::loadParams()
{
	std::string ns = ros::this_node::getName();
  std::string param_name;

  param_name = ns + "/critical_angle";
  if (!ros::param::get(param_name, critical_angle_)) {
    critical_angle_ = 45.0 * M_PI / 180.0;
  }

	param_name = ns + "/curvature_scaling_angle";
  if (!ros::param::get(param_name, curvature_scaling_angle_)) {
    curvature_scaling_angle_ = critical_angle_;
  }

	param_name = ns + "/dt";
  if (!ros::param::get(param_name, dt_)) {
    dt_ = 0.05;
  }

	param_name = ns + "/desired_vel";
  if (!ros::param::get(param_name, desired_vel_)) {
    desired_vel_ = 0.5;
  }

	param_name = ns + "/max_lin_vel";
  if (!ros::param::get(param_name, max_lin_vel_)) {
    max_lin_vel_ = 0.5;
  }

	param_name = ns + "/min_lin_vel";
  if (!ros::param::get(param_name, min_lin_vel_)) {
    min_lin_vel_ = 0.0;
  }

	param_name = ns + "/max_ang_vel";
  if (!ros::param::get(param_name, max_ang_vel_)) {
    max_ang_vel_ = 0.75;
  }

	param_name = ns + "/min_ang_vel";
  if (!ros::param::get(param_name, min_ang_vel_)) {
    min_ang_vel_ = 0.0;
  }

	param_name = ns + "/max_lin_acc";
  if (!ros::param::get(param_name, max_lin_acc_)) {
    max_lin_acc_ = std::numeric_limits<double>::max();
  }

	param_name = ns + "/max_lin_dec";
  if (!ros::param::get(param_name, max_lin_dec_)) {
    max_lin_dec_ = std::numeric_limits<double>::max();
  }

	param_name = ns + "/look_ahead_distance";
  if (!ros::param::get(param_name, look_ahead_distance_)) {
    look_ahead_distance_ = 1.0;
  }

	param_name = ns + "/look_ahead_error_margin";
  if (!ros::param::get(param_name, look_ahead_error_margin_)) {
    look_ahead_error_margin_ = 0.25;
  }

	param_name = ns + "/goal_reaching_threshold";
  if (!ros::param::get(param_name, goal_reaching_threshold_)) {
    goal_reaching_threshold_ = 0.5;
  }

	param_name = ns + "/fixed_frame";
  if (!ros::param::get(param_name, fixed_frame_)) {
    fixed_frame_ = "world";
  }

	param_name = ns + "/robot_frame";
  if (!ros::param::get(param_name, robot_frame_)) {
    robot_frame_ = "base_link";
  }

	
}