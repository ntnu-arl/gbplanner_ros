#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

class smb_teleop_twist_joy
{
public:
	smb_teleop_twist_joy()
	{
		cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		freeze_pub_ = n_.advertise<std_msgs::Empty>("soft_emergency_stop", 1);
		twist_mux_e_stop_pub_ = n_.advertise<std_msgs::Bool>("cmd_vel_e_stop", 1);
		joy_sub_ = n_.subscribe("joy", 1, &smb_teleop_twist_joy::joyCb, this);
		tim_ = n_.createTimer(ros::Duration(0.1), &smb_teleop_twist_joy::timerCb, this);
		max_velocity_angular = n_.param("max_velocity_angular", 5);
		max_velocity_linear = n_.param("max_velocity_linear", 5);
		twist_mux_publish = true;
	}

	void joyCb(const sensor_msgs::Joy &msg)
	{
		if (msg.buttons[5])
		{ // Check the button number
			// Freeze
			std_msgs::Empty e_msg;
			freeze_pub_.publish(e_msg);
			output.linear.x = 0;
			output.angular.z = 0;
		}
		else if (msg.buttons[4])
		{
			std_msgs::Bool e_msg;
			twist_mux_publish = !twist_mux_publish;
			e_msg.data = twist_mux_publish;
			if (twist_mux_publish)
			{
				ROS_WARN("twist_mux stopped");
			}
			else
			{
				ROS_WARN("twist_mux started");
			}
			twist_mux_e_stop_pub_.publish(e_msg);
		}
		else
		{
			output.linear.x = msg.axes[3] * max_velocity_linear;
			output.angular.z = msg.axes[2] * max_velocity_angular;
		}
		cmd_vel_pub_.publish(output);
	}

	void timerCb(const ros::TimerEvent &event)
	{
		cmd_vel_pub_.publish(output);
	}

private:
	ros::NodeHandle n_;
	ros::Publisher cmd_vel_pub_;
	ros::Publisher freeze_pub_;
	ros::Publisher twist_mux_e_stop_pub_;
	ros::Subscriber joy_sub_;
	ros::Timer tim_;

	float max_velocity_linear;
	float max_velocity_angular;

	bool twist_mux_publish;

	geometry_msgs::Twist output;

}; // class

int main(int argc, char **argv)
{
	ros::init(argc, argv, "smb_teleop_twist_joy_node");
	smb_teleop_twist_joy ob;
	ros::spin();
	return 0;
}