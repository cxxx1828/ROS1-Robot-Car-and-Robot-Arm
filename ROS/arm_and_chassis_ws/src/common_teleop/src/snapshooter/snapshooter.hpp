
#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>

#include <string>
#include <vector>
using namespace std;

class snapshooter {
public:
	snapshooter(ros::NodeHandle& nh);
	~snapshooter();
	
private:
	ros::NodeHandle& nh;
	
	string log_dir;
	
	bool log_en;
	ros::Subscriber snapshoot_sub;
	void snapshoot_cb(const std_msgs::Bool::ConstPtr& msg);
	ros::Timer log_period_timer;
	void log_period_cb(const ros::TimerEvent& e);
	
	ros::Subscriber joint_sub;
	void joint_cb(const sensor_msgs::JointState::ConstPtr& msg);
	ros::Subscriber pose_sub;
	void pose_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
	ros::Subscriber joy_sub;
	void joy_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
	
};
