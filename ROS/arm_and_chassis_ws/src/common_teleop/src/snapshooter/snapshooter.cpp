
#include "snapshooter.hpp"

#define LOOP_HZ 25

#define DEBUG(x) do{ ROS_INFO_STREAM(#x << " = " << x); }while(0)

#include <filesystem>
namespace fs = std::filesystem;
#include <fstream>
using namespace std;

snapshooter::snapshooter(ros::NodeHandle& nh) :
	nh(nh),
	log_en(false)
{
	
	ros::param::param(
		"~log_dir",
		log_dir,
		string("~/.ros/")
	);
	
	//fs::create_directories(log_dir);TODO Does not exists on RPi
	
	snapshoot_sub = nh.subscribe(
		"snapshoot",
		1,
		&snapshooter::snapshoot_cb,
		this
	);
	log_period_timer = nh.createTimer(
		// Little more than usual period,
		// because could have hole between snapshoot_cb() and log_period_cb().
		//ros::Duration(1.0/LOOP_HZ)*1.1,
		ros::Duration(1.0/LOOP_HZ),
		&snapshooter::log_period_cb,
		this,
		true // oneshot
	);
	
	joint_sub = nh.subscribe(
		"joint",
		1,
		&snapshooter::joint_cb,
		this
	);
	pose_sub = nh.subscribe(
		"pose",
		1,
		&snapshooter::pose_cb,
		this
	);
	joy_sub = nh.subscribe(
		"joy",
		1,
		&snapshooter::joy_cb,
		this
	);
}

snapshooter::~snapshooter() {
}

void snapshooter::snapshoot_cb(const std_msgs::Bool::ConstPtr& msg) {
	if(msg->data){
		// Start period passed.
		log_en = true;
		log_period_timer.stop();
		log_period_timer.start();
	}
}

void snapshooter::log_period_cb(const ros::TimerEvent& e) {
	// Period passed.
	log_en = false;
	
	//TODO save to file.
}

void snapshooter::joint_cb(
	const sensor_msgs::JointState::ConstPtr& msg
) {
	if(log_en){
		//TODO this is hack.
		ofstream ofs;
		string fn = log_dir + "/for_arm_routines.tsv";
		ofs.open(fn, std::ios_base::app);
		ofs << msg->header.stamp;
		ofs << '\t';
		ofs << " = [";
		//int n = msg->position.size();
		int n = 4; //DOF TODO
		for(int i = 0; i < n; i++){
			if(i != 0){
				ofs << " ";
			}
			ofs << msg->position[i];
		}
		ofs << "]";
		ofs << endl;
		ofs.close();
	}
}
void snapshooter::pose_cb(
	const geometry_msgs::TwistStamped::ConstPtr& msg
) {
	//TODO
}
void snapshooter::joy_cb(
	const geometry_msgs::TwistStamped::ConstPtr& msg
) {
	//TODO
}
