
#include "snapshooter.hpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "snapshooter");
	ros::NodeHandle nh;

	snapshooter inst(nh);
	
	ros::spin();
	
	return 0;
}
