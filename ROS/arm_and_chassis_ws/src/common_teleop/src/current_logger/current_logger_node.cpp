#include "UartVoltageLog.hpp"


int main(int argc, char** argv) {

	ros::init(argc, argv, "current_logger", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");

	std::string topic_name =
		"/current_logger/current";
	/*
	ros::Publisher pub = nh.advertise<std_msgs::Float64>(
		topic_name,
		1
	);
	*/
	ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>(
		topic_name,
		1
	);

	//std_msgs::Float64 msg;
	geometry_msgs::TwistStamped msg;

	sample_t sample_buffer[N_SAMPLES];

	UART u(
		DEV,
		BAUD_RATE
	);

	u.read(sample_buffer);

/*
	//ADD FOR LOGS!
	//Fills up logs too much after some time(that's why its disabled for now)
	ofstream current_log;

	string current_log_fn = exec("roslaunch-logs");
	current_log_fn.pop_back();
	current_log_fn += "/current.log";
	current_log.open(current_log_fn);

	if(current_log.is_open()){
		current_log << setprecision(10);
		current_log << "t" << '\t' << "current" << endl;

		for(int i = 0; i < N_SAMPLES; i++){
			const double T = 1.0/F_SMPL;
			double t = i*T;
			uint16_t sample = sample_buffer[i];
			current_log << t << '\t' << sample << endl;
		}
	}
	else{
		ROS_WARN(
			"Cannot open log file \"%s\"!",
			current_log_fn.c_str()
		);
	}
*/

	while(nh.ok()){

		for(int i = 0; i < N_SAMPLES; i++){
			//msg.data = (sample_buffer[i] * 5) / 1024.0;
			msg.twist.linear.x = (sample_buffer[i] * 5) / 1024.0;
			ros::Time begin = ros::Time::now();
			msg.header.stamp = begin;
			pub.publish(msg);
			ros::spinOnce();
			ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.001));

			const double T = 1.0/F_SMPL;
			double t = i*T;
			uint16_t sample = sample_buffer[i];
			//current_log << t << '\t' << sample << endl;
		}
		u.read(sample_buffer);
	}

	//current_log.close();

	return 0;
}
