#include "UartVoltageLog.hpp"

vector<double> read_currentArray(){

	UART u(
	DEV,
	BAUD_RATE
	);
	
	sample_t sample_buffer[N_SAMPLES];

	u.read(sample_buffer);

	vector<double> current_values;

	for(int i = 0; i < N_SAMPLES; i++){
		current_values.push_back((sample_buffer[i] * 5) / 1024.0);
	}

	return current_values;
}

vector<double> read_currentMean(){

	UART u(
	DEV,
	BAUD_RATE
	);
	
	sample_t sample_buffer[N_SAMPLES];

	u.read(sample_buffer);

	vector<double> current_values;
	vector<double> value;


	for(int i = 0; i < N_SAMPLES; i++){
		current_values.push_back((sample_buffer[i] * 5) / 1024.0);
	}

	value.push_back(0);

	for(int i = 0; i < N_SAMPLES; i++){
		value[0] += current_values[i];
	}

	value[0] /= N_SAMPLES;

	return value;
}

//Use for Ros Timer
/*
 void callback(const ros::TimerEvent& event, std_msgs::Float64 msg, ros::Publisher pub){

	msg.data = sample_buffer[position];
	position++;
	pub.publish(msg);
	
	if(position == N_SAMPLES){
		position = 0;
		u.read(sample_buffer);
	}
 }
*/
