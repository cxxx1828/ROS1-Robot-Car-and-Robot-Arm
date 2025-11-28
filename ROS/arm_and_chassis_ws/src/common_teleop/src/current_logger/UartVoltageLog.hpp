#pragma once

///////////
// Config.
#include "config.h"
///////////

#include "UART.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <bitset>
#include <vector>
#include "tf/transform_listener.h"
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include "exec.hpp"

using namespace std;

#define DEBUG(var) do { cout << #var << " = " << var << endl; } while(false)

#define N_SAMPLES (1 << LOG2_SAMPLES)

typedef uint16_t sample_t;

vector<double> read_currentArray();

vector<double> read_currentMean();

void callback(const ros::TimerEvent&, std_msgs::Float64, ros::Publisher);
