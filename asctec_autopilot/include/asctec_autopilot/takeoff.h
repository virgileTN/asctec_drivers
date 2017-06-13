#ifndef ASCTEC_TAKEOFF_H
#define ASCTEC_TAKEOFF_H

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <math.h>
#include <signal.h>

#include "asctec_msgs/ControllerOutput.h"
#include "asctec_msgs/CtrlInput.h"
#include "asctec_msgs/IMUCalcData.h"

bool stop;

ros::Publisher publisher;
ros::Subscriber subscriber_thrust;
ros::Subscriber subscriber_IMU;

void Callback(asctec_msgs::ControllerOutput msg);
void Callback3(asctec_msgs::IMUCalcData msg);
void ctrlc_gestion(int signal);

#endif
