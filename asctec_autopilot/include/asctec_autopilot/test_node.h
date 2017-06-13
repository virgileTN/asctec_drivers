#ifndef ASCTEC_AUTOPILOT_TEST_NODE_H
#define ASCTEC_AUTOPILOT_TEST_NODE_H

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <math.h>
#include <signal.h>
#include "asctec_msgs/GPSData.h"
#include "asctec_msgs/IMUCalcData.h"
#include "asctec_msgs/waypoint.h"
#include "asctec_msgs/ACK.h"
//#include "asctec_msgs/CurrentWay.h"

struct coordinates
{
  //latitude/longitude in deg * 10ˆ7
  int latitude;
  int longitude;
  //compass reading: angle reference for angle_yaw: 0..360000; 1000 = 1 degree
  int mag_heading;
  //height in mm (after data fusion)
  int height;
};
struct coordinates current, home;

bool stop;
bool msg_ack;
bool ack_received;
char code;

ros::Publisher publisher;
ros::Subscriber subscriber_gps;
ros::Subscriber subscriber_imu;
ros::Subscriber subscriber_ack;

void Callback(asctec_msgs::GPSData msg);
void Callback2(asctec_msgs::IMUCalcData msg);
void Callback3(asctec_msgs::ACK msg);
void ctrlc_gestion(int signal);

#endif
