#ifndef ASCTEC_AUTOPILOT_DRONE_FORMATION4_SANS_WP_INTER_H
#define ASCTEC_AUTOPILOT_DRONE_FORMATION4_SANS_WP_INTER_H

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <math.h>
#include <signal.h>
#include "asctec_msgs/GPSData.h"
#include "asctec_msgs/IMUCalcData.h"
#include "asctec_msgs/waypoint.h"
#include "asctec_msgs/ACK.h"
#include "asctec_msgs/CurrentWay.h"

#define MEAN_EARTH_DIAMETER	12756274.0
#define UMR	0.017453292519943295769236907684886		//PI/180
#define PI 3.1415926535897932384626433832795

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

short nav_stat, nav_stat2, nav_stat3, nav_stat4;

ros::Publisher publisher, publisher2, publisher3, publisher4, publisher_tmp;
ros::Subscriber subscriber_gps;
ros::Subscriber subscriber_imu;
ros::Subscriber subscriber_ack;
ros::Subscriber subscriber_cw, subscriber_cw2, subscriber_cw3, subscriber_cw4;

void Callback(asctec_msgs::GPSData msg);
void Callback2(asctec_msgs::IMUCalcData msg);
void Callback3(asctec_msgs::ACK msg);
void Callback_cw(asctec_msgs::CurrentWay msg);
void Callback_cw2(asctec_msgs::CurrentWay msg);
void Callback_cw3(asctec_msgs::CurrentWay msg);
void Callback_cw4(asctec_msgs::CurrentWay msg);
void ctrlc_gestion(int signal);
double strtodouble(std::string str);
void xy2latlon(double lat0, double lon0, double X, double Y, double *lat, double *lon);

#endif
