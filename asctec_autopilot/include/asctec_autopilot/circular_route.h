#ifndef CIRCULAR_ROUTE_H
#define CIRCULAR_ROUTE_H
#include "ros/ros.h"
#include <iostream>
#include <string>
#include <math.h>
#include <signal.h>
#include "asctec_msgs/GPSData.h"
#include "asctec_msgs/waypoint.h"
#include "asctec_msgs/ACK.h"

struct coordinates
{
  //latitude/longitude in deg * 10^7
  int latitude;
  int longitude;
};
struct coordinates home, current;

bool stop;
bool msg_ack;
bool ack_received;
char code;

ros::Publisher publisher;
ros::Subscriber subscriber_gps;
ros::Subscriber subscriber_ack;
//ros::Subscriber subscriber_cw;

void Callback(asctec_msgs::GPSData msg);
void Callback3(asctec_msgs::ACK msg);
void ctrlc_gestion(int signal);
bool prox(asctec_msgs::waypoint wpoint);

#endif
