#include "asctec_autopilot/waypoint_test.h"

#define MEAN_EARTH_DIAMETER 12756274
#define UMR 0.017453292519943295769236907684886
#define PI 3.1415926535897932384626433832795

using namespace std;

void Callback(asctec_msgs::GPSData msg)
{
  current.latitude = msg.latitude;
  current.longitude = msg.longitude;
}

void Callback3(asctec_msgs::ACK msg)
{
	  ack_received = msg.ack_received;
	  code = msg.code;

	  msg_ack=true;
}

void ctrlc_gestion(int signal)
{
  stop=true;
  cin.putback(-1);
}

double strtodouble(string str)
{
  double result;
  
  istringstream iss;
  iss.str(str);
  iss >> result;

  return result;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_node");

	ros::NodeHandle n;
	//Déclaration des paramètres
	int w1_lat;
	int w1_long;
	n.getParam("w1_lat", w1_lat);
	n.getParam("w1_long", w1_long);

	int w2_lat;
	int w2_long;
	n.getParam("w2_lat", w2_lat);
	n.getParam("w2_long", w2_long);

	int w3_lat;
	int w3_long;
	n.getParam("w3_lat", w3_lat);
	n.getParam("w3_long", w3_long);

	
	
	//Topic
	publisher = n.advertise<asctec_msgs::waypoint>("/asctec/WAYPOINT_INPUT6", 1);
	subscriber_gps = n.subscribe("/asctec/GPS_DATA6", 1, Callback, ros::TransportHints().tcpNoDelay());
	subscriber_ack = n.subscribe("/asctec/ACK6", 1, Callback3, ros::TransportHints().tcpNoDelay());

	//Sigaction
	struct sigaction s_action;
	s_action.sa_handler = ctrlc_gestion;
	sigemptyset(&s_action.sa_mask);
	s_action.sa_flags = 0;
	sigaction(SIGINT, &s_action, NULL);

	stop=false;


	while (!stop)
	{
	asctec_msgs::waypoint msg;
	int data;
	size_t n;


	cout << "Choose waypoint (1..3) : ";
	cin>>data;



	if(data==1){
		msg.X= w1_long ;
		msg.Y= w1_lat;
	}else if(data==2){
		msg.X= w2_long;
		msg.Y= w2_lat;
	}else if(data==3){
		msg.X= w3_long;
		msg.Y= w3_lat;
	}else if(data==4){
		msg.X= current.longitude;
		msg.Y= current.latitude;
	}else{
		msg.X= current.longitude;
		msg.Y= current.latitude;
	}
	ros::spinOnce();

	if(stop) return 0;

	msg.wp_number=1;
	msg.properties=0x11;
	msg.max_speed=100;

	msg.chksum = (short) 0xAAAA + msg.yaw + msg.height + msg.time + msg.X + msg.Y + msg.max_speed + msg.pos_acc + msg.properties + msg.wp_number;

	publisher.publish(msg);

	ros::Duration(0.5).sleep();
	ros::spinOnce();

	//Check of waypoint transmission
	if(msg_ack==true)
	{
		if(ack_received)
		{
			cout << "ACK Received\n";
			if(code==38) cout << "Set home\n";
			else if(code==40) cout << "Go to home\n";
			     else if(code==39) cout << "Land here\n";
				   else if(code==36) cout << "Waypoint sent\n";
					else cout << "Incorrect ACK\n";
		}
	}
	else cout << "No ACK Received\n";
	msg_ack=false;
	ROS_ERROR("Waypoint: %d longitude ; %d latitude", msg.X, msg.Y);
	}
  return 0;
}
