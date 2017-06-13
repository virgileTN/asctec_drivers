#include "asctec_autopilot/circular_route.h"

#define MEAN_EARTH_DIAMETER 12756274
#define UMR 0.017453292519943295769236907684886
#define PI 3.1415926535897932384626433832795
#define INT 3000;

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

bool prox(asctec_msgs::waypoint waypoint){
	bool ret;	
	if (((waypoint.X-(INT*360*10000000)/(PI*MEAN_EARTH_DIAMETER)<=current.longitude)&&(current.longitude<=waypoint.X+(INT*360*10000000)/(PI*MEAN_EARTH_DIAMETER)))&&((waypoint.Y-(INT*360*10000000)/(PI*MEAN_EARTH_DIAMETER)<=current.latitude)&&(current.latitude<=waypoint.X+(INT*360*10000000)/(PI*MEAN_EARTH_DIAMETER)))){
		ret=true;
	}else{
		ret=false;
	}
	return ret;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_node");

	ros::NodeHandle n;
	
	//Déclaration des paramètres
	int radius;
	n.getParam("radius", radius);
	int lap_nb;
	n.getParam("lap_nb", lap_nb);
	int int_angle;
	n.getParam("int_angle", int_angle);
	
	int center_lat =0;
	int center_long=0;
	
	
	//Topic
	publisher = n.advertise<asctec_msgs::waypoint>("/asctec/WAYPOINT_INPUT6", 1);
	subscriber_gps = n.subscribe("/asctec/GPS_DATA6", 1, Callback, ros::TransportHints().tcpNoDelay());
	subscriber_ack = n.subscribe("/asctec/ACK3", 1, Callback3, ros::TransportHints().tcpNoDelay());

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
	string data;
    size_t n;

	//INPUT circle center
    /*Longitude*/
    cout << "longitude (10^-7 °) : ";
    getline (cin,data);

    n=data.find(' ');
    if(n != string::npos)
    {
      bool negative=(data[0]=='-');

      string temp=data.substr(0,n);
      data=data.substr(n+1);
      center_long=abs(strtodouble(temp))*10000000;
	
      n=data.find(' ');
      temp=data.substr(0,n);
      data=data.substr(n+1);
      center_long=center_long + strtodouble(temp)*10000000/60;

      center_long=center_long + strtodouble(data)*10000000/3600;
      if(negative) center_long=-center_long;
	  msg.X=center_long;
    }
    else
    {
      msg.X=strtodouble(data);
	  center_long=strtodouble(data);
    }

    ros::spinOnce();

    if(msg.X==0) msg.X=current.longitude;

    if(stop) return 0;

    if(msg.X!=1 && msg.X!=2 && msg.X!=3)
    {
      /*Latitude*/
      cout << "latitude (10^-7 °) : ";
      getline (cin,data);

      n=data.find(' ');
      if(n != string::npos)
      {
        string temp=data.substr(0,n);
        data=data.substr(n+1);
        center_lat=strtodouble(temp)*10000000;

        n=data.find(' ');
        temp=data.substr(0,n);
        data=data.substr(n+1);
        center_lat=center_lat + strtodouble(temp)*10000000/60;

        center_lat=center_lat + strtodouble(data)*10000000/3600;
		msg.Y=center_lat;
      }
      else
      {
        msg.Y=strtodouble(data);
		center_lat=strtodouble(data);
      }

      if(msg.Y==0) msg.Y=current.latitude;

      if(stop) return 0;

      msg.time=500;

      msg.pos_acc=2500;
	  
	  /*Height*/
      cout << "Height (millimeters) : ";
	  cin >> msg.height;
    }
    else
    {
      msg.Y=0;
      msg.height=0;
      msg.yaw=0;
      msg.time=0;
      msg.pos_acc=0;
    }

    msg.wp_number=1;
    msg.properties=0x11;
    msg.max_speed=100;

    msg.chksum = (short) 0xAAAA + msg.yaw + msg.height + msg.time + msg.X + msg.Y + msg.max_speed + msg.pos_acc + msg.properties + msg.wp_number;

    publisher.publish(msg);

    ros::Duration(0.5).sleep();
    ros::spinOnce();
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
	
//JOIN the 1st point of the circle
	/*Convert into GPS coordinates*/
	int theta =0;
	theta = (radius*360*10000000)/(PI*MEAN_EARTH_DIAMETER);
	msg.X = center_long;
	msg.Y = center_lat + theta;

	msg.chksum = (short) 0xAAAA + msg.yaw + msg.height + msg.time + msg.X + msg.Y + msg.max_speed + msg.pos_acc + msg.properties + msg.wp_number;

    publisher.publish(msg);

    ros::Duration(0.5).sleep();
    ros::spinOnce();
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
 
//CREATE the waypoint list
	/*Determination of n*/
	int m =0;
	m = (int) lap_nb*(360/int_angle);
	
	/*List*/
	asctec_msgs::waypoint waypoint_list[m];
	
	/*ADD values*/
	int i=1; 
	for(i=1; i<m; i++){
		waypoint_list[i].X = center_long + radius * cos((PI/2)-(i*int_angle *(PI/180));
		waypoint_list[i].Y = center_lat + radius * sin((PI/2)-(i*int_angle *(PI/180));
	}
	
//Follow the route
	int timer=1;
	while(timer<m){
		
		/*Initialize the loop*/
		msg.X = waypoint_list[1].X;
		msg.Y = waypoint_list[1].Y;
		msg.chksum = (short) 0xAAAA + msg.yaw + msg.height + msg.time + msg.X + msg.Y + msg.max_speed + msg.pos_acc + msg.properties + msg.wp_number;

		publisher.publish(msg);

		ros::Duration(0.5).sleep();
		ros::spinOnce();
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
		
		/*loop*/
		if(prox(waypoint_list[timer]){
			timer++;
			msg.X = waypoint_list[timer].X;
			msg.Y = waypoint_list[timer].Y;
			msg.chksum = (short) 0xAAAA + msg.yaw + msg.height + msg.time + msg.X + msg.Y + msg.max_speed + msg.pos_acc + msg.properties + msg.wp_number;

			publisher.publish(msg);

			ros::Duration(0.5).sleep();
			ros::spinOnce();
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
		}
	}
	/*Return to the center of the circle*/
	msg.X=center_long;
	msg.Y=center_lat;
	msg.chksum = (short) 0xAAAA + msg.yaw + msg.height + msg.time + msg.X + msg.Y + msg.max_speed + msg.pos_acc + msg.properties + msg.wp_number;

	publisher.publish(msg);

	ros::Duration(0.5).sleep();
	ros::spinOnce();
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
	stop=true;
  }
}	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
