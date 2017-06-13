#include "asctec_autopilot/test_node2.h"

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

  publisher = n.advertise<asctec_msgs::waypoint>("/asctec/WAYPOINT_INPUT4", 1);
  subscriber_gps = n.subscribe("/asctec/GPS_DATA4", 1, Callback, ros::TransportHints().tcpNoDelay());
  subscriber_ack = n.subscribe("/asctec/ACK4", 1, Callback3, ros::TransportHints().tcpNoDelay());

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

    /*Longitude*/
    cout << "longitude (10^-7 °) : ";
    getline (cin,data);

    n=data.find(' ');
    if(n != string::npos)
    {
      bool negative=(data[0]=='-');

      string temp=data.substr(0,n);
      data=data.substr(n+1);
      msg.X=abs(strtodouble(temp))*10000000;

      n=data.find(' ');
      temp=data.substr(0,n);
      data=data.substr(n+1);
      msg.X=msg.X + strtodouble(temp)*10000000/60;

      msg.X=msg.X + strtodouble(data)*10000000/3600;
      if(negative) msg.X=-msg.X;
    }
    else
    {
      msg.X=strtodouble(data);
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
        msg.Y=strtodouble(temp)*10000000;

        n=data.find(' ');
        temp=data.substr(0,n);
        data=data.substr(n+1);
        msg.Y=msg.Y + strtodouble(temp)*10000000/60;

        msg.Y=msg.Y + strtodouble(data)*10000000/3600;
      }
      else
      {
        msg.Y=strtodouble(data);
      }

      if(msg.Y==0) msg.Y=current.latitude;

      if(stop) return 0;

      msg.time=500;

      msg.pos_acc=2500;
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
  }

  return 0;
}
