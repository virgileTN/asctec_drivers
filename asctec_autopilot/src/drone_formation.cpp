#include "asctec_autopilot/drone_formation.h"

using namespace std;

void Callback(asctec_msgs::GPSData msg)
{
  current.latitude = msg.latitude;
  current.longitude = msg.longitude;
}

void Callback2(asctec_msgs::IMUCalcData msg)
{
  current.height = msg.height;
  current.mag_heading = msg.mag_heading;
}

void Callback3(asctec_msgs::ACK msg)
{
  ack_received = msg.ack_received;
  code = msg.code;

  msg_ack=true;
}

void Callback_cw(asctec_msgs::CurrentWay msg)
{
  nav_stat = msg.navigation_status;
}

void Callback_cw2(asctec_msgs::CurrentWay msg)
{
  nav_stat2 = msg.navigation_status;
}

void Callback_cw3(asctec_msgs::CurrentWay msg)
{
  nav_stat3 = msg.navigation_status;
}

void ctrlc_gestion(int signal)
{
  pthread_cancel(thread);
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

void xy2latlon(double lat0, double lon0, double X, double Y, double *lat, double *lon)	//X: East, Y: North in m; lat0,lon0: Reference coordinates; lat,lon: current GPS measurement
{
  *lat=lat0+Y/MEAN_EARTH_DIAMETER*360./PI;
  *lon=lon0+X/MEAN_EARTH_DIAMETER*360./PI/cos(lat0*UMR);
}

void* offset(void* data)
{
  while(!stop && !stop2)
  {
    offset_();
  }

  return NULL;
}

bool offset_()
{
  string dat;

  if(stop || stop2) return false;

  cout << "modifier hauteur du drone (1 : principal, 2 : gauche, 3 : droite) : ";
  getline (cin,dat);
  drone=strtodouble(dat);

  if(stop || stop2 || (drone!=1 && drone!=2 && drone!=3)) return false;

  cout << "De combien (m) ? ";
  getline (cin,dat);

  switch(drone)
  {
    case 1:
    {
      offset1=offset1+1000*strtodouble(dat);
      new_offset=true;
      break;
    }
    case 2:
    {
      offset2=offset2+1000*strtodouble(dat);
      new_offset=true;
      break;
    }
    case 3:
    {
      offset3=offset3+1000*strtodouble(dat);
      new_offset=true;
      break;
    }
  }

  return true;
}

void correct_height()
{
  new_offset=false;

  switch(drone)
  {
    case 1:
    {
      msg.height=height+offset1;
      msg.chksum = (short) 0xAAAA + msg.yaw + msg.height + msg.time + msg.X + msg.Y + msg.max_speed + msg.pos_acc + msg.properties + msg.wp_number;
      publisher.publish(msg);
      break;
    }
    case 2:
    {
      msg2.height=height+diff_hauteur+offset2;
      msg2.chksum = (short) 0xAAAA + msg2.yaw + msg2.height + msg2.time + msg2.X + msg2.Y + msg2.max_speed + msg2.pos_acc + msg2.properties + msg2.wp_number;
      publisher2.publish(msg2);
      break;
    }
    case 3:
    {
      msg3.height=height-diff_hauteur+offset3;
      msg3.chksum = (short) 0xAAAA + msg3.yaw + msg3.height + msg3.time + msg3.X + msg3.Y + msg3.max_speed + msg3.pos_acc + msg3.properties + msg3.wp_number;
      publisher3.publish(msg3);
      break;
    }
  }
}

bool wait_for_wp()
{
  ros::Duration(0.5).sleep();
  ros::spinOnce();
  while(!((nav_stat&0x01)&(nav_stat2&0x01)&(nav_stat3&0x01)))
  {
    ros::Duration(0.1).sleep();
    if(stop) return false;

    if(new_offset)
    {
      correct_height();
      ros::Duration(0.5).sleep();
    }

    ros::spinOnce();
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone_formation");

  ros::NodeHandle n;

  publisher = n.advertise<asctec_msgs::waypoint>("/asctec/WAYPOINT_INPUT3", 1);
  publisher2 = n.advertise<asctec_msgs::waypoint>("/asctec/WAYPOINT_INPUT4", 1);
  publisher3 = n.advertise<asctec_msgs::waypoint>("/asctec/WAYPOINT_INPUT5", 1);
  subscriber_gps = n.subscribe("/asctec/GPS_DATA3", 1, Callback, ros::TransportHints().tcpNoDelay());
  subscriber_imu = n.subscribe("/asctec/IMU_CALCDATA3", 1, Callback2, ros::TransportHints().tcpNoDelay());
  subscriber_ack = n.subscribe("/asctec/ACK3", 1, Callback3, ros::TransportHints().tcpNoDelay());
  subscriber_cw = n.subscribe("/asctec/CURRENT_WAY3", 1, Callback_cw, ros::TransportHints().tcpNoDelay());
  subscriber_cw2 = n.subscribe("/asctec/CURRENT_WAY4", 1, Callback_cw2, ros::TransportHints().tcpNoDelay());
  subscriber_cw3 = n.subscribe("/asctec/CURRENT_WAY5", 1, Callback_cw3, ros::TransportHints().tcpNoDelay());

  struct sigaction s_action;
  s_action.sa_handler = ctrlc_gestion;
  sigemptyset(&s_action.sa_mask);
  s_action.sa_flags = 0;
  sigaction(SIGINT, &s_action, NULL);

  stop=false;

  double ancien_X=0;
  double ancien_Y=0;

  offset1=0;
  offset2=0;
  offset3=0;
  drone=0;
  new_offset=false;

  while (!stop)
  {
    string data;
    int longitude_dest, latitude_dest;

    /*Longitude*/
    cout << "longitude (10^-7 °) : ";
    getline (cin,data);
    longitude_dest=strtodouble(data);

    ros::spinOnce();
    if(longitude_dest==0) longitude_dest=current.longitude;

    if(stop) return 0;

    /*Latitude*/
    cout << "latitude (10^-7 °) : ";
    getline (cin,data);
    latitude_dest=strtodouble(data);

    ros::spinOnce();
    if(latitude_dest==0) latitude_dest=current.latitude;

    if(stop) return 0;

    /*Height*/
    cout << "height (m) : ";
    getline (cin,data);
    height=round(1000*strtodouble(data));

    ros::spinOnce();
    if(height==0) height=current.height;

    msg.height=height+offset1;

    if(stop) return 0;

    int vitesse;
    n.getParam("vitesse", vitesse);

    msg.time=0;
    msg.pos_acc=2500;
    msg.wp_number=1;
    msg.properties=0x17;
    msg.max_speed=vitesse;

    msg2.time=0;
    msg2.pos_acc=2500;
    msg2.wp_number=1;
    msg2.properties=0x17;
    msg2.max_speed=vitesse;

    msg3.time=0;
    msg3.pos_acc=2500;
    msg3.wp_number=1;
    msg3.properties=0x17;
    msg3.max_speed=vitesse;



    /*Waypoints initiaux, mise en formation*/
    ros::spinOnce();

    double distance;
    n.getParam("distance", distance);

    double diff_long=(longitude_dest-current.longitude);
    double diff_lat=(latitude_dest-current.latitude);
    double diff_long2=pow(diff_long,2);
    double diff_lat2=pow(diff_lat,2);

    double Y=sqrt((pow(distance,2)*diff_lat2)/(diff_lat2+diff_long2*(pow(cos(current.latitude*UMR),2))));
    if(diff_lat>0) Y=-Y;
    double X=sqrt((pow(distance,2)*diff_long2*pow(cos(current.latitude*UMR),2))/(diff_lat2+diff_long2*(pow(cos(current.latitude*UMR),2))));
    if(diff_long>0) X=-X;

    double X2 = (sqrt(3)/2)*X + Y/2;
    double Y2 = -X/2 + (sqrt(3)/2)*Y;
    double X3 = (sqrt(3)/2)*X - Y/2;
    double Y3 = X/2 + (sqrt(3)/2)*Y;

    double cos=(ancien_X*X+ancien_Y*Y)/(sqrt(ancien_X*ancien_X+ancien_Y*ancien_Y)*sqrt(X*X+Y*Y));

    //Si l'angle entre l'opposé du vecteur direction actuel et le nouveau vecteur direction du drone principal est inférieur à 30° en valeur absolue, on inverse les rôles des deux drones pour qu'ils ne se croisent pas.
    if(cos>=(sqrt(3)/2) && !(ancien_X==0 && ancien_Y==0)){
      publisher_tmp=publisher3;
      publisher3 = publisher2;
      publisher2 = publisher_tmp;
    }      
    ancien_X=-X;
    ancien_Y=-Y;

    double lat,lon;

    xy2latlon(current.latitude/10000000.0,current.longitude/10000000.0,X2,Y2,&lat,&lon);

    n.getParam("diff_hauteur", diff_hauteur);

    msg2.X=lon*10000000;
    msg2.Y=lat*10000000;
    msg2.height=height+diff_hauteur+offset2;

    xy2latlon(current.latitude/10000000.0,current.longitude/10000000.0,X3,Y3,&lat,&lon);

    msg3.X=lon*10000000;
    msg3.Y=lat*10000000;
    msg3.height=height-diff_hauteur+offset3;

    /*Yaw : orientation*/
    if(longitude_dest==current.longitude && latitude_dest==current.latitude)
    {
      msg.yaw=1000*current.mag_heading;
    }
    else
    {
        X=-X;
        Y=-Y;
        double angle=asin(X/sqrt(X*X+Y*Y));

        if(X<0 && Y<0) angle=angle+3.*PI/2.;
        else if(X<0) angle=angle+2.*PI;
            else if(Y<0) angle=angle+PI/2.;

        msg.yaw=1000*angle*180./PI;
    }

    msg2.yaw=msg.yaw-45000;
    if(msg2.yaw<0) msg2.yaw=msg2.yaw+360000;

    msg3.yaw=msg.yaw+45000;
    if(msg3.yaw>=360000) msg3.yaw=msg3.yaw-360000;

    msg.X=current.longitude;
    msg.Y=current.latitude;

    msg.chksum = (short) 0xAAAA + msg.yaw + msg.height + msg.time + msg.X + msg.Y + msg.max_speed + msg.pos_acc + msg.properties + msg.wp_number;
    msg2.chksum = (short) 0xAAAA + msg2.yaw + msg2.height + msg2.time + msg2.X + msg2.Y + msg2.max_speed + msg2.pos_acc + msg2.properties + msg2.wp_number;
    msg3.chksum = (short) 0xAAAA + msg3.yaw + msg3.height + msg3.time + msg3.X + msg3.Y + msg3.max_speed + msg3.pos_acc + msg3.properties + msg3.wp_number;

    publisher.publish(msg);

    /*Si l'angle entre l'opposé du vecteur direction actuel et le nouveau vecteur direction du drone principal est compris entre 30° et 90° en valeur absolue, les drones ne devraient à priori pas se croiser sauf s'il y a un lag et qu'un drone part en retard.
    On les fait se placer tour à tour pour éviter toute collision en cas de lag*/
    if(cos<(sqrt(3)/2) && cos>=0 && !(ancien_X==0 && ancien_Y==0)) {
      if(ancien_X*Y-ancien_Y*X>=0) {
        publisher3.publish(msg3);
        ros::Duration(0.5).sleep();
        ros::spinOnce();
        while(!((nav_stat&0x01)&(nav_stat2&0x01)))
        {
          ros::Duration(0.1).sleep();
          if(stop) return 0;

          ros::spinOnce();
        }
        publisher2.publish(msg2);
      }
      else {
        publisher2.publish(msg2);
        ros::Duration(0.5).sleep();
        ros::spinOnce();
        while(!((nav_stat&0x01)&(nav_stat3&0x01)))
        {
          ros::Duration(0.1).sleep();
          if(stop) return 0;

          ros::spinOnce();
        }
        publisher3.publish(msg3);
      }
    }
    else
    {
      publisher2.publish(msg2);
      publisher3.publish(msg3);
    }

    if(!wait_for_wp()) return 0;

    cout << "Mise en place terminé.\n";



    /*Correction de la hauteur par l'utilisateur si nécessaire*/
    stop2=false;
    while(offset_())
    {
      if(new_offset)
      {
        correct_height();
      }
    }

    if(!wait_for_wp()) return 0;

    cout << "Mise en place2.\n";

    pthread_create(&thread, NULL, offset, NULL);

    

    /*Waypoint intermédiaire*/

    /*double distance_trajet=sqrt(pow(diff_lat/10000000.0*MEAN_EARTH_DIAMETER*PI/360.,2)+pow(diff_long/10000000.0*MEAN_EARTH_DIAMETER*PI/360.*cos(current.latitude*UMR),2));

    double X2p=(X2/distance)*distance_trajet/sqrt(3);
    double Y2p=(Y2/distance)*distance_trajet/sqrt(3);
    double X3p=(X3/distance)*distance_trajet/sqrt(3);
    double Y3p=(Y3/distance)*distance_trajet/sqrt(3);*/

    msg.X=diff_long/2+current.longitude;
    msg.Y=diff_lat/2+current.latitude;
    msg.chksum = (short) 0xAAAA + msg.yaw + msg.height + msg.time + msg.X + msg.Y + msg.max_speed + msg.pos_acc + msg.properties + msg.wp_number;

    xy2latlon((msg.Y)/10000000.0,(msg.X)/10000000.0,2*X2,2*Y2,&lat,&lon);

    msg2.X=lon*10000000;
    msg2.Y=lat*10000000;
    msg2.chksum = (short) 0xAAAA + msg2.yaw + msg2.height + msg2.time + msg2.X + msg2.Y + msg2.max_speed + msg2.pos_acc + msg2.properties + msg2.wp_number;

    xy2latlon((msg.Y)/10000000.0,(msg.X)/10000000.0,2*X3,2*Y3,&lat,&lon);

    msg3.X=lon*10000000;
    msg3.Y=lat*10000000;
    msg3.chksum = (short) 0xAAAA + msg3.yaw + msg3.height + msg3.time + msg3.X + msg3.Y + msg3.max_speed + msg3.pos_acc + msg3.properties + msg3.wp_number;

    publisher.publish(msg);
    publisher2.publish(msg2);
    publisher3.publish(msg3);

    if(!wait_for_wp()) return 0;

    cout << "Mi-parcours atteint.\n";



    /*Waypoint final*/

    msg.X=longitude_dest;
    msg.Y=latitude_dest;
    msg.chksum = (short) 0xAAAA + msg.yaw + msg.height + msg.time + msg.X + msg.Y + msg.max_speed + msg.pos_acc + msg.properties + msg.wp_number;

    xy2latlon(latitude_dest/10000000.0,longitude_dest/10000000.0,X2,Y2,&lat,&lon);

    msg2.X=lon*10000000;
    msg2.Y=lat*10000000;
    msg2.chksum = (short) 0xAAAA + msg2.yaw + msg2.height + msg2.time + msg2.X + msg2.Y + msg2.max_speed + msg2.pos_acc + msg2.properties + msg2.wp_number;

    xy2latlon(latitude_dest/10000000.0,longitude_dest/10000000.0,X3,Y3,&lat,&lon);

    msg3.X=lon*10000000;
    msg3.Y=lat*10000000;
    msg3.chksum = (short) 0xAAAA + msg3.yaw + msg3.height + msg3.time + msg3.X + msg3.Y + msg3.max_speed + msg3.pos_acc + msg3.properties + msg3.wp_number;

    publisher.publish(msg);
    publisher2.publish(msg2);
    publisher3.publish(msg3);

    if(!wait_for_wp()) return 0;

    cout << "Destination atteinte.\n";



    /*Réglage du yaw à destination*/

    msg2.yaw=msg.yaw-120000;
    if(msg2.yaw<0) msg2.yaw=msg2.yaw+360000;
    msg2.chksum = (short) 0xAAAA + msg2.yaw + msg2.height + msg2.time + msg2.X + msg2.Y + msg2.max_speed + msg2.pos_acc + msg2.properties + msg2.wp_number;

    msg3.yaw=msg.yaw+120000;
    if(msg3.yaw>=360000) msg3.yaw=msg3.yaw-360000;
    msg3.chksum = (short) 0xAAAA + msg3.yaw + msg3.height + msg3.time + msg3.X + msg3.Y + msg3.max_speed + msg3.pos_acc + msg3.properties + msg3.wp_number;

    publisher2.publish(msg2);
    publisher3.publish(msg3);

    if(!wait_for_wp()) return 0;

    cout << "Yaw réglé.\n";

    stop2=true;
    cin.putback('0');
    pthread_cancel(thread);
    cin.get();

  }

  return 0;
}
