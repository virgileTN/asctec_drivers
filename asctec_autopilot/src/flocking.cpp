#include "asctec_autopilot/flocking.h"

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

void Callback_cw4(asctec_msgs::CurrentWay msg)
{
  nav_stat4 = msg.navigation_status;
}

void Callback_cw5(asctec_msgs::CurrentWay msg)
{
  nav_stat5 = msg.navigation_status;
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

  cout << "modifier hauteur du drone (1 : principal, 2 : gauche, 3 : droite, 4 : superieur, 5 arriere ) : ";
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
	case 4:
	{
		offset4 = offset4 + 1000 * strtodouble(dat);
		new_offset = true;
		break;
	}
	case 5:
	{
		offset5 = offset5 + 1000 * strtodouble(dat);
		new_offset = true;
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
	case 4:
	{
		msg4.height = height - diff_hauteur + offset4;
		msg4.chksum = (short)0xAAAA + msg4.yaw + msg4.height + msg4.time + msg4.X + msg4.Y + msg4.max_speed + msg4.pos_acc + msg4.properties + msg4.wp_number;
		publisher4.publish(msg4);
		break;
	}
	case 5:
	{
		msg5.height = height - diff_hauteur + offset5;
		msg5.chksum = (short)0xAAAA + msg5.yaw + msg5.height + msg5.time + msg5.X + msg5.Y + msg5.max_speed + msg5.pos_acc + msg5.properties + msg5.wp_number;
		publisher5.publish(msg5);
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
  
int nbdrones; 

	cout << "Formation a 3 ou 4 Drones ? (sans le superieur) ";
		cin >> nbdrones;
		cin.clear();
	
switch (nbdrones)
{
	case 3: {

bool sup;		
string i;
string j;
string k;
string l;
	
std::string pub1="/asctec/WAYPOINT_INPUT";
std::string sub1="/asctec/CURRENT_WAY";
std::string GPS1="/asctec/GPS_DATA";
std::string IMU1="/asctec/IMU_CALCDATA";
std::string ACK1="/asctec/ACK";

std::string pub2="/asctec/WAYPOINT_INPUT";
std::string pub3="/asctec/WAYPOINT_INPUT";
std::string pub4="/asctec/WAYPOINT_INPUT";

std::string sub2="/asctec/CURRENT_WAY";
std::string sub3="/asctec/CURRENT_WAY";
std::string sub4="/asctec/CURRENT_WAY";	

	cout << "Y'a t'il un drone superieur ? oui=1, non=0 ";
	cin >> sup;
	
	
	cout << "Quel est le code du drone Leader ? ";
	cin.clear();
	cin >> i;
	pub1+=i+"";
	cout << "pub1 " + pub1 + ".\n";
	sub1+=i+"";
	GPS1+=i+"";
	IMU1+=i+"";
	ACK1+=i+"";
	cin.clear();
 	cout << "Quel est le code du drone Arriere Gauche ? ";
	cin >> j;
	pub2+=j+"";
	cout << "pub2 " + pub2 + ".\n";
	sub2+=j+"";
	cin.clear();
	cout << "Quel est le code du drone Arriere Droit ? ";
	cin >> k;
	pub3+=k+"";
	cout << "pub3 " + pub3 + ".\n";
	sub3+=k+"";
	cin.clear();
if (sup) {
	cout << "Quel est le code du drone Superieur ? ";
	cin >> l;
	pub4+=l+"";
	cout << "pub4 " + pub4 + ".\n";
	sub4+=l+"";
	cin.clear();}
	
  publisher = n.advertise<asctec_msgs::waypoint>(pub1, 1);
  publisher2 = n.advertise<asctec_msgs::waypoint>(pub2, 1);
  publisher3 = n.advertise<asctec_msgs::waypoint>(pub3, 1);

  subscriber_gps = n.subscribe(GPS1, 1, Callback, ros::TransportHints().tcpNoDelay());
  subscriber_imu = n.subscribe(IMU1, 1, Callback2, ros::TransportHints().tcpNoDelay());
  subscriber_ack = n.subscribe(ACK1, 1, Callback3, ros::TransportHints().tcpNoDelay());
  subscriber_cw = n.subscribe(sub1, 1, Callback_cw, ros::TransportHints().tcpNoDelay());
  subscriber_cw2 = n.subscribe(sub2, 1, Callback_cw2, ros::TransportHints().tcpNoDelay());
  subscriber_cw3 = n.subscribe(sub3, 1, Callback_cw3, ros::TransportHints().tcpNoDelay());
if (sup) {
	subscriber_cw4 = n.subscribe(sub4, 1, Callback_cw4, ros::TransportHints().tcpNoDelay());
	publisher4 = n.advertise<asctec_msgs::waypoint>(pub4, 1);
}
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
  offset4 = 0;
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

	msg4.time=0;
    msg4.pos_acc=2500;
    msg4.wp_number=1;
    msg4.properties=0x17;
    msg4.max_speed=vitesse;

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

	//4th drone - in the middle of the formation, but higher
	double X4 = (X + X2 + X3) / 3;
	double Y4 = (Y + Y2 + Y3) / 3;


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

	//4th drone
	xy2latlon(current.latitude / 10000000.0, current.longitude / 10000000.0, X4, Y4, &lat, &lon);

	msg4.X = lon * 10000000;
	msg4.Y = lat * 10000000;
	//4th drone height -- same altitude difference, but on top
	msg4.height = height + 2*diff_hauteur + offset4;

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

	//4th drone - same yaw as the leader
	msg4.yaw = msg.yaw;

    msg.chksum = (short) 0xAAAA + msg.yaw + msg.height + msg.time + msg.X + msg.Y + msg.max_speed + msg.pos_acc + msg.properties + msg.wp_number;
    msg2.chksum = (short) 0xAAAA + msg2.yaw + msg2.height + msg2.time + msg2.X + msg2.Y + msg2.max_speed + msg2.pos_acc + msg2.properties + msg2.wp_number;
    msg3.chksum = (short) 0xAAAA + msg3.yaw + msg3.height + msg3.time + msg3.X + msg3.Y + msg3.max_speed + msg3.pos_acc + msg3.properties + msg3.wp_number;
	msg4.chksum = (short)0xAAAA + msg4.yaw + msg4.height + msg4.time + msg4.X + msg4.Y + msg4.max_speed + msg4.pos_acc + msg4.properties + msg4.wp_number;

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

    double X2p=(X2/distance)*distance_trajet/sqrt("/asctec/WAYPOINT_INPUT43);
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

	//4th drone
	xy2latlon((msg.Y) / 10000000.0, (msg.X) / 10000000.0, 2 * X4, 2 * Y4, &lat, &lon);

	msg4.X = lon * 10000000;
	msg4.Y = lat * 10000000;
	msg4.chksum = (short)0xAAAA + msg4.yaw + msg4.height + msg4.time + msg4.X + msg4.Y + msg4.max_speed + msg4.pos_acc + msg4.properties + msg4.wp_number;


    publisher.publish(msg);
    publisher2.publish(msg2);
    publisher3.publish(msg3);
	//4th drone
	publisher4.publish(msg4);

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

	//4th drone -- WTF???
	xy2latlon(latitude_dest / 10000000.0, longitude_dest / 10000000.0, X4, Y4, &lat, &lon);

	msg4.X = lon * 10000000;
	msg4.Y = lat * 10000000;
	msg4.chksum = (short)0xAAAA + msg4.yaw + msg4.height + msg4.time + msg4.X + msg4.Y + msg4.max_speed + msg4.pos_acc + msg4.properties + msg4.wp_number;


    publisher.publish(msg);
    publisher2.publish(msg2);
    publisher3.publish(msg3);
	//4th drone
	publisher4.publish(msg4);

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
case 4: {

bool sup;		
string i;
string j;
string k;
string l;
string m;	
std::string pub1="/asctec/WAYPOINT_INPUT";
std::string sub1="/asctec/CURRENT_WAY";
std::string GPS1="/asctec/GPS_DATA";
std::string IMU1="/asctec/IMU_CALCDATA";
std::string ACK1="/asctec/ACK";

std::string pub2="/asctec/WAYPOINT_INPUT";
std::string pub3="/asctec/WAYPOINT_INPUT";
std::string pub4="/asctec/WAYPOINT_INPUT";
std::string pub5="/asctec/WAYPOINT_INPUT";

std::string sub2="/asctec/CURRENT_WAY";
std::string sub3="/asctec/CURRENT_WAY";
std::string sub4="/asctec/CURRENT_WAY";
std::string sub5="/asctec/CURRENT_WAY";

	cout << "Vous avez selectionnez une formation en losange";
	cout << "Y'a t'il un drone superieur ? oui=1, non=0";
	cin >> sup;
	cout << "Quel est le code du drone Leader ?";
	cin >> i;
	pub1+=i+"";
	cout << "pub1 " + pub1 + ".\n";
	sub1+=i+"";
	GPS1+=i+"";
	IMU1+=i+"";
	ACK1+=i+"";
	cin.clear();
 	cout << "Quel est le code du drone Milieu Gauche ?";
	cin >> j;
	pub2+=j+"";
	cout << "pub2 " + pub2 + ".\n";
	sub2+=j+"";
	cin.clear();
	cout << "Quel est le code du drone Milieu Droit ?";
	cin >> k;
	pub3+=k+"";
	cout << "pub3 " + pub3 + ".\n";
	sub3+=k+"";
	cin.clear();
	if(sup){
	cout << "Quel est le code du drone Superieur ?";
	cin >> l;
	pub4+=l+"";
	cout << "pub4 " + pub4 + ".\n";
	sub4+=l+"";
	cin.clear();}
	cout << "Quel est le code du drone Arriere ?";
	cin >> m;
	pub5+=m+"";
	cout << "pub5 " + pub5 + ".\n";
	sub5+=m+"";
	cin.clear();
	
  publisher = n.advertise<asctec_msgs::waypoint>(pub1, 1);
  publisher2 = n.advertise<asctec_msgs::waypoint>(pub2, 1);
  publisher3 = n.advertise<asctec_msgs::waypoint>(pub3, 1);
  publisher5 = n.advertise<asctec_msgs::waypoint>(pub5, 1);
  subscriber_gps = n.subscribe(GPS1, 1, Callback, ros::TransportHints().tcpNoDelay());
  subscriber_imu = n.subscribe(IMU1, 1, Callback2, ros::TransportHints().tcpNoDelay());
  subscriber_ack = n.subscribe(ACK1, 1, Callback3, ros::TransportHints().tcpNoDelay());
  subscriber_cw = n.subscribe(sub1, 1, Callback_cw, ros::TransportHints().tcpNoDelay());
  subscriber_cw2 = n.subscribe(sub2, 1, Callback_cw2, ros::TransportHints().tcpNoDelay());
  subscriber_cw3 = n.subscribe(sub3, 1, Callback_cw3, ros::TransportHints().tcpNoDelay());
  subscriber_cw5 = n.subscribe(sub5, 1, Callback_cw5, ros::TransportHints().tcpNoDelay());
if(sup){
publisher4 = n.advertise<asctec_msgs::waypoint>(pub4, 1);
subscriber_cw4 = n.subscribe(sub4, 1, Callback_cw4, ros::TransportHints().tcpNoDelay());}

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
  //4th drone offset -- declaration needed!
  offset4 = 0;
  offset5 = 0;
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

	msg4.time=0;
    msg4.pos_acc=2500;
    msg4.wp_number=1;
    msg4.properties=0x17;
    msg4.max_speed=vitesse;
    
	msg5.time=0;
    msg5.pos_acc=2500;
    msg5.wp_number=1;
    msg5.properties=0x17;
    msg5.max_speed=vitesse;
    
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
	double X5 = X;
	double Y5 = Y;
	//4th drone - in the middle of the formation, but higher
	double X4 = (X + X2 + X3 + X5) / 4;
	double Y4 = (Y + Y2 + Y3) + X5 / 4;


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
	
	xy2latlon(current.latitude/10000000.0,current.longitude/10000000.0,X5,Y5,&lat,&lon);

    msg5.X=lon*10000000;
    msg5.Y=lat*10000000;
    msg5.height=msg.height+2*diff_hauteur;

	//4th drone
	xy2latlon(current.latitude / 10000000.0, current.longitude / 10000000.0, X4, Y4, &lat, &lon);

	msg4.X = lon * 10000000;
	msg4.Y = lat * 10000000;
	//4th drone height -- same altitude difference, but on top
	msg4.height = height + 3*diff_hauteur + offset4;

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
    //Drone arrière meme que le leader
	msg5.yaw = msg.yaw;
	//4th drone - same yaw as the leader
	msg4.yaw = msg.yaw;

    msg.chksum = (short) 0xAAAA + msg.yaw + msg.height + msg.time + msg.X + msg.Y + msg.max_speed + msg.pos_acc + msg.properties + msg.wp_number;
    msg2.chksum = (short) 0xAAAA + msg2.yaw + msg2.height + msg2.time + msg2.X + msg2.Y + msg2.max_speed + msg2.pos_acc + msg2.properties + msg2.wp_number;
    msg3.chksum = (short) 0xAAAA + msg3.yaw + msg3.height + msg3.time + msg3.X + msg3.Y + msg3.max_speed + msg3.pos_acc + msg3.properties + msg3.wp_number;
	msg4.chksum = (short)0xAAAA + msg4.yaw + msg4.height + msg4.time + msg4.X + msg4.Y + msg4.max_speed + msg4.pos_acc + msg4.properties + msg4.wp_number;
	msg5.chksum = (short)0xAAAA + msg5.yaw + msg5.height + msg5.time + msg5.X + msg5.Y + msg5.max_speed + msg5.pos_acc + msg5.properties + msg5.wp_number;
    publisher.publish(msg);
	publisher5.publish(msg5);
	publisher4.publish(msg4);
	
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

    double X2p=(X2/distance)*distance_trajet/sqrt("/asctec/WAYPOINT_INPUT43);
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

	xy2latlon((msg.Y) / 10000000.0, (msg.X) / 10000000.0, 2 * X4, 2 * Y4, &lat, &lon);
	msg4.X = lon * 10000000;
	msg4.Y = lat * 10000000;
	msg4.chksum = (short)0xAAAA + msg4.yaw + msg4.height + msg4.time + msg4.X + msg4.Y + msg4.max_speed + msg4.pos_acc + msg4.properties + msg4.wp_number;

	xy2latlon((msg.Y) / 10000000.0, (msg.X) / 10000000.0, 2 * X5, 2 * Y5, &lat, &lon);
	msg5.X = lon * 10000000;
	msg5.Y = lat * 10000000;
	msg5.chksum = (short)0xAAAA + msg5.yaw + msg5.height + msg5.time + msg5.X + msg5.Y + msg5.max_speed + msg5.pos_acc + msg5.properties + msg5.wp_number;

    publisher.publish(msg);
    publisher2.publish(msg2);
    publisher3.publish(msg3);
	publisher4.publish(msg4);
	publisher5.publish(msg5);
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

	//4th drone -- WTF???
	xy2latlon(latitude_dest / 10000000.0, longitude_dest / 10000000.0, X4, Y4, &lat, &lon);

	msg4.X = lon * 10000000;
	msg4.Y = lat * 10000000;
	msg4.chksum = (short)0xAAAA + msg4.yaw + msg4.height + msg4.time + msg4.X + msg4.Y + msg4.max_speed + msg4.pos_acc + msg4.properties + msg4.wp_number;

	xy2latlon(latitude_dest / 10000000.0, longitude_dest / 10000000.0, X5, Y5, &lat, &lon);

	msg5.X = lon * 10000000;
	msg5.Y = lat * 10000000;
	msg5.chksum = (short)0xAAAA + msg5.yaw + msg5.height + msg5.time + msg5.X + msg5.Y + msg5.max_speed + msg5.pos_acc + msg5.properties + msg5.wp_number;
    
    publisher.publish(msg);
    publisher2.publish(msg2);
    publisher3.publish(msg3);
	publisher4.publish(msg4);
	publisher5.publish(msg5);
	
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
}
}
