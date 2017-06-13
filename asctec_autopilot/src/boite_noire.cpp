#include "ros/ros.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include "asctec_msgs/LLStatus.h"
#include "asctec_msgs/GPSData.h"
#include "asctec_msgs/IMUCalcData.h"
#include "asctec_msgs/ControllerOutput.h"
#include "asctec_msgs/CtrlInput.h"
#include "asctec_msgs/CurrentWay.h"
#include "asctec_msgs/ACK.h"
#include "asctec_msgs/waypoint.h"


using namespace std;

//*********************Instantiation**********************
double latitudeKML, longitudeKML, heightBAROKML;
string SubGPSData, SubLLStatus, SubIMUCalcData,SubACK, SubCurrentWay, SubControllerOutput, SubCtrlInput, Subwaypoint ;	
int latitude,longitude,GPSheight,GPSspeed_x,GPSspeed_y,GPSheading,horizontal_accuracy,vertical_accuracy,speed_accuracy;
int numSV,GPSstatus,battery_voltage_1,battery_voltage_2,LLSstatus,cpu_load,compass_enabled,chksum_error,flying,motors_on;
int flightMode,up_time,angle_nick,angle_roll,angle_yaw,angvel_nick,angvel_roll,angvel_yaw,acc_x_calib,acc_y_calib,acc_z_calib;
int acc_x,acc_y,acc_z,acc_angle_nick,acc_angle_roll,acc_absolute_value,Hx,Hy,Hz,mag_heading,speed_x,speed_y,speed_z,Baroheight;
int dheight,dheight_reference,height_reference,nav_stat,dummy1,properties,nr_of_wp,current_wp,current_wp_memlocation,status;
int dummy2,distance_to_wp,code,nick,roll,yaw,thrust,com_pitch,com_roll,com_yaw,com_thrust,com_ctrl,com_chksum,wp_wp_number;
int wp_properties,wp_max_speed,wp_time,wp_pos_acc,wp_chksum,wp_X,wp_Y,wp_yaw,wp_height;
bool ack_received;


void Callback1(asctec_msgs::GPSData msg)
{
//latitude/longitude in deg * 10^7
	latitude = msg.latitude;
	longitude = msg.longitude;

//GPS height in mm
	GPSheight = msg.height;

//speed in x (E/W) and y(N/S) in mm/s
	GPSspeed_x = msg.speed_x;
	GPSspeed_y = msg.speed_y;

//GPS heading in deg * 100
	GPSheading = msg.heading;

//accuracy estimates in mm and mm/s
	horizontal_accuracy = msg.horizontal_accuracy;
	vertical_accuracy = msg.vertical_accuracy;
	speed_accuracy = msg.speed_accuracy;

//number of satellite vehicles used in NAV solution
	numSV = msg.numSV;

//GPS status information; 0x03 = valid GPS fix
	GPSstatus = msg.status;
}

void Callback2(asctec_msgs::LLStatus msg)
{

//battery voltages in mV
	battery_voltage_1 = msg.battery_voltage_1;
	battery_voltage_2 = msg.battery_voltage_2;
//dont care
	LLSstatus = msg.status;
//Controller cycles per second (should be about 1000)
	cpu_load = msg.cpu_load;
//dont care
	compass_enabled = msg.compass_enabled;
	chksum_error = msg.chksum_error;
	flying = msg.flying;
	motors_on = msg.motors_on;
	flightMode = msg.flightMode;
//Time motors are turning
	up_time = msg.up_time;

}

void Callback3(asctec_msgs::IMUCalcData msg)
{

//angles derived by integration of gyro_outputs, drift compensated by data fusion;
//90000..+90000 pitch(nick) and roll, 0..360000 yaw; 1000 = 1 degree

	angle_nick = msg.angle_nick;
	angle_roll = msg.angle_roll;
	angle_yaw = msg.angle_yaw;

//angular velocities, raw values [16 bit], bias free, in 0.0154 degree/s (=> 64.8 = 1 degree/s)

	angvel_nick = msg.angvel_nick;
	angvel_roll = msg.angvel_roll;
	angvel_yaw = msg.angvel_yaw;

//acc-sensor outputs, calibrated: -10000..+10000 = -1g..+1g

	acc_x_calib = msg.acc_x_calib;
	acc_y_calib = msg.acc_y_calib;
	acc_z_calib = msg.acc_z_calib;

//horizontal / vertical accelerations: -10000..+10000 = -1g..+1g

	acc_x = msg.acc_x;
	acc_y = msg.acc_y;
	acc_z = msg.acc_z;

//reference angles derived by accelerations only: -90000..+90000; 1000 = 1 degree

	acc_angle_nick = msg.acc_angle_nick;
	acc_angle_roll = msg.acc_angle_roll;

//total acceleration measured (10000 = 1g)

	acc_absolute_value = msg.acc_absolute_value;

//magnetic field sensors output, offset free and scaled; units not determined, 
//as only the direction of the field vector is taken into account

	Hx = msg.Hx;
	Hy = msg.Hy;
	Hz = msg.Hz;

//compass reading: angle reference for angle_yaw: 0..360000; 1000 = 1 degree

	mag_heading = msg.mag_heading;

//pseudo speed measurements: integrated accelerations, pulled towards zero; units unknown;
//used for short-term position stabilization

	speed_x = msg.speed_x;
	speed_y = msg.speed_y;
	speed_z = msg.speed_z;

//height in mm (after data fusion)

	Baroheight = msg.height;

//diff. height in mm/s (after data fusion)

	dheight = msg.dheight;

//diff. height measured by the pressure sensor [mm/s]

	dheight_reference = msg.dheight_reference;

//height measured by the pressure sensor [mm]

	height_reference = msg.height_reference;
}

void Callback4(asctec_msgs::ControllerOutput msg)
{
//attitude controller outputs; 0..200 = -100 .. +100 %
	nick = msg.nick;
	roll = msg.roll;
	yaw = msg.yaw;

//current thrust (height controller output); 0..200 = 0..100%
    thrust = msg.thrust;

}

void Callback5(asctec_msgs::ACK msg)
{
//Bool
	ack_received = msg.ack_received;
//int
	code = msg.code;
}



void Callback6(asctec_msgs::CurrentWay msg)
{
	nav_stat = msg.navigation_status;
	dummy1 = msg.dummy1;
	properties = msg.properties;
	nr_of_wp = msg.nr_of_wp;
	current_wp = msg.current_wp;
	current_wp_memlocation = msg.current_wp_memlocation;
	status = msg.status;
	dummy2 = msg.dummy2;
	distance_to_wp = msg.distance_to_wp;
}

void Callback7(asctec_msgs::CtrlInput msg)
{
// serial command (=Scientific Interface)
// Pitch input: -2047 .. 2047 (0 = neutral)
com_pitch = msg.pitch;
// Roll input: -2047 .. 2047 (0 = neutral)
com_roll = msg.roll;
// R/C Stick input: -2047 .. 2047 (0 = neutral)
com_yaw = msg.yaw;
// Collective: 0 .. 4095 (= 0 .. 100%)
com_thrust = msg.thrust;
// control byte:
//    bit 0: pitch control enabled
//    bit 1: roll control enabled
//    bit 2: yaw control enabled
//    bit 3: thrust control enabled
//  These bits can be used to only enable one axis at a time and thus to control
//  the other axes manually. This usually helps a lot to set up and finetune
//  controllers for each axis seperately.
com_ctrl = msg.ctrl;
com_chksum = msg.chksum;
}


void Callback8(asctec_msgs::waypoint msg)
{
wp_wp_number = msg.wp_number;
wp_properties = msg.properties;
wp_max_speed = msg.max_speed;
wp_time = msg.time;
wp_pos_acc = msg.pos_acc;
wp_chksum = msg.chksum;

wp_X = msg.X;
wp_Y = msg.Y;
wp_yaw = msg.yaw;
wp_height = msg.height;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "trace_gps");

	ros::NodeHandle n;
	//SubCurrentWay, SubCtrlInput, Subwaypoint
	SubGPSData="/asctec/GPS_DATA";
	SubLLStatus="/asctec/LL_STATUS";
	SubIMUCalcData="/asctec/IMU_CALCDATA";
	SubControllerOutput="/asctec/CONTROLLER_OUTPUT";
	SubACK="/asctec/ACK";
	SubCurrentWay="/asctec/CURRENT_WAY";
	SubCtrlInput="/asctec/CTRL_INPUT";
	Subwaypoint="/asctec/WAYPOINT";

	int j;
	string k;
	double frequence;

	cout << "Quel drone devons nous suivre ? \n";
	cin >> k;
	SubGPSData+=k+"";
	SubLLStatus+=k+"";
	SubIMUCalcData+=k+"";
	SubControllerOutput+=k+"";
	SubACK+=k+"";
	SubCurrentWay+=k+"";
	SubCtrlInput+=k+"";
	Subwaypoint+=k+"";

	cout << "GPSData = " << SubGPSData << "\n";
	cout << "LLStatus = " << SubLLStatus << "\n";
	cout << "IMUCalcData = " << SubIMUCalcData << "\n";
	cout << "ControllerOutput = " << SubControllerOutput << "\n";
	cout << "ACK = " << SubACK << "\n";
	cout << "CurrentWay = " << SubCurrentWay << "\n";
	cout << "CtrlInput = " << SubCtrlInput << "\n";
	cout << "Waypoint = " << Subwaypoint << "\n";


	cin.clear();

	cout << "A quelle frequence ? (en seconde) .\n";
	cin >> j;
	frequence = (double)1/j;
	cin.clear();
	ros::init(argc, argv, "trace_gps");


	cout << "Initialisation des subribers\n";

	ros::Subscriber subscriber1 = n.subscribe(SubGPSData, 1, Callback1, ros::TransportHints().tcpNoDelay());
	cout << "GPSData souscrit\n";
	ros::Subscriber subscriber2 = n.subscribe(SubLLStatus, 1, Callback2, ros::TransportHints().tcpNoDelay());
	cout << "LLStatus souscrit\n";  
	ros::Subscriber subscriber3 = n.subscribe(SubIMUCalcData, 1, Callback3, ros::TransportHints().tcpNoDelay());
	cout << "IMUCalcData souscrit\n";
	ros::Subscriber subscriber4 = n.subscribe(SubControllerOutput, 1, Callback4, ros::TransportHints().tcpNoDelay());
	cout << "ControllerOutput souscrit\n";
	ros::Subscriber subscriber5 = n.subscribe(SubACK, 1, Callback5, ros::TransportHints().tcpNoDelay());
	cout << "ACK souscrit\n";	
	ros::Subscriber subscriber6 = n.subscribe(SubCurrentWay, 1, Callback6, ros::TransportHints().tcpNoDelay());
	cout << "CurrentWay souscrit\n";
	ros::Subscriber subscriber7 = n.subscribe(SubCtrlInput, 1, Callback7, ros::TransportHints().tcpNoDelay());
	cout << "CtrlInput souscrit\n";
	ros::Subscriber subscriber8 = n.subscribe(Subwaypoint, 1, Callback8, ros::TransportHints().tcpNoDelay());
	cout << "Waypoint souscrit\n";
	
	cout << "C'est parti ! \n";

	ofstream trajet("/home/asctec/donneesGPS.kml");
	ofstream tableau_GPSDATA("/home/asctec/tableau_GPSDATA.txt");
	ofstream tableau_LLSSTATUS("/home/asctec/tableau_LLSSTATUS.txt");
	ofstream tableau_IMUCalcData("/home/asctec/tableau_IMUCalcData.txt");
	ofstream tableau_ControllerOutput("/home/asctec/tableau_ControllerOutput.txt");
	ofstream tableau_ACK("/home/asctec/tableau_ACK.txt");
	ofstream tableau_CurrentWay("/home/asctec/tableau_CurrentWay.txt");
	ofstream tableau_CtrlInput("/home/asctec/tableau_CtrlInput.txt");
	ofstream tableau_Waypoint("/home/asctec/tableau_Waypoint.txt");


	trajet<<"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n<Document>\n<Folder>\n<name>Asctec"<<k<<"</name>\n<Style id=\"yellowLineGreenPoly\">\n<LineStyle>\n<color>7f00ffff</color>\n<width>4</width>";
	trajet<<"</LineStyle>\n<PolyStyle>\n<color>7f00ff00</color>\n</PolyStyle>\n</Style>\n<Placemark>\n<name>Absolute Extruded</name>\n<description>Transparent green wall with yellow outlines</description>\n<styleUrl>#yellowLineGreenPoly</styleUrl>\n";
	trajet<<"<LineString>\n<altitudeMode>relativeToGround</altitudeMode>\n<coordinates>";
    tableau_GPSDATA<<"latitude, longitude, GPSheight, GPSspeed_x, GPSspeed_y, GPSheading, horizontal_accuracy, vertical_accuracy, speed_accuracy, numSV, GPSstatus;\n";
    tableau_LLSSTATUS<<"	battery_voltage_1, battery_voltage_2, status, cpu_load, compass_enabled, chksum_error, flying, motors_on, flightMode, up_time\n";
    tableau_IMUCalcData<<"	angle_nick ,angle_roll, angle_yaw, angvel_nick, angvel_roll, angvel_yaw, acc_x_calib,acc_y_calib,acc_z_calib, acc_x, acc_y, acc_z, acc_angle_nick, acc_angle_roll, acc_absolute_value, Hx, Hy, Hz, mag_heading, speed_x, speed_y, speed_z, Baroheight, dheight, dheight_reference, height_reference\n";
    tableau_ControllerOutput<<"nick ,roll ,yaw ,thrust\n";
    tableau_ACK<<"ack_received, code\n";
	tableau_CurrentWay<<"nav_stat, dummy1, properties, nr_of_wp, current_wp, current_wp_memlocation, status, dummy2, distance_to_wp,\n";
	tableau_CtrlInput<<"com_pitch, com_roll, com_yaw, com_thrust, com_ctrl, com_chksum\n";
	tableau_Waypoint<<"wp_wp_number, wp_properties, wp_max_speed, wp_time, wp_pos_acc, wp_chksum, wp_X, wp_Y, wp_yaw, wp_height, com_pitch\n";
	cout << "Fichiers Crees\n";
	int l=0;
	while(ros::ok())
	{	
		l=l+1;

		cout << "nombre de points : " << l << "\n";
		ros::Rate loop_rate(frequence);
		if (longitude!=0)
		{
			longitudeKML = longitude/10000000;
			latitudeKML = latitude/10000000;
			heightBAROKML = Baroheight/1000;
			trajet <<setprecision(10)<<longitudeKML<<","<<latitudeKML<<","<<heightBAROKML<<"\n";
tableau_GPSDATA<<latitude<<","<<longitude<<","<<GPSheight<<","<<GPSspeed_x<<","<<GPSspeed_y<<","<<GPSheading<<","<<horizontal_accuracy<<","<<vertical_accuracy<<","<<speed_accuracy<<","<<numSV<<","<<GPSstatus<<"\n";
tableau_LLSSTATUS<<battery_voltage_1<<","<<battery_voltage_2<<","<<LLSstatus<<","<<cpu_load<<","<<compass_enabled<<","<<chksum_error<<","<<flying<<","<<motors_on<<","<<flightMode<<","<<up_time<<"\n";
tableau_IMUCalcData<<angle_nick<<","<<angle_roll<<","<<angle_yaw<<","<<angvel_nick<<","<<angvel_roll<<","<<angvel_yaw<<","<<acc_x_calib<<","<<acc_y_calib<<","<<acc_z_calib<<","<<acc_x<<","<<acc_y<<","<<acc_z<<","<<acc_angle_nick<<","<<acc_angle_roll<<","<<acc_absolute_value<<","<<Hx<<","<<Hy<<","<<Hz<<","<<mag_heading<<","<<speed_x<<","<<speed_y<<","<<speed_z<<","<<Baroheight<<","<<dheight<<","<<dheight_reference<<","<<height_reference<<"\n";
tableau_ControllerOutput << nick <<","<< roll <<","<< yaw <<","<< thrust <<"\n";
tableau_ACK << ack_received <<","<< code <<"\n";
tableau_CurrentWay<<nav_stat<<","<<dummy1<<","<<properties<<","<<nr_of_wp<<","<<current_wp<<","<<current_wp_memlocation<<","<<status<<","<<dummy2<<","<<distance_to_wp<<","<<"\n";
tableau_CtrlInput<<com_pitch<<","<<com_roll<<","<<com_yaw<<","<<com_thrust<<","<<com_ctrl<<","<<com_chksum<<"\n";
tableau_Waypoint<<wp_wp_number<<","<<wp_properties<<","<<wp_max_speed<<","<<wp_time<<","<<wp_pos_acc<<","<<wp_chksum<<","<<wp_X<<","<<wp_Y<<","<<wp_yaw<<","<<wp_height<<","<<com_pitch<<"\n";
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	trajet<<"</coordinates>\n</LineString>\n</Placemark>\n</Folder>\n</Document>\n</kml>";
	trajet.close();
    tableau_GPSDATA.close();
    tableau_LLSSTATUS.close();
    tableau_IMUCalcData.close();
    tableau_ControllerOutput.close();
    tableau_ACK.close();
    tableau_CurrentWay.close();
    tableau_CtrlInput.close();
    tableau_Waypoint.close();
}
