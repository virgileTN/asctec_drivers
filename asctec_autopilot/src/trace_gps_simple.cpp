#include "ros/ros.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include "asctec_msgs/LLStatus.h"
#include "asctec_msgs/GPSData.h"
#include "asctec_msgs/IMUCalcData.h"

using namespace std;
double latitude, longitude, batterie, heightBARO, hauteurGPS;
std::string gps, batt, haut;	

void Callback1(asctec_msgs::GPSData msg)
{
  latitude = msg.latitude;
  longitude = msg.longitude;
  hauteurGPS = msg.height;
}


void Callback2(asctec_msgs::LLStatus msg)
{
  batterie = msg.battery_voltage_1;
}

void Callback3(asctec_msgs::IMUCalcData msg)
{
  heightBARO = msg.height;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "trace_gps");

	ros::NodeHandle n;
	std::string gps="/asctec/GPS_DATA";
	std::string batt="/asctec/LL_STATUS";
	std::string haut="/asctec/IMU_CALCDATA";
	int j;
	string k;
	double frequence;

	cout << "Quel drone devons nous suivre ? \n";
	cin >> k;
	gps+=k+"";
	batt+=k+"";
	haut+=k+"";
	cout << "GPS = " << gps << "\n";
	cout << "Batterie = " << batt << "\n";
	cout << "Hauteur = " << haut << "\n";

	cin.clear();
	
	cout << "A quelle frequence ? (en seconde) .\n";
	cin >> j;
	frequence = (double)1/j;
	cin.clear();
	ros::init(argc, argv, "trace_gps");
	

	cout << "Initialisation des subribers\n";
  	ros::Subscriber subscriber1 = n.subscribe(gps, 1, Callback1, ros::TransportHints().tcpNoDelay());
	cout << "GPS souscrit\n";
  	ros::Subscriber subscriber2 = n.subscribe(batt, 1, Callback2, ros::TransportHints().tcpNoDelay());
	cout << "Batterie souscrite\n";  
  	ros::Subscriber subscriber3 = n.subscribe(haut, 1, Callback3, ros::TransportHints().tcpNoDelay());
	cout << "Hauteur souscrite\n";

	cout << "C'est parti ! \n";

	ofstream files("/home/asctec/donneesGPS.kml");
	ofstream tableau("/home/asctec/tableau_donnees.txt");

	files<<"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n<Document>\n<Folder>\n<name>Asctec"<<k<<"</name>\n<Style id=\"yellowLineGreenPoly\">\n<LineStyle>\n<color>7f00ffff</color>\n<width>4</width>";
	files<<"</LineStyle>\n<PolyStyle>\n<color>7f00ff00</color>\n</PolyStyle>\n</Style>\n<Placemark>\n<name>Absolute Extruded</name>\n<description>Transparent green wall with yellow outlines</description>\n<styleUrl>#yellowLineGreenPoly</styleUrl>\n";
    	files<<"<LineString>\n<altitudeMode>relativeToGround</altitudeMode>\n<coordinates>";
	tableau<<"points"<<","<<"longitude"<<","<<"latitude"<<","<<"heightBARO"<<","<<"hauteurGPS"<<","<<"batterie"<<"\n";
	cout << "Fichier Cree\n";
	int l=0;
	while(ros::ok())
	{	
		l=l+1;
		
		cout << "nombre de points : " << l << "\n";
		ros::Rate loop_rate(frequence);
		//fprintf(files, "%2.7f,%2.7f,%3.3f\n",longitude/10000000,latitude/10000000,hauteurGPS/1000);
		if (longitude!=0){
		files<<setprecision(10)<<longitude/10000000<<","<<latitude/10000000<<","<<heightBARO/1000<<"\n";
		tableau<<l<<","<<setprecision(10) <<longitude/10000000<< ","<<latitude/10000000<< ","<<heightBARO<<","<<hauteurGPS/1000<<","<<batterie<<"\n";}
		loop_rate.sleep();
		ros::spinOnce();
	}
	files<<"</coordinates>\n</LineString>\n</Placemark>\n</Folder>\n</Document>\n</kml>";
	files.close();
	tableau.close();	
}
