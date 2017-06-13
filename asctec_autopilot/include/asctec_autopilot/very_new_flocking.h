#ifndef ASCTEC_AUTOPILOT_FDP_H
#define ASCTEC_AUTOPILOT_FDP_H

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <signal.h>
#include <pthread.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include "asctec_msgs/GPSData.h"
#include "asctec_msgs/IMUCalcData.h"
#include "asctec_msgs/waypoint.h"
#include "asctec_msgs/ACK.h"
#include "asctec_msgs/CurrentWay.h"

#define MEAN_EARTH_DIAMETER	12756274.0
#define UMR	0.017453292519943295769236907684886		//PI/180
#define PI 3.1415926535897932384626433832795
#define R	6378137 // Rayon moyen de la terre = MED/2
#define E	0.0818191910428158 // Elipsoïde


using namespace std;
using namespace ros;


struct coordinatesGeo
{
  //latitude/longitude in deg * 10ˆ7
  int latitude;
  int longitude;
};
struct coordinatesMerc
{
    int X;
    int Y;
};

struct decalage
{
    int X;
    int Y;
};

struct MesDrones
{
  //Numero du drone (son code)
  int numero;
  //latitude/longitude in deg * 10ˆ7
  coordinatesGeo positionGeo;
  coordinatesMerc positionMerc;
  //Coordonnée de destination
  coordinatesGeo destinationGeo;
  coordinatesMerc destinationMerc;  
  // Decalage par rapport au leader (en metres)
  decalage DecalXY;
  // Hauteur désirée (en mm)
  int hauteur;
  // Yaw suivant
  int angle;
  //compass reading: angle reference for angle_yaw: 0..360000; 1000 = 1 degree
  int GPSdirection;
  int IMUdirection;
  //height in mm (de l'IMU)
  int GPSheight;
  int IMUheight;
  //Le Waypoint actuel
  int wp_actuel;
  //Navigation Statut
  //A comprendre grace à boite_noire
  int nav_stat;
  //Acknowlegement
  bool ACK;
  int code;
  Publisher WPpublisher;

  asctec_msgs::waypoint WPmessage;
  string Pub_Waypoint_Input;
  string Sub_GPS;
  string Sub_Current_Way;
  string Sub_ACK;
  string Sub_IMU;
};

class Flocking {
public :
Flocking();
ros::NodeHandle n;
vector<MesDrones> Drone;
string ListeDrone;
int vitesse;
string EcartX;
string EcartY;
string VLatitude;
string VLongitude;
int CodeSup;
int HauteurSup;
string HauteurForm;

void new_waypoint();
void EcartSup(int a);
std::vector <coordinatesGeo> VectorDest(string lati, string longi);
std::vector <int> VectorParsing(string s);
void Parsing(string s);
coordinatesGeo CalcPosGeo(coordinatesGeo pt, double angle, int NumSuiveur );
void ChangeAngle(int NumeroDrone, int angle);
void Publication(int NumeroDrone);
double SF(double lati);
double distance(int a, int b);
coordinatesGeo mercaToGeo(coordinatesMerc pt);
double CapDestination(int a);
coordinatesMerc geoToMerca(coordinatesGeo pt);
void Callback_ACK(const asctec_msgs::ACKConstPtr msg, int DroneNumero);
void Callback_IMU(const asctec_msgs::IMUCalcDataConstPtr msg, int DroneNumero);
void Callback_NavStat(const asctec_msgs::CurrentWayConstPtr msg, int DroneNumero);
void Callback_GPS(const asctec_msgs::GPSDataConstPtr msg, int DroneNumero);
string Concatenation(string s, int Num);
};



#endif
