#include "asctec_autopilot/new_flocking.h"

using namespace std;
using namespace ros;
void Flocking::Callback_GPS(const asctec_msgs::GPSDataConstPtr msg, int DroneNumero)
{
    Drone[DroneNumero].positionGeo.latitude = msg->latitude;
    Drone[DroneNumero].positionGeo.longitude = msg->longitude;
    Drone[DroneNumero].GPSdirection = msg->heading;
    Drone[DroneNumero].GPSheight = msg->height;
    Drone[DroneNumero].positionMerc = geoToMerca(Drone[DroneNumero].positionGeo);
}

void Flocking::Callback_NavStat(const asctec_msgs::CurrentWayConstPtr msg, int DroneNumero)
{
    Drone[DroneNumero].nav_stat = msg->navigation_status;
}

void Flocking::Callback_IMU(const asctec_msgs::IMUCalcDataConstPtr msg, int DroneNumero)
{
    Drone[DroneNumero].IMUheight = msg->height;
    Drone[DroneNumero].IMUdirection = msg->mag_heading;
}


void Flocking::Callback_ACK(const asctec_msgs::ACKConstPtr msg, int DroneNumero)
{
    Drone[DroneNumero].ACK = msg->ack_received;
    Drone[DroneNumero].code = msg->code;
    /* POURQUOI ? */
    //msg_ack=true;
}

Flocking::Flocking()
{   
    n.getParam("ListeDrone", ListeDrone);
    n.getParam("vitesse", vitesse);
    n.getParam("EcartX", EcartX);
    n.getParam("EcartY", EcartY);
    n.getParam("CodeSup", CodeSup);
    n.getParam("VLatitude", VLatitude);
    n.getParam("VLongitude", VLongitude);
    n.getParam("HauteurSup", HauteurSup);
    n.getParam("HauteurForm", HauteurForm);

    vector<int> vecteurDrone = VectorParsing(ListeDrone);
    vector<int> vecteurEcartX = VectorParsing(EcartX);
    vector<int> vecteurEcartY = VectorParsing(EcartY);
    vector<int> vecteurHauteur = VectorParsing(HauteurForm);
    long unsigned a = vecteurDrone.size();
    for(long unsigned int i = 0; i<= a; i++){
        MesDrones DroneTemp ;
        DroneTemp.numero = vecteurDrone[i];
        DroneTemp.DecalXY.X = vecteurEcartX[i];
        DroneTemp.DecalXY.Y = vecteurEcartY[i];
        DroneTemp.hauteur = vecteurHauteur[i];
	DroneTemp.Pub_Waypoint_Input = "/asctec/WAYPOINT_INPUT";
	DroneTemp.Sub_GPS = "/asctec/GPS_DATA";
	DroneTemp.Sub_Current_Way = "/asctec/CURRENT_WAY";
	DroneTemp.Sub_ACK = "/asctec/ACK";
	DroneTemp.Sub_IMU = "/asctec/IMU_CALCDATA";

        Drone.push_back(DroneTemp);
    }
    // Creation de Table de Souscriveur
a = Drone.size();
    if (CodeSup > 0){
        a++;
    }

    Subscriber* subscriber_Gps_tab = new ros::Subscriber[a];
    Subscriber* subscriber_CW_tab = new ros::Subscriber[a];
    Subscriber* subscriber_IMU_tab = new ros::Subscriber[a];
    Subscriber* subscriber_ACK_tab = new ros::Subscriber[a];


    for (long unsigned int i=0; i<= a; i++){
        if (Drone[i].numero != 0){
            Drone[i].Pub_Waypoint_Input+=Drone[i].numero+"";
            Drone[i].Sub_GPS+=Drone[i].numero+"";
            Drone[i].Sub_Current_Way+=Drone[i].numero+"";
            Drone[i].Sub_ACK+=Drone[i].numero+"";
            Drone[i].Sub_IMU+=Drone[i].numero+"";

subscriber_Gps_tab[i]=n.subscribe<asctec_msgs::GPSData>(Drone[i].Sub_GPS,100, boost::bind(&Flocking::Callback_GPS,this,_1,i));
subscriber_CW_tab[i]=n.subscribe<asctec_msgs::CurrentWay>(Drone[i].Sub_Current_Way,100, boost::bind(&Flocking::Callback_NavStat,this,_1,i));
subscriber_ACK_tab[i]=n.subscribe<asctec_msgs::ACK>(Drone[i].Sub_ACK,100, boost::bind(&Flocking::Callback_ACK,this,_1,i));
subscriber_IMU_tab[i]=n.subscribe<asctec_msgs::IMUCalcData>(Drone[i].Sub_IMU,100, boost::bind(&Flocking::Callback_IMU,this,_1,i));

            Drone[i].WPpublisher = n.advertise<asctec_msgs::waypoint> (Drone[i].Pub_Waypoint_Input, 1);

            cout << "Drone n° : " << Drone[i].numero << " instantié. \n";

            //********************* A FINIR ******************
            // Remplir les décalages des drone en fonction de la formation
            // Ainsi que les hauteurs désirées

            }
        }

    if(CodeSup > 0){

        MesDrones DroneTemp;
        DroneTemp.numero = CodeSup;
        DroneTemp.hauteur = HauteurSup;
        Drone.push_back(DroneTemp);
        long unsigned int i = Drone.size();
        Drone[i].Pub_Waypoint_Input+=Drone[i].numero+"";
        Drone[i].Sub_GPS+=Drone[i].numero+"";
        Drone[i].Sub_Current_Way+=Drone[i].numero+"";
        Drone[i].Sub_ACK+=Drone[i].numero+"";
        Drone[i].Sub_IMU+=Drone[i].numero+"";

subscriber_Gps_tab[i]=n.subscribe<asctec_msgs::GPSData>(Drone[i].Sub_GPS,100, boost::bind(&Flocking::Callback_GPS,this,_1,i));
subscriber_CW_tab[i]=n.subscribe<asctec_msgs::CurrentWay>(Drone[i].Sub_Current_Way,100, boost::bind(&Flocking::Callback_NavStat,this,_1,i));
subscriber_ACK_tab[i]=n.subscribe<asctec_msgs::ACK>(Drone[i].Sub_ACK,100, boost::bind(&Flocking::Callback_ACK,this,_1,i));
subscriber_IMU_tab[i]=n.subscribe<asctec_msgs::IMUCalcData>(Drone[i].Sub_IMU,100, boost::bind(&Flocking::Callback_IMU,this,_1,i));
  

        Drone[i].WPpublisher = n.advertise<asctec_msgs::waypoint>(Drone[i].Pub_Waypoint_Input, 1);

        cout << "Drone supérieur n° : " << Drone[i].numero << " instantié."<< "\n";

        //********************* A FINIR ******************
        // Remplir les décalages des drones en fonction de la formation
        // Ainsi que les hauteurs désirées
        EcartSup(i);
        cout << "Drone Supérieur : " << Drone[i].numero << "instantié."<< "\n";
    }
    cout << "Il y a :" << Drone.size() << " drones \n";
    cout << "Instantiation Terminée." << "\n";
}


    // converti des coordonnées Geo en Mercator
    coordinatesMerc Flocking::geoToMerca(coordinatesGeo pt)
    {
        double a = 6378137;
        double e = 0.0818191910428158;
        double l = pt.latitude*M_PI/180;
        double sinl = sin(l);
        coordinatesMerc out;
        out.X = round(a*pt.longitude*M_PI/180);
        out.Y = round(a*log(tan(l/2+M_PI_4)*pow((1-e*sinl)/(1+e*sinl),(e/2))));
        return out;

    }


    // Donne le Cap de la destination suivante
    double Flocking::CapDestination(int a){
        double angle = atan2(Drone[a].positionMerc.X-Drone[a].destinationMerc.X,Drone[a].positionMerc.Y-Drone[a].destinationMerc.Y);
        // conversion de
        if (angle < 0){
            angle = angle + (2*PI);
        }
        return angle;
    }
    

    // Converti des coordonnées Mercator en Geo
    coordinatesGeo Flocking::mercaToGeo(coordinatesMerc pt)
    {
        double a = 6378137;
        double e = 0.0818191910428158;
        double e2 = e*e;
        double e4 = e2*e2;
        double e6 = e4*e2;
        double e8 = e4*e4;

        double d2 = e2/2 + 5*e4/24 + e6/12 + 13*e8/360;
        double d4 = 7*e4/48 + 29*e6/240 + 811*e8/11520;
        double d6 = 7*e6/120 + 81*e8/1120;
        double d8 = 4279*e8/161280;

        coordinatesGeo out;

        out.longitude = round((pt.X/a)*180/M_PI);
        double ki = atanh(pt.Y/a);
        double phi = ki + d2*sin(2*ki) + d4*sin(4*ki) + d6*sin(6*ki) + d8*sin(8*ki);
        out.latitude = round(phi*180/M_PI);

        return out;
    }


    // Donne la distance entre deux drones
    double Flocking::distance(int a, int b)
    {
        double latit = (Drone[a].positionGeo.latitude+Drone[b].positionGeo.latitude)/2;
        return (sqrt(pow((Drone[a].positionMerc.X-Drone[b].positionMerc.X),2)+pow((Drone[a].positionMerc.Y-Drone[b].positionMerc.Y),2)))/SF(latit);
    }

    // Donne le "Scale Factor" (utilisé ailleurs)
    double Flocking::SF(double lati)
    {
        double e = 0.0818191910428158;
        double l = lati*M_PI/180;
        double sinLati = sin(l);
        return sqrt(1-e*e*sinLati*sinLati)/cos(l);
    }

    // Publie la destination du drone (En point GEO)
    void Flocking::Publication(int NumeroDrone){
        Drone[NumeroDrone].WPmessage.time=0;
        Drone[NumeroDrone].WPmessage.pos_acc=2500;
        Drone[NumeroDrone].WPmessage.wp_number=1;
        Drone[NumeroDrone].WPmessage.properties=0x17;
        Drone[NumeroDrone].WPmessage.max_speed=vitesse;
        Drone[NumeroDrone].WPmessage.height = Drone[NumeroDrone].hauteur;
        Drone[NumeroDrone].WPmessage.yaw = Drone[NumeroDrone].angle;
        Drone[NumeroDrone].WPmessage.X = Drone[NumeroDrone].destinationGeo.latitude;
        Drone[NumeroDrone].WPmessage.Y = Drone[NumeroDrone].destinationGeo.longitude;
        Drone[NumeroDrone].WPmessage.chksum = (short) 0xAAAA + Drone[NumeroDrone].WPmessage.yaw + Drone[NumeroDrone].WPmessage.height + Drone[NumeroDrone].WPmessage.time + Drone[NumeroDrone].WPmessage.X + Drone[NumeroDrone].WPmessage.Y + Drone[NumeroDrone].WPmessage.max_speed + Drone[NumeroDrone].WPmessage.pos_acc + Drone[NumeroDrone].WPmessage.properties + Drone[NumeroDrone].WPmessage.wp_number;
        Drone[NumeroDrone].WPpublisher.publish(Drone[NumeroDrone].WPmessage);
    }
    void Flocking::ChangeAngle(int NumeroDrone, int angle){
        Drone[NumeroDrone].WPmessage.time=0;
        Drone[NumeroDrone].WPmessage.pos_acc=2500;
        Drone[NumeroDrone].WPmessage.wp_number=1;
        Drone[NumeroDrone].WPmessage.properties=0x17;
        Drone[NumeroDrone].WPmessage.max_speed=vitesse;
        Drone[NumeroDrone].WPmessage.yaw = angle;
        Drone[NumeroDrone].WPmessage.height = Drone[NumeroDrone].hauteur;
        Drone[NumeroDrone].WPmessage.X = Drone[NumeroDrone].positionGeo.latitude;
        Drone[NumeroDrone].WPmessage.Y = Drone[NumeroDrone].positionGeo.longitude;
        Drone[NumeroDrone].WPmessage.chksum = (short) 0xAAAA + Drone[NumeroDrone].WPmessage.yaw + Drone[NumeroDrone].WPmessage.height + Drone[NumeroDrone].WPmessage.time + Drone[NumeroDrone].WPmessage.X + Drone[NumeroDrone].WPmessage.Y + Drone[NumeroDrone].WPmessage.max_speed + Drone[NumeroDrone].WPmessage.pos_acc + Drone[NumeroDrone].WPmessage.properties + Drone[NumeroDrone].WPmessage.wp_number;
        Drone[NumeroDrone].WPpublisher.publish(Drone[NumeroDrone].WPmessage);
    }




    // Fonction sortant la position géographique de destination grace à un angle et à la position du Leader
    coordinatesGeo Flocking::CalcPosGeo(coordinatesGeo pt, double angle, int NumSuiveur ){
        coordinatesMerc Merc = geoToMerca(pt);
        int X = Drone[NumSuiveur].DecalXY.X;
        int Y = Drone[NumSuiveur].DecalXY.Y;
        Merc.X = round(Merc.X+ X*(cos(angle))+Y*(sin(angle)));
        Merc.Y = round(Merc.Y+ Y*(cos(angle))-X*(sin(angle)));
        Drone[NumSuiveur].angle = angle;
        coordinatesGeo out = mercaToGeo(Merc);        
        return out;
    }
/*
// Fonction permettant de "hacher" le .launch afin de connaitre la liste de drone présents
void Parsing(string s)
{
    string delimiter = " ";
    string token;
    while ((pos = s.find(delimiter)) != std::string::npos)
    {
        MesDrones DroneTemp ;
        token = s.substr(0, pos);
        DroneTemp.numero = atoi(token.c_str()); //A checker ATOI
        s.erase(0, pos + delimiter.length());
        Drone.push_back(DroneTemp);
    }
}
*/
// Fonction permettant de "hacher" le .launch dans un vecteur afin de connaitre la liste de drone présents
std::vector<int> Flocking::VectorParsing(string s)
{
    string delimiter = ",";
    string token;
    size_t pos;
    std::vector<int> out;

    while ((pos = s.find(delimiter)) != std::string::npos)
    {
        token = s.substr(0, pos);
        out.push_back(atoi(token.c_str())); //A checker ATOI
        s.erase(0, pos + delimiter.length());
    }
    return out;
}
std::vector<coordinatesGeo> Flocking::VectorDest(string lati, string longi){
    vector<coordinatesGeo> out;
    vector<int> Vlati = VectorParsing(lati);
    vector<int> Vlongi = VectorParsing(longi);
long unsigned a = Vlati.size();
        for( long unsigned int i =0; i<= a; i++){
            coordinatesGeo Tempout;
            Tempout.latitude = Vlati[i];
            Tempout.longitude = Vlongi[i];
            out.push_back(Tempout);
        }
    return out;
}

void Flocking::EcartSup(int a){
    int minX = Drone[0].DecalXY.X;
    int minY = Drone[0].DecalXY.Y;
    int maxX = Drone[0].DecalXY.X;
    int maxY = Drone[0].DecalXY.Y;
long unsigned b = Drone.size();
    for (long unsigned int i=0; i< b; i++){
        if (Drone[i].DecalXY.X < minX ){
            minX = Drone[i].DecalXY.X;
        }
        if (Drone[i].DecalXY.X > maxX ){
            maxX = Drone[i].DecalXY.X;
        }
        if (Drone[i].DecalXY.Y < minY ){
            minY = Drone[i].DecalXY.X;
        }
        if (Drone[i].DecalXY.Y > maxY ){
            maxY = Drone[i].DecalXY.X;
        }
        Drone[a].DecalXY.X = round((minX + maxX)/2);
        Drone[a].DecalXY.Y = round((minY + maxY)/2);
    }
}

// Boolean pour savoir si les drones sont en attentes
/*
bool DronesPres()
{
    bool pres = true;
long unsigned a = Drone.size();
    for (int i=0; i<= a; i++){
        if (Drone[i].numero != 0){
            if (Drone[i].nav_stat = false){
                bool pres = false;
            }
        }
    }


    return pres;
}
*/

void Flocking::new_waypoint(){
    string s;
    coordinatesGeo pt;
    cout << "Latitude de destination ?\n";
    cin >> pt.latitude;
    cin.clear();
    cout << "Longitude de destination ?\n";
    cin >> pt.longitude;
    cin.clear();
    //pour utiliser le tableau de points
    if (pt.latitude < 100){
	vector<coordinatesGeo> temppt;
        temppt = VectorDest(VLatitude,VLongitude);
        pt = temppt[pt.latitude];
    }

    Drone[0].destinationGeo = pt;
    Drone[0].angle = CapDestination(0);
    ChangeAngle(0, Drone[0].angle);
long unsigned a = Drone.size();
    for (long unsigned int i=1; i<= a; i++)
    {
        Drone[i].destinationGeo = CalcPosGeo(Drone[0].positionGeo, Drone[0].angle,i);
        Drone[i].angle = Drone[0].angle;
    }
    cout << "Envoi de la mise en place ?\n";
    cin >> s;
    cin.clear();
    for (long unsigned int i=1; i<= a; i++)
    {
        Publication(i);
    }
    cout << "Mise en place...\n";
    cout << "Depart vers la nouvelle destination ?\n";
    cin >> s;
    cin.clear();
    for (long unsigned int i=0; i<= a; i++)
    {
        Drone[i].destinationGeo = CalcPosGeo(Drone[0].destinationGeo, Drone[0].angle,i);
    }
    for (long unsigned int i=0; i<= a; i++){
        Publication(i);
    }

}



int main(int argc, char **argv) {
	ros::init(argc, argv,"New_Flocking");
    Flocking flock;
    while(ros::ok())
    {
    ros::spinOnce();
    flock.new_waypoint();
    }
}


