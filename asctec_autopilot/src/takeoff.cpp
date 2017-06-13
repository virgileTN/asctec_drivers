#include "asctec_autopilot/takeoff.h"

using namespace std;

//Var static
int thrust_th = 0;
int thrust    = 0;
int v_z       = 0;
int z         = 0;

//Callbacks
void Callback(asctec_msgs::ControllerOutput msg)
{
	thrust = msg.thrust;
}

void Callback3(asctec_msgs::IMUCalcData msg)
{
	v_z   = msg.speed_z;
	z     = msg.height;
}

//Controler sigaction
void ctrlc_gestion(int signal)
{
	stop=true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_node");

	ros::NodeHandle n;

  //Topics
	publisher         = n.advertise<asctec_msgs::CtrlInput>("/asctec_msgs/CtrlInput6", 1);
	subscriber_thrust = n.subscribe("/asctec/CONTROLLER_OUTPUT6", 1, Callback, ros::TransportHints().tcpNoDelay());
	subscriber_IMU    = n.subscribe("/asctec/IMU_CALCDATA6", 1, Callback3, ros::TransportHints().tcpNoDelay());
  
  //Paramètres
	int z_th; //altitude au repos
	n.getParam("z_th", z_th);
	int v_z_th; //accélération de décollage
	n.getParam("v_z_th", v_z_th);
	int pas; //incrémentation du thrust
	n.getParam("pas", pas);
  
  //z0 repère initial et relatif d'altitude
	int z0 = z;
	
  //Sigaction 
	struct sigaction s_action;
	s_action.sa_handler = ctrlc_gestion;
	sigemptyset(&s_action.sa_mask);
	s_action.sa_flags = 0;
	sigaction(SIGINT, &s_action, NULL);

	stop=false;

  while (!stop)
  {
  asctec_msgs::CtrlInput msg;
  msg.ctrl = 3;
  //Contrôle du décollage
	while( z-z0<z_th ){
		while( v_z < v_z_th ){
			thrust_th  = thrust_th + pas;
			msg.thrust = thrust_th;
			publisher.publish(msg);
			ros::Duration(0.5).sleep();
		}
	}
  //Contrôle de l'altitude au repos après décollage
	while( v_z!=0 ){
		if( v_z>0 ){
			thrust     = thrust - pas;
			msg.thrust = thrust;
			publisher.publish(msg);
			ros::Duration(0.5).sleep();
		}else if( v_z<0 ){
			thrust     = thrust + pas;
			msg.thrust = thrust;
			publisher.publish(msg);
			ros::Duration(0.5).sleep();
		}
	}
  }	
} 
