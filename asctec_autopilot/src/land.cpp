#include "asctec_autopilot/land.h"

using namespace std;

//Var static
int thrust_th = 0;
int thrust    = 0;
int v_z       = 0;


//Callbacks
void Callback(asctec_msgs::ControllerOutput msg)
{
	thrust = msg.thrust;
}

void Callback3(asctec_msgs::IMUCalcData msg)
{
	v_z    = msg.speed_z;
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
	int pas; //incrémentation du thrust
	n.getParam("pas", pas);
  
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
		thrust_th = thrust;
		//Contrôle de la décente "en douceur"
		while( !( v_z <0 ) ){
			thrust_th  = thrust_th - pas;
			msg.thrust = thrust_th;
			publisher.publish(msg);
			ros::Duration(0.5).sleep();
		}
	}	
} 
