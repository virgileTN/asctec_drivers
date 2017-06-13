#include "ros/ros.h"
#include <iostream>
#include "asctec_msgs/LLStatus.h"
#include "asctec_msgs/IMUCalcData.h"

int hautBARO1, hautBARO2, hautBARO3, hautBARO4, hautBARO5, hautBARO6;
int batterie1, batterie2, batterie3, batterie4, batterie5, batterie6;

bool one;
void Hauteur1(asctec_msgs::IMUCalcData msg)
{
  hautBARO1=msg.height;
}
void Hauteur2(asctec_msgs::IMUCalcData msg)
{
  hautBARO2=msg.height;
}
void Hauteur3(asctec_msgs::IMUCalcData msg)
{
  hautBARO3=msg.height;
}
void Hauteur4(asctec_msgs::IMUCalcData msg)
{
  hautBARO4=msg.height;
}
void Hauteur5(asctec_msgs::IMUCalcData msg)
{
  hautBARO5=msg.height;
}
void Hauteur6(asctec_msgs::IMUCalcData msg)
{
  hautBARO6=msg.height;
}
void Callback1(asctec_msgs::LLStatus msg)
{
  batterie1 = msg.battery_voltage_1;
}
void Callback2(asctec_msgs::LLStatus msg)
{
  batterie2 = msg.battery_voltage_1;
}
void Callback3(asctec_msgs::LLStatus msg)
{
  batterie3 = msg.battery_voltage_1;
}
void Callback4(asctec_msgs::LLStatus msg)
{
  batterie4 = msg.battery_voltage_1;
}
void Callback5(asctec_msgs::LLStatus msg)
{
  batterie5 = msg.battery_voltage_1;
}
void Callback6(asctec_msgs::LLStatus msg)
{
  batterie6 = msg.battery_voltage_1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "batterie");
  ros::NodeHandle n;
  ros::Subscriber subscriber1 = n.subscribe("/asctec/LL_STATUS1", 1, Callback1, ros::TransportHints().tcpNoDelay());  
  ros::Subscriber subscriber2 = n.subscribe("/asctec/LL_STATUS2", 1, Callback2, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subscriber3 = n.subscribe("/asctec/LL_STATUS3", 1, Callback3, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subscriber4 = n.subscribe("/asctec/LL_STATUS4", 1, Callback4, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subscriber5 = n.subscribe("/asctec/LL_STATUS5", 1, Callback5, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subscriber6 = n.subscribe("/asctec/LL_STATUS6", 1, Callback6, ros::TransportHints().tcpNoDelay());
ros::Subscriber subhaut1 = n.subscribe("/asctec/IMU_CALCDATA1", 1, Hauteur1, ros::TransportHints().tcpNoDelay());  
ros::Subscriber subhaut2 = n.subscribe("/asctec/IMU_CALCDATA2", 1, Hauteur2, ros::TransportHints().tcpNoDelay());  
ros::Subscriber subhaut3 = n.subscribe("/asctec/IMU_CALCDATA3", 1, Hauteur3, ros::TransportHints().tcpNoDelay());  
ros::Subscriber subhaut4 = n.subscribe("/asctec/IMU_CALCDATA4", 1, Hauteur4, ros::TransportHints().tcpNoDelay());  
ros::Subscriber subhaut5 = n.subscribe("/asctec/IMU_CALCDATA5", 1, Hauteur5, ros::TransportHints().tcpNoDelay());  
ros::Subscriber subhaut6 = n.subscribe("/asctec/IMU_CALCDATA6", 1, Hauteur6, ros::TransportHints().tcpNoDelay());  

  ros::Rate loop_rate(10);

  one=false;

  while(ros::ok())
  {
    loop_rate.sleep();

    ros::spinOnce();

    system("clear");
    std::cout << "firefly1 : " << (double) batterie1/1000 << " hauteur : " << (double) hautBARO1/1000 << "\n";
    std::cout << "firefly2 : " << (double) batterie2/1000 << " hauteur : " << (double) hautBARO2/1000 << "\n";
    std::cout << "firefly3 : " << (double) batterie3/1000 << " hauteur : " << (double) hautBARO3/1000 << "\n";
    std::cout << "firefly4 : " << (double) batterie4/1000 << " hauteur : " << (double) hautBARO4/1000 << "\n";
    std::cout << "firefly5 : " << (double) batterie5/1000 << " hauteur : " << (double) hautBARO5/1000 << "\n";
    std::cout << "firefly6 : " << (double) batterie6/1000 << " hauteur : " << (double) hautBARO6/1000 << "\n";

    bool critic=(batterie1<10300 && batterie1!=0) || (batterie2<10300 && batterie2!=0) || (batterie3<10300 && batterie3!=0) || (batterie4<10300 && batterie4!=0) || (batterie5<10300 && batterie5!=0) || (batterie6<10300 && batterie6!=0);
    if(!one&&critic) {
      system("aplay -c 1 -q -t wav son.wav&");
      one=true;
    }
  }

  return 0;
}
