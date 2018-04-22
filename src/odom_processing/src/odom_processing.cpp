#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sstream>

ros::Subscriber sub;
ros::Publisher pub;
std_msgs::String state_string;
unsigned int state;

void odom_callback(const nav_msgs::Odometry msg)
{
  //TODO: this entire thing
  ++state;
  std::stringstream ss;
  ss << state;
  state_string.data = ss.str();
  pub.publish(state_string);

  double vx = msg.twist.twist.linear.x;
  double l = 0.508;
  double omega = msg.twist.twist.angular.z;

  double Vl = vx - ( omega * (l/2) );
  double Vr = vx + ( omega * (l/2) );

  std::ofstream out("odom.csv", std::ios::app);
  out << state << "," << Vl << "," << Vr << std::endl;
  out.close();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_processing");

  ros::NodeHandle n;

  sub = n.subscribe("/grudsby/odometry", 1000, odom_callback);
  pub = n.advertise<std_msgs::String>("state_counter", 1000);

  state = 0;

  ros::Rate loop_rate(50);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
} 
