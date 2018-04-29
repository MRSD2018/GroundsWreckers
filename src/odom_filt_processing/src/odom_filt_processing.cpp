#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <tf/transform_datatypes.h>

ros::Subscriber sub;
ros::Publisher pub;
std_msgs::String state_string;
unsigned int state;
double prev_x = 0;
double prev_y = 0;
double prev_theta = 0;

void odom_callback(const nav_msgs::Odometry msg)
{
  ++state;
  std::stringstream ss;
  ss << state;
  state_string.data = ss.str();
  pub.publish(state_string);

  double dx = msg.pose.pose.position.x - prev_x;
  double dy = msg.pose.pose.position.y - prev_y;

  prev_x = msg.pose.pose.position.x;
  prev_y = msg.pose.pose.position.y;

  tf::Quaternion q(
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,  
      msg.pose.pose.orientation.z,  
      msg.pose.pose.orientation.w);  

  tf::Matrix3x3 m(q);

  double r, p, theta;
  m.getRPY(r, p, theta);

  double d_theta = theta - prev_theta;
  prev_theta = theta;

  std::ofstream out("odom_filterd.csv", std::ios::app);
  out << state << "," << dx << "," << dy << "," << d_theta << std::endl;
  out.close();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_filt_processing");

  ros::NodeHandle n;

  sub = n.subscribe("/odometry/filtered", 1000, odom_callback);
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
