#include <ros/ros.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <iostream>
#include <fstream>
#include <ros/console.h>
#include <std_msgs/String.h>

ros::Subscriber land_sub;
ros::Subscriber state_sub;
std::string state;

void apriltag_detection_callback(const apriltags_ros::AprilTagDetectionArray msg)
{
 //TODO: Figure out timestep rate
 //TODO: Round apriltag timestamp to closest timstep
  

  std::ofstream out("landmarks.csv", std::ios::app);
  for(auto &tag : msg.detections)
  {  
    //get the data from the message
    int landmark_id = tag.id;

    //apriltags says z is forward, but we like it better when x is forward
    double delta_x = tag.pose.pose.position.z;// + 0.93209511; //hardcode the offset from camera to base because easier
    double delta_y = tag.pose.pose.position.x;
    double delta_z = 0;
    //std::cout << "Tag: " << landmark_id << ", " << delta_x << ", " << delta_y << ", " << delta_z << std::endl;

    //double timestamp_sec = msg.detections[0].pose.header.stamp.sec; //TODO: stamp.sec?? stamp.nsec??
    //double timestamp_nano = msg.detections[0].pose.header.stamp.nsec;
    //std::cout << "Time: " << timestamp_sec << "\tnano: " << timestamp_nano << std::endl;

    //get timestamp into correct format 

    //Dump to csv
    out << state << "," <<  landmark_id << "," << delta_x << "," << delta_y  << "," << delta_z << std::endl;
  }
  out.close();
}

void state_callback(const std_msgs::String msg)
{
  state = msg.data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "landmark_processing");

  ros::NodeHandle n;

  land_sub = n.subscribe("tag_detections", 1000, apriltag_detection_callback);
  state_sub = n.subscribe("state_counter", 1000, state_callback);

  //std::cout.precision(10);

  ros::Rate loop_rate(50);
  
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

