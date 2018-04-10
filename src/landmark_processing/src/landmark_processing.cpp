#include <ros/ros.h>
#include <motion_decoder/image_converter.hpp>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <iostream>
#include <fstream>

ros::Subscriber sub;
ImageConverter *ic;

void apriltag_detection_callback(const apriltags_ros::AprilTagDetectionArray msg)
{
 //TODO: Figure out timestep rate
 //TODO: Round apriltag timestamp to closest timstep
  
  //get the data from the message
  unsigned int landmark_id = msg.detections[0].id;
  double delta_x = msg.detection[0].pose.pose.position.x + 0.93209511; //hardcode the offset from camera to base because easier
  double delta_y = msg.detection[0].pose.pose.position.y;
  ros::Time timestamp_sec = msg.detection[0].pose.header.stamp.sec; //TODO: stamp.sec?? stamp.nsec??
  ros::Time timestamp_nano = msg.detection[0].pose.header.stamp.nsec; 

  //get timestamp into correct format 


  //Dump to csv
  std::ofstream out("landmarks.csv", std::ios::app);
  out << timestamp << "," <<  landmark_i d<< "," << x << "," << y  << std::endl;
  out.close();

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");

  ros::NodeHandle n;

  ImageCoverter converter;

  ic = &converter;
  sub = n.subscribe("tag_detections", 1000, apriltag_detection_callback);

  ros::Rate loop_rate(50);
  
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
