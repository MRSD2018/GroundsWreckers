#include <ros/ros.h>
#include <motion_decoder/image_converter.hpp>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>


ros::Subscriber sub;
ImageConverter *ic;

void apriltag_detection_callback(const apriltags_ros::AprilTagDetectionArray msg)
{
 //TODO: Transform measurement to be from Grudsby Base rather than Grudsby Camera
 //TODO: Figure out timestep rate
 //TODO: Round apriltag timestamp to closest timstep
 //TODO: Format and dump out landmark info 
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
