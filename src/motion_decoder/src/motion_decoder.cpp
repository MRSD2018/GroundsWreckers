#include <ros/ros.h>
#include <motion_decoder/image_converter.hpp>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>


ImageConverter* ic;
ros::Subscriber sub;

void apriltag_detection_callback(const apriltags_ros::AprilTagDetectionArray msg)
{
  //ROS_INFO("In subscribe\n");
  //TODO: Parse message and publish transforms as apriltag_tf and camera
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  if (msg.detections.size() > 0)
  {
    transform.setOrigin(tf::Vector3(msg.detections[0].pose.pose.position.x, msg.detections[0].pose.pose.position.y, msg.detections[0].pose.pose.position.z) );
    q.setRotation(tf::Vector3(msg.detections[0].pose.pose.orientation.x, msg.detections[0].pose.pose.orientation.y, msg.detections[0].pose.pose.orientation.z), 
      msg.detections[0].pose.pose.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera", "april_tf"));
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  
  ros::NodeHandle n;
  //TODO: Add a subscriber to get the AprilTag detections The callback function skeleton is given.
  ImageConverter converter;
  ic = &converter;
  sub = n.subscribe("tag_detections", 1000, apriltag_detection_callback);

  ros::Rate loop_rate(50);
  ROS_INFO("In main\n");
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
