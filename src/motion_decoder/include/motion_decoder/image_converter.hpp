#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <tf/transform_listener.h>


static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  

public:
  
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void setTagLocations(float x_det, float y_det, float z_det)
  {
    //TODO: Update tag locations
    //480. 640
    x_loc = (((x_det + 1) / 2.0) * 480.0) + 75.0;
    y_loc = (((y_det + 1) / 2.0) * 640.0) - 75.0;
    //std::cout << x_det << ", " << y_det << std::endl;
    //std::cout << x_loc << ", " << y_loc << std::endl;
    x_arr.push_back(x_loc);
    y_arr.push_back(y_loc);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //TODO: Draw circles at tag locations on image. 
    try
    {
      listener.lookupTransform("camera", "april_tf",  
                                ros::Time(0), transform);
      tf::Point point= transform.getOrigin();
      setTagLocations(point.x(),point.y(),point.z());
      //std::cout << cv_ptr->image.rows << ", " << cv_ptr->image.cols << std::endl;
      // Draw an example circle on the video stream
      for(int i = 0; i < x_arr.size(); i++)
      {
        cv::circle(cv_ptr->image, cv::Point(x_arr[i], y_arr[i]), 10, CV_RGB(255,0,0));
      }
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
    }

       // Output modified video stream
      // Update GUI Window
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(3);
  
      // TODO:Output modified video stream
      // Convert the modified frames into sensor_msgs::Image message and publish it using image_pub
      image_pub_.publish(cv_ptr->toImageMsg());
  }

private:
  float x_loc ,y_loc;
  std::vector<float> x_arr;
  std::vector<float> y_arr;
  tf::TransformListener listener;
  tf::StampedTransform transform;
};
