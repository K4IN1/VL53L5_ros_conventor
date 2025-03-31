#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"


ros::Publisher pub;
void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  try
  {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    // ROS_INFO("Input value at (0,0): %d", cv_ptr->image.at<uint16_t>(0, 0));
    cv::Mat float_image;
    cv_ptr->image.convertTo(float_image, CV_32F ,0.001);
    sensor_msgs::ImagePtr converted_msg = cv_bridge::CvImage(msg->header, 
              sensor_msgs::image_encodings::TYPE_32FC1, 
              float_image).toImageMsg();
    // ROS_INFO("Output value at (0,0): %f", float_image.at<float>(0, 0));
    pub.publish(converted_msg);
      
    ROS_DEBUG("Image converted and published successfully");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("CV Bridge error: %s", e.what());
  }
  catch (std::exception& e)
  {
    ROS_ERROR("Error converting image: %s", e.what());
  }

  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::Rate loop_rate(15);
  pub = nh.advertise<sensor_msgs::Image>("output", 15);
  ros::Subscriber sub = nh.subscribe("input", 15, imageCallback);
  ROS_INFO("conventer is running");
  ros::spin();
  return 0;
}