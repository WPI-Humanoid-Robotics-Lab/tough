#include "val_perception_bringup/left_image_inverter.h"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

namespace perception_common{

LeftImageInverter::LeftImageInverter(ros::NodeHandle& nh):it(nh){
  pub = it.advertise("/image",1);
}

void LeftImageInverter::subscribeImage(){
  sub = it.subscribe("/multisense/camera/left/image_rect_color",1, &LeftImageInverter::getLeftImageCB,this);
}

void LeftImageInverter::getLeftImageCB(const sensor_msgs::ImageConstPtr& msg){
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
    cv::Mat invertedLeftImage;
    cv::Mat leftImage = cv_bridge::toCvShare(msg, "bgr8")->image;
    //invert left image
    cv::flip(leftImage, invertedLeftImage, 0);
    sensor_msgs::ImageConstPtr image = cv_bridge::CvImage(std_msgs::Header(), "bgr8",invertedLeftImage).toImageMsg();

    pub.publish(image);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  }
LeftImageInverter::~LeftImageInverter(){
}
}
