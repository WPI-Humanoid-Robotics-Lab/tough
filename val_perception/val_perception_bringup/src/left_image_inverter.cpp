#include "val_perception_bringup/left_image_inverter.h"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
using namespace cv;
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


    Mat invertedLeftImage;
    Mat leftImage = cv_bridge::toCvShare(msg, "bgr8")->image;
    //invert left image
    flip(leftImage, invertedLeftImage, -1);
    int w = invertedLeftImage.size().width;
    int h = invertedLeftImage.size().height;
    // horizontal line
    line(invertedLeftImage,Point(0,h/2),Point(w,h/2),Scalar(0,255,0),2,8);
    // vertical line
    line(invertedLeftImage,Point(w/2,0),Point(w/2,h),Scalar(0,255,0),2,8);
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
