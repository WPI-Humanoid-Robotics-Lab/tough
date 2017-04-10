#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "val_task_common/finishbox_detector.h"
#include "val_common/val_common_names.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  finishbox_detector fd(n);
  ros::spin();

  return 0;
}


finishbox_detector::finishbox_detector(ros::NodeHandle nh):nh_(nh), it_(nh)
{
    imageSub_ = it_.subscribe(VAL_COMMON_NAMES::RECTIFIED_IMAGE_TOPIC ,1,&finishbox_detector::imageCallback,this,image_transport::TransportHints("compressed"));
//    imagePub_ = nh_.advertise<sensor_msgs::Image>("testImage",10);
}


finishbox_detector::~finishbox_detector()
{

}


void finishbox_detector::imageCallback(sensor_msgs::ImageConstPtr &in_img)
{

}


void finishbox_detector::detectFinishBox(sensor_msgs::Image &img)
{

}
