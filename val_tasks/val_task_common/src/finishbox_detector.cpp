#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <cv_bridge/cv_bridge.h>
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


finishbox_detector::finishbox_detector(ros::NodeHandle &nh)
{
    imageSub_ = nh.subscribe("/multisense/camera/left/image_rect_color", 10, &finishbox_detector::imageCallback, this, (new ros::TransportHints())->unreliable());
    imagePub_ = nh.advertise<sensor_msgs::Image>("testImage",10);
}


finishbox_detector::~finishbox_detector()
{

}


void finishbox_detector::imageCallback(const sensor_msgs::Image &in_img)
{

}


void finishbox_detector::detectFinishBox(sensor_msgs::Image &img)
{

}
