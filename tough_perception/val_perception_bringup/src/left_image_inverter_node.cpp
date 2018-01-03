#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "val_perception_bringup/left_image_inverter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "left_image_inverter");
  ros::NodeHandle nh;

  tough_perception_common::LeftImageInverter getImage(nh);

  getImage.subscribeImage();

  ros::spin();
  
}
