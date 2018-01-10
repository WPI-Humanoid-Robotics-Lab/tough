#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "src_task_common/left_image_inverter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "left_image_inverter");
  ros::NodeHandle nh;

  LeftImageInverter getImage(nh);

  getImage.subscribeImage();

  ros::spin();
  
}
