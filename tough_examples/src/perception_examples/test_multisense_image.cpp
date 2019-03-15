#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>
// #include <tough_perception_common/MultisenseImage.h>
#include <tough_perception_common/MultisenseImageInterface.h>
#include <opencv2/core.hpp>
#include <opencv/highgui.h>

void show_image(cv::Mat &image, std::string name)
{
  cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
  cv::imshow(name, image);
  ROS_INFO("Press any key continue");
  // while (cv::waitKey(1) != 27 && cv::waitKey(1) != 'q')
  //   ;
  cv::waitKey(0);
  ROS_INFO("closing window");
  cv::destroyWindow(name);
  ros::Duration(0.5).sleep(); // wait some time for the window to destroy cleanly.
}

void scale_depth_image(cv::Mat &depth_in_cv32F)
{
  double min;
  double max;
  cv::minMaxIdx(depth_in_cv32F, &min, &max);
  cv::Mat adjMap;
  cv::convertScaleAbs(depth_in_cv32F, adjMap, 255 / max);
  depth_in_cv32F = adjMap.clone();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_multisense_image");
  ros::NodeHandle nh;
  bool status;

  tough_perception::MultisenseImageInterfacePtr imageHandler;
  imageHandler = tough_perception::MultisenseImageInterface::getMultisenseImageInterface(nh);

  cv::Mat image;

  ROS_INFO_STREAM("[Height]" << imageHandler->getHeight()
                             << " [width]"
                             << imageHandler->getWidth());

  status = imageHandler->getImage(image);
  ROS_INFO("image status %s", status ? "true" : "false");
  if (status)
    show_image(image, "RGB Image");

  status = imageHandler->getDepthImage(image);
  ROS_INFO("image status %s", status ? "true" : "false");
  scale_depth_image(image);
  if (status)
    show_image(image, "Depth Image");

  status = imageHandler->getCostImage(image);
  ROS_INFO("image status %s", status ? "true" : "false");
  if (status)
    show_image(image, "Cost Image");

  status = imageHandler->getDisparity(image);
  ROS_INFO("image status %s", status ? "true" : "false");
  if (status)
    show_image(image, "Disparity Image from sensor_msg");

  status = imageHandler->getDisparity(image, true);
  ROS_INFO("image status %s", status ? "true" : "false");
  if (status)
    show_image(image, "Disparity Image from stereo_msg");

  return 0;
}
