#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>
// #include <tough_perception_common/MultisenseImage.h>
#include <tough_perception_common/MultisenseImageInterface.h>
#include <opencv2/core.hpp>
#include <opencv/highgui.h>

#define PRINT_STATUS(status) ROS_INFO("image status %s", status ? "true" : "false")

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

  tough_perception::MultisenseCameraModel cam_model;
  imageHandler->getCameraInfo(cam_model);

  ROS_INFO_STREAM("[Height]" << cam_model.height);
  ROS_INFO_STREAM("[width]" << cam_model.width);
  ROS_INFO_STREAM("[fx]" << cam_model.fx);
  ROS_INFO_STREAM("[fy]" << cam_model.fy);
  ROS_INFO_STREAM("[cx]" << cam_model.cx);
  ROS_INFO_STREAM("[cy]" << cam_model.cy);
  ROS_INFO_STREAM("[K]\n"
                  << cam_model.K);
  ROS_INFO_STREAM("[P]\n"
                  << cam_model.P);
  ROS_INFO_STREAM("[distortion_model] " << cam_model.distortion_model);

  status = imageHandler->getImage(image);
  PRINT_STATUS(status);
  if (status)
    show_image(image, "RGB Image");

  status = imageHandler->getDepthImage(image);
  PRINT_STATUS(status);
  scale_depth_image(image);
  if (status)
    show_image(image, "Depth Image");

  status = imageHandler->getCostImage(image);
  PRINT_STATUS(status);
  if (status)
    show_image(image, "Cost Image");

  status = imageHandler->getDisparity(image);
  PRINT_STATUS(status);
  if (status)
    show_image(image, "Disparity Image from sensor_msg");

  status = imageHandler->getDisparity(image, true);
  PRINT_STATUS(status);
  if (status)
    show_image(image, "Disparity Image from stereo_msg");

  return 0;
}
