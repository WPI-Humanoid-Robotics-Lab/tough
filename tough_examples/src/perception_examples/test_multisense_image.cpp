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
  ROS_INFO("Press any key to continue");
  cv::waitKey(0);
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

  ROS_INFO("is Multisense active %s", imageHandler->isSensorActive() ? "true" : "false");

  std::cout << "-" << std::endl;
  std::cout << "[Height]" << cam_model.height << std::endl;
  std::cout << "[width]" << cam_model.width << std::endl;
  std::cout << "[fx]" << cam_model.fx << std::endl;
  std::cout << "[fy]" << cam_model.fy << std::endl;
  std::cout << "[cx]" << cam_model.cx << std::endl;
  std::cout << "[cy]" << cam_model.cy << std::endl;
  std::cout << "[K]\n"
            << cam_model.K << std::endl;
  std::cout << "[P]\n"
            << cam_model.P << std::endl;
  std::cout << "[distortion_model] " << cam_model.distortion_model << std::endl;
  std::cout << "-" << std::endl;

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

  imageHandler->shutdown();
  ROS_INFO("is Multisense active %s", imageHandler->isSensorActive() ? "true" : "false");
  imageHandler->start();
  ROS_INFO("is Multisense active %s", imageHandler->isSensorActive() ? "true" : "false");

  status = imageHandler->getDisparity(image, true);
  PRINT_STATUS(status);
  if (status)
    show_image(image, "Disparity Image from stereo_msg");

  return 0;
}
