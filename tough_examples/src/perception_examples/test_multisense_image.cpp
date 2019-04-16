#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>
// #include <tough_perception_common/MultisenseImage.h>
#include <tough_perception_common/MultisenseImageInterface.h>
#include <opencv2/core.hpp>
#include <opencv/highgui.h>

void printStatus(const std::string& image_name, bool status)
{
  ROS_INFO("%s status %s", image_name.c_str(), status ? "true" : "false");
}

void show_image(cv::Mat& image, std::string name)
{
  cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
  cv::imshow(name, image);
  ROS_INFO("Press any key to continue");
  cv::waitKey(0);
  cv::destroyWindow(name);
  ros::Duration(0.5).sleep();  // wait some time for the window to destroy cleanly.
}

void scale_depth_image(cv::Mat& depth_in_cv32F)
{
  double min;
  double max;
  cv::minMaxIdx(depth_in_cv32F, &min, &max);
  cv::Mat adjMap;
  cv::convertScaleAbs(depth_in_cv32F, adjMap, 255 / max);
  depth_in_cv32F = adjMap.clone();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_multisense_image");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  bool status;

  tough_perception::MultisenseImageInterfacePtr imageHandler;
  imageHandler = tough_perception::MultisenseImageInterface::getMultisenseImageInterface(nh);
  imageHandler->setSpindleSpeed(2.0);
  cv::Mat image;

  tough_perception::MultisenseCameraModel cam_model;
  imageHandler->getCameraInfo(cam_model);

  ROS_INFO("is Multisense active %s", imageHandler->isSensorActive() ? "true" : "false");

  cam_model.printCameraConfig();
  status = imageHandler->getImage(image);
  printStatus("RGB Image", status);
  if (status)
    show_image(image, "RGB Image");

  // gazebo doesn't provide depth images
  status = imageHandler->getDepthImage(image);
  printStatus("Depth Image", status);
  scale_depth_image(image);
  if (status)
    show_image(image, "Depth Image");

  // gazebo doesn't provide cost images
  status = imageHandler->getCostImage(image);
  printStatus("Cost Image", status);
  if (status)
    show_image(image, "Cost Image");

  // gazebo doesn't provide disparity using images types
  status = imageHandler->getDisparity(image);
  printStatus("Disparity Image from sensor_msg", status);
  if (status)
    show_image(image, "Disparity Image from sensor_msg");

  // When using gazebo, call getDisparity from stereo message
  bool use_stereo_msg = true;
  status = imageHandler->getDisparity(image, use_stereo_msg);
  printStatus("Disparity Image from stereo_msg", status);
  if (status)
    show_image(image, "Disparity Image from stereo_msg");

  imageHandler->shutdown();
  ROS_INFO("is Multisense active %s", imageHandler->isSensorActive() ? "true" : "false");
  imageHandler->start();
  ROS_INFO("is Multisense active %s", imageHandler->isSensorActive() ? "true" : "false");

  status = imageHandler->getDisparity(image, use_stereo_msg);
  printStatus("Disparity Image from stereo_msg", status);
  if (status)
    show_image(image, "Disparity Image from stereo_msg");

  spinner.stop();
  return 0;
}
