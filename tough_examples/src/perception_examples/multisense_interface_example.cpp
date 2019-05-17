#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <tough_perception_common/MultisenseInterface.h>
#include <opencv2/core.hpp>
#include <opencv/highgui.h>

void printStatus(const std::string &image_name, bool status)
{
  ROS_INFO("%s status %s", image_name.c_str(), status ? "true" : "false");
}

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
  ros::AsyncSpinner spinner(1);
  spinner.start();
  bool status;

  tough_perception::MultisenseInterfacePtr imageHandler;
  imageHandler = tough_perception::MultisenseInterface::getMultisenseInterface(nh);
  imageHandler->setSpindleSpeed(2.0);
  cv::Mat RGB_image;
  cv::Mat depth_image;
  cv::Mat cost_image;
  cv::Mat disparity_image;

  tough_perception::MultisenseCameraModel cam_model;
  imageHandler->getCameraInfo(cam_model);

  ROS_INFO("is Multisense active %s", imageHandler->isSensorActive() ? "true" : "false");

  cam_model.printCameraConfig();
  status = imageHandler->getImage(RGB_image);
  printStatus("RGB Image", status);
  if (status)
    show_image(RGB_image, "RGB Image");

  // gazebo doesn't provide depth images
  status = imageHandler->getDepthImage(depth_image);
  printStatus("Depth Image", status);
  scale_depth_image(depth_image);
  if (status)
    show_image(depth_image, "Depth Image");

  // gazebo doesn't provide cost images
  status = imageHandler->getCostImage(cost_image);
  printStatus("Cost Image", status);
  if (status)
    show_image(cost_image, "Cost Image");

  // gazebo doesn't provide disparity using images types
  status = imageHandler->getDisparity(disparity_image);
  printStatus("Disparity Image from sensor_msg", status);
  if (status)
    show_image(disparity_image, "Disparity Image from sensor_msg");

  // When using gazebo, call getDisparity from stereo message
  bool use_stereo_msg = true;
  status = imageHandler->getDisparity(disparity_image, use_stereo_msg);
  printStatus("Disparity Image from stereo_msg", status);
  if (status)
    show_image(disparity_image, "Disparity Image from stereo_msg");

  imageHandler->shutdown();
  ROS_INFO("is Multisense active %s", imageHandler->isSensorActive() ? "true" : "false");
  imageHandler->start();
  ROS_INFO("is Multisense active %s", imageHandler->isSensorActive() ? "true" : "false");

  status = imageHandler->getDisparity(disparity_image, use_stereo_msg);
  printStatus("Disparity Image from stereo_msg", status);
  if (status)
    show_image(disparity_image, "Disparity Image from stereo_msg");

  tough_perception::StereoPointCloudColor::Ptr cloud = tough_perception::StereoPointCloudColor::Ptr(new tough_perception::StereoPointCloudColor());
  status = imageHandler->getStereoData(disparity_image, RGB_image, cloud);
  printStatus("Stereo point cloud", status);
  if (status)
  {
    std::cout << cloud->size() << std::endl;
    // std::cout << cloud->points.data << std::endl;
  }

  spinner.stop();
  return 0;
}
