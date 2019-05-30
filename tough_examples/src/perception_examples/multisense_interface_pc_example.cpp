#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <tough_perception_common/MultisenseInterface.h>
#include <opencv2/core.hpp>
#include <opencv/highgui.h>

using namespace tough_perception;

void printStatus(const std::string &image_name, bool status)
{
    ROS_INFO("%s status %s", image_name.c_str(), status ? "true" : "false");
}

void printAssemblerStatus(const LASER_ASSEMBLER_STATUS &assembler_state_)
{
    if (assembler_state_ == LASER_ASSEMBLER_STATUS::ACTIVE)
        ROS_INFO("assembler active");
    else if (assembler_state_ == LASER_ASSEMBLER_STATUS::PAUSE)
        ROS_INFO("assembler paused");
    else if (assembler_state_ == LASER_ASSEMBLER_STATUS::RESET)
        ROS_INFO("assembler reset");
    else
        ROS_INFO("assembler state unknown");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_multisense_pointcloud");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    bool status;
    LASER_ASSEMBLER_STATUS assembler_state_;
    PointCloud::Ptr point_cloud; // this is pcl::PointCloud<PointXYZ>::Ptr

    MultisenseInterfacePtr multisense_handler;
    multisense_handler = MultisenseInterface::getMultisenseInterface(nh);
    multisense_handler->setSpindleSpeed(2.0);

    MultisenseCameraModel cam_model;
    multisense_handler->getCameraInfo(cam_model);

    ROS_INFO("is Multisense active %s", multisense_handler->isSensorActive() ? "true" : "false");
    cam_model.printCameraConfig();

    assembler_state_ = multisense_handler->getAssemblerStatus();
    printAssemblerStatus(assembler_state_);

    multisense_handler->setAssemblerStatus(LASER_ASSEMBLER_STATUS::RESET);
    ros::Duration(1.0).sleep();
    assembler_state_ = multisense_handler->getAssemblerStatus();
    printAssemblerStatus(assembler_state_);

    multisense_handler->setAssemblerStatus(LASER_ASSEMBLER_STATUS::ACTIVE);
    ros::Duration(1.0).sleep();
    assembler_state_ = multisense_handler->getAssemblerStatus();
    printAssemblerStatus(assembler_state_);

    std::cout << __LINE__ << std::endl;
    status = multisense_handler->getLaserPointCloud(point_cloud);
    std::cout << __LINE__ << std::endl;
    printStatus("Laser point cloud", status);
    if (status)
        ROS_INFO_STREAM("Laser point cloud size: " << point_cloud->size());
    else
        ROS_INFO_STREAM("check if field laser assembler is running");

    spinner.stop();
    return 0;
}
