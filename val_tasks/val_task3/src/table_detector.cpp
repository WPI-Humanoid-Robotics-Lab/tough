#include <val_task3/table_detector.h>

table_detector::table_detector(ros::NodeHandle nh) : nh_(nh), point_cloud_combiner_(nh, "/leftFoot", "/left_camera_frame")
{
}


bool table_detector::find_table() {
    src_perception::LaserPointCloud::Ptr cloud;

    if (point_cloud_combiner_.giveLaserCloudWrtLFoot(cloud)) {
        ROS_INFO("got a point cloud");
    } else {
        ROS_INFO("no point cloud");
    }
}
