#include <perception_common/laser2point_cloud.h>
#include <perception_common/perception_common_names.h>
#include "/home/sumanth/catkin_ws/src/space_robotics_challenge/val_description/include/val_description/val_description_names.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "laser2point_cloud");
    ros::NodeHandle n;
    Laser2PointCloud laser2point(n, PERCEPTION_COMMON_NAMES::MULTISENSE_LASER_SCAN_TOPIC, DESCRIPTION_COMMON_NAMES::L_FOOT_TF, PERCEPTION_COMMON_NAMES::MULTISENSE_STEREO_CLOUD_TOPIC);

    ros::spin();

    return 0;
}
