#include <val_laser2point_cloud/val_laser2point_cloud_node.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "laser2point_cloud");
    ros::NodeHandle n;
    Laser2PointCloud laser2point(n, "/multisense/lidar_scan","/leftFoot","/multisense/points2");

    ros::spin();

    return 0;
}
