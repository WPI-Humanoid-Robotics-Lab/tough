#include "val_task2/solar_array_detector.h"

int main(int argc, char** argv)
{
    ros::init (argc,argv,"array_table_detector");
    ros::NodeHandle nh;
    geometry_msgs::Pose2D rover_pose;
    geometry_msgs::Quaternion quaternion;
    quaternion.w = 0.927;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = -0.40452093478;
    rover_pose.x = 8.442;
    rover_pose.y = 1.104;
    rover_pose.theta = tf::getYaw(quaternion);
    bool foundArrayLoc;
//x: 3.65252542496
//y: 1.20730876923
//z: 0.0
//orientation:
//x: 0.0
//y: 0.0
//z: 0.927069752274
//w: 0.374888882761
// 8.442, 1.104, 0.000 0.000, 0.000, -0.376, 0.927
    CoarseArrayDetector ad(nh);
    while(ros::ok())
    {
        foundArrayLoc = ad.getArrayPosition(rover_pose);
        ros::spinOnce();
    }

}
