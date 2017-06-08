#include "val_task2/solar_array_detector.h"

int main(int argc, char** argv)
{
    ros::init (argc,argv,"array_table_detector");
    ros::NodeHandle nh;
    geometry_msgs::Pose2D rover_pose;
    geometry_msgs::Pose coarse_array_pose;
    geometry_msgs::Quaternion quaternion;
    quaternion.w = 0.708325254252;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = 0.705886204843;
    rover_pose.x = 0.183420181274;
    rover_pose.y = 0.339629471302;
    rover_pose.theta = tf::getYaw(quaternion);
    bool foundArrayLoc = false;
    int numIterations = 0;
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
    while(!foundArrayLoc)
    {
        foundArrayLoc = ad.getArrayPosition(rover_pose, coarse_array_pose);
        //ROS_INFO(foundArrayLoc ? "***** Coarse Array Pose found" : "xxxxx Coarse Array Pose not found");
        ros::spinOnce();
        numIterations++;
    }

}
