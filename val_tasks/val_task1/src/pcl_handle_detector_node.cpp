#include "val_task1/pcl_handle_detector.h"

int main(int argc, char** argv)
{
    ros::init (argc,argv,"findpclHandleDetector");
    ros::NodeHandle nh;
    geometry_msgs::Pose panel_loc;
    panel_loc.position.x =  3.06;
    panel_loc.position.y = 0.255;
    panel_loc.position.z = 0;
    tf::Quaternion q = tf::createQuaternionFromYaw(M_PI_4);
    tf::quaternionTFToMsg(q, panel_loc.orientation);
//    panel_loc.orientation.x = 0;
//    panel_loc.orientation.y = 0;
//    panel_loc.orientation.z = -0.69335672234;
//    panel_loc.orientation.w = 0.720594515373;

    ros::Rate loop(1);
    pcl_handle_detector  obj(nh,panel_loc);
    while(ros::ok())
    {
        std::vector<geometry_msgs::Point> handle_locations;
        obj.getDetections(handle_locations);
        for (auto location : handle_locations){
            ROS_INFO("X: %.2f Y: %.2f Z: %.2f", location.x, location.y, location.z);
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
