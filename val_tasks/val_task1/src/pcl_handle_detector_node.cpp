#include "val_task1/pcl_handle_detector.h"

int main(int argc, char** argv)
{
    ros::init (argc,argv,"findpclHandleDetector");
    ros::NodeHandle nh;
    geometry_msgs::Pose panel_loc;
    panel_loc.position.x =  2.42782497479;
    panel_loc.position.y = -0.107031979006;
    panel_loc.position.z = 0;
    panel_loc.orientation.x = 0;
    panel_loc.orientation.y = 0;
    panel_loc.orientation.z = -0.698524173162;
    panel_loc.orientation.w = 0.715586458444;

    float offset = 1.1;

    pcl_handle_detector  obj(nh,offset,panel_loc);
    while(ros::ok())
    {
        ros::spin();
    }

    return 0;
}
