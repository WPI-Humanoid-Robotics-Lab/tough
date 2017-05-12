#include "val_task1/pcl_handle_detector.h"

int main(int argc, char** argv)
{
    ros::init (argc,argv,"findpclHandleDetector");
    ros::NodeHandle nh;
    geometry_msgs::Pose panel_loc;
    panel_loc.position.x =  2.35336247755;
    panel_loc.position.y = -0.13536594331;
    panel_loc.position.z = 0;
    panel_loc.orientation.x = 0;
    panel_loc.orientation.y = 0;
    panel_loc.orientation.z = -0.69335672234;
    panel_loc.orientation.w = 0.720594515373;

    float offset = 1.1;

    pcl_handle_detector  obj(nh,offset,panel_loc);
    while(ros::ok())
    {
        ros::spin();
    }

    return 0;
}
