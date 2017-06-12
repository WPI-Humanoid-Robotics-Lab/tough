#include <val_task3/leak_detector.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "leak_detector_node");
    ros::NodeHandle nh_;

    leakDetector leak_detector(nh_);

    ROS_INFO("generating search points");
    float v_s = 0.835;
    float v_e = 1.713;

    geometry_msgs::Point h_s, h_e;
    h_s.x = 0.2; h_s.y = 0.2; h_s.z = v_e;
    h_e.x = 0.9; h_e.y = 0.2; h_e.z = v_s;

    std::vector<geometry_msgs::Point> points;
    leak_detector.generateSearchWayPoints(h_s, h_e, v_s, v_e, points);

    ros::spin();

    return 0;
}
