#include <val_task3/leak_detector_grabber.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "leak_detector_grabber");
    ros::NodeHandle nh;
    leakDetectorGrabber ldg(nh);

    ROS_INFO("starting leak detector grabber");

    geometry_msgs::Pose goal;
    goal.position.x = 0.0851;
    goal.position.y = -0.811;
    goal.position.z = 0.97;
    goal.orientation.w = 1.0;
    ldg.graspDetector(goal);

    ros::spin();
    return 0;
}
