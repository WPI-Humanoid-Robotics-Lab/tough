#include <val_task3/leak_detector.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "leak_detector_node");
    ros::NodeHandle nh_;

    if (argc != 3) {
        ROS_ERROR("Run leak_detector_node with 2 arguments: side ['left' or 'right'], and thumbwards [1 for thumbwards, 0 otherwise]");
        return 1;
    } else if (std::strcmp(argv[1], "left") != 0 && std::strcmp(argv[1], "right") != 0) {
        ROS_ERROR("First argument to leak detector must be the string 'left' or 'right'");
        return 1;
    } else if (std::strcmp(argv[2], "1") != 0 && std::strcmp(argv[2], "0") != 0) {
        ROS_ERROR("First argument to leak detector must be 1 or 0");
        return 1;
    }

    RobotSide side = (std::strcmp(argv[1], "left") == 0) ? RobotSide::LEFT : RobotSide::RIGHT;
    bool thumbwards = (std::strcmp(argv[2], "1") == 0);

    leakDetector leak_detector(nh_, side, thumbwards);

    ROS_INFO("Starting leak detection");
    leak_detector.findLeak();
    ROS_INFO("Found leak OR node is shutting down");

    return 0;
}
