#include <src_task_common/finish_box_detector.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "FinishBoxDetector");
    ros::NodeHandle n;
    FinishBoxDetector mg(n);
    ros::spin();
    return 0;
}
