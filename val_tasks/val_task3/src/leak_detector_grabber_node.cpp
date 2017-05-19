#include <val_task3/leak_detector_grabber.h>
#include <val_control/robot_state.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "leak_detector_grabber");
    ros::NodeHandle nh;
    leakDetectorGrabber ldg(nh);
    RobotStateInformer* rs = RobotStateInformer::getRobotStateInformer(nh);

    ROS_INFO("starting leak detector grabber");

    geometry_msgs::Pose goal;
    geometry_msgs::Pose finalGoal;

    goal.orientation.x = -0.504;
    goal.orientation.y = 0.535;
    goal.orientation.z = 0.504;
    goal.orientation.w = 0.453;
   // goal.orientation.w = 1;

    rs->transformPose(goal,finalGoal,VAL_COMMON_NAMES::PELVIS_TF);

    finalGoal.position.x = -0.645;
    finalGoal.position.y = 0.199;
    finalGoal.position.z = 0.887;

    ldg.graspDetector(finalGoal);

    ros::spin();
    return 0;
}
