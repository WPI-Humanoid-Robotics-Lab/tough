#include <val_task3/leak_detector_grabber.h>
#include <val_control/robot_state.h>
#include <val_task_common/val_task_common_utils.h>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "leak_detector_grabber");
    ros::NodeHandle nh;
    leakDetectorGrabber ldg(nh);
    RobotStateInformer* rs = RobotStateInformer::getRobotStateInformer(nh);

    ROS_INFO("starting leak detector grabber");

    geometry_msgs::Pose goal;
    geometry_msgs::Pose fGoal;
    goal.orientation.x = std::atof(argv[4]);
    goal.orientation.y = std::atof(argv[5]);
    goal.orientation.z = std::atof(argv[6]);
    goal.orientation.w = std::atof(argv[7]);

    taskCommonUtils::fixHandFramePose(RIGHT,goal);

//    goal.orientation.x = -0.504;
//    goal.orientation.y = 0.535;
//    goal.orientation.z = 0.504;
//    goal.orientation.w = 0.453;

//    rs->transformPose(goal,fGoal,VAL_COMMON_NAMES::PELVIS_TF);


    if(argc == 8){

        goal.position.x = std::atof(argv[1]);
        goal.position.y = std::atof(argv[2]);
        goal.position.z = std::atof(argv[3]);
    }

    else{

        goal.position.x = -0.445;
        goal.position.y = 0.159;
        goal.position.z = 0.927;
    }

    ldg.graspDetector(goal);

    ros::spin();
    return 0;
}
