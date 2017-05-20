#include <val_task3/leak_detector_grabber.h>
#include <val_control/robot_state.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "leak_detector_grabber");
    ros::NodeHandle nh;
    leakDetectorGrabber ldg(nh);
    RobotStateInformer* rs = RobotStateInformer::getRobotStateInformer(nh);

    ROS_INFO("starting leak detector grabber");

    geometry_msgs::Pose goal;
    geometry_msgs::Pose fGoal;

    goal.orientation.x = -0.504;
    goal.orientation.y = 0.535;
    goal.orientation.z = 0.504;
    goal.orientation.w = 0.453;

    rs->transformPose(goal,fGoal,VAL_COMMON_NAMES::PELVIS_TF);


    if(argc == 4){

        fGoal.position.x = std::atof(argv[1]);
        fGoal.position.y = std::atof(argv[2]);
        fGoal.position.z = std::atof(argv[3]);
    }

    else{

        fGoal.position.x = -0.445;
        fGoal.position.y = 0.159;
        fGoal.position.z = 0.927;
    }

    ldg.graspDetector(fGoal);

    ros::spin();
    return 0;
}
