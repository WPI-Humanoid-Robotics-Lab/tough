#include <val_task3/leak_detector_grabber.h>
#include <val_control/robot_state.h>
#include <val_task3/val_task3_utils.h>


/*
 * We need to move close to the table and ensure that
 * hand are up. The position of the robot should be carefully
 * chosen. We need to decide which hand to use based on
 * the pose of the object
 * */


int main(int argc, char **argv){

    ros::init(argc, argv, "leak_detector_grabber");
    ros::NodeHandle nh;
    leakDetectorGrabber ldg(nh);
    task3Utils utils(nh);

    //utils.beforePanelManipPose();

    ROS_INFO("starting leak detector grabber");

    geometry_msgs::Pose goal;

    if(argc == 8){

        goal.position.x = std::atof(argv[1]);
        goal.position.y = std::atof(argv[2]);
        goal.position.z = std::atof(argv[3]);
        goal.orientation.x = std::atof(argv[4]);
        goal.orientation.y = std::atof(argv[5]);
        goal.orientation.z = std::atof(argv[6]);
        goal.orientation.w = std::atof(argv[7]);
    }

    else{

        ROS_INFO("Please enter the goal Pose");
    }

    ldg.graspDetector(RIGHT,goal);

    ros::spin();
    return 0;
}
