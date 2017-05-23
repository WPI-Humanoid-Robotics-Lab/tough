#include <val_task3/leak_detector_grabber.h>
#include <val_controllers/robot_state.h>
#include <val_task3/val_task3_utils.h>


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
    }

    else{

        goal.position.x = -0.445;
        goal.position.y = 0.159;
        goal.position.z = 0.927;
    }


    goal.orientation.x = std::atof(argv[4]);
    goal.orientation.y = std::atof(argv[5]);
    goal.orientation.z = std::atof(argv[6]);
    goal.orientation.w = std::atof(argv[7]);

    ldg.graspDetector(RIGHT,goal);

    ros::spin();
    return 0;
}
