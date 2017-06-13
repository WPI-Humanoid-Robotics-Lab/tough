#include <val_task3/leak_detector_grabber.h>
#include <val_controllers/robot_state.h>
#include <val_task3/val_task3_utils.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


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
    if (argc == 8){
        goal.position.x = std::atof(argv[1]);
        goal.position.y = std::atof(argv[2]);
        goal.position.z = std::atof(argv[3]);
        goal.orientation.x = std::atof(argv[4]);
        goal.orientation.y = std::atof(argv[5]);
        goal.orientation.z = std::atof(argv[6]);
        goal.orientation.w = std::atof(argv[7]);
    } else if (argc == 1) {
        ROS_INFO("Leak detector grabber will listen for pose from Rviz on /initialpose");

        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr topic_pose =
                ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/initialpose");
        goal = topic_pose->pose.pose;
        ROS_INFO_STREAM("Leak detector grabber heard your pose (frame " << topic_pose->header.frame_id << ")");
    } else {
        ROS_INFO("Please enter 7 arguments to specify the goal pose or 0 to listen for the published pose");
        return 1;
    }

    // To be done - add another seed point for better planning and manipulation
    ldg.graspDetector(goal);

    ros::spin();
    return 0;
}
