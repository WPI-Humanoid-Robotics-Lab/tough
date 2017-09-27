#include <val_task3/leak_detector_grabber.h>
#include <tough_controller_interface/robot_state.h>
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

    ros::Publisher log_pub = nh.advertise<std_msgs::String>("/field/log",10);

    //utils.beforePanelManipPose();

    ROS_INFO("starting leak detector grabber");
    std_msgs::String log_msg;

    geometry_msgs::Pose goal;
    if (argc == 8){
        goal.position.x = std::atof(argv[1]);
        goal.position.y = std::atof(argv[2]);
        goal.position.z = std::atof(argv[3]);
        goal.orientation.x = std::atof(argv[4]);
        goal.orientation.y = std::atof(argv[5]);
        goal.orientation.z = std::atof(argv[6]);
        goal.orientation.w = std::atof(argv[7]);
        log_msg.data = "Leak detector grabber starting with manual pose";
        log_pub.publish(log_msg);
    } else if (argc == 1 || argc == 2) {
        ROS_INFO("Leak detector grabber listening for pose from Rviz on /initialpose");
        log_msg.data = "Leak detector grabber listening for pose from Rviz on /initialpose";
        log_pub.publish(log_msg);

        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr topic_pose =
                ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/initialpose");
        goal = topic_pose->pose.pose;

        if (argc == 2) {
            goal.position.z = std::atof(argv[1]);
        }

        ROS_INFO_STREAM("Leak detector grabber heard your pose (frame " << topic_pose->header.frame_id << ")");
        log_msg.data = "Leak detector grabber starting with published pose";
        log_pub.publish(log_msg);
    } else {
        ROS_INFO("Please enter 7 arguments to specify the goal pose, 0 to listen for the published pose and set z "
        "automatically, or 1 to listen for the published pose and specify z");
        log_msg.data = "ERROR: Wrong number of arguments to leak_detector_grabber";
        log_pub.publish(log_msg);
        return 1;
    }

    // To be done - add another seed point for better planning and manipulation
    ldg.graspDetector(goal);

    ros::spin();
    return 0;
}
