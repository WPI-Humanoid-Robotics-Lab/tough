#include <val_task1/panel_detection.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "panel_detector");
    ros::NodeHandle nh;
    ros::Publisher goalPub = nh.advertise<geometry_msgs::PoseStamped>("/valkyrie/goal",1);
    panel_detector obj(nh);
    int NUM_SAMPLES = 1;
    std::vector<geometry_msgs::Pose> detections;
    int trials;
    while(ros::ok() && detections.size() < NUM_SAMPLES){
        ros::spinOnce();
        obj.getDetections(detections);
        trials = obj.getDetectionTries();
    }

    std::vector<geometry_msgs::Pose> poses;
    obj.getDetections(poses);

    ROS_INFO("Tried %d time and succeeded %d times", trials, (int)poses.size());

    //this can be used if we need to find median/mode
    // std::sort(poses.begin(),poses .end(), poseComparator);

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    goal.pose = poses[NUM_SAMPLES -1];
    goalPub.publish(goal);

    ros::Duration(1).sleep();
    ROS_INFO("Exiting panel detector node");

    return 0;
}
