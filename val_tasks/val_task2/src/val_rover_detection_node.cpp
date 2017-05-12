#include "val_task2/val_rover_detection.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "rover_detection");

    ros::NodeHandle nh;

    RoverDetector obj(nh);

    ros::Rate loop(1);

    while(ros::ok()){
        std::vector<geometry_msgs::Pose> poses;
        if(obj.getDetections(poses))
            for (size_t i = 0; i < poses.size(); ++i){
                ROS_INFO_STREAM("x : "<<poses[i].position.x<<"y : "<<poses[i].position.y<<"z : "<<poses[i].position.z);
            }
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
