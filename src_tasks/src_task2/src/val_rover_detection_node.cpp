#include "src_task2/val_rover_detection.h"
#include "tough_common/val_common_names.h"
#include "src_task2/val_task2_utils.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "rover_detection");

    ros::NodeHandle nh;

    RoverDetector obj(nh);
    task2Utils utils(nh);

    //    RoverDetector obj2(nh);
    int NUM_SAMPLES  = 1;
    ros::Rate loop(0.5);
    std::vector<std::vector<geometry_msgs::Pose>> poseWaypoints;
    poseWaypoints.clear();
    utils.taskLogPub(">>>>>Rover Position<<<<<");
    while(ros::ok() && poseWaypoints.size() < NUM_SAMPLES ){
        if(obj.getDetections(poseWaypoints))
            for (size_t i = 0; i < poseWaypoints.size(); ++i){
                for (auto pose :  poseWaypoints[i]){
                    ROS_INFO_STREAM("x : "<<pose.position.x<<"y : "<<pose.position.y<<"z : "<<pose.position.z);
                    utils.taskLogPub("x : "+ std::to_string(pose.position.x) + " y : " + std::to_string(pose.position.y) + " theta : " + std::to_string(tf::getYaw(pose.orientation)));
                }
                ROS_INFO("---");
                utils.taskLogPub("---");
            }
        ros::spinOnce();
        loop.sleep();
    }

    ROVER_SIDE roverSide;
    if(obj.getRoverSide(roverSide)){
        std::string str = (roverSide == ROVER_SIDE::RIGHT) ? "Rover on Right" : "Rover on Left";
        std::cout <<  str <<std::endl;
    }
    else{
        std::cout<<"Rover side could not be determined"<<std::endl;
    }
    ros::Duration(0.2).sleep();
    return 0;
}
