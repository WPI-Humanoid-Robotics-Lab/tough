#include "val_task2/val_rover_detection.h"
#include "val_common/val_common_names.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "rover_detection");

    ros::NodeHandle nh;
    ros::Publisher goalPub = nh.advertise<geometry_msgs::PoseStamped>("/valkyrie/goal",1);

    RoverDetector obj(nh);
    //    RoverDetector obj2(nh);
    int NUM_SAMPLES  = 1;
    ros::Rate loop(1);
    std::vector<std::vector<geometry_msgs::Pose>> poseWaypoints;
    poseWaypoints.clear();

    while(ros::ok() && poseWaypoints.size() < NUM_SAMPLES ){
        if(obj.getDetections(poseWaypoints))
            for (size_t i = 0; i < poseWaypoints.size(); ++i){
                for (auto pose :  poseWaypoints[i]){
                    ROS_INFO_STREAM("x : "<<pose.position.x<<"y : "<<pose.position.y<<"z : "<<pose.position.z);
                }
                ROS_INFO("---");
            }
        ros::spinOnce();
        loop.sleep();
    }
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    goal.pose = poseWaypoints[NUM_SAMPLES -1][2];
    //    goalPub.publish(goal);
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
