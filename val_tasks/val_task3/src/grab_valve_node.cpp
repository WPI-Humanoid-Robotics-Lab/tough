#include "val_task3/rotate_valve.h"
#include "val_controllers/val_gripper_control.h"
#include <ihmc_msgs/StopAllTrajectoryRosMessage.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grab_valve_node");
    ros::NodeHandle nh;
    int gripper;

    ROS_INFO("Starting valve grabber");

    rotateValve rotate(nh);
    gripperControl gc(nh);


    geometry_msgs::Point valveCentre;
    std::vector<geometry_msgs::Pose> points;
    bool walkingResult, trajectoryResult;
    task3Utils t3Utils(nh);

    ros::Publisher stopTraj= nh.advertise<ihmc_msgs::StopAllTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/stop_all_trajectories",1,true);
    ihmc_msgs::StopAllTrajectoryRosMessage stopMsg;
    stopMsg.unique_id=45;

    if(argc == 5){

        gc.openGripper(LEFT);

        valveCentre.x = std::atof(argv[1]);
        valveCentre.y = std::atof(argv[2]);
        valveCentre.z = std::atof(argv[3]);
        gripper        = std::atof(argv[4]);
        t3Utils.task3LogPub("grab_valve_node: Received the centre coordinates for valve");
    }

    else {

        ROS_INFO("Please enter correct arguments");
    }



    walkingResult = rotate.reOrientbeforgrab(valveCentre);
    ros::Duration(2.0).sleep();

    if(walkingResult){
        //Rotating the valve six times
        t3Utils.task3LogPub("grab_valve_node: trying to rotate the valve");
        for(size_t i = 1; i< 4; ++i){
            ROS_INFO("Try Number : %d", i );
            t3Utils.task3LogPub("grab_valve_node : Try Number :" + std::to_string(i));
            // rotate.grab_valve(pt);
            rotate.grab_valve(valveCentre);
            if (gripper == 1) gc.closeGripper(LEFT);
            rotate.compute_traj(valveCentre,0.18,points);
            trajectoryResult = rotate.move_valve(points);
            if(!trajectoryResult) t3Utils.task3LogPub("trajectory was not planned");


        }
         t3Utils.task3LogPub("grab_valve_node: Done Rotating the valve");
    }

    else{

        ROS_INFO("The path was not planned, Not doing anything");
        t3Utils.task3LogPub("grab_valve_node : The path was not planned, Not doing anything");
    }

    ros::Duration(1.0).sleep();

    ros::spin();
    return 0;
}



