#include "val_task3/rotate_valve.h"
#include "val_controllers/val_gripper_control.h"
#include <ihmc_msgs/StopAllTrajectoryRosMessage.h>
#include "val_task3/door_opener.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grab_valve_node");
    ros::NodeHandle nh;

    ROS_INFO("Starting valve grabber");

    rotateValve rotate(nh);
    gripperControl gc(nh);
    doorOpener doorOpen(nh);
    geometry_msgs::Point valveCentre;
    std::vector<geometry_msgs::Pose> points;
    geometry_msgs::Point pt;


    ros::Publisher stopTraj= nh.advertise<ihmc_msgs::StopAllTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/stop_all_trajectories",1,true);
    ihmc_msgs::StopAllTrajectoryRosMessage stopMsg;
    stopMsg.unique_id=45;

    //valve centre value
    if(argc == 4){

        gc.openGripper(LEFT);

        valveCentre.x = std::atof(argv[1]);
        valveCentre.y = std::atof(argv[2]);
        valveCentre.z = std::atof(argv[3]);
       }

    else {

        ROS_INFO("Please enter correct arguments");
    }

    //Positioning the robot to rotate the valve
    rotate.reOrientbeforgrab(valveCentre);


    //Rotating the valve six times
    for(size_t i = 1; i< 7; ++i){
        ROS_INFO("Try Number : %d", i );
        // rotate.grab_valve(pt);
        rotate.grab_valve(valveCentre);
        rotate.compute_traj(valveCentre,0.18,points);
        rotate.move_valve(points);
    }

    ros::Duration(3.0).sleep();

    //Opening the door
    geometry_msgs::Pose valveCentrePose;
    valveCentrePose.position = valveCentre;
    doorOpen.openDoor(valveCentrePose);

    ros::spin();
    return 0;
}



