#include "src_task3/rotate_valve.h"
#include "tough_controller_interface/gripper_control_interface.h"
#include <ihmc_msgs/StopAllTrajectoryRosMessage.h>
#include "src_task3/door_opener.h"
#include "src_task3/door_valve_detector.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grab_valve_node");
    ros::NodeHandle nh;

    ROS_INFO("Starting valve grabber");

    ros::Rate loop(0.3);
    size_t retry_count = 0;

    RotateValve rotate(nh);
    gripperControl gc(nh);
    DoorOpener doorOpen(nh);


    geometry_msgs::Point valveCentre;
    std::vector<geometry_msgs::Pose> points;
    std::vector<geometry_msgs::Pose> valveCentres;
    //geometry_msgs::Point pt;

    std::string robot_name;
    nh.getParam("ihmc_ros/robot_name", robot_name);

    ros::Publisher stopTraj= nh.advertise<ihmc_msgs::StopAllTrajectoryRosMessage>("/ihmc_ros/"+ robot_name +"/control/stop_all_trajectories",1,true);
    ihmc_msgs::StopAllTrajectoryRosMessage stopMsg;
    stopMsg.unique_id=45;

    DoorValvedetector detectionObj(nh);


    // Do door detection stuff

    bool success = false;
    while (retry_count++ < 5 ){
        success = detectionObj.getDetections(valveCentres);
        if (success){
            break;
        }
        ros::spinOnce();
        loop.sleep();
    }


    if(success){

        ROS_INFO_STREAM("Valve centre detected at x: " << valveCentres[0].position.x
                << " y : " << valveCentres[0].position.y << " z : " << valveCentres[0].position.z);
    }

    else {

        ROS_INFO("Unable to detect the valve");
    }



    //Positioning the robot to rotate the valve
    valveCentre = valveCentres[0].position;
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
    //valveCentrePose.position = valveCentre;
    doorOpen.openDoor(valveCentres[0]);

    ros::spin();
    return 0;
}




