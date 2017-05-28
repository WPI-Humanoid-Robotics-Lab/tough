#include "val_task3/rotate_valve.h"
#include "val_controllers/val_gripper_control.h"
#include <ihmc_msgs/StopAllTrajectoryRosMessage.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "grab_valve_node");
    ros::NodeHandle nh;
    rotateValve rotate(nh);
    ROS_INFO("Starting valve grabber");
    gripperControl gc(nh);

    ros::Publisher stopTraj= nh.advertise<ihmc_msgs::StopAllTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/stop_all_trajectories",1,true);
    ihmc_msgs::StopAllTrajectoryRosMessage stopMsg;
    stopMsg.unique_id=45;

    if(argc == 4){
        geometry_msgs::Point pt;
        pt.x = std::atof(argv[1]);
        pt.y = std::atof(argv[2]);
        pt.z = std::atof(argv[3]);
        char input;
        while(1)
        {
            std::cout<<"************ ************ ************ \n";
            std::cout<<"enter choice \n";
            std::cout<<" y - Execute quater circle motion \n";
            std::cout<<" q - exit code \n";
            std::cout<<" s - stop trajectories \n";
            std::cin>>input;
            if(input =='y')
            {

                gc.openGripper(LEFT);
                rotate.grab_valve(pt);
                std::vector<geometry_msgs::Pose> points;
                geometry_msgs::Point cen;
                cen.x=0.576;
                cen.y=0.279;
                cen.z=1.16;

                rotate.compute_traj(cen,0.18,points);
                rotate.move_valve(points);
            }
            else if(input =='q')
            {
                std::cout<<"Exiting Code \n";
                exit(0);
            }
            else if(input =='s')
            {
                stopTraj.publish(stopMsg);
                std::cout<<"Stopped All Trajectories \n";
            }
            else
            {
                std::cout<<"Choose wisely \n";
            }
        }


    } else{
        ROS_INFO("Arguments incorrect");
        return -1;
    }

    ros::spin();
    return 0;
}



