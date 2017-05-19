#include <iostream>
#include<val_footstep/ValkyrieWalker.h>
#include<ihmc_msgs/FootstepDataListRosMessage.h>
#include"geometry_msgs/Pose2D.h"
#include "val_common/val_common_defines.h"
#include <ihmc_msgs/StopAllTrajectoryRosMessage.h>

using namespace std;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "walk_test");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    ValkyrieWalker walk(nh, 1.0,1.0,0);
    ros::Publisher stopTraj= nh.advertise<ihmc_msgs::StopAllTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/stop_all_trajectories",1,true);
    ihmc_msgs::StopAllTrajectoryRosMessage stopMsg;
    stopMsg.unique_id=45;

    char input;
    int hand;
    float stepHeight;
    float stepLength=0.0;
    float curl;
    float nudgeDistance;
    float offset;

    geometry_msgs::Pose2D goal;
    goal.x=0.0;
    goal.y=0.0;
    goal.theta=-1.57;
    //    walk.walkToGoal(goal);


    while(ros::ok())
    {
        cout<<"************ ************ ************ \n";
        cout<<"enter choice \n";
        cout<<"q - exit code \n";
        cout<<"s - stop trajectories \n";
        cout<<"l - load \n";
        cout<<"u - up \n";
        cout<<"c - curl \n";
        cout<<"f - forward \n";
        cout<<"p - place leg \n";

        cin>>input;
        if(input=='q')
        {
            cout<<"Exiting Code \n";
            exit(0);
        }
        else if(input=='u')
        {
            cout<<"enter side \n";
            cin>>hand;
            cout<<"enter step height \n";
            cin>>stepHeight;
            //            cout<<"enter step length \n";
            //            cin>>stepLength;
            if(hand==0)
                walk.raiseLeg(LEFT,stepHeight,stepLength);
            else
                walk.raiseLeg(RIGHT,stepHeight,stepLength);
        }
        else if(input=='l')
        {

            cout<<"enter side \n";
            cin>>hand;
            if(hand==0){
                walk.load_eff(LEFT,EE_LOADING::LOAD);
            }
            else  walk.load_eff(RIGHT,EE_LOADING::LOAD);
        }
        else if(input=='c')
        {

            cout<<"enter side \n";
            cin>>hand;
            cout<<"enter curl radius \n";
            cin>>curl;
            if(hand==0){
                walk.curlLeg(LEFT,curl);
            }
            else  walk.curlLeg(RIGHT,curl);
        }
        else if(input=='p')
        {
            cout<<"enter side \n";
            cin>>hand;
            cout<<"enter offset \n";
            cin>>offset;
            if(hand==0){
                walk.placeLeg(LEFT,offset);
            }
            else  walk.placeLeg(RIGHT,offset);
        }
        else if(input=='f')
        {

            cout<<"enter side \n";
            cin>>hand;
            cout<<"enter forward distance \n";
            cin>>nudgeDistance;
            if(hand==0){
                walk.nudgeFoot(LEFT,nudgeDistance);
            }
            else  walk.nudgeFoot(RIGHT,nudgeDistance);
        }
        else if(input=='s')
        {
            stopTraj.publish(stopMsg);
            cout<<"Stopped All Trajectories \n";
        }
        else cout<<"invalid input \n";
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
