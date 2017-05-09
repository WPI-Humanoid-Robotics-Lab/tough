#include <ros/ros.h>
#include <string>
#include <ihmc_msgs/StopAllTrajectoryRosMessage.h>
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include "val_common/val_common_names.h"
#include <val_control/robot_state.h>
#include <val_control/val_arm_navigation.h>
using namespace std;



int main(int argc, char** argv){

    ros::init(argc, argv, "test_move_handle");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    char input;
    float xChange, yChange, zChange;
    int hand;
    RobotStateInformer* robot_state_;
    robot_state_ = RobotStateInformer::getRobotStateInformer(nh);
    armTrajectory armTraj(nh);
    vector<float> leftPosLowerLimits={-2.85,-1.519,-3,1,-2.174,-2.019,-0.62,-0.36};
    vector<float> leftPosUpperLimits={2.0,1.266,2.18,0.12,3.14,0.625,0.49};
    vector<float> rightPosLowerLimits={-2.85,-1.266,-3,1,-0.12,-2.019,-0.625,-0.49};
    vector<float> rightPosUpperLimits={2.0,1.519,2.18,2.174,3.14,0.62,0.36};

    vector<float> torqueLimits={190,190,65,65,26,14,14};

    vector<float> lowerLimits;
    vector<float> upperLimits;

    geometry_msgs::Pose current_pose,new_pose;
    string sideName,arm;
    std::vector<float> jointEfforts,jointPositions;
    ros::Publisher stopTraj= nh.advertise<ihmc_msgs::StopAllTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/stop_all_trajectories",1,true);
    ihmc_msgs::StopAllTrajectoryRosMessage stopMsg;
    stopMsg.unique_id=45;
    int retry=0;


    while(ros::ok())
    {
        cout<<"************ ************ ************ \n";
        cout<<"enter choice \n";
        cout<<"q - exit code \n";
        cout<<"s - stop trajectoris \n";
        cout<<"t - status \n";
        cout<<"c - configure position \n";

        cin>>input;
        if(input=='q')
        {
            cout<<"Exiting Code \n";
            exit(0);
        }
        else if(input=='s')
        {
            stopTraj.publish(stopMsg);
            cout<<"Stopped All Trajectories \n";
        }
        else if(input =='t')
        {
            cout<<"Enter side (Left-0) (Right-1) \n";
            cin>>hand;
            armSide side = hand ==0 ? LEFT : RIGHT;

            sideName = hand == LEFT ? "/leftMiddleFingerPitch1Link" : "/rightMiddleFingerPitch1Link";
            arm = hand == LEFT ? "left_arm" : "right_arm";
            lowerLimits = hand == LEFT ? leftPosLowerLimits : rightPosLowerLimits;
            upperLimits = hand == LEFT ? leftPosUpperLimits : rightPosUpperLimits;

            robot_state_->getCurrentPose(sideName,current_pose);
            cout<<"Current Position: "<<current_pose.position.x<<"\t"<<current_pose.position.y<<"\t"<<current_pose.position.z<<"\n";
            retry=0;
            while (!robot_state_->getJointPositions(arm,jointPositions) && retry++ <5);

            cout<<"Joint Positions: \n";
            for (int i = 0; i < 7; ++i) {
                cout<<lowerLimits[i]<<"\t"<<jointPositions[i]<<"\t"<<upperLimits[i]<<"\n";
            }
            cout<<"\n";
            ros::Duration(1).sleep();

            retry=0;
            while (!robot_state_->getJointEfforts(arm,jointEfforts) && retry++ <5);

            cout<<"Joint Effort: \n";
            for (int i = 0; i < 7; ++i) {
                cout<<torqueLimits[i]<<"\t"<<jointEfforts[i]<<"\n";
            }
            cout<<"\n";

        }
        else if(input=='c')
        {

            cout<<"Enter side (Left-0) (Right-1) \n";
            cin>>hand;
            armSide side = hand ==0 ? LEFT : RIGHT;

            sideName = hand == LEFT ? "/leftMiddleFingerPitch1Link" : "/rightMiddleFingerPitch1Link";
            arm = hand == LEFT ? "left_arm" : "right_arm";
            lowerLimits = hand == LEFT ? leftPosLowerLimits : rightPosLowerLimits;
            upperLimits = hand == LEFT ? leftPosUpperLimits : rightPosUpperLimits;

            robot_state_->getCurrentPose(sideName,current_pose);
            cout<<"Current Position: "<<current_pose.position.x<<"\t"<<current_pose.position.y<<"\t"<<current_pose.position.z<<"\n";
            retry=0;
            while (!robot_state_->getJointPositions(arm,jointPositions) && retry++ <5);

            cout<<"Joint Positions: \n";
            for (int i = 0; i < 7; ++i) {
                cout<<lowerLimits[i]<<"\t"<<jointPositions[i]<<"\t"<<upperLimits[i]<<"\n";
            }
            cout<<"\n";



            cout<<"Enter change in x \n";
            cin>>xChange;
            cout<<"Enter change in y \n";
            cin>>yChange;
            cout<<"Enter change in z \n";
            cin>>zChange;

            current_pose.position.x+=xChange;
            current_pose.position.y+=yChange;
            current_pose.position.z+=zChange;

            cout<<"Final Position Planned: "<<current_pose.position.x<<"\t"<<current_pose.position.y<<"\t"<<current_pose.position.z<<"\n";
            armTraj.moveArmInTaskSpace(side, current_pose, 3.0);
            ros::Duration(2).sleep();

            robot_state_->getCurrentPose(sideName,new_pose);
            cout<<"Final Position Actual: "<<new_pose.position.x<<"\t"<<new_pose.position.y<<"\t"<<new_pose.position.z<<"\n";
            cout<<"Error Position   : "<<new_pose.position.x-current_pose.position.x<<"\t"<<new_pose.position.y-current_pose.position.y<<"\t"<<new_pose.position.z-current_pose.position.z<<"\n";
            cout<<"Error Orientation: "<<new_pose.orientation.w-current_pose.orientation.w<<"\t"<<new_pose.orientation.x-current_pose.orientation.x<<"\t"<<new_pose.orientation.y-current_pose.orientation.y<<"\t"<<new_pose.orientation.z-current_pose.orientation.z<<"\n";

            retry=0;
            while (!robot_state_->getJointEfforts(arm,jointEfforts) && retry++ <5);

            cout<<"Joint Effort: \n";
            for (int i = 0; i < 7; ++i) {
                cout<<torqueLimits[i]<<"\t"<<jointEfforts[i]<<"\n";
            }
            cout<<"\n";
        }
        else
            cout<<"invalid"<<endl;
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}

