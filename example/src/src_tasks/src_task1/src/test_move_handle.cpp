#include <ros/ros.h>
#include <string>
#include <ihmc_msgs/StopAllTrajectoryRosMessage.h>
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include "tough_common/val_common_names.h"
#include <tough_controller_interface/robot_state.h>
#include <tough_controller_interface/arm_control_interface.h>
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
    RobotDescription* rd_ = RobotDescription::getRobotDescription(nh);
    ArmControlInterface armTraj(nh);
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
    std::string robot_name;
    nh.getParam("ihmc_ros/robot_name", robot_name);

    ros::Publisher stopTraj= nh.advertise<ihmc_msgs::StopAllTrajectoryRosMessage>("/ihmc_ros/"+ robot_name +"/control/stop_all_trajectories",1,true);
    ihmc_msgs::StopAllTrajectoryRosMessage stopMsg;
    stopMsg.unique_id=45;
    int retry=0;
    std::vector<float> jointPos,closeGrasp;
    std::vector<float> closeRightGrasp,closeLeftGrasp,openGrasp;
    closeRightGrasp={1.09,1.47,1.84,0.90,1.20,1.51,0.99,1.34,1.68,0.55,0.739,0.92,1.40};
    closeLeftGrasp={-1.09,-1.47,-1.84,-0.90,-1.20,-1.51,-0.99,-1.34,-1.68,-0.55,-0.739,-0.92,1.40};
    openGrasp={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    float diffClose,diffOpen;

    while(ros::ok())
    {
        cout<<"************ ************ ************ \n";
        cout<<"enter choice \n";
        cout<<"q - exit code \n";
        cout<<"s - stop trajectoris \n";
        cout<<"t - status \n";
        cout<<"c - configure position \n";
        cout<<"g - grasp joints \n ";

        cin>>input;
        if(input=='q')
        {
            cout<<"Exiting Code \n";
            exit(0);
        }
        else if(input=='g')
        {
            //            cout<<robot_state_->getJointPosition("leftIndexFingerPitch1")<<"\n";
            //            cout<<robot_state_->getJointPosition("leftIndexFingerPitch2")<<"\n";
            //            cout<<robot_state_->getJointPosition("leftIndexFingerPitch3")<<"\n";
            //            cout<<robot_state_->getJointPosition("leftMiddleFingerPitch1")<<"\n";
            //            cout<<robot_state_->getJointPosition("leftMiddleFingerPitch2")<<"\n";
            //            cout<<robot_state_->getJointPosition("leftMiddleFingerPitch3")<<"\n";
            //            cout<<robot_state_->getJointPosition("leftPinkyPitch1")<<"\n";
            //            cout<<robot_state_->getJointPosition("leftPinkyPitch2")<<"\n";
            //            cout<<robot_state_->getJointPosition("leftPinkyPitch3")<<"\n";
            //            cout<<robot_state_->getJointPosition("leftThumbPitch1")<<"\n";
            //            cout<<robot_state_->getJointPosition("leftThumbPitch2")<<"\n";
            //            cout<<robot_state_->getJointPosition("leftThumbPitch3")<<"\n";
            //            cout<<robot_state_->getJointPosition("leftThumbRoll")<<"\n";

            cout<<"Enter side (Left-0) (Right-1) \n";
            cin>>hand;

            closeGrasp= hand ==0 ? closeLeftGrasp :closeRightGrasp;
            jointPos.clear();
            if(hand ==1)
            {
//                cout<<"Right Selected \n";
//                cout<<robot_state_->getJointPosition("rightIndexFingerPitch1")<<"\n";
//                cout<<robot_state_->getJointPosition("rightIndexFingerPitch2")<<"\n";
//                cout<<robot_state_->getJointPosition("rightIndexFingerPitch3")<<"\n";
//                cout<<robot_state_->getJointPosition("rightMiddleFingerPitch1")<<"\n";
//                cout<<robot_state_->getJointPosition("rightMiddleFingerPitch2")<<"\n";
//                cout<<robot_state_->getJointPosition("rightMiddleFingerPitch3")<<"\n";
//                cout<<robot_state_->getJointPosition("rightPinkyPitch1")<<"\n";
//                cout<<robot_state_->getJointPosition("rightPinkyPitch2")<<"\n";
//                cout<<robot_state_->getJointPosition("rightPinkyPitch3")<<"\n";
//                cout<<robot_state_->getJointPosition("rightThumbPitch1")<<"\n";
//                cout<<robot_state_->getJointPosition("rightThumbPitch2")<<"\n";
//                cout<<robot_state_->getJointPosition("rightThumbPitch3")<<"\n";
//                cout<<robot_state_->getJointPosition("rightThumbRoll")<<"\n";

                jointPos.push_back(robot_state_->getJointPosition("rightIndexFingerPitch1"));
                jointPos.push_back(robot_state_->getJointPosition("rightIndexFingerPitch2"));
                jointPos.push_back(robot_state_->getJointPosition("rightIndexFingerPitch3"));
                jointPos.push_back(robot_state_->getJointPosition("rightMiddleFingerPitch1"));
                jointPos.push_back(robot_state_->getJointPosition("rightMiddleFingerPitch2"));
                jointPos.push_back(robot_state_->getJointPosition("rightMiddleFingerPitch3"));
                jointPos.push_back(robot_state_->getJointPosition("rightPinkyPitch1"));
                jointPos.push_back(robot_state_->getJointPosition("rightPinkyPitch2"));
                jointPos.push_back(robot_state_->getJointPosition("rightPinkyPitch3"));
                jointPos.push_back(robot_state_->getJointPosition("rightThumbPitch1"));
                jointPos.push_back(robot_state_->getJointPosition("rightThumbPitch2"));
                jointPos.push_back(robot_state_->getJointPosition("rightThumbPitch3"));
                jointPos.push_back(robot_state_->getJointPosition("rightThumbRoll"));
            }
            else
            {
//                cout<<"Left Selected \n";
//                cout<<robot_state_->getJointPosition("leftIndexFingerPitch1")<<"\n";
//                cout<<robot_state_->getJointPosition("leftIndexFingerPitch2")<<"\n";
//                cout<<robot_state_->getJointPosition("leftIndexFingerPitch3")<<"\n";
//                cout<<robot_state_->getJointPosition("leftMiddleFingerPitch1")<<"\n";
//                cout<<robot_state_->getJointPosition("leftMiddleFingerPitch2")<<"\n";
//                cout<<robot_state_->getJointPosition("leftMiddleFingerPitch3")<<"\n";
//                cout<<robot_state_->getJointPosition("leftPinkyPitch1")<<"\n";
//                cout<<robot_state_->getJointPosition("leftPinkyPitch2")<<"\n";
//                cout<<robot_state_->getJointPosition("leftPinkyPitch3")<<"\n";
//                cout<<robot_state_->getJointPosition("leftThumbPitch1")<<"\n";
//                cout<<robot_state_->getJointPosition("leftThumbPitch2")<<"\n";
//                cout<<robot_state_->getJointPosition("leftThumbPitch3")<<"\n";
//                cout<<robot_state_->getJointPosition("leftThumbRoll")<<"\n";

                jointPos.push_back(robot_state_->getJointPosition("leftIndexFingerPitch1"));
                jointPos.push_back(robot_state_->getJointPosition("leftIndexFingerPitch2"));
                jointPos.push_back(robot_state_->getJointPosition("leftIndexFingerPitch3"));
                jointPos.push_back(robot_state_->getJointPosition("leftMiddleFingerPitch1"));
                jointPos.push_back(robot_state_->getJointPosition("leftMiddleFingerPitch2"));
                jointPos.push_back(robot_state_->getJointPosition("leftMiddleFingerPitch3"));
                jointPos.push_back(robot_state_->getJointPosition("leftPinkyPitch1"));
                jointPos.push_back(robot_state_->getJointPosition("leftPinkyPitch2"));
                jointPos.push_back(robot_state_->getJointPosition("leftPinkyPitch3"));
                jointPos.push_back(robot_state_->getJointPosition("leftThumbPitch1"));
                jointPos.push_back(robot_state_->getJointPosition("leftThumbPitch2"));
                jointPos.push_back(robot_state_->getJointPosition("leftThumbPitch3"));
                jointPos.push_back(robot_state_->getJointPosition("leftThumbRoll"));
            }
            diffClose=0;
            diffOpen=0;
            for (size_t i = 0; i < closeGrasp.size(); ++i) {
//                cout<<jointPos[i]<<"\t"<<closeGrasp[i]<<"\t"<<(jointPos[i]-closeGrasp[i])<<"\t"<<diffClose<<"\n";
                diffClose+=fabs(jointPos[i]-closeGrasp[i]);
                diffOpen+=fabs(jointPos[i]-openGrasp[i]);
            }
            cout<<"diff close "<<diffClose<<"\n";
            cout<<"diff open "<<diffOpen<<"\n";
            if(fabs(diffOpen)<0.1)
            {
                cout<<"Opened Gripper \n";
            }
            else if(fabs(diffClose)<0.1)
            {
                cout<<"Closed Gripper \n";
            }
            else
            {
                cout<<"Handle Grabbed or Unknown State\n";
            }
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
            RobotSide side = hand ==0 ? LEFT : RIGHT;

            sideName = hand == LEFT ? rd_->getLeftEEFrame() : rd_->getRightEEFrame();
            arm = hand == LEFT ? rd_->getLeftPalmFrame() : rd_->getRightPalmFrame();
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
            RobotSide side = hand ==0 ? LEFT : RIGHT;

            sideName = hand == LEFT ? rd_->getLeftEEFrame() : rd_->getRightEEFrame();
            arm = hand == LEFT ? rd_->getLeftPalmFrame() : rd_->getRightPalmFrame();
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

