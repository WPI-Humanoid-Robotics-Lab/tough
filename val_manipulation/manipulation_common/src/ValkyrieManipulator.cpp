#include<ros/ros.h>
#include<ihmc_msgs/WholeBodyTrajectoryRosMessage.h>
#include<val_common/val_common_defines.h>

class ValkyrieManipulator{
public:
    ValkyrieManipulator(ros::NodeHandle &nh);
    void sendTestMessage();

private:
    //    ihmc_msgs::WholeBodyTrajectoryRosMessagePtr m_wholebodyMsg;
    ros::Publisher m_wholebodyPub;
    ihmc_msgs::ArmTrajectoryRosMessage appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage msg, float time, float* pos);
};

ValkyrieManipulator::ValkyrieManipulator(ros::NodeHandle &nh){
    m_wholebodyPub = nh.advertise<ihmc_msgs::WholeBodyTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/whole_body_trajectory",1);

}

ihmc_msgs::ArmTrajectoryRosMessage ValkyrieManipulator::appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage msg, float time, float* pos)
{

    ihmc_msgs::TrajectoryPoint1DRosMessage p;
    ihmc_msgs::OneDoFJointTrajectoryRosMessage t;
    t.trajectory_points.clear();

    for (int i=0;i<7;i++)
    {
        p.time = time;
        p.position = pos[i];
        p.velocity = 0;
        t.trajectory_points.push_back(p);
        msg.joint_trajectory_messages[i].trajectory_points.push_back(p);
    }

    return msg;
}

void ValkyrieManipulator::sendTestMessage(){
    ihmc_msgs::WholeBodyTrajectoryRosMessage wholebodyMsg;
    wholebodyMsg.unique_id = -1;\

    ihmc_msgs::ArmTrajectoryRosMessage armTrajRight;
    armTrajRight.joint_trajectory_messages.clear();

    float BUTTON_PRESS_PREPARE_R [] = {0.0, 0.25, 0.2, 1.1, 0.0, 0.0, 0.0};

    armTrajRight.joint_trajectory_messages.resize(7);
    armTrajRight.robot_side = RIGHT;
    armTrajRight.unique_id = -11;

    armTrajRight = appendTrajectoryPoint(armTrajRight, 2, BUTTON_PRESS_PREPARE_R);


    ihmc_msgs::ArmTrajectoryRosMessage armTrajLeft;
    armTrajLeft.joint_trajectory_messages.clear();

    float BUTTON_PRESS_PREPARE_L [] = {0.0, -0.25, -0.2, -1.1, 0.0, 0.0, 0.0};


    armTrajLeft.joint_trajectory_messages.resize(7);
    armTrajLeft.robot_side = LEFT;
    armTrajLeft.unique_id = -21;


    armTrajLeft = appendTrajectoryPoint(armTrajLeft, 2, BUTTON_PRESS_PREPARE_L);


    wholebodyMsg.right_arm_trajectory_message = armTrajRight;
    wholebodyMsg.left_arm_trajectory_message = armTrajLeft;

            ihmc_msgs::PelvisTrajectoryRosMessage pelvisMsg;
            pelvisMsg.unique_id = -2;
            wholebodyMsg.pelvis_trajectory_message;

            ihmc_msgs::ChestTrajectoryRosMessage chestMsg;
            chestMsg.unique_id= -3;
            wholebodyMsg.chest_trajectory_message = chestMsg;

            ihmc_msgs::FootTrajectoryRosMessage footmsg;
            wholebodyMsg.left_foot_trajectory_message = footmsg;
            wholebodyMsg.left_foot_trajectory_message.unique_id = -4;
            wholebodyMsg.right_foot_trajectory_message = footmsg;
            wholebodyMsg.right_foot_trajectory_message.unique_id = -5;

            ihmc_msgs::HandTrajectoryRosMessage handMsg;
            wholebodyMsg.left_hand_trajectory_message = handMsg;
            wholebodyMsg.left_hand_trajectory_message.unique_id = -6;
            wholebodyMsg.right_hand_trajectory_message = handMsg;
            wholebodyMsg.right_hand_trajectory_message.unique_id = -6;

    //    m_wholebodyMsg->left_hand_trajectory_message;
    //    m_wholebodyMsg->right_hand_trajectory_message;

    m_wholebodyPub.publish(wholebodyMsg);
}


int main(int argc, char** argv){

    ros::init(argc, argv,"wholebodyManipulator");
    ros::NodeHandle nh;
    ValkyrieManipulator m(nh);
    while (ros::ok()){
        m.sendTestMessage();
        ros::spinOnce();
        ros::Duration(1).sleep();
    }


}
