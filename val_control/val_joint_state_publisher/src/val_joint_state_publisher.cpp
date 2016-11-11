#include"val_joint_state_publisher/val_joint_state_publisher.h"

void JointStatePublisher::robotJointsCallBack(sensor_msgs::JointStatePtr msg) {

    sensor_msgs::JointState outputMsg;
    outputMsg.header = msg->header;
    int index=0;

    for (const std::string &currentJoint : msg->name){
        if (currentJoint != "hokuyo_joint"){
            outputMsg.name.push_back(        msg->name[index]);
            outputMsg.position.push_back(msg->position[index]);
            outputMsg.velocity.push_back(msg->velocity[index]);
            outputMsg.effort.push_back(    msg->effort[index]);
        }
        index ++;
    }
    m_jointStatePub.publish(outputMsg);

}

JointStatePublisher::JointStatePublisher(ros::NodeHandle nh, std::string jointStateTopicNm):
    m_jointStateSub(nh.subscribe(jointStateTopicNm,100, &JointStatePublisher::robotJointsCallBack, this)) {

    m_jointStatePub = nh.advertise<sensor_msgs::JointState>("/val/joint_states",10);

}


int main(int argc, char** argv){
    ros::init(argc, argv, "val_joint_state_publisher");
    ros::NodeHandle nh;
    JointStatePublisher obj(nh,"/ihmc_ros/valkyrie/output/joint_states");
    ros::spin();
    return 0;
}
