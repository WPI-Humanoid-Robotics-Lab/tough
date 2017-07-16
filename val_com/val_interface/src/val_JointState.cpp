#include<val_interface/val_JointState.h>

valJointState::valJointState (ros::NodeHandle nh){
    nh_ = nh;
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1000);
    hw_joint_state_sub_ = nh_.subscribe("hardware_joint_states", 1000, &valJointState::hwjointStateCb, this);
    joint_state_sub_ = nh_.subscribe("joint_states", 1, &valJointState::jointStateCb, this);

}

valJointState::~valJointState(){

}

sensor_msgs::JointState valJointState::fetchJointState()
{
    return hw_jointstate_;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "joint_states");
    ros::NodeHandle nh;

    valJointState obj(nh);
    
    ros::Rate r(10);

    while(ros::ok())
    {

        std::cout << obj.fetchJointState();
        ros::spinOnce();
        r.sleep();
    }

}

void valJointState::hwjointStateCb(const sensor_msgs::JointState::ConstPtr& msg) {
    hw_jointstate_=*msg;
    joint_state_pub_.publish(hw_jointstate_);

}

void valJointState::jointStateCb(const sensor_msgs::JointStateConstPtr &state)
{

    { // scoped mutex starts
        std::lock_guard<std::mutex> lock(jointcb_mutex_);
        joint_state_ = *state;
    }

}

sensor_msgs::JointState valJointState::getJointStates(void)
{
    { // scoped mutex starts
        std::lock_guard<std::mutex> lock(jointcb_mutex_);
        return joint_state_;
    }
}
