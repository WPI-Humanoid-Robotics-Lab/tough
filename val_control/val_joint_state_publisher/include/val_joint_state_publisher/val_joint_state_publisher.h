#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class JointStatePublisher{

private:
    ros::Subscriber m_jointStateSub;

    ros::Publisher m_jointStatePub;

    void robotJointsCallBack(sensor_msgs::JointStatePtr msg);

public:
    JointStatePublisher(ros::NodeHandle nh, std::string jointStateTopicNm);


};
