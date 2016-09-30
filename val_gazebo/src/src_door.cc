#include "src_door/src_door.h"
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

using namespace gazebo;

SRCDoor::SRCDoor() {}

void SRCDoor::run(int argc, char** argv)
{
  ros::init(argc, argv, "src_door");

  ros::NodeHandle nh;
  sub = nh.subscribe("open_door", 10, &SRCDoor::callback, this);

  client::setup(argc, argv);
  transport::NodePtr node(new transport::Node());
  node->Init();

  this->pub = node->Advertise<msgs::JointCmd>("~/src_door/joint_cmd");

  this->pub->WaitForConnection();

  ros::spin();
}

void SRCDoor::callback(const std_msgs::Bool &msg)
{

    msgs::JointCmd joint_msg;
    joint_msg.set_name("src_door::hinge");

    if (msg.data) {
      ROS_INFO("open");
      joint_msg.set_force(-10);
    }else{
      ROS_INFO("close");
      joint_msg.set_force(10);
    }

    pub->Publish(joint_msg);
}

int main(int argv, char **argc)
{
  SRCDoor srcdoor;
  srcdoor.run(argv, argc);
}
