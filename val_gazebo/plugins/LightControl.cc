#include "LightControl.hh"

#include <gazebo/common/Events.hh>
#include <gazebo/common/Color.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/rendering/Visual.hh>
#include <boost/algorithm/string/replace.hpp>

#include <ros/ros.h>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LightControl)

LightControl::LightControl()
{
}

LightControl::~LightControl()
{
}

void LightControl::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  std::string model_name = model->GetName();
  boost::replace_all(model_name, "::", "_");
  std::string topic_name = model_name + "_control";

  if (sdf->HasElement("topicName"))
  {
    topic_name = sdf->Get<std::string>("topicName");
  }

  ros::NodeHandle private_nh;

  ROS_INFO("subscribing to topic %s", topic_name.c_str());
  _sub = private_nh.subscribe(topic_name, 10, &LightControl::OnCmd, this);

  updateConn = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&LightControl::Update, this, _1));
}

void LightControl::Update(const common::UpdateInfo &info) {
  ROS_INFO("update...");
}

void LightControl::OnCmd(const std_msgs::ColorRGBA &msg) {
  common::Color color = common::Color(msg.r, msg.g, msg.b, msg.a);
  ROS_INFO("setting color");
}
