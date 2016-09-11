#pragma once

#include <sdf/sdf.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>


#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>

namespace gazebo
{

class LightControl : public ModelPlugin
{
  public:
    LightControl();
    ~LightControl();

    /// \brief Load the dc motor and configures it according to the sdf.
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

    /// \brief Update the torque on the joint from the dc motor each timestep.
    void Update(const common::UpdateInfo &info);

  private:

    void PublishInfo();

    physics::ModelPtr model;
    physics::LinkPtr body;
    event::ConnectionPtr updateConn;
    transport::NodePtr node;
    transport::PublisherPtr pose_pub;

    void OnCmd(const std_msgs::ColorRGBA &msg);
    ros::Subscriber _sub;

};

}
