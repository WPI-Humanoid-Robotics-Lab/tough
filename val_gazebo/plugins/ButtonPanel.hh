#pragma once

#include <sdf/sdf.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/common/Events.hh>
#include <ros/ros.h>

using namespace gazebo;
class ButtonPanel: public ModelPlugin
{
  public:
    ButtonPanel();
    ~ButtonPanel();

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
    void Update(const common::UpdateInfo &info);

  private:
    physics::ModelPtr _model;
    physics::JointPtr _joint;
    event::ConnectionPtr _updateConn;
    double _pressed_position_lower;
    double _pressed_position_upper;
    std::string _joint_name;
    bool _was_pressed;

    ros::Publisher _pub;
};
