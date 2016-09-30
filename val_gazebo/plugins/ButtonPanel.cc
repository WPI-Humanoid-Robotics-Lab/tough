#include "ButtonPanel.hh"
#include <gazebo/common/Events.hh>
#include <std_msgs/Bool.h>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ButtonPanel)

ButtonPanel::ButtonPanel() : _pressed_position_lower(0.0), _pressed_position_upper(0.01), _was_pressed(false) {}

ButtonPanel::~ButtonPanel() {}

void ButtonPanel::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  _model = model;

  if (sdf->HasElement("joint"))
  {
    _joint_name = sdf->Get<std::string>("joint");
    gzwarn << "joint: " << _joint_name << std::endl;
    _joint = _model->GetJoint(_joint_name);
  }
  else
  {
    gzwarn << "missing joint element. Plugin will do nothing." << std::endl;
  }

  if (sdf->HasElement("pressed_position_lower"))
  {
    _pressed_position_lower = sdf->Get<double>("pressed_position_lower");
  }
  else
  {
    gzwarn << "missing pressed_position_lower element. Assuming 0.0" << std::endl;
  }

  if (sdf->HasElement("pressed_position_upper"))
  {
    _pressed_position_upper = sdf->Get<double>("pressed_position_upper");
  }
  else
  {
    gzwarn << "missing pressed_position_upper element. Assuming 0.01" << std::endl;
  }

  ros::NodeHandle nh;
  _pub = nh.advertise<std_msgs::Bool>("button_pressed", 10, true); // yes latch

  _updateConn = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ButtonPanel::Update, this, _1));

  ROS_INFO("lower: %f upper: %f", _pressed_position_lower, _pressed_position_upper);

}

void ButtonPanel::Update(const common::UpdateInfo &info)
{
  if (_joint)
  {
    double joint_position = _joint->GetAngle(0).Radian();
    if (joint_position > _pressed_position_lower && joint_position < _pressed_position_upper)
    {
      if (!_was_pressed)
      {
        _was_pressed = true;
        std_msgs::Bool msg;
        msg.data = true;
        _pub.publish(msg);
      }
    }
    else if (_was_pressed)
    {
      std_msgs::Bool msg;
      msg.data = false;
      _pub.publish(msg);
      _was_pressed = false;
    }
  }
  else {
    ROS_INFO_ONCE("No joint found with the name %s", _joint_name.c_str());
  }
}
