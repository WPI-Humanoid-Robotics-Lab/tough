#include "LightControl.hh"

#include <gazebo/common/Events.hh>
#include <gazebo/common/Color.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/rendering/Visual.hh>
#include <boost/algorithm/string/replace.hpp>
#include <ignition/math.hh>

#include <ros/ros.h>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LightControl)


LightControl::LightControl() : seq_index(0)
{
  // should be random
  // for now it's just each light sequentially turns red
  for (int i = 0; i < SEQUENCE_LENGTH; i++) {
    seq_t new_light;
    new_light.light_num = std::rand() % NUM_LIGHTS;
    new_light.duration = (std::rand() % (MAX_ON_TIME - MIN_ON_TIME)) + MIN_ON_TIME; // from 5 to 20 seconds
    int color_index = std::rand() % 3;
    switch (color_index) {
      case 0:
        new_light.color = common::Color::Red;
        break;
      case 1:
        new_light.color = common::Color::Green;
        break;
      case 2:
        new_light.color = common::Color::Blue;
        break;
    }
    sequence[i] = new_light;
  }
}

LightControl::~LightControl()
{
}

void LightControl::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  this->_model = model;
  std::string model_name = model->GetName();
  boost::replace_all(model_name, "::", "_");
  std::string topic_name = model_name + "_control";

  if (sdf->HasElement("topicName"))
  {
    topic_name = sdf->Get<std::string>("topicName");
  }

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  visual_pub = node->Advertise<msgs::Visual>("~/visual", 10);

  ros::NodeHandle private_nh("~");

  ROS_INFO("subscribing to: %s", topic_name.c_str());
  _sub = private_nh.subscribe(topic_name, 10, &LightControl::OnCmd, this);

  updateConn = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&LightControl::Update, this, _1));

  start_time = ros::Time::now();
}

void LightControl::Update(const common::UpdateInfo &info) {

  ros::Time now = ros::Time::now();
  if (now - start_time > ros::Duration(sequence[seq_index].duration))
  {
    seq_index++;
    start_time = now;
    ROS_INFO("seq index %i", seq_index);

    if (seq_index >= SEQUENCE_LENGTH)
    {
      seq_index = 0;
    }
  }

  msgs::Visual* visual_msgs[NUM_LIGHTS];

  for (int i = 0; i < NUM_LIGHTS; i++)
  {
    visual_msgs[i] = new msgs::Visual();

    std::string visual_name = "light" + std::to_string(i);
    visual_msgs[i]->set_name(visual_name);
    visual_msgs[i]->set_visible(true);
    visual_msgs[i]->set_parent_name("light_panel::base");
    visual_msgs[i]->set_cast_shadows(false);

    msgs::Geometry *geomMsg = visual_msgs[i]->mutable_geometry();
    geomMsg->set_type(gazebo::msgs::Geometry::CYLINDER);
    geomMsg->mutable_cylinder()->set_radius(0.05);
    geomMsg->mutable_cylinder()->set_length(0.01);

    math::Pose model_pose = _model->GetWorldPose();
    float x = model_pose.pos.x + 0.51;
    float y = model_pose.pos.y + (i - (NUM_LIGHTS/2)) * LIGHT_SPACING;
    float z = model_pose.pos.z + 0.75;

    msgs::Set(visual_msgs[i]->mutable_pose(), ignition::math::Pose3d(x, y, z, 0, 1.5707, 0));

    if (sequence[seq_index].light_num == i)
    {
      msgs::Set(visual_msgs[i]->mutable_material()->mutable_diffuse(), sequence[seq_index].color);
    }
    else {
      msgs::Set(visual_msgs[i]->mutable_material()->mutable_diffuse(), common::Color::Black);
    }
    visual_pub->Publish(*visual_msgs[i]);

  }
}

void LightControl::OnCmd(const std_msgs::ColorRGBA &msg) {
}
