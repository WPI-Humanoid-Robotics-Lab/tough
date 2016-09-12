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

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
    void Update(const common::UpdateInfo &info);

  private:
    void OnCmd(const std_msgs::ColorRGBA &msg);

    physics::ModelPtr _model;
    physics::LinkPtr body;
    event::ConnectionPtr updateConn;
    transport::NodePtr node;
    transport::PublisherPtr visual_pub;

    ros::Subscriber _sub;
    ros::Time start_time;

    static constexpr int SEQUENCE_LENGTH = 20;
    static constexpr int NUM_LIGHTS = 7;
    static constexpr float LIGHT_SPACING = 0.15;
    static constexpr int MIN_ON_TIME = 1;
    static constexpr int MAX_ON_TIME = 4;


    typedef struct {
      int light_num;
      float duration;
      common::Color color;
    } seq_t;

    std::array<seq_t, SEQUENCE_LENGTH> sequence;

    int seq_index;
};

}
