#include <tough_controller_interface/head_control_interface.h>
#include <stdlib.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <tough_common/tough_common_names.h>

int main(int argc, char** argv)
{
  const float degToRad = M_PI / 180.0f;
  ros::init(argc, argv, "test_head_navigation", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  HeadControlInterface* headTraj;

  ros::Publisher log_pub = nh.advertise<std_msgs::String>(TOUGH_COMMON_NAMES::LOG_TOPIC, 10);
  const auto log_msg = [&log_pub](const std::string& str) {
    std_msgs::String msg;
    msg.data = ros::this_node::getName() + ": " + str;
    log_pub.publish(msg);
    ROS_INFO("%s", msg.data.c_str());
  };

  // wait a reasonable amount of time for the subscriber to connect
  ros::Time wait_until = ros::Time::now() + ros::Duration(0.5);
  while (log_pub.getNumSubscribers() == 0 && ros::Time::now() < wait_until)
  {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }

  headTraj = new HeadControlInterface(nh);

  if (argc != 4)
  {
    log_msg("Unexpected arguments, expected roll pitch yaw (in degrees). Exiting.");
    return 1;
  }
  else
  {
    ROS_INFO("Moving the head");
    float roll = std::atof(argv[1]) * degToRad;
    float pitch = std::atof(argv[2]) * degToRad;
    float yaw = std::atof(argv[3]) * degToRad;

    log_msg("Moving head to direction " + std::to_string(roll) + ", " + std::to_string(pitch) + ", " +
            std::to_string(yaw));
    headTraj->moveHead(roll, pitch, yaw);
  }

  ros::spinOnce();
  ros::Duration(2).sleep();
  log_msg("Motion finished");

  delete headTraj;
  return 0;
}
