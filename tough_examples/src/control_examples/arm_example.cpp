#include <tough_controller_interface/arm_control_interface.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_arm");
  ros::NodeHandle nh;
  std::string LOG_TOPIC = "/field/log";
  ros::Publisher log_pub = nh.advertise<std_msgs::String>(LOG_TOPIC, 10);
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

  ROS_INFO("Moving the arms");
  ArmControlInterface armTraj(nh);
  if (argc != 4)
  {
    log_msg("Expected 3 arguments, got " + std::to_string(argc - 1));
  }
  else
  {
    int inputSide = std::atof(argv[1]);
    int jointNumber = std::atof(argv[2]);
    float jointAngle = std::atof(argv[3]);

    RobotSide side;
    std::string side_str;
    if (inputSide == 0)
    {
      side = LEFT;
      side_str = "left";
    }
    else
    {
      side = RIGHT;
      side_str = "right";
    }

    log_msg("Moving " + side_str + " joint " + std::to_string(jointNumber) + " to " + std::to_string(jointAngle));
    armTraj.moveArmJoint(side, jointNumber, jointAngle);
    ros::spinOnce();
    ros::Duration(2).sleep();
    log_msg("Motion complete");
  }

  return 0;
}
