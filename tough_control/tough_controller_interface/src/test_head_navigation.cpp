#include <tough_controller_interface/head_control_interface.h>
#include <stdlib.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <tough_common/val_common_names.h>

HeadTrajectory* headTraj;

void headCallBack(std_msgs::Float32MultiArray msg){
    if(msg.data.size() != 3){
        return;
    }
    float roll = msg.data[0];
    float pitch = msg.data[1];
    float yaw = msg.data[2];
//    ROS_INFO("Roll : %f Pitch : %f Yaw : %f", roll, pitch, yaw);
    headTraj->moveHead(roll, pitch, yaw);
    ros::Duration(0.5).sleep();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_head_navigation", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  ros::Publisher log_pub = nh.advertise<std_msgs::String>(VAL_COMMON_NAMES::LOG_TOPIC, 10);
  const auto log_msg = [&log_pub](const std::string &str) {
      std_msgs::String msg;
      msg.data = ros::this_node::getName() + ": " + str;
      log_pub.publish(msg);
      ROS_INFO("%s", msg.data.c_str());
  };

  // wait a reasonable amount of time for the subscriber to connect
  ros::Time wait_until = ros::Time::now() + ros::Duration(0.5);
  while (log_pub.getNumSubscribers() == 0 && ros::Time::now() < wait_until) {
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }

  headTraj = new HeadTrajectory(nh);
  ros::Subscriber sub = nh.subscribe("/head_control",10, headCallBack);

  if(argc != 4){
    log_msg("Unexpected arguments, expected roll pitch yaw (in degrees). Exiting.");
      return 1;
  } else {
      ROS_INFO("Moving the head");
    float roll = std::atof(argv[1]);
    float pitch = std::atof(argv[2]);
    float yaw = std::atof(argv[3]);

      log_msg("Moving head to direction " + std::to_string(roll) + ", " + std::to_string(pitch) + ", " + std::to_string(yaw));
    headTraj->moveHead(roll, pitch, yaw);
  }

  ros::spinOnce();
  ros::Duration(2).sleep();
    log_msg("Motion finished");
  return 0;
}
