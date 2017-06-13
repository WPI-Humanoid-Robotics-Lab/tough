#include <val_controllers/val_head_navigation.h>
#include <stdlib.h>
#include <std_msgs/Float32MultiArray.h>

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

  headTraj = new HeadTrajectory(nh);
  ros::Subscriber sub = nh.subscribe("/head_control",10, headCallBack);

  if(argc != 4){
        ros::spin();

  } else {
      ROS_INFO("Moving the head");
    float roll = std::atof(argv[1]);
    float pitch = std::atof(argv[2]);
    float yaw = std::atof(argv[3]);
    headTraj->moveHead(roll, pitch, yaw);
  }

  ros::spinOnce();
  ros::Duration(2).sleep();
  return 0;
}
