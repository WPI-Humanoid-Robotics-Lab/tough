#include <val_control/val_head_navigation.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_head_navigation");
  ros::NodeHandle nh;
  ROS_INFO("Moving the head");

  HeadTrajectory headTraj(nh);

  if(argc != 4){
    // Rotate 45 degrees to the right
    geometry_msgs::Quaternion q;
    q.w = 0.9238;
    q.x = 0.0;
    q.y = 0.0;
    q.z = -0.3826;
    headTraj.moveHead(q);

    ros::Duration(2).sleep();

    // Shake head to say no
    headTraj.moveHead({{0, 0, 45}, {0, 0, -45}, {0, 0, 45}, {0, 0, -45}}, 4.0f);
  } else {
    float roll = std::atof(argv[1]);
    float pitch = std::atof(argv[2]);
    float yaw = std::atof(argv[3]);
    headTraj.moveHead(roll, pitch, yaw);
  }

  ros::spin();

  return 0;
}
