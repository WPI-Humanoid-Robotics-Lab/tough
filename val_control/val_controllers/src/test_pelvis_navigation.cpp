#include <val_controllers/val_pelvis_navigation.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_pelvis_navigation");
  ros::NodeHandle nh;
  ROS_INFO("Moving the pelvis");

  pelvisTrajectory pelvisTraj(nh);

  if(argc != 2)
  {
    pelvisTraj.controlPelvisHeight(1.25f);
    ros::Duration(2).sleep();
    pelvisTraj.controlPelvisHeight(0.9f);
    ros::Duration(2).sleep();
    pelvisTraj.controlPelvisHeight(1.0f);
  }
  else {
    float height = std::atof(argv[1]);
    pelvisTraj.controlPelvisHeight(height);
  }


  ros::spin();

  return 0;
}
