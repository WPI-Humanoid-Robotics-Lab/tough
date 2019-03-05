#include <tough_common/robot_state.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_jointState", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  RobotStateInformer* obj = RobotStateInformer::getRobotStateInformer(nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<double> positions;

  std::cout << "Efforts of left arm: ";
  obj->getJointEfforts("left_arm", positions);
  float totalEffort;
  for (auto i : positions)
  {
    totalEffort += fabs(i);
  }
  std::cout << "Total Effort : " << totalEffort << std::endl;

  ros::Rate loopRate(10);

  std::map<RobotSide, geometry_msgs::Wrench> wrenches;
  while (ros::ok())
  {
    ROS_INFO("Double support : %s", obj->isRobotInDoubleSupport() ? "True" : "False");

    obj->getFootWrenches(wrenches);

    ROS_INFO("Foot force on left leg in z axis %.2f", wrenches[LEFT].force.z);
    loopRate.sleep();
  }

  return 0;
}
