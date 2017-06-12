#include <val_controllers/robot_state.h>
#include <val_controllers/val_arm_navigation.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_jointState", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  RobotStateInformer* obj = RobotStateInformer::getRobotStateInformer(nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate loop(60);

    std::vector<float> positions;
//    obj->getJointPositions(positions);

//    std::cout << "Positions of all joints :";
//    for (auto i : positions)
//    {
//      std::cout << i << " ";
//    }
//    std::cout << std::endl;

//    ROS_INFO("rightShoulderRoll : %.2f  %.2f  %.2f", obj->getJointPosition("rightShoulderRoll"),
//             obj->getJointVelocity("rightShoulderRoll"), obj->getJointEffort("rightShoulderRoll"));

//    std::vector<std::string> jointNames;

//    obj->getJointNames(jointNames);

//    for (auto i : jointNames)
//    {
//      std::cout << i << " ";
//    }

//    std::cout << std::endl;

//    obj->getJointPositions("/ihmc_ros/valkyrie/left_arm_joint_names", positions);
//    std::cout << "Position of left arm: ";
//    for (auto i : positions)
//    {
//      std::cout << i << " ";
//    }
//    std::cout << std::endl;

//    std::cout << "Velocities of left arm: ";
//    obj->getJointVelocities("left_arm_joint_names", positions);
//    for (auto i : positions)
//    {
//      std::cout << i << " ";
//    }
//    std::cout << std::endl;

    std::cout << "Efforts of left arm: ";
    obj->getJointEfforts("left_arm", positions);
    float totalEffort;
    for (auto i : positions)
    {
      totalEffort += fabs(i);
    }
    std::cout << "Total Effort : "<<totalEffort<< std::endl;
    ros::Duration(2).sleep();
    ros::spinOnce();
    //loop.sleep();


  return 0;
}
