#include <tough_controller_interface/arm_control_interface.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_arm_navigation");
  ros::NodeHandle nh;
  ROS_INFO("Moving the arms");
  ArmControlInterface armTraj(nh);
  while (true)
  {
    std::cout << "Arm motion\n"
                 "1. Move a joint using joint number\n"
                 "2. Move to a 3D point in task space wrt Pelvis\n"
                 "3. Move to a 6D point in task wrt Pelvis\n"
                 "4. Move to a specific configuration\n"
                 "5. Move arm to default pose\n"
                 "6. Move arm to zero pose\n"
                 "7. Run Demo - Don't do this\n"
                 "Select a motion or q to quit:";
    int choice, inputSide;
    std::cin >> choice;
    switch (choice)
    {
      case 1:
      {
        std::cout << "Enter <side> <jointNumber> <JointAngle> :";
        int jointNumber;
        float jointAngle;
        std::cin >> inputSide >> jointNumber >> jointAngle;

        RobotSide side;
        if (inputSide == 0)
        {
          side = LEFT;
        }
        else
        {
          side = RIGHT;
        }
        armTraj.moveArmJoint(side, jointNumber, jointAngle);
        break;
      }
      case 2:
      {
        std::cout << "Enter <side> <x> <y> <z> :";
        geometry_msgs::Pose pt;
        std::cin >> inputSide >> pt.position.x >> pt.position.y >> pt.position.z;
        pt.orientation.w = 1.0;
        RobotSide side;
        if (inputSide == 0)
        {
          side = LEFT;
        }
        else
        {
          side = RIGHT;
        }
        armTraj.moveArmInTaskSpace(side, pt, 3.0);
        break;
      }
      case 3:
      {
        std::cout << "Enter <side> <x> <y> <z> <ax> <ay> <az> <aw>:";
        geometry_msgs::Pose pt;
        std::cin >> inputSide >> pt.position.x >> pt.position.y >> pt.position.z >> pt.orientation.x >>
            pt.orientation.y >> pt.orientation.z >> pt.orientation.w;
        RobotSide side;
        if (inputSide == 0)
        {
          side = LEFT;
        }
        else
        {
          side = RIGHT;
        }
        armTraj.moveArmInTaskSpace(side, pt, 3.0);
        break;
      }
      case 4:
      {
        std::cout << "Enter <side> <q0> <q1> <q2> <q3> <q4> <q5> <q6>:";
        float q0, q1, q2, q3, q4, q5, q6;
        std::cin >> inputSide >> q0 >> q1 >> q2 >> q3 >> q4 >> q5 >> q6;
        std::vector<std::vector<double> > armData;

        RobotSide side = inputSide == 0 ? RobotSide::LEFT : RobotSide::RIGHT;

        armData.push_back({ q0, q1, q2, q3, q4, q5, q6 });
        armTraj.moveArmJoints(side, armData, 2.0f);
        break;
      }
      case 5:
      {
        std::cout << "Enter <side> :";
        std::cin >> inputSide;
        RobotSide side;
        if (inputSide == 0)
        {
          side = LEFT;
        }
        else
        {
          side = RIGHT;
        }
        armTraj.moveToDefaultPose(side);
        break;
      }
      case 6:
      {
        std::cout << "Enter <side> :";
        std::cin >> inputSide;
        RobotSide side;
        if (inputSide == 0)
        {
          side = LEFT;
        }
        else
        {
          side = RIGHT;
        }
        armTraj.moveToZeroPose(side);
        break;
      }
      case 7:
      {
        // Set the pose of the left arm to extend it to the front
        ArmControlInterface::ArmJointData l;
        l.side = LEFT;
        l.arm_pose = { 1.57f, 1.2f, -1.57f, 0.0f, 0.0f, 0.0f, 0.0f };
        l.time = 2;

        // Set the pose of the right arm to extend it to the front
        ArmControlInterface::ArmJointData r;
        r.side = RIGHT;
        r.arm_pose = { -1.57f, 1.2f, 1.57f, 0.0f, 0.0f, 0.0f, 0.0f };
        r.time = 2;

        // Combine the left and right arm movements
        std::vector<ArmControlInterface::ArmJointData> hug_start;
        hug_start.push_back(r);
        hug_start.push_back(l);

        // Set the pose of the left arm to embrace
        ArmControlInterface::ArmJointData l2;
        l2.side = LEFT;
        l2.arm_pose = { 1.57f, 1.2f, -1.57f, -1.1f, 0.0f, 0.0f, 0.0f };
        l2.time = 2;

        // Set the pose of the right arm to embrace
        ArmControlInterface::ArmJointData r2;
        r2.side = RIGHT;
        r2.arm_pose = { -1.57f, 1.2f, 1.57f, 1.1f, 0.0f, 0.0f, 0.0f };
        r2.time = 2;

        // Combine the left and right arm movements
        std::vector<ArmControlInterface::ArmJointData> hug_end;
        hug_end.push_back(r2);
        hug_end.push_back(l2);

        // Apply the first set of arm movements
        armTraj.moveArmJoints(hug_start);
        ros::Duration(2.5).sleep();

        // Finish with the last set of arm movements
        armTraj.moveArmJoints(hug_end);

        ros::Duration(2.5).sleep();

        // Move arms to specific points in space
        geometry_msgs::Pose right;
        right.position.x = 0.8;
        right.position.y = 0.4;
        right.position.z = 1.7;
        right.orientation.w = 1.0;

        geometry_msgs::Pose left;
        left.position.x = 2.0;
        left.position.y = 2.0;
        left.position.z = 1.5;
        left.orientation.w = 1.0;

        ArmControlInterface::ArmTaskSpaceData rts;
        rts.pose = right;
        rts.time = 2.0;
        rts.side = RIGHT;

        ArmControlInterface::ArmTaskSpaceData lts;
        lts.pose = left;
        lts.time = 2.0;
        lts.side = LEFT;

        std::vector<ArmControlInterface::ArmTaskSpaceData> ts;
        ts.push_back(rts);
        ts.push_back(lts);

        armTraj.moveArmInTaskSpace(ts);
        break;
      }
      default:
      {
        return 0;
      }
    }
    ros::Duration(2).sleep();
  }

  return 0;
}
