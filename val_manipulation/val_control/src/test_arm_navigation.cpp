#include <val_control/val_arm_navigation.h>
#include <stdlib.h>
#include <stdio.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_arm_navigation");
  ros::NodeHandle nh;
  ROS_INFO("Moving the arms");
  armTrajectory armTraj(nh);

  // Set the pose of the left arm to extend it to the front
  armTrajectory::armJointData l;
  l.side = LEFT;
  l.arm_pose = {1.57f, 1.2f, -1.57f, 0.0f, 0.0f, 0.0f, 0.0f};
  l.time = 3;

  // Set the pose of the right arm to extend it to the front
  armTrajectory::armJointData r;
  r.side = RIGHT;
  r.arm_pose = {-1.57f, 1.2f, 1.57f, 0.0f, 0.0f, 0.0f, 0.0f};
  r.time = 3;

  // Combine the left and right arm movements
  std::vector<armTrajectory::armJointData> hug_start;
  hug_start.push_back(r);
  hug_start.push_back(l);

  // Set the pose of the left arm to embrace
  armTrajectory::armJointData l2;
  l2.side = LEFT;
  l2.arm_pose = {1.57f, 1.2f, -1.57f, -1.2f, 0.0f, 0.0f, 0.0f};
  l2.time = 3;

  // Set the pose of the right arm to embrace
  armTrajectory::armJointData r2;
  r2.side = RIGHT;
  r2.arm_pose = {-1.57f, 1.2f, 1.57f, 1.2f, 0.0f, 0.0f, 0.0f};
  r2.time = 3;

  // Combine the left and right arm movements
  std::vector<armTrajectory::armJointData> hug_end;
  hug_end.push_back(r2);
  hug_end.push_back(l2);

  // Extend the right arm out to the side
  armTraj.moveToZeroPose(RIGHT);
  ros::Duration(3).sleep();

  // Extend the left arm out to the side
  armTraj.moveToZeroPose(LEFT);
  ros::Duration(3).sleep();

  // Apply the first set of arm movements
  armTraj.moveArmJoints(hug_start);
  ros::Duration(3).sleep();

  // Finish with the last set of arm movements
  armTraj.moveArmJoints(hug_end);

  while(ros::ok())
  {}

  return 0;
}
