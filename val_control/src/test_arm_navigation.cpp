#include <val_control/val_arm_navigation.h>
#include <stdlib.h>
#include <stdio.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_arm_navigation");
  ros::NodeHandle nh;
  ROS_INFO("Moving the arms");
  armTrajectory armTraj(nh);

  if (argc == 4){

      int   jointNumber = std::atoi(argv[2]);
      float jointAngle = std::atof(argv[3]);

      armSide side;
      if(std::atoi(argv[1]) == 0){
        side = LEFT;
      } else {
        side = RIGHT;
      }
      armTraj.moveArmJoint(side, jointNumber,jointAngle);
  }
  else if(argc == 5){
    geometry_msgs::Pose pt;
    pt.position.x = std::atof(argv[2]);
    pt.position.y = std::atof(argv[3]);
    pt.position.z = std::atof(argv[4]);
    pt.orientation.w = 1.0;
    armSide side;
    if(std::atoi(argv[1]) == 0){
      side = LEFT;
    } else {
      side = RIGHT;
    }
    armTraj.moveArmInTaskSpace(side, pt, 3.0);
  } else if(argc == 9){

      geometry_msgs::Pose pt;
      pt.position.x = std::atof(argv[2]);
      pt.position.y = std::atof(argv[3]);
      pt.position.z = std::atof(argv[4]);
      pt.orientation.x = std::atof(argv[5]);
      pt.orientation.y = std::atof(argv[6]);
      pt.orientation.z = std::atof(argv[7]);
      pt.orientation.w = std::atof(argv[8]);

      armSide side;
      if(std::atoi(argv[1]) == 0){
        side = LEFT;
      } else {
        side = RIGHT;
      }
      armTraj.moveArmInTaskSpace(side, pt, 3.0);
  } else if(false){
    // Set the pose of the left arm to extend it to the front
    armTrajectory::armJointData l;
    l.side = LEFT;
    l.arm_pose = {1.57f, 1.2f, -1.57f, 0.0f, 0.0f, 0.0f, 0.0f};
    l.time = 2;

    // Set the pose of the right arm to extend it to the front
    armTrajectory::armJointData r;
    r.side = RIGHT;
    r.arm_pose = {-1.57f, 1.2f, 1.57f, 0.0f, 0.0f, 0.0f, 0.0f};
    r.time = 2;

    // Combine the left and right arm movements
    std::vector<armTrajectory::armJointData> hug_start;
    hug_start.push_back(r);
    hug_start.push_back(l);

    // Set the pose of the left arm to embrace
    armTrajectory::armJointData l2;
    l2.side = LEFT;
    l2.arm_pose = {1.57f, 1.2f, -1.57f, -1.1f, 0.0f, 0.0f, 0.0f};
    l2.time = 2;

    // Set the pose of the right arm to embrace
    armTrajectory::armJointData r2;
    r2.side = RIGHT;
    r2.arm_pose = {-1.57f, 1.2f, 1.57f, 1.1f, 0.0f, 0.0f, 0.0f};
    r2.time = 2;

    // Combine the left and right arm movements
    std::vector<armTrajectory::armJointData> hug_end;
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

    armTrajectory::armTaskSpaceData rts;
    rts.pose = right;
    rts.time = 2.0;
    rts.side = RIGHT;

    armTrajectory::armTaskSpaceData lts;
    lts.pose = left;
    lts.time = 2.0;
    lts.side = LEFT;

    std::vector<armTrajectory::armTaskSpaceData> ts;
    ts.push_back(rts);
    ts.push_back(lts);


    armTraj.moveArmInTaskSpace(ts);
  }


  while(ros::ok())
  {}

  return 0;
}
