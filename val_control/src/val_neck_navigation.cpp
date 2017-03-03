#include <val_control/val_neck_navigation.h>

const double degToRad = M_PI / 180;

NeckTrajectory::NeckTrajectory(ros::NodeHandle nh):nh_(nh), NUM_NECK_JOINTS(3)
{
    neckTrajPublisher =
            nh_.advertise<ihmc_msgs::NeckTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/neck_trajectory",1,true);
}

NeckTrajectory::~NeckTrajectory()
{
}

void NeckTrajectory::appendTrajectoryPoint(ihmc_msgs::NeckTrajectoryRosMessage &msg, float time, std::vector<float> pos)
{
  for (int i = 0; i < NUM_NECK_JOINTS; i++)
  {
      ihmc_msgs::TrajectoryPoint1DRosMessage p;
      ihmc_msgs::OneDoFJointTrajectoryRosMessage t;
      t.trajectory_points.clear();

      p.time = time;
      p.position = pos[i];
      p.velocity = 0;
      p.unique_id = NeckTrajectory::neck_id;
      t.trajectory_points.push_back(p);
      t.unique_id = NeckTrajectory::neck_id;
      msg.joint_trajectory_messages[i].trajectory_points.push_back(p);
  }
}


void NeckTrajectory::moveNeckJoints(const std::vector<std::vector<float> > &neck_pose, const float time)
{
  ihmc_msgs::NeckTrajectoryRosMessage msg;

  NeckTrajectory::neck_id--;
  msg.unique_id = NeckTrajectory::neck_id;

  // Add all neck trajectory points to the trajectory message
  for(auto pose=neck_pose.begin(); pose != neck_pose.end(); pose++){
    if(pose->size() != NUM_NECK_JOINTS)
      ROS_ERROR("Check number of trajectory points");
    appendTrajectoryPoint(msg, time / neck_pose.size(), *pose);
  }
  // publish the message
  neckTrajPublisher.publish(msg);
}

int NeckTrajectory::getNumNeckJoints() const
{
    return NUM_NECK_JOINTS;
}
