#include <val_control/val_neck_navigation.h>

const double degToRad = M_PI / 180;

NeckTrajectory::NeckTrajectory(ros::NodeHandle nh):nh_(nh), NUM_NECK_JOINTS(2)
{
    neckTrajPublisher =
            nh_.advertise<ihmc_msgs::NeckTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/neck_trajectory",1,true);
}

NeckTrajectory::~NeckTrajectory()
{
}


void NeckTrajectory::moveNeckJoints(std::vector<float> &neck_data, const float time)
{
  ihmc_msgs::NeckTrajectoryRosMessage msg;
  ihmc_msgs::OneDoFJointTrajectoryRosMessage data;
  ihmc_msgs::TrajectoryPoint1DRosMessage p;

  NeckTrajectory::neck_id--;
  msg.unique_id = NeckTrajectory::neck_id;
  data.unique_id = NeckTrajectory::neck_id;

  for (int i=0;i<NUM_NECK_JOINTS;i++)
  {
    p.time = time;
    p.position = neck_data[i];
    p.velocity = 0;
    p.unique_id = NeckTrajectory::neck_id;
    data.trajectory_points.push_back(p);
    msg.joint_trajectory_messages[i].trajectory_points.push_back(p);
  }
  // publish the message
  neckTrajPublisher.publish(msg);
}

int NeckTrajectory::getNumNeckJoints() const
{
    return NUM_NECK_JOINTS;
}
