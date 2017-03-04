#include <val_control/val_head_navigation.h>

const double degToRad = M_PI / 180;

HeadTrajectory::HeadTrajectory(ros::NodeHandle nh):nh_(nh)
{
    headTrajPublisher =
            nh_.advertise<ihmc_msgs::HeadTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/head_trajectory",1,true);
}

HeadTrajectory::~HeadTrajectory()
{
}


void HeadTrajectory::moveHead(float roll, float pitch, float yaw, const float time)
{
  ihmc_msgs::HeadTrajectoryRosMessage msg;
  ihmc_msgs::SO3TrajectoryPointRosMessage data;

  roll = degToRad * roll;
  pitch = degToRad * pitch;
  yaw = degToRad * yaw;

  data.time = time;
  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  tf::quaternionTFToMsg(q, data.orientation);
  geometry_msgs::Vector3 v;
  v.x = 0.0;
  v.y = 0.0;
  v.z = 0.0;
  data.angular_velocity = v;

  HeadTrajectory::head_id--;
  msg.unique_id = HeadTrajectory::head_id;
  msg.execution_mode = msg.OVERRIDE;

  msg.taskspace_trajectory_points.clear();

  msg.taskspace_trajectory_points.push_back(data);

  // publish the message
  headTrajPublisher.publish(msg);
}

void HeadTrajectory::moveHead(const geometry_msgs::Quaternion &quaternion, const float time)
{
  ihmc_msgs::HeadTrajectoryRosMessage msg;
  ihmc_msgs::SO3TrajectoryPointRosMessage data;

  data.orientation = quaternion;

  geometry_msgs::Vector3 v;
  v.x = 0.0;
  v.y = 0.0;
  v.z = 0.0;
  data.angular_velocity = v;

  HeadTrajectory::head_id--;
  msg.unique_id = HeadTrajectory::head_id;
  msg.execution_mode = msg.OVERRIDE;

  msg.taskspace_trajectory_points.clear();

  msg.taskspace_trajectory_points.push_back(data);

  // publish the message
  headTrajPublisher.publish(msg);
}
