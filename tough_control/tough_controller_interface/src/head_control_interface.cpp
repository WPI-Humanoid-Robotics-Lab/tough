#include <tough_controller_interface/head_control_interface.h>
#include <tf/transform_listener.h>
#include <tough_common/robot_state.h>

const double degToRad = M_PI / 180;

HeadControlInterface::HeadControlInterface(ros::NodeHandle nh):ToughControllerInterface(nh)
{
  neckTrajPublisher =
          nh_.advertise<ihmc_msgs::NeckTrajectoryRosMessage>( control_topic_prefix_ + "/neck_trajectory",1,true);
  headTrajPublisher =
            nh_.advertise<ihmc_msgs::HeadTrajectoryRosMessage>(control_topic_prefix_ + "/head_trajectory",1,true);
  NUM_NECK_JOINTS = rd_->getNumberOfNeckJoints();
}

HeadControlInterface::~HeadControlInterface()
{
}

void HeadControlInterface::appendNeckTrajectoryPoint(ihmc_msgs::NeckTrajectoryRosMessage &msg, float time, std::vector<float> pos)
{
  for (int i = 0; i < NUM_NECK_JOINTS; i++)
  {
      ihmc_msgs::TrajectoryPoint1DRosMessage p;
      ihmc_msgs::OneDoFJointTrajectoryRosMessage t;
      t.trajectory_points.clear();

      p.time = time;
      p.position = pos[i];
      p.unique_id = HeadControlInterface::id_;
      t.trajectory_points.push_back(p);
      t.unique_id = HeadControlInterface::id_;
      msg.joint_trajectory_messages.push_back(t);
  }
}


void HeadControlInterface::moveHead(float roll, float pitch, float yaw, const float time)
{
  roll = degToRad * roll;
  pitch = degToRad * pitch;
  yaw = degToRad * yaw;

  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs::Quaternion quaternion;
  tf::quaternionTFToMsg(q, quaternion);
  moveHead(quaternion, time);
}

void HeadControlInterface::moveHead(const geometry_msgs::Quaternion &quaternion, const float time)
{
  ihmc_msgs::HeadTrajectoryRosMessage msg;
  ihmc_msgs::SO3TrajectoryPointRosMessage data;


  // transorm point from pelvis to world frame
  tf::TransformListener listener;

  geometry_msgs::QuaternionStamped quatInWorldFrame;
  quatInWorldFrame.header.frame_id = rd_->getPelvisFrame();
  quatInWorldFrame.header.stamp = ros::Time(0);
  quatInWorldFrame.quaternion = quaternion;

  state_informer_->transformQuaternion(quatInWorldFrame,quatInWorldFrame);

  data.orientation = quatInWorldFrame.quaternion;

  geometry_msgs::Vector3 v;
  v.x = 0.0;
  v.y = 0.0;
  v.z = 0.0;
  data.angular_velocity = v;

  HeadControlInterface::id_--;
  msg.unique_id = HeadControlInterface::id_;
  msg.execution_mode = msg.OVERRIDE;

  msg.taskspace_trajectory_points.clear();

  msg.taskspace_trajectory_points.push_back(data);

  // publish the message
  headTrajPublisher.publish(msg);
}


void HeadControlInterface::moveHead(const std::vector<std::vector<float> > &trajectory_points, const float time)
{
  ihmc_msgs::HeadTrajectoryRosMessage msg;

  HeadControlInterface::id_--;
  msg.unique_id = HeadControlInterface::id_;
  msg.execution_mode = msg.OVERRIDE;

  msg.taskspace_trajectory_points.clear();

  for(int i = 0; i < trajectory_points.size(); i++)
  {
    ihmc_msgs::SO3TrajectoryPointRosMessage data;

    float roll = degToRad * trajectory_points[i][0];
    float pitch = degToRad * trajectory_points[i][1];
    float yaw = degToRad * trajectory_points[i][2];

    data.time = time;
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    //transorm point from pelvis to world frame
    tf::TransformListener listener;

    geometry_msgs::QuaternionStamped quatInWorldFrame;
    quatInWorldFrame.header.frame_id = rd_->getPelvisFrame();
    quatInWorldFrame.header.stamp = ros::Time(0);

    tf::quaternionTFToMsg(q, quatInWorldFrame.quaternion);

    state_informer_->transformQuaternion(quatInWorldFrame,quatInWorldFrame);

    data.orientation = quatInWorldFrame.quaternion;

    geometry_msgs::Vector3 v;
    v.x = 0.0;
    v.y = 0.0;
    v.z = 0.0;
    data.angular_velocity = v;

    msg.taskspace_trajectory_points.push_back(data);
  }
  // publish the message
  headTrajPublisher.publish(msg);
}

void HeadControlInterface::moveNeckJoints(const std::vector<std::vector<float> > &neck_pose, const float time)
{
  ihmc_msgs::NeckTrajectoryRosMessage msg;

  HeadControlInterface::id_--;
  msg.unique_id = HeadControlInterface::id_;

  // Add all neck trajectory points to the trajectory message
  for(int i = 0; i < neck_pose.size(); i++){
    if(neck_pose[i].size() != NUM_NECK_JOINTS)
      ROS_ERROR("Check number of trajectory points");
    appendNeckTrajectoryPoint(msg, time / neck_pose.size(), neck_pose[i]);
  }
  // publish the message
  neckTrajPublisher.publish(msg);
}

int HeadControlInterface::getNumNeckJoints() const
{
    return NUM_NECK_JOINTS;
}
