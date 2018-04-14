#include <tough_controller_interface/head_control_interface.h>
#include <tf/transform_listener.h>
#include <tough_controller_interface/robot_state.h>

int HeadControlInterface::head_id = -1;

const double degToRad = M_PI / 180;

HeadControlInterface::HeadControlInterface(ros::NodeHandle nh):nh_(nh)
{
    std::string robot_name;
    nh.getParam("ihmc_ros/robot_name", robot_name);

  neckTrajPublisher =
          nh_.advertise<ihmc_msgs::NeckTrajectoryRosMessage>("/ihmc_ros/"+ robot_name +"/control/neck_trajectory",1,true);
  headTrajPublisher =
            nh_.advertise<ihmc_msgs::HeadTrajectoryRosMessage>("/ihmc_ros/"+ robot_name +"/control/head_trajectory",1,true);
  currentState_ = RobotStateInformer::getRobotStateInformer(nh);
  rd_ = RobotDescription::getRobotDescription(nh_);
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
      p.velocity = 0;
      p.unique_id = HeadControlInterface::head_id;
      t.trajectory_points.push_back(p);
      t.unique_id = HeadControlInterface::head_id;
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

  currentState_->transformQuaternion(quatInWorldFrame,quatInWorldFrame);

  data.orientation = quatInWorldFrame.quaternion;

  geometry_msgs::Vector3 v;
  v.x = 0.0;
  v.y = 0.0;
  v.z = 0.0;
  data.angular_velocity = v;

  HeadControlInterface::head_id--;
  msg.unique_id = HeadControlInterface::head_id;
  msg.execution_mode = msg.OVERRIDE;

  msg.taskspace_trajectory_points.clear();

  msg.taskspace_trajectory_points.push_back(data);

  // publish the message
  headTrajPublisher.publish(msg);
}


void HeadControlInterface::moveHead(const std::vector<std::vector<float> > &trajectory_points, const float time)
{
  ihmc_msgs::HeadTrajectoryRosMessage msg;

  HeadControlInterface::head_id--;
  msg.unique_id = HeadControlInterface::head_id;
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

    currentState_->transformQuaternion(quatInWorldFrame,quatInWorldFrame);

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

  HeadControlInterface::head_id--;
  msg.unique_id = HeadControlInterface::head_id;

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

geometry_msgs::Quaternion HeadControlInterface::getHeadOrientation()
{
        geometry_msgs::Pose head_pose;
        currentState_->getCurrentPose(rd_->getHeadFrame(), head_pose, rd_->getWorldFrame());

        return head_pose.orientation;
}
