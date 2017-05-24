#include<val_controllers/val_arm_navigation.h>
#include<stdlib.h>
#include<visualization_msgs/Marker.h>

int armTrajectory::arm_id = -1;

//add default pose for both arms. the values of joints are different.
armTrajectory::armTrajectory(ros::NodeHandle nh):nh_(nh),
    ZERO_POSE{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    DEFAULT_RIGHT_POSE{-0.2f, 1.2f, 0.7222f, 1.5101f, 0.0f, 0.0f, 0.0f},
    DEFAULT_LEFT_POSE{-0.2f, -1.2f, 0.7222f, -1.5101f, 0.0f, 0.0f, 0.0f},
    NUM_ARM_JOINTS(7){
    //tf_listener_ = new tf2_ros::TransformListener(this->tf_buffer_);
    armTrajectoryPublisher = nh_.advertise<ihmc_msgs::ArmTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/arm_trajectory", 1,true);
    handTrajectoryPublisher = nh_.advertise<ihmc_msgs::HandDesiredConfigurationRosMessage>("/ihmc_ros/valkyrie/control/hand_desired_configuration", 1,true);
    taskSpaceTrajectoryPublisher = nh_.advertise<ihmc_msgs::HandTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/hand_trajectory", 1, true);
    markerPub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 1, true);
    stateInformer_ = RobotStateInformer::getRobotStateInformer(nh_);
    //this->armTrajectorySunscriber = nh_.subscribe("/ihmc_ros/valkyrie/output/ha", 20,&ValkyrieWalker::footstepStatusCB, this);
    joint_limits_left_.resize(NUM_ARM_JOINTS);
    joint_limits_right_.resize(NUM_ARM_JOINTS);

    joint_limits_left_[0]={-2.85,2.0};
    joint_limits_left_[1]={-1.519,1.266};
    joint_limits_left_[2]={-3.1,2.18};
    joint_limits_left_[3]={-2.174,0.12};
    joint_limits_left_[4]={-2.019,3.14};
    joint_limits_left_[5]={-0.62,0.625};
    joint_limits_left_[6]={-0.36,0.49};

    joint_limits_right_[0]={-2.85,2.0};
    joint_limits_right_[1]={-1.266,1.519};
    joint_limits_right_[2]={-3.1,2.18};
    joint_limits_right_[3]={-0.12,2.174};
    joint_limits_right_[4]={-2.019,3.14};
    joint_limits_right_[5]={-0.625,0.62};
    joint_limits_right_[6]={-0.48,0.36};

}

armTrajectory::~armTrajectory(){
    armTrajectorySunscriber.shutdown();
}


void armTrajectory::appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage &msg, float time, std::vector<float> pos)
{

    std::vector<std::pair<float, float> > joint_limits_;
    joint_limits_= msg.robot_side == LEFT ? joint_limits_left_ : joint_limits_right_;
    for (int i=0;i<NUM_ARM_JOINTS;i++)
    {
        ihmc_msgs::TrajectoryPoint1DRosMessage p;
        pos[i] = pos[i] < joint_limits_[i].first  ? joint_limits_[i].first : pos[i];
        pos[i] = pos[i] > joint_limits_[i].second ? joint_limits_[i].second : pos[i];
        p.time = time;
        p.position = pos[i];
        p.velocity = 0;
        p.unique_id = armTrajectory::arm_id;
        msg.joint_trajectory_messages[i].trajectory_points.push_back(p);
    }

    return;
}

void armTrajectory::moveToDefaultPose(armSide side)
{
    ihmc_msgs::ArmTrajectoryRosMessage arm_traj;
    arm_traj.joint_trajectory_messages.clear();

    arm_traj.joint_trajectory_messages.resize(NUM_ARM_JOINTS);
    arm_traj.robot_side = side;
    armTrajectory::arm_id--;
    arm_traj.unique_id = armTrajectory::arm_id;
    if(side == RIGHT)
        appendTrajectoryPoint(arm_traj, 1, DEFAULT_RIGHT_POSE);
    else
        appendTrajectoryPoint(arm_traj, 1, DEFAULT_LEFT_POSE);

    armTrajectoryPublisher.publish(arm_traj);

}

void armTrajectory::moveToZeroPose(armSide side)
{
    ihmc_msgs::ArmTrajectoryRosMessage arm_traj;
    arm_traj.joint_trajectory_messages.clear();

    arm_traj.joint_trajectory_messages.resize(NUM_ARM_JOINTS);
    arm_traj.robot_side = side;
    armTrajectory::arm_id--;
    arm_traj.unique_id = armTrajectory::arm_id;

    appendTrajectoryPoint(arm_traj, 2, ZERO_POSE);

    armTrajectoryPublisher.publish(arm_traj);

}

void armTrajectory::moveArmJoints(const armSide side, const std::vector<std::vector<float>> &arm_pose,const float time){

    ihmc_msgs::ArmTrajectoryRosMessage arm_traj;
    arm_traj.joint_trajectory_messages.clear();


    arm_traj.joint_trajectory_messages.resize(NUM_ARM_JOINTS);
    arm_traj.robot_side = side;
    armTrajectory::arm_id--;
    arm_traj.unique_id = armTrajectory::arm_id;
    for(auto i=arm_pose.begin(); i != arm_pose.end(); i++){
           if(i->size() != NUM_ARM_JOINTS)
           ROS_WARN("Check number of trajectory points");
        appendTrajectoryPoint(arm_traj, time/arm_pose.size(), *i);
    }

    armTrajectoryPublisher.publish(arm_traj);
}


void armTrajectory::moveArmJoints(std::vector<armJointData> &arm_data){

    ihmc_msgs::ArmTrajectoryRosMessage arm_traj_r;
    ihmc_msgs::ArmTrajectoryRosMessage arm_traj_l;

    arm_traj_r.joint_trajectory_messages.clear();
    arm_traj_r.joint_trajectory_messages.resize(NUM_ARM_JOINTS);


    armTrajectory::arm_id--;
    arm_traj_r.unique_id = armTrajectory::arm_id;
    armTrajectory::arm_id--;
    arm_traj_l.joint_trajectory_messages.clear();
    arm_traj_l.joint_trajectory_messages.resize(NUM_ARM_JOINTS);
    arm_traj_l.unique_id = armTrajectory::arm_id;

    for(std::vector<armJointData>::iterator i=arm_data.begin(); i != arm_data.end(); i++){

        if(i->arm_pose.size() != NUM_ARM_JOINTS){
            ROS_INFO("Check number of trajectory points");
            return;
        }

        if(i->side == RIGHT){
            arm_traj_r.robot_side = i->side;
            appendTrajectoryPoint(arm_traj_r, i->time, i->arm_pose);
        }

        else {
            arm_traj_l.robot_side = i->side;
            appendTrajectoryPoint(arm_traj_l, i->time, i->arm_pose);
        }

    }

    armTrajectoryPublisher.publish(arm_traj_r);
    ros::Duration(0.02).sleep();
    armTrajectoryPublisher.publish(arm_traj_l);
}


void armTrajectory::moveArmMessage(const ihmc_msgs::ArmTrajectoryRosMessage &msg){
    this->armTrajectoryPublisher.publish(msg);
    armTrajectory::arm_id--;

}
int armTrajectory::getnumArmJoints() const
{
    return NUM_ARM_JOINTS;
}

void armTrajectory::closeHand(const armSide side)
{
    ihmc_msgs::HandDesiredConfigurationRosMessage msg;
    msg.robot_side = side;
    msg.hand_desired_configuration = msg.CLOSE;
    armTrajectory::arm_id--;
    msg.unique_id = armTrajectory::arm_id;
    this->handTrajectoryPublisher.publish(msg);
}

void armTrajectory::moveArmInTaskSpaceMessage(const armSide side, const ihmc_msgs::SE3TrajectoryPointRosMessage &point, int baseForControl)
{
    ihmc_msgs::HandTrajectoryRosMessage msg;
    msg.robot_side = side;
    msg.base_for_control = baseForControl;
    msg.taskspace_trajectory_points.push_back(point);
    msg.execution_mode = msg.OVERRIDE;
    armTrajectory::arm_id--;
    msg.unique_id = armTrajectory::arm_id;
    taskSpaceTrajectoryPublisher.publish(msg);
}

void armTrajectory::moveArmInTaskSpace(const armSide side, const geometry_msgs::Pose &pose, const float time)
{
  ihmc_msgs::SE3TrajectoryPointRosMessage point;
  poseToSE3TrajectoryPoint(pose, point);
  point.time = time;
  this->moveArmInTaskSpaceMessage(side, point);
}

void armTrajectory::moveArmInTaskSpace(std::vector<armTaskSpaceData> &arm_data, int baseForControl)
{
  ihmc_msgs::HandTrajectoryRosMessage msg_l;
  ihmc_msgs::HandTrajectoryRosMessage msg_r;

  msg_l.taskspace_trajectory_points.clear();
  msg_r.taskspace_trajectory_points.clear();
  armTrajectory::arm_id--;
  msg_l.unique_id = armTrajectory::arm_id;
  msg_l.base_for_control = baseForControl;
  msg_l.execution_mode = msg_l.OVERRIDE;
  armTrajectory::arm_id--;
  msg_r.unique_id = armTrajectory::arm_id;
  msg_r.base_for_control = baseForControl;
  msg_r.execution_mode = msg_r.OVERRIDE;


  for(std::vector<armTaskSpaceData>::iterator i=arm_data.begin(); i != arm_data.end(); i++){

      if(i->side == RIGHT){
          msg_r.robot_side = i->side;
          ihmc_msgs::SE3TrajectoryPointRosMessage point;
          poseToSE3TrajectoryPoint(i->pose, point);
          point.time = i->time;
          msg_r.taskspace_trajectory_points.push_back(point);
      }

      else {
        msg_l.robot_side = i->side;
        ihmc_msgs::SE3TrajectoryPointRosMessage point;
        poseToSE3TrajectoryPoint(i->pose, point);
        point.time = i->time;
        msg_l.taskspace_trajectory_points.push_back(point);
      }

  }

  taskSpaceTrajectoryPublisher.publish(msg_r);
  ros::Duration(0.02).sleep();
  taskSpaceTrajectoryPublisher.publish(msg_l);
}

void armTrajectory::poseToSE3TrajectoryPoint(const geometry_msgs::Pose &pose, ihmc_msgs::SE3TrajectoryPointRosMessage &point)
{

  point.position.x = pose.position.x;
  point.position.y = pose.position.y;
  point.position.z = pose.position.z;
  point.orientation.w = pose.orientation.w;
  point.orientation.x = pose.orientation.x;
  point.orientation.y = pose.orientation.y;
  point.orientation.z = pose.orientation.z;
  point.unique_id = 255;
  return;
}

void armTrajectory::moveArmTrajectory(const armSide side, const trajectory_msgs::JointTrajectory &traj){

    ihmc_msgs::ArmTrajectoryRosMessage arm_traj;
    arm_traj.joint_trajectory_messages.clear();

    arm_traj.joint_trajectory_messages.resize(NUM_ARM_JOINTS);
    arm_traj.robot_side = side;
    armTrajectory::arm_id--;
    arm_traj.unique_id = armTrajectory::arm_id;

    for(auto i=traj.points.begin(); i < traj.points.end(); i++){
        appendTrajectoryPoint(arm_traj, *i);
    }
    ROS_INFO("Publishing Arm Trajectory");
    armTrajectoryPublisher.publish(arm_traj);
}

bool armTrajectory::nudgeArm(const armSide side, const direction drct, float nudgeStep){

    geometry_msgs::PoseStamped      world_values;
    geometry_msgs::PoseStamped      palm_values;

    world_values.header.frame_id=VAL_COMMON_NAMES::WORLD_TF;

    std::string target_frame = side == LEFT ? "/leftMiddleFingerPitch1Link" : "/rightMiddleFingerPitch1Link";

    try{
        tf::StampedTransform            tf_palm_values;
        tf_listener_.waitForTransform(VAL_COMMON_NAMES::PELVIS_TF,target_frame, ros::Time(0),ros::Duration(2));
        tf_listener_.lookupTransform(VAL_COMMON_NAMES::PELVIS_TF, target_frame, ros::Time(0),tf_palm_values);

        tf::pointTFToMsg(tf_palm_values.getOrigin(), palm_values.pose.position);
        tf::quaternionTFToMsg(tf_palm_values.getRotation(), palm_values.pose.orientation);
        palm_values.header.frame_id=VAL_COMMON_NAMES::PELVIS_TF;

    }
    catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        ros::spinOnce();
        return false;
    }

    if     (drct == direction::LEFT)     palm_values.pose.position.y += nudgeStep;
    else if(drct == direction::RIGHT)    palm_values.pose.position.y -= nudgeStep;
    else if(drct == direction::UP)       palm_values.pose.position.z += nudgeStep;
    else if(drct == direction::DOWN)     palm_values.pose.position.z -= nudgeStep;
    else if(drct == direction::FRONT)    palm_values.pose.position.x += nudgeStep;
    else if(drct == direction::BACK)     palm_values.pose.position.x -= nudgeStep;

    try{
        tf_listener_.waitForTransform(VAL_COMMON_NAMES::PELVIS_TF,VAL_COMMON_NAMES::WORLD_TF, ros::Time(0),ros::Duration(2));
        tf_listener_.transformPose(VAL_COMMON_NAMES::WORLD_TF,palm_values,world_values);
    }
    catch (tf::TransformException ex) {
        ROS_WARN("%s",ex.what());
        ros::spinOnce();
        return false;
    }

    moveArmInTaskSpace(side,world_values.pose, 0.0f);
    return true;
}


bool armTrajectory::nudgeArmLocal(const armSide side, const direction drct, float nudgeStep){

    geometry_msgs::PoseStamped      world_values;
    world_values.header.frame_id=VAL_COMMON_NAMES::WORLD_TF;

    std::string target_frame = side == LEFT ? "/leftMiddleFingerPitch1Link" : "/rightMiddleFingerPitch1Link";
    int signInverter = side == LEFT ? 1 : -1;

    geometry_msgs::PoseStamped palm_values;
    palm_values.header.frame_id = target_frame;
    palm_values.pose.orientation.w = 1.0f;

    if     (drct == direction::FRONT)     palm_values.pose.position.y += nudgeStep * signInverter;
    else if(drct == direction::BACK)      palm_values.pose.position.y -= nudgeStep * signInverter;
    else if(drct == direction::LEFT)      palm_values.pose.position.z += nudgeStep;
    else if(drct == direction::RIGHT)     palm_values.pose.position.z -= nudgeStep;
    else if(drct == direction::UP)        palm_values.pose.position.x += nudgeStep;
    else if(drct == direction::DOWN)      palm_values.pose.position.x -= nudgeStep;

    try{
        tf_listener_.waitForTransform(VAL_COMMON_NAMES::PELVIS_TF,VAL_COMMON_NAMES::WORLD_TF, ros::Time(0),ros::Duration(2));
        tf_listener_.transformPose(VAL_COMMON_NAMES::WORLD_TF,palm_values,world_values);
    }
    catch (tf::TransformException ex) {
        ROS_WARN("%s",ex.what());
        ros::spinOnce();
        return false;
    }

    moveArmInTaskSpace(side,world_values.pose, 0.0f);
    return true;
}

void armTrajectory::appendTrajectoryPoint(ihmc_msgs::ArmTrajectoryRosMessage &msg, trajectory_msgs::JointTrajectoryPoint point)
{

    if(point.positions.size() != NUM_ARM_JOINTS) {
        ROS_WARN("Check number of trajectory points");
        return;
    }
    std::vector<std::pair<float, float> > joint_limits_;
    joint_limits_= msg.robot_side == LEFT ? joint_limits_left_ : joint_limits_right_;

    for (int i=0;i<NUM_ARM_JOINTS;i++)
    {
        ihmc_msgs::TrajectoryPoint1DRosMessage p;
//        point.positions[i] = point.positions[i] <= joint_limits_[i].first  ? joint_limits_[i].first : point.positions[i];
//        point.positions[i] = point.positions[i] >= joint_limits_[i].second ? joint_limits_[i].second : point.positions[i];
        if(point.positions[i] <= joint_limits_[i].first)
        {
            std::cout<<"wrapped lower point "<<point.positions[i]<<"\n";
            point.positions[i]=joint_limits_[i].first;

        }
        else if(point.positions[i] >= joint_limits_[i].second)
        {
            std::cout<<"wrapped upper point "<<point.positions[i]<<"\n";
            point.positions[i]=joint_limits_[i].second;

        }
        p.time = point.time_from_start.toSec();
        p.position = point.positions[i];
        p.velocity = point.velocities[i];
        p.unique_id = armTrajectory::arm_id;
        msg.joint_trajectory_messages[i].trajectory_points.push_back(p);
    }

    return;
}


//armside is used to determine which side to use.
//transforms the poses to the worldframe regardless.
//POSES MUST BE IN WORLD FRAME;
//Add conversion of posestamped to world frame if it not already in world frame
bool armTrajectory::generate_task_space_data(const std::vector<geometry_msgs::PoseStamped>& input_poses,const armSide input_side,const float desired_time, std::vector<armTrajectory::armTaskSpaceData> &arm_data_vector)
{

  float time_delta = desired_time == 0 ? 0 : desired_time/input_poses.size();
  for(int i=0 ; i < input_poses.size(); i++)
  {
    geometry_msgs::PoseStamped input_pose=input_poses.at(i);
    armTrajectory::armTaskSpaceData task_space_data;
    task_space_data.side = input_side;
    task_space_data.pose = input_pose.pose;
    task_space_data.time = time_delta;

    arm_data_vector.push_back(task_space_data);
  }
  return true;
}

bool armTrajectory::moveArmJoint(const armSide side, int jointNumber, const float targetAngle) {

    ros::spinOnce(); //ensure that the joints are updated
    std::string param = side == LEFT ? "left_arm" : "right_arm";

    std::vector<float> positions;
    if (stateInformer_->getJointPositions(param, positions)){

        for (auto i : positions){
            std::cout<<i<<" ";
        }
        std::cout<<std::endl;

        positions[jointNumber] = targetAngle;

        for (auto i : positions){
            std::cout<<i<<" ";
        }
        std::cout<<std::endl;

        std::vector<std::vector<float>> trajectory;
        trajectory.push_back(positions);
        moveArmJoints(side,trajectory,1.0f );
        return true;
    }
    return false;
}
