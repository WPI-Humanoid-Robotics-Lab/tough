#include <tough_controller_interface/chest_control_interface.h>
#include <tf/transform_listener.h>
#include <tough_common/val_common_names.h>

#define TO_RADIANS M_PI / 180.0 //goes probably in utils which stores similar math operation parameters

ChestControlInterface::ChestControlInterface(ros::NodeHandle nh):nh_(nh)
{
    std::string robot_name;
    nh.getParam("ihmc_ros/robot_name", robot_name);

    chestTrajPublisher_ =
            nh_.advertise<ihmc_msgs::ChestTrajectoryRosMessage>("/ihmc_ros/"+ robot_name +"/control/chest_trajectory",1,true);
    state_informer_ = RobotStateInformer::getRobotStateInformer(nh_);
    rd_ = RobotDescription::getRobotDescription(nh_);
}

ChestControlInterface::~ChestControlInterface()
{
}

void ChestControlInterface::controlChest(float roll , float pitch , float yaw, float time)
{
    ihmc_msgs::ChestTrajectoryRosMessage msg;
    ihmc_msgs::SO3TrajectoryPointRosMessage data;
    roll  =  roll*TO_RADIANS;
    pitch = pitch*TO_RADIANS;
    yaw   =   yaw*TO_RADIANS;

    data.time = time;
    tf::Quaternion quatInPelvisFrame;
    quatInPelvisFrame.setRPY(roll,pitch,yaw);
    geometry_msgs::Quaternion quatInWorldFrame;
    tf::quaternionTFToMsg(quatInPelvisFrame, quatInWorldFrame);
    state_informer_->transformQuaternion(quatInWorldFrame, quatInWorldFrame, rd_->getPelvisFrame());

//    //transorm point from pelvis to world frame
//    tf::TransformListener listener;

//    geometry_msgs::QuaternionStamped quatInWorldFrame;
//    quatInWorldFrame.header.frame_id= VAL_COMMON_NAMES::PELVIS_TF;
//    quatInWorldFrame.header.stamp = ros::Time(0);
//    tf::quaternionTFToMsg(quatInPelvisFrame, quatInWorldFrame.quaternion);

//    try
//    {
//        listener.waitForTransform(VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF, ros::Time(0), ros::Duration(3.0));
//        listener.transformQuaternion(VAL_COMMON_NAMES::WORLD_TF, quatInWorldFrame, quatInWorldFrame);

//    }
//    catch (tf::TransformException ex)
//    {
//        ROS_WARN("%s",ex.what());
//        return;
//    }
    data.orientation = quatInWorldFrame;

    geometry_msgs::Vector3 v;
    v.x = 0.3;
    v.y = 0.3;
    v.z = 0.3;
    data.angular_velocity = v;

    msg.unique_id =13;
    msg.execution_mode = 0;


    msg.taskspace_trajectory_points.push_back(data);

    // publish the message
    chestTrajPublisher_.publish(msg);
}
