#include <val_control/val_chest_navigation.h>
#include <tf/transform_listener.h>
#include <val_common/val_common_names.h>

#define TO_RADIANS M_PI / 180.0 //goes probably in utils which stores similar math operation parameters

chestTrajectory::chestTrajectory(ros::NodeHandle nh):nh_(nh)
{
    chestTrajPublisher =
            nh_.advertise<ihmc_msgs::ChestTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/chest_trajectory",1,true);
}

chestTrajectory::~chestTrajectory()
{
}

void chestTrajectory::controlChest(float roll , float pitch , float yaw, float time)
{
    ihmc_msgs::ChestTrajectoryRosMessage msg;
    ihmc_msgs::SO3TrajectoryPointRosMessage data;
    roll  =  roll*TO_RADIANS;
    pitch = pitch*TO_RADIANS;
    yaw   =   yaw*TO_RADIANS;

    data.time = time;
    tf::Quaternion quatInPelvisFrame;
    quatInPelvisFrame.setRPY(roll,pitch,yaw);

    //transorm point from pelvis to world frame
    tf::TransformListener listener;

    geometry_msgs::QuaternionStamped quatInWorldFrame;
    quatInWorldFrame.header.frame_id= VAL_COMMON_NAMES::PELVIS_TF;
    quatInWorldFrame.header.stamp = ros::Time(0);
    tf::quaternionTFToMsg(quatInPelvisFrame, quatInWorldFrame.quaternion);

    try
    {
        listener.waitForTransform(VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::PELVIS_TF, ros::Time(0), ros::Duration(3.0));
        listener.transformQuaternion(VAL_COMMON_NAMES::WORLD_TF, quatInWorldFrame, quatInWorldFrame);

    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("%s",ex.what());
        return;
    }
    data.orientation = quatInWorldFrame.quaternion;

    geometry_msgs::Vector3 v;
    v.x = 0.3;
    v.y = 0.3;
    v.z = 0.3;
    data.angular_velocity = v;

    msg.unique_id =13;
    msg.execution_mode = 0;


    msg.taskspace_trajectory_points.push_back(data);

    // publish the message
    chestTrajPublisher.publish(msg);
}
