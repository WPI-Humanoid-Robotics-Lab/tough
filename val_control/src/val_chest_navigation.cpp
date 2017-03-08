#include <val_control/val_chest_navigation.h>

const double halfC = M_PI / 180; //goes probably in utils which stores similar math operation parameters
chestTrajectory::chestTrajectory(ros::NodeHandle nh):nh_(nh)
{
    chestTrajPublisher =
            nh_.advertise<ihmc_msgs::ChestTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/chest_trajectory",1,true);
}

chestTrajectory::~chestTrajectory()
{
}

void chestTrajectory::controlChest(float roll , float pitch , float yaw)
{

    ihmc_msgs::ChestTrajectoryRosMessage msg;
    ihmc_msgs::SO3TrajectoryPointRosMessage data;
    roll  =  roll*halfC;
    pitch = pitch*halfC;
    yaw   =   yaw*halfC;

    data.time = 0.0;
    tf::Quaternion q;
    q.setRPY(roll,pitch,yaw);
    tf::quaternionTFToMsg(q,data.orientation);
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
