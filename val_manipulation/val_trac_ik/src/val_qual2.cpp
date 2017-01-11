#include <ros/ros.h>
#include <val_manipulation/val_arm_navigation_.h>
#include <val_manipulation/val_pelvis_navigation.h>
#include <val_footstep/ValkyrieWalker.h>
#include <ihmc_msgs/ChestTrajectoryRosMessage.h>
#include <tf2/utils.h>

enum sm {
    WALK_TO_DOOR = 0,
    PREPARE_START,
    PRESS_BUTTON_RETRACT,
    WALK_THROUGH_DOOR,
    EXIT
};

void executeSM(sm state);
bool OrientChest(float roll, float pitch, float yaw, ros::Publisher pub);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Qual2");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<ihmc_msgs::ChestTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/chest_trajectory", 1);
    ros::Rate loop_rate(10);

    armTrajectory armtraj(nh);
    pelvisTrajectory pelvisTraj(nh);

    // optimum values for walking
    ValkyrieWalker walk(nh, 0.47, 0.47);
    walk.setSwing_height(0.15);

    sm state = PREPARE_START;

    while(ros::ok())
    {
        switch (state)
        {
        case PREPARE_START:
        {
            //ROS_INFO("preparing Robot");
            //ROS_INFO("get the pelvis up");
            pelvisTraj.controlPelvisHeight(1.06);
            ros::Duration(0.5).sleep();
            //ROS_INFO("prepare arm and walk to start");
            armtraj.buttonPressPrepareArm(RIGHT);
            walk.WalkNStepsForward(1,0.35,0,false,RIGHT);

            state = WALK_TO_DOOR;

            break;
        }
        case WALK_TO_DOOR:
        {
            //ROS_INFO("walking to the door");
            walk.WalkNStepsForward(5,0.51,0, false, RIGHT);

            state = PRESS_BUTTON_RETRACT;
            break;
        }
        case PRESS_BUTTON_RETRACT:
        {
            //ROS_INFO("Press button and retract");
            //ROS_INFO("press button");
            OrientChest(0,0,3.5,pub);
            ros::Duration(0.5).sleep();
            //ROS_INFO("retract arm");
            armtraj.walkPoseArm(RIGHT);
            OrientChest(0,0,0,pub);

            state = WALK_THROUGH_DOOR;

            break;
        }
        case WALK_THROUGH_DOOR:
        {
            //ROS_INFO("walking through to door");

            walk.WalkNStepsForward(4,0.5,0);
            state = EXIT;

            break;
        }

        default:
        EXIT:

            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


bool OrientChest(float roll, float pitch, float yaw, ros::Publisher pub){
    ihmc_msgs::ChestTrajectoryRosMessage msg;
    std::vector<ihmc_msgs::SO3TrajectoryPointRosMessage> trajPointsVec;
    ihmc_msgs::SO3TrajectoryPointRosMessage trajPoint;
    tf::Quaternion angles;
    angles.setRPY((tfScalar)roll*3.1427/180, (tfScalar)pitch*3.1427/180, (tfScalar)yaw*3.1427/180);
    geometry_msgs::Vector3 vel;
    vel.x= 0.3;
    vel.y = 0.3;
    vel.z = 0.3;
    //ROS_INFO("Executing chest trajectory");
    geometry_msgs::Quaternion angles2;
    angles2.x = angles.getX();
    angles2.y = angles.getY();
    angles2.z = angles.getZ();
    angles2.w = angles.getW();


    trajPoint.angular_velocity = vel;
    trajPoint.orientation = angles2;
    trajPoint.time = 0;
    trajPointsVec.push_back(trajPoint);

    msg.execution_mode = 0;
    msg.unique_id = 13;
    msg.taskspace_trajectory_points = trajPointsVec;

//    ros::Publisher pub = nh.advertise<ihmc_msgs::ChestTrajectoryRosMessage>("/ihmc_ros/valkyrie/control/chest_trajectory", 1);


    pub.publish(msg);

}
