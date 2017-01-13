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
    ValkyrieWalker walk(nh, 0.305, 0.435);
    walk.setSwing_height(0.15);

    sm state = PREPARE_START;

    while(ros::ok())
    {
        switch (state)
        {
        case PREPARE_START:
        {
            pelvisTraj.controlPelvisHeight(1.06);
            walk.WalkNStepsForward(1,0.35,0,false,RIGHT);
            ros::Duration(0.5).sleep();
            armtraj.buttonPressPrepareArm(RIGHT);
            state = WALK_TO_DOOR;

            break;
        }
        case WALK_TO_DOOR:
        {
            walk.WalkNStepsForward(5,0.51,0, false, RIGHT);
            OrientChest(0,0,3.5,pub);
            ros::Duration(0.25).sleep();
            OrientChest(0,0,0,pub);
            armtraj.walkPoseArm(RIGHT);
            walk.setWalkParms(0.31, 0.44, 0);
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

    geometry_msgs::Quaternion angles2;
    angles2.x = angles.getX();
    angles2.y = angles.getY();
    angles2.z = angles.getZ();
    angles2.w = angles.getW();

    trajPoint.orientation = angles2;
    trajPoint.time = 0.0;
    trajPointsVec.push_back(trajPoint);

    msg.execution_mode = 0;
    msg.unique_id = 13;
    msg.taskspace_trajectory_points = trajPointsVec;

    pub.publish(msg);

}
