
#include <val_controllers/val_arm_navigation.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>

bool executeTrajectory2Stage(armTrajectory traj, geometry_msgs::Pose &pose){

    traj.moveArmInTaskSpace(armSide::RIGHT, pose, 2.0f);
    ros::Duration(3.0).sleep();

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "com_experiment");
    ros::NodeHandle nh;

    armTrajectory armTraj(nh);

    geometry_msgs::Pose goal;

    // read last goal position, iterator, and runID from log file
    goal.position.x = 0.28;
    goal.position.y = -0.7;
    goal.position.z = 1.0;

    goal.orientation.w=1.0;

    int iterator;
    int runID = 0;

    while (ros::ok()){
        for (; iterator < 100; ++iterator){
            //execute trajectory using 2-stage planning
            executeTrajectory2Stage(armTraj, goal);

            //check if robot is standing

            //log the goal pose, COM location, and run ID

            //reset to default pose

            ++runID;
        }

        // change the goal pose

        iterator = 0;
    }
    return 0;
}


