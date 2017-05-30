#include "val_task3/val_stairclimb.h"
#include "val_common/val_common_defines.h"


StairClimb::StairClimb(ros::NodeHandle n)
{
    walker_ = new ValkyrieWalker(n, 0.7, 0.7, 0, 0.18);
}

bool StairClimb::walkToSetPosition(geometry_msgs::Pose2D goal)
{
    walker_->walkToGoal(goal);
}

bool StairClimb::takeStep(armSide side,float stepLength, float stepHeight)
{
    /*
     * Stair case climbing involves six stages
     * Stage 1: Lift Leg vertically high to 1.5 times the step height
     * Stage 2: Place Leg on the Step
     * Stage 3: Bend the chest forward and move the hands up
     * Stage 4: Lift the swing leg in a way to curl behind the body
     * Stage 5: Place swing foot on the stair
     * Stage 6: Orient the chest back to zero position
    */

    // Stage 1
    float raise_factor=1.5;
    walker_->raiseLeg(side,stepHeight,stepLength);

}

int main(int argc, char** argv){

    ros::init(argc, argv, "stair_climb");
    ros::NodeHandle nh;

//    StepDetector step(nh);
//    bool foundSteps = false;
//    std::vector<double> coefficients;
//    geometry_msgs::Point dirVector;
//    geometry_msgs::Point stairLoc;

//    coefficients = { 0.0588899, -2.50763,	0, 2.91158};
//    dirVector.x = 1;
//    dirVector.y = 0.0234843;
//    dirVector.z = 0;
//    stairLoc.x = 3.56882;
//    stairLoc.y = -1.07728;
//    stairLoc.z = 0.758028;
//    std::vector<pcl::PointXYZ> steps_location;

//    // step_location will have the stair locations in world coordinate frame
//    foundSteps = step.getStepsPosition(coefficients, dirVector, stairLoc, steps_location);

//    geometry_msgs::Pose2D walkPoint;
//    walkPoint.x= 3.56882;
//    walkPoint.y=-1.07728;
//    walkPoint.theta=0.0;

//    StairClimb stair(nh);
//    stair.walkToSetPosition(walkPoint);

    return 0;
}
