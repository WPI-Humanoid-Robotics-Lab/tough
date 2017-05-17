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
    walker_->raiseLeg(side,raise_factor*stepHeight);

}
