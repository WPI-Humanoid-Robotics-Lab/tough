//Author: Syon Khosla
//Date (of last edit): April 24, 2018
//Change to task area thing

#include <tough_controller_interface/arm_control_interface.h>
#include <tough_controller_interface/gripper_control_interface.h>
int main(int argc, char **argv)
{
    // Initialize a ros node
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Create an object of ArmControlInterface - used for actually altering the different aspects of the arms of the robot
    ArmControlInterface armInt(nh);

    RobotSide side = RobotSide::LEFT;
    RobotStateInformer *current_state_ = RobotStateInformer::getRobotStateInformer(nh);

    // change the Right arm position to ZeroPose. This is a non-blocking call.
    RobotDescription *rd_ = RobotDescription::getRobotDescription(nh);
    geometry_msgs::QuaternionStamped leftHandOrientation_;
    leftHandOrientation_.header.frame_id = rd_->getPelvisFrame();

    leftHandOrientation_.quaternion.x = 0.604;
    leftHandOrientation_.quaternion.y = 0.434;
    leftHandOrientation_.quaternion.z = -0.583;
    leftHandOrientation_.quaternion.w = 0.326;

    GripperControlInterface gripper_(nh);
    ROS_INFO("Opening Grippers");
    gripper_.controlGripper(side, GRIPPER_STATE::OPEN);

    geometry_msgs::Pose finalGoal;
    geometry_msgs::Point finalPoint;

    finalPoint.x = 0.7;
    finalPoint.y = 0.7;
    finalPoint.z = 0.7;

    finalGoal.orientation = leftHandOrientation_;
    finalGoal.position = finalPoint;

    current_state_->transformQuaternion(leftHandOrientation_);

    current_state_->transformPoint(finalPoint, finalPoint, TOUGH_COMMON_NAMES::WORLD_TF, rd_->getPelvisFrame());
    finalPoint.x -= 0.05;

    current_state_->transformPoint(finalPoint, finalPoint, rd_->getPelvisFrame(), TOUGH_COMMON_NAMES::WORLD_TF);

    armInt.moveArmInTaskSpace(side, finalGoal, 3.0);

    return 0;
}

//End of program
