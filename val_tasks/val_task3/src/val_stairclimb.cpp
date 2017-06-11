#include "val_task3/val_stairclimb.h"
#include "val_common/val_common_defines.h"
#include "val_footstep/ValkyrieWalker.h"


StairClimb::StairClimb(ros::NodeHandle n)
{
    walker_ = new ValkyrieWalker(n, 0.7, 0.7, 0, 0.18);
    robot_state_=RobotStateInformer::getRobotStateInformer(n);
    chest_controller_ = new chestTrajectory(n);

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

    float raise_factor=1.5;
    float lower_factor=-0.5;
    float pitch_angle=20;
    float curl_radius=0.2;


    // second leg movement
    walker_->curlLeg(side,curl_radius);
    ros::Duration(1).sleep();
    walker_->placeLeg(side,raise_factor*stepHeight);
    ros::Duration(1).sleep();
    walker_->nudgeFoot(side,stepLength);
    ros::Duration(1).sleep();
    walker_->raiseLeg(side,lower_factor*stepHeight,0.0);
    ros::Duration(1).sleep();
    walker_->load_eff(side,EE_LOADING::LOAD);
    ros::Duration(1).sleep();
    chest_controller_->controlChest(0,pitch_angle,0);
    ros::Duration(1).sleep();
}

bool StairClimb::climbStairs(std::vector<float> horizontals, std::vector<float> verticals)
{
    // find current position and first stair position

    //    ihmc_msgs::FootstepDataRosMessage::Ptr current(new ihmc_msgs::FootstepDataRosMessage());
    //    walker_->getCurrentStep(RIGHT, *current);

    armSide side = RIGHT;
    for (size_t i= 0; i < horizontals.size(); ++i) {
        takeStep(side,horizontals[i],verticals[i]);

    }

}

bool StairClimb::takeFistStep(armSide side, float stepLength, float stepHeight)
{
    float raise_factor=1.5;
    float lower_factor=-0.5;
    float pitch_angle=20;
    float curl_radius=0.2;

    // fist leg movement
    walker_->raiseLeg(side,raise_factor*stepHeight,0.0);
    ros::Duration(1).sleep();
    walker_->nudgeFoot(side,stepLength);
    ros::Duration(1).sleep();
    walker_->raiseLeg(side,lower_factor*stepHeight,0.0);
    ros::Duration(1).sleep();
    walker_->load_eff(side,EE_LOADING::LOAD);
    ros::Duration(1).sleep();
    chest_controller_->controlChest(0,pitch_angle,0);
    ros::Duration(1).sleep();
}

int main(int argc, char** argv){

    ros::init(argc, argv, "stair_climb",ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    ValkyrieWalker walk(nh, 1.0,1.0,0);
    StairClimb stair(nh);

    ROS_INFO("hERE 1");
    stair_detector_2 detector(nh);
    std::vector<geometry_msgs::Pose> detections;

    while(!detector.getDetections(detections))
    {
        ros::spinOnce();
    }
    ROS_INFO("hERE 2");

    std::vector<float> step_length,step_height;

    geometry_msgs::Pose2D goal;
    goal.x=detections[0].position.x;
    goal.y=detections[0].position.y;
    goal.theta=tf::getYaw(detections[0].orientation);
    walk.walkToGoal(goal);

    ROS_INFO("hERE 3");

    ros::Duration(1).sleep();

    for (int i = 0; i < detections.size(); ++i) {
        std::cout<<"x: "<<detections[i].position.x<<" y: "<<detections[i].position.y<<" z: "<<detections[i].position.z<<"\n";
    }
    ROS_INFO("hERE 4");

    for (int i = 1; i < 10; ++i) {
        step_height.push_back(detections[i].position.z-detections[i-1].position.z);
        std::cout<<"step height at "<<i<<"is"<<(detections[i].position.z-detections[i-1].position.z)<<"\n";
    }

    ROS_INFO("hERE 5");

    step_length.assign(10,0.24);
    //    stair.climbStairs(step_length,step_height);
    ros::Duration(2).sleep();
    return 0;
}
