#include "val_task3/val_stairclimb.h"
#include "val_common/val_common_defines.h"
#include "val_footstep/RobotWalker.h"
#include "val_controllers/val_arm_navigation.h"


StairClimb::StairClimb(ros::NodeHandle n)
{
    walker_ = new RobotWalker(n, 0.7, 0.7, 0, 0.18);
    robot_state_=RobotStateInformer::getRobotStateInformer(n);
    chest_controller_ = new chestTrajectory(n);

}

bool StairClimb::walkToSetPosition(geometry_msgs::Pose2D goal)
{
    walker_->walkToGoal(goal);

}


int main(int argc, char** argv){

    ros::init(argc, argv, "stair_climb",ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    RobotWalker walk(nh, 1.0,1.0,0);
    StairClimb stair(nh);
    chestTrajectory chest_controller(nh);
    armTrajectory arm_controller(nh);

    stair_detector_2 detector(nh);
    std::vector<geometry_msgs::Pose> detections;

    while(!detector.getDetections(detections)) {
        ROS_INFO_STREAM_THROTTLE(5, "Stairs detections: " << detections.size());
        ros::spinOnce();
    }


    geometry_msgs::Pose2D goal;
    goal.x=detections[0].position.x;
    goal.y=detections[0].position.y;
    goal.theta=tf::getYaw(detections[0].orientation);
    stair.walkToSetPosition(goal);


    std::vector<float> step_length ={0.28,0.2031,0.2031,0.2031,0.2031,0.2031,0.2031,0.2031,0.2031,0.2031};
    std::vector<float> step_height ={0.12,0.2389,0.2389,0.2389,0.2389,0.2389,0.2389,0.2389,0.2389,0.2389};
    float start_length =0.28;
    float start_height =0.12;

    float length_offset =0.2031;
    float height_offset =0.25;

    std::vector<float> mod_step_length,mod_step_height;
    int num_steps = 10;
    for (size_t i = 0; i < num_steps; ++i) {
        mod_step_length.push_back(start_length+i*length_offset);
        mod_step_length.push_back(start_length+i*length_offset);
        mod_step_height.push_back(start_height+i*height_offset);
        mod_step_height.push_back(start_height+i*height_offset);
    }

    ROS_INFO("Setting arm positions");
    std::vector< std::vector<float> > armData;
    armData.clear();
    armData.push_back({-0.67,1.24,0.49,1.49,0.97,0.0,0.});
    arm_controller.moveArmJoints(RIGHT, armData, 2.0f);
    ros::Duration(0.2).sleep();
    armData.clear();
    armData.push_back({-0.67,-1.24,0.49,-1.49,0.97,0.0,0.});
    arm_controller.moveArmJoints(LEFT, armData, 2.0f);
    ros::Duration(1).sleep();

    ROS_INFO("Orienting chest");
    chest_controller.controlChest(0,20.0,0);
    ros::Duration(1).sleep();
    ROS_INFO("Start Climbing");
    walk.climbStair(mod_step_length,mod_step_height,armSide::RIGHT);
    return 0;
}
