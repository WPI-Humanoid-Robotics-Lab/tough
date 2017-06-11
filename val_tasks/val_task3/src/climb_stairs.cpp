#if(0)
#include <footstep_planner/FootstepPlannerNode.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <val_footstep/ValkyrieWalker.h>
#include <visualization_msgs/MarkerArray.h>
#include <thread>
#include <val_controllers/val_chest_navigation.h>
#include <val_controllers/val_pelvis_navigation.h>
#include <val_controllers/val_arm_navigation.h>

ValkyrieWalker *walk;
chestTrajectory *chest;
pelvisTrajectory *pelvis;
armTrajectory *arm;

void WalkToGoal(geometry_msgs::Pose2D goal) {
    bool success = walk->walkToGoal(goal);
    std::string result = success ? "Succeded" : "Failed";
}

void nav_goal_cb(const geometry_msgs::PoseStamped::Ptr &goal_3d)
{

    geometry_msgs::Pose2D goal_2d;
    goal_2d.x = goal_3d->pose.position.x;
    goal_2d.y = goal_3d->pose.position.y;
    goal_2d.theta = tf::getYaw(goal_3d->pose.orientation);
    std::thread t1 = std::thread(WalkToGoal, goal_2d);
    t1.detach();
}

//void republish_footsteps(const visualization_msgs::MarkerArray::Ptr inMsg){

//    visualization_msgs::MarkerArray msg = *inMsg;
//    for(size_t i = 0; i<msg.markers.size(); i++) {
//        msg.markers[i].ns = "valkyrie";
//    }
//    footstep_marker_pub->publish(msg);
//}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "climb_stairs_node");
    ros::NodeHandle nh_;
    //    ros::Publisher temp = nh_.advertise<visualization_msgs::MarkerArray>("/valkyrie/footstep_markers",10);
    //    footstep_marker_pub = &temp;
    walk = new ValkyrieWalker(nh_, 0.8f, 0.8f, 0, 0.3);
    chest = new chestTrajectory(nh_);
    pelvis = new pelvisTrajectory(nh_);
    arm = new armTrajectory(nh_);

    //    ros::Subscriber nav_goal_sub    = nh_.subscribe("/valkyrie/goal", 1, &nav_goal_cb);
    //    ros::Subscriber footstep_points = nh_.subscribe("/footstep_planner/footsteps_array", 1,&republish_footsteps);
    //footstep_planner::FootstepPlannerNode planner;

    chest->controlChest(0,20,0);
    ros::Duration(0.1).sleep();
    //arm->moveArmJoint(RIGHT, 1, )
    walk->walkNSteps(1, 0.3, 0.0, true, RIGHT);
    ros::Duration(1).sleep();
    pelvis->controlPelvisHeight(1.0);
    ros::Duration(1).sleep();
    walk->load_eff(armSide::RIGHT, EE_LOADING::LOAD);
    walk->walkNSteps(1, 0.3, 0.0, true, LEFT);

    ros::Duration(1).sleep();
    pelvis->controlPelvisHeight(1.0);
    ros::Duration(1).sleep();
    walk->load_eff(armSide::LEFT, EE_LOADING::LOAD);
    walk->walkNSteps(1, 0.25, 0.0, true, RIGHT);
    ros::Duration(1).sleep();
    pelvis->controlPelvisHeight(1.0);
    ros::Duration(1).sleep();
    walk->load_eff(armSide::RIGHT, EE_LOADING::LOAD);
    walk->walkNSteps(1, 0.25, 0.0, true, LEFT);


    ros::Duration(1).sleep();
    pelvis->controlPelvisHeight(1.0);
    ros::Duration(1).sleep();
    walk->load_eff(armSide::LEFT, EE_LOADING::LOAD);
    walk->walkNSteps(1, 0.25, 0.0, true, RIGHT);
    ros::Duration(1).sleep();
    pelvis->controlPelvisHeight(1.0);
    ros::Duration(1).sleep();
    walk->load_eff(armSide::RIGHT, EE_LOADING::LOAD);
    walk->walkNSteps(1, 0.25, 0.0, true, LEFT);

    ros::Duration(1).sleep();
    pelvis->controlPelvisHeight(1.0);
    ros::Duration(1).sleep();
    walk->load_eff(armSide::LEFT, EE_LOADING::LOAD);
    walk->walkNSteps(1, 0.25, 0.0, true, RIGHT);
    ros::Duration(1).sleep();
    pelvis->controlPelvisHeight(1.0);
    ros::Duration(1).sleep();
    walk->load_eff(armSide::RIGHT, EE_LOADING::LOAD);
    walk->walkNSteps(1, 0.25, 0.0, true, LEFT);

    ros::Duration(1).sleep();
    pelvis->controlPelvisHeight(1.0);
    ros::Duration(1).sleep();
    walk->load_eff(armSide::LEFT, EE_LOADING::LOAD);
    walk->walkNSteps(1, 0.25, 0.0, true, RIGHT);
    ros::Duration(1).sleep();
    pelvis->controlPelvisHeight(1.0);
    ros::Duration(1).sleep();
    walk->load_eff(armSide::RIGHT, EE_LOADING::LOAD);
    walk->walkNSteps(1, 0.25, 0.0, true, LEFT);

    ros::Duration(1).sleep();
    pelvis->controlPelvisHeight(1.0);
    ros::Duration(1).sleep();
    walk->load_eff(armSide::LEFT, EE_LOADING::LOAD);
    walk->walkNSteps(1, 0.25, 0.0, true, RIGHT);
    ros::Duration(1).sleep();
    pelvis->controlPelvisHeight(1.0);
    ros::Duration(1).sleep();
    walk->load_eff(armSide::RIGHT, EE_LOADING::LOAD);
    walk->walkNSteps(1, 0.25, 0.0, true, LEFT);

    ros::Duration(1).sleep();
    pelvis->controlPelvisHeight(1.0);
    ros::Duration(1).sleep();
    walk->load_eff(armSide::LEFT, EE_LOADING::LOAD);
    walk->walkNSteps(1, 0.25, 0.0, true, RIGHT);
    ros::Duration(1).sleep();
    pelvis->controlPelvisHeight(1.0);
    ros::Duration(1).sleep();
    walk->load_eff(armSide::RIGHT, EE_LOADING::LOAD);
    walk->walkNSteps(1, 0.25, 0.0, true, LEFT);

    ros::Duration(1).sleep();
    pelvis->controlPelvisHeight(1.0);
    ros::Duration(1).sleep();
    walk->load_eff(armSide::LEFT, EE_LOADING::LOAD);
    walk->walkNSteps(1, 0.25, 0.0, true, RIGHT);
    ros::Duration(1).sleep();
    pelvis->controlPelvisHeight(1.0);
    ros::Duration(1).sleep();
    walk->load_eff(armSide::RIGHT, EE_LOADING::LOAD);
    walk->walkNSteps(1, 0.25, 0.0, true, LEFT);

    ros::Duration(1).sleep();
    pelvis->controlPelvisHeight(1.0);
    ros::Duration(1).sleep();
    walk->load_eff(armSide::LEFT, EE_LOADING::LOAD);
    walk->walkNSteps(1, 0.25, 0.0, true, RIGHT);
    ros::Duration(1).sleep();
    pelvis->controlPelvisHeight(1.0);
    ros::Duration(1).sleep();
    walk->load_eff(armSide::RIGHT, EE_LOADING::LOAD);
    walk->walkNSteps(1, 0.25, 0.0, true, LEFT);


    ros::spin();
    delete walk;
    return 0;


}

#endif

#include <val_task3/climb_stairs.h>

climbStairs::climbStairs(ros::NodeHandle nh): nh_(nh)
{
    walker_ = new ValkyrieWalker(nh_, 0.8f, 0.8f, 0, 0.3);
    chest_  = new chestTrajectory(nh_);
    pelvis_ = new pelvisTrajectory(nh_);
    arm_    = new armTrajectory(nh_);
}

climbStairs::~climbStairs()
{
    if (walker_ != nullptr) delete walker_;
    if (chest_ != nullptr) delete chest_;
    if (pelvis_ != nullptr) delete pelvis_;
    if (arm_ != nullptr) delete arm_;
}

void climbStairs::climb_stairs()
{
    // set the chest orientation
    chest_->controlChest(0,20,0);
    ros::Duration(1).sleep();

    // set the arms orientation
    arm_->moveArmJoint(RIGHT, 1, 0.7);
    ros::Duration(1).sleep();
    arm_->moveArmJoint(LEFT, 1, -0.7);

    // first step
    walker_->walkNSteps(1, 0.3, 0.0, true, RIGHT);
    ros::Duration(1).sleep();
    pelvis_->controlPelvisHeight(1.0);
    ros::Duration(1).sleep();
    walker_->load_eff(armSide::RIGHT, EE_LOADING::LOAD);
    walker_->walkNSteps(1, 0.3, 0.0, true, LEFT);

    // next 8 steps
    for (int i=0; i<8; i++)
    {
        ros::Duration(1).sleep();
        pelvis_->controlPelvisHeight(1.0);
        ros::Duration(1).sleep();
        walker_->load_eff(armSide::LEFT, EE_LOADING::LOAD);
        walker_->walkNSteps(1, 0.25, 0.0, true, RIGHT);
        ros::Duration(1).sleep();
        pelvis_->controlPelvisHeight(1.0);
        ros::Duration(1).sleep();
        walker_->load_eff(armSide::RIGHT, EE_LOADING::LOAD);
        walker_->walkNSteps(1, 0.25, 0.0, true, LEFT);
    }
}
