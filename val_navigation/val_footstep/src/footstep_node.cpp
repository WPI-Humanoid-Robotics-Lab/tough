#include <footstep_planner/FootstepPlannerNode.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <val_footstep/ValkyrieWalker.h>
#include <thread>

ValkyrieWalker *walk;

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "footstep_node");
    ros::NodeHandle nh_;
    walk = new ValkyrieWalker(nh_);
    ros::Subscriber nav_goal_sub    = nh_.subscribe("/valkyrie/goal", 1, &nav_goal_cb);
    footstep_planner::FootstepPlannerNode planner;
    ros::spin();
    delete walk;
    return 0;


}
