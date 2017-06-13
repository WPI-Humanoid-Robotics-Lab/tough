#include <footstep_planner/FootstepPlannerNode.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <val_footstep/ValkyrieWalker.h>
#include <visualization_msgs/MarkerArray.h>
#include <thread>

ValkyrieWalker *walk;
//ros::Publisher *footstep_marker_pub;

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
    ros::init(argc, argv, "footstep_node");
    ros::NodeHandle nh_;
//    ros::Publisher temp = nh_.advertise<visualization_msgs::MarkerArray>("/valkyrie/footstep_markers",10);
//    footstep_marker_pub = &temp;
    walk = new ValkyrieWalker(nh_, 0.8f, 0.8f, 0, 0.18);
    ros::Subscriber nav_goal_sub    = nh_.subscribe("/valkyrie/goal", 1, &nav_goal_cb);
    ros::Subscriber nav_goal_sub2    = nh_.subscribe("/move_base_simple/goal", 1, &nav_goal_cb);
//    ros::Subscriber footstep_points = nh_.subscribe("/footstep_planner/footsteps_array", 1,&republish_footsteps);
    footstep_planner::FootstepPlannerNode planner;
    ros::spin();
    delete walk;
    return 0;


}
