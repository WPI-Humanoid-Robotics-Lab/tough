#include <footstep_planner/FootstepPlannerNode.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <val_footstep/ValkyrieWalker.h>
#include <visualization_msgs/MarkerArray.h>
#include <thread>

ValkyrieWalker *walk;
RobotStateInformer *current_state;
ihmc_msgs::FootstepDataListRosMessage list;
geometry_msgs::Pose pelvisPose;
//ros::Publisher *footstep_marker_pub;

void WalkToGoal(geometry_msgs::Pose2D goal) {
    current_state->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF, pelvisPose);
    list.footstep_data_list.clear();
    list.default_transfer_time = 1.0;
    list.default_swing_time = 1.0;
    list.execution_mode = 0;
    list.unique_id = ValkyrieWalker::id;

    bool success = walk->getFootstep(goal,list);
    std::string result = success ? "Succeded" : "Failed";
}

double distanceBetweenPoints(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2){
    return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2) + pow(point1.z - point2.z, 2));
}

void publish_footsteps_cb(const std_msgs::Empty msg){
    if(list.footstep_data_list.empty())
        return;

    geometry_msgs::Pose currPelvisPose;
    current_state->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF, currPelvisPose);

//    if(distanceBetweenPoints(currPelvisPose.position, pelvisPose.position) > 0.05)
//        return;

    walk->walkGivenSteps(list, false);
    list.footstep_data_list.clear();
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
    current_state = RobotStateInformer::getRobotStateInformer(nh_);
    ros::Subscriber nav_goal_sub    = nh_.subscribe("/valkyrie/goal", 1, &nav_goal_cb);
    ros::Subscriber nav_goal_sub2    = nh_.subscribe("/move_base_simple/goal", 1, &nav_goal_cb);
    ros::Subscriber publishFootsteps_sub  = nh_.subscribe("/approve_footsteps", 1, &publish_footsteps_cb);
    //    ros::Subscriber footstep_points = nh_.subscribe("/footstep_planner/footsteps_array", 1,&republish_footsteps);
    footstep_planner::FootstepPlannerNode planner;
    ros::spin();
    delete walk;
    return 0;


}
