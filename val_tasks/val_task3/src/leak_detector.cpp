#include "val_task3/leak_detector.h"


leakDetector::leakDetector(ros::NodeHandle nh):
    nh_(nh)
{
    leak_sb_ = nh_.subscribe("/task3/checkpoint5/leak", 10, &leakDetector::leakMsgCB, this);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>( "leak_search_points", 10, true);
    left_arm_planner_ = new cartesianPlanner("leftMiddleFingerGroup", "/world"); //leftPalm
    wholebody_controller_= new wholebodyManipulation(nh_);
}

leakDetector::~leakDetector()
{
    //shutdown subscribers
    leak_sb_.shutdown();

    if(left_arm_planner_ != nullptr)      delete left_arm_planner_;
    if(wholebody_controller_ != nullptr)  delete wholebody_controller_;
}

void leakDetector::leakMsgCB(const srcsim::Leak &leakmsg)
{
    leak_value_ = leakmsg.value;
}


void leakDetector::generateSearchWayPoints(geometry_msgs::Point horz_left_top, geometry_msgs::Point horz_right_bottom, float ver_low_limit, float ver_high_limit, std::vector<geometry_msgs::Point>& way_points)
{
    // generate way points with the dimension of the field of view of the tool

    int points_in_vertical_line = (ver_high_limit - ver_low_limit)/VERTICAL_WIDTH + 1;
    int points_in_horizontal_line = (horz_right_bottom.y - horz_left_top.y)/HORIZONTAL_WIDTH + 1;

    geometry_msgs::Point point;
    point.x = horz_left_top.x;
    point.y = horz_left_top.y;
    point.z = ver_high_limit;

    for (int i=0; i<points_in_horizontal_line; i++)
    {
        for (int j=0; j<points_in_vertical_line; j++)
        {
            point.z = (i%2 == 0) ? point.z-VERTICAL_WIDTH : point.z+VERTICAL_WIDTH;
            way_points.push_back(point);
        }
        point.y += HORIZONTAL_WIDTH;
        way_points.push_back(point);
    }

    ROS_INFO("%d search points generated", way_points.size());
    // visulaize points
    visulatiseSearchPoints(way_points, horz_left_top, horz_right_bottom);
}

void leakDetector::findLeak (std::vector<geometry_msgs::Point>& way_points, geometry_msgs::Point& leak_point)
{
    // plan the trajectory
    moveit_msgs::RobotTrajectory traj;
    left_arm_planner_->getTrajFromCartPoints(way_points, traj, false);
    ROS_INFO("trajectory generated");

    // execute the trajectory
    wholebody_controller_->compileMsg(armSide::LEFT, traj.joint_trajectory);
    ROS_INFO("trajectory sent to controllers");
}

// helper functions
double leakDetector::getLeakValue() const
{
    return leak_value_;
}

void leakDetector::setLeakValue(double leak_value)
{
    leak_value_ = leak_value;
}

void leakDetector::visulatiseSearchPoints(std::vector<geometry_msgs::Point> &points, geometry_msgs::Point horz_left_top, geometry_msgs::Point horz_right_bottom)
{
    visualization_msgs::MarkerArray marker_array = visualization_msgs::MarkerArray();

    visualization_msgs::Marker marker;
    marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    marker.header.stamp = ros::Time();
    marker.ns = "leak";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.lifetime = ros::Duration(0);

    int i=0;
    for (i=0; i<points.size();i++)
    {
        marker.id = i;
        marker.pose.position = points[i];
        marker_array.markers.push_back(marker);
    }

    // add end points
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    marker.id = i++;
    marker.pose.position = horz_left_top;
    marker_array.markers.push_back(marker);

    marker.id = i++;
    marker.pose.position = horz_right_bottom;
    marker_array.markers.push_back(marker);

    marker_pub_.publish(marker_array);
    ros::Duration(0.2).sleep();
}

