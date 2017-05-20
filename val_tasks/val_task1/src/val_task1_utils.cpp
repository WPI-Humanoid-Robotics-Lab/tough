#include <val_task1/val_task1_utils.h>

task1Utils::task1Utils(ros::NodeHandle nh):
    nh_(nh)
{
    // subscriber for the satellite message
    satellite_sub_ = nh_.subscribe("/task1/checkpoint2/satellite", 10, &task1Utils::satelliteMsgCB, this);

    // marker publisher
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>( "handle_path", 10, true);
}

task1Utils::~task1Utils()
{

}


void task1Utils::satelliteMsgCB(const srcsim::Satellite& msg)
{
    // update the message
    msg_ = msg;
}

// satellite dish helper tasks
bool task1Utils::isPitchCorrectNow(void)
{
    return msg_.pitch_correct_now;
}

bool task1Utils::isYawCorrectNow(void)
{
    return msg_.yaw_correct_now;
}

bool task1Utils::isPitchCompleted(void)
{
    return msg_.pitch_completed;
}

bool task1Utils::isYawCompleted(void)
{
    return msg_.yaw_completed;
}

double task1Utils::getPitchDiff (void)
{
    return (msg_.target_pitch - msg_.current_pitch);
}

double task1Utils::getYawDiff (void)
{
    return (msg_.target_yaw - msg_.current_yaw);
}

valueDirection task1Utils::getPitchValueDirection(double current_value, controlSelection control)
{
    valueDirection ret = valueDirection::NOT_INITIALISED;

    static double prev_value = 0;
    static bool once = true;

    // only on init
    if (once)
    {
        // initialise preveious valuie and return
        prev_value = current_value;

        // reset the once flag
        once = false;

        ROS_INFO("initialised");
    }
    else if (fabs(fabs(current_value) - fabs(prev_value)) > MINIMUM_MOVMENT_IN_RAD) // moved atleast by 5 degrees
    {
        ROS_INFO("moved by 5deg");

        if ((control == controlSelection::PITCH ? msg_.target_pitch : msg_.target_yaw) < prev_value)
        {
            ret = valueDirection::VALUE_INCREASING;
            ROS_INFO("increasing");
        }
        else
        {
            ret = valueDirection::VALUE_DECREASING;
            ROS_INFO("decreasing");
        }

        // set the once flag for next time
        once = true;
    }


    return ret;
}

void task1Utils::getCircle3D(geometry_msgs::Point center, geometry_msgs::Point start, geometry_msgs::Pose pose, const std::vector<float> planeCoeffs, std::vector<geometry_msgs::Pose> &points, float radius, int steps)
{
    if (planeCoeffs.size() != 4){
        ROS_INFO("Please check the plane coefficiants");
    }

    // plane co-effecients
    float a = planeCoeffs.at(0);
    float b = planeCoeffs.at(1);
    float c = planeCoeffs.at(2);
    float d = planeCoeffs.at(3);

    // starting point
    geometry_msgs::Pose circle_point_pose;
    circle_point_pose.position.x = start.x;
    circle_point_pose.position.y = start.y;
    circle_point_pose.position.z = start.z;
    circle_point_pose.orientation = pose.orientation;
    points.push_back(circle_point_pose);

    float dist = fabs(a*start.x  + b*start.y + c*start.z  + d )/sqrt(pow(a,2) + pow(b,2) + pow(c,2));

    geometry_msgs::Pose start_pose;
    start_pose.position.x = start.x;
    start_pose.position.y = start.z;
    start_pose.position.y = start.z;

    for (int i=0;i<steps; i++)
    {
        // angle to the first point
        float alpha = atan2((start_pose.position.y - center.y),(start_pose.position.x - center.x));
        circle_point_pose.position.x = center.x + radius*cos(alpha - (float)(2*M_PI/steps));
        circle_point_pose.position.y = center.y + radius*sin(alpha - (float)(2*M_PI/steps));

        //point.position.z = -(a*point.position.x  + b* point.position.y + d)/c;
        circle_point_pose.position.z = -(a*circle_point_pose.position.x  + b* circle_point_pose.position.y + + d)/c + dist; //(d - dist+.05) )/c;

        // orientation
        circle_point_pose.orientation = pose.orientation;
        points.push_back(circle_point_pose);

        start_pose = circle_point_pose;
    }
}

void task1Utils::visulatise6DPoints(std::vector<geometry_msgs::Pose> &points)
{
    // visulation of the circle
    visualization_msgs::MarkerArray marker_array = visualization_msgs::MarkerArray();

    visualization_msgs::Marker marker;
    marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    marker.header.stamp = ros::Time();
    marker.ns = "circle";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = points[0];
    marker.scale.x = 0.1;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 0.6;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(0);
    marker_array.markers.push_back(marker);

    for (int i = 1; i < points.size(); i++) {
        visualization_msgs::Marker marker_tangent;
        marker_tangent.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
        marker_tangent.header.stamp = ros::Time();
        marker_tangent.ns = "circle";
        marker_tangent.id = i;
        marker_tangent.type = visualization_msgs::Marker::ARROW;
        marker_tangent.action = visualization_msgs::Marker::ADD;
        marker_tangent.pose = points[i];

        //        ROS_INFO_STREAM(i<<"pose"<<points[i].position);
        marker_tangent.scale.x = 0.05;
        marker_tangent.scale.y = 0.006;
        marker_tangent.scale.z = 0.006;
        marker_tangent.color.a = 0.6;
        marker_tangent.color.r = ((i==1) ? 1.0: 0.0);
        marker_tangent.color.g = ((i==1) ? 0.0: 1.0);
        marker_tangent.color.b = 0.0;
        marker_tangent.lifetime = ros::Duration(0);
        marker_array.markers.push_back(marker_tangent);

        visualization_msgs::Marker marker_point;
        marker_point.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
        marker_point.header.stamp = ros::Time();
        marker_point.ns = "circle";
        marker_point.id = i+points.size();
        marker_point.type = visualization_msgs::Marker::CUBE;
        marker_point.action = visualization_msgs::Marker::ADD;
        marker_point.pose = points[i];

        //        ROS_INFO_STREAM(i<<"pose"<<points[i].position);
        marker_point.scale.x = 0.005;
        marker_point.scale.y = 0.005;
        marker_point.scale.z = 0.005;
        marker_point.color.a = 0.6;
        marker_point.color.r = 0.0;
        marker_point.color.g = .0;
        marker_point.color.b = 1.0;
        marker_point.lifetime = ros::Duration(0);
        marker_array.markers.push_back(marker_point);

    }

    marker_pub_.publish(marker_array);
    ros::Duration(0.2).sleep();
}

