#include "src_task3/leak_detector.h"

// Update period: how often a new trajectory is published
#define WAYPOINT_PERIOD 0.2f
// Sweep period: time for an entire up-and-down arm motion (think of it like the period of a sine wave)
#define ARM_SWEEP_PERIOD 2.f

leakDetector::leakDetector(ros::NodeHandle nh, RobotSide side, bool thumbwards):
    nh_(nh), task3Utils_(nh), side_(side), thumbwards_(thumbwards), leak_found_(false)
{
    leak_sb_ = nh_.subscribe("/task3/checkpoint5/leak", 10, &leakDetector::leakMsgCB, this);
    leak_loc_pub_ = nh.advertise<geometry_msgs::PoseStamped>("leak_location", 10, true);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10, true);

    current_state_ = RobotStateInformer::getRobotStateInformer(nh_);
    rd_ = RobotDescription::getRobotDescription(nh_);
    pelvis_controller_  = new PelvisControlInterface(nh_);
    arm_controller_     = new ArmControlInterface(nh_);
}

leakDetector::~leakDetector()
{
    //shutdown subscribers
    leak_sb_.shutdown();
    if (pelvis_controller_ != nullptr)         delete pelvis_controller_;
    if (arm_controller_ != nullptr)            delete arm_controller_;
}

void leakDetector::leakMsgCB(const srcsim::Leak &leakmsg)
{
    if (leakmsg.value > 0.101) {
        geometry_msgs::PoseStamped leak_loc;
        leak_loc.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
        leak_loc.header.stamp = ros::Time::now();

        std::string palm_frame = side_ == RobotSide::LEFT ? rd_->getLeftPalmFrame() : rd_->getRightPalmFrame();
        current_state_->getCurrentPose(palm_frame, leak_loc.pose, leak_loc.header.frame_id);

        leak_loc_pub_.publish(leak_loc);

        std::stringstream out;
        out << "LEAK DETECTED AT POSITION " << leak_loc.pose.position.x << ", " << leak_loc.pose.position.y
            << ", " << leak_loc.pose.position.z << ", ORIENTATION (xyzw) " << leak_loc.pose.orientation.x << ", "
            << leak_loc.pose.orientation.y << ", " << leak_loc.pose.orientation.z << ", "
            << leak_loc.pose.orientation.w;

        task3Utils_.task3LogPub(out.str());

        leak_found_ = true;
    }
}

void leakDetector::findLeak() {
    pelvis_controller_->controlPelvisHeight(1.0);
    ros::Duration(2).sleep();

    // Move arm to start configuration
    // Joint order: leftShoulderPitch, leftShoulderRoll, leftShoulderYaw, leftElbowPitch, leftForearmYaw,
    // leftWristRoll, leftWristPitch
    std::vector<float> configuration;
    if (side_ == RobotSide::LEFT && thumbwards_) {
        // Left arm, detector thumbwards
        configuration = {0.f, -1.f, 0.f, -0.68f, 1.45, -0.5f, 0.f};
    } else if (side_ == RobotSide::LEFT) {
        // Left arm, detector anti-thumbwards
        configuration = {0.f, -1.f, 0.f, -0.68f, -1.45f, 0.6f, 0.f};
    } else if (thumbwards_) {
        // Right arm, detector thumbwards
        configuration = {0.f, 1.f, 0.f, 0.6f, 1.44f, 0.6f, 0.f};
    } else {
        // Right arm, detector anti-thumbwards
        configuration = {0.f, 1.f, 0.f, 0.6f, -2.f, 0.6f, 0.f};
    }

    task3Utils_.task3LogPub("findLeak moving to start configuration");

    const std::size_t moving_joint = 1; // I think this is always the same?
    float min_moving_joint_angle = configuration[moving_joint];
    float max_moving_joint_angle = (side_ == RobotSide::LEFT) ? 0.88f : -0.80f;

    float center = (min_moving_joint_angle + max_moving_joint_angle) / 2;
    float amplitude = max_moving_joint_angle - center;

    // Start from center, there's a seed point about there that we should move to first
    configuration[moving_joint] = center;
    arm_controller_->moveArmJoints(side_, {configuration}, 2.f);

    std::vector<std::vector<float>> waypoints;
    for (float time = 0; time < ARM_SWEEP_PERIOD; time += WAYPOINT_PERIOD) {
        // Triangle wave from https://en.wikipedia.org/wiki/Triangle_wave#Definitions, modified to start at 0
        float k = 2 * std::abs(2 * (time / ARM_SWEEP_PERIOD - 0.25f - std::floor(time / ARM_SWEEP_PERIOD + 0.25f))) - 1;

        configuration[moving_joint] = center + k * amplitude;

        waypoints.push_back(configuration);
    }

    ROS_INFO_STREAM("Generated trajectory with " << waypoints.size() << " waypoints");

    task3Utils_.task3LogPub("findLeak beginning arm sweep");
    ros::Rate rate(ros::Duration(ARM_SWEEP_PERIOD));
    while (ros::ok() && !leak_found_) {
        arm_controller_->moveArmJoints(side_, waypoints, WAYPOINT_PERIOD);

        ros::spinOnce();
        rate.sleep();
    }
}

void leakDetector::visulatiseSearchPoints(std::vector<geometry_msgs::Pose> &poses, geometry_msgs::Point horz_left_top, geometry_msgs::Point horz_right_bottom)
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
    for (i=0; i<poses.size();i++)
    {
        marker.id = i;
        marker.pose.position = poses[i].position;
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

