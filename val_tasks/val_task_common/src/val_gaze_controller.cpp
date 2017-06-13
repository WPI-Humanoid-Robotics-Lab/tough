//
// Created by will on 6/11/17.
//

#include <val_controllers/val_head_navigation.h>
#include <val_controllers/robot_state.h>
#include <eigen_conversions/eigen_msg.h>

// minimum time it takes to execute a head trajectory, in seconds
#define TRAJ_STEP_TIME 0.5f
// maximum difference in degrees between waypoints
#define MAX_ANGLE 5


std::unique_ptr<HeadTrajectory> headTraj;
RobotStateInformer *current_state;
//ros::Publisher pub;

void clickedPointCB(const geometry_msgs::PointStamped &point_in) {
    geometry_msgs::Point point_pelvisframe;
    current_state->transformPoint(point_in.point, point_pelvisframe, point_in.header.frame_id, VAL_COMMON_NAMES::PELVIS_TF);
    geometry_msgs::Pose head_pelvisframe;
    current_state->getCurrentPose(VAL_COMMON_NAMES::ROBOT_HEAD_FRAME_TF, head_pelvisframe, VAL_COMMON_NAMES::PELVIS_TF);

    double x = point_pelvisframe.x - head_pelvisframe.position.x;
    double y = point_pelvisframe.y - head_pelvisframe.position.y;
    double z = point_pelvisframe.z - head_pelvisframe.position.z;
    double d = std::sqrt(x*x + y*y);

    double roll = 0;
    double yaw = std::atan2(y, x);
    double pitch = -std::atan2(z, d);

    // From this point down, all computations use degrees (including headTraj->moveHead)
    yaw *= 180.f / M_PI;
    pitch *= 180.f / M_PI;

    ROS_INFO_STREAM(std::fixed << std::setprecision(1)
                               << "angles before limit are 0, " << pitch << ", " << yaw);


    // Apply limits: -90 <= yaw <= 90
    yaw = std::max(-90., std::min(yaw, 90.));

    // If yaw is above 50 deg, the pitch is limited to 40 deg max; otherwise it's limited to 55 - 0.5*yaw
    // (formula deterimed experimentally) (note negative yaw means to look upwards)
    double pitch_upper_limit = (std::abs(yaw) > 50) ? 40 : (50 - 0.5 * std::abs(yaw));

    // Apply limits: -50 < pitch < pitch_upper_limit
    pitch = std::max(-50., std::min(pitch, pitch_upper_limit));


    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(head_pelvisframe.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double prev_roll, prev_pitch, prev_yaw;
    tf::Matrix3x3(quat).getRPY(prev_roll, prev_pitch, prev_yaw);

    prev_roll *= 180.f / M_PI;
    prev_pitch *= 180.f / M_PI;
    prev_yaw *= 180.f / M_PI;

    prev_roll = 180 + prev_roll; // roll is defined upside-down in head frame
    if (prev_roll > 180) prev_roll -= 360;

    double roll_err = roll - prev_roll;
    double pitch_err = pitch - prev_pitch;
    double yaw_err = yaw - prev_yaw;
    double largest_err = std::max(std::abs(roll_err), std::max(std::abs(pitch_err), std::abs(yaw_err)));
    std::size_t n_steps = static_cast<std::size_t>(largest_err / MAX_ANGLE);

    // One more waypoint than steps because waypoints starts with the current state
    std::vector<std::vector<float>> waypoints(n_steps + 1);
    for (std::size_t i = 0; i < n_steps + 1; i++) {
        double roll_i = prev_roll + roll_err * i / n_steps;
        double pitch_i = prev_pitch + pitch_err * i / n_steps;
        double yaw_i = prev_yaw + yaw_err * i / n_steps;

        ROS_INFO_STREAM(std::fixed << std::setprecision(1)
                                   << "waypoint " << roll_i << ", " << pitch_i << ", " << yaw_i);

        waypoints[i] = {static_cast<float>(roll_i), static_cast<float>(pitch_i), static_cast<float>(yaw_i)};
    }

    headTraj->moveHead(waypoints, TRAJ_STEP_TIME);

    ros::Duration(TRAJ_STEP_TIME * (n_steps + 1));
    ROS_INFO_STREAM(std::fixed << std::setprecision(1)
                               << "val_gaze_controller turned head to " << roll << ", " << pitch << ", " << yaw);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "val_gaze_controller");
    ros::NodeHandle nh;

    headTraj.reset(new HeadTrajectory(nh));
    current_state = RobotStateInformer::getRobotStateInformer(nh);

    ros::Subscriber clicked_point_subs = nh.subscribe("clicked_point", 1, &clickedPointCB);
//    pub = nh.advertise<geometry_msgs::PoseStamped>("gaze_controller_pose", 1);

    ros::spin();
}