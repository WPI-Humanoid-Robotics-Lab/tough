//
// Created by will on 6/11/17.
//

#include <val_controllers/val_head_navigation.h>
#include <val_controllers/robot_state.h>
#include <eigen_conversions/eigen_msg.h>

// minimum time it takes to execute a head trajectory, in seconds
#define MIN_TRAJ_TIME 2.f

std::unique_ptr<HeadTrajectory> headTraj;
RobotStateInformer *current_state;
//ros::Publisher pub;

void clickedPointCB(const geometry_msgs::PointStamped &point_in) {
    geometry_msgs::Point point_pelvisframe;
    current_state->transformPoint(point_in.point, point_pelvisframe, point_in.header.frame_id, VAL_COMMON_NAMES::PELVIS_TF);
    geometry_msgs::Pose head_pelvisframe;
    current_state->getCurrentPose(VAL_COMMON_NAMES::ROBOT_HEAD_FRAME_TF, head_pelvisframe, VAL_COMMON_NAMES::PELVIS_TF);

    Eigen::Vector3d given_pt, head_pt;
    tf::pointMsgToEigen(point_pelvisframe, given_pt);
    tf::pointMsgToEigen(head_pelvisframe.position, head_pt);

    Eigen::Quaterniond rotation_quat = Eigen::Quaterniond::FromTwoVectors(
            Eigen::Vector3d::UnitX(),
            given_pt - head_pt
    );

    Eigen::Vector3f rpy = rotation_quat.toRotationMatrix().eulerAngles(0, 1, 2).cast<float>();

    // From this point down, all computations use degrees (including headTraj->moveHead)
    rpy *= 180.f / M_PI;

    ROS_INFO_STREAM("val_gaze_controller requested angle was " << std::fixed << rpy.transpose().format({1}));

    if (std::abs(rpy[0]) > 20) {
        ROS_WARN_STREAM("gaze_controller ignoring point with roll = " << std::fixed << std::setprecision(0) << rpy[0]);
        return;
    }

    // test yaw first on purpose because pitch limits are complicated
    if (std::abs(rpy[2]) > 50) {
        ROS_WARN_STREAM("gaze_controller ignoring point with yaw = " << std::fixed << std::setprecision(0) << rpy[2]);
        return;
    }

    // If yaw is above 50 deg, the pitch is limited to 40 deg max; otherwise it's limited to 55 - 0.5*yaw
    // (formula deterimed experimentally) (note negative yaw means to look upwards)
    float pitch_upper_limit = (std::abs(rpy[2]) > 50) ? 40 : (50 - 0.5f * std::abs(rpy[2]));
    if (rpy[1] > pitch_upper_limit || rpy[1] < -45) {
        ROS_WARN_STREAM("gaze_controller ignoring point with pitch = " << std::fixed << std::setprecision(0) << rpy[1]
                                                                       << " (upper limit " << pitch_upper_limit << ")");
        return;
    }

    std::vector<std::vector<float>> trajectory_points;

    // Extreme angles are more likely to lead to collision. If any angle is above 20, 40, or 30 for r,p,y respectively,
    // play it safe and first reset to 0, 0, 0 before moving to the desired angle.
    if (std::abs(rpy[0]) > 20 || std::abs(rpy[1]) > 40 || std::abs(rpy[2]) > 30) {
        trajectory_points.push_back({0, 0, 0});
    }

    trajectory_points.push_back({rpy[0], rpy[1], rpy[2]});

    float traj_time = std::max(MIN_TRAJ_TIME, std::ceil(rpy.maxCoeff() / 20.f));

    headTraj->moveHead(trajectory_points, traj_time);
    if (trajectory_points.size() > 1) {
        ROS_INFO_STREAM("val_gaze_controller turning head to 0, 0, 0 before desired goal to prevent collision");
        ros::Duration(traj_time).sleep();
    }

    ROS_INFO_STREAM("val_gaze_controller turning head to " << std::fixed << rpy.transpose().format({1}));
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