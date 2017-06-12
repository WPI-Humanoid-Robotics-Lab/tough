//
// Created by will on 6/11/17.
//

#include <val_controllers/val_head_navigation.h>
#include <val_controllers/robot_state.h>
#include <eigen_conversions/eigen_msg.h>

std::unique_ptr<HeadTrajectory> headTraj;
RobotStateInformer *current_state;

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
    geometry_msgs::Quaternion quat_ros;
    tf::quaternionEigenToMsg(rotation_quat, quat_ros);

    headTraj->moveHead(quat_ros);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "val_gaze_controller");
    ros::NodeHandle nh;

    headTraj.reset(new HeadTrajectory(nh));
    current_state = RobotStateInformer::getRobotStateInformer(nh);

    ros::Subscriber clicked_point_subs = nh.subscribe("clicked_point", 1, &clickedPointCB);

    ros::spin();
}