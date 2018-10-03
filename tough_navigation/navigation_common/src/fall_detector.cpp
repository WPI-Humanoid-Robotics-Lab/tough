#include <navigation_common/fall_detector.h>

FallDetector::FallDetector(ros::NodeHandle nh, std::string foot_frame, std::string root_frame, std::string world_frame):
    nh_(nh), foot_frame_(foot_frame), root_frame_(root_frame), world_frame_(world_frame)
{
    // robot is not fallen on init
    isrobot_fallen_ = false;

    rd_ = RobotDescription::getRobotDescription(nh);
    current_state_ = RobotStateInformer::getRobotStateInformer(nh);

    falltrack_thread_ = std::thread(&FallDetector::trackRobotFall, this);
    falltrack_thread_.detach();
}

FallDetector::FallDetector(ros::NodeHandle nh):
    nh_(nh)
{
    // robot is not fallen on init
    isrobot_fallen_ = false;

    rd_ = RobotDescription::getRobotDescription(nh);
    current_state_ = RobotStateInformer::getRobotStateInformer(nh);

    foot_frame_ = rd_->getLeftFootFrameName();
    root_frame_ = rd_->getPelvisFrame();
    world_frame_ = rd_->getWorldFrame();

    falltrack_thread_ = std::thread(&FallDetector::trackRobotFall, this);
    falltrack_thread_.detach();
}


FallDetector::~FallDetector(){
}

bool FallDetector::isRobotFallen()
{
    return isrobot_fallen_;
}

void  FallDetector::trackRobotFall(void)
{
    ros::Rate rate(10.0);
    tf::StampedTransform foot_transform, root_transform;

    while (ros::ok())
    {
        current_state_->getTransform(foot_frame_, foot_transform);
        current_state_->getTransform(root_frame_, root_transform);
        isrobot_fallen_ = (fabs(fabs(root_transform.getOrigin().getZ()) - fabs(foot_transform.getOrigin().getZ())) < ZTHRESHOLD);
        ros::spinOnce();
        rate.sleep();
    }
}
