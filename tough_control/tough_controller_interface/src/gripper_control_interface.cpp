#include <tough_controller_interface/gripper_control_interface.h>
#include <tf/transform_listener.h>
#include <map>

GripperControlInterface::GripperControlInterface(ros::NodeHandle nh) : ToughControllerInterface(nh){

    gripperPublisher_ =
            nh_.advertise<ihmc_msgs::HandDesiredConfigurationRosMessage>(control_topic_prefix_ +"/hand_desired_configuration",1,true);

}

GripperControlInterface::~GripperControlInterface(){

}

void GripperControlInterface::controlGripper(const RobotSide side, int configuration){
    ihmc_msgs::HandDesiredConfigurationRosMessage msg;
    generateGripperMessage(side, configuration, msg);
    gripperPublisher_.publish(msg);

}

void GripperControlInterface::generateGripperMessage(const RobotSide side, const int configuration, ihmc_msgs::HandDesiredConfigurationRosMessage &msg){
    msg.hand_desired_configuration = configuration;
    msg.unique_id = GripperControlInterface::id_++;
    msg.robot_side = side;
}

void GripperControlInterface::closeGripper(const RobotSide side)
{
    controlGripper(side, ihmc_msgs::HandDesiredConfigurationRosMessage::CLOSE);
}

void GripperControlInterface::openGripper(const RobotSide side)
{
    controlGripper(side, ihmc_msgs::HandDesiredConfigurationRosMessage::OPEN);
}

bool GripperControlInterface::getTaskSpaceState(geometry_msgs::Pose &pose, RobotSide side, std::string fixedFrame)
{
    return state_informer_->getCurrentPose(side == RobotSide::LEFT ? rd_->getLeftEEFrame(): rd_->getRightEEFrame(),
                                           pose,
                                           fixedFrame);
}

bool GripperControlInterface::getJointSpaceState(std::vector<double> &joints, RobotSide side)
{
    joints.clear();
    double jointPosition = state_informer_->getJointPosition(side == RobotSide::LEFT ? rd_->getLeftEEFrame() : rd_->getRightEEFrame());
    joints.push_back(jointPosition);
    return true;
}



void GripperControlInterface::setMode(const RobotSide side, const GRIPPER_MODES mode)
{
    controlGripper(side, mode);
}


void GripperControlInterface::resetGripper(const RobotSide side)
{
    controlGripper(side, GRIPPER_MODES::RESET);
}

void GripperControlInterface::openThumb(const RobotSide side)
{
    controlGripper(side, ihmc_msgs::HandDesiredConfigurationRosMessage::OPEN_THUMB);
}

void GripperControlInterface::closeThumb(const RobotSide side)
{
    controlGripper(side, ihmc_msgs::HandDesiredConfigurationRosMessage::CLOSE_THUMB);
}

void GripperControlInterface::openFingers(const RobotSide side)
{
    controlGripper(side, ihmc_msgs::HandDesiredConfigurationRosMessage::OPEN_FINGERS);
}


void GripperControlInterface::closeFingers(const RobotSide side)
{
    controlGripper(side, ihmc_msgs::HandDesiredConfigurationRosMessage::CLOSE_FINGERS);
}

void GripperControlInterface::crush(const RobotSide side)
{
    controlGripper(side, ihmc_msgs::HandDesiredConfigurationRosMessage::CRUSH);

}

std::string GripperControlInterface::getModeName(const GRIPPER_MODES mode) const
{
    return GRIPPER_MODE_NAMES.at(mode);
}


