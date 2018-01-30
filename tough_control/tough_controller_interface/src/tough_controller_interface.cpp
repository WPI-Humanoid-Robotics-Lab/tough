#include "tough_controller_interface/tough_controller_interface.h"

long ToughControllerInterface::id_ = 1;

ToughControllerInterface::ToughControllerInterface(ros::NodeHandle nh){

    std::string robot_name;
    nh.getParam("ihmc_ros/robot_name", robot_name);

    control_topic_prefix_ = "ihmc_ros/"+robot_name+"/control";
    output_topic_prefix_ = "ihmc_ros/"+robot_name+"/output";

    state_informer_ = RobotStateInformer::getRobotStateInformer(nh);
    rd_ = RobotDescription::getRobotDescription(nh_);

}

ToughControllerInterface::~ToughControllerInterface(){

}
