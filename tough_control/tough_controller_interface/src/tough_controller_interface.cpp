#include "tough_controller_interface/tough_controller_interface.h"

long ToughControllerInterface::id_ = -1;

ToughControllerInterface::ToughControllerInterface(ros::NodeHandle nh){

    if(!nh.getParam("ihmc_ros/robot_name", robot_name_)){
        ROS_ERROR("ihmc_ros/robot_name parameter is not on the server. Using valkyrie by default");
        robot_name_ = "valkyrie";
    }

    control_topic_prefix_ = "ihmc_ros/"+robot_name_+"/control";
    output_topic_prefix_ = "ihmc_ros/"+robot_name_+"/output";

    state_informer_ = RobotStateInformer::getRobotStateInformer(nh);
    rd_ = RobotDescription::getRobotDescription(nh_);

}

ToughControllerInterface::~ToughControllerInterface(){

}
