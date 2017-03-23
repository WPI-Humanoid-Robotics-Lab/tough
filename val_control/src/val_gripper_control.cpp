#include <val_control/val_gripper_control.h>
#include <tf/transform_listener.h>
#include <val_common/val_common_names.h>

gripperControl::gripperControl(ros::NodeHandle nh) : nh_(nh){

    leftGripperContPublisher =
            nh_.advertise<std_msgs::Float64MultiArray>("/left_hand_position_controller/command",1,true);
    rightGripperContPublisher =
            nh_.advertise<std_msgs::Float64MultiArray>("/right_hand_position_controller/command",1,true);
}

gripperControl::~gripperControl(){

}

void gripperControl::controlGripper(const armSide side, const std::vector<double> gripperData){

    std_msgs::Float64MultiArray msg;
    msg.data.clear();

    if (gripperData.size() != 5){
        ROS_ERROR("Please check the size of the vector");
    }

     msg.data = gripperData;

    if (side == LEFT){
        leftGripperContPublisher.publish(msg);
    }

    else {
        rightGripperContPublisher.publish(msg);
    }
}

void gripperControl::closeGripper(const armSide side)
{
    std_msgs::Float64MultiArray msg;
    msg.data.clear();

    if (side == LEFT){
        msg.data = CLOSE_LEFT_GRIPPER;
        leftGripperContPublisher.publish(msg);
    }
    else {
        msg.data = CLOSE_RIGHT_GRIPPER;
        rightGripperContPublisher.publish(msg);
    }
}

void gripperControl::openGripper(const armSide side)
{
    std_msgs::Float64MultiArray msg;
    msg.data.clear();
    msg.data.resize(5);
    if (side == LEFT){
        leftGripperContPublisher.publish(msg);
    }
    else {
        rightGripperContPublisher.publish(msg);
    }

}
