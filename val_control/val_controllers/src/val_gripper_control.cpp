#include <val_controllers/val_gripper_control.h>
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

void gripperControl::controlGripper(const armSide side, GRIPPER_STATE state)
{
    std_msgs::Float64MultiArray msg;
    msg.data.clear();

    switch (state) {
    case GRIPPER_STATE::OPEN:
        msg.data.resize(5);
        break;

    case GRIPPER_STATE::OPEN_THUMB_IN:
        msg.data.resize(5);
        msg.data[0] = 1.4;
        break;

    case GRIPPER_STATE::CLOSE:
        msg.data = side == LEFT? CLOSE_LEFT_GRIPPER : CLOSE_RIGHT_GRIPPER;
        break;

    case GRIPPER_STATE::CUP:
        msg.data = side == LEFT? CUP_LEFT_GRIPPER : CUP_RIGHT_GRIPPER;
        break;

    default:
        break;
    }

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
