#include <val_interface/val_interface.h>

valInterface::valInterface(ros::NodeHandle &nh):
    nh_(nh)
{
    pelvisMiddle_ = new imu(nh, "/pelvisMiddleImu/imu");
    pelvisRear_ = new imu(nh, "/pelvisRearImu/imu");
    leftTorso_ = new imu(nh, "/leftTorsoImu/imu");
    force_torque_states_=new forcesensor(nh,"/force_torque_states");
}

valInterface::~valInterface()
{

}

// get IMU
sensor_msgs::Imu valInterface::getPelvisMiddleImu(void){
    return pelvisMiddle_->getIMU();
}

sensor_msgs::Imu valInterface::getPelvisRearIMU(void){
    return pelvisRear_->getIMU();
}

sensor_msgs::Imu valInterface::getLeftTorsoIMU(void) {
    return leftTorso_->getIMU();
}

// get force sensor and torque data
geometry_msgs::Wrench valInterface::getForceSensLeftFoot(void){

    // check if the data is recieved
    if (!force_torque_states_->getForce().forceTorque.empty())
    {
        return force_torque_states_->getForce().forceTorque[0];
    }
}

geometry_msgs::Wrench valInterface::getForceSensLeftFootOffset(void){

    // check if the data is recieved
    if (!force_torque_states_->getForce().forceTorque.empty())
    {
        return force_torque_states_->getForce().forceTorque[1];
    }
}

geometry_msgs::Wrench valInterface::getForceSensRightFoot(void){

    // check if the data is recieved
    if (!force_torque_states_->getForce().forceTorque.empty())
    {
        return force_torque_states_->getForce().forceTorque[2];
    }
}

geometry_msgs::Wrench valInterface::getForceSensRightFootOffset(void){

    // check if the data is recieved
    if (!force_torque_states_->getForce().forceTorque.empty())
    {
        return force_torque_states_->getForce().forceTorque[3];
    }
}

// get the joint states
