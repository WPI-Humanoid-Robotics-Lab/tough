#include<val_interface/forcesensor_interface.h>

//constructor

forcesensor::forcesensor(ros::NodeHandle &nh, const std::string forcesensorTopic):
    nh_(nh), forcesensorTopic_(forcesensorTopic)
{
    sub_force_=nh.subscribe(forcesensorTopic_,1,&forcesensor::forcesensor_callback,this);

}

void forcesensor::forcesensor_callback(const val_hardware_msgs::valAtiSensor& msg_in)
{
    {
     //acquire the lock
     std::lock_guard<std::mutex> guard(mtx_);
     msg_ = msg_in;
    }
}


val_hardware_msgs::valAtiSensor forcesensor::getForce(void)
{
    { //acquire the lock
        std::lock_guard<std::mutex> guard(mtx_);
        return  msg_;
    }
}
