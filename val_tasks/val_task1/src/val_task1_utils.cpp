#include <val_task1/val_task1_utils.h>

task1Utils::task1Utils(ros::NodeHandle nh):
    nh_(nh)
{
    // subscriber for the satellite message
    satellite_sub_ = nh_.subscribe("/task1/checkpoint2/satellite", 10, &task1Utils::satelliteMsgCB, this);
}

task1Utils::~task1Utils()
{

}


void task1Utils::satelliteMsgCB(const srcsim::Satellite& msg)
{
    // update the message
    msg_ = msg;
}

// satellite dish helper tasks
bool task1Utils::isPitchCorrectNow(void)
{
    return msg_.pitch_correct_now;
}

bool task1Utils::isYawCorrectNow(void)
{
    return msg_.yaw_correct_now;
}

bool task1Utils::isPitchCompleted(void)
{
    return msg_.pitch_completed;
}

bool task1Utils::isYawCompleted(void)
{
    return msg_.yaw_completed;
}

double task1Utils::getPitchDiff (void)
{
    return (msg_.target_pitch - msg_.current_pitch);
}

double task1Utils::getYawDiff (void)
{
    return (msg_.target_yaw - msg_.current_yaw);
}
