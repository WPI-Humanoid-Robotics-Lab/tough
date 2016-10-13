/**
 ********************************************************************************************************
 * @file    Multisense_Back.cpp
 * @brief   Receive msg from GUI to commend the Atlas
 * @details This node subscribes message from Nudge GUI and use API to adjust the position of the hand.
 ********************************************************************************************************
 */

# include "perception_common/MultisenseBackend.hpp"

//using namespace multisense_control_controller;

MultisenseBackendController::MultisenseBackendController(ros::NodeHandle &nh):
										_nh(new ros::NodeHandle(nh)),
										multisenseControl(new drc_perception::MultisenseControl(*this->_nh))
{

    this->_sub = this->_nh->subscribe(WRECS_NAMES::ATLAS_GUI_MULTISENSE_TOPIC,10,&MultisenseBackendController::ProcessMsg,this);

    this->_pub = this->_nh->advertise<wrecs_msgs::Multisense_Backend_Info>(WRECS_NAMES::ATLAS_MULTISENSE_TOPIC,1);
}

MultisenseBackendController::~MultisenseBackendController()
{
    delete _nh;
}

void MultisenseBackendController::ProcessMsg(wrecs_msgs::Multisense_Backend_Info msg)
{
	std::cout<<"receive msg"<<std::endl;
    _stored_msg = msg;
    SendCMD();
}

void MultisenseBackendController::SendCMD()
{
    // if(_stored_msg.isLEDOn == wrecs_msgs::Multisense_Backend_Info::LEDOn)
    // 	multisenseControl->setLEDPower(wrecs_msgs::Multisense_Backend_Info::LEDOn);
    // else if(_stored_msg.isLEDOn == wrecs_msgs::Multisense_Backend_Info::LEDOff)
    // 	multisenseControl->setLEDPower(wrecs_msgs::Multisense_Backend_Info::LEDOff);
    // else
    // 	multisenseControl->setLEDPower(_stored_msg.power);

    // multisenseControl->setExposureThresh(_stored_msg.exposure);
    // multisenseControl->setLaserSpeed(_stored_msg.spindle);

    if(_stored_msg.powerClick == true)
    {
        if(_stored_msg.isLEDOn == wrecs_msgs::Multisense_Backend_Info::LEDOn)
        {
            double power=pow(_stored_msg.power,2.0); 
            multisenseControl->setLEDPower(power);
        }
        else if(_stored_msg.isLEDOn == wrecs_msgs::Multisense_Backend_Info::LEDOff)
            multisenseControl->setLEDPower(wrecs_msgs::Multisense_Backend_Info::LEDOff);
    }
    if(_stored_msg.exposureClick == true)
    {
        multisenseControl->setExposureThresh(_stored_msg.exposure/200.00);
    }
    if(_stored_msg.spindleClick == true)
    {
        multisenseControl->setLaserSpeed(_stored_msg.spindle);
    }
    if(_stored_msg.autoExpClick==true)
    {
        if(_stored_msg.autoExp)
        {
            multisenseControl->enableAutoExposure();
            multisenseControl->setExposureThresh(_stored_msg.exposure/1000.00);
        }
        else
        {
            multisenseControl->disableAutoExposure();
        }
    }



}

void MultisenseBackendController::UpdateGUIBackward()
{
    _update_msg.power= multisenseControl->getLEDPower();
    _update_msg.spindle=multisenseControl->getLaserSpeed();
    _update_msg.exposure=multisenseControl->getExposure();
    if(_update_msg.power == _update_msg.LEDOn)
    {
        _update_msg.isLEDOn = _update_msg.LEDOn;
    }
    else if(_update_msg.power == _update_msg.LEDOff)
    {
        _update_msg.isLEDOn = _update_msg.LEDOff;
    }

    SendBack();

    // cannot get the speed and the exposure.
}

void MultisenseBackendController::SendBack()
{
    wrecs_msgs::Multisense_Backend_Info msg;
    msg = _update_msg;
    _pub.publish(msg);
}

int main(int argc, char **argv)
{
    std::shared_ptr<MultisenseBackendController> multisense_backend;
    ros::init(argc,argv,"MultisenseBackendController");

    ros::NodeHandle nh("~");

    double period = 5;

    if (nh.getParam("bundle_period", period))
    {
      std::cout << "Got bundle period param: " << period << std::endl;
    }

    multisense_backend.reset(new MultisenseBackendController(nh));

    ros::spin();
    return 0;
}
