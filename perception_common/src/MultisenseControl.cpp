/**
 ********************************************************************************************************
 * @file    MultisenseControl.cpp
 * @brief   Class controling the multisense
 * @details Allows you to set parameters and setting more cleanly
 ********************************************************************************************************
 */
#include <perception_common/MultisenseControl.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <std_msgs/Float64.h>
#include <wrecs_common/WRECS_Names.h>


/************************************************** CONSTANTS *******************************************/
namespace multisense_control {
	const double EXPOSURE_THRESH_MIN = 0.0;
	const double EXPOSURE_THRESH_MAX = 1.0;
}


namespace drc_perception{
/**
 * @note The default constructor, for some reason the motor_pub_ crashes the system in simulator
 */
MultisenseControl::MultisenseControl(ros::NodeHandle &nh)
{
	//BW: Need to use private node handle to access parameters specific to your node
   	ros::NodeHandle pnh("~");
   	//BW: Need to set default values in cases where the launch does not consist of it
   	if(!pnh.getParam("lowSpeed", speed_low_))
   	{
   		speed_low_ = 0.1f;
   	}
   	if(!pnh.getParam("highSpeed", speed_high_))
  	{
  		speed_high_ =4.0f;
   	}
   	if(!pnh.getParam("lowResolution", reso_low_))
   	{
   		reso_low_ = std::string("1024x544x128");
   	}

   	if(!pnh.getParam("highResolution", reso_high_))
   	{
   		reso_high_= std::string("2048x1088x256");
   	}

	auto_exp_enabled_=true;
#ifdef GAZEBO_SIMULATION
   	motor_pub_=ros::Publisher(nh.advertise<std_msgs::Float64>(WRECS_NAMES::MULTISENSE_CONTROL_MOTOR_TOPIC, 1));
   	fps_pub_=ros::Publisher(nh.advertise<std_msgs::Float64>(WRECS_NAMES::MULTISENSE_CONTROL_FPS_TOPIC, 1));
#endif
}
/**
 * @note this function will set the configurations that are availabe to
 * /multisense_sl/set_parameters
 */
void MultisenseControl::setConf(const dynamic_reconfigure::Config& conf)
{
   	dynamic_reconfigure::ReconfigureRequest  srv_req;
   	dynamic_reconfigure::ReconfigureResponse srv_resp;
   	srv_req.config = conf;

   	ros::service::call(WRECS_NAMES::MULTISENSE_CONTROL_SERVICE, srv_req, srv_resp);
}
/**
 * @note returns normalized power which is in [0,1]
 */
double MultisenseControl::getLEDPower()
{
	ros::NodeHandle nh;
	double led;
	if(nh.getParam(WRECS_NAMES::MULTISENSE_LED_DUTY_CYCLE,led))
	{
		return led;
	}
	return -1;
}


/**
 * @note returns normalized power which is in [0,5]
 */
double MultisenseControl::getLaserSpeed()
{
	ros::NodeHandle nh;
	double speed;
	if(nh.getParam(WRECS_NAMES::MULTISENSE_LASER_SPEED,speed))
	{
		return speed;
	}
	return -1;
}

double MultisenseControl::getExposure()
{
	ros::NodeHandle nh;
	double exposure;
	if(nh.getParam(WRECS_NAMES::MULTISENSE_MAX_EXPOSURE,exposure))
	{
		return exposure;
	}
	return -1;
}
/**
 * @note this function sets the appends the motor speed settings to the config
 */
void MultisenseControl::setMotorSpeed(double radPerSec, dynamic_reconfigure::Config &conf)
{
    dynamic_reconfigure::DoubleParameter double_param;
    double_param.name  = "motor_speed";
    double_param.value = radPerSec;
    conf.doubles.push_back(double_param);
}
/**
 * @note sets resolution of camera after v3,3
 */
void MultisenseControl::setResolution(const std::string& res, dynamic_reconfigure::Config &conf)
{
    dynamic_reconfigure::StrParameter str_param;
    str_param.name  = "resolution";
    str_param.value = res;
    conf.strs.push_back(str_param);

}
/**
 * @note in high mode the laser scans slowly but the camera is the highest resolution
 */
void MultisenseControl::setHighMode()
{
	dynamic_reconfigure::Config  conf ;
	setMotorSpeed(speed_low_, conf);
	setResolution(reso_high_, conf);
	setConf(conf);
	ROS_INFO_STREAM("Changing to High Mode laser: "<<speed_low_<<", camera: "<<reso_high_);
}
/**
 * @note in low mode the laser scans in the highest speed but the camera is low resolution
 */
void MultisenseControl::setLowMode()
{
	dynamic_reconfigure::Config  conf ;
	setMotorSpeed(speed_high_, conf);
	setResolution(reso_low_, conf);
	setConf(conf);
	ROS_INFO_STREAM("Changing to Low Mode laser: "<<speed_high_<<", camera: "<<reso_low_);
}
/**
 * @note the led is set to max power whenits switched on
 */
void MultisenseControl::switchOnLight(dynamic_reconfigure::Config  &conf)
{
	dynamic_reconfigure::BoolParameter bool_param;
	bool_param.name  = "lighting";
	bool_param.value = 1;
	conf.bools.push_back(bool_param);

	dynamic_reconfigure::DoubleParameter dbl_param;
	dbl_param.name = "led_duty_cycle";
	dbl_param.value =1.0f;
	conf.doubles.push_back(dbl_param);
}
/**
 * @note normalized power is the input [0,1]
 */
void MultisenseControl::setLEDPower(double power)
{
	dynamic_reconfigure::Config  conf;
	dynamic_reconfigure::DoubleParameter dbl_param;
	dbl_param.name = "led_duty_cycle";
	dbl_param.value =power;
	conf.doubles.push_back(dbl_param);
        dynamic_reconfigure::BoolParameter bool_param;
	bool_param.name  = "lighting";
	if(power>0.0)	
           bool_param.value = 1;
        else
	   bool_param.value = 0;
	conf.bools.push_back(bool_param);
	setConf(conf);
	ROS_INFO_STREAM("Setting Multisense LED power to: "<<power);
}
/**
 * @note fps is from 0-30
 */
void MultisenseControl::setFPS(const float fps, dynamic_reconfigure::Config  &conf)
{
	dynamic_reconfigure::DoubleParameter dbl_param;
	dbl_param.name = "fps";
	dbl_param.value =fps;
	conf.doubles.push_back(dbl_param);
}
void MultisenseControl::setFPS(const float &fps)
{
#ifndef GAZEBO_SIMULATION
	dynamic_reconfigure::Config  conf;
	setFPS(fps,conf);
	setConf(conf);
#else
	std_msgs::Float64 f;
	f.data=fps;
	fps_pub_.publish(f);
#endif
	ROS_INFO_STREAM("Setting Multisense Head to: "<<fps<<" fps");
}
/**
 * @note high speed Laser, high fps camera, LED on
 */
void MultisenseControl::startNormal()
{
	dynamic_reconfigure::Config  conf;
	setMotorSpeed(speed_high_, conf);
	setResolution(reso_low_, conf);
	switchOnLight(conf);
	setFPS(30.0,conf);
	setConf(conf);
	ROS_INFO_STREAM("Setting Multisense Head in Normal mode: "<<speed_high_<<", camera: "<<reso_low_);
}
/**
 * @note speed in is inthe range 0-5.2 for the SIM alone I do the hack of directly setting the spindle speed
 */
void MultisenseControl::setLaserSpeed(double speed)
{
#ifndef GAZEBO_SIMULATION
	dynamic_reconfigure::Config  conf;
	setMotorSpeed(speed, conf);
	setConf(conf);
#else
	std_msgs::Float64 f;
	f.data=speed;
	motor_pub_.publish(f);
#endif
	ROS_INFO_STREAM("Changing to laser speed: "<<speed);
}

/**
 * @note the formal parameter must be in the range [0.0, 1.0]
 */
void MultisenseControl::setExposureThresh(double value)
{
	// if (value >= multisense_control::EXPOSURE_THRESH_MIN &&
	// 	value <= multisense_control::EXPOSURE_THRESH_MAX) {

		dynamic_reconfigure::Config conf;
		dynamic_reconfigure::DoubleParameter dbl_params;

		if(auto_exp_enabled_)
		{
			//dbl_params.name  = "auto_exposure_thresh";
			return;
		}
		else
			dbl_params.name  = "exposure_time";

		dbl_params.value = value;

		conf.doubles.push_back(dbl_params);
		setConf(conf);
		ROS_INFO_STREAM("Setting auto exposure thresh to: " << value);
	//}
}

void MultisenseControl::setExposure(double value)
{

	dynamic_reconfigure::Config conf;
	dynamic_reconfigure::DoubleParameter dbl_params;

	dbl_params.name  = "exposure_time";

	dbl_params.value = value;

	conf.doubles.push_back(dbl_params);
	setConf(conf);
	ROS_INFO_STREAM("Setting auto exposure thresh to: " << value);

}

void MultisenseControl::disableAutoExposure()
{
	dynamic_reconfigure::Config conf;
	dynamic_reconfigure::BoolParameter bool_params;

	bool_params.name  = "auto_exposure";
	bool_params.value = false;
	auto_exp_enabled_=false;
	conf.bools.push_back(bool_params);
	setConf(conf);	
}

void MultisenseControl::enableAutoExposure()
{
	dynamic_reconfigure::Config conf;
	dynamic_reconfigure::BoolParameter bool_params;

	bool_params.name  = "auto_exposure";
	bool_params.value = true;
	auto_exp_enabled_=true;
	conf.bools.push_back(bool_params);
	setConf(conf);	
}


MultisenseControl::~MultisenseControl()
{
}

}

