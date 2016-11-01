/**
 ********************************************************************************************************
 * @file 		MultisenseControl.h
 * @brief		implements functions to control the settings in multisense.
 * @details 	gives functions specific to the modes of operation related to perception
 ********************************************************************************************************
 */

/*** INCLUDE FILES ***/
#include <ros/ros.h>
#include <perception_common/global.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

namespace drc_perception{
/**
 * @brief   The helper class to control multisense settings
 */
class MultisenseControl
{
private:
	DISALLOW_COPY_AND_ASSIGN(MultisenseControl)
	bool	auto_exp_enabled_;

#ifdef GAZEBO_SIMULATION
	ros::Publisher motor_pub_;
	ros::Publisher fps_pub_;
#endif

    double speed_low_, speed_high_ ;
    std::string reso_low_, reso_high_ ;
	/**
	 * @brief sets the config for dynamic reconfiguration
	 * @param conf - the dynamic reconfigure object containing the settings to be set
	 */
	void setConf(const dynamic_reconfigure::Config& conf);

	/**
	 * @brief set the motor speed and modify the dynamic reconfig files accordingly
	 * @param radPerSec the speed in radians per sec
	 * @param conf the modified dynamic reconfigure file
	 */
	void setMotorSpeed(double radPerSec, dynamic_reconfigure::Config& conf);
	/**
	 * @brief set the camera resolution and modify the dynamic reconfig files accordingly
	 * @param res the resolution as a string eg "2048x1088x64"
	 * @param conf the modified dynamic reconfigure file
	 */
	void setResolution(const std::string& res, dynamic_reconfigure::Config& conf);
	/**
	 * @brief sets the reconfig setting for the lights to be switched on
	 */
	void switchOnLight(dynamic_reconfigure::Config  &conf);
	/**
	 * @brief This function sets the camera to the capture data at the desired fps(<=30).
	 * @param fps	the fps we want the camera to operate at
	 * @param conf	the config data struct that needs to be set
	 */
	void setFPS(const float fps, dynamic_reconfigure::Config  &conf);

public:
	MultisenseControl(ros::NodeHandle &nh);
	/**
	 * @brief sets the camera in the high scan mode of operation. see the launch file for high scan mode
	 * 		  settings.
	 */
	void disableAutoExposure();

	void enableAutoExposure();
	/**
	 * @brief sets the camera in the high scan mode of operation. see the launch file for high scan mode
	 * 		  settings.
	 */
	void setHighMode();
	/**
	 * @brief sets the camera in the low scan mode of operation. see the launch file for settings.
	 */
	void setLowMode();
	/**
	 * @brief sets the reconfig settings for the lights on the multisense head to be turned off
	 */
	void switchOFFLight();
	/**
	 * @brief this function reads the LED power from the param server
	 * @return the LED power
	 */
	double getLEDPower();
	/**
	 * @brief this function reads the Laser speed from the param server
	 * @return the Laser Speed
	 */
	double getLaserSpeed();
	/**
	 * @brief this function reads the exposure max time from the param server
	 * @return the exposure
	 */
	double getExposure();
	/**
	 * @brief this function is used to set the speed of the spindle.
	 * @param speed the spindle speed
	 */
	void setLaserSpeed(double speed);
	/**
	 * @brief this function sets the camera power to the given power
	 * @param power the power of the led [0,1]
	 */
	void setLEDPower(double power);
	/**
	 * @brief changes the autoexposure setting of the multisense head.
	 */
	void changeAutoExposure();
	/**
	 * @brief this makes the multisense head start in a predefined mode where the LED is on.
	 */
	void startNormal();
	void setFPS(const float &fps);

	/**
	 * @brief changes the exposure thresh's value
	 */
	void setExposureThresh(double value);

	void setExposure(double value);

	//BW always declare your destroyer as virtual so that classes which inherit your class can destroy it
	virtual ~MultisenseControl();
};
}
