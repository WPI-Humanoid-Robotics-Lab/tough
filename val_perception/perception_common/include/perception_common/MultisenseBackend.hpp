
#ifndef MULTISENSE_BACKEND_HPP
#define	MULTISENSE_BACKEND_HPP
/******************** INCLUDED FILES ********************/
# include "wrecs_msgs/Multisense_Backend_Info.h"
# include "ros/ros.h"
# include "ros/subscriber.h"
# include "ros/publisher.h"
# include "wrecs_common/WRECS_Names.h"
# include "MultisenseControl.h"
#define MULTISENSE_BACKEND_NAME "Multisense Backend"


/********************* CONTROLLER **********************/

/**
 * @brief	connect the actions of the GUI to methods in MultisenseControl.h
 */
class MultisenseBackendController
{
	public:
		/**
		 * @brief   Constructor for MultisenseControlController
		 */
	explicit MultisenseBackendController(ros::NodeHandle &nh);

		/**
		 * @brief  Destructor for MultisenseControlController
		 */
	virtual ~MultisenseBackendController();

		void ProcessMsg(wrecs_msgs::Multisense_Backend_Info msg);

        void UpdateGUIBackward();

        void SendCMD();

        void SendBack();


	private:

        drc_perception::MultisenseControl *multisenseControl;

        dynamic_reconfigure::Config conf;                           // for the multisense spindle

        ros::Subscriber _sub;
        ros::Publisher _pub;

        wrecs_msgs::Multisense_Backend_Info _stored_msg;
        wrecs_msgs::Multisense_Backend_Info _update_msg;

        ros::NodeHandle *_nh;


};

#endif // MULTISENSE_BACKEND_HPP

