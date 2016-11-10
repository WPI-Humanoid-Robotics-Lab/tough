#include "ros/ros.h"
#include <cstdlib>
#include"geometry_msgs/PoseStamped.h"
#include"geometry_msgs/PoseWithCovarianceStamped.h"
#include"geometry_msgs/Pose2D.h"
#include <humanoid_nav_msgs/PlanFootsteps.h>
#include <humanoid_nav_msgs/StepTarget.h>
#include <val_footstep/StepTargetArray.h>
#include "std_msgs/Float64MultiArray.h"
#include <boost/array.hpp>
#include"geometry_msgs/Vector3.h"
#include"geometry_msgs/Quaternion.h"
#include "ihmc_msgs/FootstepDataListRosMessage.h"
#include "ihmc_msgs/FootstepDataRosMessage.h"
#include "ihmc_msgs/FootstepStatusRosMessage.h"
#include <tf2_ros/transform_listener.h>
#include"tf2_ros/buffer.h"
#include <geometry_msgs/TransformStamped.h>
#include "std_msgs/String.h"
#include "ros/time.h"



void statusCallback(const ihmc_msgs::FootstepStatusRosMessage)
{

}



int main(int argc, char **argv)

{
    ros::init(argc, argv, "pass_footstep");
    ros::NodeHandle n;
    ros::ServiceClient footStep_client = n.serviceClient <humanoid_nav_msgs::PlanFootsteps> ("pass_footsteps");
    ros::Publisher footStepsToVal = n.advertise<ihmc_msgs::FootstepDataListRosMessage>("/ihmc_ros/valkyrie/control/footstep_list",1,true);
    ros::Subscriber footStepStatus = n.subscribe("/ihmc_ros/valkyrie/output/footstep_status",20,statusCallback);
    std::string robot_name,right_foot_frame,left_foot_frame;
    std_msgs::String Right_Foot_Frame,Left_Foot_Frame;

    if (n.getParam("/ihmc_ros/robot_name",robot_name))
    {
        if(n.getParam("/ihmc_ros/valkyrie/right_foot_frame_name", right_foot_frame) && n.getParam("/ihmc_ros/valkyrie/left_foot_frame_name", left_foot_frame))
        {
            Right_Foot_Frame.data = right_foot_frame;
            Left_Foot_Frame.data = left_foot_frame;
            ROS_INFO("%s", Right_Foot_Frame.data.c_str());
            ROS_INFO("%s", Left_Foot_Frame.data.c_str());
        }
    }
    else
    {
        ROS_ERROR("Failed to get param 'my_param'");
    }

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate loop_rate(10);
    ros::Duration(1).sleep();

    return 0;
}
