
// There is no status feed back from VAl only if the step is taken or not


#ifndef VALKYRIE_WALKER_H
#define VALKYRIE_WALKER_H

#include "ros/ros.h"
#include"geometry_msgs/Pose2D.h"
#include <humanoid_nav_msgs/PlanFootsteps.h>
#include "ihmc_msgs/FootstepDataListRosMessage.h"
#include "ihmc_msgs/FootstepDataRosMessage.h"
#include "ihmc_msgs/FootstepStatusRosMessage.h"
#include <tf2_ros/transform_listener.h>
#include"tf2_ros/buffer.h"
#include <geometry_msgs/TransformStamped.h>
#include "std_msgs/String.h"
#include "ros/time.h"
#include "tf/tf.h"
#include <val_common/val_common_defines.h>

/**
 * @brief The ValkyrieWalker class This class provides access to the footsteps of valkyrie. It can be used
 * to get current position of the steps or to make Valkyrie walk given number of steps.
 */
class ValkyrieWalker
{

public:
    /**
     * @brief ValkyrieWalker This class provides access to the footsteps of valkyrie. It can be used
     * to get current position of the steps or to make Valkyrie walk given number of steps.
     * @param nh    ros Nodehandle to which the publishers and subscribers will be attached
     * @param InTransferTime    transfer time in seconds for every step
     * @param InSwingTime   swing time in seconds for every step
     * @param InMode    execution mode of the steps. this can be 0 = OVERRIDE or 1 = QUEUE.
     * @param swingHeight   swing height to be used for every step
     */
    ValkyrieWalker(ros::NodeHandle nh, double InTransferTime = 1.5,double InSwingTime =1.5 , int InMode = 0, double swingHeight = 0.2);
    ~ValkyrieWalker();

    /**
     * @brief walkToGoal walks to a given 2D point in a map. needs a map either from map server or from octomap server
     * @param goal  pose2d message giving position and orientation of goal point.
     * @return true if footstep planning is successful else false
     */
    bool walkToGoal( geometry_msgs::Pose2D &goal);


    bool walkNSteps(int n, float x_offset, float y_offset, bool continous, armSide startLeg);

    /// \todo implement this function
    //bool WalkNStepsBackward(int n, float x_offset = 0.4, float y_offset=0.0, bool continous = false);

    bool walkPreComputedSteps(const std::vector<float> x_offset, const std::vector<float> y_offset, armSide startleg);
    bool walkGivenSteps(ihmc_msgs::FootstepDataListRosMessage& list );
    inline void setWalkParms(float InTransferTime,float InSwingTime, int InMode)
    {
        this->transfer_time = InTransferTime;
        this->swing_time = InSwingTime;
        this->exe_mode = InMode;
    }

    double getSwing_height() const;
    inline void setSwing_height(double value)
    {
        swing_height = value;
    }

private:
    ros::NodeHandle n;
    ros::ServiceClient footstep_client ;
    ros::Publisher footsteps_to_val ;
    ros::Subscriber footstep_status ;
    int step_counter;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tf_listener;
    double transfer_time,swing_time, swing_height;
    int exe_mode;
    static int id ;
    ros::Time cbTime;

    std_msgs::String right_foot_frame,left_foot_frame;

    void footstepStatusCB(const ihmc_msgs::FootstepStatusRosMessage & msg);
    void getCurrentStep(int side , ihmc_msgs::FootstepDataRosMessage& foot);
    ihmc_msgs::FootstepDataRosMessage* getOffsetStep(int side, float x, float y);

    /// \todo wrong implementation. get rid of this
    void waitForSteps( int n);

    bool getFootstep(geometry_msgs::Pose2D &goal,ihmc_msgs::FootstepDataListRosMessage &list);

};
 int ValkyrieWalker::id = -1;

#endif
