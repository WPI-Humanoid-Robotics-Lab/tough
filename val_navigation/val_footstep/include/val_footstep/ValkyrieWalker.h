#ifndef VALKYRIE_WALKER_H
#define VALKYRIE_WALKER_H

#include "ros/ros.h"
#include"geometry_msgs/Pose2D.h"
#include <humanoid_nav_msgs/PlanFootsteps.h>
#include "ihmc_msgs/FootstepDataListRosMessage.h"
#include "ihmc_msgs/FootstepDataRosMessage.h"
#include "ihmc_msgs/FootstepStatusRosMessage.h"
#include "ihmc_msgs/FootTrajectoryRosMessage.h"
#include "ihmc_msgs/WholeBodyTrajectoryRosMessage.h"
#include<ihmc_msgs/EndEffectorLoadBearingRosMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include "std_msgs/String.h"
#include "ros/time.h"
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include <val_common/val_common_defines.h>
#include <val_controllers/robot_state.h>



/**
 * @brief The RobotWalker class is an API over IHMC footstep messages.
 * It uses a footstep planner to plan footsteps.
 * It can plan footstep trajectories.
 */
class ValkyrieWalker
{

public:
    /**
     * @brief RobotWalker provides access to the footsteps of robot. It can be used
     * to get current position of the steps or to make  the robot walk given number of steps.
     * @param nh                Ros Nodehandle to which the publishers and subscribers will be attached
     * @param inTransferTime    Transfer time in seconds for every step
     * @param inSwingTime       Swing time in seconds for every step
     * @param inMode            Execution mode of the steps. this can be 0 = OVERRIDE or 1 = QUEUE.
     * @param swingHeight       Swing height to be used for every step
     * @todo Implement singleton pattern. There should be only one object of this class available of this class.
     */
    ValkyrieWalker(ros::NodeHandle nh, double inTransferTime = 1.5,double inSwingTime =1.5 , int inMode = 0, double swing_height_ = 0.2);
    ~ValkyrieWalker();

    /**
     * @brief walkToGoal walks to a given 2D point in a map. It needs a map either from map server or from octomap server
     * @param goal      Pose2d message giving position and orientation of goal point.
     * @return true     If footstep planning is successful else false
     */
    bool walkToGoal(geometry_msgs::Pose2D &goal, bool waitForSteps=true);

    /**
     * @brief walkNSteps Makes the robot walk given number of steps.
     * @param numSteps   Number of steps to walk
     * @param xOffset    Distance to travel forward in half stride. First step is half the stride length as both the feet are assumed to be together.
     *                   This is defined wrt World.
     * @param yOffset    Distance to travel sideways in half a stride length. First step is half the stride length as both the feet are assumed to be together.
     *                   This is defined wrt World.
     * @param continous  If this is set to true, the robot stops with one foot forward. if it is false, both the feet are together at the end of walk.
     * @param startLeg   Leg to be used to start walking. It can be RIGHT or LEFT
     * @return
     */
    bool walkNSteps(int numSteps, float xOffset, float yOffset=0.0f, bool continous=false, armSide startLeg=RIGHT, bool waitForSteps=true);

    /**
     * @brief walkNSteps Makes the robot walk given number of steps.
     * @param numSteps   Number of steps to walk
     * @param xOffset    Distance to travel forward in half stride. First step is half the stride length as both the feet are assumed to be together.
     *                   This is defined wrt Pelvis. Forward - positive x axis , Left - positive y axis
     * @param yOffset    Distance to travel sideways in half a stride length. First step is half the stride length as both the feet are assumed to be together.
     *                   This is defined wrt Pelvis. Forward - positive x axis , Left - positive y axis
     * @param continous  If this is set to true, the robot stops with one foot forward. if it is false, both the feet are together at the end of walk.
     * @param startLeg   Leg to be used to start walking. It can be RIGHT or LEFT
     * @return
     */
    bool walkNStepsWRTPelvis(int numSteps, float xOffset, float yOffset=0.0f, bool continous=false, armSide startLeg=RIGHT, bool waitForSteps=true);

    /**
     * @brief walkPreComputedSteps If the steps to be sent to the robot are not identical, use this function to send steps that are precomputed.
     * @param xOffset  is a vector of float. Each value represents offset in x direction of individual step
     * @param yOffset  is a vector of float with size same as that of x_offset. Each value represents offset in y direction of individual step
     * @param startleg  Leg to be used to start walking. It can be RIGHT or LEFT
     * @return
     */
    bool walkPreComputedSteps(const std::vector<float> xOffset, const std::vector<float> yOffset, armSide startleg);

    /**
     * @brief walkGivenSteps This function publishes a given list of ros messages of type ihmc_msgs::FootstepDataListRosMessage to the robot.
     * @param list           List of steps in ihmc_msgs::FootstepDataListRosMessage format.
     * @return
     */
    bool walkGivenSteps(ihmc_msgs::FootstepDataListRosMessage& list , bool waitForSteps=true);

    /**
     * @brief setWalkParms      Set the values of walking parameters
     * @param InTransferTime    Transfer time is the time required for the robot to switch its weight from one to other while walking.
     * @param InSwingTime       Swing time is the time required for the robot to swing its leg to the given step size.
     * @param InMode            Execution mode defines if steps are to be queued with previous steps or override and start a walking message. Only Override is supported in this version.
     * @todo create separate messages for each of the parameters.
     */
    inline void setWalkParms(float inTransferTime,float inSwingTime, int inMode)
    {
        this->transfer_time_ = inTransferTime;
        this->swing_time_ = inSwingTime;
        this->execution_mode_ = inMode;
    }

    /**
     * @brief getSwingHeight fetch the swing height used for steps.
     * @return returns the swing_height of the current object.
     */
    double getSwingHeight() const;

    /**
     * @brief setSwingHeight Sets swing_height for walking.
     * @param value          Value is the swing_height that determines how high a feet should be lifted while walking in meters. It should be between 0.1 and 0.25     *
     */
    inline void setSwingHeight(double value)
    {
        swing_height_ = value;
    }
    bool turn(armSide side);
    bool walkLocalPreComputedSteps(const std::vector<float> x_offset, const std::vector<float> y_offset, armSide startLeg);
    RobotStateInformer *current_state_;
    bool curlLeg(armSide side, float radius);
    bool placeLeg(armSide side, float offset=0.1f);
    bool nudgeFoot(armSide side, float distance);
    void getCurrentStep(int side , ihmc_msgs::FootstepDataRosMessage& foot);
    bool raiseLeg(armSide side, float height,float stepLength);
    void load_eff(armSide side, EE_LOADING load);
    bool walk_rotate(float angle);
    bool climbStair(const std::vector<float> x_offset, const std::vector<float> z_offset, armSide startLeg);
    bool getFootstep(geometry_msgs::Pose2D &goal,ihmc_msgs::FootstepDataListRosMessage &list);
    static int id ;
private:

    double transfer_time_,swing_time_, swing_height_;
    int execution_mode_;
    int step_counter_;
    ros::NodeHandle     nh_;
    ros::Time           cbTime_;
    ros::Publisher      footsteps_pub_ ,nudgestep_pub_,loadeff_pub;
    ros::Subscriber     footstep_status_ ;
    ros::ServiceClient  footstep_client_ ;
    tf::TransformListener       tf_listener_;

    std_msgs::String right_foot_frame_,left_foot_frame_;

    void footstepStatusCB(const ihmc_msgs::FootstepStatusRosMessage & msg);

    void waitForSteps( int n);
    ihmc_msgs::FootstepDataRosMessage::Ptr getOffsetStep(int side, float x, float y);
    ihmc_msgs::FootstepDataRosMessage::Ptr getOffsetStepWRTPelvis(int side , float x, float y);

};

#endif  //VALKYRIE_WALKER_H
