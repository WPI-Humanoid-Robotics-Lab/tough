#ifndef VAL_HEAD_NAVIGATION_H
#define VAL_HEAD_NAVIGATION_H

#include <ros/ros.h>
#include <ihmc_msgs/HeadTrajectoryRosMessage.h>
#include <tf/tf.h>
#include <math.h>
#include <time.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

/**
 * @brief The HeadTrajectory class provides ability to move the head of valkyrie. Current implementation provides the ability to move the head to a set roll, pitch, and yaw.
 */
class HeadTrajectory {

private:
    static int head_id;
    ros::NodeHandle nh_;
    ros::Publisher headTrajPublisher;

public:

  /**
   * @brief The HeadTrajectory class provides ability to move the head of valkyrie. Current implementation provides the ability to move the head to a set roll, pitch, and yaw.
   */
    HeadTrajectory(ros::NodeHandle nh);
    ~HeadTrajectory();

    /**
     * @brief moveHead Moves the robot head to the given roll, pitch, and yaw.
     * @param roll The roll in degrees.
     * @param pitch The pitch in degrees.
     * @param yaw The yaw in degrees.
     * @param time The time it takes to move to the given orientation. Default is 1.0
     */
    void moveHead(float roll, float pitch, float yaw, const float time = 1.0f);

    /**
     * @brief moveHead Moves the robot head by the given quaternion.
     * @param quaternion The quaternion representing the rotation of the head.
     * @param time The time it takes to move to the given orientation. Default is 1.0
     */
    void moveHead(const geometry_msgs::Quaternion &quaternion, const float time = 1.0f);


    void moveHead(const std::vector<std::vector<float> > &trajectory_points, const float time = 1.0f);
};

int HeadTrajectory::head_id = -1;

#endif // VAL_HEAD_NAVIGATION_H
