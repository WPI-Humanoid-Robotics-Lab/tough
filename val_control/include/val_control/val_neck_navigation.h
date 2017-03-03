#ifndef VAL_NECK_NAVIGATION_H
#define VAL_NECK_NAVIGATION_H

#include <ros/ros.h>
#include <ihmc_msgs/NeckTrajectoryRosMessage.h>
#include <ihmc_msgs/OneDoFJointTrajectoryRosMessage.h>
#include <ihmc_msgs/TrajectoryPoint1DRosMessage.h>
#include <tf/tf.h>
#include <math.h>
#include <time.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

/**
 * @brief The NeckTrajectory class provides ability to move the neck of valkyrie. Current implementation provides joint level control.
 */
class NeckTrajectory {

private:
    static int neck_id;
    const int NUM_NECK_JOINTS;
    ros::NodeHandle nh_;
    ros::Publisher neckTrajPublisher;

public:

  /**
   * @brief The NeckTrajectory class provides ability to move the head of valkyrie. Current implementation provides joint level control.
   */
    NeckTrajectory(ros::NodeHandle nh);
    ~NeckTrajectory();

    /**
     * @brief getNumNeckJoints Gives back the number of neck joints for Valkyrie R5
     * @return The number of neck joints.
     */
    int getNumNeckJoints() const;

    void moveNeckJoints(std::vector<float> &neck_data, const float time);

};

int NeckTrajectory::neck_id = -1;

#endif // VAL_NECK_NAVIGATION_H
