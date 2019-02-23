#ifndef ARUCODETECTOR_H_
#define ARUCODETECTOR_H_

#include <tough_perception_common/global.h>
#include <fiducial_msgs/FiducialTransform.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>

namespace tough_perception
{

class ArucoDetector
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber aruco_sub;
    bool broadcastTF;
    std::string frame_id_;
    std::string aruco_sub_topic_;
    std::vector<fiducial_msgs::FiducialTransform> detected_markers;

    std::mutex marker_mutex;

    void fiducialCallback(const fiducial_msgs::FiducialTransformArrayConstPtr msg);
    geometry_msgs::PoseStampedPtr createPoseMessages(fiducial_msgs::FiducialTransform &object_);
    void setFrameId(std::string frame_id);
    void setTopic(std::string topic);

public:
    ArucoDetector(ros::NodeHandle& n);
    void getDetectedObjects(std::map<std::string, geometry_msgs::PoseStampedPtr> &detected_objects);
    
};

} // tough_perception

#endif