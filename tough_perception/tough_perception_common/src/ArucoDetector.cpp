#include <tough_perception_common/ArucoDetector.h>


namespace tough_perception
{
ArucoDetector::ArucoDetector(ros::NodeHandle& n)
: nh_(n), 
broadcastTF(false)
{
    aruco_sub = nh_.subscribe(aruco_sub_topic_,1, &ArucoDetector::fiducialCallback, this);
}

void ArucoDetector::fiducialCallback(const fiducial_msgs::FiducialTransformArrayConstPtr msg)
{
    std::lock_guard<std::mutex> lock(marker_mutex);
    detected_markers.clear();

    for (auto msg_: msg->transforms)
    {   
        detected_markers.push_back(msg_);
    }

}

geometry_msgs::PoseStampedPtr 
ArucoDetector::createPoseMessages(fiducial_msgs::FiducialTransform &object_)
{
    geometry_msgs::PoseStampedPtr p = boost::make_shared<geometry_msgs::PoseStamped>();
    
    p->header.stamp = ros::Time();
    p->header.frame_id = frame_id_;
    p->pose.position.x = object_.transform.translation.x;
    p->pose.position.y = object_.transform.translation.y;
    p->pose.position.z = object_.transform.translation.z;

    p->pose.orientation.x = object_.transform.rotation.x;
    p->pose.orientation.y = object_.transform.rotation.y;
    p->pose.orientation.z = object_.transform.rotation.z;
    p->pose.orientation.w = object_.transform.rotation.w;

    return p;

}

void ArucoDetector::setFrameId(std::string frame_id)
{
    frame_id_ = frame_id;
}

void ArucoDetector::setTopic(std::string topic)
{
    aruco_sub_topic_ = topic;
}

void ArucoDetector::getDetectedObjects(std::map<std::string, geometry_msgs::PoseStampedPtr> &detected_objects)
{
    ros::spinOnce();
    ros::Duration(1).sleep();

    std::lock_guard<std::mutex> lock(marker_mutex);
    for (auto marker_:detected_markers)
    {
        detected_objects[std::to_string(marker_.fiducial_id)] = createPoseMessages(marker_);
    }
}

}// tough_perception

