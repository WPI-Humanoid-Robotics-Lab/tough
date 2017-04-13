#ifndef FINISHBOX_DETECTOR_H
#define FINISHBOX_DETECTOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>

class finishbox_detector{
public:
    finishbox_detector(ros::NodeHandle &nh);
    ~finishbox_detector();

private:

    ros::Subscriber imageSub_;
    ros::Publisher  imagePub_;

    void imageCallback(const sensor_msgs::Image &in_img);
    void detectFinishBox(sensor_msgs::Image &img);

};

#endif // FINISHBOX_DETECTOR_H
