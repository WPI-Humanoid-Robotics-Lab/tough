#ifndef FINISHBOX_DETECTOR_H
#define FINISHBOX_DETECTOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>

class finishbox_detector{
public:
    finishbox_detector(ros::NodeHandle nh);
    ~finishbox_detector();

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber imageSub_;
    image_transport::Publisher  imagePub_;

    void imageCallback(sensor_msgs::ImageConstPtr &in_img);
    void detectFinishBox(sensor_msgs::Image &img);

};

#endif // FINISHBOX_DETECTOR_H
