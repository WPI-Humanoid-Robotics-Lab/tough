#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

namespace perception_common{
class LeftImageInverter{
public:
  image_transport::ImageTransport it;
  image_transport::Publisher pub;
  image_transport::Subscriber sub;
  void getLeftImageCB(const sensor_msgs::ImageConstPtr&);
  void subscribeImage();
  LeftImageInverter(ros::NodeHandle& nh);
  virtual ~LeftImageInverter();
//  void publishLeftImage(const sensor_msgs::ImageConstPtr& msg);
};
}
