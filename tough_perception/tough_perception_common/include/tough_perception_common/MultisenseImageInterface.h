#ifndef MULTISENSEIMAGEINTERFACE_H_
#define MULTISENSEIMAGEINTERFACE_H_

/*** INCLUDE FILES ***/
#include <tough_perception_common/global.h>
#include <tough_perception_common/perception_common_names.h>
#include <image_transport/image_transport.h>
#include <multisense_ros/RawCamConfig.h>
#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <image_transport/subscriber_filter.h>
#include <memory>

namespace tough_perception
{

class MultisenseImageInterface
{
private:
  DISALLOW_COPY_AND_ASSIGN(MultisenseImageInterface);

  // cv::Mat image_;
  // cv::Mat disparity_;
  // cv::Mat depth_;
  // cv::Mat cost_;

  sensor_msgs::ImageConstPtr img_ = nullptr;
  sensor_msgs::ImageConstPtr depth_ = nullptr;
  sensor_msgs::CameraInfoConstPtr camera_info_;
  cv_bridge::CvImagePtr cv_ptr_;

  struct
  {
    cv::Mat camera_;
    cv::Mat_<double> Q_matrix_;
    int height_;
    int width_;
    float fps_;
    float gain_;
    float exposure_;
    float baselength_;
  } settings;

  ros::NodeHandle nh_;
  ros::AsyncSpinner spinner;

  // std::string image_topic_ = PERCEPTION_COMMON_NAMES::MULTISENSE_LEFT_IMAGE_COLOR_TOPIC;
  std::string image_topic_ = "/multisense/left/image_rect_color";
  // std::string disp_topic_ = PERCEPTION_COMMON_NAMES::MULTISENSE_LEFT_DISPARITY_TOPIC;
  std::string disp_topic_ = "/multisense/left/disparity";
  // std::string depth_topic_ = PERCEPTION_COMMON_NAMES::MULTISENSE_LEFT_DEPTH_TOPIC;
  std::string depth_topic_ = "/multisense/depth";
  // std::string depth_cost_topic_ = PERCEPTION_COMMON_NAMES::MULTISENSE_CONTROL_FPS_TOPIC;
  std::string depth_cost_topic_ = "/multisense/left/cost"; // sensor_msg/Image
  std::string multisense_topic_ = PERCEPTION_COMMON_NAMES::MULTISENSE_RAW_CAM_CONFIG_TOPIC;
  std::string camera_info_topic = "/multisense/left/image_rect_color/camera_info";

  image_transport::ImageTransport it_;
  std::string transport_hint_ = "compressed";

  image_transport::Subscriber cam_sub_;
  image_transport::Subscriber cam_sub_depth_;
  ros::Subscriber camera_info_sub_;

  static MultisenseImageInterface *current_object_;
  MultisenseImageInterface(ros::NodeHandle nh);

  void imageCB(const sensor_msgs::ImageConstPtr &img);
  void depthCB(const sensor_msgs::ImageConstPtr &img);
  void camera_infoCB(const sensor_msgs::CameraInfoConstPtr camera_info);

public:
  static MultisenseImageInterface *
  getMultisenseImageInterface(ros::NodeHandle nh);
  ~MultisenseImageInterface();

  bool getImage(cv::Mat &img);
  bool getDisparity(cv::Mat &disp_img);
  bool getDepthImage(cv::Mat &depth_img);
  int getHeight();
  int getWidth();
};

typedef MultisenseImageInterface *MultisenseImageInterfacePtr;

} // namespace tough_perception

#endif