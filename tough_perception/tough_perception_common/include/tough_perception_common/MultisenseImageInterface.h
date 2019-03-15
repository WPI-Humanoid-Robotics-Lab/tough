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
#include <mutex>
#include <Eigen/Dense>

namespace tough_perception
{

class MultisenseCameraModel
{
public:
  int width;
  int height;
  double fx;
  double fy;
  double cx;
  double cy;
  std::string distortion_model = "";
  Eigen::Matrix3d K;
  Eigen::MatrixXd P;
};

class MultisenseImageInterface
{
private:
  DISALLOW_COPY_AND_ASSIGN(MultisenseImageInterface);

  sensor_msgs::ImageConstPtr img_ = nullptr;
  sensor_msgs::ImageConstPtr depth_ = nullptr;
  sensor_msgs::ImageConstPtr cost_ = nullptr;
  stereo_msgs::DisparityImageConstPtr disparity_ = nullptr;
  sensor_msgs::ImageConstPtr disparity_sensor_msg_ = nullptr;
  sensor_msgs::CameraInfoConstPtr camera_info_ = nullptr;
  cv_bridge::CvImagePtr cv_ptr_;

  std::mutex image_mutex;
  std::mutex depth_mutex;
  std::mutex cost_mutex;
  std::mutex disparity_mutex;
  std::mutex disparity_sensor_msg_mutex;
  std::mutex camera_info_mutex;

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
  // std::string disp_topic_ = PERCEPTION_COMMON_NAMES::MULTISENSE_LEFT_DISPARITY_TOPIC;
  // std::string depth_topic_ = PERCEPTION_COMMON_NAMES::MULTISENSE_LEFT_DEPTH_TOPIC;
  // std::string depth_cost_topic_ = PERCEPTION_COMMON_NAMES::MULTISENSE_CONTROL_FPS_TOPIC;
  std::string multisense_topic_ = PERCEPTION_COMMON_NAMES::MULTISENSE_RAW_CAM_CONFIG_TOPIC;

  std::string image_topic_ = "/multisense/left/image_rect_color";
  std::string disp_topic_ = "/multisense/left/disparity_image";
  std::string disp_sensor_msg_topic_ = "/multisense/left/disparity";
  std::string depth_topic_ = "/multisense/depth";
  std::string cost_topic_ = "/multisense/left/cost"; // sensor_msg/Image
  std::string camera_info_topic = "/multisense/left/image_rect_color/camera_info";

  image_transport::ImageTransport it_;
  std::string transport_hint_ = "compressed";

  image_transport::Subscriber cam_sub_;
  image_transport::Subscriber cam_sub_depth_;
  image_transport::Subscriber cam_sub_cost_;
  image_transport::Subscriber cam_sub_disparity_sensor_msg_;

  ros::Subscriber cam_sub_disparity_;
  ros::Subscriber camera_info_sub_;

  static MultisenseImageInterface *current_object_;
  MultisenseImageInterface(ros::NodeHandle nh);

  void imageCB(const sensor_msgs::ImageConstPtr &img);
  void depthCB(const sensor_msgs::ImageConstPtr &img);
  void costCB(const sensor_msgs::ImageConstPtr &img);
  void disparityCB(const stereo_msgs::DisparityImageConstPtr &disp);
  void disparitySensorMsgCB(const sensor_msgs::ImageConstPtr &disp);
  void camera_infoCB(const sensor_msgs::CameraInfoConstPtr camera_info);

  // helper functions
  bool processDisparity(const sensor_msgs::Image &disp, cv::Mat &disp_img);
  bool processImage(const sensor_msgs::ImageConstPtr &in,
                    cv::Mat &out,
                    int image_encoding,
                    std::string out_encoding,
                    std::mutex &resource_mutex);

public:
  static MultisenseImageInterface *
  getMultisenseImageInterface(ros::NodeHandle nh);
  ~MultisenseImageInterface();

  bool getImage(cv::Mat &img);
  bool getDisparity(cv::Mat &disp_img, bool from_stereo_msg = false);
  bool getDepthImage(cv::Mat &depth_img);
  bool getCostImage(cv::Mat &cost_img);
  bool getCameraInfo(MultisenseCameraModel &pinhole_model);
  int getHeight();
  int getWidth();
};

typedef MultisenseImageInterface *MultisenseImageInterfacePtr;

} // namespace tough_perception

#endif