/**
 * @file MultisenseInterface.h
 * @author Ameya Wagh (aywagh@wpi.edu)
 * @brief Helper functions for robot perception
 * @version 0.1
 * @date 2019-05-30
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef MULTISENSEINTERFACE_H_
#define MULTISENSEINTERFACE_H_

/*** INCLUDE FILES TOUGH ***/
#include <tough_perception_common/global.h>
#include <tough_perception_common/perception_common_names.h>
#include <tough_perception_common/PerceptionHelper.h>

/*** INCLUDE FILES ROS ***/
#include <image_transport/image_transport.h>
#include <multisense_ros/RawCamConfig.h>
#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <image_transport/subscriber_filter.h>

/*** INCLUDE FILES ROS ***/
#include <memory>
#include <mutex>
#include <Eigen/Dense>

typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> exactTimePolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> approxTimePolicy;

namespace tough_perception
{

bool isActive(const sensor_msgs::ImageConstPtr &some_msg);
bool isActive(const stereo_msgs::DisparityImageConstPtr &some_msg);
bool isActive(const sensor_msgs::PointCloud2Ptr &some_msg);

void resetMsg(sensor_msgs::ImageConstPtr &some_msg);
void resetMsg(stereo_msgs::DisparityImageConstPtr &some_msg);
void resetMsg(sensor_msgs::PointCloud2Ptr &some_msg);

enum class LASER_ASSEMBLER_STATUS
{
  RESET = 0,
  PAUSE,
  ACTIVE
};

class MultisenseInterface;
/**
 * @brief MultisenseCamerModel class stores/provides camera information. 
 * 
 */
class MultisenseCameraModel
{
public:
  /**
 * @brief Construct a new Multisense Camera Model object
 *  for a pinhole camera
 */
  MultisenseCameraModel();
  /**
   * @brief print the camera config to standard output
   * 
   */
  void printCameraConfig();
  friend MultisenseInterface;

  int width;                         // image width in pixels
  int height;                        // image height in pixels
  double fx;                         // focal length in x
  double fy;                         // focal length in y
  double cx;                         // center offset in x
  double cy;                         // center offset in y
  double tx;                         // negative baseline
  std::string distortion_model = ""; // camera distortion model
  Eigen::Matrix3d K;                 // K is the camera intrinsic matrix
  Eigen::MatrixXd P;                 // P is camera distortion matrix
  Eigen::Matrix4d Q;                 // Q is transformation matrix from pixel to world

private:
  /**
   * @brief Computes Q matrix
   * 
   */
  void computeQ();
};

class MultisenseInterface
{
private:
  DISALLOW_COPY_AND_ASSIGN(MultisenseInterface);

  // ros message place holders
  sensor_msgs::ImageConstPtr img_ = nullptr;
  sensor_msgs::ImageConstPtr depth_ = nullptr;
  sensor_msgs::ImageConstPtr cost_ = nullptr;
  stereo_msgs::DisparityImageConstPtr disparity_ = nullptr;
  sensor_msgs::ImageConstPtr disparity_sensor_msg_ = nullptr;
  sensor_msgs::CameraInfoConstPtr camera_info_ = nullptr;
  sensor_msgs::PointCloud2Ptr assembled_pc_msg = nullptr;
  std_msgs::Int8ConstPtr assembler_status_ = nullptr;
  cv_bridge::CvImagePtr cv_ptr_;

  // mutex locks
  std::mutex image_mutex;
  std::mutex depth_mutex;
  std::mutex cost_mutex;
  std::mutex disparity_mutex;
  std::mutex disparity_sensor_msg_mutex;
  std::mutex camera_info_mutex;
  std::mutex assembled_pc_mutex;

  bool is_simulation_;

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

  // ros topics
  // @todo: move the hardcoded topic names to perception_common_names
#ifdef GAZEBO_SIMULATION
  std::string image_topic_ = PERCEPTION_COMMON_NAMES::MULTISENSE_LEFT_IMAGE_COLOR_COMPRESSED_TOPIC;
  std::string disp_topic_ = PERCEPTION_COMMON_NAMES::MULTISENSE_LEFT_DISPARITY_TOPIC; // stereo message
  std::string depth_topic_ = PERCEPTION_COMMON_NAMES::MULTISENSE_LEFT_DEPTH_TOPIC;    // image
  std::string camera_info_topic = PERCEPTION_COMMON_NAMES::MULTISENSE_LEFT_CAMERA_INFO_TOPIC;
#else
  std::string image_topic_ = "/multisense/left/image_rect_color";
  std::string disp_topic_ = "/multisense/left/disparity_image";
  std::string depth_topic_ = "/multisense/depth";
  std::string camera_info_topic = "/multisense/left/image_rect_color/camera_info";
#endif
  std::string multisense_topic_ = PERCEPTION_COMMON_NAMES::MULTISENSE_RAW_CAM_CONFIG_TOPIC;
  std::string depth_cost_topic_ = PERCEPTION_COMMON_NAMES::MULTISENSE_DEPTH_COST_TOPIC; // image
  std::string disp_sensor_msg_topic_ = "/multisense/left/disparity";
  std::string cost_topic_ = "/multisense/left/cost"; // sensor_msg/Image

  std::string multisense_motor_topic_ = PERCEPTION_COMMON_NAMES::MULTISENSE_CONTROL_MOTOR_TOPIC;
  // std::string assembled_pc_topic_ = PERCEPTION_COMMON_NAMES::ASSEMBLED_LASER_CLOUD_TOPIC;
  std::string assembled_pc_topic_ = "/atlas/assembled_cloud2";
  std::string assembler_status_topic_ = "/atlas/assembler_status";
  std::string assembler_reset_topic_ = "/atlas/reset_pointcloud";
  std::string assembler_pause_topic_ = "/atlas/pause_pointcloud";

  image_transport::ImageTransport it_;
  std::string transport_hint_ = "compressed";

  // ros subscribers
  image_transport::Subscriber cam_sub_;
  image_transport::Subscriber cam_sub_depth_;
  image_transport::Subscriber cam_sub_cost_;
  image_transport::Subscriber cam_sub_disparity_sensor_msg_;

  ros::Subscriber cam_sub_disparity_;
  ros::Subscriber camera_info_sub_;

  ros::Subscriber assembled_pc_sub_;
  ros::Subscriber assembler_status_sub_;

  ros::Publisher multisense_motor_speed_pub_;
  ros::Publisher reset_assembler_pub_;
  ros::Publisher pause_assembler_pub_;

  std::shared_ptr<message_filters::Synchronizer<exactTimePolicy>> sync_img_depth_exact;
  std::shared_ptr<message_filters::Synchronizer<exactTimePolicy>> sync_img_depth_approx;

  static MultisenseInterface *current_object_;
  MultisenseInterface(ros::NodeHandle nh, bool is_simulation);

  // callbacks
  void imageCB(const sensor_msgs::ImageConstPtr &img);
  void depthCB(const sensor_msgs::ImageConstPtr &img);
  void costCB(const sensor_msgs::ImageConstPtr &img);
  void disparityCB(const stereo_msgs::DisparityImageConstPtr &disp);
  void disparitySensorMsgCB(const sensor_msgs::ImageConstPtr &disp);
  void camera_infoCB(const sensor_msgs::CameraInfoConstPtr camera_info);
  void assembled_pcCB(const sensor_msgs::PointCloud2Ptr &assembled_pc);
  void assemblerStatusCB(const std_msgs::Int8ConstPtr &status);

  // helper functions
  bool processDisparity(const sensor_msgs::Image &disp, cv::Mat &disp_img);
  bool processImage(const sensor_msgs::ImageConstPtr &in, cv::Mat &out, int image_encoding, std::string out_encoding,
                    std::mutex &resource_mutex);

public:
  /**
 * @brief Get the Multisense Image Interface object
 * 
 * @param nh Nodehandle to the running ros node
 * @param is_simulation 
 * @return MultisenseInterface* 
 */
  static MultisenseInterface *getMultisenseInterface(ros::NodeHandle nh, bool is_simulation = false);
  ~MultisenseInterface();

  /**
 * @brief Get RGB image from multisense camera
 * 
 * @param img reference to be updated
 * @return true when image is received 
 * @return false when image cannot be fetched
 */
  bool getImage(cv::Mat &img);

  /**
   * @brief Get Disparity image from multisense camera
   * 
   * @param disp_img reference to be updated
   * @param from_stereo_msg Set this to true for Gazebo and false otherwise 
   * @return true when image is received
   * @return false when image cannot be fetched
   */
  bool getDisparity(cv::Mat &disp_img, bool from_stereo_msg = false);

  /**
   * @brief Get the Depth Image from multisense camera
   * 
   * @param depth_img reference to be updated
   * @return true when image is received 
   * @return false when image cannot be fetched
   */
  bool getDepthImage(cv::Mat &depth_img);

  /**
   * @brief Get the Cost Image from multisense camera
   * 
   * @param cost_img 
   * @return true 
   * @return false 
   */
  bool getCostImage(cv::Mat &cost_img);

  /**
   * @brief Get the Camera Info from multisense camera
   * 
   * @param pinhole_model 
   * @return true 
   * @return false 
   */
  bool getCameraInfo(MultisenseCameraModel &pinhole_model);

  /**
   * @brief Provides disparity and rgb image along with organized pointcloud based off of those images
   * 
   * @param dispImage disparity image
   * @param colorImage color image
   * @param pointcloud organized RGBXYZ pointcloud
   * @return true when all the references can be updated
   * @return false when either of the three fails
   */
  bool getStereoData(cv::Mat &dispImage, cv::Mat &colorImage, tough_perception::StereoPointCloudColor::Ptr &pointcloud);

  /**
   * @brief Get the Laser Point Cloud from assembled point cloud
   * 
   * @param pc 
   * @return true 
   * @return false 
   */
  bool getLaserPointCloud(PointCloud::Ptr &pc_);

  /**
   * @brief Get the Assembler Status 
   * 
   * @return LASER_ASSEMBLER_STATUS 
   */
  LASER_ASSEMBLER_STATUS getAssemblerStatus();

  /**
   * @brief Get the Height of image
   * 
   * @return int 
   */
  int getHeight();

  /**
   * @brief Get the Width of image
   * 
   * @return int 
   */
  int getWidth();

  /**
   * @brief Check if Multisense sensor is active
   * 
   * @return true 
   * @return false 
   */
  bool isSensorActive();

  /**
   * @brief Set the Spindle Speed to given value in Gazebo
   * 
   * @param speed 
   */
  void setSpindleSpeed(double speed = 2.0f);

  /**
   * @brief Set the Assembler Status of periodic snapshotter node
   * 
   * @param LASER_ASSEMBLER_STATUS status 
   */
  void setAssemblerStatus(LASER_ASSEMBLER_STATUS status);

  /**
   * @brief Start subscribers and publishers to talk with Multisense sensor
   * 
   * @return true 
   * @return false 
   */
  bool start();

  /**
   * @brief Stop subscribers listening to Multisense sensor
   * 
   * @return true 
   * @return false 
   */
  bool shutdown();
};

/**
 * @brief Handy typedef for MultisenseInterface pointer.
 * @todo: This should be a boost shared pointer
 */
typedef MultisenseInterface *MultisenseInterfacePtr;
} // namespace tough_perception

#endif