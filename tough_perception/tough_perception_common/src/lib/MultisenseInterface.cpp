#include <tough_perception_common/MultisenseInterface.h>
#include <tough_perception_common/perception_common_names.h>
#include <tough_perception_common/PerceptionHelper.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <opencv2/core/eigen.hpp>

namespace tough_perception
{
using namespace std;

// MultisenseCameraModel
MultisenseCameraModel::MultisenseCameraModel()
{
  tx = -0.07;
}

void MultisenseCameraModel::computeQ()
{
  this->Q << this->fy * this->tx, 0.0, 0.0, -this->fy * this->cx * this->tx, // row 1
      0.0, this->fx * this->tx, 0.0, -this->fx * this->cy * this->tx,        //
      0.0, 0.0, 0.0, this->fx * this->fy * this->tx,                         //
      0.0, 0.0, -this->fy, 0.0;
}

void MultisenseCameraModel::printCameraConfig()
{
  std::cout << "-" << std::endl
            << "[Height]" << height << std::endl
            << "[width]" << width << std::endl
            << "[fx]" << fx << std::endl
            << "[fy]" << fy << std::endl
            << "[cx]" << cx << std::endl
            << "[cy]" << cy << std::endl
            << "[K]\n"
            << K << std::endl
            << "[P]\n"
            << P << std::endl
            << "[Q]\n"
            << Q << std::endl
            << "[distortion_model] " << distortion_model << std::endl
            << "-" << std::endl;
  return;
}

MultisenseInterfacePtr MultisenseInterface::current_object_ = nullptr;

MultisenseInterfacePtr MultisenseInterface::getMultisenseInterface(ros::NodeHandle nh,
                                                                   bool is_simulation)
{
  if (MultisenseInterface::current_object_ == nullptr)
  {
    static MultisenseInterface obj(nh, is_simulation);
    current_object_ = &obj;
  }
  return current_object_;
}
/*
 *@ToDO: add sync callbacks
 *@ToDO: create a class for MultisenseImage Exceptions(is it reall needed? not sure)
 */
void MultisenseInterface::imageCB(const sensor_msgs::ImageConstPtr &img)
{
  ROS_INFO_ONCE("Listening to %s", cam_sub_.getTopic().c_str());
  // did not use mutex lock_guard as it is a blocking call which
  // is not good for callbacks
  if (image_mutex.try_lock())
  {
    img_ = img;
    image_mutex.unlock();
  }
}

void MultisenseInterface::depthCB(const sensor_msgs::ImageConstPtr &img)
{
  ROS_INFO_ONCE("Listening to %s", cam_sub_depth_.getTopic().c_str());
  if (depth_mutex.try_lock())
  {
    depth_ = img;
    depth_mutex.unlock();
  }
}

void MultisenseInterface::costCB(const sensor_msgs::ImageConstPtr &img)
{
  ROS_INFO_ONCE("Listening to %s", cam_sub_depth_.getTopic().c_str());
  if (cost_mutex.try_lock())
  {
    cost_ = img;
    cost_mutex.unlock();
  }
}

void MultisenseInterface::disparityCB(const stereo_msgs::DisparityImageConstPtr &disp)
{
  ROS_INFO_ONCE("Listening to %s", cam_sub_disparity_.getTopic().c_str());
  if (disparity_mutex.try_lock())
  {
    disparity_ = disp;
    disparity_mutex.unlock();
  }
}
void MultisenseInterface::disparitySensorMsgCB(const sensor_msgs::ImageConstPtr &disp)
{
  ROS_INFO_ONCE("Listening to %s", cam_sub_disparity_sensor_msg_.getTopic().c_str());
  if (disparity_sensor_msg_mutex.try_lock())
  {
    disparity_sensor_msg_ = disp;
    disparity_sensor_msg_mutex.unlock();
  }
}

void MultisenseInterface::camera_infoCB(const sensor_msgs::CameraInfoConstPtr camera_info)
{
  ROS_INFO_ONCE("Listening to %s", camera_info_sub_.getTopic().c_str());
  if (camera_info_mutex.try_lock())
  {
    camera_info_ = camera_info;
    camera_info_mutex.unlock();
  }
}

void MultisenseInterface::assembled_pcCB(const sensor_msgs::PointCloud2Ptr &assembled_pc)
{
  ROS_INFO_ONCE("Listening to %s", assembled_pc_sub_.getTopic().c_str());
  if (assembled_pc_mutex.try_lock())
  {
    assembled_pc_msg = assembled_pc;
    assembled_pc_mutex.unlock();
  }
}

void MultisenseInterface::assemblerStatusCB(const std_msgs::Int8ConstPtr &status)
{
  assembler_status_ = status;
}

bool MultisenseInterface::start()
{
  ROS_INFO("Starting MultisenseInterface");
  cam_sub_ = it_.subscribe(image_topic_, 1, &MultisenseInterface::imageCB, this);

  cam_sub_depth_ = it_.subscribe(depth_topic_, 1, &MultisenseInterface::depthCB, this);

  cam_sub_cost_ = it_.subscribe(cost_topic_, 1, &MultisenseInterface::costCB, this);

  cam_sub_disparity_sensor_msg_ =
      it_.subscribe(disp_sensor_msg_topic_, 1, &MultisenseInterface::disparitySensorMsgCB, this);

  cam_sub_disparity_ = nh_.subscribe(disp_topic_, 1, &MultisenseInterface::disparityCB, this);

  camera_info_sub_ = nh_.subscribe(camera_info_topic, 1, &MultisenseInterface::camera_infoCB, this);

  assembled_pc_sub_ = nh_.subscribe(assembled_pc_topic_, 1, &MultisenseInterface::assembled_pcCB, this);

  assembler_status_sub_ = nh_.subscribe(assembler_status_topic_, 1, &MultisenseInterface::assemblerStatusCB, this);

  multisense_motor_speed_pub_ = nh_.advertise<std_msgs::Float64>(multisense_motor_topic_, 1);
  reset_assembler_pub_ = nh_.advertise<std_msgs::Empty>(assembler_reset_topic_, 1);
  pause_assembler_pub_ = nh_.advertise<std_msgs::Bool>(assembler_pause_topic_, 1);

  ros::Duration(1).sleep();
  ros::spinOnce();
}

MultisenseInterface::MultisenseInterface(ros::NodeHandle nh, bool is_simulation)
    : nh_(nh), it_(nh_), is_simulation_(is_simulation)
{
  start();
  ros::Duration(0.5).sleep();
}

bool MultisenseInterface::processDisparity(const sensor_msgs::Image &disp, cv::Mat &disp_img)
{
  if (disp.data.size() == 0)
    return false;
  try
  {
    int source_type = cv_bridge::getCvType(disp.encoding);
    uint8_t depth =
        sensor_msgs::image_encodings::bitDepth(disp.encoding); // the size of the disparity data can be 16 or 32
    if (depth == 32)
    {
      cv::Mat_<float> disparity(disp.height, disp.width,
                                const_cast<float *>(reinterpret_cast<const float *>(&disp.data[0])));
      disp_img = disparity.clone();
      return true;
    }
    else if (depth == 16)
    {
      cv::Mat_<uint16_t> disparityOrigP(disp.height, disp.width,
                                        const_cast<uint16_t *>(reinterpret_cast<const uint16_t *>(&disp.data[0])));
      cv::Mat_<float> disparity(disp.height, disp.width);
      disparity = disparityOrigP / 16.0f;
      disp_img = disparity.clone();
      return true;
    }
    else
    {
      ROS_WARN("disparity depth not recognized");
      return false;
    }
  }
  catch (std::exception ex)
  {
    ROS_ERROR_STREAM("Exception: " << ex.what());
    return false;
  }
}

bool MultisenseInterface::processImage(const sensor_msgs::ImageConstPtr &in, cv::Mat &out, int image_encoding,
                                       std::string out_encoding, std::mutex &resource_mutex)
{
  std::lock_guard<std::mutex> guard(resource_mutex);
  if (in == nullptr)
    return false;
  if (in->data.size() == 0)
    return false;
  try
  {
    if (cv_bridge::getCvType(in->encoding) != image_encoding)
    {
      ROS_ERROR_STREAM("Unsupported image encoding :" << cv_bridge::getCvType(in->encoding));
      return false;
    }
    cv_ptr_ = cv_bridge::toCvCopy(in, out_encoding);
    out = cv_ptr_->image.clone();
    return true;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return false;
  }
}

bool MultisenseInterface::getDisparity(cv::Mat &disp_img, bool from_stereo_msg)
{
  bool status;
  if (from_stereo_msg)
  {
    ROS_DEBUG("Fetching disparity from stereo_msg");
    std::lock_guard<std::mutex> guard(disparity_mutex);
    if (disparity_ == nullptr)
      return false;
    status = processDisparity(disparity_->image, disp_img);
  }
  else
  {
    ROS_DEBUG("Fetching disparity from sensor_msg");
    std::lock_guard<std::mutex> guard(disparity_sensor_msg_mutex);
    if (disparity_sensor_msg_ == nullptr)
      return false;
    status = processDisparity(*disparity_sensor_msg_, disp_img);
  }
  return status;
}

bool MultisenseInterface::getImage(cv::Mat &img)
{
  return processImage(img_, img, CV_8UC3, sensor_msgs::image_encodings::BGR8, image_mutex);
}

bool MultisenseInterface::getDepthImage(cv::Mat &depth_img)
{
  return processImage(depth_, depth_img, CV_32F, sensor_msgs::image_encodings::TYPE_32FC1, depth_mutex);
}
bool MultisenseInterface::getCostImage(cv::Mat &cost_img)
{
  return processImage(cost_, cost_img, CV_8U, sensor_msgs::image_encodings::MONO8, cost_mutex);
}
int MultisenseInterface::getHeight()
{
  if (camera_info_ == nullptr)
  {
    ROS_ERROR_STREAM("Could not listen to " << camera_info_sub_.getTopic());
    return -1;
  }
  return camera_info_->height;
}
int MultisenseInterface::getWidth()
{
  if (camera_info_ == nullptr)
  {
    ROS_ERROR_STREAM("Could not listen to " << camera_info_sub_.getTopic());
    return -1;
  }
  return camera_info_->width;
}

bool MultisenseInterface::getCameraInfo(MultisenseCameraModel &pinhole_model)
{
  if (camera_info_ != nullptr)
  {
    std::lock_guard<std::mutex> guard(camera_info_mutex);
    pinhole_model.width = camera_info_->width;
    pinhole_model.height = camera_info_->height;
    pinhole_model.K = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(camera_info_->K.data());
    pinhole_model.P = Eigen::Matrix<double, 3, 4, Eigen::RowMajor>(camera_info_->P.data());
    pinhole_model.fx = pinhole_model.K(0, 0);
    pinhole_model.fy = pinhole_model.K(1, 1);
    pinhole_model.cx = pinhole_model.K(0, 2);
    pinhole_model.cy = pinhole_model.K(1, 2);
    pinhole_model.distortion_model = camera_info_->distortion_model;
    pinhole_model.computeQ();
    return true;
  }
  return false;
}

bool MultisenseInterface::getStereoData(cv::Mat &dispImage, cv::Mat &colorImage,
                                        tough_perception::StereoPointCloudColor::Ptr &pointcloud)
{
  bool status = getImage(colorImage);
  status = status && getDisparity(dispImage, true);

  if (!status)
    return false;

  MultisenseCameraModel cam_model;
  this->getCameraInfo(cam_model);

  generateOrganizedRGBDCloud(dispImage, colorImage, cam_model.Q, pointcloud);

  return true;
}

bool MultisenseInterface::getLaserPointCloud(PointCloud::Ptr &pc_)
{
  if (!pc_) // handle uninitialized pointcloud pointers
    pc_ = PointCloud::Ptr(new tough_perception::PointCloud());

  std::lock_guard<std::mutex> guard(assembled_pc_mutex);
  convertROStoPCL<PointCloud::Ptr>(assembled_pc_msg, pc_);
  return pc_->empty() ? false : true;
}

LASER_ASSEMBLER_STATUS MultisenseInterface::getAssemblerStatus()
{
  return static_cast<LASER_ASSEMBLER_STATUS>(assembler_status_->data);
}

void MultisenseInterface::setAssemblerStatus(LASER_ASSEMBLER_STATUS status)
{
  if (status == LASER_ASSEMBLER_STATUS::RESET)
  {
    std_msgs::Empty emptyMsg;
    reset_assembler_pub_.publish(emptyMsg);
  }
  else if (status == LASER_ASSEMBLER_STATUS::PAUSE)
    pause_assembler_pub_.publish(true);
  else if (status == LASER_ASSEMBLER_STATUS::ACTIVE)
    pause_assembler_pub_.publish(false);
}

bool MultisenseInterface::isSensorActive()
{
  return isActive(img_) || isActive(depth_) || isActive(cost_) || isActive(disparity_sensor_msg_) ||
         isActive(disparity_) || isActive(assembled_pc_msg);
}

bool MultisenseInterface::shutdown()
{
  if (isSensorActive())
  {
    ROS_INFO("Shutting down MultisenseInterface");
    cam_sub_.shutdown();
    cam_sub_depth_.shutdown();
    cam_sub_cost_.shutdown();
    cam_sub_disparity_.shutdown();
    cam_sub_disparity_sensor_msg_.shutdown();
    camera_info_sub_.shutdown();
    assembled_pc_sub_.shutdown();

    resetMsg(img_);
    resetMsg(depth_);
    resetMsg(cost_);
    resetMsg(disparity_);
    resetMsg(disparity_sensor_msg_);
    resetMsg(assembled_pc_msg);

    ros::Duration(0.5).sleep();
  }
}
void MultisenseInterface::setSpindleSpeed(double speed)
{
  multisense_motor_speed_pub_.publish(speed);
}

MultisenseInterface::~MultisenseInterface()
{
  //   shutdown();
}

bool isActive(const sensor_msgs::ImageConstPtr &some_msg)
{
  return some_msg != nullptr;
}

bool isActive(const stereo_msgs::DisparityImageConstPtr &some_msg)
{
  return some_msg != nullptr;
}

bool isActive(const sensor_msgs::PointCloud2Ptr &some_msg)
{
  return some_msg != nullptr;
}

void resetMsg(sensor_msgs::ImageConstPtr &some_msg)
{
  some_msg = nullptr;
}

void resetMsg(stereo_msgs::DisparityImageConstPtr &some_msg)
{
  some_msg = nullptr;
}

void resetMsg(sensor_msgs::PointCloud2Ptr &some_msg)
{
  some_msg = nullptr;
}

} // namespace tough_perception