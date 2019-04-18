#include <tough_perception_common/MultisenseImageInterface.h>
#include <tough_perception_common/perception_common_names.h>
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
  this->Q << this->fy * this->tx, 0.0, 0.0, -this->fy * this->cx * this->tx,  // row 1
      0.0, this->fx * this->tx, 0.0, -this->fx * this->cy * this->tx,         //
      0.0, 0.0, 0.0, this->fx * this->fy * this->tx,                          //
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

MultisenseImageInterfacePtr MultisenseImageInterface::current_object_ = nullptr;

MultisenseImageInterfacePtr MultisenseImageInterface::getMultisenseImageInterface(ros::NodeHandle nh,
                                                                                  bool is_simulation)
{
  if (MultisenseImageInterface::current_object_ == nullptr)
  {
    static MultisenseImageInterface obj(nh, is_simulation);
    current_object_ = &obj;
  }
  return current_object_;
}
/*
 *@ToDO: add sync callbacks
 *@ToDO: create a class for MultisenseImage Exceptions(is it reall needed? not sure)
 */
void MultisenseImageInterface::imageCB(const sensor_msgs::ImageConstPtr& img)
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

void MultisenseImageInterface::depthCB(const sensor_msgs::ImageConstPtr& img)
{
  ROS_INFO_ONCE("Listening to %s", cam_sub_depth_.getTopic().c_str());
  if (depth_mutex.try_lock())
  {
    depth_ = img;
    depth_mutex.unlock();
  }
}

void MultisenseImageInterface::costCB(const sensor_msgs::ImageConstPtr& img)
{
  ROS_INFO_ONCE("Listening to %s", cam_sub_depth_.getTopic().c_str());
  if (cost_mutex.try_lock())
  {
    cost_ = img;
    cost_mutex.unlock();
  }
}

void MultisenseImageInterface::disparityCB(const stereo_msgs::DisparityImageConstPtr& disp)
{
  ROS_INFO_ONCE("Listening to %s", cam_sub_disparity_.getTopic().c_str());
  if (disparity_mutex.try_lock())
  {
    disparity_ = disp;
    disparity_mutex.unlock();
  }
}
void MultisenseImageInterface::disparitySensorMsgCB(const sensor_msgs::ImageConstPtr& disp)
{
  ROS_INFO_ONCE("Listening to %s", cam_sub_disparity_sensor_msg_.getTopic().c_str());
  if (disparity_sensor_msg_mutex.try_lock())
  {
    disparity_sensor_msg_ = disp;
    disparity_sensor_msg_mutex.unlock();
  }
}

void MultisenseImageInterface::camera_infoCB(const sensor_msgs::CameraInfoConstPtr camera_info)
{
  ROS_INFO_ONCE("Listening to %s", camera_info_sub_.getTopic().c_str());
  if (camera_info_mutex.try_lock())
  {
    camera_info_ = camera_info;
    camera_info_mutex.unlock();
  }
}

bool MultisenseImageInterface::start()
{
  ROS_INFO("Starting MultisenseImageInterface");
  cam_sub_ = it_.subscribe(image_topic_, 1, &MultisenseImageInterface::imageCB, this);

  cam_sub_depth_ = it_.subscribe(depth_topic_, 1, &MultisenseImageInterface::depthCB, this);

  cam_sub_cost_ = it_.subscribe(cost_topic_, 1, &MultisenseImageInterface::costCB, this);

  cam_sub_disparity_sensor_msg_ =
      it_.subscribe(disp_sensor_msg_topic_, 1, &MultisenseImageInterface::disparitySensorMsgCB, this);

  cam_sub_disparity_ = nh_.subscribe(disp_topic_, 1, &MultisenseImageInterface::disparityCB, this);

  camera_info_sub_ = nh_.subscribe(camera_info_topic, 1, &MultisenseImageInterface::camera_infoCB, this);

  multisense_motor_speed_pub_ = nh_.advertise<std_msgs::Float64>(multisense_motor_topic_, 1);

  ros::Duration(1).sleep();
  ros::spinOnce();
}

MultisenseImageInterface::MultisenseImageInterface(ros::NodeHandle nh, bool is_simulation)
  : nh_(nh), it_(nh_), is_simulation_(is_simulation)
{
  start();
  ros::Duration(0.5).sleep();
}

bool MultisenseImageInterface::processDisparity(const sensor_msgs::Image& disp, cv::Mat& disp_img)
{
  if (disp.data.size() == 0)
    return false;
  try
  {
    int source_type = cv_bridge::getCvType(disp.encoding);
    uint8_t depth =
        sensor_msgs::image_encodings::bitDepth(disp.encoding);  // the size of the disparity data can be 16 or 32
    if (depth == 32)
    {
      cv::Mat_<float> disparity(disp.height, disp.width,
                                const_cast<float*>(reinterpret_cast<const float*>(&disp.data[0])));
      disp_img = disparity.clone();
      return true;
    }
    else if (depth == 16)
    {
      cv::Mat_<uint16_t> disparityOrigP(disp.height, disp.width,
                                        const_cast<uint16_t*>(reinterpret_cast<const uint16_t*>(&disp.data[0])));
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

bool MultisenseImageInterface::processImage(const sensor_msgs::ImageConstPtr& in, cv::Mat& out, int image_encoding,
                                            std::string out_encoding, std::mutex& resource_mutex)
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
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return false;
  }
}

bool MultisenseImageInterface::getDisparity(cv::Mat& disp_img, bool from_stereo_msg)
{
  bool status;
  if (from_stereo_msg)
  {
    ROS_INFO("Fetching disparity from stereo_msg");
    std::lock_guard<std::mutex> guard(disparity_mutex);
    if (disparity_ == nullptr)
      return false;
    status = processDisparity(disparity_->image, disp_img);
  }
  else
  {
    ROS_INFO("Fetching disparity from sensor_msg");
    std::lock_guard<std::mutex> guard(disparity_sensor_msg_mutex);
    if (disparity_sensor_msg_ == nullptr)
      return false;
    status = processDisparity(*disparity_sensor_msg_, disp_img);
  }
  return status;
}

bool MultisenseImageInterface::getImage(cv::Mat& img)
{
  return processImage(img_, img, CV_8UC3, sensor_msgs::image_encodings::BGR8, image_mutex);
}

bool MultisenseImageInterface::getDepthImage(cv::Mat& depth_img)
{
  return processImage(depth_, depth_img, CV_32F, sensor_msgs::image_encodings::TYPE_32FC1, depth_mutex);
}
bool MultisenseImageInterface::getCostImage(cv::Mat& cost_img)
{
  return processImage(cost_, cost_img, CV_8U, sensor_msgs::image_encodings::MONO8, cost_mutex);
}
int MultisenseImageInterface::getHeight()
{
  if (camera_info_ == nullptr)
  {
    ROS_ERROR_STREAM("Could not listen to " << camera_info_sub_.getTopic());
    return -1;
  }
  return camera_info_->height;
}
int MultisenseImageInterface::getWidth()
{
  if (camera_info_ == nullptr)
  {
    ROS_ERROR_STREAM("Could not listen to " << camera_info_sub_.getTopic());
    return -1;
  }
  return camera_info_->width;
}

bool MultisenseImageInterface::getCameraInfo(MultisenseCameraModel& pinhole_model)
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

bool MultisenseImageInterface::isSensorActive()
{
  return isActive(img_) || isActive(depth_) || isActive(cost_) || isActive(disparity_sensor_msg_) ||
         isActive(disparity_);
}

bool MultisenseImageInterface::shutdown()
{
  if (isSensorActive())
  {
    ROS_INFO("Shutting down MultisenseImageInterface");
    cam_sub_.shutdown();
    cam_sub_depth_.shutdown();
    cam_sub_cost_.shutdown();
    cam_sub_disparity_.shutdown();
    cam_sub_disparity_sensor_msg_.shutdown();
    camera_info_sub_.shutdown();

    resetMsg(img_);
    resetMsg(depth_);
    resetMsg(cost_);
    resetMsg(disparity_);
    resetMsg(disparity_sensor_msg_);

    ros::Duration(0.5).sleep();
  }
}
void MultisenseImageInterface::setSpindleSpeed(double speed)
{
  multisense_motor_speed_pub_.publish(speed);
}

MultisenseImageInterface::~MultisenseImageInterface()
{
  //   shutdown();
}

bool isActive(const sensor_msgs::ImageConstPtr& some_msg)
{
  return some_msg != nullptr;
}

bool isActive(const stereo_msgs::DisparityImageConstPtr& some_msg)
{
  return some_msg != nullptr;
}

void resetMsg(sensor_msgs::ImageConstPtr& some_msg)
{
  some_msg = nullptr;
}

void resetMsg(stereo_msgs::DisparityImageConstPtr& some_msg)
{
  some_msg = nullptr;
}

void generateOrganizedRGBDCloud(const cv::Mat& dispImage, const cv::Mat& colorImage, const Eigen::Matrix4d Qmat,
                                tough_perception::StereoPointCloudColor::Ptr& cloud)
{
  int width = dispImage.cols;
  int height = dispImage.rows;

  cloud->resize(width * height);
  cloud->height = height;
  cloud->width = width;

  for (int u = 0; u < dispImage.rows; u++)
    for (int v = 0; v < dispImage.cols; v++)
    {
      const float& currentPixelDisp = dispImage.at<float>(cv::Point(v, u));
      if (currentPixelDisp == 0.0)
        continue;
      Eigen::Vector4d pixelCoordVec(v, u, currentPixelDisp, 1);
      pixelCoordVec = Qmat * pixelCoordVec;
      pixelCoordVec /= pixelCoordVec(3);

      tough_perception::StereoPointColor& pt = cloud->at(v, u);

      pt.x = pixelCoordVec(0);
      pt.y = pixelCoordVec(1);
      pt.z = pixelCoordVec(2);

      const cv::Vec3b& rgb = colorImage.at<cv::Vec3b>(cv::Point(v, u));
      pt.b = rgb.val[0];
      pt.g = rgb.val[1];
      pt.r = rgb.val[2];
    }
}

}  // namespace tough_perception