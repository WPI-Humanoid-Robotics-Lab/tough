/**
 ********************************************************************************************************
 * @file    MultisenseImage.cpp
 * @brief   MultisenseImage class
 * @details Used to get the camera images from the camera
 ********************************************************************************************************
 */

/**
 * TODO: the camera config must be loaded from the topics for gazebo simulation also
 */

#include <tough_perception_common/MultisenseImage.h>
#include <tough_perception_common/perception_common_names.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace tough_perception
{
using namespace std;

/**
 *  These variables are needed to prevent multiple subscribers from being generated and to be only
 *  generated in request.
 */
bool MultisenseImage::image_callback_active_  = false;
bool MultisenseImage::disp_callback_active_   = false;
bool MultisenseImage::config_callback_active_ = false;
bool MultisenseImage::depth_callback_active_  = false;
bool MultisenseImage::cost_callback_active_   = false;
/**
 *  @note do we need the NodeHandle passed??? Think benny
 */
MultisenseImage::MultisenseImage(ros::NodeHandle& n)
  : nh_(n), 
  new_image_(false), 
  new_disp_(false), 
  new_depth_(false), 
  new_cost_(false), 
  it_(nh_), 
  sync_(nullptr)
{
 
}

void MultisenseImage::setDepthTopic(const std::string& topic)
{
  depth_topic_ = topic;
}

void MultisenseImage::setImageTopic(const std::string& topic)
{
  image_topic_ = topic;
}

/**
 * @note this callback is needed both for the instrinsic and the Q matrix
 *        it will change when the resolution changes
 */
void MultisenseImage::loadCameraConfig(const multisense_ros::RawCamConfigConstPtr& config)
{
  settings.Q_matrix_ = cv::Mat_<double>(4, 4, 0.0);
  settings.Q_matrix_(0, 0) =  config->fy * config->tx;
  settings.Q_matrix_(1, 1) =  config->fx * config->tx;
  settings.Q_matrix_(0, 3) = -config->fy * config->cx * config->tx;
  settings.Q_matrix_(1, 3) = -config->fx * config->cy * config->tx;
  settings.Q_matrix_(2, 3) =  config->fx * config->fy * config->tx;
  settings.Q_matrix_(3, 2) = -config->fy;
  settings.Q_matrix_(3, 3) =  0.0f;

  settings.camera_ = (cv::Mat_<float>(3, 3) << config->fx, 0.0, config->cx, 0.0, config->fy, config->cy, 0.0, 0.0, 1.0);

  settings.width_       = config->width;
  settings.height_      = config->height;
  settings.fps_         = config->frames_per_second;
  settings.exposure_    = config->exposure_time;
  settings.gain_        = config->gain;
  settings.baselength_  = fabs(config->tx);

  ROS_INFO_STREAM_ONCE("Received new camera config: " << settings.Q_matrix_);
}

void MultisenseImage::syncCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::ImageConstPtr& dimg)
{
  loadImage(img);
  loadDisparityImageSensorMsgs(dimg);
}

void MultisenseImage::syncDepthCallback(const sensor_msgs::ImageConstPtr& img, 
                                        const sensor_msgs::ImageConstPtr& dimg,
                                        const sensor_msgs::ImageConstPtr& cimg)
{
  loadImage(img);
  loadDepthImage(dimg);
  loadCostImage(cimg);
}

/**
 * @note callback for loading images
 */
void MultisenseImage::loadImage(const sensor_msgs::ImageConstPtr& img)
{
  try
  {
    int source_type = cv_bridge::getCvType(img->encoding);
    if (source_type == CV_8UC3)
    {
      cv_ptr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    else
    {
      cv_ptr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    image_      = cv_ptr_->image.clone();
    new_image_  = true;
    img_header_ = cv_ptr_->header;

    ROS_INFO_ONCE("Received new image size: %d x %d", image_.rows, image_.cols);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR_STREAM("Exception: " << e.what());
  }
}

/**
 * @note callback for loading images
 */
void MultisenseImage::loadDepthImage(const sensor_msgs::ImageConstPtr& img)
{
  try
  {
    int source_type = cv_bridge::getCvType(img->encoding);
    if (source_type == CV_32FC1)
    {
      cv_ptr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    else
    {
      ROS_ERROR_STREAM("Unsupported depth map?");
      return;
    }
    depth_        = cv_ptr_->image.clone();
    new_depth_    = true;
    depth_header_ = cv_ptr_->header;

    ROS_INFO_ONCE("Received new depth image size: %d x %d", depth_.rows, depth_.cols);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR_STREAM("Exception: " << e.what());
  }
}

void MultisenseImage::loadCostImage(const sensor_msgs::ImageConstPtr& img)
{
  try
  {
    int source_type = cv_bridge::getCvType(img->encoding);
    if (source_type == CV_8U)
    {
      cv_ptr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
    {
      ROS_ERROR_STREAM("Unsupported cost map?");
      return;
    }
    cost_        = cv_ptr_->image.clone();
    new_cost_    = true;
    cost_header_ = cv_ptr_->header;

    ROS_INFO_ONCE("Received new cost image size: %d x %d", cost_.rows, cost_.cols);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR_STREAM("Exception: " << e.what());
  }
}

/*
 *  @note callback for simulation disparity as it is published as stereo_msgs instead of sensor_msgs
 */
void MultisenseImage::loadDisparityImageStereoMsgs(const stereo_msgs::DisparityImageConstPtr& img)
{
  try
  {
    int source_type = cv_bridge::getCvType(img->image.encoding);
    uint8_t depth =
        sensor_msgs::image_encodings::bitDepth(img->image.encoding);  // the size of the disparity data can be 16 or 32
    if (depth == 32)
    {
      cv::Mat_<float> disparity(img->image.height, img->image.width,
                                const_cast<float*>(reinterpret_cast<const float*>(&img->image.data[0])));

      disparity_   = disparity.clone();
      new_disp_    = true;
      disp_header_ = img->image.header;
    }
    else if (depth == 16)
    {
      cv::Mat_<uint16_t> disparityOrigP(img->image.height, img->image.width,
                                        const_cast<uint16_t*>(reinterpret_cast<const uint16_t*>(&img->image.data[0])));
      cv::Mat_<float> disparity(img->image.height, img->image.width);
      disparity    = disparityOrigP / 16.0f;
      disparity_   = disparity;
      new_disp_    = true;
      disp_header_ = img->image.header;
    }
    else
      ROS_WARN("disparity depth not recognized");

    ROS_INFO_ONCE("Received new disparity image size: %d x %d", disparity_.rows, disparity_.cols);
  }
  catch (std::exception ex)
  {
    ROS_ERROR_STREAM("Exception: " << ex.what());
  }
}

/**
 * @note callback for loading disparity
 */

void MultisenseImage::loadDisparityImageSensorMsgs(const sensor_msgs::ImageConstPtr& img)
{
  try
  {
    int source_type = cv_bridge::getCvType(img->encoding);
    uint8_t depth =
        sensor_msgs::image_encodings::bitDepth(img->encoding);  // the size of the disparity data can be 16 or 32
    if (depth == 32)
    {
      cv::Mat_<float> disparity(img->height, img->width,
                                const_cast<float*>(reinterpret_cast<const float*>(&img->data[0])));

      disparity_   = disparity.clone();
      new_disp_    = true;
      disp_header_ = img->header;
    }
    else if (depth == 16)
    {
      cv::Mat_<uint16_t> disparityOrigP(img->height, img->width,
                                        const_cast<uint16_t*>(reinterpret_cast<const uint16_t*>(&img->data[0])));
      cv::Mat_<float> disparity(img->height, img->width);
      disparity    = disparityOrigP / 16.0f;
      disparity_   = disparity.clone();
      new_disp_    = true;
      disp_header_ = img->header;
    }
    else
      ROS_WARN("disparity depth not recognized");

    ROS_INFO_ONCE("Received new disparity image size: %d x %d , depth: %d", disparity_.rows, disparity_.cols, depth);
  }
  catch (std::exception ex)
  {
    ROS_ERROR_STREAM("Exception: " << ex.what());
  }
}

/**
 * @note starts the subscriber if not started
 */
bool MultisenseImage::getDisparityImage(cv::Mat& disp_img)
{
  if (!disp_callback_active_)
  {

    disp_sub_ = it_.subscribe(disp_topic_, 1, &MultisenseImage::loadDisparityImageSensorMsgs, this);
    disp_callback_active_ = true;
    ROS_INFO_STREAM("Listening to: " << disp_sub_.getTopic() << endl);
    ros::Duration(1).sleep();
    ros::spinOnce();
  }
  if (new_disp_)
  {
    if (disparity_.empty())
      return false;
    disp_img  = disparity_;
    new_disp_ = false;
    return true;
  }

  return false;
}

bool MultisenseImage::getDepthImage(cv::Mat& depth_img)
{
  if (!depth_callback_active_)
  {
    depth_sub_ = it_.subscribe(depth_topic_, 1, &MultisenseImage::loadDepthImage, this);
    depth_callback_active_ = true;
    ROS_INFO_STREAM("Listening to: " << depth_sub_.getTopic() << endl);
    ros::Duration(1).sleep();
    ros::spinOnce();
  }
  if (new_depth_)
  {
    if (depth_.empty())
      return false;
    depth_img  = depth_;
    new_depth_ = false;
    return true;
  }

  return false;
}
/**
 * @note starts subscriber if not srtaed. TODO: try breaking it using multiple objects of the class
 */
bool MultisenseImage::getImage(cv::Mat& img)
{
  if (!image_callback_active_)
  {
    cam_sub_ = it_.subscribe(image_topic_, 1, &MultisenseImage::loadImage, this);
    image_callback_active_ = true;
    ROS_INFO_STREAM("Listening to: " << cam_sub_.getTopic() << endl);
    ros::Duration(1).sleep();
    ros::spinOnce();
  }

  if (new_image_)
  {
    img = image_;
    if (img.empty())
      return false;
    new_image_ = false;
    return true;
  }
  return false;
}

bool MultisenseImage::getCostImage(cv::Mat& img)
{
  if (!cost_callback_active_)
  {
    cost_sub_ = it_.subscribe(depth_cost_topic_, 1, &MultisenseImage::loadCostImage, this);
    cost_callback_active_ = true;
    ROS_INFO_STREAM("Listening to: " << cost_sub_.getTopic() << endl);
    ros::Duration(1).sleep();
    ros::spinOnce();
  }

  if (new_cost_)
  {
    img = cost_;
    if (img.empty())
      return false;
    new_cost_ = false;
    return true;
  }
  return false;
}

/**
 * @note none
 */
bool MultisenseImage::getCameraInfo(cv::Mat& cam)
{
  if (!config_callback_active_)
  {
    multisense_sub_ = nh_.subscribe(multisense_topic_, 1, &MultisenseImage::loadCameraConfig, this);
    config_callback_active_ = true;
    ROS_INFO_STREAM("Listening to: " << multisense_sub_.getTopic() << endl);
    ros::Duration(1).sleep();
    ros::spinOnce();
  }
  if (!settings.camera_.empty())
  {
    cam = settings.camera_;
    return true;
  }
  return false;
}
/**
 * @note none
 */
bool MultisenseImage::getQMatrix(cv::Mat& Q)
{
  if (!config_callback_active_)
  {
    multisense_sub_ = nh_.subscribe(multisense_topic_, 1, &MultisenseImage::loadCameraConfig, this);
    config_callback_active_ = true;
    ROS_INFO_STREAM("Listening to: " << multisense_sub_.getTopic() << endl);
    ros::Duration(1).sleep();
    ros::spinOnce();
  }
  if (!settings.Q_matrix_.empty())
  {
    Q = settings.Q_matrix_;
    return true;
  }
  return false;
}

bool MultisenseImage::getSyncImages(cv::Mat& color, cv::Mat& disp)
{
  ROS_INFO_STREAM_ONCE("Requesting synchornized image");
  if (sync_ == nullptr)
  {
    cam_sub_.shutdown();
    sync_cam_sub_ = new image_transport::SubscriberFilter(it_, image_topic_, 1);
    // cam_sub_=sync_cam_sub_.getSubscriber();
    image_callback_active_ = true;
    ROS_INFO_STREAM("Listening to: " << sync_cam_sub_->getTopic() << endl);
    disp_sub_.shutdown();

    sync_disp_sub_ = new image_transport::SubscriberFilter(it_, disp_topic_, 1);

    // disp_sub_=sync_disp_sub_.getSubscriber();
    disp_callback_active_ = true;
    ROS_INFO_STREAM("Listening to: " << sync_disp_sub_->getTopic() << endl);

    sync_.reset(
        new message_filters::Synchronizer<exactTimePolicy>(exactTimePolicy(1000), *sync_cam_sub_, *sync_disp_sub_));

    sync_->registerCallback(boost::bind(&MultisenseImage::syncCallback, this, _1, _2));
    ros::Duration(1).sleep();
    ros::spinOnce();
  }

  ROS_INFO_STREAM_ONCE("image topic: " << image_topic_);
  ROS_INFO_STREAM_ONCE("disp topic: " << disp_topic_);

  if (!getImage(color) || !getDisparityImage(disp))
  {
    return false;
  }
  return true;
}

bool MultisenseImage::getSyncDepthImages(cv::Mat& color, cv::Mat& disp, cv::Mat& cost)
{
  ROS_INFO_STREAM_ONCE("Requesting synchornized depth image");
  if (sync_depth_ == nullptr)
  {
    cam_sub_.shutdown();
    sync_cam_sub_ = new image_transport::SubscriberFilter(it_, image_topic_, 1);
    // cam_sub_=sync_cam_sub_.getSubscriber();
    image_callback_active_ = true;
    ROS_INFO_STREAM("Listening to: " << sync_cam_sub_->getTopic() << endl);
    depth_sub_.shutdown();

    ROS_INFO("DRCSIM not enabled");
    sync_cam_depth_sub_ = new image_transport::SubscriberFilter(it_, depth_topic_, 1);
    sync_cam_cost_sub_  = new image_transport::SubscriberFilter(it_, depth_cost_topic_, 1);


    ROS_INFO_STREAM("Listening to: " << sync_cam_depth_sub_->getTopic() << endl);
    ROS_INFO_STREAM("Listening to: " << sync_cam_cost_sub_->getTopic() << endl);
    sync_depth_.reset(new message_filters::Synchronizer<depthImageCostExactTimePolicy>(
        depthImageCostExactTimePolicy(1000), *sync_cam_sub_, *sync_cam_depth_sub_, *sync_cam_cost_sub_));
    sync_depth_->registerCallback(boost::bind(&MultisenseImage::syncDepthCallback, this, _1, _2, _3));
    ros::Duration(1).sleep();
    ros::spinOnce();
  }

  if (!getImage(color) || !getDepthImage(disp) || !getCostImage(cost))
  {
    return false;
  }
  return true;
}

// this function is a temp function that I am implementing, maybe the statistics on the fps must be maintained
// internally and
// not by the called class;
bool MultisenseImage::getTime(ros::Time& time)
{
  if (disp_header_.stamp == ros::Time(0))
    time = img_header_.stamp;
  else
    time = disp_header_.stamp;
  return true;
}
bool MultisenseImage::getSyncImageswTime(cv::Mat& color, cv::Mat& disp, ros::Time& time)
{
  if (!getSyncImages(color, disp))
    return false;
  // assert(color.empty());
  time = img_header_.stamp;
  return true;
}
bool MultisenseImage::getSyncDepthImageswTime(cv::Mat& color, cv::Mat& disp, cv::Mat& cost, ros::Time& time)
{
  if (!getSyncDepthImages(color, disp, cost))
  {
    return false;
  }
  time = img_header_.stamp;
  return true;
}
/**
 * @note none
 */
MultisenseImage::~MultisenseImage()
{
  //	sync_->~Synchronizer();
  //  sync_->~Synchronizer();
  //  delete sync_;
  //	sync_cam_sub_->unsubscribe();
  //	sync_disp_sub_->unsubscribe();
  //	delete sync_cam_sub_;
  //	delete sync_disp_sub_;
}

} /* namespace tough_perception */
