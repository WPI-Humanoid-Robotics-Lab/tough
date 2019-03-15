#include <tough_perception_common/MultisenseImageInterface.h>
#include <tough_perception_common/perception_common_names.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace tough_perception
{
using namespace std;

MultisenseImageInterfacePtr
    MultisenseImageInterface::current_object_ = nullptr;

MultisenseImageInterfacePtr
MultisenseImageInterface::getMultisenseImageInterface(ros::NodeHandle nh)
{
    if (MultisenseImageInterface::current_object_ == nullptr)
    {
        static MultisenseImageInterface obj(nh);
        current_object_ = &obj;
    }
    return current_object_;
}
/*
*@ToDo: add mutex to all the callbacks
*@ToDo: add a common method to process sensor_msg image
*@ToDO: add efficient method to return camera_info/config
*@ToDO: add sync callbacks
*@ToDO: create a class for MultisenseImage Exceptions(is it reall needed? not sure)
*@ToDo: add method to check if device running
*/
void MultisenseImageInterface::imageCB(const sensor_msgs::ImageConstPtr &img)
{
    ROS_INFO_ONCE("Listening to %s", cam_sub_.getTopic().c_str());
    img_ = img;
}

void MultisenseImageInterface::depthCB(const sensor_msgs::ImageConstPtr &img)
{
    ROS_INFO_ONCE("Listening to %s", cam_sub_depth_.getTopic().c_str());
    depth_ = img;
}

void MultisenseImageInterface::costCB(const sensor_msgs::ImageConstPtr &img)
{
    ROS_INFO_ONCE("Listening to %s", cam_sub_depth_.getTopic().c_str());
    cost_ = img;
}

void MultisenseImageInterface::disparityCB(const stereo_msgs::DisparityImageConstPtr &disp)
{
    ROS_INFO_ONCE("Listening to %s", cam_sub_disparity_.getTopic().c_str());
    disparity_ = disp;
}
void MultisenseImageInterface::disparitySensorMsgCB(const sensor_msgs::ImageConstPtr &disp)
{
    ROS_INFO_ONCE("Listening to %s", cam_sub_disparity_sensor_msg_.getTopic().c_str());
    disparity_sensor_msg_ = disp;
}

void MultisenseImageInterface::camera_infoCB(const sensor_msgs::CameraInfoConstPtr camera_info)
{
    ROS_INFO_ONCE("Listening to %s", camera_info_sub_.getTopic().c_str());
    camera_info_ = camera_info;
}

MultisenseImageInterface::MultisenseImageInterface(ros::NodeHandle nh)
    : nh_(nh),
      it_(nh_),
      spinner(2)
{
    cam_sub_ = it_.subscribe(image_topic_, 1,
                             &MultisenseImageInterface::imageCB, this);

    cam_sub_depth_ = it_.subscribe(depth_topic_, 1,
                                   &MultisenseImageInterface::depthCB, this);

    cam_sub_cost_ = it_.subscribe(cost_topic_, 1,
                                  &MultisenseImageInterface::costCB, this);

    cam_sub_disparity_sensor_msg_ = it_.subscribe(disp_sensor_msg_topic_, 1,
                                                  &MultisenseImageInterface::disparitySensorMsgCB, this);

    cam_sub_disparity_ = nh_.subscribe(disp_topic_, 1,
                                       &MultisenseImageInterface::disparityCB, this);

    camera_info_sub_ = nh_.subscribe(camera_info_topic, 1,
                                     &MultisenseImageInterface::camera_infoCB, this);

    ROS_INFO("Starting MultisenseImageInterface");
    spinner.start();
    ros::Duration(0.5).sleep();
}

bool MultisenseImageInterface::processDisparity(const sensor_msgs::Image &disp, cv::Mat &disp_img)
{

    if (disp.data.size() == 0)
        return false;
    try
    {
        int source_type = cv_bridge::getCvType(disp.encoding);
        uint8_t depth = sensor_msgs::image_encodings::bitDepth(disp.encoding); // the size of the disparity data can be 16 or 32
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

bool MultisenseImageInterface::getImage(cv::Mat &img)
{
    ROS_INFO("fetching image");
    if (img_ == nullptr)
        return false;
    if (img_->data.size() == 0)
        return false;
    try
    {
        ROS_INFO("converting image");
        if (cv_bridge::getCvType(img_->encoding) != CV_8UC3)
        {
            ROS_ERROR_STREAM("Unsupported image encoding :" << cv_bridge::getCvType(img_->encoding));
            return false;
        }
        cv_ptr_ = cv_bridge::toCvCopy(img_, sensor_msgs::image_encodings::BGR8);
        img = cv_ptr_->image.clone();
        return true;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR_STREAM("Exception: " << e.what());
        return false;
    }
}

bool MultisenseImageInterface::getDisparity(cv::Mat &disp_img, bool from_stereo_msg)
{
    bool status;
    if (from_stereo_msg)
    {
        ROS_INFO("Fetching disparity from stereo_msg");
        if (disparity_ == nullptr)
            return false;
        status = processDisparity(disparity_->image, disp_img);
    }
    else
    {
        ROS_INFO("Fetching disparity from sensor_msg");
        if (disparity_sensor_msg_ == nullptr)
            return false;
        status = processDisparity(*disparity_sensor_msg_, disp_img);
    }
    return status;
}

bool MultisenseImageInterface::getDepthImage(cv::Mat &depth_img)
{
    ROS_INFO("fetching image");
    if (depth_ == nullptr)
        return false;
    if (depth_->data.size() == 0)
        return false;
    try
    {
        ROS_INFO("converting image");
        if (cv_bridge::getCvType(depth_->encoding) != CV_32F)
        {
            ROS_ERROR_STREAM("Unsupported image encoding :" << cv_bridge::getCvType(depth_->encoding));
            return false;
        }
        cv_ptr_ = cv_bridge::toCvCopy(depth_, sensor_msgs::image_encodings::TYPE_32FC1);
        depth_img = cv_ptr_->image.clone();
        return true;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR_STREAM("Exception: " << e.what());
        return false;
    }
}
bool MultisenseImageInterface::getCostImage(cv::Mat &cost_img)
{
    ROS_INFO("fetching image");
    if (cost_ == nullptr)
        return false;
    if (cost_->data.size() == 0)
        return false;
    try
    {
        ROS_INFO("converting image");
        if (cv_bridge::getCvType(cost_->encoding) != CV_8U)
        {
            ROS_ERROR_STREAM("Unsupported image encoding :" << cv_bridge::getCvType(cost_->encoding));
            return false;
        }
        cv_ptr_ = cv_bridge::toCvCopy(cost_, sensor_msgs::image_encodings::MONO8);
        cost_img = cv_ptr_->image.clone();
        return true;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR_STREAM("Exception: " << e.what());
        return false;
    }
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

MultisenseImageInterface::~MultisenseImageInterface()
{
    ROS_INFO("Shutting down MultisenseImageInterface");
    // spinner.stop();
    cam_sub_.shutdown();
    cam_sub_depth_.shutdown();
    cam_sub_cost_.shutdown();
    cam_sub_disparity_.shutdown();
    cam_sub_disparity_sensor_msg_.shutdown();
    camera_info_sub_.shutdown();
    ros::Duration(0.5).sleep();
}

} // namespace tough_perception