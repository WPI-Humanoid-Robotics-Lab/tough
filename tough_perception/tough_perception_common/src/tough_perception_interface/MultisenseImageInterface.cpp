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

void MultisenseImageInterface::imageCB(const sensor_msgs::ImageConstPtr &img)
{
    ROS_INFO_ONCE("Listening to %s", cam_sub_.getTopic().c_str());
    img_ = img;
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

    camera_info_sub_ = nh_.subscribe(camera_info_topic, 1,
                                     &MultisenseImageInterface::camera_infoCB, this);

    ROS_INFO("Starting MultisenseImageInterface");
    spinner.start();
    ros::Duration(0.5).sleep();
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
    }
}
bool MultisenseImageInterface::getDisparity(cv::Mat &disp_img)
{
    return true;
}
bool MultisenseImageInterface::getDepthImage(cv::Mat &depth_img)
{
    return true;
}
int MultisenseImageInterface::getHeight()
{
    return camera_info_->height;
}
int MultisenseImageInterface::getWidth()
{
    return camera_info_->width;
}

MultisenseImageInterface::~MultisenseImageInterface()
{
    ROS_INFO("Shutting down MultisenseImageInterface");
    // spinner.stop();
    cam_sub_.shutdown();
    camera_info_sub_.shutdown();
    ros::Duration(0.5).sleep();
}

} // namespace tough_perception