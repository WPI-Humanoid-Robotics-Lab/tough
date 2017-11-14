    /**
 ********************************************************************************************************
 * @file 		MultisenseImage.h
 * @brief		This file contains the multisenseImage class that is used to convert from multisense
 * 				image topics. Wrapper class.
 * @details 	load disparity, load image, load camera config files
 ********************************************************************************************************
 */

#ifndef MULTISENSEIMAGE_H_
#define MULTISENSEIMAGE_H_

/*** INCLUDE FILES ***/
#include <perception_common/global.h>
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

namespace tough_perception {

class MultisenseImage {

    DISALLOW_COPY_AND_ASSIGN(MultisenseImage)

    cv::Mat 				image_;
    cv::Mat         		disparity_;
    cv::Mat					depth_;
    cv::Mat					cost_;

    struct
    {
        cv::Mat         		camera_;
        cv::Mat_<double>		Q_matrix_;
        int						height_;
        int 					width_;
        float					fps_;
        float					gain_;
        float					exposure_;
        float					baselength_;
    }settings;									//this is the data that is available within the config message that multisense
    //uses so

    std_msgs::Header		img_header_;
    std_msgs::Header		disp_header_;
    std_msgs::Header		depth_header_;
    std_msgs::Header		cost_header_;

    cv_bridge::CvImagePtr   cv_ptr_;			//feels like it has both the header and the image information
    //use this data type instead of the image_ and img_header_??

    ros::NodeHandle 		nh_;
    std::string 			image_topic_,
    disp_topic_,
    depth_topic_,
    depth_cost_topic_,
    multisense_topic_;

    bool					new_image_;
    bool            		new_disp_;
    bool            		new_depth_;
    bool            		new_cost_;

    static bool				image_callback_active_;
    static bool				disp_callback_active_;
    static bool				config_callback_active_;
    static bool				cost_callback_active_;
    static bool				depth_callback_active_;

    image_transport::ImageTransport it_;

    image_transport::Subscriber 		cam_sub_;
    image_transport::SubscriberFilter   *sync_cam_sub_;
    image_transport::SubscriberFilter   *sync_cam_depth_sub_;
    image_transport::SubscriberFilter   *sync_cam_cost_sub_;
#ifndef GAZEBO_SIMULATION
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> exactTimePolicy;
    std::shared_ptr<message_filters::Synchronizer< exactTimePolicy > > sync_;
    image_transport::SubscriberFilter   *sync_disp_sub_;

#else
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, stereo_msgs::DisparityImage> approxTimePolicy;
    std::shared_ptr<message_filters::Synchronizer< approxTimePolicy > > sync_;
#endif

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> depthImageCostExactTimePolicy;
    std::shared_ptr<message_filters::Synchronizer< depthImageCostExactTimePolicy > > sync_depth_;

    image_transport::Subscriber 		depth_sub_;
    image_transport::Subscriber 		cost_sub_;
#ifndef GAZEBO_SIMULATION
    image_transport::Subscriber 		disp_sub_;

#else
    ros::Subscriber 					disp_sub_;
    message_filters::Subscriber<stereo_msgs::DisparityImage> *sync_disp_sub_;
#endif
    ros::Subscriber						multisense_sub_;

    /**
     * @brief this function is the callback for loading the images, as of now it needs the image topic to
     *        have the camerainfo be published, but I think this should be removed as multisense head does
     *        not necesarrily publish the right topics.
     * @param img the image from the multisense head
     */
    void loadImage(const sensor_msgs::ImageConstPtr &img);


    /**
     * @brief this function is the callback for loading the depth image
     * @param depth_img the depth image from the multisense head
     */
    void loadDepthImage(const sensor_msgs::ImageConstPtr &depth_img);

    void loadCostImage(const sensor_msgs::ImageConstPtr &img);

    /**
     * @brief this function loads the disparity image
     * @param img the ros image published
     */
    void loadDisparityImageSensorMsgs(const sensor_msgs::ImageConstPtr &img);

    /**
     * @brief this function loads the camera parameter from the multisense topic
     * @param config the ros data published by the multisense head
     */
    void loadCameraConfig(const multisense_ros::RawCamConfigConstPtr &config);

#ifdef GAZEBO_SIMULATION
    /**
     * @brief this function loads the camera parameter from the multisense in SIM topic
     * @param config the ros data published by the multisense head in SIM
     */
    void loadDisparityImageStereoMsgs(const stereo_msgs::DisparityImageConstPtr &img);
#endif

#ifndef GAZEBO_SIMULATION
    void syncCallback(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::ImageConstPtr &dimg);
#else
    void syncCallback(const sensor_msgs::ImageConstPtr &img, const stereo_msgs::DisparityImageConstPtr &dimg);
#endif
    void syncDepthCallback(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::ImageConstPtr &dimg, const sensor_msgs::ImageConstPtr &cimg);
public:
    /**
     * @brief Constructor
     * @param nh the ros nodehandle - why do i have this as an argument? expect for the fact to make sure
     * 		  the constructor knows that it is ros. I have no idea why I do this!
     */
    MultisenseImage(ros::NodeHandle &nh_);

    void setDepthTopic(const std::string &topic);

    /**
     * @brief gives an image loaded
     * @param img as a refernce which will be filled by the image from the multisense head
     * @return	if we have a new image
     */
    bool giveImage(cv::Mat &img);

    /**
     * @brief gives the camera intrinsic matrix
     * @param cam the cmaera matrix
     * @return true if you have a new camera matrix
     */
    bool giveCameraInfo(cv::Mat &cam);
    /**
     * @brief gives the disparity image that the camera has for the disparity
     * @param disp_img the disparity image that the multisense gets
     * @return true if you have new disparity image
     */
    bool giveDisparityImage(cv::Mat &disp_img);

    /**
     * @brief the Q matrix that is used to convert disparity to 3D points
     * @param Q the matrix as CV::mat
     * @return true if a new value was recieved from the head
     */
    bool giveQMatrix(cv::Mat &Q);

    /**
     * @brief this function makes sure you get a synchronized pair of color img and disp image
     * @param color the color image
     * @param disp  the dispaity image
     * @return
     */
    bool giveSyncImages(cv::Mat &color, cv::Mat &disp);

    /**
     * @brief This function was introduced in situations where you want to track the fps of the images being collected
     * @param color the color image
     * @param disp	the disparity image
     * @param time  the timestamp on the recieved image
     * @return
     */
    bool giveSyncImageswTime(cv::Mat &color, cv::Mat &disp, ros::Time &time);

    bool giveCostImage(cv::Mat &img);

    bool giveSyncDepthImages(cv::Mat &color, cv::Mat &disp, cv::Mat &cost);

    bool giveSyncDepthImageswTime(cv::Mat &color, cv::Mat &disp, cv::Mat &cost, ros::Time &time);

    /**
     * @brief A function to get the height in situations where you have not yet received an image.
     *        The height is read directly from the config message.
     * @return the height as an integer
     */
    int giveHeight()
    {
        return settings.height_;
    }
    /**
     * @brief A function to get the width in situations where you have not yet received an image.
     *        The width is read directly from the config message.
     * @return the width as an integer
     */
    int giveWidth()
    {
        return settings.width_;
    }
    /**
     * @brief the baselength as read from the config message.
     * @return the baselength as a floating point number
     */
    float giveBaseLength()
    {
        return settings.baselength_;
    }

    /**
     * @brief this function is used to get the time the image was received. This need to be redone
     * @param time the time of image capture as told by ROS
     * @return always true
     */
    bool giveTime(ros::Time &time);

    /**
     * @brief this function is used to get the depth image that was recieved
     * @param depth_img	the depth image that was recieved
     * @return	true if new depth image
     */
    bool giveDepthImage(cv::Mat &depth_img);

    virtual ~MultisenseImage();
};

} /* namespace src_perception */

#endif /* MULTISENSEIMAGE_H_ */
