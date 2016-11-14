#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <perception_common/MultisenseImage.h>
#include <perception_common/MultisensePointCloud.h>
#include <perception_common/PointCloudHelper.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>//
#include <val_common/val_common_names.h>


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

#include <std_msgs/Int32MultiArray.h>


/**
 * @brief The ImageFrame class Datatype for storing image frames
 */
namespace src_qual1_task
{
    class ImageFrame
    {
    public:
        /**
         * @brief m_originalImage
         */
        cv::Mat                     m_originalImage;        //original image

        /**
         * @brief m_disparityImage
         */
        cv::Mat                     m_disparityImage;       //disparity image

        /**
         * @brief m_qMatrix
         */
        cv::Mat                     m_qMatrix;          //Q matrix

        /**
         * @brief ImageFrame
         */
        ImageFrame() {}

        /**
             * the copy constructor that ensures deep copy
             * @param i the image frame
             */
        ImageFrame(ImageFrame& i)
        {
            m_originalImage = i.m_originalImage.clone();
            m_disparityImage = i.m_disparityImage.clone();
            m_qMatrix = i.m_qMatrix.clone();
            ROS_DEBUG("HEADER DETECTLED");
        }
    };

    /**
     * @brief The LedDetector class
     */
    class LedDetector
    {
        public:

            /**
             * @brief LedDetector
             * @param nh
             */
            LedDetector(ros::NodeHandle nh);

            ~LedDetector();

            /**
             * @brief m_oldImage
             */
            cv::Mat                     m_oldImage;         //old image

            /**
             * @brief m_oldImage
             */
            bool flag = false;

            /**
             * @brief m_multisenseImagePtr
             */
            src_perception::MultisenseImage         *m_multisenseImagePtr;

            /**
             * @brief m_multisensePcPtr
             */
            src_perception::MultisensePointCloud    *m_multisensePcPtr;


            bool detectLed();
            void getLedColor();
            void getLedLocation();

            /**
             * @brief DetectLED
             * @param new_image
             *
             * todo : return the status as true or false
             */
            void DetectLED(const cv::Mat &new_image);
        private:
            /**
             * @brief m_stereoImage3D
             */
            cv::Mat                                 m_stereoImage3D;

            /**
             * @brief m_imageXYZpub
             */
            ros::Publisher                          m_imageXYZpub;

            /**
             * @brief m_imageRGBpub
             */
            ros::Publisher                          m_imageRGBpub;

            /**
             * @brief old_image
             */
            cv::Mat                                 old_image;

            /**
             * @brief m_gradientContours
             */
            std::vector<std::vector<cv::Point> > m_gradientContours;
            //    cv::RNG rng(12345);

            /**
             * @brief m_randomGen
             */
            cv::RNG m_randomGen;

            /**
             * @brief m_cvDepthPtr
             */
            cv_bridge::CvImagePtr m_cvDepthPtr;

            /**
             * @brief m_baseFrame
             */
            std::string m_baseFrame = VAL_COMMON_NAMES::HOKUYO_LINK_TF;

            /**
             * @brief m_fixedFrame
             */
            std::string m_leftCameraOpticalFrame = VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF;
    };
}
