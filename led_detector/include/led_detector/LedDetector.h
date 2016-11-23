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
#include <led_detector/LedPositionColor.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
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
         * @brief m_originalImage - Original image obtained from Multisense headset
         */
        cv::Mat                     m_originalImage;        //original image

        /**
         * @brief m_disparityImage - Disparity image obtained from Multisense headset
         */
        cv::Mat                     m_disparityImage;       //disparity image

        /**
         * @brief m_qMatrix - Q matrix obtained from Multisense headset
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
        }
    };

    /**
     * @brief The LedDetector class
     */
    class LedDetector
    {
        public:
             LedDetector(ros::NodeHandle nh);
            ~LedDetector();
            bool detectLight();
            bool getLight(cv::Mat &new_image,geometry_msgs::Point &pixelCoordinates);
            bool getPoseRGB(ImageFrame &img_frame,geometry_msgs::Point &pixelCoordinates);

            /**
             * @brief message - Message of type LedPositionColor (Custom defined message type with x,y,z,r,g,b,a)
              */
            led_detector::LedPositionColor message;
            
            /**
             * @brief m_multisenseImagePtr - Pointer to original image obtained from Multisense headset
             */
            src_perception::MultisenseImage         *m_multisenseImagePtr;
            
            /**
             * @brief m_multisensePcPtr - Pointer to Point Cloud obtained from Multisense headset
             */
            src_perception::MultisensePointCloud    *m_multisensePcPtr;

            /**
             * @brief m_baseFrame
             */
            std::string m_baseFrame = VAL_COMMON_NAMES::HOKUYO_LINK_TF;

            /**
             * @brief m_fixedFrame
             */
            std::string m_leftCameraOpticalFrame = VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF;

           

            
        protected:

            // ros::NodeHandle                         _nh;
            
            /**
             * @brief m_stereoImage3D
             */
            cv::Mat                                 m_stereoImage3D;

            /**
             * @brief m_imageRGBpub - Publisher that publishes xyz and rgb data to topic "/detect/light/rgbxyz"
             */
            ros::Publisher                          m_imageRGBXYZpub;

            /**
             * @brief m_gradientContours - 
             */
            std::vector<std::vector<cv::Point> > m_gradientContours;
            //    cv::RNG rng(12345);

            /**
             * @brief m_randomGen
             */
            cv::RNG m_randomGen;
      
    };
}
