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
#include "srcsim/Console.h"
#include <stdlib.h>
#include <math.h>
#include "alglibmisc.h"


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
            void errorCorrection(std::vector<srcsim::Console> &data, std::vector<double> &pos);
            void kdtreeInit();
            int getMode (int daArray[], int iSize);
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
            cv::Mat m_stereoImage3D;

            cv::Mat m_qMatrix;

            /**
             * @brief m_imageRGBpub - Publisher that publishes xyz and rgb data to topic "/detect/light/rgbxyz"
             */
            ros::Publisher m_imageRGBXYZpub;
            ros::Publisher m_lightPub;

            /**
             * @brief m_gradientContours - 
             */
            std::vector<std::vector<cv::Point> > m_gradientContours;
            //    cv::RNG rng(12345);

            /**
             * @brief m_randomGen
             */
            cv::RNG m_randomGen;

            alglib::kdtree kdt;
            double error_x[43] = {52.099999999999909, 78.499999999999773, -140.79999999999973, 25.099999999999909, 24.800000000000182, 83.000000000000455, 42.5, 47.749999999999773, -178.69999999999936, -189.00000000000045, 89.350000000000364, 48.200000000000273, -32.099999999999909, -1.3000000000004093, 88.800000000000182, 53.899999999999864, 71.400000000000091, 31.199999999999818, 4.5000000000004547, 42.299999999999955, 37.200000000000273, 0.3000000000001819, 19.400000000000091, 10.799999999999727, 34.300000000000182, 53.450000000000045, 86.200000000000045, -29.099999999999682, 49.099999999999909, -11.149999999999864, 44.799999999999727, 84.899999999999636, 39.700000000000045, 62.900000000000091, 73.400000000000091, 100.29999999999973, 20.799999999999727, 74.800000000000182, 54.900000000000091, 89.300000000000182, -4.7500000000002274, 33.099999999999454, 39.800000000000182};
            double error_y[43] = {-0.59999999999996589, -24.300000000000011, -0.70000000000001705, -4.6000000000000227, -11.199999999999932, -16.400000000000006, -17.100000000000023, 1.0999999999999943, 31.399999999999977, -32.0, -4.1500000000000057, -15.200000000000045, -9.2000000000000455, -6.9000000000000341, 6.0, 1.9499999999999886, -14.900000000000034, -11.700000000000045, -7.0999999999999659, -1.4499999999999886, -2.8000000000000682, -5.4000000000000341, -9.7000000000000455, -7.1999999999999886, -14.5, -16.199999999999989, -2.2000000000000028, 0.59999999999999432, 0.85000000000002274, -6.75, -14.799999999999955, -16.600000000000023, -1.5500000000000114, -20.899999999999977, 4.0, -17.199999999999989, -6.5999999999999659, -3.5499999999999972, -17.150000000000034, -3.0999999999999943, -4.2999999999999545, 0.30000000000001137, -12.199999999999932};
            double error_z[43] = {0.60000000000002274, 3.5499999999999829, 18.200000000000045, -5.5, -5.6999999999999993, 5.7000000000000455, 1.6499999999999773, 3.5499999999999829, 24.5, 27.300000000000011, 6.0, -3.8999999999999915, -9.3000000000000682, -5.0999999999999659, 3.4000000000000341, -3.8999999999999915, 2.1999999999999886, -4.5999999999999943, -4.0, -3.5999999999999943, -6.1999999999999993, -4.8000000000000114, -5.0, -4.0, -5.2999999999999972, 1.3499999999999943, 3.2500000000000142, -9.4000000000000057, 0.69999999999998863, -5.0499999999999829, -0.099999999999965894, 3.1999999999999602, -0.90000000000003411, 2.6999999999999318, 4.2999999999999545, 4.3000000000000114, -7.0, 3.6500000000000057, 3.0999999999999943, 6.4000000000000057, -6.5500000000000114, -7.2000000000000028, -5.3999999999999915};

            const int m_readingThreshold = 3;

            std::vector<srcsim::Console> m_readings;
      
    };
}
