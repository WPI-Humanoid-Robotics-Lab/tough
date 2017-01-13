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

            double error_x[43] = {82.799999999999983, 83.714285714285651, 33.175000000000011, 147.13636363636368, 40.937500000000171, 34.766666666666502, 78.762500000000102, 11.149999999999977, 163.17999999999967, 46.340000000000174, 192.30000000000018, 43.872222222222291, 71.9769230769231, 39.083333333333258, 34.9866666666668, 24.062499999999915, 56.810526315789524, 22.937499999999716, 26.828571428571358, 70.439999999999912, 62.209090909091053, 53.142105263157788, 22.41666666666659, 75.404166666666626, 17.612500000000125, 64.225000000000023, 59.328571428571195, 22.542857142856942, 10.600000000000104, 89.500000000000171, 74.681818181818187, 6.1857142857143117, 22.189999999999873, 60.807692307692307, 26.319047619047609, 100.98571428571437, 31.153846153846082, 40.678571428571459, 93.16250000000025, 91.60909090909081, 174.04999999999987, 187.02500000000009, 15.39333333333334};
            double error_y[43] = {12.28571428571429, 8.0428571428571694, 13.250000000000057, 4.863636363636366, 13.737500000000004, 2.0333333333333221, 1.6249999999999964, 6.7500000000000142, 29.859999999999992, 14.433333333333326, 39.883333333333347, 1.5555555555555525, 19.507692307692306, 12.650000000000025, 13.553333333333352, 5.381249999999973, 3.6105263157894658, 4.7874999999999943, 11.235714285714282, 2.0300000000000069, 15.490909090909099, 15.078947368421058, 3.8916666666666608, 14.229166666666666, 3.1187499999999986, 3.800000000000002, 4.8142857142856963, 9.3928571428571299, 3.4714285714285609, 3.9250000000000007, 22.572727272727288, 6.5714285714285792, 9.3150000000000084, 1.2615384615384728, 2.900000000000015, 13.900000000000009, 11.553846153846145, 2.7142857142857224, 0.95000000000000995, 15.872727272727275, 25.583333333333332, 31.999999999999986, 4.8000000000000078};
            double error_z[43] = {4.4000000000000012, 3.999999999999992, 5.0874999999999968, 19.390909090909101, 2.6062500000000064, 4.7666666666666648, 4.1124999999999972, 4.9999999999999716, 21.840000000000011, 4.6066666666666629, 27.116666666666674, 3.9944444444444391, 2.6769230769230559, 4.6833333333333371, 2.619999999999997, 4.0874999999999808, 2.2578947368421041, 6.1875000000000018, 5.3857142857142852, 2.7200000000000015, 1.4363636363636194, 1.9789473684210479, 3.8333333333333428, 3.6000000000000014, 7.0437500000000064, 2.0083333333333351, 3.9714285714285649, 5.0, 6.7285714285714322, 6.0624999999999964, 3.0181818181818016, 5.0142857142857258, 8.4799999999999933, 2.0076923076923006, 5.5714285714285721, 4.3571428571428568, 4.546153846153846, 4.5285714285714294, 5.4000000000000092, 3.8454545454545515, 24.01666666666668, 27.024999999999991, 3.1733333333333236};
            const int m_readingThreshold = 3;

            std::vector<srcsim::Console> m_readings;
      
    };
}
