#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <perception_common/MultisenseImage.h>
#include <perception_common/MultisensePointCloud.h>
#include <perception_common/PointCloudHelper.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>//



#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/Point.h>
#include <std_msgs/Int32MultiArray.h>
// #include <sensor_msgs/PointCloud2.h>


// Datatype for storing image frames
class ImageFrame
{
    public:
        cv::Mat                     img;        //original image
        cv::Mat                     disp;       //disparity image
        cv::Mat                     Q;          //Q matrix
        cv::Mat                     K;

        ImageFrame() {}

        /**
         * the copy constructor that ensures deep copy
         * @param i the image frame
         */
        ImageFrame(ImageFrame& i)
        {
            img = i.img.clone();
            disp = i.disp.clone();
            Q = i.Q.clone();
            K = i.K.clone();
        }
};

class LightDetector
{
public:
    src_perception::MultisenseImage         *multisenseImage;
    src_perception::MultisensePointCloud    *multisensePc;
    src_perception::LaserPointCloud::Ptr    laserCloud;//!!!
    LightDetector(ros::NodeHandle);
    ~LightDetector();

    bool detectLed();
    void getLedColor();
    void getLedLocation();
private:
    ros::NodeHandle                         nh;
    cv::Mat                                 stereoImage3D;
    ros::Publisher                          pubLedCoordinatesCam;
    // ros::Subscriber                         _scribble_sub;//!!!!!!!
    ros::Publisher                          pubRGBColor;

};