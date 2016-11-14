#include <led_detector/LedDetector.h>
#include <ros/ros.h>
#include <perception_common/MultisenseImage.h>
//#include <perception_common/MultisensePointCloud.h>
using namespace cv;
using namespace std;
using namespace src_perception;

namespace src_qual1_task
{
    LedDetector::LedDetector(ros::NodeHandle nh)
            //:m_multisenseImagePtr(new src_perception::MultisenseImage(nh)),m_multisensePcPtr(new src_perception::MultisensePointCloud(rosHandle, m_baseFrame, m_leftCameraOpticalFrame)))
    {
        m_multisenseImagePtr = NULL;
        //m_multisensePcPtr = NULL;
        m_randomGen= cv::RNG(12345);
        m_imageRGBpub = nh.advertise<std_msgs::Int32MultiArray>("/detect/light/rgb", 100);
        m_imageXYZpub = nh.advertise<geometry_msgs::Point>("/detect/light/xy", 100);
        cv::namedWindow("Raw Image with Contours"); //For demo only
    }
    LedDetector::~LedDetector()
    {
    }


    bool LedDetector::detectLed()
    {

        // This is the function where we should implement the algorithm
        ImageFrame              img_frame;
        cv::Mat                 img_3d;
        if (m_multisenseImagePtr == NULL)   // || m_multisensePcPtr == NULL)
            return false;
        //image_frame.m_originalImage is the cv image to process
        while(ros::ok())
        {
            if (m_multisenseImagePtr->giveSyncImages(img_frame.m_originalImage, img_frame.m_disparityImage))
            {
                if (img_frame.m_qMatrix.empty())
                {
                    if (!m_multisenseImagePtr->giveQMatrix(img_frame.m_qMatrix))
                        continue;
                    else
                        m_multisenseImagePtr->giveQMatrix(img_frame.m_qMatrix);
                }

                // ROS_DEBUG("BEFORE REPROJECT");
                cv::reprojectImageTo3D(img_frame.m_disparityImage, img_3d, img_frame.m_qMatrix, false);
                // ROS_DEBUG("AFTER REPROJECT");
    			
                if (flag == false)
    			{
      				m_oldImage = img_frame.m_originalImage;
      				flag = true;
    			}

                cout<<"Pre function"<<endl;
                //ROS_DEBUG("BEFORE DETECTLED");
                DetectLED(img_frame.m_originalImage);
                //ROS_DEBUG("AFTER DETECTLED");
                cout<<"Post function"<<endl;
                
                // image_pub.publish(cv_depthptr->toImageMsg());
                // now we should use ledDetector with image_frame.m_originalImage
                // I think this also does the job?:
                // pcl::PointXYZRGB point = organized_cloud->at(index.x,index.y);''

            }
        }
    }

    void LedDetector::getLedColor(){
    }

    void LedDetector::getLedLocation(){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        StereoPointCloudColor::Ptr organized_cloud(new StereoPointCloudColor);
        geometry_msgs::PointStamped location;
        geometry_msgs::Point index;
        cv::Mat src,flipped;
        cv::Mat_<float> disp, disp_flipped;
        cv::Mat_<double> Q, Q_flipped;

        bool valid_Q=false;
        bool new_color=false;
        bool new_disp=false;
    }

    //  LED detection algorithm

    void LedDetector::DetectLED(const cv::Mat &new_image) // small change here !!!! Since no changes on the image Vinayak suggested passing it as reference
    {

        cv::Mat diff, erod, erod_dil, erod_dil_gray, erod_dil_gray_thresh;

        // Finding the difference image
        diff = abs(new_image - m_oldImage);
        //cv::imshow("Difference Image", diff);

        // Get the BGR channels
        std::vector<cv::Mat> channels(3);
        cv::split(diff, channels);

        // Add blue to green(standard conversion to grayscale is .3R + .6G + .1B) which is less than ideal.
        channels[1] = channels[1] + channels[0]/4;
        channels[1] = channels[1] + channels[0]/12;
        cv::merge(channels, diff);

        // Eroding to the difference in image to get rid of noise
        cv::erode(diff, erod, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
        //cv::imshow("Eroded Image", erod);

        // Dialate difference, to reflect size of led in the original image.
        cv::dilate(erod, erod_dil, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
        //cv::imshow("Eroded and dialated Image", erod_dil);

        // Converting difference image to grayscale
        cv::cvtColor(erod_dil, erod_dil_gray, CV_BGR2GRAY);

        // Binary thresholding of grayscale difference image
        cv::threshold(erod_dil_gray, erod_dil_gray_thresh, 45, 255, CV_THRESH_BINARY);
        //cv::imshow("Thresholded Image", erod_dil_gray_thresh);

        // Find the contours
        // Use if condition only to display
        if (countNonZero(erod_dil_gray_thresh) > 1)
        {
            cv::Mat contourOutput = erod_dil_gray_thresh.clone();
            cv::findContours(contourOutput, m_gradientContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
        }

        std::vector<std::vector<cv::Point> >contours_poly(m_gradientContours.size());
        //std::vector<cv::Rect>boundRect(contours.size());
        std::vector<cv::Point2f>center(m_gradientContours.size());
        std::vector<cv::Point2f>points;

        for( int i = 0; i< m_gradientContours.size(); i++ )
        {
            approxPolyDP( cv::Mat(m_gradientContours[i]), contours_poly[i], 3, true );
          //boundRect[i] = boundingRect(cv::Mat(contours_poly[i]));
        }

        //cv::Mat drawing = cv::Mat::zeros(new_image.size(), CV_8UC3);      //Use this to display the contours on an empty background
        for( int i = 0; i< m_gradientContours.size(); i++ )
        {
            cv::Scalar color = cv::Scalar( m_randomGen.uniform(0, 255), m_randomGen.uniform(0,255), m_randomGen.uniform(0,255) );
            drawContours(new_image, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
          //rectangle(new_image, boundRect[i].tl(), boundRect[i].br(), color, 1, 8, 0 );
        }
        for (int i = 0, j = 0; j < m_gradientContours.size(); j++)
        {
            cv::Moments moment = cv::moments((cv::Mat)m_gradientContours[j]);
            if (moment.m00)
            {
                points.push_back(cv::Point2f(moment.m10/moment.m00,moment.m01/moment.m00));
                cv::Vec3b pixel_value =  m_cvDepthPtr->image.at<cv::Vec3b>(points[i].y,points[i].x);
                std_msgs::Int32MultiArray rgb;
                geometry_msgs::Point pixelCoordinates;
                pixelCoordinates.x = points[i].x;
                pixelCoordinates.y = points[i].y;
              //pixelCoordinates.z = 0;
                rgb.data.clear();
                for (int iter=2;iter>=0;iter--)
                    rgb.data.push_back(pixel_value.val[iter]);
                m_imageRGBpub.publish(rgb);
                m_imageXYZpub.publish(pixelCoordinates);
                i++;
            }
        }
        cv::imshow("Raw Image with Contours", new_image);
        cv::waitKey(3);
        m_oldImage = new_image;
    }
        

    /**  
      *  Function to convert 2D pixel point to 3D point by extracting point
      *  from PointCloud2 corresponding to input pixel coordinate. This function
      *  can be used to get the X,Y,Z coordinates
      *  Deprecated
      */
    void pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const int u, const int v, geometry_msgs::Point &p)
    {
        // get width and height of 2D point cloud data
        int width = pCloud.width;
        int height = pCloud.height;

        // Convert from u (column / width), v (row/height) to position in array
        // where X,Y,Z data starts
        int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

        // compute position in array where x,y,z data start
        int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
        int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
        int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

        float X = 0.0;
        float Y = 0.0;
        float Z = 0.0;

        memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
        memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
        memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

        // put data into the point p
        p.x = X;
        p.y = Y;
        p.z = Z;
    }

}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "led_detection");
    ros::NodeHandle nh;//rosHandle?

    src_qual1_task::LedDetector   good(nh);
    good.detectLed();

    good.m_multisenseImagePtr = new src_perception::MultisenseImage(nh);
    //good.m_multisensePcPtr = new src_perception::MultisensePointCloud(nh);

    //multisense_Image.giveImages(disp)
    while (ros::ok())
    {
        good.detectLed();
        ros::spinOnce();
    }
    return 0;
}
