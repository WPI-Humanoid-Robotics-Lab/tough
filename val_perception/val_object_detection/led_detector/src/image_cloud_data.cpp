#include <ros/ros.h>
#include <val_common/val_common_names.h>
#include <perception_common/MultisenseImage.h>
#include <perception_common/MultisensePointCloud.h>
#include <perception_common/PointCloudHelper.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include "srcsim/Console.h"

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <memory>
#include <thread>

bool flag = true;
bool read_data = false;

ros::Publisher *m_lightPub;
cv::Mat qMatrix;
const int maxImages = 5;

std::vector<cv::Mat> image_data;
std::vector <cv::Mat> disp_data;
std::vector<std::thread> threadsVect;
std::vector<srcsim::Console> outputMessages(maxImages);


bool getLight(int index,geometry_msgs::Point &pixelCoordinates){
    cv::Mat new_image = image_data[index];
    ROS_INFO("Size of image_data after %d", image_data.size());
    std::vector<std::vector<cv::Point> > gradientContours;
    cv::RNG randomGen;
    bool lightXYDetected = false;
    cv::vector<cv::Mat> inMsgChannel(3),hsv(3);
    cv::Mat inThresh,hist,backProject,drawing;
    int histSize[] = {180,256,256};
    float hRange[] = {0,180};        //hue range
    float sRange[] = {0,256};        //saturation range
    float vRange[] = {0,256};        //values range
    int channels[] = {0,1,2};
    const float* histRange[] = { hRange, sRange,vRange};
    cv::split(new_image,inMsgChannel);
    ROS_INFO("Size of image_data after %d", image_data.size());
    // RBG channel threshold, 180 is working,lower values gives some noise
    for(int iter=0;iter<3;iter++)
        cv::threshold(inMsgChannel[iter],inMsgChannel[iter],180,255,0);


    cv::merge(inMsgChannel,inThresh);
    cv::cvtColor(inThresh,inThresh,CV_BGR2HSV);
    cv::calcHist(&inThresh,1,channels,cv::Mat(),hist,2,histSize,histRange,true,false);
    cv::split(hist,hsv);

    for(int iter=0;iter<3;iter++)
        cv::normalize(hsv[iter],hsv[iter],0,hsv[iter].rows,cv::NORM_MINMAX,-1,cv::Mat());

    cv::merge(hsv,hist);
    cv::calcBackProject(&inThresh,1,channels,hist,backProject,histRange,1,true);
    cv::bitwise_not(backProject,backProject);
    drawing = backProject.clone();
    cv::threshold(drawing,drawing,180,255,0);

    // Find the contours of the LED
    if (countNonZero(drawing) > 1)
    {
        cv::Mat contourOutput = drawing.clone();
        cv::findContours(contourOutput, gradientContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    }

    std::vector<std::vector<cv::Point> >contours_poly(gradientContours.size());       // For polygonal contour
    std::vector<cv::Point2f>points;

    // Approximate the contours to a polygonal shape
    for( int i = 0; i< gradientContours.size(); i++ )
    {
        approxPolyDP( cv::Mat(gradientContours[i]), contours_poly[i], 3, true );
    }

    // Draws the contours onto the original image. Comment this section if no need to draw contours
    for( int i = 0; i< gradientContours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( randomGen.uniform(0, 255), randomGen.uniform(0,255), randomGen.uniform(0,255) );
        drawContours(new_image, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
    }

    for (int i = 0, j = 0; j < gradientContours.size(); j++)
    {
        cv::Moments moment = cv::moments((cv::Mat)gradientContours[j]);

        if (moment.m00)
        {
            // Finding x,y coordinates of centroid of the contour
            points.push_back(cv::Point2f(moment.m10/moment.m00,moment.m01/moment.m00));
            ROS_INFO("GetLight");

            // Assigning x,y xo-ordinates in image frame
            pixelCoordinates.x = points[i].x;
            pixelCoordinates.y = points[i].y;
            ROS_INFO("Pixel Coordinates  x: %.2f y: %.2f",pixelCoordinates.x, pixelCoordinates.y);
            lightXYDetected = true;
            i++;
        }
    }

    // Comment if no need to display raw image
    //    cv::imshow("Raw Image with Contours", new_image);

    // cv::waitKey(3);//
    return lightXYDetected;
}

bool getPoseRGB(int index, geometry_msgs::Point &pixelCoordinates)
{
    bool poseXYZDetected = false;
    tf::TransformListener listener;
    src_perception::StereoPointCloudColor::Ptr organized_cloud(new src_perception::StereoPointCloudColor);
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::StampedTransform stampedTransform;
    tf::Quaternion orientation;
    pcl::PointXYZRGB pcl_point;
    srcsim::Console msg;

    try
    {
        listener.waitForTransform("/world", "/left_camera_optical_frame", ros::Time(0), ros::Duration(3.0));
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::spinOnce();
        return false;
    }

    // Obtaining a stereo point cloud for Z position and RGB values
    src_perception::PointCloudHelper::generateOrganizedRGBDCloud(disp_data[index],image_data[index],qMatrix,organized_cloud);
    pcl_point = organized_cloud->at(pixelCoordinates.x, pixelCoordinates.y);

    geometry_msgs::PointStamped light_centroid;
    light_centroid.header.frame_id= "left_camera_optical_frame";
    light_centroid.header.stamp = ros::Time::now();
    light_centroid.point.x = pcl_point.x;
    light_centroid.point.y = pcl_point.y;
    light_centroid.point.z = pcl_point.z;
    geometry_msgs::PointStamped light_centroid_head;

    try{

        listener.transformPoint("/head",light_centroid, light_centroid_head);
    }
    catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::spinOnce();
        return false;
    }

    transform.setOrigin( tf::Vector3(light_centroid_head.point.x, light_centroid_head.point.y, light_centroid_head.point.z) );
    orientation.setRPY(0, 0, 0);
    transform.setRotation(orientation);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "head", "LED_frame")); //Co-ordinates wrt left_camera_optical_frame

    msg.x = light_centroid_head.point.x;
    msg.y = light_centroid_head.point.y;
    msg.z = light_centroid_head.point.z;

    // Assign RGB values to ROS message to be published. Getting it from the cloud and not the image, values returned are 1 or 0.
    // If the value is more than 0.7 consider it as 1
    msg.r = (int)((pcl_point.r/255.0)+0.3);
    msg.g = (int)((pcl_point.g/255.0)+0.3);
    msg.b = (int)((pcl_point.b/255.0)+0.3);


    // If there is no LED turned on, then just don't detect anything
    if (msg.r == 0.0 && msg.g == 0 && msg.b == 0){
        poseXYZDetected = false;
        return poseXYZDetected;
    }

    poseXYZDetected = true;
    ROS_INFO("Publishing Message");
//    m_lightPub->publish(msg);
    outputMessages.insert(outputMessages.begin()+index,msg);
    return poseXYZDetected;
}

void imageCB(sensor_msgs::ImageConstPtr msg )
{

    if (read_data)
        return;

    if(flag)
    {
        int source_type = cv_bridge::getCvType(msg->encoding);
        cv_bridge::CvImagePtr   cv_ptr;
        cv::Mat image;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        if(image_data.size() != maxImages)
        {
            image=cv_ptr->image.clone();
            image_data.push_back(image);
            flag = !flag;
        }

    }
}

void dispCB(stereo_msgs::DisparityImageConstPtr msg )
{
    if (read_data)
        return;

    if(!flag)
    {
        std_msgs::Header		disp_header;
        cv::Mat disparity_;
        bool new_disp;
        uint8_t depth=sensor_msgs::image_encodings::bitDepth(msg->image.encoding);  //the size of the disparity data can be 16 or 32
        if (depth == 32)
        {
            cv::Mat_<float> disparity(msg->image.height, msg->image.width,
                                      const_cast<float*>(reinterpret_cast<const float*>(&msg->image.data[0])));

            disparity_=disparity.clone();
            new_disp = true;
            disp_header=msg->image.header;
        }
        else if(depth == 16)
        {
            cv::Mat_<uint16_t> disparityOrigP(msg->image.height, msg->image.width,
                                              const_cast<uint16_t*>(reinterpret_cast<const uint16_t*>(&msg->image.data[0])));
            cv::Mat_<float>   disparity(msg->image.height, msg->image.width);
            disparity = disparityOrigP / 16.0f;
            disparity_=disparity.clone();
            new_disp = true;
            disp_header=msg->image.header;
        }

        if (!new_disp)
            return;

        if(disp_data.size() != maxImages)
        {
            disp_data.push_back(disparity_);
            flag = !flag;
            if(disp_data.size() == maxImages)
                read_data = true;
        }



    }
}

void runner(int i){
    geometry_msgs::Point pnt;
    getLight(i,pnt);
    getPoseRGB(i,pnt);
}

int main(int argc, char **argv)
{
    ros::init (argc,argv, "image_disp_data");
    ros::NodeHandle nh;
    ros ::Subscriber image_data_sub = nh.subscribe("/multisense/camera/left/image_rect_color",0,imageCB);
    ros ::Subscriber disp_data_sub = nh.subscribe("/multisense/camera/disparity",0,dispCB);
    ros::Publisher temp = nh.advertise<srcsim::Console>("/srcsim/qual1/light",1);
    m_lightPub = &temp;
    ros::Time begin = ros::Time::now();
    ros::Time end;
    float time_taken ;
    src_perception::MultisenseImage *m_multisenseImagePtr = new src_perception::MultisenseImage(nh);
    m_multisenseImagePtr->giveQMatrix(qMatrix);


    while(ros::ok())
    {
        if(read_data)
        {
            end  = ros::Time::now();
            time_taken = end.toSec() - begin.toSec();
            ROS_INFO("Time taken = %lf",time_taken );

            for (size_t i=0; i< image_data.size(); i++){
                threadsVect.push_back(std::thread(runner, i));
            }
            for(size_t i=0; i<threadsVect.size(); i++)
                threadsVect[i].join();

            float meanX, meanY,meanZ, r, g, b;

            for (size_t i=0; i< outputMessages.size(); i++){

            }

            begin = ros::Time::now();
            read_data = false;

            image_data.clear();
            disp_data.clear();
            threadsVect.clear();

        }

        ros::spinOnce();
    }
    return 0;

}
