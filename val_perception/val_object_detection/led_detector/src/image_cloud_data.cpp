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
bool readData = false;
bool lightDetected = false;

ros::Publisher *m_lightPub;
cv::Mat_<double> qMatrix;
const int maxImages = 25;
tf::Vector3 previousPoint;

std::vector<cv::Mat> colorImageVect;
std::vector<cv::Mat> tempColorImageVect;
std::vector <cv::Mat> disparityImageVect;
std::vector<std::thread> threadsVect;
std::vector<srcsim::Console> outputMessages(maxImages);


bool getLight(int index,geometry_msgs::Point &pixelCoordinates, bool tempImage=false){
    cv::Mat newImage;
    if (tempImage)
        newImage = tempColorImageVect[index];
    else
        newImage = colorImageVect[index];
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
    cv::split(newImage,inMsgChannel);
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
        drawContours(newImage, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
    }

    for (int i = 0, j = 0; j < gradientContours.size(); j++)
    {
        cv::Moments moment = cv::moments((cv::Mat)gradientContours[j]);

        if (moment.m00)
        {
            // Finding x,y coordinates of centroid of the contour
            points.push_back(cv::Point2f(moment.m10/moment.m00,moment.m01/moment.m00));
//            ROS_INFO("GetLight");

            // Assigning x,y xo-ordinates in image frame
            pixelCoordinates.x = points[i].x;
            pixelCoordinates.y = points[i].y;
//            ROS_INFO("Pixel Coordinates  x: %.2f y: %.2f",pixelCoordinates.x, pixelCoordinates.y);
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
    src_perception::PointCloudHelper::generateOrganizedRGBDCloud(disparityImageVect[index],colorImageVect[index],qMatrix,organized_cloud);
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
//    ROS_INFO("Updating Message at index :%d",index );
    //    m_lightPub->publish(msg);
//    outputMessages.insert(outputMessages.begin()+index,msg);
    outputMessages[index].x=msg.x;
    outputMessages[index].y=msg.y;
    outputMessages[index].z=msg.z;
    outputMessages[index].r=msg.r;
    outputMessages[index].g=msg.g;
    outputMessages[index].b=msg.b;
    return poseXYZDetected;
}

void imageCB(sensor_msgs::ImageConstPtr msg )
{

    if (!lightDetected){
        cv_bridge::CvImagePtr   cv_ptr;
        cv::Mat image;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        image=cv_ptr->image.clone();
        tempColorImageVect.push_back(image);
        return;
    }

    if (readData){
        return;
    }

    if(flag)
    {
        cv_bridge::CvImagePtr   cv_ptr;
        cv::Mat image;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        if(colorImageVect.size() != maxImages)
        {
            image=cv_ptr->image.clone();
            colorImageVect.push_back(image);
            flag = !flag;
        }

    }
}

void dispCB(stereo_msgs::DisparityImageConstPtr msg )
{
    if (readData || !lightDetected)
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

        if(disparityImageVect.size() != maxImages)
        {
            disparityImageVect.push_back(disparity_);
            flag = !flag;
            if(disparityImageVect.size() == maxImages)
                readData = true;
        }



    }
}

void runner(int i){
//    ROS_INFO("starting thread %d", i);
    geometry_msgs::Point pnt;
    if(getLight(i,pnt))
        getPoseRGB(i,pnt);
}

int main(int argc, char **argv)
{
    //Initialize ROS system
    ros::init (argc,argv, "image_disp_data");
    ros::NodeHandle nh;
    //setup subscriber and publisher
    ros ::Subscriber colorImageSub = nh.subscribe("/multisense/camera/left/image_rect_color",1,imageCB);
    ros ::Subscriber disparityImageSub = nh.subscribe("/multisense/camera/disparity",1,dispCB);
    ros::Publisher temp = nh.advertise<srcsim::Console>("/srcsim/qual1/light",1);
    //the pointer is public so had to use a temp variable
    m_lightPub = &temp;
    ros::Rate loopRate = 100;
    ros::Time begin = ros::Time::now();
    ros::Time end;
    float time_taken ;
    float minimumLightDistance = 0.02;

    //populate the qMatrix. it wont change with every image so no point in calculatin it everytime
//    src_perception::MultisenseImage *m_multisenseImagePtr = new src_perception::MultisenseImage(nh);
//    m_multisenseImagePtr->giveQMatrix(qMatrix);
    qMatrix=cv::Mat_<double>(4,4,0.0);
    qMatrix(0,0) =  610.1799470098168 * -0.07;
    qMatrix(1,1) =  610.1799470098168 * -0.07;
    qMatrix(0,3) = -610.1799470098168 * 512.5 * -0.07;
    qMatrix(1,3) = -610.1799470098168 * 272.5 * -0.07;
    qMatrix(2,3) =  610.1799470098168 * 610.1799470098168 * -0.07;
    qMatrix(3,2) = -610.1799470098168;
    qMatrix(3,3) =  0.0f;


    while(ros::ok())
    {
        //wait for the light to be switched on
        //assuming light detection will work soon
        if (!lightDetected){
            geometry_msgs::Point pnt;
            size_t index = tempColorImageVect.size()-1;
            if (index == -1){
                ros::spinOnce();
                continue;
            }
            lightDetected = getLight(index,pnt, true);
            std::string output = lightDetected ? "Light Detected": "******Not detected";
//            ROS_INFO(output.c_str());
            end  = ros::Time::now();
            time_taken = end.toSec() - begin.toSec();
            begin = ros::Time::now();
            if(lightDetected) {
                tempColorImageVect.clear();
            }
            else {
                ros::spinOnce();
                continue;
            }
        }


           //read_data is true when the vectors are full
        if(readData)
        {
            end  = ros::Time::now();
            time_taken = end.toSec() - begin.toSec();
//            ROS_INFO("Time taken = %lf",time_taken );

            outputMessages = std::vector<srcsim::Console>(maxImages);

            //create threads for every value in vector
            for (size_t i=0; i< colorImageVect.size(); i++){
                threadsVect.push_back(std::thread(runner, i));
            }
            ROS_INFO("Running %d threads", threadsVect.size());
            //wait till all threads finish
            for(size_t i=0; i<threadsVect.size(); i++)
                threadsVect[i].join();

            threadsVect.clear();
            // calculating mean of the results
            float meanX=0.0, meanY=0.0,meanZ=0.0;
            bool skipIteration = false;

            for (size_t i=0; i< outputMessages.size(); i++){
                if (outputMessages[i].x == 0.0 && outputMessages[i].y == 0.0 && outputMessages[i].z == 0.0){
                    skipIteration = true;
                    ROS_INFO("Skipping this iteration");
                    break;
                }
                meanX += outputMessages[i].x;
                meanY += outputMessages[i].y;
                meanZ += outputMessages[i].z;
//                ROS_INFO("output message %d x:%.4f y:%.4f z:%.4f", i, outputMessages[i].x, outputMessages[i].y, outputMessages[i].z);
            }


            srcsim::Console msg;
            msg.x = meanX/outputMessages.size();
            msg.y = meanY/outputMessages.size();
            msg.z = meanZ/outputMessages.size();
            msg.r = outputMessages[0].r;
            msg.g = outputMessages[0].g;
            msg.b = outputMessages[0].b;


//            tf::Vector3 currentPoint(msg.x, msg.y, msg.z);
            // publish the message only if it is not the same light
            if (!skipIteration){
                ROS_INFO("Publishing message x:%.4f y:%.4f z:%.4f", msg.x, msg.y, msg.z);
                m_lightPub->publish(msg);
            }
//            previousPoint = currentPoint;

            begin = ros::Time::now();
            lightDetected=false;
            // clearing vectors to be used for next light
            colorImageVect.clear();
            disparityImageVect.clear();

            readData = false;
        }
        // to make sure callback is called

        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;

}
