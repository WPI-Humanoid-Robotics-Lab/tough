#include <led_detector/LedDetector.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

using namespace src_qual1_task;

LedDetector::LedDetector(ros::NodeHandle nh) {
    m_multisenseImagePtr = new src_perception::MultisenseImage(nh);
    m_randomGen= cv::RNG(12345);
    m_imageRGBXYZpub = nh.advertise<led_detector::LedPositionColor>("/detect/light/rgbxyz", 1);
    m_lightPub = nh.advertise<srcsim::Console>("/srcsim/qual1/light",1);
    m_multisenseImagePtr->giveQMatrix(m_qMatrix);
}

LedDetector::~LedDetector() {
    delete m_multisenseImagePtr;
}

bool LedDetector::getLight(cv::Mat &new_image,geometry_msgs::Point &pixelCoordinates){
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
        cv::findContours(contourOutput, m_gradientContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    }

    std::vector<std::vector<cv::Point> >contours_poly(m_gradientContours.size());       // For polygonal contour
    std::vector<cv::Point2f>points;

    // Approximate the contours to a polygonal shape
    for( int i = 0; i< m_gradientContours.size(); i++ )
    {
        approxPolyDP( cv::Mat(m_gradientContours[i]), contours_poly[i], 3, true );
    }

    // Draws the contours onto the original image. Comment this section if no need to draw contours
    for( int i = 0; i< m_gradientContours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( m_randomGen.uniform(0, 255), m_randomGen.uniform(0,255), m_randomGen.uniform(0,255) );
        drawContours(new_image, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
    }

    for (int i = 0, j = 0; j < m_gradientContours.size(); j++)
    {
        cv::Moments moment = cv::moments((cv::Mat)m_gradientContours[j]);

        if (moment.m00)
        {
            // Finding x,y coordinates of centroid of the contour
            points.push_back(cv::Point2f(moment.m10/moment.m00,moment.m01/moment.m00));
            ROS_DEBUG("GetLight");

            // Assigning x,y xo-ordinates in image frame
            pixelCoordinates.x = points[i].x;
            pixelCoordinates.y = points[i].y;
            ROS_DEBUG("Pixel Coordinates  x: %.2f y: %.2f",pixelCoordinates.x, pixelCoordinates.y);
            lightXYDetected = true;
            i++;
        }
    }

    // Comment if no need to display raw image
    //    cv::imshow("Raw Image with Contours", new_image);

    // cv::waitKey(3);//
    return lightXYDetected;
}


bool LedDetector::getPoseRGB(ImageFrame &img_frame,geometry_msgs::Point &pixelCoordinates)
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
    src_perception::PointCloudHelper::generateOrganizedRGBDCloud(img_frame.m_disparityImage,img_frame.m_originalImage,img_frame.m_qMatrix,organized_cloud);
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

    message.position.x = msg.x = light_centroid_head.point.x;
    message.position.y = msg.y = light_centroid_head.point.y;
    message.position.z = msg.z = light_centroid_head.point.z;

    // Assign RGB values to ROS message to be published. Getting it from the cloud and not the image, values returned are 1 or 0.
    // If the value is more than 0.7 consider it as 1
    message.color.r = msg.r = (int)((pcl_point.r/255.0)+0.3);
    message.color.g = msg.g = (int)((pcl_point.g/255.0)+0.3);
    message.color.b = msg.b = (int)((pcl_point.b/255.0)+0.3);
    message.color.a = 1.0;

    // If there is no LED turned on, then just don't detect anything
    if (msg.r == 0.0 && msg.g == 0 && msg.b == 0){
        m_readings.clear();
        poseXYZDetected = false;
        return poseXYZDetected;
    }

    poseXYZDetected = true;

    if(m_readings.size() < m_readingThreshold)
        m_readings.push_back(msg);
    else{
        // Publishing XYZ and RGB data
        float meanX, meanY, meanZ, meanR, meanG, meanB;
        for (size_t i = 0; i<m_readings.size(); i++){
            meanX += m_readings[i].x;
            meanY += m_readings[i].y;
            meanZ += m_readings[i].z;
            meanR += m_readings[i].r;
            meanG += m_readings[i].g;
            meanB += m_readings[i].b;
        }
        meanX = meanX/m_readings.size();
        meanY = meanY/m_readings.size();
        meanZ = meanZ/m_readings.size();
        meanR = meanR/m_readings.size();
        meanG = meanG/m_readings.size();
        meanB = meanB/m_readings.size();

        msg.x = meanX;
        msg.y = meanY;
        msg.z = meanZ;
        msg.r = meanR;
        msg.g = meanG;
        msg.b = meanB;

        m_imageRGBXYZpub.publish(message);
        m_lightPub.publish(msg);
        m_readings.clear();

    }
    return poseXYZDetected;
}

bool LedDetector::detectLight() {
    ImageFrame img_frame;
    geometry_msgs::Point pixelCoordinates;

    img_frame.m_qMatrix = this->m_qMatrix;

    if(m_multisenseImagePtr->giveImage(img_frame.m_originalImage))
        if(m_multisenseImagePtr->giveDisparityImage(img_frame.m_disparityImage))
            if(getLight(img_frame.m_originalImage, pixelCoordinates))
                return getPoseRGB(img_frame,pixelCoordinates);

    return false;

}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "led_detection");
    ros::NodeHandle nh;
    src_qual1_task::LedDetector   led_detect(nh);
    ros::Time start= ros::Time::now();
    while (ros::ok()) {
        bool success = led_detect.detectLight();
        std::string output = success ? "Button Detected in %0.6f secs" : "Detection Failed!!! in %0.6f secs";
        ROS_INFO(output.c_str(), (ros::Time::now() - start).toSec());//tbw
        ros::spinOnce();
        start= ros::Time::now();
    }

    return 0;
}
