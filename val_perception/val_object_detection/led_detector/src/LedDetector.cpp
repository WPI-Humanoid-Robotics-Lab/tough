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
    this->kdtreeInit();
}

LedDetector::~LedDetector() {
    delete m_multisenseImagePtr;
}

void LedDetector::kdtreeInit()
{
    alglib::real_2d_array centers = "[[2642.5285714285715, 357.9714285714284, 301.84285714285716], [2686.825, -555.0, 265.4125], [2602.12, -62.4, -471.85999999999996], [2831.98, 398.6, 17.74000000000001], [2811.842857142857, -531.3000000000001, 20.25714285714284], [2652.1285714285714, -250.0, 289.57142857142856], [2492.15, -552.05, 382.625], [2515.383333333333, 432.5, 379.46666666666664], [2556.7400000000002, -465.5, -454.7799999999999], [2574.94, 340.24, -456.9800000000001], [2661.1375000000003, 122.87499999999997, 288.4375], [2806.057142857143, -616.8142857142857, 86.84285714285714], [2447.357142857143, 364.0571428571428, 419.1714285714286], [2493.85, -476.30000000000007, 382.55], [2709.8, 428.98, 262.1], [2832.45, 478.775, 83.125], [2650.4285714285716, -322.4428571428572, 289.7857142857143], [2811.633333333333, -617.1666666666666, -45.366666666666646], [2558.5, -476.67999999999995, 343.46000000000004], [2831.616666666667, 396.29999999999995, 83.51666666666667], [2834.7000000000003, 481.96000000000004, 17.279999999999973], [2449.4857142857145, 437.42857142857144, 418.9857142857143], [2813.6857142857143, -532.0142857142856, -45.72857142857143], [2577.6800000000003, 358.64, 341.02], [2810.0142857142855, -615.4571428571428, 20.32857142857145], [2688.4666666666667, -482.45000000000005, 265.05], [2727.9, 196.85000000000005, 248.0], [2430.1000000000004, -480.20000000000005, 420.8], [2580.1, 431.525, 340.4], [2512.0333333333333, 362.5333333333333, 380.1166666666667], [2623.9666666666667, -481.9666666666667, 304.3666666666667], [2716.0, -328.4333333333333, 250.1], [2707.15, 356.54999999999995, 262.625], [2621.8, -554.2666666666667, 304.5], [2646.1, 427.4666666666667, 300.8666666666667], [2717.0, -249.43333333333337, 250.43333333333334], [2832.9333333333334, 401.20000000000005, -48.26666666666668], [2663.4, 196.9, 288.04999999999995], [2557.3, -554.25, 343.55], [2726.95, 118.75, 249.05], [2427.25, -550.0, 421.9], [2833.8999999999996, 489.29999999999995, -48.19999999999999], [2811.1000000000004, -535.4, 85.7]]";
    alglib::ae_int_t nx = 3;
    alglib::ae_int_t ny = 0;
    alglib::ae_int_t normtype = 2;
    alglib::integer_1d_array tag;
    tag= "[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42]";
    kdtreebuildtagged(centers,tag, nx, ny, normtype,this->kdt);
    return;
 }

int LedDetector::getMode(int daArray[], int iSize)
{
    int* ipRepetition = new int[iSize];
    for (int i = 0; i < iSize; ++i) {
        ipRepetition[i] = 0;
        int j = 0;
        bool bFound = false;
        while ((j < i) && (daArray[i] != daArray[j])) {
            if (daArray[i] != daArray[j]) {
                ++j;
            }
        }
        ++(ipRepetition[j]);
    }
    int iMaxRepeat = 0;
    for (int i = 1; i < iSize; ++i) {
        if (ipRepetition[i] > ipRepetition[iMaxRepeat]) {
            iMaxRepeat = i;
        }
    }
    delete [] ipRepetition;
    return daArray[iMaxRepeat];
}

void LedDetector::errorCorrection(std::vector<srcsim::Console> &data, std::vector<double> &pos)
{
    std::cout << "Check Point A" <<std::endl;
    int tags [3] = {100,100,100};

    alglib::ae_int_t k;
    int winnerTag;
    std::cout << "Check Point B" <<std::endl;
    for(int m =0; m < data.size();m++)
    {
        alglib::integer_1d_array tag_r;
        double x_p = data[m].x*1000;
        double y_p = data[m].y*1000;
        double z_p = data[m].z*1000;
        std::ostringstream temp;
        temp<<"[";
        temp<<x_p;
        temp<<",";
        temp<<y_p;
        temp<<",";
        temp<<z_p;
        temp<<"]";
        ROS_INFO(temp.str().c_str());
        alglib::real_1d_array point(temp.str().c_str());
        ROS_INFO(temp.str().c_str());
//        point ="[2686.825, -555.0, 265.4125]";
        k = alglib::kdtreequeryknn(kdt, point, 1);
        kdtreequeryresultstags(kdt,tag_r );
        tags[m] = int(tag_r[0]);
    }
    winnerTag = this->getMode(tags,3);
    if (tags[0] == winnerTag )
    {
        pos.push_back(data[0].x + error_x[winnerTag]/1000.0);
        pos.push_back(data[0].y + error_y[winnerTag]/1000.0);
        pos.push_back(data[0].z + error_z[winnerTag]/1000.0);
    }
    else
    {
        pos.push_back(data[1].x + error_x[winnerTag]);
        pos.push_back(data[1].y + error_y[winnerTag]);
        pos.push_back(data[1].z + error_z[winnerTag]);

    }
     std::cout << "Check Point G  " <<std::endl;
    return;
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

            meanR += m_readings[i].r;
            meanG += m_readings[i].g;
            meanB += m_readings[i].b;
        }
        std::vector<double> pos;
        this->errorCorrection(m_readings,pos);


        meanR = meanR/m_readings.size();
        meanG = meanG/m_readings.size();
        meanB = meanB/m_readings.size();
        ROS_INFO("%.4f, %.4f, %.4f", pos[0], pos[1], pos[2]);
        msg.x = pos[0];
        msg.y = pos[1];
        msg.z = pos[2];
        msg.r = meanR < 0.9 ? 0 : 1;
        msg.g = meanG < 0.9 ? 0 : 1;
        msg.b = meanB < 0.9 ? 0 : 1;
        pos.clear();
        m_imageRGBXYZpub.publish(message);
        ROS_INFO("Publishing Message");
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
