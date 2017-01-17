#include <led_detector/LedDetector.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <perception_common/periodic_snapshotter.h>

using namespace src_qual1_task;

LedDetector::LedDetector(ros::NodeHandle nh):m_laserCloudSub(nh.subscribe("/assembled_cloud2",1, &LedDetector::laserCloudCallBack, this)) {
    m_multisenseImagePtr = new src_perception::MultisenseImage(nh);
    m_randomGen= cv::RNG(12345);
    m_imageRGBXYZpub = nh.advertise<led_detector::LedPositionColor>("/detect/light/rgbxyz", 1);
    m_lightPub = nh.advertise<srcsim::Console>("/srcsim/qual1/light",1);
    m_multisenseImagePtr->giveQMatrix(m_qMatrix);
    this->kdtreeInit();
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    m_cloud= temp_cloud;
}

LedDetector::~LedDetector() {
    delete m_multisenseImagePtr;
}

void LedDetector::kdtreeInit()
{
    alglib::real_2d_array centers = "[[2649.1555555555556, -161.05555555555557, 289.97777777777776], [2684.6555555555556, 519.7666666666668, 265.5333333333333], [2810.5, -615.97, 0.6200000000000045], [2596.730769230769, -7.792307692307694, -471.1000000000001], [2559.6944444444443, -491.5944444444445, 343.2], [2810.0375, 577.4375, -45.17500000000001], [2649.35, 209.96, 289.88], [2512.0333333333333, 362.5333333333333, 380.1166666666667], [2553.6857142857143, -470.0142857142857, -454.37142857142857], [2809.9764705882353, -563.3647058823528, 86.25294117647057], [2553.225, 429.06249999999994, -454.0375000000001], [2821.395, 458.09, 84.71999999999998], [2691.5066666666667, -473.52, 264.6733333333333], [2815.8125, -437.05, 85.52499999999999], [2496.1470588235293, -490.5941176470588, 382.2470588235294], [2457.438888888889, 518.0611111111111, 402.48333333333335], [2623.704761904762, 474.6190476190476, 304.1904761904762], [2831.5, 395.31, -15.110000000000014], [2816.1375, -528.5625, -46.006249999999966], [2709.2166666666667, 310.9666666666667, 254.99166666666667], [2699.146153846154, -374.7076923076923, 260.32307692307694], [2627.6809523809525, -438.04285714285714, 303.7809523809524], [2501.4714285714285, 438.5857142857143, 381.42857142857144], [2649.857692307692, -266.71923076923076, 289.80384615384617], [2466.1222222222223, -394.35, 401.4611111111111], [2693.9285714285716, 440.3142857142857, 264.3071428571429], [2554.788888888889, 518.6666666666666, 343.84444444444443], [2818.475, -436.56874999999997, -13.456250000000011], [2430.99375, -488.74999999999994, 421.225], [2674.3, 122.05, 280.56], [2665.107692307692, -557.3461538461538, 277.65384615384613], [2563.1555555555556, -394.78888888888883, 342.87777777777774], [2440.2772727272727, 415.98636363636376, 420.16363636363644], [2643.94, 320.86000000000007, 295.74], [2817.2652173913043, 488.6652173913042, -6.15217391304347], [2713.9777777777776, -160.64444444444445, 250.79999999999998], [2815.4066666666668, -526.0333333333334, 19.81333333333336], [2806.33125, 577.83125, 53.8125], [2714.39, 210.07, 250.47], [2714.8076923076924, -242.15384615384616, 250.66923076923075], [2560.7375, -376.3374999999999, -455.2250000000001], [2573.7833333333333, 337.68333333333334, -456.8333333333332], [2567.9470588235295, 416.16470588235296, 342.06470588235294]]";
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
    int tags [m_readingThreshold] = {100} ;

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
    winnerTag = getMode(tags,data.size());
    float meanX=0.0,meanY=0.0,meanZ=0.0;
    int n_elements = 0;

    for (int m =0; m< data.size();m++ )
    {

        if (tags[m] == winnerTag )
        {
            meanX += data[m].x + error_x[winnerTag]/1000.0;
            meanY += data[m].y + error_y[winnerTag]/1000.0;
            meanZ += data[m].z + error_z[winnerTag]/1000.0;
            n_elements++;
        }
    }

    pos.push_back(meanX/n_elements);
    pos.push_back(meanY/n_elements);
    pos.push_back(meanZ/n_elements);
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
        light_centroid_head.point.x = pos[0];
        light_centroid_head.point.y = pos[1];
        light_centroid_head.point.z = pos[2];

        if(getNearestPoint(light_centroid_head, 10))
            ROS_INFO("Updated the point with lidar data");
        msg.x = light_centroid_head.point.x;
        msg.y = light_centroid_head.point.y;
        msg.z = light_centroid_head.point.z;
        msg.r = message.color.r;
        msg.g = message.color.g;
        msg.b = message.color.b;
        pos.clear();
        m_imageRGBXYZpub.publish(message);
        ROS_INFO("Publishing Message");
        m_lightPub.publish(msg);
        m_readings.clear();
        return true;
    }
    return false;
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
bool LedDetector::getNearestPoint(geometry_msgs::PointStamped &point, int K)
{
    if (m_cloud->empty()){
        ROS_INFO("Point cloud is empty");
        return false;
    }

    // store the frameID of original point so that we can retransform the output to that frame
    std::string originalFrame = point.header.frame_id;
    point.header.stamp = ros::Time(0);
    tf::TransformListener listener;

//     transform the point to world frame
    if (originalFrame != VAL_COMMON_NAMES::WORLD_TF){
        try{
            listener.waitForTransform("/head", "/world", ros::Time(0), ros::Duration(3));
            listener.transformPoint(VAL_COMMON_NAMES::WORLD_TF,point, point);
        }
        catch(tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            return false;
        }
    }

    // get a kdtree for searching point
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (m_cloud);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud);

    // index of points in the pointcloud
    std::vector<int> pointIdxNKNSearch(K);
    //squared distance of points
    std::vector<float> pointNKNSquaredDistance(K);

    // convert input point into PCL point for searching
    pcl::PointXYZ searchPoint;
    searchPoint.x = point.point.x;
    searchPoint.y = point.point.y;
    searchPoint.z = point.point.z;

    std::cout << "K nearest neighbor search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with K=" << K << std::endl;

    float meanX, meanY, meanZ;

    //perform nearestKsearch
    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
            meanX += cloud->points[ pointIdxNKNSearch[i] ].x;
            meanY += cloud->points[ pointIdxNKNSearch[i] ].y;
            meanZ += cloud->points[ pointIdxNKNSearch[i] ].z;
            std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x
                      << " " << cloud->points[ pointIdxNKNSearch[i] ].y
                      << " " << cloud->points[ pointIdxNKNSearch[i] ].z
                      << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
        }
        point.point.x = meanX = meanX/pointIdxNKNSearch.size();
        point.point.y = meanY = meanY/pointIdxNKNSearch.size();
        point.point.z = meanZ = meanZ/pointIdxNKNSearch.size();

    }

    point.header.stamp = ros::Time(0);

    //transform the point back to its original frame, if required
    if (originalFrame != VAL_COMMON_NAMES::WORLD_TF){
        try{
            listener.transformPoint(originalFrame, point, point);
        }
        catch(tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            return false;
        }
    }

    return true;
}

void LedDetector::laserCloudCallBack(const sensor_msgs::PointCloud2 &msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*m_cloud);
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "led_detection");
    ros::NodeHandle nh;
    src_qual1_task::LedDetector   led_detect(nh);
    //ROS_INFO_STREAM(led_detect.m_qMatrix);
    ros::Time start= ros::Time::now();
    unsigned int numberOfLightsDetected=0;
    while (ros::ok() and numberOfLightsDetected < 20) {
        bool success = led_detect.detectLight();
        if (success){
            numberOfLightsDetected++;
        }
//        std::string output = success ? "Button Detected in %0.6f secs" : "Detection Failed!!! in %0.6f secs";

//        ROS_INFO(output.c_str(), (ros::Time::now() - start).toSec());//tbw
        ros::spinOnce();
        start= ros::Time::now();
    }
    ros::Duration(8).sleep();

    return 0;
}
