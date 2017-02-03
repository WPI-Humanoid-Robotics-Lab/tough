#include <ros/ros.h>
#include <val_common/val_common_names.h>
#include <perception_common/MultisenseImage.h>
#include <perception_common/MultisensePointCloud.h>
#include <perception_common/PointCloudHelper.h>
#include <perception_common/periodic_snapshotter.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include "srcsim/Console.h"

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <thread>
#include "val_common/kdtree/alglibmisc.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


const bool DEBUG = true;
const int MAX_IMAGES = 25;

ros::Publisher *LIGHT_PUB;
cv::Mat_<double> q_MATRIX;
pcl::PointCloud<pcl::PointXYZ>::Ptr POINTCLOUD_PTR;

bool READ_DATA = false;
bool LIGHT_DETECTED = false;

std::vector<cv::Mat> COLOR_IMAGE_VECT;
std::vector<cv::Mat> TEMP_COLOR_IMAGE_VECT;
std::vector <cv::Mat> DISPARITY_IMAGE_VECT;
std::vector<std::thread> THREADS_VECT;
std::vector<srcsim::Console> OUTPUT_MESSAGES_VECT(MAX_IMAGES);


alglib::kdtree KD_TREE;
double ERROR_X[43] = {52.099999999999909, 78.499999999999773, -140.79999999999973, 25.099999999999909, 24.800000000000182, 83.000000000000455, 42.5, 47.749999999999773, -178.69999999999936, -189.00000000000045, 89.350000000000364, 48.200000000000273, -32.099999999999909, -1.3000000000004093, 88.800000000000182, 53.899999999999864, 71.400000000000091, 31.199999999999818, 4.5000000000004547, 42.299999999999955, 37.200000000000273, 0.3000000000001819, 19.400000000000091, 10.799999999999727, 34.300000000000182, 53.450000000000045, 86.200000000000045, -29.099999999999682, 49.099999999999909, -11.149999999999864, 44.799999999999727, 84.899999999999636, 39.700000000000045, 62.900000000000091, 73.400000000000091, 100.29999999999973, 20.799999999999727, 74.800000000000182, 54.900000000000091, 89.300000000000182, -4.7500000000002274, 33.099999999999454, 39.800000000000182};
double ERROR_Y[43] = {-0.59999999999996589, -24.300000000000011, -0.70000000000001705, -4.6000000000000227, -11.199999999999932, -16.400000000000006, -17.100000000000023, 1.0999999999999943, 31.399999999999977, -32.0, -4.1500000000000057, -15.200000000000045, -9.2000000000000455, -6.9000000000000341, 6.0, 1.9499999999999886, -14.900000000000034, -11.700000000000045, -7.0999999999999659, -1.4499999999999886, -2.8000000000000682, -5.4000000000000341, -9.7000000000000455, -7.1999999999999886, -14.5, -16.199999999999989, -2.2000000000000028, 0.59999999999999432, 0.85000000000002274, -6.75, -14.799999999999955, -16.600000000000023, -1.5500000000000114, -20.899999999999977, 4.0, -17.199999999999989, -6.5999999999999659, -3.5499999999999972, -17.150000000000034, -3.0999999999999943, -4.2999999999999545, 0.30000000000001137, -12.199999999999932};
double ERROR_Z[43] = {0.60000000000002274, 3.5499999999999829, 18.200000000000045, -5.5, -5.6999999999999993, 5.7000000000000455, 1.6499999999999773, 3.5499999999999829, 24.5, 27.300000000000011, 6.0, -3.8999999999999915, -9.3000000000000682, -5.0999999999999659, 3.4000000000000341, -3.8999999999999915, 2.1999999999999886, -4.5999999999999943, -4.0, -3.5999999999999943, -6.1999999999999993, -4.8000000000000114, -5.0, -4.0, -5.2999999999999972, 1.3499999999999943, 3.2500000000000142, -9.4000000000000057, 0.69999999999998863, -5.0499999999999829, -0.099999999999965894, 3.1999999999999602, -0.90000000000003411, 2.6999999999999318, 4.2999999999999545, 4.3000000000000114, -7.0, 3.6500000000000057, 3.0999999999999943, 6.4000000000000057, -6.5500000000000114, -7.2000000000000028, -5.3999999999999915};


bool getNearestPoint(geometry_msgs::PointStamped &point, int K=1);

/**
 * @brief getMode returns mode of an array of integers.
 * @param daArray array to be processed
 * @param iSize size of array
 * @return mode of the input array
 */
int getMode(int daArray[], int iSize)
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


/**
 * @brief kdtreeInit Initializes the kdtree object with centroids of clusters and the tags.
 */
void kdtreeInit()
{
    alglib::real_2d_array centers = "[[2642.5285714285715, 357.9714285714284, 301.84285714285716], [2686.825, -555.0, 265.4125], [2602.12, -62.4, -471.85999999999996], [2831.98, 398.6, 17.74000000000001], [2811.842857142857, -531.3000000000001, 20.25714285714284], [2652.1285714285714, -250.0, 289.57142857142856], [2492.15, -552.05, 382.625], [2515.383333333333, 432.5, 379.46666666666664], [2556.7400000000002, -465.5, -454.7799999999999], [2574.94, 340.24, -456.9800000000001], [2661.1375000000003, 122.87499999999997, 288.4375], [2806.057142857143, -616.8142857142857, 86.84285714285714], [2447.357142857143, 364.0571428571428, 419.1714285714286], [2493.85, -476.30000000000007, 382.55], [2709.8, 428.98, 262.1], [2832.45, 478.775, 83.125], [2650.4285714285716, -322.4428571428572, 289.7857142857143], [2811.633333333333, -617.1666666666666, -45.366666666666646], [2558.5, -476.67999999999995, 343.46000000000004], [2831.616666666667, 396.29999999999995, 83.51666666666667], [2834.7000000000003, 481.96000000000004, 17.279999999999973], [2449.4857142857145, 437.42857142857144, 418.9857142857143], [2813.6857142857143, -532.0142857142856, -45.72857142857143], [2577.6800000000003, 358.64, 341.02], [2810.0142857142855, -615.4571428571428, 20.32857142857145], [2688.4666666666667, -482.45000000000005, 265.05], [2727.9, 196.85000000000005, 248.0], [2430.1000000000004, -480.20000000000005, 420.8], [2580.1, 431.525, 340.4], [2512.0333333333333, 362.5333333333333, 380.1166666666667], [2623.9666666666667, -481.9666666666667, 304.3666666666667], [2716.0, -328.4333333333333, 250.1], [2707.15, 356.54999999999995, 262.625], [2621.8, -554.2666666666667, 304.5], [2646.1, 427.4666666666667, 300.8666666666667], [2717.0, -249.43333333333337, 250.43333333333334], [2832.9333333333334, 401.20000000000005, -48.26666666666668], [2663.4, 196.9, 288.04999999999995], [2557.3, -554.25, 343.55], [2726.95, 118.75, 249.05], [2427.25, -550.0, 421.9], [2833.8999999999996, 489.29999999999995, -48.19999999999999], [2811.1000000000004, -535.4, 85.7]]";
    alglib::ae_int_t nx = 3;
    alglib::ae_int_t ny = 0;
    alglib::ae_int_t normtype = 2;
    alglib::integer_1d_array tag;
    tag= "[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42]";
    alglib::kdtreebuildtagged(centers,tag, nx, ny, normtype,KD_TREE);
    return;

}

/**
 * @brief errorCorrection Aplies error correction based on clustering of lights. Cluster information is stored in kdtree. This should be called after detecting the light coordinates.
 * @param data is the input in the form of srcsim::Console object reference.
 */
void errorCorrection(srcsim::Console &data)
{
    int winnerTag;
    // error is stored in mm and the measured values are in m.
    double x_p = data.x*1000;
    double y_p = data.y*1000;
    double z_p = data.z*1000;

    alglib::ae_int_t         k;
    alglib::integer_1d_array tag_r;
    std::ostringstream       temp;
    temp<<"[" << x_p << "," << y_p <<"," << z_p <<"]";

    if(DEBUG)
        ROS_INFO(temp.str().c_str());

    // provide the input point to kdtree library
    alglib::real_1d_array point(temp.str().c_str());

    //perform knn search
    k = alglib::kdtreequeryknn(KD_TREE, point, 1);

    //get back the tag associated with the search result
    try{
        alglib::kdtreequeryresultstags(KD_TREE,tag_r );
    }
    catch (alglib::ap_error &er) {
        ROS_ERROR("%s",er.msg.c_str());
        return;
    }

    winnerTag = int(tag_r[0]);
    if(winnerTag < 0 || winnerTag > 42)
        return;
    // add error to measured values and convert back to m.
    data.x = (data.x + ERROR_X[winnerTag]/1000.0);
    data.y = (data.y + ERROR_Y[winnerTag]/1000.0);
    data.z = (data.z + ERROR_Z[winnerTag]/1000.0);
    return;
}


/**
 * @brief getLight detects light in color image and returns the pixel coordinates of the center of light.
 * @param index index of vector in which image is to be searched is present
 * @param pixelCoordinates  point object to store and return results
 * @param tempImage if true image will be retrieved from TEMP_COLOR_IMAGE_VECT otherwise COLOR_IMAGE_VECT
 * @return returns true is light is detected otherwise false
 */
bool getLight(int index,geometry_msgs::Point &pixelCoordinates, bool tempImage=false){
    cv::Mat newImage;
    if (tempImage)
        newImage = TEMP_COLOR_IMAGE_VECT[index];
    else
        newImage = COLOR_IMAGE_VECT[index];

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
        approxPolyDP(cv::Mat(gradientContours[i]), contours_poly[i], 3, true );
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

    return lightXYDetected;
}

/**
 * @brief getPoseRGB detects world coordinates and rgb value of a point from its pixel coordinates using diparity image.
 * @param index index of disparity image to be processed from DISPARITY_IMAGE_VECT
 * @param pixelCoordinates  pixel coordinates of the point that needs to be transformed to world coordinates
 * @return returns true if successful otherwise false
 */
bool getPoseRGB(int index, geometry_msgs::Point &pixelCoordinates)
{
    bool poseXYZDetected = false;
    tf::TransformListener listener;
    src_perception::StereoPointCloudColor::Ptr organized_cloud(new src_perception::StereoPointCloudColor);
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion orientation;
    pcl::PointXYZRGB pcl_point;
    srcsim::Console msg;

    // Wait for listener to initialize
    try
    {
        listener.waitForTransform(VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF, ros::Time(0), ros::Duration(3.0));
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::spinOnce();
        return false;
    }

    // Obtaining a stereo point cloud for world coordinates and RGB values
    src_perception::PointCloudHelper::generateOrganizedRGBDCloud(DISPARITY_IMAGE_VECT[index], COLOR_IMAGE_VECT[index], q_MATRIX, organized_cloud);
    pcl_point = organized_cloud->at(pixelCoordinates.x, pixelCoordinates.y);

    // store the in PointStamped to tranfor it from optical frame to head frame.
    geometry_msgs::PointStamped light_centroid;
    light_centroid.header.frame_id= VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF;
    light_centroid.header.stamp = ros::Time::now();
    light_centroid.point.x = pcl_point.x;
    light_centroid.point.y = pcl_point.y;
    light_centroid.point.z = pcl_point.z;
    geometry_msgs::PointStamped light_centroid_head;

    // transform the detected point to head frame
    try{

        listener.transformPoint(VAL_COMMON_NAMES::ROBOT_HEAD_FRAME_TF,light_centroid, light_centroid_head);
    }
    catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::spinOnce();
        return false;
    }

    // broadcast LED_frame to display it in rviz
    transform.setOrigin( tf::Vector3(light_centroid_head.point.x, light_centroid_head.point.y, light_centroid_head.point.z) );
    orientation.setRPY(0, 0, 0);
    transform.setRotation(orientation);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), VAL_COMMON_NAMES::ROBOT_HEAD_FRAME_TF, "LED_frame"));

    msg.x = light_centroid_head.point.x;
    msg.y = light_centroid_head.point.y;
    msg.z = light_centroid_head.point.z;

    // Assign RGB values to ROS message to be published. Getting it from the cloud and not the image, values returned are 1 or 0.
    // If the value is more than 0.7 consider it as 1
    msg.r = (int)((pcl_point.r/255.0)+0.3);
    msg.g = (int)((pcl_point.g/255.0)+0.3);
    msg.b = (int)((pcl_point.b/255.0)+0.3);

    // apply error correction based on clustering
    errorCorrection(msg);

    // If there is no LED turned on, then just don't detect anything
    if (msg.r == 0.0 && msg.g == 0 && msg.b == 0){
        poseXYZDetected = false;
        return poseXYZDetected;
    }

    poseXYZDetected = true;
    if (DEBUG)
        ROS_INFO("Updating Message at index :%d",index );

    // update the OUTPUT_MESSAGES_VECT for the specific index. This vector should always have an object at that index.
    OUTPUT_MESSAGES_VECT[index]=msg;

    return poseXYZDetected;
}

//  time sync callback for image and disparity
void imagedispCB(sensor_msgs::ImageConstPtr imageMsg, stereo_msgs::DisparityImageConstPtr disparityMsg  )
{
    // if light is not detected, store images in temp vector and do not process anything else.
    if (!LIGHT_DETECTED){
        // ROS_INFO("Inside time sync CB_light detect ");
        cv_bridge::CvImagePtr   cvPtr;
        cv::Mat inputImage;
        cvPtr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
        inputImage=cvPtr->image.clone();
        TEMP_COLOR_IMAGE_VECT.push_back(inputImage);
        return;
    }

    // READ_DATA is set when all the images are read. this means that image processing is in progress and
    // we should not update the vectors till the processing is complete.
    if (READ_DATA){
        return;
    }


    // for handling imagemsg
    cv_bridge::CvImagePtr   cvPtr;
    cv::Mat inputImage;
    cvPtr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);

    // for handling dispmsg
    std_msgs::Header		disparityHeader;
    cv::Mat outputDisparity;
    bool new_disp = false;
    uint8_t depth=sensor_msgs::image_encodings::bitDepth(disparityMsg->image.encoding);  //the size of the disparity data can be 16 or 32

    if (depth == 32)
    {
        cv::Mat_<float> tempDisparity(disparityMsg->image.height, disparityMsg->image.width,
                                      const_cast<float*>(reinterpret_cast<const float*>(&disparityMsg->image.data[0])));

        outputDisparity=tempDisparity.clone();
        new_disp = true;
        disparityHeader=disparityMsg->image.header;
    }
    else if(depth == 16)
    {
        cv::Mat_<uint16_t> Originaldisparity(disparityMsg->image.height, disparityMsg->image.width,
                                             const_cast<uint16_t*>(reinterpret_cast<const uint16_t*>(&disparityMsg->image.data[0])));
        cv::Mat_<float>   tempDisparity(disparityMsg->image.height, disparityMsg->image.width);
        tempDisparity = Originaldisparity / 16.0f;
        outputDisparity=tempDisparity.clone();
        new_disp = true;
        disparityHeader=disparityMsg->image.header;
    }
    if (!new_disp)
        return;

    if(COLOR_IMAGE_VECT.size() != MAX_IMAGES)
    {
        inputImage=cvPtr->image.clone();

        COLOR_IMAGE_VECT.push_back(inputImage);
        DISPARITY_IMAGE_VECT.push_back(outputDisparity);

        if(DISPARITY_IMAGE_VECT.size() == MAX_IMAGES){
            ROS_INFO("Read %d images", MAX_IMAGES);
            READ_DATA = true;
        }
    }
}

/**
 * @brief runner a function created for running multiple threads.
 * @param i index of the vectors that should be used for processing.
 */
void runner(int i){
    if (DEBUG)
        ROS_INFO("starting thread %d", i);

    geometry_msgs::Point pnt;
    if(getLight(i,pnt))
        getPoseRGB(i,pnt);
}

// When comparing messages from stereo cam point cloud, it does not make sense in comparing x,
// as we have highest error in x or z as the error in z is almost constant. Hence using y
bool isLessThan(const srcsim::Console &lhs,const srcsim::Console &rhs){
    return lhs.y< rhs.y;
}

bool getNearestPoint(srcsim::Console &msg, int K=1){
    geometry_msgs::PointStamped pnt;
    pnt.point.x = msg.x;
    pnt.point.y = msg.y;
    pnt.point.z = msg.z;
    pnt.header.frame_id=VAL_COMMON_NAMES::ROBOT_HEAD_FRAME_TF;

    bool success = laser_assembler::PeriodicSnapshotter::getNearestPoint(pnt, K);
    if(success){
        msg.x = pnt.point.x;
        msg.y = pnt.point.y;
        msg.z = pnt.point.z;
    }

    return success;
}

// read laser data and store it in pcl form for direct usage with PCL
void laserCloudCallBack(const sensor_msgs::PointCloud2 &msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*POINTCLOUD_PTR);
}



int main(int argc, char **argv)
{
    //Initialize ROS system
    ros::init (argc,argv, "image_disp_data");
    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    POINTCLOUD_PTR = temp_cloud;

    //setup subscriber and publisher
    // using Time Synchronizer
    message_filters::Subscriber<sensor_msgs::Image> colorImageSub(nh, "/multisense/camera/left/image_rect_color", 1);
    message_filters::Subscriber<stereo_msgs::DisparityImage> disparityImageSub(nh, "/multisense/camera/disparity", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, stereo_msgs::DisparityImage> sync(colorImageSub, disparityImageSub, 10);
    sync.registerCallback(boost::bind(&imagedispCB, _1, _2));

    ros::Subscriber m_laserCloudSub = nh.subscribe("/assembled_cloud2",1, laserCloudCallBack);
    ros::Publisher temp = nh.advertise<srcsim::Console>("/srcsim/qual1/light",1);
    //the pointer is public so had to use a temp variable
    LIGHT_PUB = &temp;
    unsigned int numberOfLightsDetected=0;

    ros::Rate loopRate = 100;
    ros::Time begin = ros::Time::now();
    ros::Time end;
    float time_taken ;

    //populate the qMatrix. it wont change with every image so no point in calculatin it everytime
    //    src_perception::MultisenseImage *m_multisenseImagePtr = new src_perception::MultisenseImage(nh);
    //    m_multisenseImagePtr->giveQMatrix(qMatrix);
    q_MATRIX=cv::Mat_<double>(4,4,0.0);
    q_MATRIX(0,0) =  610.1799470098168 * -0.07;
    q_MATRIX(1,1) =  610.1799470098168 * -0.07;
    q_MATRIX(0,3) = -610.1799470098168 * 512.5 * -0.07;
    q_MATRIX(1,3) = -610.1799470098168 * 272.5 * -0.07;
    q_MATRIX(2,3) =  610.1799470098168 * 610.1799470098168 * -0.07;
    q_MATRIX(3,2) = -610.1799470098168;
    q_MATRIX(3,3) =  0.0f;

    //initialize kdtree for error correction
    kdtreeInit();

    while (ros::ok())
    {
        //wait for the light to be switched on
        //assuming light detection will work soon
        if (!LIGHT_DETECTED){
            geometry_msgs::Point pnt;
            size_t index = TEMP_COLOR_IMAGE_VECT.size()-1;
            if (index == -1){
                ros::spinOnce();
                continue;
            }
            LIGHT_DETECTED = getLight(index,pnt, true);
            std::string output = LIGHT_DETECTED ? "Light Detected": "******Not detected";
//            if (DEBUG)
//                ROS_INFO(output.c_str());
            end  = ros::Time::now();
            time_taken = end.toSec() - begin.toSec();
            begin = ros::Time::now();
            if(LIGHT_DETECTED) {
                TEMP_COLOR_IMAGE_VECT.clear();
            }
            else {
                ros::spinOnce();
                continue;
            }
        }


        //read_data is true when the vectors are full
        if(READ_DATA)
        {
            end  = ros::Time::now();
            time_taken = end.toSec() - begin.toSec();
            OUTPUT_MESSAGES_VECT = std::vector<srcsim::Console>(MAX_IMAGES);

            //create threads for every value in vector
            for (size_t i=0; i< COLOR_IMAGE_VECT.size(); i++){
                THREADS_VECT.push_back(std::thread(runner, i));
            }
            ROS_INFO("Running %d threads", THREADS_VECT.size());
            //wait till all threads finish
            for(size_t i=0; i<THREADS_VECT.size(); i++)
                THREADS_VECT[i].join();

            THREADS_VECT.clear();

            bool skipIteration = false;
            std::sort(OUTPUT_MESSAGES_VECT.begin(), OUTPUT_MESSAGES_VECT.end(), isLessThan);

            for (size_t i=0; i< OUTPUT_MESSAGES_VECT.size(); i++){
                if (OUTPUT_MESSAGES_VECT[i].x == 0.0 && OUTPUT_MESSAGES_VECT[i].y == 0.0 && OUTPUT_MESSAGES_VECT[i].z == 0.0){
                    skipIteration = true;
                    ROS_INFO("Skipping this iteration");
                    break;
                }
                if(DEBUG)
                    ROS_INFO("output message %d x:%.4f y:%.4f z:%.4f", i, OUTPUT_MESSAGES_VECT[i].x, OUTPUT_MESSAGES_VECT[i].y, OUTPUT_MESSAGES_VECT[i].z);
            }

            srcsim::Console msg;
            int index = OUTPUT_MESSAGES_VECT.size()/2;
            msg.x = OUTPUT_MESSAGES_VECT[index].x;
            msg.y = OUTPUT_MESSAGES_VECT[index].y;
            msg.z = OUTPUT_MESSAGES_VECT[index].z;

            msg.r = OUTPUT_MESSAGES_VECT[index].r;
            msg.g = OUTPUT_MESSAGES_VECT[index].g;
            msg.b = OUTPUT_MESSAGES_VECT[index].b;

            // publish the message only if it is not the same light
            if (!skipIteration){
                if(getNearestPoint(msg, 10)){
                    ROS_INFO("Updated the point with lidar data");
                }
                else
                    ROS_INFO("Cannot update from pointcloud");
                ROS_INFO("Publishing message x:%.4f y:%.4f z:%.4f", msg.x, msg.y, msg.z);
                LIGHT_PUB->publish(msg);
                numberOfLightsDetected++;
                ros::Duration(0.2).sleep();
            }

            begin = ros::Time::now();
            LIGHT_DETECTED=false;

            // clearing vectors to be used for next light
            COLOR_IMAGE_VECT.clear();
            DISPARITY_IMAGE_VECT.clear();
            READ_DATA = false;

        }

        // to make sure callback is called
        ros::spinOnce();
        loopRate.sleep();
    }
    ros::Duration(7).sleep();
    return 0;

}
