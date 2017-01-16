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
#include <stdlib.h>
#include <math.h>
#include "alglibmisc.h"

bool flag = true;
bool readData = false;
bool lightDetected = false;

ros::Publisher *m_lightPub;
cv::Mat qMatrix;
const int maxImages = 5;

std::vector<cv::Mat> colorImageVect;
std::vector<cv::Mat> tempColorImageVect;
std::vector <cv::Mat> disparityImageVect;
std::vector<std::thread> threadsVect;
std::vector<srcsim::Console> outputMessages(maxImages);


alglib::kdtree kdt;
double error_x[43] = {82.799999999999983, 83.714285714285651, 33.175000000000011, 147.13636363636368, 40.937500000000171, 34.766666666666502, 78.762500000000102, 11.149999999999977, 163.17999999999967, 46.340000000000174, 192.30000000000018, 43.872222222222291, 71.9769230769231, 39.083333333333258, 34.9866666666668, 24.062499999999915, 56.810526315789524, 22.937499999999716, 26.828571428571358, 70.439999999999912, 62.209090909091053, 53.142105263157788, 22.41666666666659, 75.404166666666626, 17.612500000000125, 64.225000000000023, 59.328571428571195, 22.542857142856942, 10.600000000000104, 89.500000000000171, 74.681818181818187, 6.1857142857143117, 22.189999999999873, 60.807692307692307, 26.319047619047609, 100.98571428571437, 31.153846153846082, 40.678571428571459, 93.16250000000025, 91.60909090909081, 174.04999999999987, 187.02500000000009, 15.39333333333334};
double error_y[43] = {12.28571428571429, 8.0428571428571694, 13.250000000000057, 4.863636363636366, 13.737500000000004, 2.0333333333333221, 1.6249999999999964, 6.7500000000000142, 29.859999999999992, 14.433333333333326, 39.883333333333347, 1.5555555555555525, 19.507692307692306, 12.650000000000025, 13.553333333333352, 5.381249999999973, 3.6105263157894658, 4.7874999999999943, 11.235714285714282, 2.0300000000000069, 15.490909090909099, 15.078947368421058, 3.8916666666666608, 14.229166666666666, 3.1187499999999986, 3.800000000000002, 4.8142857142856963, 9.3928571428571299, 3.4714285714285609, 3.9250000000000007, 22.572727272727288, 6.5714285714285792, 9.3150000000000084, 1.2615384615384728, 2.900000000000015, 13.900000000000009, 11.553846153846145, 2.7142857142857224, 0.95000000000000995, 15.872727272727275, 25.583333333333332, 31.999999999999986, 4.8000000000000078};
double error_z[43] = {4.4000000000000012, 3.999999999999992, 5.0874999999999968, 19.390909090909101, 2.6062500000000064, 4.7666666666666648, 4.1124999999999972, 4.9999999999999716, 21.840000000000011, 4.6066666666666629, 27.116666666666674, 3.9944444444444391, 2.6769230769230559, 4.6833333333333371, 2.619999999999997, 4.0874999999999808, 2.2578947368421041, 6.1875000000000018, 5.3857142857142852, 2.7200000000000015, 1.4363636363636194, 1.9789473684210479, 3.8333333333333428, 3.6000000000000014, 7.0437500000000064, 2.0083333333333351, 3.9714285714285649, 5.0, 6.7285714285714322, 6.0624999999999964, 3.0181818181818016, 5.0142857142857258, 8.4799999999999933, 2.0076923076923006, 5.5714285714285721, 4.3571428571428568, 4.546153846153846, 4.5285714285714294, 5.4000000000000092, 3.8454545454545515, 24.01666666666668, 27.024999999999991, 3.1733333333333236};
const int m_readingThreshold = 3;


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



void kdtreeInit()
{
    alglib::real_2d_array centers = "[[2649.1555555555556, -161.05555555555557, 289.97777777777776], [2684.6555555555556, 519.7666666666668, 265.5333333333333], [2810.5, -615.97, 0.6200000000000045], [2596.730769230769, -7.792307692307694, -471.1000000000001], [2559.6944444444443, -491.5944444444445, 343.2], [2810.0375, 577.4375, -45.17500000000001], [2649.35, 209.96, 289.88], [2512.0333333333333, 362.5333333333333, 380.1166666666667], [2553.6857142857143, -470.0142857142857, -454.37142857142857], [2809.9764705882353, -563.3647058823528, 86.25294117647057], [2553.225, 429.06249999999994, -454.0375000000001], [2821.395, 458.09, 84.71999999999998], [2691.5066666666667, -473.52, 264.6733333333333], [2815.8125, -437.05, 85.52499999999999], [2496.1470588235293, -490.5941176470588, 382.2470588235294], [2457.438888888889, 518.0611111111111, 402.48333333333335], [2623.704761904762, 474.6190476190476, 304.1904761904762], [2831.5, 395.31, -15.110000000000014], [2816.1375, -528.5625, -46.006249999999966], [2709.2166666666667, 310.9666666666667, 254.99166666666667], [2699.146153846154, -374.7076923076923, 260.32307692307694], [2627.6809523809525, -438.04285714285714, 303.7809523809524], [2501.4714285714285, 438.5857142857143, 381.42857142857144], [2649.857692307692, -266.71923076923076, 289.80384615384617], [2466.1222222222223, -394.35, 401.4611111111111], [2693.9285714285716, 440.3142857142857, 264.3071428571429], [2554.788888888889, 518.6666666666666, 343.84444444444443], [2818.475, -436.56874999999997, -13.456250000000011], [2430.99375, -488.74999999999994, 421.225], [2674.3, 122.05, 280.56], [2665.107692307692, -557.3461538461538, 277.65384615384613], [2563.1555555555556, -394.78888888888883, 342.87777777777774], [2440.2772727272727, 415.98636363636376, 420.16363636363644], [2643.94, 320.86000000000007, 295.74], [2817.2652173913043, 488.6652173913042, -6.15217391304347], [2713.9777777777776, -160.64444444444445, 250.79999999999998], [2815.4066666666668, -526.0333333333334, 19.81333333333336], [2806.33125, 577.83125, 53.8125], [2714.39, 210.07, 250.47], [2714.8076923076924, -242.15384615384616, 250.66923076923075], [2560.7375, -376.3374999999999, -455.2250000000001], [2573.7833333333333, 337.68333333333334, -456.8333333333332], [2567.9470588235295, 416.16470588235296, 342.06470588235294]]";
    alglib::ae_int_t nx = 3;
    alglib::ae_int_t ny = 0;
    alglib::ae_int_t normtype = 2;
    alglib::integer_1d_array tag;
    tag= "[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42]";
    kdtreebuildtagged(centers,tag, nx, ny, normtype,kdt);
    return;

}


void errorCorrection(srcsim::Console &data, std::vector<double> &pos)
{
    std::cout << "Check Point A" <<std::endl;


    alglib::ae_int_t k;
    int winnerTag;
    std::cout << "Check Point B" <<std::endl;

        alglib::integer_1d_array tag_r;
        double x_p = data.x*1000;
        double y_p = data.y*1000;
        double z_p = data.z*1000;
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
        winnerTag = int(tag_r[0]);

        pos.push_back(data.x + error_x[winnerTag]/1000.0);
        pos.push_back(data.y + error_y[winnerTag]/1000.0);
        pos.push_back(data.z + error_z[winnerTag]/1000.0);
    return;
}


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
    std::vector<double> pos;
    errorCorrection(msg,pos);

    msg.x = pos[0];
    msg.z = pos[0];
    msg.y = pos[0];
    // If there is no LED turned on, then just don't detect anything
    if (msg.r == 0.0 && msg.g == 0 && msg.b == 0){
        poseXYZDetected = false;
        return poseXYZDetected;
    }

    poseXYZDetected = true;
    ROS_INFO("Updating Message at index :%d",index );
    //    m_lightPub->publish(msg);
    //    outputMessages.insert(outputMessages.begin()+index,msg);
    outputMessages[index]=msg;
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
    geometry_msgs::Point pnt;
    getLight(i,pnt);
    getPoseRGB(i,pnt);
}

int main(int argc, char **argv)
{
    //Initialize ROS system
    ros::init (argc,argv, "image_disp_data");
    ros::NodeHandle nh;
    //setup subscriber and publisher
    ros ::Subscriber colorImageSub = nh.subscribe("/multisense/camera/left/image_rect_color",0,imageCB);
    ros ::Subscriber disparityImageSub = nh.subscribe("/multisense/camera/disparity",0,dispCB);
    ros::Publisher temp = nh.advertise<srcsim::Console>("/srcsim/qual1/light",1);
    //the pointer is public so had to use a temp variable
    m_lightPub = &temp;
    ros::Rate loopRate = 35;
    ros::Time begin = ros::Time::now();
    ros::Time end;
    float time_taken ;
    std::cout<< "Check point 1" << std::endl;
    //populate the qMatrix. it wont change with every image so no point in calculatin it everytime
    src_perception::MultisenseImage *m_multisenseImagePtr = new src_perception::MultisenseImage(nh);
    m_multisenseImagePtr->giveQMatrix(qMatrix);
    std::cout<< "Check point 2" << std::endl;
    kdtreeInit();
    std::cout<< "Check point 3" << std::endl;
    while(ros::ok())
    {
        //wait for the light to be switched on
        //assuming light detection will work soon
        if (!lightDetected && !tempColorImageVect.empty()){
            geometry_msgs::Point pnt;
            lightDetected = getLight(tempColorImageVect.size()-1,pnt, true);
            std::string output = lightDetected ? "Light Detected": "******Not detected";
            ROS_INFO(output.c_str());
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
            ROS_INFO("Time taken = %lf",time_taken );

            //create threads for every value in vector
            for (size_t i=0; i< colorImageVect.size(); i++){
                threadsVect.push_back(std::thread(runner, i));
            }
            //wait till all threads finish
            for(size_t i=0; i<threadsVect.size(); i++)
                threadsVect[i].join();

            // calculating mean of the results
            float meanX =0.0 , meanY =0.0 ,meanZ =0.0;

            for (size_t i=0; i< outputMessages.size(); i++){
                meanX += outputMessages[i].x;
                meanY += outputMessages[i].y;
                meanZ += outputMessages[i].z;
            }

            srcsim::Console msg;
            msg.x = meanX/outputMessages.size();
            msg.y = meanY/outputMessages.size();
            msg.z = meanZ/outputMessages.size();
            msg.r = outputMessages[0].r;
            msg.g = outputMessages[0].g;
            msg.b = outputMessages[0].b;

            // publish the message
            m_lightPub->publish(msg);

            begin = ros::Time::now();
            lightDetected=false;
            // clearing vectors to be used for next light
            colorImageVect.clear();
            disparityImageVect.clear();
            threadsVect.clear();
            readData = false;

        }
        // to make sure callback is called

        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;

}
