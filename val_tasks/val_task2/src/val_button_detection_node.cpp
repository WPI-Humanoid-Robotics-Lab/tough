/***
    Nathaniel Goldfarb
    This node detects the frame the position of a button and create a button frame
***/
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <perception_common/MultisenseImage.h>
#include <perception_common/PointCloudHelper.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

bool processImage(const cv::Mat&, geometry_msgs::Point&);

ros::Publisher buttonDectected;


void getButtonFrame(ros::NodeHandle &nh)
{

    std::string frame = "buttonFrame";
    src_perception::MultisenseImage mi(nh);
    pcl::PointXYZRGB point;
    geometry_msgs::Point buttonCenter;
    std_msgs::Bool found;
    src_perception::StereoPointCloudColor::Ptr organized_cloud(new src_perception::StereoPointCloudColor);
    static tf::TransformBroadcaster br;
    tf::TransformListener listener;
    tf::Transform transform;
    tf::Quaternion q;
    cv::Mat img;
    cv::Mat_<float> disp;
    cv::Mat_<double> Q;


    ros::Rate rate(1000);
    while(ros::ok())
    {
        found.data = false;

        // wait till tf is loaded
        try{
            listener.waitForTransform("/world", "/left_camera_optical_frame", ros::Time(0), ros::Duration(3.0));
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::spinOnce();
            buttonDectected.publish(found);
            continue;
        }
        cv::waitKey(3);
        //get synced images from multisense. check the datatypes of arguments in function definition.
        if(mi.giveImage(img))
        {
            if(mi.giveDisparityImage(disp))
            {
                if(!mi.giveQMatrix(Q))
                {
                    ros::spinOnce();
                    continue;
                }

                src_perception::PointCloudHelper::generateOrganizedRGBDCloud(disp,img,Q,organized_cloud);
                ROS_DEBUG_STREAM("Organized cloud size: "<<organized_cloud->size());
                if( processImage(img,buttonCenter) )
                {
                    point = organized_cloud->at(buttonCenter.x,buttonCenter.y);
                    // ignore the button if it is more than 5m away on any axis
//                    if ( abs(point.x) < 5 && abs(point.y) < 5  && abs(point.z) < 5 )
//                    {
//                        ROS_DEBUG("found button at :%d, %d, %d", int(point.x) , int(point.y), int(point.z));
                    ROS_INFO_STREAM("outside func"<<point.x<<"\t"<<point.y<<"\t"<<point.z<<std::endl);
                        found.data = true;
                        transform.setOrigin( tf::Vector3(point.x , point.y, point.z) );
                        q.setRPY(0, 0, 0);
                        transform.setRotation(q);
                        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "left_camera_optical_frame", frame));

//                    }
                    rate.sleep();
                    ROS_INFO_STREAM(found<<std::endl);

                    buttonDectected.publish(found);
                } //processed image is recieved
            } // got diaparity image
        } //got color image
        ros::spinOnce();
    }

}

bool processImage(const cv::Mat& src, geometry_msgs::Point &buttonCenter )
{

    cv::Mat3b hsv;
    bool foundButton = false;
    cv::namedWindow("trackbar", 1);
    static int hfl=0,sfl=178,vfl=51,hfu=5,sfu=255,vfu=149,hsl=170,ssl=204,vsl=140,hsu=180,ssu=255,vsu=191;
    cv::createTrackbar("hfl","trackbar",&hfl,180);
    cv::createTrackbar("sfl","trackbar",&sfl,255);
    cv::createTrackbar("vfl","trackbar",&vfl,255);
    cv::createTrackbar("hfu","trackbar",&hfu,180);
    cv::createTrackbar("sfu","trackbar",&sfu,255);
    cv::createTrackbar("vfu","trackbar",&vfu,255);
    cv::createTrackbar("hsl","trackbar",&hsl,180);
    cv::createTrackbar("ssl","trackbar",&ssl,255);
    cv::createTrackbar("vsl","trackbar",&vsl,255);
    cv::createTrackbar("hsu","trackbar",&hsu,180);
    cv::createTrackbar("ssu","trackbar",&ssu,255);
    cv::createTrackbar("vsu","trackbar",&vsu,255);
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
    // detect red color in hsv image
    cv::Mat1b mask1, mask2;
    cv::inRange(hsv, cv::Scalar(hfl, sfl, vfl), cv::Scalar(hfu, sfu, vfu), mask1);
    cv::inRange(hsv, cv::Scalar(hsl, ssl, vsl), cv::Scalar(hsu, vsu, ssu), mask2);
    cv::getTrackbarPos("hfl","trackbar");
    cv::getTrackbarPos("sfl","trackbar");
    cv::getTrackbarPos("vfl","trackbar");
    cv::getTrackbarPos("hfu","trackbar");
    cv::getTrackbarPos("sfu","trackbar");
    cv::getTrackbarPos("vfu","trackbar");
    cv::getTrackbarPos("hsl","trackbar");
    cv::getTrackbarPos("ssl","trackbar");
    cv::getTrackbarPos("vsl","trackbar");
    cv::getTrackbarPos("hsu","trackbar");
    cv::getTrackbarPos("ssu","trackbar");
    cv::getTrackbarPos("vsu","trackbar");

    cv::Mat1b mask = mask1 | mask2;

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::imshow("window",mask);
    // find contours
    cv::findContours( mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    int largest_area=0;
    int largest_contour_index=0;
    cv::Point2f point;

    for( int i = 0; i< contours.size(); i++ )
    {

        //  Find the area of contour
        double area = cv::contourArea( contours[i],false);

        if(area>largest_area)
        {
            largest_area=area;
            // Store the index of largest contour
            largest_contour_index=i;
        }
    }
    //ROS_INFO_STREAM(contours[largest_contour_index]<<std::endl);
    if(!contours.empty()) //avoid seg fault at runtime by checking that the contours exist
    {
        cv::Moments moment = cv::moments((cv::Mat)contours[largest_contour_index]);

        if (moment.m00)
        {
            point = cv::Point2f(moment.m10/moment.m00,moment.m01/moment.m00);

            ROS_DEBUG("m00:%.2f, m10:%.2f, m01:%.2f",moment.m00, moment.m10, moment.m01);
            ROS_DEBUG("x:%d, y:%d", int(point.x), int(point.y));

            //assian the button center
            buttonCenter.x = int (point.x);
            buttonCenter.y = int (point.y);
            foundButton = true;
        }
    }
    ROS_INFO_STREAM("inside func"<<buttonCenter.x<<"\t"<<buttonCenter.y<<std::endl);
    return foundButton;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"buttonFrame");
    ros::NodeHandle nh;

    buttonDectected = nh.advertise<std_msgs::Bool>("button_found", 10);
    getButtonFrame(nh);


}
