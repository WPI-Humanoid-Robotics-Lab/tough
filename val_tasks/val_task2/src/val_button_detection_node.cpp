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
#include <val_common/val_common_names.h>

bool processImage(geometry_msgs::Point &buttonCenter, ros::NodeHandle &nh )
{
    src_perception::MultisenseImage mi(nh);
    pcl::PointXYZRGB point1;
    src_perception::StereoPointCloudColor::Ptr organized_cloud(new src_perception::StereoPointCloudColor);
    tf::TransformListener listener;
    cv::Mat img;
    cv::Mat_<float> disp;
    cv::Mat_<double> Q;
    geometry_msgs::PointStamped geom_point;
    bool foundButton = false;
    cv::Mat3b hsv;
    int largest_area=0;
    int largest_contour_index=0;
    cv::Point2f point;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat1b mask;
    cv::Mat1b mask1, mask2;
    double area;
    cv::Moments moment;

    if(mi.giveImage(img))
    {
        if(mi.giveDisparityImage(disp))
        {
            if(!mi.giveQMatrix(Q))
            {
                ros::spinOnce();
            }
            src_perception::PointCloudHelper::generateOrganizedRGBDCloud(disp,img,Q,organized_cloud);
            ROS_DEBUG_STREAM("Organized cloud size: "<<organized_cloud->size());

            static int hfl=0,sfl=178,vfl=51,hfu=5,sfu=255,vfu=149,hsl=170,ssl=204,vsl=140,hsu=180,ssu=255,vsu=191;

            cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
            // detect red color in hsv image

            cv::inRange(hsv, cv::Scalar(hfl, sfl, vfl), cv::Scalar(hfu, sfu, vfu), mask1);
            cv::inRange(hsv, cv::Scalar(hsl, ssl, vsl), cv::Scalar(hsu, vsu, ssu), mask2);

            mask = mask1 | mask2;

            // find contours
            cv::findContours( mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );


            for( int i = 0; i< contours.size(); i++ )
            {

                //  Find the area of contour
                area = cv::contourArea( contours[i],false);

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
                moment = cv::moments((cv::Mat)contours[largest_contour_index]);

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
            try
            {
                listener.waitForTransform(VAL_COMMON_NAMES::WORLD_TF, VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF, ros::Time(0), ros::Duration(3.0));

            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::spinOnce();
            }
            if( foundButton )
            {

                point1 = organized_cloud->at(buttonCenter.x,buttonCenter.y);
                geom_point.point.x = point1.x;
                geom_point.point.y = point1.y;
                geom_point.point.z = point1.z;
                geom_point.header.frame_id = VAL_COMMON_NAMES::LEFT_CAMERA_OPTICAL_FRAME_TF;
                listener.transformPoint(VAL_COMMON_NAMES::WORLD_TF, geom_point, geom_point);

            } //processed image is recieved
        } // got diaparity image
    } //got color image
    ros::spinOnce();
    ROS_INFO_STREAM(foundButton << std::endl);
    return foundButton;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"buttonFrame");
    ros::NodeHandle nh;
    geometry_msgs::Point buttoncenter;
    processImage(buttoncenter, nh);


}
