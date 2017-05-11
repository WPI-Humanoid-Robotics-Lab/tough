#include "val_task_common/finish_box_detector.h"
#include <val_common/val_common_names.h>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <map>
#include <iostream>
#include <cassert>

using namespace cv;

FinishBoxDetector::FinishBoxDetector(ros::NodeHandle &n):nh_(n) {

    pointcloudSub_ = nh_.subscribe("/map", 10, &FinishBoxDetector::detectFinishBox, this);
}

FinishBoxDetector::~FinishBoxDetector()
{
    pointcloudSub_.shutdown();
}

bool FinishBoxDetector::getFinishBoxCenters(std::vector<geometry_msgs::Point> &centers)
{
    centers.clear();

    if(finish_box_centers_.empty()){
        return false;
    }
    std::set<Point2D> temp_centers = finish_box_centers_;
    for(auto it = temp_centers.begin(); it != temp_centers.end(); ++it) {
        geometry_msgs::Point pt;
        pt.x = it->x*MAP_RESOLUTION+MAP_X_OFFSET;
        pt.y = it->y*MAP_RESOLUTION+MAP_Y_OFFSET;
        centers.push_back(pt);
    }

    return true;

}

void FinishBoxDetector::detectFinishBox(const nav_msgs::OccupancyGrid::Ptr msg) {
    static bool isProcessing = false;
    if(!isProcessing){
        isProcessing = true;
        ros::WallTime t = ros::WallTime::now();
        MAP_X_OFFSET = msg->info.origin.position.x;
        MAP_Y_OFFSET = msg->info.origin.position.y;
        MAP_RESOLUTION = msg->info.resolution;
        MAP_WIDTH   = msg->info.width;
        MAP_HEIGHT  = msg->info.height;
        vector<vector<Point> > contours;
        RNG rng(12345);

        uchar pv[msg->data.size()];
        for(unsigned int i = 0; i < msg->data.size(); i++) {
            pv[i] = (uchar) msg->data.at(i);
        }

        map_image_ = cv::Mat(cv::Size(MAP_WIDTH, MAP_HEIGHT), CV_8UC1);
        memcpy(map_image_.data, &pv, msg->data.size());

        cv::threshold(map_image_, map_image_,50, 255, CV_THRESH_BINARY);
        findContours( map_image_, contours, RETR_CCOMP, CHAIN_APPROX_SIMPLE ); // Find the contours in the image

        vector<vector<Point> > contours_poly( contours.size() );
        vector<Rect> boundRect( contours.size() );


        for( int i = 0; i < contours.size(); i++ ) {
            approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
            boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        }
        std::vector<int> xMin,xMax,yMin,yMax;
        /// Draw polygonal contour + bonding rects + circles
        for( int i = 0; i< contours.size(); i++ )
        {
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            //drawContours( map_image_, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
            //        std::cout<<"Area of contour :"<<contourArea( contours[i])<<std::endl;
            if(contourArea( contours[i] )>3000) // because area of just the finish box will be 3600 pixels
            {
                rectangle( map_image_, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
                if(boundRect[i].tl().x>1 && boundRect[i].br().x<999) // To avoid largest bounding box which encompasses the entire image
                {
                    xMin.push_back(boundRect[i].tl().x); xMax.push_back(boundRect[i].br().x);
                    yMin.push_back(boundRect[i].tl().y); yMax.push_back(boundRect[i].br().y);
                    //                std::cout<<"pt1:    "<<boundRect[i].tl()<<"    pt2:    "<<boundRect[i].br()<<std::endl;
                    //                break;
                }

            }

        }
        //    showImage(map_image_);

        float radius=1.3/MAP_RESOLUTION; // side of square is 3 meters

        cv::Mat circleMask,reducedImage,newMap;
        for(int k = 0; k < xMin.size(); ++k){
            for (int i = xMin[k]; i < xMax[k]; i+=2) {
                for (int j = yMin[k]; j < yMax[k]; j+=2) {
                    circleMask = cv::Mat::zeros(map_image_.size(), CV_8UC1);
                    cv::circle(circleMask, cv::Point(i, j),radius,cv::Scalar(255),-1);
                    reducedImage=cv::Mat::zeros(map_image_.size(), CV_8UC1);
                    map_image_.copyTo(reducedImage, circleMask);
                    newMap=cv::Mat::ones(map_image_.size(), map_image_.type());
                    findNonZero(reducedImage,newMap);
                    if(newMap.total()<10) // 10 used to keep some threshold
                    {
                        //                std::cout<<"BOX found"<<std::endl;
                        //                std::cout<<"Image CenterX:    "<<i<<"    Image CenterY:    "<<j<<std::endl;
                        std::cout<<"World CenterX:    "<<i*MAP_RESOLUTION+MAP_X_OFFSET<<"    World CenterY:    "<<j*MAP_RESOLUTION+MAP_Y_OFFSET<<std::endl;
                        Point2D pt;
                        pt.x = i;
                        pt.y = j;

                        finish_box_centers_.insert(pt);
                        circle( map_image_, Point(i, j), 5,  Scalar(255), 2, 8, 0 );
                        //                showImage(map_image_);
                        //                    exit(0);
                    }
                }
            }
        }
        std::cout<<"size of detected points "<<finish_box_centers_.size()<<std::endl;
        std::cout<<"Took "<<(ros::WallTime::now() - t).toSec() <<"seconds"<< std::endl;
        isProcessing = false;
    }
    return;
}
void FinishBoxDetector::showImage(cv::Mat image, std::string caption)
{

    cv::namedWindow( caption, cv::WINDOW_AUTOSIZE );
    cv::imshow( caption, image);
    cv::waitKey(0);
}
