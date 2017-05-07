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
    //    mapPub_        = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 10, true);
}

FinishBoxDetector::~FinishBoxDetector()
{
    
}

int FinishBoxDetector::calcDistance(int fromX, int fromY, int toX, int toY)
{
    int x=pow(fromX-toX,2)+pow(fromY-toY,2);
    //    std::cout<<"dist:   "<<x<<std::endl;
    return x;
}

int FinishBoxDetector::calcDistance(const Point2D &pt1, const Point2D &pt2)
{
    return calcDistance(pt1.x, pt1.y, pt2.x, pt2.y);
}


void FinishBoxDetector::detectFinishBox(const nav_msgs::OccupancyGrid::Ptr msg) {
    MAP_X_OFFSET = msg->info.origin.position.x;
    MAP_Y_OFFSET = msg->info.origin.position.y;
    MAP_RESOLUTION = msg->info.resolution;
    MAP_WIDTH   = msg->info.width;
    MAP_HEIGHT  = msg->info.height;
    
    uchar pv[msg->data.size()];
    for(unsigned int i = 0; i < msg->data.size(); i++) {
        pv[i] = (uchar) msg->data.at(i);
    }
    
    map_image_ = cv::Mat(cv::Size(MAP_WIDTH, MAP_HEIGHT), CV_8UC1);
    memcpy(map_image_.data, &pv, msg->data.size());
    
    Mat dst, dst_norm, dst_norm_scaled;
    dst = Mat::zeros( map_image_.size(), CV_32FC1 );
    
    /// Detector parameters
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;
    int thresh = 90;
    
    /// Detecting corners
    cornerHarris( map_image_, dst, blockSize, apertureSize, k, BORDER_DEFAULT );
    std::set<Point2D> corners;
    /// Normalizing
    normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    convertScaleAbs( dst_norm, dst_norm_scaled );
    /// Drawing a circle around corners
    for( int j = 0; j < dst_norm.rows ; j++ )
    { for( int i = 0; i < dst_norm.cols; i++ )
        {
            if( (int) dst_norm.at<float>(j,i) > thresh )
            {
                Point2D pt;
                pt.x = i;
                pt.y = j;
                //   circle( map_image_, Point(pt.x, pt.y ), 5,  Scalar(255), 2, 8, 0 );
                corners.insert(pt);
            }
        }
    }
    //    std::cout<<"*******************"<<std::endl;
    //    for (auto i : corners){
    //        std::cout<<i.x<<" "<<i.y<<std::endl;
    //    }
    //    std::cout<<"*******************"<<std::endl;
    //    //  std::set<Point2D> filteredCorners;
    //    // second stage filter


    auto it = corners.begin();
    while(it != corners.end()){
        auto next = std::next(it, 1);
        if (next != corners.end() && (*it == *(next))){
            corners.erase(next);
        }
        else{
            ++it;
        }
    }

    for( auto i : corners)
    {
        std::cout<<"x:  "<<i.x<<"   "<<"y:"<<i.y<<std::endl;
        circle( map_image_, Point(i.x, i.y ), 5,  Scalar(150), 2, 8, 0 );

    }
//    circle( map_image_, Point(168,467), 5,  Scalar(255), 2, 8, 0 );
//    circle( map_image_, Point(213,510), 5,  Scalar(255), 2, 8, 0 );
//    circle( map_image_, Point(212,422), 5,  Scalar(255), 2, 8, 0 );
//    circle( map_image_, Point(256,466), 5,  Scalar(255), 2, 8, 0 );



    //    int dist2=pow(3/0.05,2);
    //    int diag2=pow(3*1.41/0.05,2);
    int dist2=3850;
    int diag2=7150;
    int countdist=0;
    int countdiag=0;
    std::vector<Point2D> cornerPoints;

    for (int i = 0; i < corners.size(); ++i) {
        for (int j = 0; j < corners.size(); ++j) {
            if (i==j){
                continue;
            }
            if(fabs(calcDistance(*std::next(corners.begin(), i),*std::next(corners.begin(), j)) - dist2) < 300)
            {
                ROS_INFO("dist criteria matched");
                countdist++;
            }
            if(fabs(calcDistance(*std::next(corners.begin(), i), *std::next(corners.begin(), j)) - diag2) < 300)
            {
                ROS_INFO("diagonal criteria matched");
                countdiag++;
            }
        }
        if(countdist>2 && countdiag>1)
        {
            cornerPoints.push_back(*std::next(corners.begin(), i));
            ROS_INFO("found a point");
            countdist=0;
            countdiag=0;
        }
        else
        {
            ROS_INFO("Condition NOT Met");
            countdist=0;
            countdiag=0;
        }
    }
    Mat plotimage = Mat::zeros( map_image_.size(), CV_8UC1 );
    std::cout<<cornerPoints.size()<<std::endl;
    for (auto i:cornerPoints) {
        std::cout<<i.x<<"   "<<i.y<<std::endl;
        circle( plotimage, Point(i.x,i.y), 5,  Scalar(255), 2, 8, 0 );
    }

    //     Showing the result
    namedWindow( "plotimage", CV_WINDOW_AUTOSIZE );
    imshow( "plotimage", plotimage);
    namedWindow( "real image", CV_WINDOW_AUTOSIZE );
    imshow( "real image",map_image_ );
    waitKey(0);



}


size_t FinishBoxDetector::getIndex(float x, float y){

    trimTo2DecimalPlaces(x, y);

    x -= MAP_X_OFFSET;
    y -= MAP_Y_OFFSET;

    int index_x = x/MAP_RESOLUTION;
    int index_y = y/MAP_RESOLUTION;

    size_t index = index_y*MAP_WIDTH + index_x;

    return index;
}

void FinishBoxDetector::trimTo2DecimalPlaces(float &x, float &y) {
    int temp = round(x*(10/MAP_RESOLUTION));
    x        = floor(temp/(10.0/MAP_RESOLUTION)*100.0)/100.0;

    temp     = round(y*(10/MAP_RESOLUTION));
    y        = floor(temp/(10.0/MAP_RESOLUTION)*100.0)/100.0;

    return;
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "FinishBoxDetector");
    ros::NodeHandle n;
    FinishBoxDetector mg(n);
    ros::spin();
    return 0;
}
