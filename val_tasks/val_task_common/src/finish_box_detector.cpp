#include "val_task_common/finish_box_detector.h"
#include <val_common/val_common_names.h>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

FinishBoxDetector::FinishBoxDetector(ros::NodeHandle &n):nh_(n) {

    pointcloudSub_ = nh_.subscribe("/map", 10, &FinishBoxDetector::detectFinishBox, this);
    //    mapPub_        = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 10, true);
}

FinishBoxDetector::~FinishBoxDetector()
{

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

    //    vector<vector<Point> > contours;
    //    vector<Vec4i> hierarchy;
    //    RNG rng(12345);
    //    Mat temp;
    //    temp=map_image_;
    //    /// Find contours
    //    findContours( temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    //    /// Draw contours
    //    Mat drawing = Mat::zeros( temp.size(), CV_8UC3 );
    //    for( int i = 0; i< contours.size(); i++ )
    //    {
    //        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    //        drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
    //    }


    Mat dst, dst_norm, dst_norm_scaled;
    dst = Mat::zeros( map_image_.size(), CV_32FC1 );

    /// Detector parameters
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;
    int thresh = 150;

    /// Detecting corners
    cornerHarris( map_image_, dst, blockSize, apertureSize, k, BORDER_DEFAULT );

    /// Normalizing
    normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    convertScaleAbs( dst_norm, dst_norm_scaled );

    /// Drawing a circle around corners
    for( int j = 0; j < dst_norm.rows ; j++ )
    { for( int i = 0; i < dst_norm.cols; i++ )
        {
            if( (int) dst_norm.at<float>(j,i) > thresh )
            {
                circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(255), 2, 8, 0 );
            }
        }
    }
    //     Showing the result

    namedWindow( "actual", CV_WINDOW_AUTOSIZE );
    imshow( "actual", dst_norm_scaled);
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
