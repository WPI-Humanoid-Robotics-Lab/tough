/***
	Nathaniel Goldfarb
	This node detects the frame the position of a button and create a button frame
***/
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <perception_common/MultisenseImage.h>
#include <perception_common/MultisensePointCloud.h>
#include <perception_common/PointCloudHelper.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
#include <assert.h> 
#include <string>

using namespace cv;
using namespace std;
using namespace src_perception;

bool processImage(const cv::Mat&, geometry_msgs::Point&);

ros::Publisher buttonDectected;


void getButtonFrame(ros::NodeHandle &nh)
{

	string frame = "buttonFrame";
	src_perception::MultisenseImage mi(nh);
	pcl::PointXYZRGB point; 
	geometry_msgs::Point buttonCenter;
	std_msgs::Bool found; 
	StereoPointCloudColor::Ptr organized_cloud(new StereoPointCloudColor);
	static tf::TransformBroadcaster br;
	tf::Quaternion q;
    tf::Transform transform;
	cv::Mat img;
	cv::Mat_<float> disp;
	cv::Mat_<double> Q;


	while(ros::ok())
	{
		found.data = false;
		//get synced images from multisense. check the datatypes of arguments in function definition.
		if ( mi.giveSyncImages(img, disp) && mi.giveQMatrix(Q))
		{
			PointCloudHelper::generateOrganizedRGBDCloud(disp,img,Q,organized_cloud);
			ROS_INFO_STREAM("Organized cloud size: "<<organized_cloud->size());
			if( processImage(img,buttonCenter) )
			{
				ROS_INFO_STREAM("found button");
				found.data = true;
				point = organized_cloud->at(buttonCenter.x,buttonCenter.y);
				transform.setOrigin( tf::Vector3(point.x , point.y, point.z) );
        		q.setRPY(0, 0, 0);
        		transform.setRotation(q);
        		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "left_camera_optical_frame", frame));
				buttonDectected.publish(found);
			}	
		}
		else
		{
			ROS_INFO_STREAM("not found button");
			buttonDectected.publish(found);
		}
		
		ros::spinOnce();
	}	



}

bool processImage(const cv::Mat& src, geometry_msgs::Point &buttonCenter )
{

    // Mat dst;
    // flip(src,dst,-1);
    
    Mat3b hsv;
    bool foundButton = false;
    cvtColor(src, hsv, COLOR_BGR2HSV);
        // detect red color in hsv image
    Mat1b mask1, mask2;
    inRange(hsv, Scalar(0, 178, 51), Scalar(5, 255, 128), mask1);
    inRange(hsv, Scalar(170, 204, 140), Scalar(180, 255, 191), mask2);

    Mat1b mask = mask1 | mask2;
    imshow("Mask", mask);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
        // find contours
    findContours( mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    RNG rng(12345);
    int largest_area=0;
    int largest_contour_index=0;
    Rect bounding_rect;
        /// Draw contours
    Mat drawing = Mat::zeros( mask.size(), CV_8UC3 );
    vector<Point2f> points;

    for( int i = 0, j = 0; i< contours.size(); i++ )
    {

        //  Find the area of contour
        double a=contourArea( contours[i],false);
        if(a>largest_area){
            largest_area=a;
                //cout<<i<<" area  "<<a<<endl;
                // Store the index of largest contour
            largest_contour_index=i;
                // Find the bounding rectangle for biggest contour
            bounding_rect=boundingRect(contours[i]);
        }
        Moments moment = moments((Mat)contours[i]);

        if (moment.m00)
        {
            points.push_back(Point2f(moment.m10/moment.m00,moment.m01/moment.m00));
            std::cout<<"m00"<<moment.m00<<"m10"<<moment.m10<<"m01"<<moment.m01;
            std::cout<<"x:"<<points[j].x<<" y:"<<points[j].y<<"\n";
            //assian the button center
            buttonCenter.x = int (points[j].x);
            buttonCenter.y = int (points[j].y);
            foundButton = true;
      		return foundButton;
            j++;
        }
    }

    return foundButton;
}

int main(int argc, char** argv)
 {
 	ros::init(argc,argv,"ButtonFrame");
 	ros::NodeHandle nh;
	
	buttonDectected = nh.advertise<std_msgs::Bool>("ButtonFound", 1);
	getButtonFrame(nh);


}
