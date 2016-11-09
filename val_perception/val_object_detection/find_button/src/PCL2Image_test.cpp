// ROS core
#include <ros/ros.h>
//Image message
#include <sensor_msgs/Image.h>
//pcl::toROSMsg
#include <pcl/io/pcd_io.h>
//stl stuff
#include <string>

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
//x=340 ,y =320

class PointCloudToImage
{
public:
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();
    marker.id =  visualization_msgs::Marker::CUBE;

    if ((cloud->width * cloud->height) == 0)
      return; //return if the cloud is not dense!
    try
    {
      pcl::toROSMsg (*cloud, image_); //convert the cloud
      image_.header.frame_id = cloud->header.frame_id;
      marker.header.frame_id = image_.header.frame_id;
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to image message: " << e.what());
    }
    image_pub_.publish (image_); //publish our cloud image
  }

  PointCloudToImage () : cloud_topic_("input"),image_topic_("output")
  {
    sub_ = nh_.subscribe ("/multisense/image_points2", 30, &PointCloudToImage::cloud_cb, this);
    image_pub_ = nh_.advertise<sensor_msgs::Image> (image_topic_, 30);

    //print some info about the node
    std::string r_ct = nh_.resolveName (cloud_topic_);
    std::string r_it = nh_.resolveName (image_topic_);
    ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );
    ROS_INFO_STREAM("Publishing image on topic " << r_it );
  }

private:
  ros::NodeHandle nh_;
  sensor_msgs::Image image_; //cache the image message
  std::string cloud_topic_; //default input
  std::string image_topic_; //default output
  ros::Subscriber sub_; //cloud subscriber
  ros::Publisher image_pub_; //image message publisher
};

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "PCL2Image_test");
  PointCloudToImage pci; //this loads up the node
  ros::spin (); //where she stops nobody knows
  return 0;
}