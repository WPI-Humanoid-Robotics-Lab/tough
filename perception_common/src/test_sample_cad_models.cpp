#include "ros/ros.h"
#include "perception_common/SampleCADModels.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/cloud_viewer.h"

void foo ()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

  cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::io::loadPCDFile("/home/nbanerjee/Documents/test_pcd1.pcd", *cloud);

  ROS_INFO("Read the file successfully.");

  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  viewer.showCloud (cloud);
  while (!viewer.wasStopped ())
  {
  }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_sample_cad_models");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Publisher sampled_cloud_pub;

    perception_common::SampleCADModels  sample_model(nh, pnh);

    std::string                         filename;
    sensor_msgs::PointCloud2            cloud;
    pcl::PointCloud<pcl::PointXYZ>      normals;
    perception_common::SamplingParams   params;

    // Only works with DAE files and not STL files
    // TO-DO - Support for STL files

    //filename = std::string("package://perception_common/debris_model/jaco_link_hand.dae");
    //filename = std::string("package://perception_common/debris_model/atlas_description/robotiq_link_1.dae");
    filename = std::string("package://perception_common/debris_model/table2.dae");

    params.number_of_points = 20000;
    params.sample_type = perception_common::SampleType::RANDOM;
    params.step_size = 0.005;

    sample_model.meshToPointCloud(filename, cloud, normals, params);

    sampled_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/perception_drc/sampled_model_cloud", 10, true);

    cloud.header.frame_id = "root";
    cloud.header.stamp = ros::Time::now();

    pcl::PointCloud<pcl::PointXYZ>  pcl_cloud;

    pcl::fromROSMsg(cloud, pcl_cloud);    

    //for (int i = 0; i < 100; i++)
    //    ROS_INFO_STREAM(i << " - " << std::endl << "  " << pcl_cloud.points[i].x << " " << pcl_cloud.points[i].y << " " << pcl_cloud.points[i].z << " " << std::endl);

    ROS_INFO_STREAM("Points - " << pcl_cloud.points.size());

    pcl::io::savePCDFileASCII("/home/nbanerjee/Documents/test_pcd1.pcd", pcl_cloud);

    foo();

    while (ros::ok())
    {
        cloud.header.frame_id = "root";
        cloud.header.stamp = ros::Time::now();

        sampled_cloud_pub.publish(cloud);
        ros::spinOnce();
    }

    return 0;
}
