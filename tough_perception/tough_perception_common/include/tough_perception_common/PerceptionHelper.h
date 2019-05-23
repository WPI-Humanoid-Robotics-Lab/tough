#ifndef PERCEPTIONHELPER_H_
#define PERCEPTIONHELPER_H_

#include <Eigen/Dense>
#include <tough_perception_common/global.h>
#include <tough_perception_common/perception_common_names.h>

#include <pcl/point_representation.h>

namespace tough_perception
{

// convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZI PointTI;
typedef sensor_msgs::PointCloud2 PointCloudSensorMsg;

typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointTI> PointCloud_I;

typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

/**
 * @brief custom class required for pointcloud registeration
 * Define a new point representation for < x, y, z, curvature >
 */

class customPointRepresentation : public pcl::PointRepresentation<PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;

public:
    customPointRepresentation()
    {
        // Define the number of dimensions
        nr_dimensions_ = 4;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray(const PointNormalT &p, float *out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};

/**
 * @brief This function generates an Organized RGBD cloud using color and disparity images. 
 * 
 * @param dispImage disparity image
 * @param colorImage RGB image
 * @param Qmat  Q matrix for transforming pixel coordinates
 * @param cloud  output pointer to PointcloudXYZRGB 
 */
void generateOrganizedRGBDCloud(const cv::Mat &dispImage, const cv::Mat &colorImage, const Eigen::Matrix4d Qmat,
                                tough_perception::StereoPointCloudColor::Ptr &cloud);

/**
 * @brief Converts a ROS Point cloud message to PCL Pointcloud 
 * 
 * @tparam T 
 * @param ros_msg 
 * @param pcl_msg 
 */
template <typename T>
void convertROStoPCL(const PointCloudSensorMsg::Ptr ros_msg, T &pcl_msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*ros_msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *pcl_msg);
}

/**
 * @brief Converts a PCL Pointcloud to ROS Point cloud message 
 * 
 * @tparam T 
 * @param pcl_msg 
 * @param ros_msg 
 */
template <typename T>
void convertPCLtoROS(const T pcl_msg, PointCloudSensorMsg::Ptr &ros_msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*pcl_msg, pcl_pc2);
    pcl_conversions::moveFromPCL(pcl_pc2, *ros_msg);
}

float min_internsity(PointCloud_I::Ptr pc);
} // namespace tough_perception

#endif