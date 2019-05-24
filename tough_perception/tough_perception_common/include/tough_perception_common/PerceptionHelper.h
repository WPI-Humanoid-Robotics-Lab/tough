#ifndef PERCEPTIONHELPER_H_
#define PERCEPTIONHELPER_H_

#include <Eigen/Dense>
#include <tough_perception_common/global.h>
#include <tough_perception_common/perception_common_names.h>

#include <pcl/point_representation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

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
 * @brief A functor for PCL Passthrough filter
 * 
 * @tparam T can be PCL Point type
 */
template <typename T>
class PassThroughFilter
{
private:
    std::shared_ptr<pcl::PassThrough<T>> filter_;
    std::string axis_;
    float min_val_, max_val_;

public:
    PassThroughFilter()
    {
        filter_ = std::make_shared<pcl::PassThrough<T>>();
    }
    PassThroughFilter(std::string axis, float min_val, float max_val) : axis_(axis),
                                                                        min_val_(min_val),
                                                                        max_val_(max_val)
    {
        filter_ = std::make_shared<pcl::PassThrough<T>>();
    }
    void operator()(PointCloud_I::Ptr input_cloud,
                    std::string axis,
                    float min_val,
                    float max_val)
    {
        axis_ = axis;
        min_val_ = min_val;
        max_val_ = max_val;
        filter(input_cloud);
    }
    void operator()(PointCloud_I::Ptr input_cloud)
    {
        filter(input_cloud);
    }

private:
    void filter(PointCloud_I::Ptr input_cloud)
    {
        filter_->setInputCloud(input_cloud);
        filter_->setFilterFieldName(axis_);
        filter_->setFilterLimits(min_val_, max_val_);
        filter_->filter(*input_cloud);
    }
};

/**
 * @brief A functor for PCL Voxel Grid Filter
 * 
 * @tparam T can be PCL Point type
 */
template <typename T>
class VoxelGridFilter
{
private:
    std::shared_ptr<pcl::VoxelGrid<T>> filter_;
    float leaf_size_;

public:
    VoxelGridFilter() : leaf_size_(0.05)
    {
        filter_ = std::make_shared<pcl::VoxelGrid<T>>();
        filter_->setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    }
    VoxelGridFilter(float leaf_size) : leaf_size_(leaf_size)
    {
        filter_ = std::make_shared<pcl::PassThrough<T>>();
        filter_->setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    }
    void operator()(PointCloud_I::Ptr input_cloud, float leaf_size)
    {
        filter_->setLeafSize(leaf_size, leaf_size, leaf_size);
        filter(input_cloud);
    }
    void operator()(PointCloud_I::Ptr input_cloud, float leaf_size_x, float leaf_size_y, float leaf_size_z)
    {
        filter_->setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
        filter(input_cloud);
    }
    void operator()(PointCloud_I::Ptr input_cloud)
    {
        // ROS_INFO("grid size %f", leaf_size_);
        filter_->setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        filter(input_cloud);
    }

private:
    void filter(PointCloud_I::Ptr input_cloud)
    {
        filter_->setInputCloud(input_cloud);
        filter_->filter(*input_cloud);
    }
};

/**
 * @brief A functor to align 2 point clouds using ICP
 * 
 * @tparam T 
 */
template <typename T>
class PointCloudAligner
{
    PointCloudAligner()
    {
    }
    void operator()(PointCloud_I::Ptr input_cloud,
                    PointCloud_I::Ptr out_cloud)
    {
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

/**
 * @brief Finds the minimum intensity of point in the given point cloud
 * 
 * @param pc Point cloud with internsity
 * @return float 
 */
float min_internsity(PointCloud_I::Ptr pc);
} // namespace tough_perception

#endif