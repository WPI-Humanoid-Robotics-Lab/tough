/**
 ********************************************************************************************************
 * @file    pointcloudhelper.cpp
 * @brief   pointclod helper class
 * @details Used to assist with the point cloud data
 ********************************************************************************************************
 */

#ifndef POINTCLOUDHELPER_H_
#define POINTCLOUDHELPER_H_

#include <tough_perception_common/global.h>

namespace tough_perception
{
class PointCloudHelper
{
public:
  /**
   * @brief this function generates an organized point cloud
   * @param dispImage	the disparity image
   * @param colorImage the color image that has the rgb information
   * @param Qmat	the Q matrix that allows you to convert uv to rgb
   * @param cloud  the organized point cloud as a color image
   */
  static void generateOrganizedRGBDCloud(const cv::Mat& dispImage, const cv::Mat& colorImage, const cv::Mat Qmat,
                                         tough_perception::StereoPointCloudColor::Ptr& cloud);

  static void getLaserFOV(const float hfov, float vfov, const cv::Size img_sz, const float laser_speed,
                          const pcl::PointCloud<LaserPoint>::Ptr inp, pcl::PointCloud<LaserPoint>::Ptr& subCloud);

  static void filterLaserScan(const pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloud, std::vector<int>& indices);
};
}

#endif
